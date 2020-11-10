/*
 * file : gzd-en.c
 * desc : linux ethernet device driver for the Gen-Z demo PCIe device
 *
 * Author:      Roy Franz <roy.franz@hpe.com>
 *              Jim Hull <jim.hull@hpe.com>
 *
 * Copyright:
 *     © Copyright 2016-2017 Hewlett Packard Enterprise Development LP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/ratelimit.h>

#include <linux/etherdevice.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/skbuff.h>
#include <linux/tcp.h>
#include <linux/timex.h>

#include "gzd.h"

#define GZD_EN_DRV_NAME "gzd-en"
#define GZD_NUM_NETDEVS     1	/* Currently only 1 per card support */


/*
 * Copy packets on TX to aligned buffers, as early hardware seems to hang
 * for some alignments.
 */
#define EN_TX_COPY	1

LIST_HEAD(gzd_en_card_list);
DEFINE_MUTEX(gzd_en_lock);	/* used only during initialization */

static struct workqueue_struct *gzd_en_wq = 0;

static uint loopback = 0;
module_param(loopback, int, S_IRUGO);
MODULE_PARM_DESC(loopback, "loopback all traffic to same card");

static uint tx_wake_thresh = 10;	/* Revisit: tune this */
module_param(tx_wake_thresh, int, S_IRUGO);
MODULE_PARM_DESC(tx_wake_thresh,
		 "minimum free tx FIFO slots before waking");

/*
 * Revisit: temporary debug 
 */
static uint tx_intr_udelay = 0;
module_param(tx_intr_udelay, int, S_IRUGO);
MODULE_PARM_DESC(tx_intr_udelay,
		 "number of usecs to delay in tx_comp_irq_handler");

void gzd_net_init(struct net_device *dev);
static void gzd_en_tx_work_handler(struct work_struct *w);
static void gzd_en_fini(void);

/*
 * Request ID range.  We should have more request IDs available than tx
 * message queue depth.  The number of outstanding TX packets is limited
 * by NUM_REQ_ID.
 */
#define NUM_REQ_ID	200	/* way more than HW queue depth */
#define MIN_REQ_ID	3000
#define MAX_REQ_ID	(MIN_REQ_ID + NUM_REQ_ID - 1)

struct gzd_en_tx_pend {
#ifdef EN_TX_COPY
    void *cpu_addr;
#else
    void *skb;
#endif
    dma_addr_t dma_addr;
    size_t len;
    cycles_t start_time, resp_time;
};

struct gzd_en_card_data {
    struct list_head list;
    spinlock_t lock;
    struct gzd_core_info *core_info;
    void *hw;
    struct net_device *netdevs[GZD_NUM_NETDEVS];
    uint req_id;
    uint queue_stopped;
    struct gzd_en_tx_pend tx_pending[NUM_REQ_ID];
    cycles_t tx_overhead;
    cycles_t tx_req_resp;
};

/*
 * For now we only support a single netdev per card.  In order to support
 * more netdevs/card, some updates will need to be made regarding
 * determining the index.  Some places, like the IRQ handlers, are really
 * per-card, and we likely won't know there what netdev a packet is
 * associated with, as any actual packet handling is left to the work
 * handler.
 */
#define	GZD_NETDEV_INDEX	0
struct gzd_en_netdev_priv {
    struct net_device *dev;
    spinlock_t lock;
    struct net_device_stats stats;
    struct gzd_en_card_data *card_data;
};

struct gzd_en_msg_work {
    struct gzd_en_card_data *card;
    int tx_index;		/* For tx completion */
    struct work_struct work;
};

static int gzd_en_probe(void *arg);
static void gzd_en_remove(void *arg);
static struct gzd_driver gzd_en_driver = {
    .name = GZD_EN_DRV_NAME,
    .probe = gzd_en_probe,
    .remove = gzd_en_remove,
};

static void gzd_en_free_req_id(struct gzd_en_card_data *card,
			       int req_index);

static irqreturn_t gzd_en_tx_comp_irq_handler(int irq, void *data)
{
    cycles_t start = get_cycles();
    int cpu_id = smp_processor_id();
    struct gzd_en_card_data *card = data;
    void *hw = card->hw;
    int ret, more, handled = 0, wakeup = 0;
    uint req_id, req_index, free_reqs;
    struct gzd_en_msg_work *msg_work;
    struct gzd_en_netdev_priv *priv;
    struct device *dev = &card->core_info->pdev->dev;
    ulong flags;
    cycles_t resp_cycles;

    priv = netdev_priv(card->netdevs[GZD_NETDEV_INDEX]);
    if (tx_intr_udelay)
	udelay(tx_intr_udelay);

    do {
	more = 0;
	ret = gzd_en_driver.msg_response(hw, &req_id, &free_reqs);
	resp_cycles = get_cycles();
	if (ret < 0) {
	    dev_err(dev,
		    "%s: msg_response returned error: %d for req_id %u\n",
		    __func__, ret, req_id);
	    priv->stats.tx_errors++;
	}
	if (free_reqs >= tx_wake_thresh)
	    wakeup = 1;
	if (ret == 1 || ret == -EIO) {	/* req_id is valid */
	    more = handled = 1;
	    req_index = req_id - MIN_REQ_ID;
	    card->tx_pending[req_index].resp_time = resp_cycles;

	    dev_dbg(dev,
		    "%s: req_id=%u, intr_time=%llu, resp_time=%llu, cpu=%d, free_reqs=%u\n",
		    __func__, req_id, start, resp_cycles, cpu_id,
		    free_reqs);
	    msg_work = kmalloc(sizeof(*msg_work), GFP_ATOMIC);
	    if (!msg_work) {
		netdev_err(card->netdevs[GZD_NETDEV_INDEX],
			   "msg_work kmalloc failed\n");
		return IRQ_NONE;
	    }
	    msg_work->card = card;
	    msg_work->tx_index = req_index;
	    INIT_WORK(&msg_work->work, gzd_en_tx_work_handler);
	    queue_work(gzd_en_wq, &msg_work->work);
	}
    }
    while (more);

    if (wakeup) {
	spin_lock_irqsave(&card->lock, flags);
	if (card->queue_stopped) {
	    netdev_dbg(card->netdevs[GZD_NETDEV_INDEX],
		       "wake queue, free_reqs=%u\n", free_reqs);
	    netif_wake_queue(card->netdevs[GZD_NETDEV_INDEX]);
	    card->queue_stopped = 0;
	}
	spin_unlock_irqrestore(&card->lock, flags);
    }

    if (!handled)
	dev_dbg(dev,
		"%s: spurious, intr_time=%llu, resp_time=%llu, cpu=%d, free_reqs=%u\n",
		__func__, start, resp_cycles, cpu_id, free_reqs);

    return (handled) ? IRQ_HANDLED : IRQ_NONE;
}


struct gzd_msg_header *gzd_en_msg_next(struct gzd_en_card_data *card,
				       struct gzd_msg_header *msg_hdr)
{
    struct gzd_msg_header *next_hdr;
    void *prod_ptr, *buf_end_va;
    size_t size;
    void *hw = card->hw;
    struct gzd_msg_tbl_entry *msg_tbl = card->core_info->msg_tbl;

    gzd_en_driver.msg_read_prod_ptr(hw, msg_tbl, 0);
    prod_ptr = msg_tbl->prod_ptr_va;
    size = roundup(msg_hdr->len, sizeof(*msg_hdr));
    next_hdr = msg_hdr + (size / sizeof(*msg_hdr));
    buf_end_va = msg_tbl->buf_start_va + msg_tbl->buf_size;
    if ((void *) next_hdr >= buf_end_va)	/* wrap */
	next_hdr -= (msg_tbl->buf_size / sizeof(*msg_hdr));

    return (next_hdr != prod_ptr) ? next_hdr : 0;
}

static void gzd_en_tx_free(struct gzd_en_card_data *card, int req_index)
{
    struct device *dev = &card->core_info->pdev->dev;
    ulong flags;

#ifdef EN_TX_COPY
    dma_free_coherent(dev, card->tx_pending[req_index].len,
		      card->tx_pending[req_index].cpu_addr,
		      card->tx_pending[req_index].dma_addr);
#else
    dev_kfree_skb(card->tx_pending[req_index].skb);
    dma_unmap_single(dev,
		     card->tx_pending[req_index].dma_addr,
		     card->tx_pending[req_index].len, DMA_TO_DEVICE);
#endif
    spin_lock_irqsave(&card->lock, flags);
    card->tx_req_resp += (card->tx_pending[req_index].resp_time -
			  card->tx_pending[req_index].start_time);
    spin_unlock_irqrestore(&card->lock, flags);
    gzd_en_free_req_id(card, req_index);
}

static void gzd_en_tx_work_handler(struct work_struct *w)
{
    struct gzd_en_msg_work *msg_work;
    struct gzd_en_card_data *card;

    msg_work = container_of(w, struct gzd_en_msg_work, work);
    card = msg_work->card;
    gzd_en_tx_free(card, msg_work->tx_index);
}

#ifdef EN_RX_WORK
static void gzd_en_rx_work_handler(struct work_struct *w)
{
    struct gzd_en_msg_work *msg_work;
    struct gzd_msg_header *msg_hdr, *next_hdr;
    struct sk_buff *skb;
    struct gzd_en_card_data *card;
    struct gzd_en_netdev_priv *priv;
    struct device *dev;
    size_t total_size, behind;
    int gzd_msg_hdr_size = 16;
    void *packet_data, *buf_end_va;
    size_t packet_len;
    size_t packet_len_before_wrap;
    size_t packet_len_after_wrap;
    struct gzd_msg_tbl_entry *msg_tbl;

    msg_work = container_of(w, struct gzd_en_msg_work, work);
    card = msg_work->card;
    msg_tbl = card->core_info->msg_tbl;
    priv = netdev_priv(card->netdevs[GZD_NETDEV_INDEX]);
    dev = &card->core_info->pdev->dev;
    buf_end_va = msg_tbl->buf_start_va + msg_tbl->buf_size;

    do {
	msg_hdr = msg_tbl->cons_ptr_va;
	total_size = 0;
	do {
	    packet_len = msg_hdr->len - gzd_msg_hdr_size;
	    packet_data = (char *) msg_hdr + gzd_msg_hdr_size;
	    if ((packet_data + packet_len) > buf_end_va)
		packet_len_before_wrap = buf_end_va - packet_data;
	    else
		packet_len_before_wrap = packet_len;
	    packet_len_after_wrap = packet_len - packet_len_before_wrap;
	    /*
	     * alloc & memcpy msg 
	     */
	    skb =
		__netdev_alloc_skb_ip_align(card->netdevs
					    [GZD_NETDEV_INDEX], packet_len,
					    GFP_KERNEL);
	    if (!skb) {
		netdev_err(card->netdevs[GZD_NETDEV_INDEX],
			   "gzd rx: low on mem - packet dropped\n");
		priv->stats.rx_dropped++;
		goto msg_next;
	    }
	    skb_put(skb, packet_len);
	    if (packet_len_before_wrap)
		skb_copy_to_linear_data(skb, packet_data,
					packet_len_before_wrap);
	    if (packet_len_after_wrap)
		skb_copy_to_linear_data_offset(skb,
					       packet_len_before_wrap,
					       msg_tbl->buf_start_va,
					       packet_len_after_wrap);
	    skb->dev = card->netdevs[0];
	    skb->protocol = eth_type_trans(skb, skb->dev);
	    skb->ip_summed = CHECKSUM_UNNECESSARY;
	    priv->stats.rx_packets++;
	    priv->stats.rx_bytes += msg_hdr->len;	/* packet_len? */
	    netif_rx(skb);
	  msg_next:
	    total_size += roundup(msg_hdr->len, 16);
	    next_hdr = gzd_en_msg_next(card, msg_hdr);
	    netdev_dbg(card->netdevs[GZD_NETDEV_INDEX],
		       "rx msg=%p, next=%p, pckt_len=%zu (before=%zu/after=%zu)",
		       msg_hdr, next_hdr, packet_len,
		       packet_len_before_wrap, packet_len_after_wrap);
	    msg_hdr = next_hdr;
	}
	while (msg_hdr);

	/*
	 * move cons_ptr by total size and test if we're still behind 
	 */
	behind = gzd_en_driver.msg_update_cons_ptr(card->hw,
						   &card->msg_tbl, 0,
						   total_size);
    }
    while (behind);

    kfree(msg_work);
}

static irqreturn_t gzd_en_rx_irq_handler(int irq, void *data)
{
    struct gzd_en_card_data *card = data;
    struct gzd_msg_tbl_entry *msg_tbl = card->core_info->msg_tbl;
    struct gzd_en_msg_work *msg_work;

    gzd_en_driver.msg_read_prod_ptr(card->hw, &card->msg_tbl, 0);
    if (msg_tbl->cons_ptr == msg_tbl->prod_ptr) {
	return IRQ_HANDLED;	/* spurious */
    }
    msg_work = kmalloc(sizeof(*msg_work), GFP_ATOMIC);
    if (!msg_work) {
	netdev_err(card->netdevs[GZD_NETDEV_INDEX],
		   "msg_work kmalloc failed\n");
	return IRQ_NONE;
    }
    msg_work->card = card;
    INIT_WORK(&msg_work->work, gzd_en_rx_work_handler);
    queue_work(gzd_en_wq, &msg_work->work);

    return IRQ_HANDLED;
}
#else				// EN_RX_WORK
static irqreturn_t gzd_en_rx_irq_handler(int irq, void *data)
{
    struct gzd_en_card_data *card = data;
    struct gzd_msg_tbl_entry *msg_tbl = card->core_info->msg_tbl;
    struct gzd_msg_header *msg_hdr, *next_hdr;
    struct sk_buff *skb;
    struct gzd_en_netdev_priv *priv;
    struct device *dev;
    size_t total_size, behind;
    int gzd_msg_hdr_size = 16;
    void *packet_data, *buf_end_va;
    size_t packet_len;
    size_t packet_len_before_wrap;
    size_t packet_len_after_wrap;

    gzd_en_driver.msg_read_prod_ptr(card->hw, msg_tbl, 0);
    if (msg_tbl->cons_ptr == msg_tbl->prod_ptr) {
	return IRQ_HANDLED;	/* spurious */
    }

    priv = netdev_priv(card->netdevs[GZD_NETDEV_INDEX]);
    dev = &card->core_info->pdev->dev;
    buf_end_va = msg_tbl->buf_start_va + msg_tbl->buf_size;

    do {
	msg_hdr = msg_tbl->cons_ptr_va;
	total_size = 0;
	do {
	    packet_len = msg_hdr->len - gzd_msg_hdr_size;
	    packet_data = (char *) msg_hdr + gzd_msg_hdr_size;
	    if ((packet_data + packet_len) > buf_end_va)
		packet_len_before_wrap = buf_end_va - packet_data;
	    else
		packet_len_before_wrap = packet_len;
	    packet_len_after_wrap = packet_len - packet_len_before_wrap;
	    /*
	     * alloc & memcpy msg 
	     */
	    skb =
		netdev_alloc_skb_ip_align(card->netdevs[GZD_NETDEV_INDEX],
					  packet_len);
	    if (!skb) {
		netdev_err(card->netdevs[GZD_NETDEV_INDEX],
			   "gzd rx: low on mem - packet dropped\n");
		priv->stats.rx_dropped++;
		goto msg_next;
	    }
	    skb_put(skb, packet_len);
	    if (packet_len_before_wrap)
		skb_copy_to_linear_data(skb, packet_data,
					packet_len_before_wrap);
	    if (packet_len_after_wrap)
		skb_copy_to_linear_data_offset(skb,
					       packet_len_before_wrap,
					       msg_tbl->buf_start_va,
					       packet_len_after_wrap);
	    skb->dev = card->netdevs[0];
	    skb->protocol = eth_type_trans(skb, skb->dev);
	    skb->ip_summed = CHECKSUM_UNNECESSARY;
	    priv->stats.rx_packets++;
	    priv->stats.rx_bytes += msg_hdr->len;	/* packet_len? */
	    netif_rx(skb);
	  msg_next:
	    total_size += roundup(msg_hdr->len, 16);
	    next_hdr = gzd_en_msg_next(card, msg_hdr);
	    netdev_dbg(card->netdevs[GZD_NETDEV_INDEX],
		       "rx msg=%p, next=%p, pckt_len=%zu (before=%zu/after=%zu)",
		       msg_hdr, next_hdr, packet_len,
		       packet_len_before_wrap, packet_len_after_wrap);
	    msg_hdr = next_hdr;
	}
	while (msg_hdr);

	/*
	 * move cons_ptr by total size and test if we're still behind 
	 */
	behind = gzd_en_driver.msg_update_cons_ptr(card->hw,
						   msg_tbl, 0, total_size);
    }
    while (behind);

    return IRQ_HANDLED;
}
#endif				// EN_RX_WORK

/*
 * gzd_en_ether_cid extracts a 28-bit Global CID from the Ethernet address
 * returns -1u if not a valid Gen-Z Ethernet address 
 */

static inline uint gzd_en_ether_cid(const u8 * addr)
{
    uint cid;

    if (addr[0] != 0x5a) {
	cid = -1u;
    } else {
	cid = ((addr[1] & 0x0f) << 24) | (addr[2] << 16) |
	    (addr[3] << 8) | addr[4];
    }

    return cid;
}

/*
 * gzd_en_ether_addr generates a software asssigned Ethernet address
 * containing the component's 28-bit Global CID and netdev index
 */
static inline void gzd_en_ether_addr(u8 * addr, uint index, uint cid)
{
    addr[0] = 0x5a;		/* 'Z' (includes local assignment bit) */
    addr[1] = (cid >> 24) & 0x0f;
    addr[2] = (cid >> 16) & 0xff;
    addr[3] = (cid >> 8) & 0xff;
    addr[4] = cid & 0xff;
    addr[5] = index & 0xff;
}

/*
 * gzd_en_eth_hw_addr builds an Ethernet (MAC) address containing the CID 
 */
static inline void
gzd_en_eth_hw_addr(struct net_device *dev, uint index, uint cid)
{
    dev->addr_assign_type |= NET_ADDR_RANDOM;
    gzd_en_ether_addr(dev->dev_addr, index, cid);
}

/*
 * gzd_en_probe is called by gzd-core for each physical card in the system
 *
 */
static int gzd_en_probe(void *arg)
{
    struct gzd_en_card_data *card_data;
    struct gzd_core_info *info;
    int i, j, up;
    int ret;
    struct device *dev;

    card_data = kzalloc(sizeof(*card_data), GFP_KERNEL);
    if (!card_data)
	return -ENOMEM;

    spin_lock_init(&card_data->lock);
    card_data->hw = arg;
    card_data->req_id = MIN_REQ_ID;
    info = gzd_en_driver.info(arg);
    dev = &info->pdev->dev;
    card_data->core_info = info;
    dev_info(dev, "gzd_en: hw ptr: %p, card_data: %p\n", card_data->hw,
	     card_data);
    dev_info(dev,
	     "gzd_en: gzd_core_info: card_id=%u, num_cards=%u, num_media=%u "
	     "genz_subnets=%u, req_fifo_depth=%u, resp_fifo_depth=%u\n",
	     info->card_id, info->num_cards, info->num_media,
	     info->genz_subnets, info->req_fifo_depth,
	     info->resp_fifo_depth);
    for (i = 0; i < info->num_cards; i++)
	dev_info(dev,
		 "gzd_en: gzd_core_info:   card_ids[%d]=%u, scid=0x%x, dcid=0x%x\n",
		 i, info->card_ids[i], info->msg_info[i].scid,
		 info->msg_info[i].dcid);

    /*
     * Register interrupts 
     */
    ret =
	gzd_en_driver.request_irq((void *) card_data->hw, MsgSendInt,
				  gzd_en_tx_comp_irq_handler, 0,
				  "gzd-en-msg-send", (void *) card_data);
    if (ret) {
	dev_err(dev, "Error requesting MsgSendInt\n");
	goto out;
    }
    ret =
	gzd_en_driver.request_irq((void *) card_data->hw, MsgRecvInt,
				  gzd_en_rx_irq_handler, 0,
				  "gzd-en-msg-recv", (void *) card_data);
    if (ret) {
	dev_err(dev, "Error requesting MsgRecvInt\n");
	goto irq_release;
    }

    /*
     * Register a netdevice for each card - for now just 1 supported. 
     */
    ret = -ENODEV;
    for (i = 0; i < GZD_NUM_NETDEVS; i++) {
	struct gzd_en_netdev_priv *priv;
	card_data->netdevs[i] =
	    alloc_netdev(sizeof(struct gzd_en_netdev_priv), "gzd%d",
			 NET_NAME_UNKNOWN, gzd_net_init);

	if (!card_data->netdevs[i])
	    goto netdev_free;

	priv = netdev_priv(card_data->netdevs[i]);
	priv->card_data = card_data;
	card_data->netdevs[i]->flags &= ~IFF_MULTICAST;
    }
    for (i = 0; i < GZD_NUM_NETDEVS; i++) {
	if ((ret = register_netdev(card_data->netdevs[i]))) {
	    dev_err(dev, "gzd_en: error %i registering device \"%s\"\n",
		    ret, card_data->netdevs[i]->name);
	    goto netdev_free;
	} else {
	    /*
	     * Setting MAC in open() is too late 
	     */
	    gzd_en_eth_hw_addr(card_data->netdevs[i], i,
			       info->msg_info[info->card_index].dcid);
	    /*
	     * Set initial link state 
	     */
	    for (j = up = 0; j < MAX_AURORA; j++) {
		if (!AU_MASK(info->zlink_mask, j))
		    continue;
		if (info->link_status[j])
		    up = 1;
	    }
	    if (up)
		netif_carrier_on(card_data->netdevs[i]);
	    else
		netif_carrier_off(card_data->netdevs[i]);
	    ret = 0;
	}
    }

    mutex_lock(&gzd_en_lock);
    /*
     * Multiple cards are managed through gzd_en_card_list 
     */
    list_add_tail(&card_data->list, &gzd_en_card_list);
    mutex_unlock(&gzd_en_lock);
    dev_info(dev, "Successfully completed gzd_en_probe(), hw: %p\n",
	     card_data->hw);
    return 0;

  netdev_free:
    for (i = 0; i < GZD_NUM_NETDEVS; i++) {
	if (card_data->netdevs[i]) {
	    /*
	     * Not sure if it is OK to unregister a
	     * non-registered netdev.
	     * TODO - evaluate this if multiple netdevs/card are
	     * supported.
	     */
	    unregister_netdev(card_data->netdevs[i]);
	    free_netdev(card_data->netdevs[i]);
	}
    }

    gzd_en_driver.free_irq(card_data->hw, MsgRecvInt, (void *) card_data);
  irq_release:
    gzd_en_driver.free_irq(card_data->hw, MsgSendInt, (void *) card_data);
  out:
    dev_err(dev, "gzd_en_probe() failed, hw: %p, ret: %d\n", card_data->hw,
	    ret);
    if (ret)
	gzd_en_fini();
    return (ret);
}

static struct gzd_en_card_data *gzd_en_find_card_data(void *arg)
{
    struct gzd_en_card_data *ptr, *next;

    list_for_each_entry_safe(ptr, next, &gzd_en_card_list, list) {
	if (ptr->hw == arg)
	    return ptr;
    }

    pr_err("invalid gzd_en hw pointer: %p\n", arg);
    return 0;
}

static void gzd_en_remove(void *arg)
{
    int i;
    struct gzd_en_card_data *card_data;

    mutex_lock(&gzd_en_lock);
    card_data = gzd_en_find_card_data(arg);
    if (!card_data)
	goto unlock;


    for (i = 0; i < GZD_NUM_NETDEVS; i++) {
	if (card_data->netdevs[i]) {
	    unregister_netdev(card_data->netdevs[i]);
	    free_netdev(card_data->netdevs[i]);
	}
    }

    /*
     * free net device irq handlers 
     */
    if (gzd_en_driver.free_irq != NULL) {
	gzd_en_driver.free_irq((void *) card_data->hw,
			       MsgSendInt, (void *) card_data);
	gzd_en_driver.free_irq((void *) card_data->hw,
			       MsgRecvInt, (void *) card_data);
    }
    list_del_init(&card_data->list);
    kfree(card_data);

  unlock:
    mutex_unlock(&gzd_en_lock);
}



static int __init gzd_en_module_init(void)
{
    int ret;

    /*
     * Register with the GZD core driver.  The core will then call the
     * registered probe function, and be provided per-card information. 
     */
    ret = gzd_core_register_driver(&gzd_en_driver);

    gzd_en_wq = create_singlethread_workqueue("gzd_en_wq");
    if (!gzd_en_wq) {
	return -ENOMEM;
    }

    return ret;
}

static void gzd_en_fini(void)
{
    gzd_core_unregister_driver(&gzd_en_driver);

    if (gzd_en_wq)
	destroy_workqueue(gzd_en_wq);
}


int gzd_en_open(struct net_device *dev)
{
    netif_start_queue(dev);
    return 0;
}

int gzd_en_release(struct net_device *dev)
{
    netif_stop_queue(dev);
    return 0;
}

static int gzd_en_alloc_req_id(struct gzd_en_card_data *card)
{
    ulong flags;
    int req_id;
    int next_req_id;
    int req_index;

    spin_lock_irqsave(&card->lock, flags);
    req_id = card->req_id;
    next_req_id = (req_id < MAX_REQ_ID) ? req_id + 1 : MIN_REQ_ID;

    req_index = req_id - MIN_REQ_ID;
    /*
     * make sure our req_id/req_index is really free 
     */
    if (card->tx_pending[req_index].dma_addr) {
	netdev_err(card->netdevs[GZD_NETDEV_INDEX],
		   "%s() req_id %d not free, dma_addr=%pad\n",
		   __func__, req_id,
		   &card->tx_pending[req_index].dma_addr);
	req_id = -1;		/* error return value */
    } else {
	/*
	 * use a bogus dma_addr value to mark this slot "in use" 
	 */
	card->tx_pending[req_index].dma_addr = ~0;
    }

    card->req_id = next_req_id;
    spin_unlock_irqrestore(&card->lock, flags);
    return req_id;
}

static void
gzd_en_free_req_id(struct gzd_en_card_data *card, int req_index)
{
    ulong flags;

    spin_lock_irqsave(&card->lock, flags);
#ifdef EN_TX_COPY
    card->tx_pending[req_index].cpu_addr = 0;
#else
    card->tx_pending[req_index].skb = 0;
#endif
    card->tx_pending[req_index].dma_addr = 0;
    card->tx_pending[req_index].len = 0;
    spin_unlock_irqrestore(&card->lock, flags);
}

int gzd_en_tx(struct sk_buff *skb, struct net_device *netdev)
{
    cycles_t start = get_cycles();
    int cpu_id = smp_processor_id();
    struct gzd_en_netdev_priv *priv = netdev_priv(netdev);
    struct gzd_en_card_data *card = priv->card_data;
    struct device *dev = &card->core_info->pdev->dev;
    void *hw = card->hw;
    uint card_index = card->core_info->card_index;
    uint dcid;
    int req_id, req_index;
    int ret;
    dma_addr_t dma_addr;
    int len;
    ulong flags;
    const struct ethhdr *eth = eth_hdr(skb);
#ifdef EN_TX_COPY
    void *cpu_addr;
    cycles_t alloc_start, copy_start, copy_end;
#endif

    // Revisit: finish this
    dcid = gzd_en_ether_cid(eth->h_dest);
    if (loopback)
	dcid = card->core_info->msg_info[card_index].dcid;
    else
	dcid = card->core_info->msg_info[card_index + 1].dcid;
    req_id = gzd_en_alloc_req_id(card);
    if (req_id < 0) {
	netdev_err(card->netdevs[GZD_NETDEV_INDEX],
		   "%s() no free req_id\n", __func__);
	priv->stats.tx_errors++;
	return -ENOMEM;
    }
    req_index = req_id - MIN_REQ_ID;

    len = skb->len;

#ifdef EN_TX_COPY
    {
	alloc_start = get_cycles();
	/*
	 * HW defect requires DMA page-alignment, so alloc & copy 
	 */
	cpu_addr = dma_alloc_coherent(dev, len, &dma_addr, GFP_KERNEL);
	if (!cpu_addr || dma_addr & 0xfff) {
	    priv->stats.tx_errors++;
	    return -ENOMEM;
	}
	copy_start = get_cycles();
	memcpy(cpu_addr, skb->data, len);
	copy_end = get_cycles();
    }
#else
    dma_addr = dma_map_single(dev, skb->data, len, DMA_TO_DEVICE);
    if (dma_mapping_error(&card->core_info->pdev->dev, dma_addr)) {
	dev_kfree_skb_any(skb);
	netdev_err(card->netdevs[GZD_NETDEV_INDEX],
		   "gzd_en_tx dma mapping failed.\n");
	priv->stats.tx_errors++;
	return -ENOMEM;
    }
    dma_sync_single_for_device(dev, dma_addr, len, DMA_TO_DEVICE);
#endif
    /*
     * fill in all tx_pending info before calling msg_request as that
     * could immediately generate a tx interrupt that uses these 
     */
    card->tx_pending[req_index].dma_addr = dma_addr;
#ifdef EN_TX_COPY
    card->tx_pending[req_index].cpu_addr = cpu_addr;
#else
    card->tx_pending[req_index].skb = skb;
#endif
    card->tx_pending[req_index].len = len;
    /*
     * Todo - check packet length restrictions 
     */
    spin_lock_irqsave(&card->lock, flags);
    skb_tx_timestamp(skb);
    card->tx_pending[req_index].start_time = get_cycles();
    netdev_dbg(card->netdevs[GZD_NETDEV_INDEX],
	       "tx dma_addr=%pad, len=%d, req_id=%d, dcid=0x%x, tx_start=%llu, alloc_start=%llu, copy_start=%llu, copy_end=%llu, tx_req_start=%llu, cpu=%d",
	       &dma_addr, len, req_id, dcid, start,
	       alloc_start, copy_start, copy_end,
	       card->tx_pending[req_index].start_time, cpu_id);
    card->tx_overhead += (card->tx_pending[req_index].start_time - start);
    ret = gzd_en_driver.msg_request(hw, dma_addr, len, req_id, dcid);
    if (ret < 0) {
	if (ret == -EBUSY) {
	    /*
	     * stop the queue until there's room in the msg FIFO 
	     */
	    netdev_dbg(card->netdevs[GZD_NETDEV_INDEX],
		       "stopping due to msg_request EBUSY, req_id %d\n",
		       req_id);
	    if (!card->queue_stopped) {
		netif_stop_queue(netdev);
		card->queue_stopped = 1;
	    }
	    ret = NETDEV_TX_BUSY;
	} else {
	    netdev_err(card->netdevs[GZD_NETDEV_INDEX],
		       "msg_request returned error %d, req_id %d\n",
		       ret, req_id);
	    priv->stats.tx_errors++;
	}
	spin_unlock_irqrestore(&card->lock, flags);
	/*
	 * free up the req_id/req_index and DMA mapping 
	 */
	gzd_en_free_req_id(card, req_index);
#ifdef EN_TX_COPY
	dma_free_coherent(dev, len, cpu_addr, dma_addr);
#else
	dma_unmap_single(dev, dma_addr, len, DMA_TO_DEVICE);
#endif
	return ret;
    }

    priv->stats.tx_packets++;
    priv->stats.tx_bytes += len;
    spin_unlock_irqrestore(&card->lock, flags);

#ifdef EN_TX_COPY
    dev_kfree_skb(skb);
#endif
    return 0;
}

void gzd_en_tx_timeout(struct net_device *dev)
{
}

int gzd_en_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    netdev_err(dev, "ioctl %d called, not supported.\n", cmd);
    return -EOPNOTSUPP;
}

struct net_device_stats *gzd_en_stats(struct net_device *dev)
{
    struct gzd_en_netdev_priv *priv = netdev_priv(dev);
    return &priv->stats;
}

static const struct net_device_ops gzd_en_netdev_ops = {
    .ndo_open = gzd_en_open,
    .ndo_stop = gzd_en_release,
    .ndo_start_xmit = gzd_en_tx,
    .ndo_do_ioctl = gzd_en_ioctl,
    .ndo_get_stats = gzd_en_stats,
    .ndo_tx_timeout = gzd_en_tx_timeout,
};

static const struct gzd_en_stat {
    char name[ETH_GSTRING_LEN];
    size_t offset;
} gzd_en_ethtool_stats[] = {
    { "tx_overhead", offsetof(struct gzd_en_card_data, tx_overhead) },
    { "tx_req_resp", offsetof(struct gzd_en_card_data, tx_req_resp) },
};

static void
gzd_en_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *drvinfo)
{
    struct gzd_en_netdev_priv *priv = netdev_priv(dev);
    struct gzd_en_card_data *card = priv->card_data;

    strlcpy(drvinfo->driver, GZD_EN_DRV_NAME, sizeof(drvinfo->driver));
    strlcpy(drvinfo->version, GZD_DRV_VERS, sizeof(drvinfo->version));
    strlcpy(drvinfo->bus_info, pci_name(card->core_info->pdev),
	    sizeof(drvinfo->bus_info));
}

static void
gzd_en_get_ethtool_stats(struct net_device *dev,
			 struct ethtool_stats *stats, u64 * data)
{
    struct gzd_en_netdev_priv *priv = netdev_priv(dev);
    struct gzd_en_card_data *card = priv->card_data;
    int i;

    for (i = 0; i < ARRAY_SIZE(gzd_en_ethtool_stats); i++)
	data[i] =
	    *(u64 *) ((char *) card + gzd_en_ethtool_stats[i].offset);
}

static int gzd_en_get_sset_count(struct net_device *dev, int sset)
{
    switch (sset) {
    case ETH_SS_STATS:
	return ARRAY_SIZE(gzd_en_ethtool_stats);
    }

    return -EOPNOTSUPP;
}

static void gzd_en_get_strings(struct net_device *dev, u32 sset, u8 * data)
{
    int i;

    switch (sset) {
    case ETH_SS_STATS:
	for (i = 0; i < ARRAY_SIZE(gzd_en_ethtool_stats); i++)
	    memcpy(data + (i * ETH_GSTRING_LEN),
		   gzd_en_ethtool_stats[i].name, ETH_GSTRING_LEN);
	break;
    }
}

static int
gzd_en_get_ts_info(struct net_device *dev, struct ethtool_ts_info *info)
{
    info->so_timestamping = SOF_TIMESTAMPING_TX_SOFTWARE |
	SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;

    return 0;
}

static const struct ethtool_ops gzd_en_ethtool_ops = {
    .get_drvinfo = gzd_en_get_drvinfo,
    .get_ethtool_stats = gzd_en_get_ethtool_stats,
    .get_sset_count = gzd_en_get_sset_count,
    .get_strings = gzd_en_get_strings,
    .get_ts_info = gzd_en_get_ts_info
};

void gzd_net_init(struct net_device *dev)
{
    struct gzd_en_netdev_priv *priv;

    priv = netdev_priv(dev);
    memset(priv, 0, sizeof(struct gzd_en_netdev_priv));
    spin_lock_init(&priv->lock);
    priv->dev = dev;

    ether_setup(dev);
    dev->netdev_ops = &gzd_en_netdev_ops;
    dev->ethtool_ops = &gzd_en_ethtool_ops;
}

module_init(gzd_en_module_init);
module_exit(gzd_en_fini);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Ethernet driver for the Gen-Z demo pci device");
