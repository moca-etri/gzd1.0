/*
 * file : gzd.h
 * desc : header file for the Gen-Z demo PCIe device drivers
 *
 * Author:  Jim Hull <jim.hull@hpe.com>
 *
 * Copyright:
 *     © Copyright 2016-2017, Hewlett Packard Enterprise Development LP
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

#include <linux/list.h>
#include <linux/dma-mapping.h>

#define GZD_DRV_VERS "4.0"
#define MAX_CARDS 16u  /* total across all systems and Gen-Z subnets */
#define MAX_MEDIA MAX_CARDS
#define MAX_ZLINKS 6
#define MAX_MZLINKS 2
#define MAX_AURORA 4

#define ZLINK_MASK(_mask, _index) ((_mask) & (1u << (_index)))
#define AU_MASK(_mask, _index) ZLINK_MASK(_mask, (_index + 2))

#define KB(x)((x) * 1024ul)
#define MB(x)(KB(KB(x)))
#define GB(x)(MB(KB(x)))

//swsok
#define OLD_CHAR_READ_WRITE	//8byte alignment needed
#ifndef OLD_CHAR_READ_WRITE
//#define HYBRID_CHAR_READ_WRITE
#endif

enum {
	BlockRWInt      = 0,
	MsgSendInt      = 1,
	MsgRecvInt      = 2,
	ByteAddrErrInt  = 3,
	AddrTransErrInt = 4,
	HWApiErrInt     = 5,
	BrSeqErrInt     = 6
};

struct gzd_media_info {
	uint media_num;
	ulong size;  /* in bytes */
	phys_addr_t mmio_addr;
	void __iomem *base_addr;  /* from pci_iomap */
	uint cid;
	ulong bdevs_offset; /* cdevs start at 0, bdevs at this offset */
};

struct gzd_msg_info {
	uint scid;
	uint dcid;
};

struct gzd_core_info {
	uint num_cards;
	uint num_media;
	uint card_ids[MAX_CARDS];
	uint genz_subnets;
	uint card_id;
	uint card_index;
	uint32_t zlink_mask;
	uint req_fifo_depth;
	uint resp_fifo_depth;
	struct gzd_msg_tbl_entry *msg_tbl;
	struct pci_dev *pdev;
	struct gzd_media_info media_info[MAX_MEDIA];
	struct gzd_msg_info msg_info[MAX_CARDS];
	uint64_t link_status[MAX_AURORA];
	uint intf_state[MAX_ZLINKS+MAX_MZLINKS];
};

struct gzd_msg_tbl_entry {
	uint scid;
	dma_addr_t buf_start;           /* 16-byte alignment */
	void      *buf_start_va;        /* kernel VA for buf_start */
	size_t     buf_size;
	dma_addr_t prod_ptr_fixed_loc;  /*  8-byte alignment */
	uint64_t  *prod_ptr_fixed_va;   /* kernel VA for prod_ptr_fixed_loc */
	dma_addr_t prod_ptr;            /* return value - full addr, not offset */
	void      *prod_ptr_va;         /* kernel VA for prod_ptr */
	dma_addr_t cons_ptr;            /* 16-byte alignment */
	void      *cons_ptr_va;         /* kernel VA for cons_ptr */
};

struct gzd_msg_header {
	ushort   len;
	ushort   scid;  /* really only 11 bits */
	uint     reserved;
	uint64_t unused_prod_ptr;
};

struct gzd_driver {
	struct list_head list;
	const char *name;
	/* function pointers provided by dependent driver */
	int (*probe)(void *);  /* Revisit: what args? */
	void (*remove)(void *); /* Revisit: what args? */
	/* function pointers set by gzd-core */
	struct gzd_core_info * (*info)(void *hw);
	int (*request_irq)(void *hw, uint irq_index, irq_handler_t handler,
			   ulong irqflags, const char *devname, void *dev_id);
	int (*free_irq)(void *hw, uint irq_index, void *dev_id);
	int (*block_io_request)(void *hw, dma_addr_t host_addr,
				uint64_t genz_addr, uint len,
				uint req_id, uint dcid, uint rw);
	int (*block_io_response)(void *hw, uint *req_id, uint *free_reqs);
	int (*msg_request)(void *hw, dma_addr_t host_addr, uint len,
			   uint req_id, uint dcid);
	int (*msg_response)(void *hw, uint *req_id, uint *free_reqs);
	int (*msg_tbl_init)(void *hw,
			    struct gzd_msg_tbl_entry *tbl, uint entries);
	size_t (*msg_update_cons_ptr)(void *hw,
				struct gzd_msg_tbl_entry *tbl, uint index,
				size_t size);
	uint64_t (*msg_read_prod_ptr)(void *hw,
				struct gzd_msg_tbl_entry *tbl, uint index);
};

extern int gzd_core_register_driver(struct gzd_driver *);
extern void gzd_core_unregister_driver(struct gzd_driver *);

#define PRINT_LINE(f_, ...) \
        do{\
                printk(KERN_WARNING "%s %s %d - " f_ "\n", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);\
        }while(0);
