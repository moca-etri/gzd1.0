/*
 * file : gzd-core.c
 * desc : core linux device driver for the Gen-Z demo PCIe device
 *
 * Author:  Jim Hull <jim.hull@hpe.com>
 *          Siro Mugabi, nairobi-embedded.org
 *          Cam Macdonell <cam@cs.ualberta.ca>
 *
 * Copyright:
 *     © Copyright 2016-2017 Hewlett Packard Enterprise Development LP
 *
 * notes: Based on a skeleton driver by Siro Mugabi, nairobi-embedded.org,
 *        http://nairobi-embedded.org/src/ne_ivshmem_guest/ne_ivshmem_ldd_basic.c
 *        which itself was based on "kvm_ivshmem.c" by
 *        Cam Macdonell <cam@cs.ualberta.ca>, Copyright 2009, GPLv2
 *        See git://gitorious.org/nahanni/guest-code.git
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
#include <linux/interrupt.h>
#include <linux/ratelimit.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/bitmap.h>
#include <linux/workqueue.h>
#include <linux/delay.h>	/* Revisit: temporary */
#include <linux/timex.h>
#include <linux/kthread.h>
#include "gzd.h"


#include <asm/e820/api.h>

#include <asm/mtrr.h>

#define GZD_CORE_DEV_NAME "gzd-core"

static struct workqueue_struct *gzd_core_wq = 0;

static uint card_ids[MAX_CARDS] = { 0, -1 };

static int card_ids_argc;
module_param_array(card_ids, uint, &card_ids_argc, S_IRUGO);
MODULE_PARM_DESC(card_ids, "list of card ids");

static ulong media_size[MAX_MEDIA] = { 1024, -1 };

static int media_size_argc;
module_param_array(media_size, ulong, &media_size_argc, S_IRUGO);
MODULE_PARM_DESC(media_size, "list of media sizes, in mebibytes");

static uint genz_subnets = 1;
module_param(genz_subnets, int, S_IRUGO);
MODULE_PARM_DESC(genz_subnets, "number of subnets in Gen-Z network");

static int msg_test = 0;
module_param(msg_test, int, S_IRUGO);
MODULE_PARM_DESC(msg_test, "enable sysfs msg test");

// swsok, for PAT setting of BAR2 region
static int pat = 3;		// 0:WB, 1:WT, 2:WC, 3:UC
module_param(pat, int, S_IRUGO);
MODULE_PARM_DESC(pat, "select PAT attribute for BAR2 region");


/*
 * Revisit: temporary 
 */
static uint msg_intr_udelay = 0;
module_param(msg_intr_udelay, int, S_IRUGO);
MODULE_PARM_DESC(msg_intr_udelay,
		 "number of usecs to delay in msg_recv_interrupt");

static char *recv_buf_size = "2M";
module_param(recv_buf_size, charp, S_IRUGO);
MODULE_PARM_DESC(recv_buf_size,
		 "size of msg recv buffer in bytes, with optional [KMG]");

static uint red_green = 0;
module_param(red_green, int, S_IRUGO);
MODULE_PARM_DESC(red_green, "enable when running on a red-green FPGA");

static uint dma_mask = 64;
module_param(dma_mask, int, S_IRUGO);
MODULE_PARM_DESC(dma_mask, "set DMA mask bits");

static uint fpga_rev_check = 1;
module_param(fpga_rev_check, int, S_IRUGO);
MODULE_PARM_DESC(fpga_rev_check, "check for compatible FPGA revision");

static uint poll_interval = 500;
module_param(poll_interval, uint, S_IRUGO);
MODULE_PARM_DESC(poll_interval, "interface poll interval in millisecs");

static uint zlink_masks[MAX_CARDS] = { 0 };

static int zlink_masks_argc;
module_param_array(zlink_masks, uint, &zlink_masks_argc, S_IRUGO);
MODULE_PARM_DESC(zlink_masks, "masks of zlinks to enable");

/*
 * ============================================================ PCI
 * SPECIFIC ============================================================ 
 */
#include <linux/pci.h>

#ifndef ioread64
#ifdef readq
#define ioread64 readq
#else
#error Platform has no useable ioread64
#endif
#endif

#ifndef iowrite64
#ifdef writeq
#define iowrite64 writeq
#else
#error Platform has no useable iowrite64
#endif
#endif

#define NUM_TEST_REQ_IDS 64

enum {
    GzdNoIrq,			/* 0 */
    GzdCoreIrq,			/* 1 */
    GzdDepIrq,			/* 2 */
    GzdIrqTypes			/* 3 */
};

struct gzd_core_pci_info {
    uint32_t zlink_mask;	/* supported Zlinks */
};

static struct gzd_core_pci_info gzd_core_pci_info_table[] = {
    { 0x07 },			/* AlphaData - Zlinks 0-2 only */
    { 0x3f }			/* Bittware - all 6 Zlinks */
};

struct gzd_core_intr {
    uint irq_type;
    struct {
	irq_handler_t handler;
	ulong irqflags;
	const char *devname;
	void *dev_id;
    } irq_save[GzdIrqTypes];
};

#define GZD_CORE_MSI_COUNT 7

struct gzd_core_hw {
    struct list_head list;
    struct pci_dev *pdev;
    struct gzd_core_pci_info *pci_info;
    struct gzd_core_info info;
    uint card_index;
    /*
     * (mmio) control registers, i.e. the "register memory region" 
     */
    spinlock_t lock;		/* protects all control register accesses */
    void __iomem *regs_base_addr;	/* from pci_iomap */
    resource_size_t regs_start;
    resource_size_t regs_len;
    /*
     * media mmio region 
     */
    void __iomem *media_base_addr;	/* from pci_iomap */
    resource_size_t media_mmio_start;
    resource_size_t media_mmio_len;
    /*
     * irq handling 
     */
    unsigned int base_irq;
    struct gzd_core_intr irq_info[GZD_CORE_MSI_COUNT];
    /*
     * switch core route programming 
     */
    uint64_t sci_index;
    uint64_t sci_cid;
    uint64_t sci_route;
    /*
     * memory map testing 
     */
    uint64_t memory_map_offset;
    size_t memory_map_size;
    /*
     * block I/O testing 
     */
    uint64_t block_io_offset;
    size_t block_io_size;
    uint block_io_cardid;
    wait_queue_head_t block_io_queue;
     DECLARE_BITMAP(block_io_done, NUM_TEST_REQ_IDS);
    /*
     * messaging testing 
     */
    uint64_t msg_offset;
    uint msg_cardid;
    wait_queue_head_t msg_send_queue;
    wait_queue_head_t msg_recv_queue;
     DECLARE_BITMAP(msg_done, NUM_TEST_REQ_IDS);
    struct gzd_msg_tbl_entry msg_tbl;
    atomic_long_t msg_recv_waiting;
    atomic_long_t msg_recv_serving;
    struct mutex msg_recv_lock;
    struct list_head msg_recv_list;
    /*
     * block I/O & messaging testing 
     */
    uint test_min_req_id;
    uint test_max_req_id;
    uint test_req_id;
    /*
     * FIFO management 
     */
    uint block_num_reqs;
    uint block_num_resps;
    uint msg_num_reqs;
    uint msg_num_resps;
    /*
     * counters 
     */
    uint64_t block_io_request_count;
    uint64_t block_io_fifo_full_count;
    uint64_t block_io_fifo_overflow_count;
    uint64_t block_io_response_count;
    uint64_t block_io_nak_count;
    uint64_t msg_request_count;
    uint64_t msg_fifo_full_count;
    uint64_t msg_fifo_overflow_count;
    uint64_t msg_response_count;
    uint64_t msg_nak_count;
    /*
     * polling thread 
     */
    struct task_struct *thread;

    /*
     * MTRR handle
     */
    int mtrr_handle;
};

#define GZD_CORE_REG_BAR 0
#define GZD_CORE_MEDIA_BAR 2

DEFINE_MUTEX(gzd_core_lock);	/* used only during initialization */
static uint gzd_core_card_index = 0;

LIST_HEAD(gzd_core_hw_list);
LIST_HEAD(gzd_core_driver_list);

#define PCI_VENDOR_ID_ETRI      0x1058
static struct pci_device_id gzd_core_id_table[] = {
    { PCI_VDEVICE(HP_3PAR, 0x020B), 0 },	/* AlphaData v0.7*/
    { PCI_VDEVICE(HP_3PAR, 0x8032), 1 },	/* AlphaData v1.0*/
//    { PCI_VDEVICE(HP_3PAR, 0x0279), 1 },	/* Bittware */
//    { PCI_VDEVICE(HP_3PAR, 0x9024), 2 },	/* Bittware */
//    { PCI_VDEVICE(HP_3PAR, 0x0056), 3 },	/* Bittware */
    { PCI_VDEVICE(ETRI, 0x8034), 2 },	/* ETRI v1.0 */
    { PCI_VDEVICE(ETRI, 0x8035), 3 },	/* ETRI v1.0(BAR2+BAR4) */
    { PCI_VDEVICE(ETRI, 0x9034), 4 },	/* ETRI v1.0 250Mhz */
    { 0 },
};

MODULE_DEVICE_TABLE(pci, gzd_core_id_table);

#define GZD_CORE_IRQ_ID "gzd-core"

enum {
    ExpectedFpgaMajor = 4,	/* Must match FPGA exactly */
    ExpectedFpgaMinor = 0	/* FPGA should be this value or higher */
};

enum {
    BrReqCID = 0x00,		/* Bridge Requester CID */
    BrRespCID = 0x10,		/* Bridge Responder CID */
    MediaRespCID = 0x20,	/* Media Responder CID */
    MediaResp2CID = 0x30,	/* Media Responder 2 CID */
};

enum {
    IntrDisable = 0,
    IntrEnable = 1
};

/*
 * relevant control register offsets 
 */
enum {
    Host = 0x00,		/* Register Converter Block CRAB access */
    BrReq = Host + 0x00000,	/* Bridge Requester */
    BrResp = Host + 0x01000,	/* Bridge Responder */
    MediaResp = Host + 0x02000,	/* Media Responder */
    MediaResp2 = Host + 0x03000,	/* Media Responder */
    ZLink0 = Host + 0x04000,	/* Zlink 0 */
    LinkStatusCtl0 = ZLink0 + 0x008,	/* Zlink 0 Control/Status */
    ZLink1 = Host + 0x05000,	/* Zlink 1 */
    LinkStatusCtl1 = ZLink1 + 0x008,	/* Zlink 1 Control/Status */
    ZLink2 = Host + 0x06000,	/* Zlink 2 */
    LinkStatusCtl2 = ZLink2 + 0x008,	/* Zlink 2 Control/Status */
    ZLink3 = Host + 0x07000,	/* Zlink 3 */
    LinkStatusCtl3 = ZLink3 + 0x008,	/* Zlink 3 Control/Status */
    ZLink4 = Host + 0x08000,	/* Zlink 4 */
    LinkStatusCtl4 = ZLink4 + 0x008,	/* Zlink 4 Control/Status */
    ZLink5 = Host + 0x09000,	/* Zlink 5 */
    LinkStatusCtl5 = ZLink5 + 0x008,	/* Zlink 5 Control/Status */
    MZLink0 = Host + 0x0a000,	/* Media Zlink 0 */
    MLinkStatusCtl0 = MZLink0 + 0x008,	/* Media Zlink 0 Control/Status */
    MZLink1 = Host + 0x0b000,	/* Media Zlink 1 */
    MLinkStatusCtl1 = MZLink1 + 0x008,	/* Media Zlink 1 Control/Status */
    Aurora0 = Host + 0x0c000,	/* Aurora Interface 0 */
    AuStatus0 = Aurora0 + 0xf18,	/* Aurora 0 Status */
    Aurora1 = Host + 0x0d000,	/* Aurora Interface 1 */
    AuStatus1 = Aurora1 + 0xf18,	/* Aurora 1 Status */
    Aurora2 = Host + 0x0e000,	/* Aurora Interface 2 */
    AuStatus2 = Aurora2 + 0xf18,	/* Aurora 2 Status */
    Aurora3 = Host + 0x0f000,	/* Aurora Interface 3 */
    AuStatus3 = Aurora3 + 0xf18,	/* Aurora 3 Status */
    SCI00 = Host + 0x10000,	/* Switch Core Input 00 */
    SCI01 = Host + 0x11000,	/* Switch Core Input 01 */
    SCI02 = Host + 0x12000,	/* Switch Core Input 02 */
    SCI03 = Host + 0x13000,	/* Switch Core Input 03 */
    SCI04 = Host + 0x14000,	/* Switch Core Input 04 */
    SCI05 = Host + 0x15000,	/* Switch Core Input 05 */
    SCI06 = Host + 0x16000,	/* Switch Core Input 06 */
    SCI07 = Host + 0x17000,	/* Switch Core Input 07 */
    SCO00 = Host + 0x30000,	/* Switch Core Output 00 */
    SCO01 = Host + 0x31000,	/* Switch Core Output 01 */
    SCO02 = Host + 0x32000,	/* Switch Core Output 02 */
    SCO03 = Host + 0x33000,	/* Switch Core Output 03 */
    SCO04 = Host + 0x34000,	/* Switch Core Output 04 */
    SCO05 = Host + 0x35000,	/* Switch Core Output 05 */
    SCO06 = Host + 0x36000,	/* Switch Core Output 06 */
    SCO07 = Host + 0x37000,	/* Switch Core Output 07 */
    HwAPI = 0x100000,		/* HW API Registers */
    HwID = HwAPI + 0x00,	/* FPGA/Block IDs */
    BlockHostAddr = HwAPI + 0x08,	/* Blk Host Byte Address */
    BlockGenZAddr = HwAPI + 0x10,	/* Blk Gen-Z Byte Address */
    BlockRequest = HwAPI + 0x18,	/* Blk Request ID/Len/DCID */
    BlockControl = HwAPI + 0x20,	/* Blk Req/Resp Control/Status */
    MsgHostAddr = HwAPI + 0x28,	/* Msg Host Byte Address */
    MsgGenZAddr = HwAPI + 0x30,	/* Msg Gen-Z Byte Address */
    MsgRequest = HwAPI + 0x38,	/* Msg Request ID/Len/DCID */
    MsgControl = HwAPI + 0x40,	/* Msg Req/Resp Control/Status */
    HwParams = HwAPI + 0x48,	/* HW Block Parameters */
    AddrTran = 0x101000,	/* Address Translation Table */
    AtID = AddrTran + 0x00,	/* AddrTran IDs */
    AtTeWrStart = AddrTran + 0x08,	/* Tbl Entry Wr Start Addr */
    AtTeWrEnd = AddrTran + 0x10,	/* Tbl Entry Wr End Addr */
    AtTeWrDCID = AddrTran + 0x18,	/* Tbl Entry Wr DCID */
    AtTeRdStart = AddrTran + 0x20,	/* Tbl Entry Rd Start Addr */
    AtTeRdEnd = AddrTran + 0x28,	/* Tbl Entry Rd End Addr */
    AtTeRdDCID = AddrTran + 0x30,	/* Tbl Entry Rd DCID */
    AtTeControl = AddrTran + 0x38,	/* Tbl Entry Control */
    AtTblControl = AddrTran + 0x40,	/* Table Control */
    AtStatus = AddrTran + 0x48,	/* AddrTran Status */
    AtError = AddrTran + 0x50,	/* AddrTran Error */
    AtParams = AddrTran + 0x58,	/* AddrTran Block Parameters */
    SeqCtrl = 0x102000,		/* Sequencer Control */
    SeqID = SeqCtrl + 0x00,	/* SeqCntrl IDs */
    SeqTeWrSCID = SeqCtrl + 0x08,	/* Tbl Entry Wr SCID */
    SeqTeWrBufStart = SeqCtrl + 0x10,	/* Tbl Entry Wr Buf Start Addr */
    SeqTeWrBufEnd = SeqCtrl + 0x18,	/* Tbl Entry Wr Buf End Addr */
    SeqTeWrProdPtr = SeqCtrl + 0x20,	/* Tbl Entry Wr Producer Ptr Addr */
    SeqTeWrConsPtr = SeqCtrl + 0x28,	/* Tbl Entry Wr Consumer Ptr */
    SeqTeRdSCID = SeqCtrl + 0x30,	/* Tbl Entry Rd SCID */
    SeqTeRdBufStart = SeqCtrl + 0x38,	/* Tbl Entry Rd Buf Start Addr */
    SeqTeRdBufEnd = SeqCtrl + 0x40,	/* Tbl Entry Rd Buf End Addr */
    SeqTeRdProdPtr = SeqCtrl + 0x48,	/* Tbl Entry Rd Producer Ptr Addr */
    SeqTeRdConsPtr = SeqCtrl + 0x50,	/* Tbl Entry Rd Consumer Ptr */
    SeqTeControl = SeqCtrl + 0x58,	/* Tbl Entry Control */
    SeqTblControl = SeqCtrl + 0x60,	/* Tbl Control */
    SeqResetEnable = SeqCtrl + 0x68,	/* Reset/Enable Control */
    SeqStatus = SeqCtrl + 0x70,	/* Seq Status */
    SeqParams = SeqCtrl + 0x78,	/* Seq Block Parameters */
    EccCtrl = 0x200000,		/* DDR4 ECC Control */
    EccStatus = EccCtrl,	/* DDR4 ECC Status R/W */
    EccEnIrq = EccCtrl + 0x004,	/* DDR4 ECC Enable Interrupt R/W */
    EccOnOff = EccCtrl + 0x008,	/* DDR4 ECC On/Off R/W */
    EccCeCnt = EccCtrl + 0x00c,	/* Correctable Error Count R/W */
    EccCeFfd0 = EccCtrl + 0x100,	/* Correctable Error First Failing 
					 * Data[31:00] RO */
    EccCeFfd1 = EccCtrl + 0x104,	/* Correctable Error First Failing 
					 * Data[63:32] RO */
    EccCeFfd2 = EccCtrl + 0x108,	/* Correctable Error First Failing 
					 * Data[95:64] RO */
    EccCeFfd3 = EccCtrl + 0x10c,	/* Correctable Error First Failing 
					 * Data[127:96] RO */
    EccCeFfe = EccCtrl + 0x180,	/* Correctable Error First Failing ECC RO */
    EccCeFfa0 = EccCtrl + 0x1c0,	/* Correctable Error First Failing 
					 * address[31:00] RO */
    EccCeFfa1 = EccCtrl + 0x1c4,	/* Uncorrectable Error First
					 * Failing address[63:32] RO */
    EccUeFfd0 = EccCtrl + 0x200,	/* Uncorrectable Error First
					 * Failing Data[31:00] RO */
    EccUeFfd1 = EccCtrl + 0x204,	/* Uncorrectable Error First
					 * Failing Data[63:32] RO */
    EccUeFfd2 = EccCtrl + 0x208,	/* Uncorrectable Error First
					 * Failing Data[95:64] RO */
    EccUeFfd3 = EccCtrl + 0x20c,	/* Uncorrectable Error First
					 * Failing Data[127:96] RO */
    EccUeFfe = EccCtrl + 0x280,	/* Uncorrectable Error First Failing ECC
				 * RO */
    EccUeFfa0 = EccCtrl + 0x2c0,	/* Uncorrectable Error First
					 * Failing address[31:00] RO */
    EccUeFfa1 = EccCtrl + 0x2c4,	/* Uncorrectable Error First
					 * Failing address[63:32] RO */
    EccFiD0 = EccCtrl + 0x300,	/* Fault Inject Data [31:00] WO */
    EccFiD1 = EccCtrl + 0x304,	/* Fault Inject Data [63:32] WO */
    EccFiD2 = EccCtrl + 0x308,	/* Fault Inject Data [95:64] WO */
    EccFiD3 = EccCtrl + 0x30c,	/* Fault Inject Data [127:96] WO */
    EccFiEcc = EccCtrl + 0x380,	/* Fault Inject ECC WO */

};

#define I_DOWN 0x0
#define I_CFG  0x1
#define I_UP   0x2
#define I_LP   0x3
#define INTERFACE_STATE_MASK 0x7ull
#define INTERFACE_ENABLE (1ull<<16)
#define INTERFACE_RESET  (1ull<<18)
static const char *interface_state_name[] =
    { "I-Down", "I-CFG", "I-Up", "I-LP",
    "Rv4", "Rv5", "Rv6", "Rv7"
};

/*
 * ============================================================ BLUE BLOCK 
 * INITIALIZATION
 * ============================================================ 
 */
struct gzd_core_field {
    uint pos;			/* rightmost bit number */
    uint len;			/* field length, with 0 meaning "unused" */
};

struct gzd_core_reg_init {
    const uint64_t start_offset;
    const uint64_t end_offset;
    const uint64_t value;
    const uint64_t read_mask;
    const struct gzd_core_field n;
    const struct gzd_core_field m;
    const struct gzd_core_field z;
};

struct gzd_core_block_init {
    const uint64_t offset;
    const struct gzd_core_reg_init *const table;
    const uint64_t m;
};

static const struct gzd_core_reg_init br_req_init[] = {
    { 0x118, 0x118, 0x0000000000000000 },
    { 0x128, 0x128, 0x4040000000000501 },
    { 0x138, 0x138, 0x0000000000000000 },
    { 0x188, 0x188, 0x0000000000000000 },
    { 0x080, 0x080, 0x0000000000009009 },
    { 0x090, 0x090, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000300000000, 0xff00ffff00000000 },
    { 0x430, 0x430, 0x000000FF00000000 },
    { 0x510, 0x510, 0x0000000100000001, 0, { 11, 4}, { 4, 7}, { 0, 0} },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init resp_init[] = {
    { 0x118, 0x118, 0x0000000000000000 },
    { 0x128, 0x128, 0x8080000000000A02 },
    { 0x138, 0x138, 0x0000000000000000 },
    { 0x168, 0x168, 0x0000000000000000 },
    { 0x080, 0x080, 0x0000000000009000 },
    { 0x010, 0x010, 0x0000000100000000, 0xffffffff00000000 },
    { 0x400, 0x400, 0x0000000300000000, 0xff00ffff00000000 },
    { 0x430, 0x430, 0x000000FF00000000 },
    { 0x510, 0x510, 0x0000000100000001, 0, { 11, 4}, { 4, 7}, { 0, 0} },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init link_init[] = {
    { 0x008, 0x008, 0x00000000000000f8, 0x00000000ffffff00 },
    { 0x010, 0x010, 0x0000030000000008 },
    { 0xfff, 0xfff, 0 }
};

/*
 * card 0 switch core input 
 */
static const struct gzd_core_reg_init c0_sci00_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000000000000 },	/* c0 br req - illegal */
    { 0x408, 0x408, 0x0000000000000101 },	/* c0 br rsp */
    { 0x410, 0x410, 0x0000000000000102 },	/* c0 media 1 rsp */
    { 0x418, 0x418, 0x0000000000000103 },	/* c0 media 2 rsp */
    { 0x420, 0x440, 0x0000000000000000 },
    { 0x448, 0x448, 0x0000000000000104 },	/* c1 br rsp */
    { 0x450, 0x450, 0x0000000000000104 },	/* c1 media 1 rsp */
    { 0x458, 0x458, 0x0000000000000104 },	/* c1 media 2 rsp */
    { 0x460, 0x480, 0x0000000000000000 },
    { 0x488, 0x488, 0x0000000000000105 },	/* c2 br rsp (via c3) */
    { 0x490, 0x490, 0x0000000000000105 },	/* c2 media 1 rsp (via c3) */
    { 0x498, 0x498, 0x0000000000000105 },	/* c2 media 2 rsp (via c3) */
    { 0x4a0, 0x4c0, 0x0000000000000000 },
    { 0x4c8, 0x4c8, 0x0000000000000105 },	/* c3 br rsp */
    { 0x4d0, 0x4d0, 0x0000000000000105 },	/* c3 media 1 rsp */
    { 0x4d8, 0x4d8, 0x0000000000000105 },	/* c3 media 2 rsp */
    { 0x4e0, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c0_sci01_03_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000000000100 },	/* c0 br req */
    { 0x408, 0x438, 0x0000000000000000 },
    { 0x440, 0x440, 0x0000000000000104 },	/* c1 br req */
    { 0x448, 0x478, 0x0000000000000000 },
    { 0x480, 0x480, 0x0000000000000105 },	/* c2 br req (via c3) */
    { 0x488, 0x4b8, 0x0000000000000000 },
    { 0x4c0, 0x4c0, 0x0000000000000105 },	/* c3 br req */
    { 0x4c8, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c0_sci04_06_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000000000100 },	/* c0 br req */
    { 0x408, 0x408, 0x0000000000000101 },	/* c0 br rsp */
    { 0x410, 0x410, 0x0000000000000102 },	/* c0 media 1 rsp */
    { 0x418, 0x418, 0x0000000000000103 },	/* c0 media 2 rsp */
    { 0x420, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c0_sci07_init[] = {	/* unused */
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x7f8, 0x0000000000000000 },	/* clear entire table */
    { 0xfff, 0xfff, 0 }
};

/*
 * card 1 switch core input 
 */
static const struct gzd_core_reg_init c1_sci00_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000000000000 },	/* c0 br req - illegal */
    { 0x408, 0x408, 0x0000000000000104 },	/* c0 br rsp */
    { 0x410, 0x410, 0x0000000000000104 },	/* c0 media 1 rsp */
    { 0x418, 0x418, 0x0000000000000104 },	/* c0 media 2 rsp */
    { 0x420, 0x440, 0x0000000000000000 },
    { 0x448, 0x448, 0x0000000000000101 },	/* c1 br rsp */
    { 0x450, 0x450, 0x0000000000000102 },	/* c1 media 1 rsp */
    { 0x458, 0x458, 0x0000000000000103 },	/* c1 media 2 rsp */
    { 0x460, 0x480, 0x0000000000000000 },
    { 0x488, 0x488, 0x0000000000000105 },	/* c2 br rsp */
    { 0x490, 0x490, 0x0000000000000105 },	/* c2 media 1 rsp */
    { 0x498, 0x498, 0x0000000000000105 },	/* c2 media 2 rsp */
    { 0x4a0, 0x4c0, 0x0000000000000000 },
    { 0x4c8, 0x4c8, 0x0000000000000105 },	/* c3 br rsp (via c2) */
    { 0x4d0, 0x4d0, 0x0000000000000105 },	/* c3 media 1 rsp (via c2) */
    { 0x4d8, 0x4d8, 0x0000000000000105 },	/* c3 media 2 rsp (via c2) */
    { 0x4e0, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c1_sci01_03_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000000000104 },	/* c0 br req */
    { 0x408, 0x438, 0x0000000000000000 },
    { 0x440, 0x440, 0x0000000000000100 },	/* c1 br req */
    { 0x448, 0x478, 0x0000000000000000 },
    { 0x480, 0x480, 0x0000000000000105 },	/* c2 br req */
    { 0x488, 0x4b8, 0x0000000000000000 },
    { 0x4c0, 0x4c0, 0x0000000000000105 },	/* c3 br req (via c2) */
    { 0x4c8, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c1_sci04_06_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x438, 0x0000000000000000 },
    { 0x440, 0x440, 0x0000000000000100 },	/* c1 br req */
    { 0x448, 0x448, 0x0000000000000101 },	/* c1 br rsp */
    { 0x450, 0x450, 0x0000000000000102 },	/* c1 media 1 rsp */
    { 0x458, 0x458, 0x0000000000000103 },	/* c1 media 2 rsp */
    { 0x460, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c1_sci07_init[] = {	/* unused */
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x7f8, 0x0000000000000000 },	/* clear entire table */
    { 0xfff, 0xfff, 0 }
};

/*
 * card 2 switch core input 
 */
static const struct gzd_core_reg_init c2_sci00_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000000000000 },	/* c0 br req - illegal */
    { 0x408, 0x408, 0x0000000000000104 },	/* c0 br rsp (via c3) */
    { 0x410, 0x410, 0x0000000000000104 },	/* c0 media 1 rsp (via c3) */
    { 0x418, 0x418, 0x0000000000000104 },	/* c0 media 2 rsp (via c3) */
    { 0x420, 0x440, 0x0000000000000000 },
    { 0x448, 0x448, 0x0000000000000105 },	/* c1 br rsp */
    { 0x450, 0x450, 0x0000000000000105 },	/* c1 media 1 rsp */
    { 0x458, 0x458, 0x0000000000000105 },	/* c1 media 2 rsp */
    { 0x460, 0x480, 0x0000000000000000 },
    { 0x488, 0x488, 0x0000000000000101 },	/* c2 br rsp */
    { 0x490, 0x490, 0x0000000000000102 },	/* c2 media 1 rsp */
    { 0x498, 0x498, 0x0000000000000103 },	/* c2 media 2 rsp */
    { 0x4a0, 0x4c0, 0x0000000000000000 },
    { 0x4c8, 0x4c8, 0x0000000000000104 },	/* c3 br rsp */
    { 0x4d0, 0x4d0, 0x0000000000000104 },	/* c3 media 1 rsp */
    { 0x4d8, 0x4d8, 0x0000000000000104 },	/* c3 media 2 rsp */
    { 0x4e0, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c2_sci01_03_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000000000104 },	/* c0 br req (via c3) */
    { 0x408, 0x438, 0x0000000000000000 },
    { 0x440, 0x440, 0x0000000000000105 },	/* c1 br req */
    { 0x448, 0x478, 0x0000000000000000 },
    { 0x480, 0x480, 0x0000000000000100 },	/* c2 br req */
    { 0x488, 0x4b8, 0x0000000000000000 },
    { 0x4c0, 0x4c0, 0x0000000000000104 },	/* c3 br req */
    { 0x4c8, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c2_sci04_06_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x438, 0x0000000000000000 },
    { 0x440, 0x440, 0x0000000000000105 },	/* c1 br req (relay c3) */
    { 0x448, 0x448, 0x0000000000000105 },	/* c1 br rsp (relay c3) */
    { 0x450, 0x450, 0x0000000000000105 },	/* c1 media 1 rsp (relay c3) */
    { 0x458, 0x458, 0x0000000000000105 },	/* c1 media 2 rsp (relay c3) */
    { 0x460, 0x478, 0x0000000000000000 },
    { 0x480, 0x480, 0x0000000000000100 },	/* c2 br req */
    { 0x488, 0x488, 0x0000000000000101 },	/* c2 br rsp */
    { 0x490, 0x490, 0x0000000000000102 },	/* c2 media 1 rsp */
    { 0x498, 0x498, 0x0000000000000103 },	/* c2 media 2 rsp */
    { 0x4a0, 0x4b8, 0x0000000000000000 },
    { 0x4c0, 0x4c0, 0x0000000000000104 },	/* c3 br req (relay c1) */
    { 0x4c8, 0x4c8, 0x0000000000000104 },	/* c3 br rsp (relay c1) */
    { 0x4d0, 0x4d0, 0x0000000000000104 },	/* c3 media 1 rsp (relay c1) */
    { 0x4d8, 0x4d8, 0x0000000000000104 },	/* c3 media 2 rsp (relay c1) */
    { 0x4e0, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c2_sci07_init[] = {	/* unused */
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x7f8, 0x0000000000000000 },	/* clear entire table */
    { 0xfff, 0xfff, 0 }
};

/*
 * card 3 switch core input 
 */
static const struct gzd_core_reg_init c3_sci00_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000000000000 },	/* c0 br req - illegal */
    { 0x408, 0x408, 0x0000000000000104 },	/* c0 br rsp */
    { 0x410, 0x410, 0x0000000000000104 },	/* c0 media 1 rsp */
    { 0x418, 0x418, 0x0000000000000104 },	/* c0 media 2 rsp */
    { 0x420, 0x440, 0x0000000000000000 },
    { 0x448, 0x448, 0x0000000000000105 },	/* c1 br rsp (via c2) */
    { 0x450, 0x450, 0x0000000000000105 },	/* c1 media 1 rsp (via c2) */
    { 0x458, 0x458, 0x0000000000000105 },	/* c1 media 2 rsp (via c2) */
    { 0x460, 0x480, 0x0000000000000000 },
    { 0x488, 0x488, 0x0000000000000105 },	/* c2 br rsp */
    { 0x490, 0x490, 0x0000000000000105 },	/* c2 media 1 rsp */
    { 0x498, 0x498, 0x0000000000000105 },	/* c2 media 2 rsp */
    { 0x4a0, 0x4c0, 0x0000000000000000 },
    { 0x4c8, 0x4c8, 0x0000000000000101 },	/* c3 br rsp */
    { 0x4d0, 0x4d0, 0x0000000000000102 },	/* c3 media 1 rsp */
    { 0x4d8, 0x4d8, 0x0000000000000103 },	/* c3 media 2 rsp */
    { 0x4e0, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c3_sci01_03_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000000000104 },	/* c0 br req */
    { 0x408, 0x438, 0x0000000000000000 },
    { 0x440, 0x440, 0x0000000000000105 },	/* c1 br req (via c2) */
    { 0x448, 0x478, 0x0000000000000000 },
    { 0x480, 0x480, 0x0000000000000105 },	/* c2 br req */
    { 0x488, 0x4b8, 0x0000000000000000 },
    { 0x4c0, 0x4c0, 0x0000000000000100 },	/* c3 br req */
    { 0x4c8, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c3_sci04_06_init[] = {
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x400, 0x0000000000000104 },	/* c0 br req (relay c2) */
    { 0x408, 0x408, 0x0000000000000104 },	/* c0 br rsp (relay c2) */
    { 0x410, 0x410, 0x0000000000000104 },	/* c0 media 1 rsp (relay c2) */
    { 0x418, 0x418, 0x0000000000000104 },	/* c0 media 2 rsp (relay c2) */
    { 0x420, 0x478, 0x0000000000000000 },
    { 0x480, 0x480, 0x0000000000000105 },	/* c2 br req (relay c0) */
    { 0x488, 0x488, 0x0000000000000105 },	/* c2 br rsp (relay c0) */
    { 0x490, 0x490, 0x0000000000000105 },	/* c2 media 1 rsp (relay c0) */
    { 0x498, 0x498, 0x0000000000000105 },	/* c2 media 2 rsp (relay c0) */
    { 0x4a0, 0x4b8, 0x0000000000000000 },
    { 0x4c0, 0x4c0, 0x0000000000000100 },	/* c3 br req */
    { 0x4c8, 0x4c8, 0x0000000000000101 },	/* c3 br rsp */
    { 0x4d0, 0x4d0, 0x0000000000000102 },	/* c3 media 1 rsp */
    { 0x4d8, 0x4d8, 0x0000000000000103 },	/* c3 media 2 rsp */
    { 0x4e0, 0x7f8, 0x0000000000000000 },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init c3_sci07_init[] = {	/* unused */
    { 0x300, 0x300, 0x0000000000000000 },
    { 0x400, 0x7f8, 0x0000000000000000 },	/* clear entire table */
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_reg_init sco_init[] = {
    { 0x308, 0x308, 0x000000000003ffff },
    { 0xfff, 0xfff, 0 }
};

static const struct gzd_core_block_init blue_block_init[] = {
    { BrReq, br_req_init, BrReqCID },
    { BrResp, resp_init, BrRespCID },
    { MediaResp, resp_init, MediaRespCID },
    { MediaResp2, resp_init, MediaResp2CID },
    { ZLink0, link_init },
    { ZLink1, link_init },
    { 0, 0 }
};

static const struct gzd_core_block_init card0_swcore_block_init[] = {
    { SCI00, c0_sci00_init },
    { SCI01, c0_sci01_03_init },
    { SCI02, c0_sci01_03_init },
    { SCI03, c0_sci01_03_init },
    { SCI04, c0_sci04_06_init },
    { SCI05, c0_sci04_06_init },
    { SCI06, c0_sci04_06_init },
    { SCI07, c0_sci07_init },
    { SCO00, sco_init },
    { SCO01, sco_init },
    { SCO02, sco_init },
    { SCO03, sco_init },
    { SCO04, sco_init },
    { SCO05, sco_init },
    { SCO06, sco_init },
    { SCO07, sco_init },
    { 0, 0 }
};

static const struct gzd_core_block_init card1_swcore_block_init[] = {
    { SCI00, c1_sci00_init },
    { SCI01, c1_sci01_03_init },
    { SCI02, c1_sci01_03_init },
    { SCI03, c1_sci01_03_init },
    { SCI04, c1_sci04_06_init },
    { SCI05, c1_sci04_06_init },
    { SCI06, c1_sci04_06_init },
    { SCI07, c1_sci07_init },
    { SCO00, sco_init },
    { SCO01, sco_init },
    { SCO02, sco_init },
    { SCO03, sco_init },
    { SCO04, sco_init },
    { SCO05, sco_init },
    { SCO06, sco_init },
    { SCO07, sco_init },
    { 0, 0 }
};

static const struct gzd_core_block_init card2_swcore_block_init[] = {
    { SCI00, c2_sci00_init },
    { SCI01, c2_sci01_03_init },
    { SCI02, c2_sci01_03_init },
    { SCI03, c2_sci01_03_init },
    { SCI04, c2_sci04_06_init },
    { SCI05, c2_sci04_06_init },
    { SCI06, c2_sci04_06_init },
    { SCI07, c2_sci07_init },
    { SCO00, sco_init },
    { SCO01, sco_init },
    { SCO02, sco_init },
    { SCO03, sco_init },
    { SCO04, sco_init },
    { SCO05, sco_init },
    { SCO06, sco_init },
    { SCO07, sco_init },
    { 0, 0 }
};

static const struct gzd_core_block_init card3_swcore_block_init[] = {
    { SCI00, c3_sci00_init },
    { SCI01, c3_sci01_03_init },
    { SCI02, c3_sci01_03_init },
    { SCI03, c3_sci01_03_init },
    { SCI04, c3_sci04_06_init },
    { SCI05, c3_sci04_06_init },
    { SCI06, c3_sci04_06_init },
    { SCI07, c3_sci07_init },
    { SCO00, sco_init },
    { SCO01, sco_init },
    { SCO02, sco_init },
    { SCO03, sco_init },
    { SCO04, sco_init },
    { SCO05, sco_init },
    { SCO06, sco_init },
    { SCO07, sco_init },
    { 0, 0 }
};

static const struct gzd_core_block_init *card_swcore_block_init[MAX_CARDS]
    = {
    card0_swcore_block_init,
    card1_swcore_block_init,
    card2_swcore_block_init,
    card3_swcore_block_init
};

#define CREATE_MASK(pos, len)  GENMASK_ULL((pos)+(len)-1, (pos))

static uint64_t
bitfield_insert(uint64_t cur, uint pos, uint len, uint64_t new)
{
    uint64_t mask;

    mask = CREATE_MASK(pos, len);
    return (cur & ~mask) | ((new << pos) & mask);
}

static uint64_t bitfield_extract(uint64_t val, uint pos, uint len)
{
    uint64_t mask;

    mask = CREATE_MASK(pos, len);
    return (val & mask) >> pos;
}

static int
gzd_core_reg_block_init(struct gzd_core_hw *hw,
			const struct gzd_core_block_init *const block)
{
    struct device *dev = &hw->pdev->dev;
    ulong flags;
    const struct gzd_core_reg_init *reg;
    uint64_t offset;
    uint64_t value;
    uint64_t card_id = hw->info.card_id;

    spin_lock_irqsave(&hw->lock, flags);
    for (reg = block->table; reg->start_offset != 0xfff; reg++) {
	value = reg->value;
	if (reg->n.len)
	    value =
		bitfield_insert(value, reg->n.pos, reg->n.len, card_id);
	if (reg->m.len)
	    value =
		bitfield_insert(value, reg->m.pos, reg->m.len, block->m);
	if (reg->z.len)
	    value = bitfield_insert(value, reg->z.pos, reg->z.len,
				    2 * card_id + 1);
	for (offset = block->offset + reg->start_offset;
	     offset <= block->offset + reg->end_offset; offset += 8) {
	    // swsok, disable to reduce messages
	    /*
	     * dev_info(dev, "blue block init: 0x%016llx = 0x%016llx\n",
	     * offset, value);
	     */
	    iowrite64(value, hw->regs_base_addr + offset);
	}
    }

    spin_unlock_irqrestore(&hw->lock, flags);
    return 0;
}

static int
gzd_core_reg_block_compare(struct gzd_core_hw *hw,
			   const struct gzd_core_block_init *const block)
{
    /*
     * Revisit: refactor to avoid cut-n-paste duplication of
     * gzd_core_reg_block_init 
     */
    struct device *dev = &hw->pdev->dev;
    ulong flags;
    const struct gzd_core_reg_init *reg;
    uint64_t offset;
    uint64_t value, actual, mask;
    uint64_t card_id = hw->info.card_id;

    spin_lock_irqsave(&hw->lock, flags);
    for (reg = block->table; reg->start_offset != 0xfff; reg++) {
	value = reg->value;
	if (reg->n.len)
	    value =
		bitfield_insert(value, reg->n.pos, reg->n.len, card_id);
	if (reg->m.len)
	    value =
		bitfield_insert(value, reg->m.pos, reg->m.len, block->m);
	if (reg->z.len)
	    value = bitfield_insert(value, reg->z.pos, reg->z.len,
				    2 * card_id + 1);
	mask = (reg->read_mask == 0) ? -1ull : reg->read_mask;
	for (offset = block->offset + reg->start_offset;
	     offset <= block->offset + reg->end_offset; offset += 8) {
	    actual = ioread64(hw->regs_base_addr + offset);
	    if ((value & mask) != (actual & mask))
		dev_info(dev,
			 "blue block miscompare 0x%016llx: 0x%016llx != 0x%016llx\n",
			 offset, actual & mask, value & mask);
	}
    }

    spin_unlock_irqrestore(&hw->lock, flags);
    return 0;
}

static int gzd_core_blue_block_init(struct gzd_core_hw *hw)
{
    int i;
    int err = 0;
    struct device *dev = &hw->pdev->dev;
    const struct gzd_core_block_init *swcore_block_init;

    for (i = 0; blue_block_init[i].table; i++)
	gzd_core_reg_block_init(hw, &blue_block_init[i]);


    for (i = 0; blue_block_init[i].table; i++)
	gzd_core_reg_block_compare(hw, &blue_block_init[i]);

    swcore_block_init = card_swcore_block_init[hw->info.card_id];
    if (swcore_block_init) {
	for (i = 0; swcore_block_init[i].table; i++) {
	    gzd_core_reg_block_init(hw, &swcore_block_init[i]);
	    gzd_core_reg_block_compare(hw, &swcore_block_init[i]);
	}
    } else {			/* unsupported card id */
	dev_err(dev, "unsupported card id %u\n", hw->info.card_id);
	err = -EINVAL;
    }

    return err;
}

/*
 * ============================================================ SWITCH
 * CORE ROUTING SYSFS FUNCTIONS
 * ============================================================ 
 */
static ssize_t
gzd_core_show_sci_index(struct device *ddev,
			struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    /*
     * print the current sci index 
     */
    return scnprintf(buf, PAGE_SIZE - 2, "%llu\n", hw->sci_index);
}

static ssize_t
gzd_core_store_sci_index(struct device *ddev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint64_t index;
    int ret;

    ret = sscanf(buf, "%llu", &index);
    if (ret != 1)
	return -EINVAL;

    hw->sci_index = index & 0x7;	/* SCI00 - SCI07 */
    return count;
}

static ssize_t
gzd_core_show_sci_cid(struct device *ddev,
		      struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    return scnprintf(buf, PAGE_SIZE - 2, "0x%llx\n", hw->sci_cid);
}

static ssize_t
gzd_core_store_sci_cid(struct device *ddev,
		       struct device_attribute *attr,
		       const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint64_t cid;
    int ret;

    ret = sscanf(buf, "0x%llx", &cid);
    if (ret != 1)
	return -EINVAL;

    hw->sci_cid = cid & 0xfff;	/* 12-bit CIDs */
    return count;
}

static ssize_t
gzd_core_show_sci_route(struct device *ddev,
			struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint64_t offset;
    uint64_t value;
    /*
     * Return value of the scnprintf function 
     */
    ssize_t ret;
    ulong flags;

    spin_lock_irqsave(&hw->lock, flags);
    offset = SCI00 + (hw->sci_index * 0x1000) + 0x400 +
	(((hw->sci_cid >> 4) & 0xff) * 8);
    value = ioread64(hw->regs_base_addr + offset);
    spin_unlock_irqrestore(&hw->lock, flags);
    ret = scnprintf(buf, PAGE_SIZE - 2, "0x%016llx\n", value);
    return ret;
}

static ssize_t
gzd_core_store_sci_route(struct device *ddev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    struct device *dev = &hw->pdev->dev;
    uint64_t value;
    uint64_t offset;
    int ret;
    ulong flags;

    ret = sscanf(buf, "0x%llx", &value);
    if (ret != 1)
	return -EINVAL;

    spin_lock_irqsave(&hw->lock, flags);
    offset = SCI00 + (hw->sci_index * 0x1000) + 0x400 +
	(((hw->sci_cid >> 4) & 0xff) * 8);
    iowrite64(value, hw->regs_base_addr + offset);
    spin_unlock_irqrestore(&hw->lock, flags);

    dev_info(dev,
	     "sci_route write SCI%llu, offset=0x%llx, value=0x%016llx",
	     hw->sci_index, offset, value);

    return count;
}

/*
 * ============================================================ MEMORY MAP 
 * TESTING SYSFS FUNCTIONS
 * ============================================================ 
 */
static ssize_t
gzd_core_show_memory_map_offset(struct device *ddev,
				struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    /*
     * print the offset used for the memory map access 
     */
    return scnprintf(buf, PAGE_SIZE - 2, "0x%016llx\n",
		     hw->memory_map_offset);
}

static ssize_t
gzd_core_store_memory_map_offset(struct device *ddev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint64_t offset;
    int ret;

    ret = sscanf(buf, "0x%llx", &offset);
    if (ret != 1)
	return -EINVAL;

    hw->memory_map_offset = rounddown(offset, hw->memory_map_size);
    return count;
}

static ssize_t
gzd_core_show_memory_map_size(struct device *ddev,
			      struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    return scnprintf(buf, PAGE_SIZE - 2, "%zu\n", hw->memory_map_size);
}

static ssize_t
gzd_core_store_memory_map_size(struct device *ddev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    size_t size;
    int ret;

    ret = sscanf(buf, "%zu", &size);
    if (ret != 1)
	return -EINVAL;

    if ((size != 4) && (size != 8)) {
	/*
	 * User tried to set an unsupported access size, default to 8 
	 */
	size = 8;
    }
    hw->memory_map_size = size;

    /*
     * re-adjust the memory map offset to avoid non-aligned accesses when
     * setting the offset first, then the size second 
     */
    hw->memory_map_offset = rounddown(hw->memory_map_offset, size);
    return count;
}

static ssize_t
gzd_core_show_memory_map_test(struct device *ddev,
			      struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    struct device *dev = &hw->pdev->dev;
    uint64_t data;
    uint64_t *map64;
    uint32_t *map32;
    /*
     * Return value of the scnprintf function 
     */
    ssize_t ret;

    dev_info(dev,
	     "memory_map_read addr=0x%016lx, size=%lu",
	     (unsigned long) (hw->media_base_addr + hw->memory_map_offset),
	     hw->memory_map_size);

    if (hw->memory_map_size == 8) {
	map64 = (uint64_t *) (hw->media_base_addr + hw->memory_map_offset);
	data = *map64;
	ret = scnprintf(buf, PAGE_SIZE - 2, "0x%016llx\n", data);
    } else {
	map32 = (uint32_t *) (hw->media_base_addr + hw->memory_map_offset);
	data = *map32;
	ret = scnprintf(buf, PAGE_SIZE - 2, "0x%08llx\n", data);
    }

    return ret;
}

static ssize_t
gzd_core_store_memory_map_test(struct device *ddev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    struct device *dev = &hw->pdev->dev;
    uint64_t data;
    uint64_t *memory_map8b_addr;
    uint32_t *memory_map4b_addr;
    void __iomem *memory_map_addr;
    int ret;

    ret = sscanf(buf, "0x%llx", &data);
    if (ret != 1)
	return -EINVAL;

    /*
     * Calculate the address we will store the data 
     */
    memory_map_addr = hw->media_base_addr + hw->memory_map_offset;
    dev_info(dev,
	     "memory_map_write addr=0x%016lx, size=%lu, data=0x%016llx",
	     (unsigned long) memory_map_addr, hw->memory_map_size, data);

    /*
     * Take the scanned in data, and store it to the mempory map address
     * space 
     */
    if (hw->memory_map_size == 8) {
	memory_map8b_addr = (uint64_t *) memory_map_addr;
	*memory_map8b_addr = data;
    } else {
	memory_map4b_addr = (uint32_t *) memory_map_addr;
	*memory_map4b_addr = data;
    }

    return count;
}

/*
 * ============================================================ BLOCK I/O
 * TESTING SYSFS FUNCTIONS
 * ============================================================ 
 */
static ssize_t
gzd_core_show_block_io_offset(struct device *ddev,
			      struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    return scnprintf(buf, PAGE_SIZE - 2, "0x%016llx\n",
		     hw->block_io_offset);
}

static ssize_t
gzd_core_store_block_io_offset(struct device *ddev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint64_t offset;
    int ret;

    ret = sscanf(buf, "0x%llx", &offset);
    if (ret != 1)
	return -EINVAL;
    hw->block_io_offset = offset;
    return count;
}

static ssize_t
gzd_core_show_block_io_size(struct device *ddev,
			    struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    return scnprintf(buf, PAGE_SIZE - 2, "%zu\n", hw->block_io_size);
}

static ssize_t
gzd_core_store_block_io_size(struct device *ddev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    size_t size;
    int ret;

    ret = sscanf(buf, "%zu", &size);
    if (ret != 1)
	return -EINVAL;
    hw->block_io_size = size;
    return count;
}

static ssize_t
gzd_core_show_block_io_cardid(struct device *ddev,
			      struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    return scnprintf(buf, PAGE_SIZE - 2, "%u\n", hw->block_io_cardid);
}

static ssize_t
gzd_core_store_block_io_cardid(struct device *ddev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint cardid;
    int ret;

    ret = sscanf(buf, "%u", &cardid);
    if (ret != 1)
	return -EINVAL;
    hw->block_io_cardid = cardid;
    return count;
}

/*
 * forward declaration 
 */
static int __gzd_core_block_io_request(struct gzd_core_hw *hw,
				       dma_addr_t host_addr,
				       uint64_t genz_addr, uint len,
				       uint req_id, uint dcid, uint rw);

static uint gzd_core_alloc_test_req_id(struct gzd_core_hw *hw)
{
    ulong flags;
    uint req_id;

    spin_lock_irqsave(&hw->lock, flags);
    req_id = hw->test_req_id;
    hw->test_req_id = (req_id < hw->test_max_req_id) ?
	req_id + 1u : hw->test_min_req_id;
    spin_unlock_irqrestore(&hw->lock, flags);
    return req_id;
}

static ssize_t
gzd_core_show_block_io_test(struct device *ddev,
			    struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    struct device *dev = &hw->pdev->dev;
    uint64_t offset = hw->block_io_offset;
    size_t count = hw->block_io_size;
    uint dcid = (hw->block_io_cardid << 7) | MediaRespCID;
    uint req_id = gzd_core_alloc_test_req_id(hw);
    uint req_index = req_id - hw->test_min_req_id;
    void *cpu_addr;
    dma_addr_t dma_addr;
    int ret;

    dev_info(dev,
	     "block_io_read offset=0x%016llx, dcid=0x%x, req_id=%u size=%zu",
	     offset, dcid, req_id, count);
    if (count == 0)
	return -EINVAL;
    /*
     * unknown if sysfs guarantees buf is DMAable, so alloc & copy 
     */
    cpu_addr =
	dma_alloc_coherent(&pdev->dev, count, &dma_addr, GFP_KERNEL);
    if (!cpu_addr)
	return -ENOMEM;
    clear_bit(req_index, hw->block_io_done);
    ret = __gzd_core_block_io_request(hw, dma_addr, offset, count, req_id,
				      dcid, 1);
    if (ret < 0) {
	dma_free_coherent(&pdev->dev, count, cpu_addr, dma_addr);
	return ret;
    }
    wait_event_interruptible(hw->block_io_queue,
			     test_bit(req_index, hw->block_io_done));
    memcpy(buf, cpu_addr, count);
    dma_free_coherent(&pdev->dev, count, cpu_addr, dma_addr);
    return count;
}

static ssize_t
gzd_core_store_block_io_test(struct device *ddev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    struct device *dev = &hw->pdev->dev;
    uint64_t offset = hw->block_io_offset;
    uint dcid = (hw->block_io_cardid << 7) | MediaRespCID;
    uint req_id = gzd_core_alloc_test_req_id(hw);
    uint req_index = req_id - hw->test_min_req_id;
    void *cpu_addr;
    dma_addr_t dma_addr;
    int ret;

    dev_info(dev,
	     "block_io_write offset=0x%016llx, dcid=0x%x, req_id=%u size=%zu",
	     offset, dcid, req_id, count);
    /*
     * unknown if sysfs guarantees buf is DMAable, so alloc & copy 
     */
    cpu_addr =
	dma_alloc_coherent(&pdev->dev, count, &dma_addr, GFP_KERNEL);
    if (!cpu_addr)
	return -ENOMEM;
    clear_bit(req_index, hw->block_io_done);
    memcpy(cpu_addr, buf, count);
    ret = __gzd_core_block_io_request(hw, dma_addr, offset, count, req_id,
				      dcid, 0);
    if (ret < 0) {
	dma_free_coherent(&pdev->dev, count, cpu_addr, dma_addr);
	return ret;
    }
    wait_event_interruptible(hw->block_io_queue,
			     test_bit(req_index, hw->block_io_done));
    dma_free_coherent(&pdev->dev, count, cpu_addr, dma_addr);
    return count;
}

/*
 * ============================================================ MESSAGING
 * TESTING SYSFS FUNCTIONS
 * ============================================================ 
 */

/*
 * forward declarations 
 */
static int __gzd_core_msg_request(struct gzd_core_hw *hw,
				  dma_addr_t host_addr, uint len,
				  uint req_id, uint dcid);
size_t __gzd_core_msg_update_cons_ptr(struct gzd_core_hw *hw, struct gzd_msg_tbl_entry
				      *tbl, uint index, size_t size);
uint64_t __gzd_core_msg_read_prod_ptr(struct gzd_core_hw *hw,
				      struct gzd_msg_tbl_entry *tbl,
				      uint index);

static ssize_t
gzd_core_show_msg_offset(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    return scnprintf(buf, PAGE_SIZE - 2, "0x%016llx\n", hw->msg_offset);
}

static ssize_t
gzd_core_store_msg_offset(struct device *ddev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint64_t offset;
    int ret;

    ret = sscanf(buf, "0x%llx", &offset);
    if (ret != 1)
	return -EINVAL;
    hw->msg_offset = offset;
    return count;
}

static ssize_t
gzd_core_show_msg_cardid(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    return scnprintf(buf, PAGE_SIZE - 2, "%u\n", hw->msg_cardid);
}

static ssize_t
gzd_core_store_msg_cardid(struct device *ddev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint cardid;
    int ret;

    ret = sscanf(buf, "%u", &cardid);
    if (ret != 1)
	return -EINVAL;
    hw->msg_cardid = cardid;
    return count;
}

static ssize_t
gzd_core_show_msg_cons_ptr(struct device *ddev,
			   struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    (void) __gzd_core_msg_read_prod_ptr(hw, &hw->msg_tbl, 0);
    return scnprintf(buf, PAGE_SIZE - 2, "0x%016llx\n",
		     hw->msg_tbl.cons_ptr);
}

static ssize_t
gzd_core_show_msg_prod_ptr(struct device *ddev,
			   struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    (void) __gzd_core_msg_read_prod_ptr(hw, &hw->msg_tbl, 0);
    return scnprintf(buf, PAGE_SIZE - 2, "0x%016llx\n",
		     hw->msg_tbl.prod_ptr);
}

static long gzd_core_msg_request_ticket(struct gzd_core_hw *hw)
{
    return atomic_long_inc_return(&hw->msg_recv_waiting);
}

static int gzd_core_msg_check_ticket(struct gzd_core_hw *hw, long ticket)
{
    /*
     * Revisit: this does not handle wraparound 
     */
    return (atomic_long_read(&hw->msg_recv_serving) >= ticket);
}

struct gzd_core_msg_recv {
    struct gzd_core_hw *hw;
    long ticket;
    void *msg;
    struct list_head list;
};

static ssize_t
gzd_core_show_msg_test(struct device *ddev,
		       struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    struct device *dev = &hw->pdev->dev;
    struct gzd_msg_header *msg_hdr;
    struct gzd_core_msg_recv *msg_recv, *next;
    uint scid;
    void *msg_data;
    int ret;
    size_t count = 0;
    long ticket = gzd_core_msg_request_ticket(hw);

    dev_info(dev, "msg_read, ticket=%ld, waiting...\n", ticket);
    /*
     * wait until it is our turn to handle a msg 
     */
    ret = wait_event_interruptible(hw->msg_recv_queue,
				   gzd_core_msg_check_ticket(hw, ticket));
    if (ret != 0) {
	dev_info(dev, "msg_read, ticket=%ld, interrupted\n", ticket);
	/*
	 * Revisit: return ticket? 
	 */
	return -EINTR;
    }
    dev_info(dev, "msg_read, ticket=%ld, continuing...\n", ticket);
    /*
     * lock ensures atomic list manipulation 
     */
    mutex_lock(&hw->msg_recv_lock);
    list_for_each_entry_safe(msg_recv, next, &hw->msg_recv_list, list) {
	if (msg_recv->ticket == ticket) {	/* found */
	    list_del_init(&msg_recv->list);
	    break;
	}
    }
    mutex_unlock(&hw->msg_recv_lock);
    if (!msg_recv) {
	dev_err(dev, "ticket %lu not found in msg_recv_list\n", ticket);
	return -EIO;
    }
    if (msg_recv->hw != hw) {
	dev_err(dev, "ticket %lu has incorrect hw ptr (%p != %p)\n",
		ticket, msg_recv->hw, hw);
	kfree(msg_recv);
	return -EIO;
    }
    msg_hdr = msg_recv->msg;
    if (!msg_hdr) {
	dev_err(dev, "ticket %lu has NULL msg_hdr\n", ticket);
	kfree(msg_recv);
	return -EIO;
    }
    msg_data = (void *) (msg_hdr + 1);
    scid = msg_hdr->scid;
    count = msg_hdr->len - sizeof(*msg_hdr);	/* actual msg data len */
    /*
     * Revisit: what if count > PAGE_SIZE? 
     */
    memcpy(buf, msg_data, count);
    dev_info(dev, "  msg_hdr=%p, msg_data=%p, scid=0x%x, count=%zu\n",
	     msg_hdr, msg_data, scid, count);
    kfree(msg_recv->msg);
    kfree(msg_recv);
    return count;
}

static ssize_t
gzd_core_store_msg_test(struct device *ddev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    struct device *dev = &hw->pdev->dev;
    uint dcid = (hw->msg_cardid << 7) | BrRespCID;
    uint req_id = gzd_core_alloc_test_req_id(hw);
    uint req_index = req_id - hw->test_min_req_id;
    uint64_t offset = hw->msg_offset;
    void *cpu_addr;
    dma_addr_t dma_addr;
    int ret;

    dev_info(dev,
	     "msg_write dcid=0x%x, req_id=%u size=%zu offset=0x%llx",
	     dcid, req_id, count, offset);
    /*
     * unknown if sysfs guarantees buf is DMAable, so alloc & copy 
     */
    cpu_addr =
	dma_alloc_coherent(&pdev->dev, count + offset, &dma_addr,
			   GFP_KERNEL);
    if (!cpu_addr)
	return -ENOMEM;
    clear_bit(req_index, hw->msg_done);
    memcpy(cpu_addr + offset, buf, count);
    ret =
	__gzd_core_msg_request(hw, dma_addr + offset, count, req_id, dcid);
    if (ret < 0) {
	dma_free_coherent(&pdev->dev, count, cpu_addr, dma_addr);
	dev_err(dev, "msg_request error %d for req_id=%u\n", ret, req_id);
	return ret;
    }
    wait_event_interruptible(hw->msg_send_queue,
			     test_bit(req_index, hw->msg_done));
    dma_free_coherent(&pdev->dev, count, cpu_addr, dma_addr);
    return count;
}

/*
 * ============================================================
 * BLOCK/MESSAGING COUNTER SYSFS FUNCTIONS
 * ============================================================ 
 */
#define gzd_core_show_counter(_name)					\
	static ssize_t gzd_core_show_##_name(struct device *ddev,	\
					struct device_attribute *attr,	\
					char *buf)			\
	{								\
	struct pci_dev *pdev = to_pci_dev(ddev);			\
	struct gzd_core_hw *hw = pci_get_drvdata(pdev);			\
									\
	return scnprintf(buf, PAGE_SIZE-2, "%llu\n",			\
			hw->_name);					\
	}

gzd_core_show_counter(block_io_request_count)
    gzd_core_show_counter(block_io_fifo_full_count)
    gzd_core_show_counter(block_io_fifo_overflow_count)
    gzd_core_show_counter(block_io_response_count)
    gzd_core_show_counter(block_io_nak_count)
    gzd_core_show_counter(msg_request_count)
    gzd_core_show_counter(msg_fifo_full_count)
    gzd_core_show_counter(msg_fifo_overflow_count)
    gzd_core_show_counter(msg_response_count)
    gzd_core_show_counter(msg_nak_count)
#define gzd_core_store_counter(_name)					\
	static ssize_t gzd_core_store_##_name(struct device *ddev,	\
					struct device_attribute *attr,	\
					const char *buf, size_t count)	\
	{								\
	struct pci_dev *pdev = to_pci_dev(ddev);			\
	struct gzd_core_hw *hw = pci_get_drvdata(pdev);			\
	uint64_t counter;						\
	int ret;							\
									\
	ret = sscanf(buf, "%llu", &counter);				\
	if (ret != 1)							\
		return -EINVAL;						\
	hw->_name = counter;						\
	return count;							\
}
    gzd_core_store_counter(block_io_request_count)
    gzd_core_store_counter(block_io_fifo_full_count)
    gzd_core_store_counter(block_io_fifo_overflow_count)
    gzd_core_store_counter(block_io_response_count)
    gzd_core_store_counter(block_io_nak_count)
    gzd_core_store_counter(msg_request_count)
    gzd_core_store_counter(msg_fifo_full_count)
    gzd_core_store_counter(msg_fifo_overflow_count)
    gzd_core_store_counter(msg_response_count)
    gzd_core_store_counter(msg_nak_count)
    /*
     * ============================================================
     * BLOCK/MESSAGING/ADDR TRANS STATUS SYSFS FUNCTIONS
     * ============================================================ 
     */
static ssize_t gzd_core_show_status(struct device *ddev,
				    struct device_attribute *attr,
				    char *buf, const uint reg)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint64_t status;

    status = ioread64(hw->regs_base_addr + reg);
    return scnprintf(buf, PAGE_SIZE - 2, "0x%016llx\n", status);
}

static ssize_t
gzd_core_show_status32(struct device *ddev,
		       struct device_attribute *attr,
		       char *buf, const uint reg)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint32_t status;

    status = ioread32(hw->regs_base_addr + reg);
    return scnprintf(buf, PAGE_SIZE - 2, "0x%08x\n", status);
}

static ssize_t
gzd_core_show_block_status(struct device *ddev,
			   struct device_attribute *attr, char *buf)
{
    return gzd_core_show_status(ddev, attr, buf, BlockControl);
}

static ssize_t
gzd_core_show_msg_status(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
    return gzd_core_show_status(ddev, attr, buf, MsgControl);
}

static ssize_t
gzd_core_show_addr_trans_status(struct device *ddev,
				struct device_attribute *attr, char *buf)
{
    return gzd_core_show_status(ddev, attr, buf, AtStatus);
}

static ssize_t
gzd_core_show_addr_trans_error(struct device *ddev,
			       struct device_attribute *attr, char *buf)
{
    return gzd_core_show_status(ddev, attr, buf, AtError);
}

static ssize_t
gzd_core_show_seq_status(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
    return gzd_core_show_status(ddev, attr, buf, SeqStatus);
}

static ssize_t
gzd_core_store_status(struct device *ddev,
		      struct device_attribute *attr,
		      const char *buf, size_t count, const uint reg)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint64_t status;
    int ret;

    ret = sscanf(buf, "0x%llx", &status);
    if (ret != 1)
	return -EINVAL;
    iowrite64(status, hw->regs_base_addr + reg);
    return count;
}

static ssize_t
gzd_core_store_status32(struct device *ddev,
		      struct device_attribute *attr,
		      const char *buf, size_t count, const uint reg)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint32_t status;
    int ret;

    ret = sscanf(buf, "0x%x", &status);
    if (ret != 1)
	return -EINVAL;
    iowrite32(status, hw->regs_base_addr + reg);
    return count;
}


static ssize_t
gzd_core_store_block_status(struct device *ddev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
    return gzd_core_store_status(ddev, attr, buf, count, BlockControl);
}

static ssize_t
gzd_core_store_msg_status(struct device *ddev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
    return gzd_core_store_status(ddev, attr, buf, count, MsgControl);
}

static ssize_t
gzd_core_store_addr_trans_status(struct device *ddev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
    return gzd_core_store_status(ddev, attr, buf, count, AtStatus);
}

static ssize_t
gzd_core_store_addr_trans_error(struct device *ddev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
    return gzd_core_store_status(ddev, attr, buf, count, AtError);
}

static ssize_t
gzd_core_store_seq_status(struct device *ddev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
    return gzd_core_store_status(ddev, attr, buf, count, SeqStatus);
}

/*
 * ============================================================ CORE SYSF
 * FUNCTIONS ============================================================ 
 */
static ssize_t
gzd_core_show_card_id(struct device *ddev,
		      struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);

    return scnprintf(buf, PAGE_SIZE - 2, "%u\n", hw->info.card_id);
}

/*
 * ============================================================ ADDRESS
 * TRANSLATION INITIALIZATION
 * ============================================================ 
 */
static void
gzd_core_read_addr_trans_entry(struct gzd_core_hw *hw, uint index,
			       uint64_t * start, uint64_t * end,
			       uint64_t * dcid)
{
    uint64_t trig;

    /*
     * caller must already hold hw->lock 
     */
    trig = bitfield_insert(0, 0, 8, index);
    trig = bitfield_insert(trig, 17, 1, 1);
    iowrite64(trig, hw->regs_base_addr + AtTeControl);

    *start = ioread64(hw->regs_base_addr + AtTeRdStart);
    *end = ioread64(hw->regs_base_addr + AtTeRdEnd);
    *dcid = ioread64(hw->regs_base_addr + AtTeRdDCID);
}

static ssize_t
gzd_core_show_addr_trans(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint i;
    ulong flags;
    ssize_t cnt = 0;
    uint64_t entries, start, end, dcid;

    spin_lock_irqsave(&hw->lock, flags);
    entries = ioread64(hw->regs_base_addr + AtTblControl);

    for (i = 0; i < entries; i++) {
	gzd_core_read_addr_trans_entry(hw, i, &start, &end, &dcid);
	cnt += scnprintf(buf + cnt, PAGE_SIZE - 2 - cnt,
			 "addr_trans[%u] 0x%016llx-0x%016llx, dcid=0x%llx\n",
			 i, start, end, dcid);
    }
    spin_unlock_irqrestore(&hw->lock, flags);
    return cnt;
}

static ssize_t
gzd_core_show_bar0_1k(struct device *ddev,
		      struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(ddev);
    struct gzd_core_hw *hw = pci_get_drvdata(pdev);
    uint i, j;
    ulong flags;
    ssize_t cnt = 0;
    uint64_t entries;

    spin_lock_irqsave(&hw->lock, flags);
    for (i = 0; i < 64; i++) {
	cnt +=
	    scnprintf(buf + cnt, PAGE_SIZE - 2 - cnt, "0x%x:\t",
		      i * 4 * 8);
	for (j = 0; j < 4; j++) {
	    entries = ioread64(hw->regs_base_addr + i * 4 * 8 + j * 8);
	    cnt +=
		scnprintf(buf + cnt, PAGE_SIZE - 2 - cnt, "0x%016llx ",
			  entries);
	}
	cnt += scnprintf(buf + cnt, PAGE_SIZE - 2 - cnt, "\n");
    }
    spin_unlock_irqrestore(&hw->lock, flags);
    return cnt;
}

static ssize_t
gzd_core_show_ecc_status(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
    return gzd_core_show_status32(ddev, attr, buf, EccStatus);
}

static ssize_t
gzd_core_store_ecc_status(struct device *ddev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
    return gzd_core_store_status32(ddev, attr, buf, count, EccStatus);
}

static ssize_t
gzd_core_show_ecc_en_irq(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
    return gzd_core_show_status32(ddev, attr, buf, EccEnIrq);
}

static ssize_t
gzd_core_store_ecc_en_irq(struct device *ddev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
    return gzd_core_store_status32(ddev, attr, buf, count, EccEnIrq);
}

static ssize_t
gzd_core_show_ecc_on_off(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
    return gzd_core_show_status32(ddev, attr, buf, EccOnOff);
}

static ssize_t
gzd_core_store_ecc_on_off(struct device *ddev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
    return gzd_core_store_status32(ddev, attr, buf, count, EccOnOff);
}

static ssize_t
gzd_core_show_ecc_ce_cnt(struct device *ddev,
			 struct device_attribute *attr, char *buf)
{
    return gzd_core_show_status32(ddev, attr, buf, EccCeCnt);
}

static ssize_t
gzd_core_store_ecc_ce_cnt(struct device *ddev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
    return gzd_core_store_status32(ddev, attr, buf, count, EccCeCnt);
}


#define GZD_RO_DEVICE_ATTR(_name) \
	DEVICE_ATTR(_name, S_IRUGO, gzd_core_show_##_name, NULL)
#define GZD_RW_DEVICE_ATTR(_name) \
	DEVICE_ATTR(_name, S_IRUGO|S_IWUSR, gzd_core_show_##_name, \
		gzd_core_store_##_name)

static GZD_RO_DEVICE_ATTR(card_id);
static GZD_RO_DEVICE_ATTR(addr_trans);
static GZD_RW_DEVICE_ATTR(sci_index);
static GZD_RW_DEVICE_ATTR(sci_cid);
static GZD_RW_DEVICE_ATTR(sci_route);
static GZD_RW_DEVICE_ATTR(memory_map_offset);
static GZD_RW_DEVICE_ATTR(memory_map_size);
static GZD_RW_DEVICE_ATTR(memory_map_test);
static GZD_RW_DEVICE_ATTR(block_io_offset);
static GZD_RW_DEVICE_ATTR(block_io_size);
static GZD_RW_DEVICE_ATTR(block_io_cardid);
static GZD_RW_DEVICE_ATTR(block_io_test);
static GZD_RW_DEVICE_ATTR(msg_offset);
static GZD_RW_DEVICE_ATTR(msg_cardid);
static GZD_RW_DEVICE_ATTR(msg_test);
static GZD_RW_DEVICE_ATTR(block_io_request_count);
static GZD_RW_DEVICE_ATTR(block_io_fifo_full_count);
static GZD_RW_DEVICE_ATTR(block_io_fifo_overflow_count);
static GZD_RW_DEVICE_ATTR(block_io_response_count);
static GZD_RW_DEVICE_ATTR(block_io_nak_count);
static GZD_RW_DEVICE_ATTR(msg_request_count);
static GZD_RW_DEVICE_ATTR(msg_fifo_full_count);
static GZD_RW_DEVICE_ATTR(msg_fifo_overflow_count);
static GZD_RW_DEVICE_ATTR(msg_response_count);
static GZD_RW_DEVICE_ATTR(msg_nak_count);
static GZD_RO_DEVICE_ATTR(msg_cons_ptr);
static GZD_RO_DEVICE_ATTR(msg_prod_ptr);
static GZD_RW_DEVICE_ATTR(block_status);
static GZD_RW_DEVICE_ATTR(msg_status);
static GZD_RW_DEVICE_ATTR(addr_trans_status);
static GZD_RW_DEVICE_ATTR(addr_trans_error);
static GZD_RW_DEVICE_ATTR(seq_status);
// swsok, for debug
static GZD_RO_DEVICE_ATTR(bar0_1k);
// swsok, for ECC Controller
static GZD_RW_DEVICE_ATTR(ecc_status);
static GZD_RW_DEVICE_ATTR(ecc_en_irq);
static GZD_RW_DEVICE_ATTR(ecc_on_off);
static GZD_RW_DEVICE_ATTR(ecc_ce_cnt);

static struct attribute *gzd_core_device_attrs[] = {
    &dev_attr_card_id.attr,
    &dev_attr_addr_trans.attr,
    &dev_attr_sci_index.attr,
    &dev_attr_sci_cid.attr,
    &dev_attr_sci_route.attr,
    &dev_attr_memory_map_offset.attr,
    &dev_attr_memory_map_size.attr,
    &dev_attr_memory_map_test.attr,
    &dev_attr_block_io_offset.attr,
    &dev_attr_block_io_size.attr,
    &dev_attr_block_io_cardid.attr,
    &dev_attr_block_io_test.attr,
    &dev_attr_msg_offset.attr,
    &dev_attr_msg_cardid.attr,
    &dev_attr_msg_cons_ptr.attr,
    &dev_attr_msg_prod_ptr.attr,
    &dev_attr_msg_test.attr,
    &dev_attr_block_io_request_count.attr,
    &dev_attr_block_io_fifo_full_count.attr,
    &dev_attr_block_io_fifo_overflow_count.attr,
    &dev_attr_block_io_response_count.attr,
    &dev_attr_block_io_nak_count.attr,
    &dev_attr_msg_request_count.attr,
    &dev_attr_msg_fifo_full_count.attr,
    &dev_attr_msg_fifo_overflow_count.attr,
    &dev_attr_msg_response_count.attr,
    &dev_attr_msg_nak_count.attr,
    &dev_attr_block_status.attr,
    &dev_attr_msg_status.attr,
    &dev_attr_addr_trans_status.attr,
    &dev_attr_addr_trans_error.attr,
    &dev_attr_seq_status.attr,
    &dev_attr_bar0_1k.attr,
    &dev_attr_ecc_status.attr,
    &dev_attr_ecc_en_irq.attr,
    &dev_attr_ecc_on_off.attr,
    &dev_attr_ecc_ce_cnt.attr,
    NULL
};

static struct attribute_group gzd_core_device_attr_group = {
    .name = "gzd-core",
    .attrs = gzd_core_device_attrs,
};

static ulong
gzd_core_write_addr_trans_entry(struct gzd_core_hw *hw, uint index)
{
    struct device *dev = &hw->pdev->dev;
    uint64_t start, end, dcid, trig;
    ulong size;

    /*
     * caller must already hold hw->lock 
     */
    size = hw->info.media_info[index].size;
    if (size > 0) {
	start =
	    hw->info.media_info[index].mmio_addr - hw->media_mmio_start;
	end = start + size - 1;
	dcid = hw->info.media_info[index].cid;
	dev_info(dev, "addr_trans[%u]: 0x%016llx-0x%016llx, "
		 "dcid=0x%llx\n", index, start, end, dcid);
	trig = bitfield_insert(0, 0, 8, index);
	trig = bitfield_insert(trig, 16, 1, 1);
	iowrite64(start, hw->regs_base_addr + AtTeWrStart);
	iowrite64(end, hw->regs_base_addr + AtTeWrEnd);
	iowrite64(dcid, hw->regs_base_addr + AtTeWrDCID);
	iowrite64(trig, hw->regs_base_addr + AtTeControl);
    }

    return size;
}

static int gzd_core_addr_trans_init(struct gzd_core_hw *hw)
{
    struct device *dev = &hw->pdev->dev;
    ulong flags;
    uint i, entries, num_media;
    uint64_t params;

    num_media = hw->info.num_media;
    spin_lock_irqsave(&hw->lock, flags);

    /*
     * Revisit: check status/error regs (0x48,0x50) and clear? 
     */
    /*
     * Table Address Width is AtParams{31:24} 
     */
    params = ioread64(hw->regs_base_addr + AtParams);
    dev_info(dev, "AtParams = 0x%016llx\n", params);
    entries = min(1u << bitfield_extract(params, 24, 8), num_media);
    if (entries < num_media) {
	dev_warn(dev, "insufficient Address Translation Table "
		 "entries for %u media\n", num_media);
	hw->info.num_media = entries;
    }

    for (i = 0; i < entries; i++)
	gzd_core_write_addr_trans_entry(hw, i);

    iowrite64((uint64_t) entries, hw->regs_base_addr + AtTblControl);
    spin_unlock_irqrestore(&hw->lock, flags);
    return 0;
}

#define GZD_RESET_WAIT_NS 500ul	/* 0.5us */

static void gzd_core_reset_and_disable(struct gzd_core_hw *hw)
{
    ulong flags;
    uint64_t val = 0;

    /*
     * Revisit: DDR Reset? 
     */
    val = bitfield_insert(val, 2, 1, 1);	/* Bridge Req/Resp Reset */
    val = bitfield_insert(val, 3, 1, 1);	/* Bridge Msg Reset */
    val = bitfield_insert(val, 6, 1, 1);	/* Gen-Z Hard Reset */
    val = bitfield_insert(val, 4, 1, 0);	/* Bridge Req/Resp Disable 
						 */
    val = bitfield_insert(val, 5, 1, 0);	/* Bridge Msg Disable */
    spin_lock_irqsave(&hw->lock, flags);
    iowrite64(val, hw->regs_base_addr + SeqResetEnable);
    iowrite64(0ull, hw->regs_base_addr + SeqTblControl);
    iowrite64(0ull, hw->regs_base_addr + AtTblControl);
    ndelay(GZD_RESET_WAIT_NS);	/* Wait 0.5us after blue blocks reset */
    spin_unlock_irqrestore(&hw->lock, flags);
}

static void gzd_core_enable(struct gzd_core_hw *hw)
{
    ulong flags;
    uint64_t val;

    spin_lock_irqsave(&hw->lock, flags);
    val = ioread64(hw->regs_base_addr + SeqResetEnable);
    val = bitfield_insert(val, 4, 1, 1);	/* Bridge Req/Resp Enable */
    val = bitfield_insert(val, 5, 1, 1);	/* Bridge Msg Enable */
    iowrite64(val, hw->regs_base_addr + SeqResetEnable);
    spin_unlock_irqrestore(&hw->lock, flags);
}

/*
 * allocate new core_hw structure and link it to the pdev 
 */
static struct gzd_core_hw *gzd_core_hw_alloc(struct pci_dev *pdev,
					     struct gzd_core_pci_info
					     *pci_info)
{
    struct gzd_core_hw *hw;

    hw = kzalloc(sizeof(*hw), GFP_KERNEL);
    if (!hw)
	return 0;

    hw->pdev = pdev;
    hw->pci_info = pci_info;
    pci_set_drvdata(pdev, hw);
    spin_lock_init(&hw->lock);
    init_waitqueue_head(&hw->block_io_queue);
    init_waitqueue_head(&hw->msg_send_queue);
    init_waitqueue_head(&hw->msg_recv_queue);

    mutex_lock(&gzd_core_lock);
    hw->card_index = gzd_core_card_index++;
    mutex_unlock(&gzd_core_lock);

    return hw;
}

/*
 * initialize the hw->info fields 
 */
static void gzd_core_hw_info_init(struct gzd_core_hw *hw)
{
    struct device *dev = &hw->pdev->dev;
    int i, j;
    ulong cid, scid, dcid, size;
    ulong total_size = 0;
    uint64_t params;

    hw->info.genz_subnets = max(genz_subnets, 1u);
    hw->info.num_cards = min(max((uint) card_ids_argc, 1u), MAX_CARDS) /
	hw->info.genz_subnets;
    for (i = 0, j = hw->card_index; i < hw->info.num_cards;
	 i++, j += hw->info.genz_subnets) {
	hw->info.card_ids[i] = card_ids[j];
	dcid = bitfield_insert(0, 0, 7, BrRespCID);
	dcid = bitfield_insert(dcid, 7, 4, hw->info.card_ids[i]);
	hw->info.msg_info[i].dcid = dcid;
	/*
	 * Revisit: for red/green-only bitstreams, scid must match
	 * requester's dcid (which is BrRespCID), not the "real" scid
	 * (which is BrReqCID) 
	 */
	if (red_green) {
	    scid = dcid;
	} else {
	    scid = bitfield_insert(0, 0, 7, BrReqCID);
	    scid = bitfield_insert(scid, 7, 4, hw->info.card_ids[i]);
	}
	hw->info.msg_info[i].scid = scid;
    }
    hw->info.card_id = card_ids[hw->card_index];
    hw->info.card_index = 0;
    hw->info.num_media =
	min(max
	    ((uint) media_size_argc / hw->info.genz_subnets,
	     hw->info.num_cards), MAX_MEDIA / hw->info.genz_subnets);
    for (i = 0, j = hw->card_index; i < hw->info.num_media;
	 i++, j = (j + hw->info.genz_subnets) % hw->info.num_media) {
	cid = bitfield_insert(0, 0, 7, MediaRespCID);
	cid = bitfield_insert(cid, 7, 4, hw->info.card_ids[i]);
	hw->info.media_info[i].media_num = hw->info.card_ids[i];
	hw->info.media_info[i].cid = cid;
	size = MB(media_size[j]);
	hw->info.media_info[i].size = size;
	hw->info.media_info[i].mmio_addr =
	    hw->media_mmio_start + total_size;
	hw->info.media_info[i].base_addr =
	    hw->media_base_addr + total_size;
	total_size += size;
    }
    params = ioread64(hw->regs_base_addr + HwParams);
    dev_info(dev, "HwParams = 0x%016llx\n", params);

//    hw->info.req_fifo_depth = (1u << bitfield_extract(params, 0, 8)) - 1u;
//    hw->info.resp_fifo_depth = (1u << bitfield_extract(params, 8, 8)) - 1u;
    hw->info.req_fifo_depth = 3u;
    hw->info.resp_fifo_depth = 7u;
    hw->info.msg_tbl = &hw->msg_tbl;
    hw->info.pdev = hw->pdev;
    hw->info.zlink_mask = (hw->card_index < zlink_masks_argc) ?
	zlink_masks[hw->card_index] : hw->pci_info->zlink_mask;
    hw->info.zlink_mask &= ((1u << MAX_ZLINKS) - 1u);
    hw->info.zlink_mask |=
	(hw->info.zlink_mask & ((1u << MAX_MZLINKS) - 1u)) << MAX_ZLINKS;
}

static void gzd_core_add_hw_probe_drivers(struct gzd_core_hw *const hw)
{
    struct gzd_driver *ptr, *next;

    /*
     * must add to hw list before calling probe functions 
     */
    list_add_tail(&hw->list, &gzd_core_hw_list);

    /*
     * call probe function for each registered driver 
     */
    list_for_each_entry_safe(ptr, next, &gzd_core_driver_list, list) {
	ptr->probe(hw);
    }
}

static void gzd_core_remove_drivers(struct gzd_core_hw *const hw)
{
    struct gzd_driver *ptr, *next;

    /*
     * call remove function for each registered driver 
     */
    list_for_each_entry_safe(ptr, next, &gzd_core_driver_list, list) {
	ptr->remove(hw);
    }
}

#define MSG_BUF_OFFSET 16	/* HW requires multiple of 16, >= 16 */

static void gzd_core_remove_hw(struct gzd_core_hw *const hw)
{
    list_del_init(&hw->list);
    dma_free_coherent(&hw->pdev->dev,
		      hw->msg_tbl.buf_size + MSG_BUF_OFFSET,
		      hw->msg_tbl.prod_ptr_fixed_va,
		      hw->msg_tbl.prod_ptr_fixed_loc);
    /*
     * Revisit: need to free any items on msg_recv_list 
     */
    kfree(hw);
}

static char *gzd_core_sprintf_xilinx_timestamp(uint64_t timestamp,
					       char *buf, size_t buflen)
{
    uint64_t year, month, day, hour, minute, second;
    char *month_name[] = {
	"Jan", "Feb", "Mar", "Apr", "May", "Jun",
	"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

    day = bitfield_extract(timestamp, 27, 5);
    month = bitfield_extract(timestamp, 23, 4);
    year = bitfield_extract(timestamp, 17, 6) + 2000;
    hour = bitfield_extract(timestamp, 12, 5);
    minute = bitfield_extract(timestamp, 6, 6);
    second = bitfield_extract(timestamp, 0, 6);

    scnprintf(buf, buflen, "%llu %s %llu %llu:%02llu:%02llu",
	      day, month_name[month - 1], year, hour, minute, second);
    return buf;
}

static uint
gzd_core_interface_enable(struct gzd_core_hw *const hw,
			  const uint64_t offset, int disable_first)
{
    uint64_t status;
    uint cur_state;

    /*
     * check I-STATUS Interface State - if I-CFG, then Enable Interface 
     */
    status = ioread64(hw->regs_base_addr + offset);
    cur_state = (uint) (status & INTERFACE_STATE_MASK);
    if (cur_state == I_CFG || (!disable_first && cur_state == I_UP)) {
	if (disable_first) {
	    status &= ~INTERFACE_ENABLE;
	    iowrite64(status, hw->regs_base_addr + offset);
	}
	status |= INTERFACE_ENABLE;
	iowrite64(status, hw->regs_base_addr + offset);
    }

    return cur_state;
}

static const uint64_t au_offset[] = { AuStatus0, AuStatus1,
    AuStatus2, AuStatus3
};

static const uint64_t zl_offset[] = { LinkStatusCtl0, LinkStatusCtl1,
    LinkStatusCtl2, LinkStatusCtl3,
    LinkStatusCtl4, LinkStatusCtl5,
    MLinkStatusCtl0, MLinkStatusCtl1
};

static void gzd_core_link_status(struct gzd_core_hw *const hw)
{
    struct device *dev = &hw->pdev->dev;
    struct gzd_core_info *info = &hw->info;
    uint64_t status[MAX_AURORA] = { 0 };
    uint cur_state;
    int i, j, done, num_zlinks = 0;
    int zdone[MAX_ZLINKS + MAX_MZLINKS] = { 0 };
    cycles_t start, duration = 0;

    for (j = 0; j < (MAX_ZLINKS + MAX_MZLINKS); j++)
	if (ZLINK_MASK(hw->info.zlink_mask, j))
	    num_zlinks++;

    /*
     * wait for at least one Aurora link to come up 
     */
    for (i = 0; i < 100; i++) {
	for (j = 0; j < MAX_AURORA; j++) {
	    if (AU_MASK(hw->info.zlink_mask, j))
		status[j] = ioread64(hw->regs_base_addr + au_offset[j]);
	}
	// swsok, disable to reduce messages
	/*
	 * dev_info(dev, "Aurora status[%d]: link0=0x%llx, link1=0x%llx, "
	 * "link2=0x%llx, link3=0x%llx\n", i, status[0], status[1],
	 * status[2], status[3]);
	 */
	for (j = done = 0; j < MAX_AURORA; j++)
	    if (AU_MASK(hw->info.zlink_mask, j) && status[j])
		done = 1;
	if (done)
	    break;
	start = get_cycles();
	udelay(10000);		/* 10 ms */
	duration += (get_cycles() - start);
    }

    /*
     * save initial Aurora link status in core info 
     */
    for (j = 0; j < MAX_AURORA; j++)
	info->link_status[j] = status[j];

    if (duration) {
	dev_info(dev, "%llu ns = %llu cycles\n", 10000ull * i * 1000ull,
		 duration);
    }

    for (i = done = 0; i < 100 && done < num_zlinks; i++) {
	for (j = 0; j < (MAX_ZLINKS + MAX_MZLINKS); j++) {
	    if (!ZLINK_MASK(hw->info.zlink_mask, j) || zdone[j])
		continue;
	    cur_state = gzd_core_interface_enable(hw, zl_offset[j], 0);
	    if (cur_state == I_CFG || cur_state == I_UP) {
		dev_info(dev,
			 "%sinterface%d in %s - enabling\n",
			 (j >= MAX_ZLINKS) ? "media " : "",
			 (j >= MAX_ZLINKS) ? j - MAX_ZLINKS : j,
			 interface_state_name[cur_state]);
		zdone[j] = 1;
		done++;
	    }
	    info->intf_state[j] = cur_state;
	}
	if (done < num_zlinks)
	    udelay(10);
    }
}

static struct gzd_core_hw *gzd_core_validate_hw(void *hw)
{
    struct gzd_core_hw *ptr, *next;

    list_for_each_entry_safe(ptr, next, &gzd_core_hw_list, list) {
	if (ptr == hw)
	    return ptr;
    }

    pr_err("invalid gzd_core hw pointer: %p\n", hw);
    return 0;
}

long int num_request = 0;
long int num_response = 0;

static int __gzd_core_block_io_request(struct gzd_core_hw *hw, dma_addr_t host_addr, uint64_t genz_addr, uint len, uint req_id, uint dcid, uint rw	/* w=0, 
																			 * r=1 
																			 */ )
{
    ulong flags;
    uint64_t request, trig, resp;
    uint num_resps, num_reqs, req_overflow, resp_overflow, overflow = 0;
    int err, free_reqs, free_resps;

    spin_lock_irqsave(&hw->lock, flags);
    free_reqs = hw->info.req_fifo_depth - hw->block_num_reqs;
    free_resps = hw->info.resp_fifo_depth - hw->block_num_resps -
	hw->block_num_reqs;
    /*
     * There is a small window in the HW where an outstanding request is
     * not tracked by either the HW num_reqs or num_resps value. To
     * compensate, we leave one extra slot in the resp FIFO - hence the "> 
     * 1" here and the "<= 2" in the EBUSY if. 
     */
    if (!(free_reqs > 0 && free_resps > 1)) {
	/*
	 * our driver-maintained counts indicate full or overflow - check
	 * the HW before returning an error 
	 */
	resp = ioread64(hw->regs_base_addr + BlockControl);
	num_resps = (uint) bitfield_extract(resp, 56, 8);
	num_reqs = (uint) bitfield_extract(resp, 48, 8);
	req_overflow = (uint) bitfield_extract(resp, 13, 1);
	resp_overflow = (uint) bitfield_extract(resp, 15, 1);
	hw->block_num_resps = num_resps;
	hw->block_num_reqs = num_reqs;
	/*
	 * recompute free_reqs, free_resps 
	 */
	free_reqs = hw->info.req_fifo_depth - hw->block_num_reqs;
	free_resps = hw->info.resp_fifo_depth - hw->block_num_resps -
	    hw->block_num_reqs;
	overflow = req_overflow || resp_overflow;
    }
    if (overflow || free_reqs < 0 || free_resps < 0) {
	hw->block_io_fifo_overflow_count++;
	dev_err(&hw->pdev->dev,
		"%s overflow=%u, free_reqs=%d, free_resps=%d\n", __func__,
		overflow, free_reqs, free_resps);
	err = -ENOSPC;
	goto unlock;
    } else if (free_reqs <= 0 || free_resps <= 2) {
	hw->block_io_fifo_full_count++;
	err = -EBUSY;
	goto unlock;
    }
    hw->block_io_request_count++;
    hw->block_num_reqs++;

    iowrite64(host_addr, hw->regs_base_addr + BlockHostAddr);
    iowrite64(genz_addr, hw->regs_base_addr + BlockGenZAddr);
    request = bitfield_insert(0, 0, 11, dcid);
    request = bitfield_insert(request, 16, 16, len);
    request = bitfield_insert(request, 32, 15, req_id);
    iowrite64(request, hw->regs_base_addr + BlockRequest);
    trig = bitfield_insert(0, 1, 1, rw);
    trig = bitfield_insert(trig, 0, 1, 1);	/* doorbell */
    iowrite64(trig, hw->regs_base_addr + BlockControl);
    spin_unlock_irqrestore(&hw->lock, flags);
    return 0;

  unlock:
    spin_unlock_irqrestore(&hw->lock, flags);
    return err;
}

static int gzd_core_block_io_request(void *arg, dma_addr_t host_addr, uint64_t genz_addr, uint len, uint req_id, uint dcid, uint rw	/* w=0, 
																	 * r=1 
																	 */ )
{
    struct gzd_core_hw *hw;

    hw = gzd_core_validate_hw(arg);
    if (!hw)
	return -EINVAL;

    return __gzd_core_block_io_request(hw, host_addr, genz_addr, len,
				       req_id, dcid, rw);
}

static int
__gzd_core_block_io_response(struct gzd_core_hw *hw,
			     uint * req_id, uint * free_reqs)
{
    ulong flags;
    int err = 0, done;
    uint num_resps, num_reqs, req_overflow, resp_overflow, nak;
    uint64_t resp, advance, overflow = 0;

    spin_lock_irqsave(&hw->lock, flags);
    resp = ioread64(hw->regs_base_addr + BlockControl);
    done = (int) bitfield_extract(resp, 8, 1);
    num_resps = (uint) bitfield_extract(resp, 56, 8);
    num_reqs = (uint) bitfield_extract(resp, 48, 8);
    req_overflow = (uint) bitfield_extract(resp, 13, 1);
    resp_overflow = (uint) bitfield_extract(resp, 15, 1);
    hw->block_num_resps = num_resps;
    hw->block_num_reqs = num_reqs;
    /*
     * There is a small window in the HW where an outstanding request is
     * not tracked by either the HW num_reqs or num_resps value. To
     * compensate, we leave one extra slot in the resp FIFO - hence the "- 
     * 1" here. 
     */
    *free_reqs = (uint) max(0,
			    min((int) (hw->info.req_fifo_depth - num_reqs),
				(int) (hw->info.resp_fifo_depth -
				       num_resps - num_reqs - 1)));

    if (done) {			/* get req_id, check for NAK, advance
				 * response FIFO */
	hw->block_io_response_count++;
	*req_id = (uint) bitfield_extract(resp, 32, 15);
	nak = (uint) bitfield_extract(resp, 47, 1);
	// swsok, ignore NAK
	/*
	 * if (nak) { hw->block_io_nak_count++; err = -EIO; } 
	 */
	advance = bitfield_insert(0, 8, 1, 1);
	iowrite64(advance, hw->regs_base_addr + BlockControl);
	dev_dbg(&hw->pdev->dev,
		"%s: done, num_resps=%u, num_reqs=%u, req_id=%u, nak=%u\n",
		__func__, num_resps, num_reqs, *req_id, nak);
    }

    if (req_overflow || resp_overflow) {	/* clear FIFOs, return
						 * error */
	if (req_overflow) {
	    overflow = bitfield_insert(overflow, 12, 1, 1);
	    overflow = bitfield_insert(overflow, 13, 1, 1);
	    dev_err(&hw->pdev->dev, "block req FIFO overflow "
		    "(block_control=0x%llx)\n", resp);
	}
	if (resp_overflow) {
	    overflow = bitfield_insert(overflow, 14, 1, 1);
	    overflow = bitfield_insert(overflow, 15, 1, 1);
	    dev_err(&hw->pdev->dev, "block resp FIFO overflow "
		    "(block_control=0x%llx)\n", resp);
	}
	iowrite64(overflow, hw->regs_base_addr + BlockControl);
	err = -ENOSPC;		/* Revisit: appropriate error? */
    }

    spin_unlock_irqrestore(&hw->lock, flags);
    return err ? err : done;
}

/*
 * Return Reason req_id valid? free_reqs valid? -EINVAL bad hw ptr N N
 * -ENOSPC FIFO overflow N Y -EIO NAK response Y Y 0 no response N Y 1
 * valid response Y Y 
 */
static int
gzd_core_block_io_response(void *arg, uint * req_id, uint * free_reqs)
{
    struct gzd_core_hw *hw;

    hw = gzd_core_validate_hw(arg);
    if (!hw)
	return -EINVAL;

    return __gzd_core_block_io_response(hw, req_id, free_reqs);
}

static irqreturn_t gzd_core_error_interrupt(int irq, void *dev_id)
{
    struct gzd_core_hw *hw = dev_id;
    struct device *dev = &hw->pdev->dev;
    uint irq_index = irq - hw->base_irq;
    ulong flags;
    uint64_t block_control, msg_control, at_status, at_error, seq_status;

    dev_err_ratelimited(dev, "error interrupt %d (hw=%p, irq_index=%u)\n",
			irq, hw, irq_index);

    spin_lock_irqsave(&hw->lock, flags);
    /*
     * read BlockControl, MsgControl, AtStatus, AtError, SeqStatus 
     */
    block_control = ioread64(hw->regs_base_addr + BlockControl);
    msg_control = ioread64(hw->regs_base_addr + MsgControl);
    at_status = ioread64(hw->regs_base_addr + AtStatus);
    at_error = ioread64(hw->regs_base_addr + AtError);
    seq_status = ioread64(hw->regs_base_addr + SeqStatus);
    spin_unlock_irqrestore(&hw->lock, flags);
    dev_err_ratelimited(dev,
			"  block_control = 0x%llx\n"
			"  msg_control   = 0x%llx\n"
			"  at_status     = 0x%llx\n"
			"  at_error      = 0x%llx\n"
			"  seq_status    = 0x%llx\n",
			block_control, msg_control,
			at_status, at_error, seq_status);
    return IRQ_HANDLED;
}

//#define DEBUG
static irqreturn_t gzd_core_block_io_interrupt(int irq, void *dev_id)
{
    struct gzd_core_hw *hw = dev_id;
    struct device *dev = &hw->pdev->dev;
    int ret, handled = 0;
    uint req_id=0, req_index, free_reqs;

printk(KERN_ERR "%s %s %d ----------------\n", __FILE__, __FUNCTION__, __LINE__);

    dev_info_ratelimited(dev, "block i/o interrupt %d (hw=%p)\n", irq, hw);
    do {
	ret = gzd_core_block_io_response(hw, &req_id, &free_reqs);
	dev_info_ratelimited(dev,
			     "  req_id=%u, free_reqs=%u, ret=%d\n",
			     req_id, free_reqs, ret);
	if (ret == 0) {
	    break;		/* not ours */
	} else if (ret == 1) {
	    handled = 1;
	    req_index = req_id - hw->test_min_req_id;
	    set_bit(req_index, hw->block_io_done);
	    wake_up_interruptible(&hw->block_io_queue);
	}
    }
    while (ret == 1);

    return (handled) ? IRQ_HANDLED : IRQ_NONE;
}

static int
__gzd_core_msg_request(struct gzd_core_hw *hw,
		       dma_addr_t host_addr, uint len,
		       uint req_id, uint dcid)
{
    cycles_t start = get_cycles();
    ulong flags;
    uint64_t request, trig, resp;
    uint num_resps, num_reqs, req_overflow, resp_overflow, overflow = 0;
    int err, free_reqs, free_resps;

    spin_lock_irqsave(&hw->lock, flags);
    free_reqs = hw->info.req_fifo_depth - hw->msg_num_reqs;
    free_resps = hw->info.resp_fifo_depth - hw->msg_num_resps -
	hw->msg_num_reqs;
    /*
     * There is a small window in the HW where an outstanding request is
     * not tracked by either the HW num_reqs or num_resps value. To
     * compensate, we leave one extra slot in the resp FIFO - hence the "> 
     * 1" here and the "<= 2" in the EBUSY if. 
     */
    if (!(free_reqs > 0 && free_resps > 1)) {
	/*
	 * our driver-maintained counts indicate full or overflow - check
	 * the HW before returning an error 
	 */
	resp = ioread64(hw->regs_base_addr + MsgControl);
	num_resps = (uint) bitfield_extract(resp, 56, 8);
	num_reqs = (uint) bitfield_extract(resp, 48, 8);
	req_overflow = (uint) bitfield_extract(resp, 13, 1);
	resp_overflow = (uint) bitfield_extract(resp, 15, 1);
	hw->msg_num_resps = num_resps;
	hw->msg_num_reqs = num_reqs;
	/*
	 * recompute free_reqs, free_resps 
	 */
	free_reqs = hw->info.req_fifo_depth - hw->msg_num_reqs;
	free_resps = hw->info.resp_fifo_depth - hw->msg_num_resps -
	    hw->msg_num_reqs;
	overflow = req_overflow || resp_overflow;
    }
    if (overflow || free_reqs < 0 || free_resps < 0) {
	hw->msg_fifo_overflow_count++;
	dev_err(&hw->pdev->dev,
		"%s overflow=%u, free_reqs=%d, free_resps=%d\n", __func__,
		overflow, free_reqs, free_resps);
	err = -ENOSPC;
	goto unlock;
    } else if (free_reqs <= 0 || free_resps <= 2) {
	hw->msg_fifo_full_count++;
	err = -EBUSY;
	goto unlock;
    }
    hw->msg_request_count++;
    hw->msg_num_reqs++;
    iowrite64(host_addr, hw->regs_base_addr + MsgHostAddr);
    request = bitfield_insert(0, 0, 11, dcid);
    request = bitfield_insert(request, 16, 16, len);
    request = bitfield_insert(request, 32, 15, req_id);
    iowrite64(request, hw->regs_base_addr + MsgRequest);
    trig = bitfield_insert(0, 0, 1, 1);	/* doorbell */
    iowrite64(trig, hw->regs_base_addr + MsgControl);
    spin_unlock_irqrestore(&hw->lock, flags);
    dev_dbg(&hw->pdev->dev,
	    "%s: start=%llu, end=%llu, req_id=%u\n",
	    __func__, start, get_cycles(), req_id);
    return 0;

  unlock:
    spin_unlock_irqrestore(&hw->lock, flags);
    return err;
}

static int
gzd_core_msg_request(void *arg, dma_addr_t host_addr, uint len,
		     uint req_id, uint dcid)
{
    struct gzd_core_hw *hw;

    hw = gzd_core_validate_hw(arg);
    if (!hw)
	return -EINVAL;

    return __gzd_core_msg_request(hw, host_addr, len, req_id, dcid);
}

static int
__gzd_core_msg_response(struct gzd_core_hw *hw,
			uint * req_id, uint * free_reqs)
{
    cycles_t start = get_cycles();
    ulong flags;
    int err = 0, done;
    uint num_resps, num_reqs, nak, req_overflow, resp_overflow;
    uint64_t resp, advance, overflow = 0;

    spin_lock_irqsave(&hw->lock, flags);
    resp = ioread64(hw->regs_base_addr + MsgControl);
    done = (int) bitfield_extract(resp, 8, 1);
    num_resps = (uint) bitfield_extract(resp, 56, 8);
    num_reqs = (uint) bitfield_extract(resp, 48, 8);
    req_overflow = (uint) bitfield_extract(resp, 13, 1);
    resp_overflow = (uint) bitfield_extract(resp, 15, 1);
    hw->msg_num_resps = num_resps;
    hw->msg_num_reqs = num_reqs;
    /*
     * There is a small window in the HW where an outstanding request is
     * not tracked by either the HW num_reqs or num_resps value. To
     * compensate, we leave one extra slot in the resp FIFO - hence the "- 
     * 1" here. 
     */
    *free_reqs = (uint) max(0,
			    min((int) (hw->info.req_fifo_depth - num_reqs),
				(int) (hw->info.resp_fifo_depth -
				       num_resps - num_reqs - 1)));

    if (done) {			/* get req_id, check for NAK, advance
				 * response FIFO */
	hw->msg_response_count++;
	*req_id = (uint) bitfield_extract(resp, 32, 15);
	nak = (uint) bitfield_extract(resp, 47, 1);
	if (nak) {
	    hw->msg_nak_count++;
	    dev_err(&hw->pdev->dev, "NAK: req_id 0x%x\n", *req_id);
	    err = -EIO;
	}
	advance = bitfield_insert(0, 8, 1, 1);
	iowrite64(advance, hw->regs_base_addr + MsgControl);
	dev_dbg(&hw->pdev->dev,
		"%s: done, num_resps=%u, num_reqs=%u, req_id=%u, nak=%u\n",
		__func__, num_resps, num_reqs, *req_id, nak);
    }

    if (req_overflow || resp_overflow) {	/* clear FIFOs, return
						 * error */
	if (req_overflow) {
	    overflow = bitfield_insert(overflow, 12, 1, 1);
	    overflow = bitfield_insert(overflow, 13, 1, 1);
	    dev_err(&hw->pdev->dev, "msg req FIFO overflow "
		    "(msg_control=0x%llx)\n", resp);
	}
	if (resp_overflow) {
	    overflow = bitfield_insert(overflow, 14, 1, 1);
	    overflow = bitfield_insert(overflow, 15, 1, 1);
	    dev_err(&hw->pdev->dev, "msg resp FIFO overflow "
		    "(msg_control=0x%llx)\n", resp);
	}
	iowrite64(overflow, hw->regs_base_addr + MsgControl);
	err = -ENOSPC;		/* Revisit: appropriate error? */
    }

    spin_unlock_irqrestore(&hw->lock, flags);
    dev_dbg(&hw->pdev->dev,
	    "%s: start=%llu, end=%llu, req_id=%u, ret=%d, free_reqs=%u, caller=%pS\n",
	    __func__, start, get_cycles(), *req_id, err ? err : done,
	    *free_reqs, __builtin_return_address(0));
    return err ? err : done;
}

/*
 * Return Reason req_id valid? free_reqs valid? -EINVAL bad hw ptr N N
 * -ENOSPC FIFO overflow N Y -EIO NAK response Y Y 0 no response N Y 1
 * valid response Y Y 
 */
static int
gzd_core_msg_response(void *arg, uint * req_id, uint * free_reqs)
{
    struct gzd_core_hw *hw;

    hw = gzd_core_validate_hw(arg);
    if (!hw)
	return -EINVAL;

    return __gzd_core_msg_response(hw, req_id, free_reqs);
}

static irqreturn_t gzd_core_msg_send_interrupt(int irq, void *dev_id)
{
    struct gzd_core_hw *hw = dev_id;
    struct device *dev = &hw->pdev->dev;
    int ret, more, handled = 0;
    uint req_id, req_index, free_reqs;

    dev_info_ratelimited(dev, "msg send interrupt %d (hw=%p)\n", irq, hw);
    do {
	ret = gzd_core_msg_response(hw, &req_id, &free_reqs);
	dev_info_ratelimited(dev,
			     "  req_id=%u, free_reqs=%u, ret=%d\n",
			     req_id, free_reqs, ret);
	if (ret == 0) {
	    break;		/* not ours */
	} else if (ret == 1 || ret == -EIO) {	/* req_id is valid */
	    more = handled = 1;
	    req_index = req_id - hw->test_min_req_id;
	    set_bit(req_index, hw->msg_done);
	    wake_up_interruptible(&hw->msg_send_queue);
	}
    }
    while (more);

    return (handled) ? IRQ_HANDLED : IRQ_NONE;
}

struct gzd_msg_header *gzd_core_msg_next(struct gzd_core_hw *hw,
					 struct gzd_msg_header *msg_hdr)
{
    struct gzd_msg_header *next_hdr;
    void *prod_ptr, *buf_end_va;
    size_t size;

    __gzd_core_msg_read_prod_ptr(hw, &hw->msg_tbl, 0);
    prod_ptr = hw->msg_tbl.prod_ptr_va;
    size = roundup(msg_hdr->len, sizeof(*msg_hdr));
    next_hdr = msg_hdr + (size / sizeof(*msg_hdr));
    buf_end_va = hw->msg_tbl.buf_start_va + hw->msg_tbl.buf_size;
    if ((void *) next_hdr >= buf_end_va)	/* wrap */
	next_hdr -= (hw->msg_tbl.buf_size / sizeof(*msg_hdr));

    return (next_hdr != prod_ptr) ? next_hdr : 0;
}

struct gzd_core_msg_work {
    struct gzd_core_hw *hw;
    struct work_struct work;
};

static int
gzd_core_msg_test_recv(struct gzd_core_hw *hw,
		       struct gzd_msg_header *msg_hdr)
{
    struct device *dev = &hw->pdev->dev;
    struct gzd_core_msg_recv *msg_recv;
    void *buf_end_va;
    size_t msg_len_before_wrap;
    size_t msg_len_after_wrap;
    long serving;

    msg_recv = kmalloc(sizeof(*msg_recv), GFP_KERNEL);
    if (!msg_recv) {
	dev_err_ratelimited(dev, "msg_recv kmalloc failed\n");
	return -ENOMEM;
    }
    msg_recv->hw = hw;
    buf_end_va = hw->msg_tbl.buf_start_va + hw->msg_tbl.buf_size;
    if (((void *) msg_hdr + msg_hdr->len) > buf_end_va)
	msg_len_before_wrap = buf_end_va - (void *) msg_hdr;
    else
	msg_len_before_wrap = msg_hdr->len;
    msg_len_after_wrap = msg_hdr->len - msg_len_before_wrap;
    /*
     * alloc & memcpy msg 
     */
    msg_recv->msg = kmalloc(msg_hdr->len, GFP_KERNEL);
    if (!msg_recv->msg) {
	dev_err_ratelimited(dev, "msg_recv msg kmalloc failed\n");
	return -ENOMEM;
    }
    memcpy(msg_recv->msg, msg_hdr, msg_len_before_wrap);
    if (msg_len_after_wrap)
	memcpy(msg_recv->msg + msg_len_before_wrap,
	       hw->msg_tbl.buf_start_va, msg_len_after_wrap);
    /*
     * must hold lock before updating msg_recv ticket or the waiters could
     * see the updated serving but not find their ticket in the list 
     */
    mutex_lock(&hw->msg_recv_lock);
    serving = atomic_long_inc_return(&hw->msg_recv_serving);
    msg_recv->ticket = serving;
    dev_info_ratelimited(dev,
			 "  msg hdr=%p len=%hu, scid=0x%hx, serving=%ld\n",
			 msg_hdr, msg_hdr->len, msg_hdr->scid, serving);
    /*
     * add msg_recv to list - under lock 
     */
    list_add_tail(&msg_recv->list, &hw->msg_recv_list);
    mutex_unlock(&hw->msg_recv_lock);
    wake_up_interruptible(&hw->msg_recv_queue);

    return 0;
}

static void gzd_core_msg_work_handler(struct work_struct *w)
{
    struct gzd_core_msg_work *msg_work;
    struct gzd_core_hw *hw;
    struct device *dev;
    struct gzd_msg_header *msg_hdr;
    size_t total_size, behind;
    int err;

    msg_work = container_of(w, struct gzd_core_msg_work, work);
    hw = msg_work->hw;
    dev = &hw->pdev->dev;
    dev_info_ratelimited(dev, "msg work %p (hw=%p)\n", msg_work, hw);
    do {
	msg_hdr = hw->msg_tbl.cons_ptr_va;
	total_size = 0;
	do {
	    if (msg_test) {
		err = gzd_core_msg_test_recv(hw, msg_hdr);
		if (err)
		    return;
	    }
	    total_size += roundup(msg_hdr->len, sizeof(*msg_hdr));
	    msg_hdr = gzd_core_msg_next(hw, msg_hdr);
	}
	while (msg_hdr);

	/*
	 * move cons_ptr by total size and test if we're still behind 
	 */
	behind = __gzd_core_msg_update_cons_ptr(hw, &hw->msg_tbl, 0,
						total_size);
	dev_info_ratelimited(dev, "msg behind=%zu\n", behind);
    }
    while (behind);

    kfree(msg_work);
}

static irqreturn_t gzd_core_msg_recv_interrupt(int irq, void *dev_id)
{
    struct gzd_core_hw *hw = dev_id;
    struct device *dev = &hw->pdev->dev;
    struct gzd_core_msg_work *msg_work;

    if (msg_intr_udelay)
	udelay(msg_intr_udelay);
    (void) __gzd_core_msg_read_prod_ptr(hw, &hw->msg_tbl, 0);
    dev_info_ratelimited(dev, "msg recv interrupt %d (hw=%p, udelay=%u)\n",
			 irq, hw, msg_intr_udelay);
    if (hw->msg_tbl.cons_ptr == hw->msg_tbl.prod_ptr) {
	dev_info_ratelimited(dev, "  spurious (cons_ptr == prod_ptr)\n");
	return IRQ_HANDLED;
    } else {
	dev_info_ratelimited(dev, "  cons_ptr=%pad, prod_ptr=%pad\n",
			     &hw->msg_tbl.cons_ptr, &hw->msg_tbl.prod_ptr);
    }
    msg_work = kmalloc(sizeof(*msg_work), GFP_ATOMIC);
    if (!msg_work) {
	dev_err_ratelimited(dev, "msg_work kmalloc failed\n");
	return IRQ_NONE;
    }
    msg_work->hw = hw;
    INIT_WORK(&msg_work->work, gzd_core_msg_work_handler);
    queue_work(gzd_core_wq, &msg_work->work);

    return IRQ_HANDLED;
}

#define in_range(val, start, end) ((val) >= (start) && (val) <= (end))
#define is_aligned(addr, alignment) (((addr) & ((alignment) - 1)) == 0)

static int
gzd_core_write_msg_tbl_entry(struct gzd_core_hw *hw, uint index,
			     struct gzd_msg_tbl_entry *entry)
{
    struct device *dev = &hw->pdev->dev;
    uint64_t scid, start, end, prod, cons, trig;
    const uint align = sizeof(struct gzd_msg_header);

    /*
     * caller must already hold hw->lock 
     */
    scid = entry->scid;
    start = entry->buf_start;
    end = start + entry->buf_size;	/* HW requires 1 past "true" end */
    prod = entry->prod_ptr_fixed_loc;
    /*
     * the HW cons_ptr is an offset 
     */
    cons = entry->cons_ptr - start;
    if (end <= start || !in_range(entry->cons_ptr, start, end) ||
	!is_aligned(cons, align) || !is_aligned(prod, 8) ||
	!is_aligned(start, align) || !is_aligned(end, align))
	return -EINVAL;
    dev_info(dev, "msg_tbl[%u]: 0x%016llx-0x%016llx, "
	     "scid=0x%llx, prod=0x%llx, cons=0x%llx\n",
	     index, start, end, scid, prod, cons);
    trig = bitfield_insert(0, 0, 8, index);
    trig = bitfield_insert(trig, 16, 1, 1);	/* write scid */
    trig = bitfield_insert(trig, 17, 1, 1);	/* write buffer addrs */
    trig = bitfield_insert(trig, 18, 1, 1);	/* write prod_ptr addr */
    trig = bitfield_insert(trig, 19, 1, 1);	/* write cons_ptr */
    iowrite64(scid, hw->regs_base_addr + SeqTeWrSCID);
    iowrite64(start, hw->regs_base_addr + SeqTeWrBufStart);
    iowrite64(end, hw->regs_base_addr + SeqTeWrBufEnd);
    iowrite64(prod, hw->regs_base_addr + SeqTeWrProdPtr);
    iowrite64(cons, hw->regs_base_addr + SeqTeWrConsPtr);
    iowrite64(trig, hw->regs_base_addr + SeqTeControl);

    return 0;
}

static int
gzd_core_msg_tbl_init(void *arg, struct gzd_msg_tbl_entry *tbl,
		      uint entries)
{
    struct gzd_core_hw *hw;
    ulong flags;
    uint i;
    int ret;

    hw = gzd_core_validate_hw(arg);
    if (!hw)
	return -EINVAL;

    spin_lock_irqsave(&hw->lock, flags);

    /*
     * Revisit: check status reg (0x70) and clear? 
     */
    /*
     * Revisit: no param reg with max table entries? 
     */
    /*
     * Revisit: current HW only supports 1 entry 
     */
    if (entries != 1u)
	return -EINVAL;

    for (i = 0; i < entries; i++) {
	ret = gzd_core_write_msg_tbl_entry(hw, i, &tbl[i]);
	if (ret < 0) {
	    entries = 0;
	    break;
	}
	/*
	 * update prod_ptrs 
	 */
	__gzd_core_msg_read_prod_ptr(hw, tbl, i);
    }

    iowrite64((uint64_t) entries, hw->regs_base_addr + SeqTblControl);
    spin_unlock_irqrestore(&hw->lock, flags);
    return ret;
}

/*
 * returns current prod_ptr offset 
 */
/*
 * also returns current prod_ptr and prod_ptr_va in tbl[index] 
 */
uint64_t
__gzd_core_msg_read_prod_ptr(struct gzd_core_hw *hw,
			     struct gzd_msg_tbl_entry *tbl, uint index)
{
    uint64_t prod_offset;

    /*
     * Read current prod_ptr using prod_ptr_fixed_va; HW keeps as offset 
     */
    prod_offset = *tbl[index].prod_ptr_fixed_va;
    tbl[index].prod_ptr = prod_offset + tbl[index].buf_start;
    tbl[index].prod_ptr_va = prod_offset + tbl[index].buf_start_va;

    return prod_offset;
}

/*
 * returns current prod_ptr offset 
 */
/*
 * also returns current prod_ptr and prod_ptr_va in tbl[index] 
 */
uint64_t
gzd_core_msg_read_prod_ptr(void *arg, struct gzd_msg_tbl_entry *tbl,
			   uint index)
{
    struct gzd_core_hw *hw;

    hw = gzd_core_validate_hw(arg);
    if (!hw)
	return -EINVAL;

    return __gzd_core_msg_read_prod_ptr(hw, tbl, index);
}

/*
 * returns how far behind prod_ptr the updated cons_ptr is 
 */
/*
 * also returns current prod_ptr in tbl[index].prod_ptr 
 */
size_t
__gzd_core_msg_update_cons_ptr(struct gzd_core_hw *hw,
			       struct gzd_msg_tbl_entry *tbl,
			       uint index, size_t size)
{
    ulong flags;
    uint64_t prod_offset, cons, trig;
    size_t buf_size, cur_cons_offset, new_cons_offset, cons_diff;

    buf_size = tbl[index].buf_size;
    cur_cons_offset = tbl[index].cons_ptr - tbl[index].buf_start;
    new_cons_offset = (cur_cons_offset + size) % buf_size;
    if (cur_cons_offset + size != new_cons_offset)	/* wrap */
	size = new_cons_offset - cur_cons_offset;	/* this is <= 0 */

    spin_lock_irqsave(&hw->lock, flags);
    tbl[index].cons_ptr += size;
    tbl[index].cons_ptr_va += size;
    /*
     * the HW cons_ptr is an offset 
     */
    cons = new_cons_offset;
    iowrite64(cons, hw->regs_base_addr + SeqTeWrConsPtr);
    trig = bitfield_insert(0, 0, 8, index);
    trig = bitfield_insert(trig, 19, 1, 1);	/* write cons ptr */
    iowrite64(trig, hw->regs_base_addr + SeqTeControl);

    /*
     * Read current prod_ptr 
     */
    prod_offset = __gzd_core_msg_read_prod_ptr(hw, tbl, index);
    /*
     * compute how far behind prod_ptr the updated cons_ptr is 
     */
    if (new_cons_offset <= prod_offset)
	cons_diff = prod_offset - new_cons_offset;
    else
	cons_diff = buf_size - (new_cons_offset - prod_offset);

    spin_unlock_irqrestore(&hw->lock, flags);
    return cons_diff;
}

/*
 * returns how far behind prod_ptr the updated cons_ptr is 
 */
/*
 * also returns current prod_ptr in tbl[index].prod_ptr 
 */
size_t
gzd_core_msg_update_cons_ptr(void *arg, struct gzd_msg_tbl_entry *tbl,
			     uint index, size_t size)
{
    struct gzd_core_hw *hw;

    hw = gzd_core_validate_hw(arg);
    if (!hw)
	return -EINVAL;

    return __gzd_core_msg_update_cons_ptr(hw, tbl, index, size);
}

/*
 * forward declarations 
 */
static int __gzd_core_request_irq(struct gzd_core_hw *hw,
				  uint irq_index,
				  irq_handler_t handler,
				  ulong irqflags, const char *devname,
				  void *dev_id, uint irq_type);
static int __gzd_core_free_irq(struct gzd_core_hw *hw, uint irq_index,
			       void *dev_id, uint irq_type);

static int gzd_core_thread(void *data)
{
    struct gzd_core_hw *hw = data;
    struct device *dev = &hw->pdev->dev;
    struct gzd_core_info *info = &hw->info;
    uint64_t status, prev_link_status;
    uint prev_state[MAX_ZLINKS + MAX_MZLINKS];
    uint cur_state[MAX_ZLINKS + MAX_MZLINKS];
    int j;
    ulong flags;

    while (1) {
	if (kthread_should_stop())
	    break;
	/*
	 * check Aurora link status 
	 */
	spin_lock_irqsave(&hw->lock, flags);
	for (j = 0; j < MAX_AURORA; j++) {
	    if (!AU_MASK(hw->info.zlink_mask, j))
		continue;
	    prev_link_status = info->link_status[j];
	    status = ioread64(hw->regs_base_addr + au_offset[j]);
	    /*
	     * print message on status changes 
	     */
	    if (prev_link_status != status) {
		// swsok, disable to reduce messages
		/*
		 * dev_info(dev, "Aurora link%d status=0x%llx\n", j,
		 * status);
		 */
		info->link_status[j] = status;
	    }
	}
	/*
	 * check interface state 
	 */
	for (j = 0; j < (MAX_ZLINKS + MAX_MZLINKS); j++) {
	    if (!ZLINK_MASK(hw->info.zlink_mask, j))
		continue;
	    prev_state[j] = info->intf_state[j];
	    cur_state[j] = gzd_core_interface_enable(hw, zl_offset[j], 1);
	    info->intf_state[j] = cur_state[j];
	    if (cur_state[j] == I_CFG) {
		dev_info(dev,
			 "%sinterface%d in %s - enabling\n",
			 (j >= MAX_ZLINKS) ? "media " : "",
			 (j >= MAX_ZLINKS) ? j - MAX_ZLINKS : j,
			 interface_state_name[cur_state[j]]);
	    }
	}
	spin_unlock_irqrestore(&hw->lock, flags);
	/*
	 * print message on state changes 
	 */
	for (j = 0; j < (MAX_ZLINKS + MAX_MZLINKS); j++) {
	    if (!ZLINK_MASK(hw->info.zlink_mask, j))
		continue;
	    if (prev_state[j] != cur_state[j]) {
		dev_info(dev, "%sinterface%d transition from %s to %s\n",
			 (j >= MAX_ZLINKS) ? "media " : "",
			 (j >= MAX_ZLINKS) ? j - MAX_ZLINKS : j,
			 interface_state_name[prev_state[j]],
			 interface_state_name[cur_state[j]]);
	    }
	}
	/*
	 * sleep for a while 
	 */
	msleep(poll_interval);
    }

    return 0;
}

static int gzd_core_start_thread(struct gzd_core_hw *hw)
{
    int ret = 0;

    hw->thread = kthread_run(gzd_core_thread, (void *) hw, "gzd-core");
    if (IS_ERR(hw->thread)) {
	ret = PTR_ERR(hw->thread);
	hw->thread = 0;
    }

    return ret;
}

static int __init
gzd_core_probe(struct pci_dev *pdev, const struct pci_device_id *pdev_id)
{
    int err;
    int using_dac = 0;
    struct device *dev = &pdev->dev;
    struct gzd_core_hw *hw;
    uint64_t hwid, timestamp, major, minor;
    char timestr[32];
    void *cpu_addr = 0;
    dma_addr_t dma_addr;
    size_t count = memparse(recv_buf_size, 0);
    struct gzd_core_pci_info *pci_info;
    int nvec;
    unsigned int genz_ver_major=0, genz_ver_minor=0;

//printk(KERN_WARNING "pdev->device=0x%4hX\n", pdev->device);

    switch(pdev->device){
    case 0x020b:
	genz_ver_minor = 7;
	break;
    case 0x8032:
    case 0x8034:
	genz_ver_major = 1;
	break;
    case 0x8035:
	genz_ver_major = 1;
	genz_ver_minor = 1;
	break;
    case 0x9034:
	genz_ver_major = 1;
	break;
    default:
	printk(KERN_ERR "pci device %s is not a Gen-Z H/W.\n", pci_name(pdev));
	return -EINVAL;
    }
    printk(KERN_WARNING "Gen-Z H/W v%u.%u\n", genz_ver_major, genz_ver_minor);

//swsok, to be successful to load this module after pmem module initialized.
usleep_range(2000000, 4000000);

    pci_info = &gzd_core_pci_info_table[pdev_id->driver_data];
    hw = gzd_core_hw_alloc(pdev, pci_info);
    if (!hw)
	return -ENOMEM;

    err = pci_enable_device(pdev);
    if (err) {
	dev_err(dev, "pci_enable_device probe error %d for device %s\n",
		err, pci_name(pdev));
	goto free_mem;
    }

    if (pat != 4) {
	    err = pci_request_regions(pdev, GZD_CORE_DEV_NAME);
	    if (err < 0) {
		dev_err(dev, "pci_request_regions error %d\n", err);
		goto pci_disable;
	    }
    }
    /*
     * bar: media data mmio region 
     */
    hw->media_mmio_start = pci_resource_start(pdev, GZD_CORE_MEDIA_BAR);
    hw->media_mmio_len = pci_resource_len(pdev, GZD_CORE_MEDIA_BAR);
    // swsok, iomap with cache policy, set PAT for BAR2/BAR4 region
    // hw->media_base_addr = pci_iomap(pdev, GZD_CORE_MEDIA_BAR, 0);
    switch (pat) {
    case 0:			// WB
	hw->media_base_addr =
	    ioremap_cache(hw->media_mmio_start, hw->media_mmio_len);
	break;
    case 1:			// WT
	hw->media_base_addr =
	    ioremap_wt(hw->media_mmio_start, hw->media_mmio_len);
	break;
    case 2:			// WC
	hw->media_base_addr =
	    ioremap_wc(hw->media_mmio_start, hw->media_mmio_len);
	break;
    case 3:			// UC
	hw->media_base_addr =
	    ioremap_nocache(hw->media_mmio_start, hw->media_mmio_len);
	break;
    case 4:			//No ioremap and set mtrr
	hw->media_base_addr = 1;
	break;
    default:			// Error. fix it with pat=3
	pat = 3;
	hw->media_base_addr = pci_iomap(pdev, GZD_CORE_MEDIA_BAR, 0);
    }
    // swsok

    if (!hw->media_base_addr) {
	dev_err(dev, "cannot iomap bar %u region of size %lu\n",
		GZD_CORE_MEDIA_BAR, (unsigned long) hw->media_mmio_len);
	err = -EINVAL;
	goto pci_release;
    }

    /*
     * bar: control registers 
     */
    hw->regs_start = pci_resource_start(pdev, GZD_CORE_REG_BAR);
    hw->regs_len = pci_resource_len(pdev, GZD_CORE_REG_BAR);
    hw->regs_base_addr = pci_iomap(pdev, GZD_CORE_REG_BAR, 0);
    if (!hw->regs_base_addr) {
	dev_err(dev, "cannot iomap bar %u registers of size %lu\n",
		GZD_CORE_REG_BAR, (unsigned long) hw->regs_len);
	err = -EINVAL;
	goto reg_release;
    }

    /*
     * DMA mask: module-param-specified bits if available, else 32 bits 
     */
    if ((dma_mask > 32 && dma_mask <= 64) &&
	!pci_set_dma_mask(pdev, DMA_BIT_MASK(dma_mask))) {
	using_dac = 1;
    } else {
	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (err) {
	    dev_err(dev, "no usable DMA configuration\n");
	    goto reg_release;
	}
    }

    /*
     * interrupts: get MSI vectors 
     */
    nvec =
	pci_alloc_irq_vectors(pdev, 1, GZD_CORE_MSI_COUNT,
			      PCI_IRQ_MSI | PCI_IRQ_MSIX);
    if (nvec < 0) {
	dev_err(dev, "cannot enable MSI\n");
	err = -EINVAL;
	goto reg_release;
    }
    hw->base_irq = pdev->irq;

    hwid = ioread64(hw->regs_base_addr + HwID);
    timestamp = bitfield_extract(hwid, 32, 32);
    major = bitfield_extract(hwid, 8, 8);
    minor = bitfield_extract(hwid, 0, 8);
    dev_info(dev,
	     "gzd-core version %s, FPGA ID %s rev %llu.%llu (0x%016llx)\n",
	     GZD_DRV_VERS, gzd_core_sprintf_xilinx_timestamp(timestamp,
							     timestr, 32),
	     major, minor, hwid);
    if (fpga_rev_check && major < ExpectedFpgaMajor) {
	dev_err(dev, "FPGA revision incompatible with this driver\n"
		"please update FPGA bitstream to rev %u.%u\n",
		ExpectedFpgaMajor, ExpectedFpgaMinor);
	err = -EINVAL;
	goto msi_release;
    } else if (fpga_rev_check && major > ExpectedFpgaMajor) {
	dev_err(dev, "gzd-core revision incompatible with this FPGA\n"
		"please update driver to rev %llu.%llu\n", major, minor);
	err = -EINVAL;
	goto msi_release;
    } else if (fpga_rev_check && minor < ExpectedFpgaMinor) {
	dev_warn(dev, "FPGA minor revision is lower than driver\n"
		 "some driver features may not function\n");
    }
    dev_info(dev, "media_mmio iomap base = 0x%lx\n",
	     (unsigned long) hw->media_base_addr);
    dev_info(dev, "media_mmio_start = 0x%lx media_mmio_len = %lu\n",
	     (unsigned long) hw->media_mmio_start,
	     (unsigned long) hw->media_mmio_len);
    dev_info(dev, "regs iomap base = 0x%lx, irqs = %u-%u\n",
	     (unsigned long) hw->regs_base_addr,
	     hw->base_irq, hw->base_irq + GZD_CORE_MSI_COUNT - 1);
    dev_info(dev, "regs_addr_start = 0x%lx regs_len = %lu\n",
	     (unsigned long) hw->regs_start, (unsigned long) hw->regs_len);

    pci_set_master(pdev);
    gzd_core_hw_info_init(hw);
    dev_info(dev, "card id %u, hw=%p\n", hw->info.card_id, hw);
    gzd_core_reset_and_disable(hw);
//addr_trans must be called after PMEM initializtion done.
    gzd_core_addr_trans_init(hw);

//gzd_core_blue_block_init() must be not called for new gzd-1.0 H/W 
    if (genz_ver_major < 1 && gzd_core_blue_block_init(hw) != 0) {
	dev_err(dev, "blue block init failed\n");
	err = -EINVAL;
	goto msi_release;
    }

    gzd_core_enable(hw);
    dev_info(dev, "gzd_core_enable done\n");
    gzd_core_link_status(hw);
    dev_info(dev, "gzd_link_status done\n");
    gzd_core_add_hw_probe_drivers(hw);

    /*
     * Initialize the memory map offset and size to their initial values 
     */
    hw->memory_map_offset = 0;
    hw->memory_map_size = 8;

    /*
     * Revisit: arbitrary values 
     */
    hw->test_req_id = hw->test_min_req_id = 1234;
    hw->test_max_req_id = hw->test_min_req_id + NUM_TEST_REQ_IDS;
    err = __gzd_core_request_irq(hw, BlockRWInt,
				 gzd_core_block_io_interrupt,
				 0, GZD_CORE_IRQ_ID "-block-rw", hw,
				 GzdCoreIrq);
    if (err)
	goto msi_release;
    err = __gzd_core_request_irq(hw, MsgSendInt,
				 gzd_core_msg_send_interrupt,
				 0, GZD_CORE_IRQ_ID "-msg-send", hw,
				 GzdCoreIrq);
    if (err)
	goto irq_release;

    cpu_addr = dma_alloc_coherent(dev, count, &dma_addr, GFP_KERNEL);
    if (!cpu_addr)
	goto irq_release;
    /*
     * Revisit: for red/green-only bitstreams, assume "loopback" to
     * ourselves; otherwise, assume there's a second card 
     */
    if (red_green)
	hw->msg_tbl.scid = hw->info.msg_info[hw->info.card_index].scid;
    else
	hw->msg_tbl.scid = hw->info.msg_info[hw->info.card_index + 1].scid;
    /*
     * use first 8 bytes for prod_ptr; buf_start is next MSG_BUF_OFFSET
     * aligned location 
     */
    hw->msg_tbl.buf_start = dma_addr + MSG_BUF_OFFSET;
    hw->msg_tbl.buf_start_va = cpu_addr + MSG_BUF_OFFSET;
    hw->msg_tbl.buf_size = count - MSG_BUF_OFFSET;
    hw->msg_tbl.prod_ptr_fixed_loc = dma_addr;
    hw->msg_tbl.prod_ptr_fixed_va = cpu_addr;
    hw->msg_tbl.cons_ptr = hw->msg_tbl.buf_start;
    hw->msg_tbl.cons_ptr_va = hw->msg_tbl.buf_start_va;
    err = gzd_core_msg_tbl_init(hw, &hw->msg_tbl, 1);
    if (err)
	goto dma_release;
    if (msg_test) {
	atomic_long_set(&hw->msg_recv_waiting, 0);
	atomic_long_set(&hw->msg_recv_serving, 0);
	INIT_LIST_HEAD(&hw->msg_recv_list);
	mutex_init(&hw->msg_recv_lock);
    }
    err = __gzd_core_request_irq(hw, MsgRecvInt,
				 gzd_core_msg_recv_interrupt,
				 0, GZD_CORE_IRQ_ID "-msg-recv", hw,
				 GzdCoreIrq);
    if (err)
	goto dma_release;
    err = __gzd_core_request_irq(hw, ByteAddrErrInt,
				 gzd_core_error_interrupt,
				 0, GZD_CORE_IRQ_ID "-byte-addr-err", hw,
				 GzdCoreIrq);
    if (err)
	goto dma_release;
    err = __gzd_core_request_irq(hw, AddrTransErrInt,
				 gzd_core_error_interrupt,
				 0, GZD_CORE_IRQ_ID "-addr-trans-err", hw,
				 GzdCoreIrq);
    if (err)
	goto dma_release;
    err = __gzd_core_request_irq(hw, HWApiErrInt,
				 gzd_core_error_interrupt,
				 0, GZD_CORE_IRQ_ID "-hw-api-err", hw,
				 GzdCoreIrq);
    if (err)
	goto dma_release;
    err = __gzd_core_request_irq(hw, BrSeqErrInt,
				 gzd_core_error_interrupt,
				 0, GZD_CORE_IRQ_ID "-br-seq-err", hw,
				 GzdCoreIrq);
    if (err)
	goto dma_release;
    err = sysfs_create_group(&dev->kobj, &gzd_core_device_attr_group);
    if (err)
	goto dma_release;
    err = gzd_core_start_thread(hw);
    if (err)
	goto sysfs_remove;
    return 0;

  sysfs_remove:
    sysfs_remove_group(&dev->kobj, &gzd_core_device_attr_group);
  dma_release:
    dma_free_coherent(dev, count, cpu_addr, dma_addr);
  irq_release:
    __gzd_core_free_irq(hw, BlockRWInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, MsgSendInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, MsgRecvInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, ByteAddrErrInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, AddrTransErrInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, HWApiErrInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, BrSeqErrInt, hw, GzdCoreIrq);
  msi_release:
    pci_disable_msi(pdev);
  reg_release:
if ( hw->media_base_addr > 1 )
    pci_iounmap(pdev, hw->media_base_addr);
  pci_release:
    pci_release_regions(pdev);
  pci_disable:
    pci_disable_device(pdev);
  free_mem:
    kfree(hw);
    return err;
}

static int gzd_core_stop_thread(struct gzd_core_hw *hw)
{
    int ret = 0;

    if (hw->thread)
	ret = kthread_stop(hw->thread);
    hw->thread = 0;
    return ret;
}

static void gzd_core_remove(struct pci_dev *pdev)
{
    struct gzd_core_hw *hw;

    hw = pci_get_drvdata(pdev);

    gzd_core_stop_thread(hw);
    sysfs_remove_group(&pdev->dev.kobj, &gzd_core_device_attr_group);
    gzd_core_remove_drivers(hw);
    gzd_core_reset_and_disable(hw);
    __gzd_core_free_irq(hw, MsgSendInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, MsgRecvInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, BlockRWInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, ByteAddrErrInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, AddrTransErrInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, HWApiErrInt, hw, GzdCoreIrq);
    __gzd_core_free_irq(hw, BrSeqErrInt, hw, GzdCoreIrq);
    pci_disable_msi(pdev);
    pci_clear_master(pdev);
    pci_iounmap(pdev, hw->regs_base_addr);
    if ( hw->media_base_addr > 1 )
	pci_iounmap(pdev, hw->media_base_addr);
    gzd_core_remove_hw(hw);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
}

static struct pci_driver gzd_core_pci_driver = {
    .name = GZD_CORE_DEV_NAME,
    .id_table = gzd_core_id_table,
    .probe = gzd_core_probe,
    .remove = gzd_core_remove,
};

/*
 * ============================================================ THE
 * DEPENDENT DRIVER INTERFACE
 * ============================================================ 
 */

static struct gzd_core_info *gzd_core_get_info(void *arg)
{
    struct gzd_core_hw *hw;

    hw = gzd_core_validate_hw(arg);
    if (!hw)
	return 0;

    return &hw->info;
}

static void
gzd_core_enable_intr(struct gzd_core_hw *hw, uint irq_index, uint state)
{
    ulong flags;
    uint64_t val;

    if (irq_index < GZD_CORE_MSI_COUNT) {
	spin_lock_irqsave(&hw->lock, flags);
	val = ioread64(hw->regs_base_addr + SeqResetEnable);
	val = bitfield_insert(val, irq_index + 8, 1, state);
	iowrite64(val, hw->regs_base_addr + SeqResetEnable);
	spin_unlock_irqrestore(&hw->lock, flags);
	dev_info(&hw->pdev->dev, "%s: val=0x%llx\n", __func__, val);
    }
}

static int
__gzd_core_request_irq(struct gzd_core_hw *hw, uint irq_index,
		       irq_handler_t handler, ulong irqflags,
		       const char *devname, void *dev_id, uint irq_type)
{
    int ret;
    uint old_irq_type;
    irq_handler_t old_handler;
    ulong old_irqflags;
    const char *old_devname;
    void *old_dev_id;

    if (irq_index >= GZD_CORE_MSI_COUNT)
	return -EINVAL;

    if (!(irq_type == GzdCoreIrq || irq_type == GzdDepIrq))
	return -EINVAL;

    old_irq_type = hw->irq_info[irq_index].irq_type;
    old_handler = hw->irq_info[irq_index].irq_save[old_irq_type].handler;
    old_irqflags = hw->irq_info[irq_index].irq_save[old_irq_type].irqflags;
    old_devname = hw->irq_info[irq_index].irq_save[old_irq_type].devname;
    old_dev_id = hw->irq_info[irq_index].irq_save[old_irq_type].dev_id;

    if (old_irq_type != GzdNoIrq) {
	gzd_core_enable_intr(hw, irq_index, IntrDisable);
	free_irq(hw->base_irq + irq_index, old_dev_id);
    }

    ret = request_irq(hw->base_irq + irq_index, handler, irqflags,
		      devname, dev_id);
    if (ret == 0) {
	gzd_core_enable_intr(hw, irq_index, IntrEnable);
	hw->irq_info[irq_index].irq_type = irq_type;
	hw->irq_info[irq_index].irq_save[irq_type].handler = handler;
	hw->irq_info[irq_index].irq_save[irq_type].irqflags = irqflags;
	hw->irq_info[irq_index].irq_save[irq_type].devname = devname;
	hw->irq_info[irq_index].irq_save[irq_type].dev_id = dev_id;
    }
    return ret;
}

static int
gzd_core_request_irq(void *arg, uint irq_index,
		     irq_handler_t handler, ulong irqflags,
		     const char *devname, void *dev_id)
{
    struct gzd_core_hw *hw;

    hw = gzd_core_validate_hw(arg);
    if (!hw)
	return -EINVAL;
    return __gzd_core_request_irq(hw, irq_index, handler, irqflags,
				  devname, dev_id, GzdDepIrq);
}

static int
__gzd_core_free_irq(struct gzd_core_hw *hw, uint irq_index,
		    void *dev_id, uint irq_type)
{
    int ret;
    uint cur_irq_type, prev_irq_type;
    irq_handler_t prev_handler;
    ulong prev_irqflags;
    const char *prev_devname;
    void *prev_dev_id;

    if (irq_index >= GZD_CORE_MSI_COUNT)
	return -EINVAL;

    if (!(irq_type == GzdCoreIrq || irq_type == GzdDepIrq))
	return -EINVAL;

    cur_irq_type = hw->irq_info[irq_index].irq_type;
    if (cur_irq_type != irq_type)
	return -EINVAL;

    /*
     * disable interrupt and free current irq 
     */
    gzd_core_enable_intr(hw, irq_index, IntrDisable);
    free_irq(hw->base_irq + irq_index, dev_id);

    /*
     * request previous irq and re-enable interrupt 
     */
    prev_irq_type = cur_irq_type - 1;
    if (prev_irq_type == GzdNoIrq)
	return 0;

    prev_handler = hw->irq_info[irq_index].irq_save[prev_irq_type].handler;
    prev_irqflags =
	hw->irq_info[irq_index].irq_save[prev_irq_type].irqflags;
    prev_devname = hw->irq_info[irq_index].irq_save[prev_irq_type].devname;
    prev_dev_id = hw->irq_info[irq_index].irq_save[prev_irq_type].dev_id;
    ret =
	request_irq(hw->base_irq + irq_index, prev_handler, prev_irqflags,
		    prev_devname, prev_dev_id);
    if (ret == 0) {
	gzd_core_enable_intr(hw, irq_index, IntrEnable);
	hw->irq_info[irq_index].irq_type = prev_irq_type;
    }
    return ret;
}

static int gzd_core_free_irq(void *arg, uint irq_index, void *dev_id)
{
    struct gzd_core_hw *hw;

    hw = gzd_core_validate_hw(arg);
    if (!hw)
	return -EINVAL;
    return __gzd_core_free_irq(hw, irq_index, dev_id, GzdDepIrq);
}

int gzd_core_register_driver(struct gzd_driver *drv)
{
    struct gzd_core_hw *hw, *next;

    pr_info("gzd-core registering driver %s\n", drv->name);
    drv->info = gzd_core_get_info;
    drv->request_irq = gzd_core_request_irq;
    drv->free_irq = gzd_core_free_irq;
    drv->block_io_request = gzd_core_block_io_request;
    drv->block_io_response = gzd_core_block_io_response;
    drv->msg_request = gzd_core_msg_request;
    drv->msg_response = gzd_core_msg_response;
    drv->msg_tbl_init = gzd_core_msg_tbl_init;
    drv->msg_update_cons_ptr = gzd_core_msg_update_cons_ptr;
    drv->msg_read_prod_ptr = gzd_core_msg_read_prod_ptr;
    list_add_tail(&drv->list, &gzd_core_driver_list);
    /*
     * call probe function for each card 
     */
    list_for_each_entry_safe(hw, next, &gzd_core_hw_list, list) {
	drv->probe(hw);
    }
    return 0;
}

EXPORT_SYMBOL(gzd_core_register_driver);

void gzd_core_unregister_driver(struct gzd_driver *drv)
{
    struct gzd_core_hw *hw, *next;

    pr_info("gzd-core unregistering driver %s\n", drv->name);
    /*
     * call remove function for each card 
     */
    list_for_each_entry_safe(hw, next, &gzd_core_hw_list, list) {
	drv->remove(hw);
    }
    list_del_init(&drv->list);
}

EXPORT_SYMBOL(gzd_core_unregister_driver);

/*
 * ============================================================ MODULE
 * INIT/EXIT ============================================================ 
 */
static int __init gzd_core_init(void)
{
    int err = -ENOMEM;

    /*
     * register pci device driver 
     */
    err = pci_register_driver(&gzd_core_pci_driver);
    if (err < 0) {
	pr_err("pci_register_driver error %d\n", err);
	goto exit;
    }

    gzd_core_wq = create_singlethread_workqueue("gzd_core_wq");
    if (!gzd_core_wq) {
	err = -ENOMEM;
	goto pci_release;
    }

    return 0;

  pci_release:
    pci_unregister_driver(&gzd_core_pci_driver);
  exit:
    return err;
}

static void __exit gzd_core_fini(void)
{
    if (gzd_core_wq)
	destroy_workqueue(gzd_core_wq);
    pci_unregister_driver(&gzd_core_pci_driver);
}

module_init(gzd_core_init);
module_exit(gzd_core_fini);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Core driver for the Gen-Z demo pci device");
