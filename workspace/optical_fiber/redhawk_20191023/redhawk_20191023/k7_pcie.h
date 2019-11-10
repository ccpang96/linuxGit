#ifndef __V7_PCIE_H
#define __V7_PCIE_H

#include <linux/io.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>


#define DRVER_NAME      "k7"

typedef struct {
	resource_size_t mmio_start;
	resource_size_t mmio_end;
	resource_size_t mmio_len;
	unsigned long mmio_flags;
	unsigned char *mem;
	unsigned char *vmem;
} bar_info_t;

typedef struct k7_device{
	struct pci_dev *dev;
	int legacy_en; /*legacy 閬椾骇*/
	int msi_en;
	int msix_en;
	int in_use;
	int irq;
	int irq_num;
	int irq_en;
	int bar_num;
	bar_info_t bar[4];

	/* board info */
	unsigned char revision;
	unsigned char irq_pin;
	unsigned char irq_line;
	unsigned short sub_vendor_id;
	unsigned short sub_system_id;
	unsigned short vendor_id;
	unsigned short device_id;

	/* char device */
	struct cdev cdev;
	dev_t devno;
}k7_pcie_t;

typedef struct
{
	unsigned long long vir;
	unsigned long phy;
	int size;
}dm_t;

typedef struct {
	int major;
	struct cdev mycdev;
	struct class *myclass;
	struct device *mydev;
} char_drv_info_t;

typedef struct {
	unsigned int t;
	unsigned int l;
	unsigned char v[0];
} tlv_t;

typedef struct {
	unsigned int addr;
	unsigned int value;
	//unsigned int *buffer;
	unsigned int size;
	unsigned int count;
	unsigned char bAttr;
	unsigned char bEP;
	unsigned char bTD;
	unsigned char bTC;
	unsigned char bType;
	unsigned char bFmt;
	unsigned char bFdwbe;
	unsigned char bldwbe;
	unsigned char bTag;
	unsigned short wRID;
}k7_pcie_ioctl_command_t;

typedef struct {
	unsigned int size;
	unsigned int count;
	unsigned char bAttr;
	unsigned char bEP;
	unsigned char bTD;
	unsigned char bTC;
	unsigned char bType;
	unsigned char bFmt;
	unsigned char bFdwbe;
	unsigned char bldwbe;
	unsigned char bTag;
	unsigned short wRID;
}k7_pcie_dma_t;


#define BIT_SET(r, v) ( r = ( r | 1<<v ))
#define BIT_CLR(r, v) ( r = ( r & (~(1<<v)) ) )
#define ALIGN_N_BYTE(p, n) (((unsigned int)p+n-1)&~(n-1))

//define the char device  command
#define K7_PCIE_REG_WRITE	0x0001	/* write fpga register command */
#define K7_PCIE_REG_READ		0x0002	/* read fpga register command */
#define K7_PCIE_CFG_WRITE	0x0003	/* write pcie configure space */
#define K7_PCIE_CFG_READ		0x0004	/* read pcie configure space */
#define K7_PCIE_DMA_WRITE	0x0005	/* config the dma parameter:address,size,count,and write direction */
#define K7_PCIE_DMA_READ		0x0006	/* config the dma parameter:address,size,count,and read direction */
#define K7_PCIE_INT_CLEAR	0x0007  	/* clear the interrupt */

#define K7_PCIE_GET_DMA		0x0010  	/* clear the interrupt */
#define K7_PCIE_GET_DMA_BAK	0x0020  	/* yangbt add 20190312 */
 
//define the fpga register
#define	DMA_WR_START_EN_OFFSET			0x50
#define	DMA_WR_ADDR_LOW_OFFSET			0x54
#define	DMA_WR_ADDR_HIGH_OFFSET			0x58
#define	DMA_WR_TLP_PACKET_CNT_OFFSET		0x5c
#define	DMA_WR_TLP_DATA_LENGTH_OFFSET		0x60
#define	DMA_WR_STOP_EN_OFFSET			0x64
#define	DMA_INT_CLEAR_OFFSET			0x68
#define	PCIE_CMMD_ADDR0_OFFSET			0x6c
#define	PCIE_CMMD_ADDR1_OFFSET			0x70
#define	DMA_FIFO_RST_OFFSET			0x74
#define	DMA_PCIE_FRAM_GAP_OFFSET			0x78
#define	DMA_ACK_TIME_CNT_OFFSET			0x7c
#define	DMA_TX_ALL_FRAME_CNT_OFFSET		0x80
#define	DMA_INT_STAT_OFFSET			0x84

#endif
