/*
//作者:yangbt
//日期:2018.08.01
//文件功能:实现ioctl函数调用，并操作ifc设备/dev/mem进行读写数据
*/


#ifndef __PCIE_TEST_H
#define __PCIE_TEST_H

#define LOOP_COUNT		6

#define K7_PCIE_BASEADDR	0x50100000			//512KB

#define K7_PCIE_GET_DMA		0x0010  /* clear the interrupt */
#define K7_PCIE_GET_DMA_BAK	0x0020  /* yangbt add 20190312 */

#undef FRAME_DATA_CHECK		//
#undef FRAME_HEAD_CHECK		//
//#define 
#define CH_0 1
#define CH_1 2		//hzm tian jia
#define DATA_CHECK
#define SAVE_COUNT		1

typedef unsigned char		UINT8;
typedef unsigned short		UINT16;
typedef unsigned int			UINT32;
typedef unsigned long long		UINT64;
#define BIT14			  (0x1<<14)
#define BIT15		          (0x1<<15)

typedef struct
{
	unsigned long long vir;
	unsigned long phy;
	int size;
}dm_t;

#define FPGA_BASE_ADDR				0x0000

/* DMA Register Locations (byte offset from BAR 0) */
#define 	VERSION_OFFSET					(FPGA_BASE_ADDR + 0x0000)// 版本号
#define	SOFTRESET_OFFSET					(FPGA_BASE_ADDR + 0x0004)// 软件复位
#define	SYSTEM_SOFTWARE_RESET_CNT_OFFSET			(FPGA_BASE_ADDR + 0x0008)
#define	SYSTEM_SOFTWARE_RESET_EN_OFFSET				(FPGA_BASE_ADDR + 0x000c)
#define	PCIE_IPCORE_RESET_CNT_OFFSET				(FPGA_BASE_ADDR + 0x0010)
#define	PCIE_IPCORE_RESET_EN_OFFSET				(FPGA_BASE_ADDR + 0x0014)
#define	DDR3_IPCORE_RESET_CNT_OFFSET				(FPGA_BASE_ADDR + 0x0018)
#define	DDR3_IPCORE_RESET_EN_OFFSET				(FPGA_BASE_ADDR + 0x001c)
#define	AD_SRC_SELECT_OFFSET					(FPGA_BASE_ADDR + 0x0020)
#define	DDR3_DEPTH_CONF_OFFSET					(FPGA_BASE_ADDR + 0x0024)
#define	DDR3_DEPTH2_CONF_OFFSET					(FPGA_BASE_ADDR + 0x0028)

#define	DMA_WR_START_EN_OFFSET 					(FPGA_BASE_ADDR + 0x0050)
#define	DMA_WR_ADDR_LOW_OFFSET 					(FPGA_BASE_ADDR + 0x0054)
#define	DMA_WR_ADDR_HIGH_OFFSET 				(FPGA_BASE_ADDR + 0x0058)
#define	DMA_WR_TLP_PACKET_CNT_OFFSET 				(FPGA_BASE_ADDR + 0x005c)
#define	DMA_WR_TLP_DATA_LENGTH_OFFSET 				(FPGA_BASE_ADDR + 0x0060)
#define	DMA_WR_STOP_EN_OFFSET 					(FPGA_BASE_ADDR + 0x0064)
#define	DMA_INT_CLEAR_OFFSET 					(FPGA_BASE_ADDR + 0x0068)
#define	PCIE_CMMD_ADDR0_OFFSET 					(FPGA_BASE_ADDR + 0x006c)
#define	PCIE_CMMD_ADDR1_OFFSET 					(FPGA_BASE_ADDR + 0x0070)
#define	DMA_FIFO_RST_OFFSET 					(FPGA_BASE_ADDR + 0x0074)
#define	DMA_PCIE_FRAM_GAP_OFFSET 				(FPGA_BASE_ADDR + 0x0078)
#define	DMA_ACK_TIME_CNT_OFFSET 				(FPGA_BASE_ADDR + 0x007c)
#define	DMA_TX_ALL_FRAME_CNT_OFFSET 				(FPGA_BASE_ADDR + 0x0080)
#define	DMA_INT_STAT_OFFSET 					(FPGA_BASE_ADDR + 0x0084)

#define	DEBUG_CONFIG_OFFSET					(FPGA_BASE_ADDR + 0x017c)

#define GTX_TX_CONTROL_WORD_CNT_OFFSET				(FPGA_BASE_ADDR + 0x0200)
#define GTX_TX_FIFO_DATA_ADDR0_OFFSET				(FPGA_BASE_ADDR + 0x0204)
#define GTX_TX_FIFO_DATA_ADDR1_OFFSET				(FPGA_BASE_ADDR + 0x0208)
#define GTX_TX_START_ADDR_OFFSET				(FPGA_BASE_ADDR + 0x020c)
#define GTX_TX_MODE_OFFSET					(FPGA_BASE_ADDR + 0x0210)
#define GTX_TX_CONSTANT_CNT_OFFSET				(FPGA_BASE_ADDR + 0x0214)
#define GTX_TX_DATA_CNT_OFFSET					(FPGA_BASE_ADDR + 0x0218)
#define	GTX_RX_DATA_CHANNEL_EN_OFFSET				(FPGA_BASE_ADDR + 0x021c)
#define	GTX_RX_CONTROL_WORD_CNT_OFFSET				(FPGA_BASE_ADDR + 0x0220)
#define GTX_TX_KEY_EN_OFFSET					(FPGA_BASE_ADDR + 0x0224)
#define GTX_TX_IDLE_CHAR_FLAG					(FPGA_BASE_ADDR + 0x0228)
#define GTX_TX_SW_RST_ADDR					(FPGA_BASE_ADDR + 0x022c)
#define GTX_TX_FIFO_RST_ADDR					(FPGA_BASE_ADDR + 0x0230)

#endif
