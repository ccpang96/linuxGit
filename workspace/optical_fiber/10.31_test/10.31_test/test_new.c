
#include <stdio.h>
#include <linux/types.h>
//#include <asm/irq.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <pthread.h>
#include <sys/mman.h>		//For Mman Operation
//#include <linux/signal.h>
#include <linux/sched.h>
//#include <linux/interrupt.h>

#include "pcie_test.h"
unsigned char *K7_mmapAddr = NULL;
UINT16 rsizeNum = 32;			//note byte: 32*4 = 128(max)
UINT16 rcountNum = 8*4;		//note count*size*4 <= 1MB(driver limites)
volatile char saveBuffer[SAVE_COUNT + 1][4096] = {0};
int memfd, k7fd;
dm_t dm_whole;
//firstFlag=;
unsigned long byteswap(unsigned long int x)
{
   unsigned long int a,b,c,d,z;
   a=(x&0xff);
   a=(a<<24);
   
   b=(x&0xff00);
   b=(b<<8);
   
   c=(x&0xff0000);
   c=(c>>8);
   
   d=(x&0xff000000);
   d=(d>>24);
       
   z=(a|b|c|d);
   return z;
}
void k7_write(unsigned int addr, unsigned int idata)
{        
	*(unsigned int*)(K7_mmapAddr + addr) = byteswap((unsigned int)(idata));                              
}

unsigned int k7_read(unsigned int addr)
{
    unsigned int data;

	data = *(unsigned int*)(K7_mmapAddr + addr); 
	
    //printf("read K7 addr:%x,data:%x,\n",(K7_mmapAddr + addr),data);        

	return byteswap(data); 
}
int gtxChannelIsSync(unsigned int channel)
{
	if ((k7_read(0x160) & channel) == channel)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//GTx channel Sync
void waitGtxChannelSync(unsigned int channel)
{
	while (1)
	{
		if (gtxChannelIsSync(channel) == 1)
		{
			break;
		}
		else
		{
			usleep(10);
		}
	}
}
void pcie_clearInt(void)
{
	k7_write(DMA_INT_CLEAR_OFFSET, 0x1);
}

void pcie_stopFpga(void)
{
	k7_write(DMA_WR_STOP_EN_OFFSET, 0x1);
}
void recvChannelConfig(unsigned int channel)
{
	if ((channel == 0) || (channel > 3)) 
	{
		printf("Please input right channel 1-3\n");
	}
	else
	{
		//add 20191010 double channel
		k7_write(GTX_RX_DATA_CHANNEL_EN_OFFSET, channel);
	}
}

void pcie_prepare(void)
{
	int ret, i = 0, j = 0;
	unsigned int readback;

	UINT8 bAttr = 0;
	UINT8 bEP = 0;
	UINT8 bTD = 0;
	UINT8 bTC = 0;
	UINT8 bType = 0;
	UINT8 bFmt = 2;  	//010:MEM_WR32, 011:MEM_WR64
	UINT8 bFdwbe = 0xf;
	UINT8 bldwbe = 0xf;
	UINT8 bTag = 0;
	UINT16 wRID = 0x0100;
	
	UINT32 tlpsCmd0, tlpsCmd1;
	UINT32 LowerAddr, UpperAddr;

	tlpsCmd0 = ((rsizeNum & 0x3FF) | 			/* tlps[0:9] - DMA TLP size */
				((bAttr & 0x3) << 12) | 	/* tlps[12:13] -*/
				(bEP ? BIT14 : 0) | 		/* tlps[14] enable EP TLP */
				(bTD ? BIT15 : 0) | 		/* tlps[15] enable TD TLP */
				((bTC & 0x7) << 20) | 		/* tlps[20:22] TC TLP */
				((bType & 0x1F) << 24) | 	/* tlps[24:28] Type TLP */
				((bFmt & 0x7) << 29));		/* tlps[29:31] Fmt TLP */

	tlpsCmd1 = ((bFdwbe & 0xF) | 			/* tlps[0:3] - 1st DW BE */
				((bldwbe & 0xF) << 4) | 	/* tlps[4:7] - last DW BE*/
				((bTag & 0xFF) << 8) | 		/* tlps[8:15] -tag */
				((wRID & 0xFFFF) << 16));	/* tlps[16:31] - Requester ID */

	LowerAddr = (UINT32)(dm_whole.phy & 0xffffffff);
	UpperAddr = (UINT32)(dm_whole.phy >> 32);
	printf("hzm\n");
	printf("LowerAddr = %#x, UpperAddr = %#x\n", LowerAddr, UpperAddr);
	printf("hzm zaici liu jihao\n");
	readback = k7_read(VERSION_OFFSET);
	printf("FPGA Version = %#x\n", readback);
	printf("tlpsCmd0 = %#x\n", tlpsCmd0);
	printf("tlpsCmd1 = %#x\n", tlpsCmd1);

	//software reset
	printf("ceshi pcie_prepare1hzm\n");
	for (i = 0; i < 1; i++)
	{
		k7_write(SYSTEM_SOFTWARE_RESET_EN_OFFSET, 1);
		sleep(2);
	}
	
	//reset dma fifo
	k7_write(DMA_FIFO_RST_OFFSET, 0x1);
	usleep(100000);
	
	//soft reset
	for (i = 0; i < 1; i++)
	{
		k7_write(SOFTRESET_OFFSET, 0x1);
		usleep(400000);
	}
	printf("ceshi pcie_prepare2hzm\n");
	//config the tlp frame gap
	k7_write(DMA_PCIE_FRAM_GAP_OFFSET, 0x2);
	k7_write(DEBUG_CONFIG_OFFSET, 0x0); 		//0:press 2:whole speed

	k7_write(AD_SRC_SELECT_OFFSET, 0x0);		//0:AD src 1:64bit press 2:64bit unpress

	//k7_write(0x18c, 0x80000);		//frame format length 0x80000 512KB

	k7_write(GTX_RX_CONTROL_WORD_CNT_OFFSET, 194);

	/* Set lower 32bit of DMA address */
	k7_write(DMA_WR_ADDR_LOW_OFFSET, LowerAddr);
	k7_write(DMA_WR_ADDR_HIGH_OFFSET, UpperAddr);

	/* Set size, traffic class, 64bit enable, upper 8bit of DMA address */
	k7_write(PCIE_CMMD_ADDR0_OFFSET, tlpsCmd0);
	k7_write(PCIE_CMMD_ADDR1_OFFSET, tlpsCmd1);

	/* Set TLP count */
	k7_write(DMA_WR_TLP_PACKET_CNT_OFFSET, rcountNum);
	printf("ceshi pcie_prepare3hzm\n");
}

void pcie_startDma(unsigned char firstFlag)
{
	if (firstFlag == 1)
	{
		k7_write(DMA_WR_START_EN_OFFSET, 0x3);
	}
	else
	{
		k7_write(DMA_WR_START_EN_OFFSET, 0x1);
	}
}
int pcie_init(void)
{
	memfd=open ("/dev/mem",O_RDWR);  

	if (memfd < 0)
	{
		perror ("open error");
		return -1;
	}

	K7_mmapAddr = (unsigned char *)mmap(0,800000,PROT_READ|PROT_WRITE,MAP_SHARED,memfd,K7_PCIE_BASEADDR);
	if (K7_mmapAddr == NULL)
	{
		printf("K7 MMAP ERROR!\n");
		return -1;
	}

	k7fd =open ("/dev/k7",O_RDWR);
	if (k7fd < 0)
	{
		perror ("open k7 error");
		return -1;
	}

	memset(&dm_whole, 0, sizeof(dm_whole));
	ioctl(k7fd, K7_PCIE_GET_DMA, (unsigned long)(&dm_whole));
	printf("dm dispaly......\n");
	printf("hzm zaici liu jihao\n");
	printf("dm.vir=%#lx\n", dm_whole.vir);
	printf("dm.phy=%#x\n", dm_whole.phy);

#if 0
	//yangbt add 20190312
	memset(&dm_whole_bak, 0, sizeof(dm_whole_bak));
	ioctl(k7fd, K7_PCIE_GET_DMA_BAK, (unsigned long)(&dm_whole_bak));
	printf("dm_bak dispaly......\n");
	printf("dm_bak.vir=%#lx\n", dm_whole_bak.vir);
	printf("dm_bak.phy=%#x\n", dm_whole_bak.phy);
#endif
	//testPciePio();

	return 0;
}

int pcie_close(void)
{
	munmap(K7_mmapAddr,800000);
	close(memfd);
	close(k7fd);
}


void gtxTxWithData(unsigned int channel)
{
	int i = 0;
	unsigned short data[2048] = {0};
	unsigned int datax[1024] = {0};
	

	for (i = 0; i < 2048; i++)
	{
		data[i] = 20;
	}
	
	memcpy(datax, data, 4096);
	
	k7_write(GTX_TX_FIFO_RST_ADDR, 1);
	k7_write(GTX_TX_CONTROL_WORD_CNT_OFFSET, 700);
	 
	//k7_write(GTX_TX_DATA_CNT_OFFSET, 4096);
	k7_write(GTX_TX_MODE_OFFSET, 0x00);  //no data, only control word
	//not ok 10c 30c 104 108 308 304 300 100 200
	//ok 004 008 00c 
	for (i = 0; i < 1024; i++)
	{
		k7_write(GTX_TX_FIFO_DATA_ADDR0_OFFSET, datax[i]);
		k7_write(GTX_TX_FIFO_DATA_ADDR1_OFFSET, datax[i]);
	}
	k7_write(GTX_TX_KEY_EN_OFFSET, 0x00c); 
	k7_write(GTX_TX_START_ADDR_OFFSET, channel);
	//printf("gtxTxWithData1hzm\n");
}

void * ThreadProc(void * lpParameter){
	int i = 3;
    	while(1){
       		gtxTxWithData(CH_0);
		i--;
    	}

    }
char pcie_dmaDone(unsigned int timeout)
{
	unsigned int count = 0;

	//usleep(1000);
	while ((k7_read(DMA_INT_STAT_OFFSET)&0x01) != 0x1)
	{
		//usleep(1);
		if (timeout != 0)
		{
			count++;
			if (count > timeout)
			{
				//printf("error: pcie dma time out!\n");
				return -1;		//time out
			}
		}
	}
	return 0;				//finished
}
int main(){
    //unsigned short tmp[2000]={0};
    static unsigned char firstFlag = 1, firstCheckFlag = 1;
    int *a;
    unsigned int timeout = 50000;
    pthread_t pid;  
	pthread_attr_t attr;// what 

    UINT16 rsizeNumChar = rsizeNum*4;
    int count=0;
	pcie_init();
	pcie_prepare();


#if (defined(CH_0) && defined(CH_1))
	recvChannelConfig(0x3);
#elif defined(CH_0)
	recvChannelConfig(0x1);
#elif defined(CH_1)
	recvChannelConfig(0x2);
#endif

	pthread_create(&pid,NULL,&ThreadProc,a );

//主线程在这里同步等待：
#if (defined(CH_0) && defined(CH_1))
	waitGtxChannelSync(0x3);
#elif (defined(CH_0))
	waitGtxChannelSync(0x1);
#elif (defined(CH_1))
	waitGtxChannelSync(0x2);
#endif
	printf("Sync Over!\n");

	pcie_startDma(firstFlag);
	firstFlag = 0;
	pcie_dmaDone(timeout);		
	if(count <= SAVE_COUNT) {
        read(k7fd, saveBuffer[count], rsizeNumChar*rcountNum);
    } 
    count++;
	pcie_clearInt();
	pcie_stopFpga();
    FILE* pfile=fopen("datatest.dat","wb");
    printf("%c\n",saveBuffer[0][0]);
    fwrite(saveBuffer[0],sizeof(char),4096,pfile);
    fclose(pfile);
}
