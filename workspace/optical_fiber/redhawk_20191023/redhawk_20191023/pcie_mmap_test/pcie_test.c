/*
//作者:
//日期:2018.08.01
//文件功能:实现ioctl函数调用，并操作IFC设备/dev/mem进行读写数据
*/


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

int memfd, k7fd;

unsigned char *K7_mmapAddr = NULL;
char *recvBuffer = NULL;
volatile char saveBuffer[SAVE_COUNT + 1][32*4*8192 + 1] = {0}; //[128 + 1][32*4*8192 + 1]
dm_t dm_whole;
#if 0
dm_t dm_whole_bak;  //yangbt add 20190312
#endif

UINT16 rsizeNum = 32;			//note byte: 32*4 = 128(max)
UINT16 rcountNum = 8192;		//note count*size*4 <= 1MB(driver limites)

unsigned int k7_read(unsigned int addr);
void testPciePio(void);

//大小端转换
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

UINT64 getFrame(UINT8 *buf)
{
	UINT64 value = 0;

	value += (UINT64)((UINT64)buf[4] << 56);
	value += (UINT64)((UINT64)buf[5] << 48);
	value += (UINT64)((UINT64)buf[6] << 40);
	value += (UINT64)((UINT64)buf[7] << 32);

	value += (UINT64)((UINT64)buf[0] << 24);
	value += (UINT64)((UINT64)buf[1] << 16);
	value += (UINT64)((UINT64)buf[2] << 8);
	value += (UINT64)((UINT64)buf[3] << 0);

	return value;
}
//初始化PICE资源j
int pcie_init(void)
{
    //整个物理内存的映射
	memfd=open ("/dev/mem",O_RDWR); //open for both reading and writing  

	if (memfd < 0)
	{
		perror ("open error");
		return -1;
	}
    //为当前进程创建并初始化一个新的线性地址区间
    //prot参数：线性区所包含的页的访问权限
    //MAP_SHARED:页可以由几个进程共享
    //addr:从何处查找一个空闲的区间
    //len:线性地址区间的长度
    //K7_PCIE_BASEADDR:可选的线性地址，内核将该地址作为新线性区从哪里开始的一个标志
    //0：文件内的偏移量
    //800000：要映射文件部分的长度
    //memfd:文件描述符表示要映射的文件
    //一个新建立的内存映射就是一个不包含任何页的线性区
	K7_mmapAddr = (unsigned char *)mmap(0,800000,PROT_READ|PROT_WRITE,MAP_SHARED,memfd,K7_PCIE_BASEADDR); 
	if (K7_mmapAddr == NULL)
	{
		printf("K7 MMAP ERROR!\n");
		return -1;
	}
    //光卡设备
	k7fd =open ("/dev/k7",O_RDWR);
	if (k7fd < 0)
	{
		perror ("open k7 error");
		return -1;
	}
    //dm_t格式数据：应该是动态内存
	memset(&dm_whole, 0, sizeof(dm_whole)); //
	ioctl(k7fd, K7_PCIE_GET_DMA, (unsigned long)(&dm_whole)); //清除中断
	printf("dm dispaly......\n");
	printf("dm.vir=%#lx\n", dm_whole.vir); //虚拟内存
	printf("dm.phy=%#x\n", dm_whole.phy);   //物理内存

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
//在地址偏移为addr的地址处写入idata数据
void k7_write(unsigned int addr, unsigned int idata)
{        
	*(unsigned int*)(K7_mmapAddr + addr) = byteswap((unsigned int)(idata));                              
}
/**
 * @brief:读取K7_mmapAddr起始后addr后的数据，并对其做大小端转换
 * @retrurn:
 */
unsigned int k7_read(unsigned int addr)
{
    unsigned int data;

	data = *(unsigned int*)(K7_mmapAddr + addr); //取该地址处的数据
	
    //printf("read K7 addr:%x,data:%x,\n",(K7_mmapAddr + addr),data);        
    //大小端转换
	return byteswap(data); 
}

int hexdump(char *name, char *buf, int len)
{
	int i, count;
	unsigned int *p;
	count = len / 32;
	count += 1;
	for (i = 0; i < count; i++) {
		p = (unsigned int *)(buf + i * 32);
		printf("mem[0x%04x] 0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,\n",
		       i * 32, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
	}
	return 0;
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

/************************
 *@brief： pcie_prepare()
 *state:   PCIE接收DMA通道准备
 */
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

	LowerAddr = (UINT32)(dm_whole.phy & 0xffffffff); //低地址
	UpperAddr = (UINT32)(dm_whole.phy >> 32);
	printf("LowerAddr = %#x, UpperAddr = %#x\n", LowerAddr, UpperAddr);

	readback = k7_read(VERSION_OFFSET); //用k7_mmapAdddr+VERSION_OFFSET之后，再进行大小端转换版本号
	printf("FPGA Version = %#x\n", readback);
	printf("tlpsCmd0 = %#x\n", tlpsCmd0);
	printf("tlpsCmd1 = %#x\n", tlpsCmd1);

	//software reset
	for (i = 0; i < 1; i++)
	{
		k7_write(SYSTEM_SOFTWARE_RESET_EN_OFFSET, 1); //系统软件复位使能
		sleep(2); //睡眠2ms
	}
	
	//reset dma fifo
	k7_write(DMA_FIFO_RST_OFFSET, 0x1); //使能DMA FIFO复位信号
	usleep(100000); //睡眠100ms
	
	//soft reset :总共就写一次 搞个循环有个锤锤用
	for (i = 0; i < 1; i++)
	{
		k7_write(SOFTRESET_OFFSET, 0x1); //软件复位
		usleep(400000); //睡眠400ms
	}

	//config the tlp frame gap 
    //配置传输脉冲间隔
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

void pcie_changeDmaAddr(UINT32 addr)
{
	k7_write(DMA_WR_ADDR_LOW_OFFSET, addr);
}



//等待pcie通道传输完成
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

//清除DMA完成通道标志
void pcie_clearInt(void)
{
	k7_write(DMA_INT_CLEAR_OFFSET, 0x1);
}


//停止DMA传输
void pcie_stopFpga(void)
{
	k7_write(DMA_WR_STOP_EN_OFFSET, 0x1);
}

void gtxTxWithoutData(unsigned int channel, unsigned short buff[], int len)
{
	int i = 0;
	unsigned int data = 0;
	unsigned int datax[1024] = {0};
	int lenx = (len % 2 == 0)? (len / 2) : (len / 2 + 1);

	memcpy(datax, buff, len*2);
	
	k7_write(GTX_TX_FIFO_RST_ADDR, 1);
	k7_write(GTX_TX_CONTROL_WORD_CNT_OFFSET, 700);  //real send = 700
	k7_write(GTX_TX_MODE_OFFSET, 0x00);  //no data, only control word
	//k7_write(GTX_TX_DATA_CNT_OFFSET, 4096);
	//not ok 10c 30c 104 108 308 304 300 100 200
	//ok 004 008 00c 
	for (i = 0; i < lenx; i++)
	{
		k7_write(GTX_TX_FIFO_DATA_ADDR0_OFFSET, datax[i]);
		k7_write(GTX_TX_FIFO_DATA_ADDR1_OFFSET, datax[i]);
	}
	k7_write(GTX_TX_KEY_EN_OFFSET, 0x00c);  

	k7_write(GTX_TX_START_ADDR_OFFSET, channel);
}

void gtxTxWithoutData_test(unsigned int channel)
{
	int i = 0;
	unsigned short data[200] = {0};
	unsigned int datax[100] = {0};
	
	for (i = 0; i < 200; i++)
	{
		data[i] = 4096;
	}

	memcpy(datax, data, 400);
	
	//k7_write(GTX_TX_FIFO_RST_ADDR, 1);
	k7_write(GTX_TX_CONTROL_WORD_CNT_OFFSET, 194);  //real send = 700
	k7_write(GTX_TX_KEY_EN_OFFSET, 0x00c);  
	k7_write(GTX_TX_DATA_CNT_OFFSET, 4096);
	k7_write(GTX_TX_MODE_OFFSET, 0x00);  //no data, only control word
	//not ok 10c 30c 104 108 308 304 300 100 200
	//ok 004 008 00c 
	for (i = 0; i < 97; i++)
	{
		k7_write(GTX_TX_FIFO_DATA_ADDR0_OFFSET, datax[i]);
		k7_write(GTX_TX_FIFO_DATA_ADDR1_OFFSET, datax[i]);
	}
	
	k7_write(GTX_TX_START_ADDR_OFFSET, channel);
}

void gtxTxWithData(unsigned int channel)
{
	int i = 0;
	unsigned short data[200] = {0};
	unsigned int datax[100] = {0};
	
	for (i = 0; i < 200; i++)
	{
		data[i] = 4096;
	}

	memcpy(datax, data, 400);
	
	//k7_write(GTX_TX_FIFO_RST_ADDR, 1);
	k7_write(GTX_TX_CONTROL_WORD_CNT_OFFSET, 194);
	k7_write(GTX_TX_KEY_EN_OFFSET, 0x00c);  
	k7_write(GTX_TX_DATA_CNT_OFFSET, 4096);
	k7_write(GTX_TX_MODE_OFFSET, 0x01);  //no data, only control word
	//not ok 10c 30c 104 108 308 304 300 100 200
	//ok 004 008 00c 
	for (i = 0; i < 97; i++)
	{
		k7_write(GTX_TX_FIFO_DATA_ADDR0_OFFSET, datax[i]);
		k7_write(GTX_TX_FIFO_DATA_ADDR1_OFFSET, datax[i]);
	}

	k7_write(GTX_TX_START_ADDR_OFFSET, channel);
}


void idleCharKeyConfig(unsigned int key)
{
	if ((key > 3) || (key == 0))
	{
		printf("Please input right idle key!\n");
	}
	else
	{
		k7_write(GTX_TX_IDLE_CHAR_FLAG, key);
	}
}

void gtxTxSwRst(void)
{
	k7_write(GTX_TX_SW_RST_ADDR, 1);
}
//判断channel是否是同步的
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
	while (1) //开启循环
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

void testPciePio(void)
{
	int ret = 0;
	unsigned int i = 0;
	struct timeval startTime, endTime;
	float delta_time = 0.0;

	ret = gettimeofday(&startTime,NULL);
	for (i = 0; i < 1024; i++)
	{
		k7_write(DMA_PCIE_FRAM_GAP_OFFSET, 0x2);
	}
	ret = gettimeofday(&endTime,NULL);
	delta_time = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec)/1000000.0;

	printf("Speed = %5.2fKBps\n", (4.0)/(delta_time));	
}

//主函数
int main(void)
{
	int ret, i = 0, j = 0, k = 0;
	
	UINT16 rsizeNumChar = rsizeNum*4; // 128字节
	UINT32 count = 0;
	UINT8 errCount = 0;
	UINT64 frameId = 0, preFrameId = 0;
	UINT64 data = 0, preData = 0;
	static unsigned char firstFlag = 1, firstCheckFlag = 1;
	static UINT64 firstData = 0x123456789abcdef0;

	UINT64 totalCount = 0, totalTime = 0, ddrData = 0;
	double fpgaTestSpeed = 0.0;

	struct timeval startTime, endTime; //返回从1970年开始走过的秒数以及前1s走过的微妙数
	float delta_time = 0.0;
	unsigned int timeout = 50000;

	unsigned char tempData[5] = {0};
	char tempBuf[65] = {0};
	char fileName[20] = {0};
	unsigned short data1 = 0, data2 = 0, prerData = 0;
	unsigned long dataCount = 0;
	int firstTime = 1;

	char dmaRet = 0;

	memset(&startTime, 0, sizeof(startTime));
	memset(&endTime, 0, sizeof(endTime));

	recvBuffer = (char*)malloc(rcountNum*rsizeNumChar + 1); //8192 * 128 + 1 
	memset(recvBuffer, 0xff, sizeof(recvBuffer)); //全部置为0xff

	pcie_init();
	pcie_prepare();
//这里想表达的意思是如果没有定义通道0和通道1，就使用通道3
#if (defined(CH_0) && defined(CH_1))
	recvChannelConfig(0x3);
#elif defined(CH_0)
	recvChannelConfig(0x1);
#elif defined(CH_1)
	recvChannelConfig(0x2);
#endif

//等待通道同步
#if (defined(CH_0) && defined(CH_1))
	waitGtxChannelSync(0x3); //等待对方通道同步
#elif (defined(CH_0))
	waitGtxChannelSync(0x1);
#elif (defined(CH_1))
	waitGtxChannelSync(0x2);
#endif
    //等待GTX通道同步完成
	printf("GTX Channel Sync Over!\n");

	printf("start to read buffer!\n");
	printf("0x160 = %#x, 0x150 = %#x, 0x154 = %#x, 0x180 = %#x\n", k7_read(0x160), k7_read(0x150), k7_read(0x154), k7_read(0x180));
	ret = gettimeofday(&startTime,NULL);
	while (count < LOOP_COUNT) //接收1024字节
	{
        //是否是第一次接收，如果是第一次接收，firstFlag = True;否则就是False
		pcie_startDma(firstFlag);
		firstFlag = 0;

		/*if ((count % 1024) == 0)
		{
			printf("count = %d, 0x160 = %#x, 0x190 = %#x, 0x1a0 = %#x, 0x1a4 = %#x\n", count, k7_read(0x160), k7_read(0x190), k7_read(0x1a0), k7_read(0x1a4));
		}*/
        //等待pcie通道传输完成 timeout是等待时间
		dmaRet = pcie_dmaDone(timeout); //50ms
		/*if (dmaRet < 0)
		{
			printf("DMA TimeOut, 0x160 = %#x!\n", k7_read(0x160));
			sleep(1);
			pcie_prepare();
			firstFlag = 1;
			continue;
		}*/

		/*if ((k7_read(0x160) & 0xff00) != 0x00)
		{
			printf("DMA Fifo OverFlow, 0x160 = %#x!\n", k7_read(0x160));
			sleep(1);
			pcie_prepare();
			firstFlag = 1;
			continue;
		}*/
		//如果count <= 128字节，那么说明可以接收
		if(count <= SAVE_COUNT) //128字节
		{
			ret = read(k7fd, saveBufper[count], rsizeNumChar*rcountNum); //128 * 8192
		}
		count++;
        
        //清除DMA完成通道标志
		pcie_clearInt();
	}
	ret = gettimeofday(&endTime,NULL); //获取结束的时间

    //停止DMA传输
	pcie_stopFpga();
	//ret = gettimeofday(&endTime,NULL);

	printf("0x160 = %#x, 0x190 = %#x, 0x194 = %#x\n", k7_read(0x160), k7_read(0x190), k7_read(0x194));
	printf("0x1A0 = %#x, 0x1A4 = %#x, 0x1A8 = %#x, 0x1AC = %#x, 0x1B0 = %#x\n", k7_read(0x1A0), k7_read(0x1A4), k7_read(0x1A8), k7_read(0x1AC), k7_read(0x1B0));

    //计算用了多少s
	delta_time = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec)/1000000.0;
	printf("delta_time = %10.7fs!\n", delta_time);
	//速度计算错误
    printf("Speed = %5.2fMBps\n", ((rsizeNumChar*rcountNum*10240.0)/1024.0/1024.0)/(delta_time));
	printf("k7 pcie test okay!\n");

	printf("0x158 = %#x, 0x15c = %#x, 0x164 = %#x, 0x168 = %#x, 0x188 = %#x\n", k7_read(0x158), k7_read(0x15c), k7_read(0x164), k7_read(0x168), k7_read(0x188));
	//count 是用8个字节存储的
    totalCount = (k7_read(0x15c) & 0xffff);
	totalCount = totalCount << 32;
	totalCount += k7_read(0x158);
    //time是用8个字节存储的
	totalTime = k7_read(0x168);
	totalTime = totalTime << 32;
	totalTime += k7_read(0x164);

	fpgaTestSpeed = ((totalCount * 64.0)/1024.0/1024.0)/(totalTime*4.0*0.000000001);
	printf("FPGA Test Speed = %8.3fMBps, DDR data = %uGB\n", fpgaTestSpeed, k7_read(0x188)*64/1024/1024/1024);


    


    /**
     * @brief:在此处打开了一个数据包testData.dat,想把接受的数据写入进去
     */
    
	FILE* fp = fopen("testData.dat", "wb");
	if (fp == NULL)
	{
		printf("File Open Error!\n");
	}
	else
	{
		for (i = 0; i <= SAVE_COUNT; i++)
		{
			fwrite(saveBuffer[i], rsizeNumChar*rcountNum, 1, fp);
		}
		fclose(fp);
	}

#ifdef DATA_CHECK
	char buf[513] = { 0 };
	FILE *fpr = fopen("testData.dat", "rb");
#if (defined(CH_0))
	FILE *fpw1 = fopen("1.dat", "wb");
#endif


#if (defined(CH_1))
	FILE *fpw2 = fopen("2.dat", "wb");
#endif

	while (!feof(fpr))
	{
#if (defined(CH_0))
		fread(buf, 64, 1, fpr);
		if (feof(fpr))
		{
			fclose(fpr);
			fclose(fpw1);
			fpw1 = NULL;
#if (defined(CH_1))
			fclose(fpw2);
			fpw2 = NULL;
#endif
			fpr = NULL;
			break;
		}
		fwrite(buf, 64, 1, fpw1);
#endif

#if (defined(CH_1))
		fread(buf, 64, 1, fpr);
		if (feof(fpr))
		{
			fclose(fpr);
#if (defined(CH_0))
			fclose(fpw1);
			fpw1 = NULL;
#endif
			fclose(fpw2);
			fpw2 = NULL;
			fpr = NULL;
			break;
		}
		fwrite(buf, 64, 1, fpw2);
#endif
	}

#if (defined(CH_0))
	if (fpw1 != NULL)
	{
		fclose(fpw1);
		fpw1 = NULL;
	}
#endif
#if (defined(CH_1))
	if (fpw2 != NULL)
	{
		fclose(fpw2);
		fpw2 = NULL;
	}
#endif
	if (fpr != NULL)
	{
		fclose(fpr);
		fpr = NULL;
	}
	
	FILE *fprt = NULL;
#if ((defined(CH_0)) && (defined(CH_1)))
 	for (i = 0; i < 2; i++)
#elif (defined(CH_0))
	for (i = 0; i < 1; i++)
#elif (defined(CH_1))
	for (i = 1; i < 2; i++)
#endif
	{
		firstTime = 1;
		dataCount = 0;
		errCount = 0;
		sprintf(fileName, "%d.dat", (i+1));
		printf("File Name is %s, dataCount = %d\n", fileName, dataCount);
		fprt = fopen(fileName, "rb");
		if (fprt == NULL)
		{
			return;
		}

		if (firstTime == 1)
		{
			firstTime = 0;
			fread(tempData, 4, 1, fprt);
			printf("First datas = %#x, %#x, %#x, %#x\n", tempData[0], tempData[1], tempData[2], tempData[3]);
			//getchar();
			dataCount++;
			preData = ((tempData[2] << 8) + tempData[3]);
			data1 = ((tempData[0] << 8) + tempData[1]);
			//printf("preData = %#x, data1 = %#x\r\n", preData, data1);
			if ((data1 - preData) != 1)
			{
				if ((data1 == 0x0) && (preData == 0xffff))
				{
					preData = data1;
				}
				else
				{
					printf("数据不连续，data = %#x, preData = %#x, line = %d, count = %d！\r\n", data1, preData, dataCount * 4 / 512, (dataCount * 4) % 512);
					preData = data1;
				}
			}
			else
			{

				preData = data1;
			}
		}

		while (!feof(fprt))
		{
			fread(tempData, 4, 1, fprt);
			dataCount++;
			data2 = ((tempData[2] << 8) + tempData[3]);
			data1 = ((tempData[0] << 8) + tempData[1]);
			//printf("data2 = %#x, data1 = %#x, preData = %#x\n", data2, data1, preData);
			if ((data2 - preData) != 1)
			{
				if ((data2 == 0x0) && (preData == 0xffff))
				{
					preData = data2;
				}
				else
				{
					printf("数据不连续，data = %#x, preData = %#x, line = %d, count = %d！\r\n", data2, preData, dataCount * 4 / 512, (dataCount * 4) % 512);
					preData = data2;
					errCount++;
				}
			}
			else
			{
				preData = data2;
			}

			if ((data1 - preData) != 1)
			{
				if ((data1 == 0x0) && (preData == 0xffff))
				{
					preData = data1;
				}
				else
				{
					printf("数据不连续，data = %#x, preData = %#x, line = %d, count = %d！\r\n", data1, preData, (dataCount * 4) / 512, (dataCount * 4) % 512);
					preData = data1;
					errCount++;
				}
			}
			else
			{
				preData = data1;
			}

			if (errCount >= 10)
			{
				printf("Ch %d data check is Over!\n", i);
				fclose(fprt);
				//fprt = NULL;
				break;
			}

			//getchar();
		}

		if (feof(fprt))
		{
			printf("Ch %d data check is Over!\n", i);
			fclose(fprt);
			fprt = NULL;
		}
	}
	
#endif

	free(recvBuffer);
	pcie_close();
}
