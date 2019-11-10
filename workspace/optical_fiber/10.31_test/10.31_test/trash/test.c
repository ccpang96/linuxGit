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
volatile char saveBuffer[SAVE_COUNT + 1][32*4*8192 + 1] = {0};
int memfd, k7fd;
int main(){
gtxTxWithoutData_test(1);// new thread
UINT32 count = 0;
	pcie_init();
	pcie_prepare();
#if (defined(CH_0) || defined(CH_1))
	recvChannelConfig(0x3);
#elif defined(CH_0)
	recvChannelConfig(0x1);
#elif defined(CH_1)
	recvChannelConfig(0x2);
#endif
#if (defined(CH_0) || defined(CH_1))
	waitGtxChannelSync(0x3);
#elif (defined(CH_0))
	waitGtxChannelSync(0x1);
#elif (defined(CH_1))
	waitGtxChannelSync(0x2);
#endif
	while (count < LOOP_COUNT)
	{
		pcie_startDma(firstFlag);
		firstFlag = 0;
		dmaRet = pcie_dmaDone(timeout);
		if(count <= SAVE_COUNT)
		{
			ret = read(k7fd, saveBuffer[count], rsizeNumChar*rcountNum);
		}
		count++;
		pcie_clearInt();
	}
	pcie_stopFpga();
	FILE* fp = fopen("testData.dat", "wb");
	if (fp == NULL){
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
}
