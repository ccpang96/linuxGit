/*!
 *  \file   radar_test.c
 *
 *  \brief   Unit test bench for radar signal processing chain.
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <swpform.h>
#include <stdlib.h>
#include <math.h>
#include "edmaTx.h"
#include <chains/RadarReceiverEDMA_mmwSDK/radarProcess.h>
#include <modules/utilities/cycle_measure.h>
#include <modules/utilities/radarOsal_malloc.h>

uint32_t TestStatus;



#define MAXNUMHEAPS (2)
#define L2HEAPSIZE (0x20000)
#define L2SCRATCHSIZE (0x10000)
#define DDRSCRATCHSIZE (0x10)
#define DDRHEAPSIZE (0x600000)
#pragma DATA_SECTION(memHeapL2, ".L2heap")
uint8_t memHeapL2[L2HEAPSIZE];
#pragma DATA_SECTION(ddrScratchMem, ".ddrScratchSect")
uint8_t ddrScratchMem[DDRSCRATCHSIZE];
#pragma DATA_SECTION(ddrHeapMem, ".ddrHeap")
uint8_t ddrHeapMem[DDRHEAPSIZE];
#pragma DATA_SECTION(l2ScratchMem, ".L2ScratchSect")
uint8_t l2ScratchMem[L2SCRATCHSIZE];

radarOsal_heapConfig heapconfig[MAXNUMHEAPS];



#define   LOCALINPUTBUFFERSIZEINBYTES 	(32768)

#define   MAXFFTSIZE1D 	(1024)
#define   MAXFFTSIZE2D 	(128)
#define   MAXNUMRXANT	(8)
#define   MAXNUMFRAMES 	(752)
#define	  MAX_ANG_ERROR (2.0)
#define   MAX_ERROR (0.5)

#define MAX_FILE_NAME    200
#define MAX_LINE_LEN     300


/* static allocation for test vector related buffers*/
#ifndef _WIN32
#pragma DATA_SECTION (dataIn, ".testVec_input")
#endif
cplx16_t       dataIn[MAXNUMRXANT * MAXFFTSIZE2D * MAXFFTSIZE1D];      /**< input test vec, in format of ([rxAnts][nChirpsPerFrame][nSamplesTotPerChirp]).*/

extern int32_t parseSRRDemoCfg(radarProcessConfig_t *, char *);


void main ()
{
    int32_t   i, j, i_frame, i_txAnt, numFrames; //, testError;
	cplx16_t *RESTRICT pDataIn;
	radarProcessOutput_t **pDataOut;
	radarProcessOutput_t *localDataOut;
    RadarDsp_outputBuffCntxt * outBuffCntxt;
	radarProcessConfig_t radarConfig;
	void * radarProcessHandle;
	int32_t *localPingIn, *localPongIn, *localPingOut, *localPongOut;

	//int16_t refNframe;
	//int16_t refNobj;
	//int16_t *refXY;
	//radarProcessOutput_t *inputBuf;
	//float rangeErr, angleErr, speedErr;
	ProcessErrorCodes procErrorCode;

	int32_t *scratchBuffer;


#ifdef _WIN32
	char *testListFileName = "..\\testVectors\\input\\inputTestList.txt";
	char baseDir[MAX_FILE_NAME] = "..\\testVectors\\";
#else
	char *testListFileName = "..\\..\\..\\testVectors\\input\\inputTestList.txt";
	char baseDir[MAX_FILE_NAME] = "..\\..\\..\\testVectors\\";
#endif

	char testIndxStr[MAX_LINE_LEN];
	char inpFileName[MAX_FILE_NAME] = "";
	char paramFileName[MAX_FILE_NAME] = "";
	char outResFileName[MAX_FILE_NAME] = "";
//#ifdef _WIN32
	int32_t numTotal;
//#endif
	//FILE *fpInOut1;
	FILE *fpInOut, *fpOut;

	/*Set up heap and mem osal*/
	{
		startClock();
		/* Osal functions for DSP BIOS*/
	    // heap init
		memset(heapconfig, 0, sizeof(heapconfig));
		heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapType 	= 	RADARMEMOSAL_HEAPTYPE_DDR_CACHED;
		heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr   = 	(int8_t *) &ddrHeapMem[0];
		heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize   = 	DDRHEAPSIZE;
		heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchAddr= 	NULL; 	/* not DDR scratch for TM demo  */
		heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize= 	0; 	/* not DDR scratch for TM demo  */

		heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapType 			= 	RADARMEMOSAL_HEAPTYPE_LL2;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapAddr   		= 	(int8_t *) &memHeapL2[0];
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapSize   		= 	L2HEAPSIZE;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchAddr   	= 	(int8_t *)&l2ScratchMem[0];;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize   	= 	L2SCRATCHSIZE;
		if(radarOsal_memInit(&heapconfig[0], MAXNUMHEAPS) == RADARMEMOSAL_FAIL)
		{
			printf("Error: radarOsal_memInit fail\n");
			exit(1);
		}
	}


	/*read configurations from test vector*/
	{
		FILE *fpTestList;
		
		fpTestList = fopen(testListFileName,"rt");
		assert(fpTestList != NULL);

		fscanf(fpTestList,"%s\n", &testIndxStr);
		strcpy(inpFileName, baseDir);
		strcat(inpFileName,"input\\");
		strcat(inpFileName,testIndxStr);
		strcat(inpFileName,"new.bin");

		/*generate input parameter file path*/
		strcpy(paramFileName, baseDir);
		strcat(paramFileName,"input\\");
		strcat(paramFileName,testIndxStr);
		strcat(paramFileName,".cfg");

		numFrames = parseSRRDemoCfg(&radarConfig, paramFileName);


	    // allocate memory for output data
	    localDataOut = (radarProcessOutput_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, sizeof(radarProcessOutput_t), 8);
	    pDataOut = (radarProcessOutput_t **)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, numFrames * sizeof(radarProcessOutput_t *), 8);
		for (i = 0; i < numFrames; i++)
		{
			pDataOut[i]	=	(radarProcessOutput_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, sizeof(radarProcessOutput_t), 8);
			memset(pDataOut[i], 0, sizeof(radarProcessOutput_t));
		}
		outBuffCntxt =     (RadarDsp_outputBuffCntxt *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, sizeof(RadarDsp_outputBuffCntxt), 8);
	
		/* create radar processing handle */
	    radarProcessHandle = (void *) radarProcessCreate(&radarConfig, &procErrorCode);
	    if (procErrorCode > PROCESS_OK)
	    {
	    	printf("radar process creat error! Exit!");
	    	exit(1);
	    }

	
		outBuffCntxt->numBuff = RadarDsp_outputDataType_MAX;
		outBuffCntxt->elem[RadarDsp_outputDataType_OBJ_DATA].type = RadarDsp_outputDataType_OBJ_DATA;
		outBuffCntxt->elem[RadarDsp_outputDataType_OBJ_DATA].buff = localDataOut;
		outBuffCntxt->elem[RadarDsp_outputDataType_HEAT_MAP].type = RadarDsp_outputDataType_HEAT_MAP;
		outBuffCntxt->elem[RadarDsp_outputDataType_HEAT_MAP].buff = radarConfig.heatMapMem;


		scratchBuffer   =  (int32_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 1, sizeof(int32_t) * MAXFFTSIZE1D * 4, 8);

		/*prepare input TV*/
#if 1//def _WIN32
		/*read binary input data if running on visual studio*/
		fpInOut=fopen(inpFileName, "rb");
		if(fpInOut==NULL)
		{
			printf("reading error!\n");
		}
		else
		{
			printf("reading data...\n");
		}
#endif


		// process each frame
		for (i_frame = 0; i_frame < numFrames; i_frame ++)
		{
			printf("Processing frame %d ...\n", i_frame);
			numTotal = radarConfig.numChirpPerFrame*radarConfig.numAdcSamplePerChirp *radarConfig.numAntenna;
			fread(dataIn,sizeof(cplx16_t),numTotal,fpInOut);
			if(i_frame < 305)
			//	continue;

			pDataIn = (cplx16_t *)dataIn;
			
			/* range processing loop */
			{
				int32_t blockStartIdx = 0;

				localPingIn		=	(int32_t *) &scratchBuffer[0 * MAXFFTSIZE1D];
				localPongIn		=	(int32_t *) &scratchBuffer[1 * MAXFFTSIZE1D]; 
				localPingOut	=	(int32_t *) &scratchBuffer[2 * MAXFFTSIZE1D]; 
				localPongOut	=	(int32_t *) &scratchBuffer[3 * MAXFFTSIZE1D]; 
				for (i = 0; i < (int32_t)radarConfig.numChirpPerFrame; i++)
				{
					for (i_txAnt = 0; i_txAnt < (int32_t)radarConfig.numTxAntenna; i_txAnt++ )
					{
						edmaTransfer(
							(uint32_t *)&pDataIn[blockStartIdx + 0 * radarConfig.numAdcSamplePerChirp], 
							(uint32_t *)localPingIn, 
							radarConfig.numAdcSamplePerChirp);
						for (j = 0; j < (int32_t) radarConfig.numPhyRxAntenna - 1; j++ )
						{
							if((j&1) == 0)
							{
								edmaTransfer(
									(uint32_t *)&pDataIn[blockStartIdx + (j + 1) * radarConfig.numAdcSamplePerChirp], 
									(uint32_t *)localPongIn, 
									radarConfig.numAdcSamplePerChirp);							
								while(checkEdmaReady(PINGFLAG)  == EDMANOTREADY);

								radarRangeProcessRun(radarProcessHandle, (cplx16_t *)localPingIn, (cplx16_t *)localPingOut);

								edmaTranspose((uint32_t *)localPingOut, (uint32_t *)&radarConfig.pFFT1DBuffer[(j + i_txAnt * radarConfig.numPhyRxAntenna) * radarConfig.numChirpPerFrame + i], 
									radarConfig.fftSize1D, 
									0, 
									radarConfig.numChirpPerFrame*radarConfig.numAntenna, 
									1);
							}
							else
							{
								edmaTransfer(
									(uint32_t *)&pDataIn[blockStartIdx + (j + 1) * radarConfig.numAdcSamplePerChirp], 
									(uint32_t *)localPingIn, 
									radarConfig.numAdcSamplePerChirp);							
								while(checkEdmaReady(PONGFLAG)  == EDMANOTREADY);

								radarRangeProcessRun(radarProcessHandle, (cplx16_t *)localPongIn, (cplx16_t *)localPongOut);

								edmaTranspose((uint32_t *)localPongOut, (uint32_t *)&radarConfig.pFFT1DBuffer[(j + i_txAnt * radarConfig.numPhyRxAntenna) * radarConfig.numChirpPerFrame + i], 
									radarConfig.fftSize1D, 
									0, 
									radarConfig.numChirpPerFrame*radarConfig.numAntenna, 
									1);
							}
						}
						while(checkEdmaReady(PONGFLAG)  == EDMANOTREADY);
						radarRangeProcessRun(radarProcessHandle, (cplx16_t *)localPongIn, (cplx16_t *)localPongOut);
						edmaTranspose((uint32_t *)localPongOut, (uint32_t *)&radarConfig.pFFT1DBuffer[(j + i_txAnt * radarConfig.numPhyRxAntenna) * radarConfig.numChirpPerFrame + i], 
							radarConfig.fftSize1D, 
							0, 
							radarConfig.numChirpPerFrame*radarConfig.numAntenna, 
							1);
						if (radarConfig.benchmarkPtr->bufferIdx > radarConfig.benchmarkPtr->bufferLen)
							radarConfig.benchmarkPtr->bufferIdx = 0;
						blockStartIdx += radarConfig.numAdcSamplePerChirp*radarConfig.numPhyRxAntenna;
					}
				}
			}
			/* end of range processing loop */

			/*doppler processing*/
			{
				localPingIn		=	(int32_t *) &scratchBuffer[2 * MAXFFTSIZE1D + 0 * radarConfig.numAntenna * radarConfig.fftSize2D];
				localPongIn		=	(int32_t *) &scratchBuffer[2 * MAXFFTSIZE1D + 1 * radarConfig.numAntenna * radarConfig.fftSize2D]; 
				localPingOut	=	(int32_t *) &scratchBuffer[2 * MAXFFTSIZE1D + 2 * radarConfig.numAntenna * radarConfig.fftSize2D]; 
				localPongOut	=	(int32_t *) &scratchBuffer[2 * MAXFFTSIZE1D + 3 * radarConfig.numAntenna * radarConfig.fftSize2D]; 
				/*get all antennas for one range bin*/
				for(j = 0; j < radarConfig.numAntenna; j++)
					edmaTransfer((uint32_t *)&radarConfig.pFFT1DBuffer[j * radarConfig.numChirpPerFrame], (uint32_t *)&localPingIn[j * radarConfig.fftSize2D], radarConfig.numChirpPerFrame);
				for (i = 1; i < (int32_t) radarConfig.fftSize1D; i++)
				{

					if(i&1)
					{
						edmaTransfer((uint32_t *)&radarConfig.pFFT1DBuffer[i * radarConfig.numAntenna * radarConfig.numChirpPerFrame], (uint32_t *)localPongIn, radarConfig.numChirpPerFrame * radarConfig.numAntenna);
						for(j = 0; j < radarConfig.numAntenna; j++)
							edmaTransfer((uint32_t *)&radarConfig.pFFT1DBuffer[i * radarConfig.numAntenna * radarConfig.numChirpPerFrame + j * radarConfig.numChirpPerFrame], (uint32_t *)&localPongIn[j * radarConfig.fftSize2D], radarConfig.numChirpPerFrame);
						while(checkEdmaReady(PINGFLAG)  == EDMANOTREADY);

						radarDopplerProcessRun(radarProcessHandle, (cplx16_t *) localPingIn, (float *) localPingOut);

						edmaTranspose((uint32_t *)localPingOut, (uint32_t *)&radarConfig.heatMapMem[i-1], radarConfig.fftSize2D, 0, radarConfig.fftSize1D, 1);
					}
					else
					{
						for(j = 0; j < radarConfig.numAntenna; j++)
							edmaTransfer((uint32_t *)&radarConfig.pFFT1DBuffer[i * radarConfig.numAntenna * radarConfig.numChirpPerFrame + j * radarConfig.numChirpPerFrame], (uint32_t *)&localPingIn[j * radarConfig.fftSize2D], radarConfig.numChirpPerFrame);
						while(checkEdmaReady(PONGFLAG)  == EDMANOTREADY);
						radarDopplerProcessRun(radarProcessHandle, (cplx16_t *) localPongIn, (float *) localPongOut);

						edmaTranspose((uint32_t *)localPongOut, (uint32_t *)&radarConfig.heatMapMem[i-1], radarConfig.fftSize2D, 0, radarConfig.fftSize1D, 1);
					}
				}
				while(checkEdmaReady(PONGFLAG)  == EDMANOTREADY);
				radarDopplerProcessRun(radarProcessHandle, (cplx16_t *) localPongIn, (float *) localPongOut);
				edmaTranspose((uint32_t *)localPongOut, (uint32_t *)&radarConfig.heatMapMem[i-1], radarConfig.fftSize2D, 0, radarConfig.fftSize1D, 1);
				if (radarConfig.benchmarkPtr->bufferIdx > radarConfig.benchmarkPtr->bufferLen)
					radarConfig.benchmarkPtr->bufferIdx = 0;
			}
			/*end of doppler processing*/

           /* rest of frame processing*/
			radarFrameProcessRun(radarProcessHandle, (void *) outBuffCntxt);

			memcpy(pDataOut[i_frame], localDataOut, sizeof(radarProcessOutput_t));
		}
		fclose(fpInOut);

		//now save the debug data
		strcpy(outResFileName, baseDir);
		strcat(outResFileName,"output\\");
		strcat(outResFileName,testIndxStr);
		strcat(outResFileName,"Out.bin");
		fpOut		=	fopen(outResFileName, "wb");
		fwrite(&numFrames, sizeof(int16_t), 1, fpOut);
		printf("Saving results to %s ...\n", outResFileName);
		for (i_frame = 0; i_frame < numFrames; i_frame ++)
		{
			radarProcessOutput_t *inputBuf = pDataOut[i_frame];
			i = inputBuf->object_count;
			fwrite(&i, sizeof(int16_t), 1, fpOut);

#ifdef TMDEMOV1
			fwrite(&inputBuf->location[0][0], sizeof(int16_t), i * 3, fpOut);
			fwrite(&inputBuf->velocity[0][0], sizeof(int16_t), i * 3, fpOut);
			i = inputBuf->numCluster;
			fwrite(&i, sizeof(int16_t), 1, fpOut);
			fwrite(inputBuf->report, sizeof(int16_t), i*sizeof(RADARDEMO_clusteringDBscanReport)/sizeof(int16_t), fpOut);
			i = inputBuf->numTracks;
			fwrite(&i, sizeof(int16_t), 1, fpOut);
			for (i = 0; i < inputBuf->numTracks; i++)
			{
				fwrite(&(inputBuf->trackingInfo[i].trackerID), sizeof(int16_t), 1, fpOut);
				fwrite(&(inputBuf->trackingInfo[i].state), sizeof(int16_t), 1, fpOut);
				fwrite(inputBuf->trackingInfo[i].S_hat, sizeof(float), 4, fpOut);
				fwrite(&(inputBuf->trackingInfo[i].xSize), sizeof(float), 1, fpOut);
				fwrite(&(inputBuf->trackingInfo[i].ySize), sizeof(float), 1, fpOut);
			}
#else
			fwrite(&inputBuf->range[0], sizeof(float), i, fpOut);
			fwrite(&inputBuf->angle[0], sizeof(float), i, fpOut);
			fwrite(&inputBuf->doppler[0], sizeof(float), i, fpOut);
			fwrite(&inputBuf->snr[0], sizeof(float), i, fpOut);
#endif
		}
		fclose(fpOut);

#ifdef TMDEMOV1
		//Check results
		testError = 0;
		refXY		=	(int16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, 4 * DOA_OUTPUT_MAXPOINTS*sizeof(int16_t), 8);

		strcpy(outResFileName, baseDir);
		strcat(outResFileName,"refOut\\");
		strcat(outResFileName,testIndxStr);
		strcat(outResFileName,"_cartOut.bin");
		fpInOut1		=	fopen(outResFileName, "rb");
		fread(&refNframe, sizeof(int16_t), 1, fpInOut1);
		if (refNframe != numFrames)
			testError++;
		else
		{
			for (i_frame = 0; i_frame < numFrames; i_frame ++)
			{
				inputBuf = pDataOut[i_frame];

				fread(&refNobj, sizeof(int16_t), 1, fpInOut1);
				if (inputBuf->object_count != refNobj)
				{
					testError++;
					fread(refXY, sizeof(int16_t), 4 * refNobj, fpInOut1);
				}
				else
				{
					fread(refXY, sizeof(int16_t), 4 * refNobj, fpInOut1);
					for (i = 0; i < refNobj; i++)
					{
						rangeErr = fabs(sqrt((float) (inputBuf->location[i][0] * inputBuf->location[i][0] + inputBuf->location[i][1] * inputBuf->location[i][1])) 
									   - sqrt((float) (refXY[4*i + 0] * refXY[4*i + 0] + refXY[4*i + 1] * refXY[4*i + 1])))/256.f;
						angleErr = fabs(atan2( (float)inputBuf->location[i][0], (float)inputBuf->location[i][1])
									-	atan2((float)refXY[4*i + 0], (float)refXY[4*i + 1]));
						speedErr = fabs(sqrt((float) (inputBuf->velocity[i][0] * inputBuf->location[i][0] + inputBuf->velocity[i][1] * inputBuf->location[i][1])) 
									   - sqrt((float) (refXY[4*i + 2] * refXY[4*i + 2] + refXY[4*i + 3] * refXY[4*i + 3])))/256.f;
						if ((rangeErr > MAX_ERROR)
							||(angleErr > MAX_ANG_ERROR)
							||(speedErr > MAX_ERROR))
							testError++;
					}
				}
			}
		}
		TestStatus += testError;
		fclose(fpInOut1);
#endif



#ifndef _WIN32
		{
			radarProcessBenchmarkObj * benchMkPtr = radarConfig.benchmarkPtr;

			strcpy(outResFileName, baseDir);
			strcat(outResFileName,"output\\");
			strcat(outResFileName,testIndxStr);
			strcat(outResFileName,"Cycles.xls");
			fpInOut		=	fopen(outResFileName, "w");


			fprintf(fpInOut, "\tframe\trangeInEDMA\trangeProc\trangeOutEDMA\tdopplerInEDMA\tdopplerProc\tdopplerOutEDMA\tcfarDetection\taoaInEDMA\taoaProc\tdbscan\tdataConv\n");
			for (i_frame = 0; i_frame < numFrames; i_frame ++)
			{
				fprintf(fpInOut, "\t%3d\t%7d\t%7d\t%7d\t%7d\t%7d\t%7d\t%7d\t%7d\t%7d\t%7d\t%7d\n", 
					i_frame, 
					benchMkPtr->buffer[i_frame].cfarDetectionCycles,
					benchMkPtr->buffer[i_frame].aoaCycles,
					benchMkPtr->buffer[i_frame].dbscanCycles,
					benchMkPtr->buffer[i_frame].dataConvCycles
					);
			}
			fclose(fpInOut);
		}
#endif

	    processDelete(radarProcessHandle);

		for (i = 0; i < numFrames; i++)
		{
			radarOsal_memFree(pDataOut[i], sizeof(radarProcessOutput_t));
		}
	    radarOsal_memFree(outBuffCntxt, sizeof(RadarDsp_outputBuffCntxt *));
	    radarOsal_memFree(pDataOut, numFrames * sizeof(radarProcessOutput_t *));
	    radarOsal_memFree(localDataOut, numFrames * sizeof(radarProcessOutput_t *));
	    //radarOsal_memFree(refXY, DOA_OUTPUT_MAXPOINTS*sizeof(int16_t));
	    radarOsal_memFree(scratchBuffer, sizeof(int32_t) * MAXFFTSIZE1D * 4);

		fclose(fpTestList);
		
	    radarOsal_memDeInit();
	}



	if (TestStatus == 0)
	{
		TestStatus	=	0x1651;
		printf("All tests passed!\n");
	}
	else
	{
		TestStatus	=	0xDEAD;
		printf("One or more tests failed!\n");
	}

    return;
}



int32_t parseSRRDemoCfg(radarProcessConfig_t *pParam_s, char *pFileName)
{
	FILE *pFid;
	int32_t i;
	char paramSt[MAX_LINE_LEN];
	char valueSt[MAX_LINE_LEN];
	char sLine[MAX_LINE_LEN];
	char *pParamStr = &paramSt[0];
	char *pValueStr = &valueSt[0];
	//char *pValueStr1 = &valueSt[100];
	char *pSLine = &sLine[0];
	int32_t txChnEn, txEnabled[2], txChID = 0, numFrames;
	float frequencySlopeMHzMicoSec, adcSamplePeriodMicoSec, bandWidth, centerFrequency, chirpInterval;

	pFid = fopen(pFileName,"rt");

	assert(pFid != NULL);


	for ( i = 0; i < 8; i++) //initialize with default
	{
		pParam_s->antPhaseCompCoeff[2 * i]		=	1.f;
		pParam_s->antPhaseCompCoeff[2 * i + 1]	=	0.f;
	}

	while (!feof(pFid))
	{
		fgets(pSLine, MAX_LINE_LEN, pFid);
		sscanf(pSLine,"%s %s",pParamStr, pValueStr);

		/*------------read parameters for range FFT-----------*/

		if (strcmp(pParamStr,"channelCfg") == 0)
		{
			int32_t rxChnEn, cascade;
			sscanf(pSLine,"%s %i %i %i",pParamStr, &rxChnEn, &txChnEn, &cascade);

			pParam_s->numTxAntenna = (uint16_t)_bitc4(txChnEn);
			pParam_s->numPhyRxAntenna = (uint16_t)_bitc4(rxChnEn);
			pParam_s->numAntenna = (uint16_t)_bitc4(rxChnEn) * pParam_s->numTxAntenna;

			if (pParam_s->numTxAntenna > 1)
				pParam_s->mimoModeFlag = 1;
			else
				pParam_s->mimoModeFlag = 0;
		}
		else if (strcmp(pParamStr,"adcCfg") == 0)
		{
			int32_t numAdcBits, adcOutFormat, adcJustification;
			sscanf(pSLine,"%s %i %i %i",pParamStr, &numAdcBits, &adcOutFormat, &adcJustification);
			if (numAdcBits == 2)
				pParam_s->numAdcBitsPerSample = 16;
			else if(numAdcBits == 1)
				pParam_s->numAdcBitsPerSample = 14;
			else if(numAdcBits == 0)
				pParam_s->numAdcBitsPerSample = 12;
			else
			{
				printf("adcCfg: Invalid num ADC bits setting! Exit!");
				exit(1);
			}
			if (adcOutFormat != 1)
			{
				printf("adcCfg: Error: SRR demo Requires complex ADC samples! Exit!");
				exit(1);
			}
			if (adcJustification == 1)
			{
				printf("adcCfg: Error: SRR demo Requires ADC samples being left justified! Exit!");
				exit(1);
			}
		}
		else if (strcmp(pParamStr,"adcbufCfg") == 0)
		{
			int32_t adcOutFormat, sampleSwap, chnInterleave, chirpThresh;
			sscanf(pSLine,"%s %i %i %i %i",pParamStr, &adcOutFormat, &sampleSwap, &chnInterleave, &chirpThresh);

			if (sampleSwap != 1)
			{
				printf("adcbufCfg: Error: SRR demo Requires ADC samples being Q first and I next! Exit!");
				exit(1);
			}
			if (chnInterleave != 1)
			{
				printf("adcbufCfg: Error: SRR demo Requires ADC operating in channel/antenna deinterleaved mode! Exit!");
				exit(1);
			}
		}
		else if (strcmp(pParamStr,"profileCfg") == 0)
		{
			int32_t profileID, startFreq, idleTime, adcStartTime;
			int32_t txOutPower, txPhaseShifter, txStartTime, numAdcSamples;
			int32_t adcSampRate, hpfCornerFreq1, hpfCornerFreq2, rxGain;
			char *freqSlopeConst = pValueStr;
			char *rampEndTime = &pValueStr[100];

			sscanf(pSLine,"%s %i %i %i %i %s %i %i %s %i %i %i %i %i %i",pParamStr, 
				&profileID, &startFreq, &idleTime, &adcStartTime, rampEndTime, 
				&txOutPower, &txPhaseShifter, freqSlopeConst, &txStartTime, &numAdcSamples, 
				&adcSampRate, &hpfCornerFreq1, &hpfCornerFreq2, &rxGain);
			pParam_s->numAdcSamplePerChirp	=	numAdcSamples;
			frequencySlopeMHzMicoSec		=	(float)atof(freqSlopeConst);
			adcSamplePeriodMicoSec			=	(float)(1000.0 / (float) adcSampRate);
			adcSamplePeriodMicoSec			*=	(float)numAdcSamples;
			bandWidth						=	frequencySlopeMHzMicoSec * adcSamplePeriodMicoSec * 1.0e6;
			pParam_s->cfarConfig.rangeRes				=	(3.0e8 / (2.f * bandWidth ));
			centerFrequency					=	(float)startFreq * 1.0e9 + bandWidth * 0.5;
			chirpInterval					=	(float)(((float)atof(rampEndTime) * 100) + (float)(idleTime * 100)) *1.0e-8;
		}
		else if (strcmp(pParamStr,"chirpCfg") == 0)
		{
			int32_t startIdx, endIdx, profileID, startFreq, freqSlopeVar;
			int32_t idleTime, adcStartTime;

			sscanf(pSLine,"%s %i %i %i %i %i %i %i %i",pParamStr, 
				&startIdx, &endIdx, &profileID, &startFreq, &freqSlopeVar, 
				&idleTime, &adcStartTime, &txEnabled[txChID++]);

		}
		else if (strcmp(pParamStr,"frameCfg") == 0)
		{
			int32_t chirpStartIdx, chirpEndIdx, numLoops;
			int32_t framePeriod, triggerSelect, frameTrigDelay;

			sscanf(pSLine,"%s %i %i %i %i %i %i %i",pParamStr, 
				&chirpStartIdx, &chirpEndIdx, &numLoops, &numFrames, 
				&framePeriod, &triggerSelect, &frameTrigDelay);

			pParam_s->numChirpPerFrame	=	numLoops;
			pParam_s->framePeriod		=	framePeriod;
			pParam_s->cfarConfig.velocityRes		=	(3.0e8 / (2.0 *  (float) numLoops * pParam_s->numTxAntenna * centerFrequency * chirpInterval));
		}
		else if (strcmp(pParamStr,"cfarCfg") == 0)
		{
			int32_t detectMethod, cfarDiscardLeft, cfarDiscardRight, refWinSize1, refWinSize2;
			int32_t guardWinSize1, guardWinSize2, thre1, thre2, log2Mag, clRemoval;

			sscanf(pSLine,"%s %i %i %i %i %i %i %i %i %i %i %i %i",pParamStr, 
				&detectMethod, &cfarDiscardLeft, &cfarDiscardRight, &refWinSize1, 
				&refWinSize2, &guardWinSize1, &guardWinSize2, &thre1, &thre2, &log2Mag, &clRemoval);

			pParam_s->cfarConfig.cfarMethod	=	detectMethod;
			if ((detectMethod > 4) || (detectMethod < 1))
			{
				printf("cfarCfg: Nonsupported CFAR method for SRR demo! Exit!");
				exit(1);
			}
			pParam_s->cfarConfig.cfarDiscardLeft	=	cfarDiscardLeft;
			pParam_s->cfarConfig.cfarDiscardRight	=	cfarDiscardRight;
			pParam_s->cfarConfig.refWinSize[0]		=	refWinSize1;
			pParam_s->cfarConfig.refWinSize[1]		=	refWinSize2;
			pParam_s->cfarConfig.guardWinSize[0]	=	guardWinSize1;
			pParam_s->cfarConfig.guardWinSize[1]	=	guardWinSize2;
			pParam_s->cfarConfig.thre				=	0.1f * (float)thre1;
			pParam_s->cfarConfig.dopplerSearchRelThr=	0.1f * (float)thre2;
			pParam_s->cfarConfig.log2MagFlag		=	log2Mag;
			pParam_s->cfarConfig.clRemoval			=	clRemoval;
		}
		else if (strcmp(pParamStr,"doaCfg") == 0)
		{
			int32_t doaMethod, vmaxUnroll, doaGamma, doaSideLobeLevel_dB, doaSearchRange, doaSearchRes, doaVarThr;

			sscanf(pSLine,"%s %i %i %i %i %i %i %i",pParamStr, 
				&doaMethod, &vmaxUnroll, &doaGamma, &doaSideLobeLevel_dB, &doaSearchRange, &doaSearchRes, &doaVarThr);

			pParam_s->doaConfig.doaMethod	=	doaMethod;
			if ((doaMethod != 4) && (doaMethod != 1) && (doaMethod != 3))
			{
				printf("cfarCfg: Nonsupported DoA method for SRR demo! Exit!");
				exit(1);
			}
			pParam_s->doaConfig.vmaxUnrollFlag			=	vmaxUnroll;
			pParam_s->doaConfig.doaGamma				=	(float)doaGamma * 0.001f;
			pParam_s->doaConfig.doaSideLobeLevel_dB	=	doaSideLobeLevel_dB;
			pParam_s->doaConfig.doaSearchRange		=	(float) doaSearchRange * 0.1;
			pParam_s->doaConfig.doaSearchRes			=	(float)doaSearchRes * 0.1f;
			pParam_s->doaConfig.doaVarThr				=	(float) doaVarThr * 0.1f;
		}
		else if (strcmp(pParamStr,"compRangeBiasAndRxChanPhase") == 0)
		{
			char *compCoef;
			//pParam_s->antPhaseCompCoeff

			
			compCoef	=	strtok(pSLine, " ");
			i			=	0;
			compCoef	=	strtok(NULL, " ");  // ignore range compensation for now
			//printf("%s\n", compCoef);

			while (compCoef != NULL)
			{
				compCoef	=	strtok(NULL, " ");
				pParam_s->antPhaseCompCoeff[i]	=	(float)atof(compCoef);
				i++;
				if (i >= 16)
					break;
				//printf("%s\n", compCoef);
			}
		}
#ifdef TMDEMOV1
		else if (strcmp(pParamStr,"dbscanCfg") == 0)
		{
			int32_t dbscan_nAccFrames, dbscan_vfactor, dbscan_epsilon, dbscan_weight, dbscan_minPointsInCluster, dbscan_inputScale, dbscan_coreSelect;

			sscanf(pSLine,"%s %i %i %i %i %i %i %i",pParamStr, &dbscan_coreSelect, 
				&dbscan_nAccFrames, &dbscan_epsilon, &dbscan_weight, &dbscan_vfactor, &dbscan_minPointsInCluster, &dbscan_inputScale);

			pParam_s->dbscanConfig.dbscan_coreSelction		=	dbscan_coreSelect;
			pParam_s->dbscanConfig.dbscan_nAccFrames		=	dbscan_nAccFrames;
			pParam_s->dbscanConfig.dbscan_epsilon		=	(float) dbscan_epsilon * 0.1f;
			pParam_s->dbscanConfig.dbscan_weight			=	(float) dbscan_weight * 0.1f;
			pParam_s->dbscanConfig.dbscan_vfactor			=	(float) dbscan_vfactor * 0.1f;
			pParam_s->dbscanConfig.dbscan_minPointsInCluster	=	dbscan_minPointsInCluster;
			pParam_s->dbscanConfig.dbscan_inputScale			=	dbscan_inputScale;
		}
		else if (strcmp(pParamStr,"trackingCfg") == 0)
		{
			int32_t trackerActiveThreshold, trackerForgetThreshold, trackerAssociationThreshold, measurementNoiseVariance, iirForgetFactor, tracking_coreSelect;

			sscanf(pSLine,"%s %i %i %i %i %i %i %i",pParamStr, &tracking_coreSelect, 
				&trackerActiveThreshold, &trackerForgetThreshold, &trackerAssociationThreshold, &measurementNoiseVariance, &iirForgetFactor);

			pParam_s->trackingConfig.trackercoreSelection			=	tracking_coreSelect;
			pParam_s->trackingConfig.trackerAssociationThreshold	=	(float) trackerAssociationThreshold * 0.1f;
			pParam_s->trackingConfig.measurementNoiseVariance		=	(float) measurementNoiseVariance * 0.1f;
			pParam_s->trackingConfig.trackerActiveThreshold			=	trackerActiveThreshold;
			pParam_s->trackingConfig.trackerForgetThreshold			=	trackerForgetThreshold;
			pParam_s->trackingConfig.iirForgetFactor				=	(float) iirForgetFactor * 0.01f;
		}
#endif
	}
    /* Range window parameters. */
    pParam_s->rangeWinSize     = 16;
    pParam_s->rangeWindow[0]  = 0.0800f;
    pParam_s->rangeWindow[1]  = 0.0894f;
    pParam_s->rangeWindow[2]  = 0.1173f;
    pParam_s->rangeWindow[3]  = 0.1624f;
    pParam_s->rangeWindow[4]  = 0.2231f;
    pParam_s->rangeWindow[5]  = 0.2967f;
    pParam_s->rangeWindow[6]  = 0.3802f;
    pParam_s->rangeWindow[7]  = 0.4703f;
    pParam_s->rangeWindow[8]  = 0.5633f;
    pParam_s->rangeWindow[9]  = 0.6553f;
    pParam_s->rangeWindow[10] = 0.7426f;
    pParam_s->rangeWindow[11] = 0.8216f;
    pParam_s->rangeWindow[12] = 0.8890f;
    pParam_s->rangeWindow[13] = 0.9422f;
    pParam_s->rangeWindow[14] = 0.9789f;
    pParam_s->rangeWindow[15] = 0.9976f;

    /* Doppler window parameters. */
    pParam_s->DopplerWinSize    = 4;
    pParam_s->dopplerWindow[0] = 0.1624f;
    pParam_s->dopplerWindow[1] = 0.4703f;
    pParam_s->dopplerWindow[2] = 0.8216f;
    pParam_s->dopplerWindow[3] = 0.9976f;

	pParam_s->maxNumDetObj			= (uint8_t)MAX_RESOLVED_OBJECTS_PER_FRAME;

#ifdef TMDEMOV1
	pParam_s->dbscanConfig.dbscan_maxClusters	=	CLUSTERING_MAX_CLUSTERS;
	if (DOA_OUTPUT_MAXPOINTS * pParam_s->dbscanConfig.dbscan_nAccFrames < CLUSTERING_MAX_NUMPOINTS)
		pParam_s->dbscanConfig.dbscan_maxPoints	=	DOA_OUTPUT_MAXPOINTS * pParam_s->dbscanConfig.dbscan_nAccFrames;
	else
	{
		printf("CHECK PARAMETER SETTINGS! DOA_OUTPUT_MAXPOINTS * pParam_s->dbscan_nAccFrames > CLUSTERING_MAX_NUMPOINTS!!!\n");
	}
#endif


	//pParam_s->nRxAnt = pParam_s->numAntenna;
	fclose(pFid);
	printf("samples per chirp %d, chirps per frame %d, num of antennas %d, num of frames %d\n",
			pParam_s->numAdcSamplePerChirp, pParam_s->numChirpPerFrame,
			pParam_s->numAntenna, numFrames);

	return(numFrames);
}
