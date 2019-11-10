/**
 *   @file  rangefft.c
 *
 *   @brief
 *      Range FFT
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/ 
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
 */



#include <string.h>
//#include "fft/src/fft_sp_cmplx_notwid_br.h"
#include <modules/rangeDopplerFFT/api/rangefft.h>
#include <modules/utilities/radarOsal_malloc.h>
#include <modules/fft/src/gen_twiddle_fftf.h>
#ifdef PROFILE_ON
#include <stdio.h>
#include <modules/utilities/cycle_measure.h>
#endif
typedef struct _rangeFFTParaInstance_
{

	float  *pRangeWindow; /**< pointer to pre-calcuted window coeff for range FFT. */
	uint16_t  rangeWinSize; /**< range window size. */
	float     *pDopplerWindow;/**< pointer to pre-calcuted window coeff for Doppler FFT. */
	uint16_t  DopplerWinSize; /**< Doppler window size. */
	uint16_t numAdcSamplePerChirp;/**< number of adc samples per chirp. */
	uint16_t  rangeFFTSize;/**< range FFT size. */
	uint16_t  numChirpPerFrame; /**< number of chirps per frame. */
	uint16_t  startRangeCell; /**< starting index to keep after range FFT. */
	uint16_t  endRangeCell; /**< ending index to keep after range FFT. */
	uint16_t  lineIndex; /**< line index of the current range line. */
	//uint16_t  coeffTableInd; /**< table index to find coeffThales. */
	unsigned char *brev; /**< brev for fft. */
	//float    *pCoeffThales; /**< pointer to coeff table used for Thales. */
    float *twiddleCoeff; /**< twiddle factors used for FFT. */
	float scale; /**< scale factor based on FFT size. */
	uint16_t  numAntenna; /**< number of antennas. */
	float *pDataBufIn0; /**< temp buffers used for FFT for 4 antennas. */
	float *pDataBufOut0;
	float *pDataBufIn1;
	float *pDataBufOut1;
	float *pDataBufIn2;
	float *pDataBufOut2;
	float *pDataBufIn3;
	float *pDataBufOut3;
	cplx16_t *pChirpPreprocessed;
	cplx16_t *pPing_in;
	cplx16_t *pPang_in;
	cplxf_t *pPing_out;
	cplxf_t *pPang_out;

	char *scrachMem;
	uint32_t scratchMemSize;
	EDMA_OBJ_t *dma_param[EDMA_CHANNELS];/**DMA channel information*/


}rangeFFTParaInstance_t;

extern unsigned char brev[];
extern const float scaleFactor[];
extern const float relaxFactor;
extern char start_FFT_table_index;
extern char end_FFT_table_index;


void *computeRangeFFTCreate(rangeFFTParaConfig_t *pRangeFFTParams)
{
	rangeFFTParaInstance_t *inst;
	unsigned int memoryUsed = 0, scratchMemSize;
	char *scratchMem, complex = 2;
	unsigned int cnt, fftSize;

	memoryUsed += sizeof(rangeFFTParaInstance_t);
	inst = (rangeFFTParaInstance_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(rangeFFTParaInstance_t), 8);

	//calculate the FFT size based on number of samples per chirp
	inst->numAdcSamplePerChirp = pRangeFFTParams->numAdcSamplePerChirp;
    inst->rangeFFTSize = pRangeFFTParams->rangeFFTSize;
    inst->brev = &(brev[0]);
    inst->scale = scaleFactor[pRangeFFTParams->fftIndex-start_FFT_table_index] * relaxFactor;
	inst->pRangeWindow = pRangeFFTParams->pRangeWindow;
	inst->pDopplerWindow = pRangeFFTParams->pDopplerWindow;
	inst->rangeWinSize = pRangeFFTParams->rangeWinSize;
	inst->DopplerWinSize = pRangeFFTParams->DopplerWinSize;
	inst->numChirpPerFrame = pRangeFFTParams->numChirpPerFrame;
	inst->startRangeCell = pRangeFFTParams->startRangeCell;
	inst->endRangeCell = pRangeFFTParams->endRangeCell;
	inst->numAntenna = pRangeFFTParams->numAntenna;

	 for (cnt = 0; cnt < EDMA_CHANNELS; cnt ++)
	{
		inst->dma_param[cnt] = pRangeFFTParams->dma_param[cnt];

	}



	// scratch memory is used for pDataBufIn for antenna 0,1,2,3
	// need to add more to sratchMem if other is needed
	scratchMemSize = complex * inst->rangeFFTSize * sizeof(float) * inst->numAntenna;
	// scratch memory is used for pDataBufOut for antenna 0,1,2,3, same size as pDataBufIn
	scratchMemSize = scratchMemSize << 1;
	//scratch memory for store a chirp after preprocessing
	scratchMemSize += inst->numAdcSamplePerChirp * sizeof(cplx16_t) * inst->numAntenna;
	//request scratch memory for input ping/pang
	scratchMemSize += inst->numAdcSamplePerChirp * sizeof(cplx16_t) * inst->numAntenna * 2;
	//request scratch memory for output ping/pang
	//scratchMemSize += inst->rangeFFTSize * sizeof(cplx16_t) * inst->numAntenna * 2;
	scratchMemSize += inst->rangeFFTSize * sizeof(cplxf_t) * inst->numAntenna * 2;


	//request scratch memory
	scratchMem = radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 1, scratchMemSize, 8);
	inst->scrachMem = scratchMem;
	inst->scratchMemSize = scratchMemSize;
	memoryUsed += scratchMemSize;
	fftSize = inst->rangeFFTSize;
	inst->pDataBufIn0 = (float *) (scratchMem);
	inst->pDataBufOut0 = inst->pDataBufIn0 + complex * fftSize;
	inst->pDataBufIn1 = inst->pDataBufOut0 + complex * fftSize;
	inst->pDataBufOut1 = inst->pDataBufIn1 + complex * fftSize;
	inst->pDataBufIn2 = inst->pDataBufOut1 + complex * fftSize;
	inst->pDataBufOut2 = inst->pDataBufIn2 + complex * fftSize;
	inst->pDataBufIn3 = inst->pDataBufOut2 + complex * fftSize;
	inst->pDataBufOut3 = inst->pDataBufIn3 + complex * fftSize;
	inst->pChirpPreprocessed = (cplx16_t *)(inst->pDataBufOut3 + complex * fftSize);
	inst->pPing_in = (cplx16_t *)(inst->pChirpPreprocessed + inst->numAdcSamplePerChirp * inst->numAntenna);
	inst->pPang_in = (cplx16_t *)(inst->pPing_in + inst->numAdcSamplePerChirp * inst->numAntenna );
	//inst->pPing_out = (cplx16_t *)(inst->pPang_in + inst->numAdcSamplePerChirp * inst->numAntenna );
	//inst->pPang_out = (cplx16_t *)(inst->pPing_out + fftSize * inst->numAntenna );
	inst->pPing_out = (cplxf_t *)(inst->pPang_in + inst->numAdcSamplePerChirp * inst->numAntenna );
    inst->pPang_out = (cplxf_t *)(inst->pPing_out + fftSize * inst->numAntenna );


	//request L2 memory for twiddle factors, NOT scratch
	inst->twiddleCoeff = radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * complex * fftSize*sizeof(float), 8);
	memoryUsed += 2 * complex * fftSize;
	gen_twiddle_fftSP (inst->twiddleCoeff, inst->rangeFFTSize);

	return (void *) inst;
}


int32_t computeRangeFFTChirpPreprocessing (void *handle,
		                                   cplx16_t *restrict pDataIn,
										   cplx16_t *restrict pDataOut)
{
	int32_t bytes_per_chirp_adc, i_sample, numAdcSamplePerChirp;
	int32_t sumReal1, sumImag1, sumReal2, sumImag2, sumReal3, sumImag3, sumReal4, sumImag4;
	int32_t meanReal1, meanImag1, meanReal2, meanImag2, meanReal3, meanImag3, meanReal4, meanImag4;
    uint8_t *chirpInt8, *basePtr, MSB_value;
    int16_t *chirpInt16;
    float divFactor;

    rangeFFTParaInstance_t *pRangeFFTParams = (rangeFFTParaInstance_t*)handle;
    chirpInt8 = (uint8_t *)pDataOut;
    basePtr = (uint8_t *)pDataIn;
    chirpInt16 = (int16_t *)pDataOut;
    numAdcSamplePerChirp = pRangeFFTParams->numAdcSamplePerChirp;
	bytes_per_chirp_adc = pRangeFFTParams->numAntenna * numAdcSamplePerChirp * sizeof(cplx16_t);
	sumReal1 = 0;
	sumImag1 = 0;
	sumReal2 = 0;
    sumImag2 = 0;
	sumReal3 = 0;
	sumImag3 = 0;
	sumReal4 = 0;
	sumImag4 = 0;
	divFactor = 1.0/numAdcSamplePerChirp;



	for(i_sample = 0; i_sample < bytes_per_chirp_adc; )
    {
		// sign extension for imag part
        chirpInt8[i_sample]   = *(basePtr + i_sample);
        MSB_value  = (uint8_t)(*(basePtr + i_sample + 1));
        if(MSB_value & 0x20)
        {
             chirpInt8[i_sample + 1] = (MSB_value | 0xC0);
        }
        else
           chirpInt8[i_sample + 1] = MSB_value;
        //update pointer by 2 bytes
        i_sample = i_sample + 2;

        // sign extension for real part
        chirpInt8[i_sample]   = *(basePtr + i_sample);
        MSB_value  = (uint8_t)(*(basePtr + i_sample + 1));
        if(MSB_value & 0x20)
        {
            chirpInt8[i_sample + 1] = (MSB_value | 0xC0);
        }
        else
             chirpInt8[i_sample + 1] = MSB_value;
        //update pointer by 2 bytes
        i_sample = i_sample + 2;

	}

	//now compute the DC for each antenna
	for(i_sample = 0; i_sample < numAdcSamplePerChirp; i_sample ++)
	{

	    //add up the complex value of the current chirp to get mean valie for each antenna
        sumImag1 += *chirpInt16;
	    chirpInt16 ++;
	    sumReal1 += *chirpInt16;
	    chirpInt16 ++;

	    sumImag2 += *chirpInt16;
	    chirpInt16 ++;
	    sumReal2 += *chirpInt16;
	    chirpInt16 ++;

	    sumImag3 += *chirpInt16;
        chirpInt16 ++;
        sumReal3 += *chirpInt16;
        chirpInt16 ++;

        sumImag4 += *chirpInt16;
        chirpInt16 ++;
        sumReal4 += *chirpInt16;
        chirpInt16 ++;

	}


	meanImag1 = (int16_t)(sumImag1 * divFactor);
	meanReal1 = (int16_t)(sumReal1 * divFactor);
	meanImag2 = (int16_t)(sumImag2 * divFactor);
	meanReal2 = (int16_t)(sumReal2 * divFactor);
	meanImag3 = (int16_t)(sumImag3 * divFactor);
	meanReal3 = (int16_t)(sumReal3 * divFactor);
	meanImag4 = (int16_t)(sumImag4 * divFactor);
	meanReal4 = (int16_t)(sumReal4 * divFactor);



	//now subtract the DC for each antenna
	chirpInt16 = (int16_t *)pDataOut;
	for(i_sample = 0; i_sample < numAdcSamplePerChirp; i_sample ++)
	{

		*chirpInt16 -= meanImag1;
		chirpInt16 ++;
		*chirpInt16 -= meanReal1;
		chirpInt16 ++;

		*chirpInt16 -= meanImag2;
		chirpInt16 ++;
		*chirpInt16 -= meanReal2;
		chirpInt16 ++;

		*chirpInt16 -= meanImag3;
		chirpInt16 ++;
		*chirpInt16 -= meanReal3;
		chirpInt16 ++;

		*chirpInt16 -= meanImag4;
		chirpInt16 ++;
		*chirpInt16 -= meanReal4;
		chirpInt16 ++;

    }

	return RANGEFFT_OK;

}

int32_t computeRangeFFTFrameRun (void *handle, cplx16_t *restrict pDataIn, cplxf_t *restrict pDataOut)
{


	uint16_t i_lines;
	int16_t numChirpPerFrame;
	rangeFFTParaInstance_t *pRangeFFTParams = (rangeFFTParaInstance_t*)handle;
	cplx16_t *buffer_in_ping, *buffer_in_pong, *buffer_in;
	cplxf_t *buffer_out_ping,*buffer_out_pong, *buffer_out;
	uint32_t bytesPerChirp, status, samplePerChirp;


	numChirpPerFrame = pRangeFFTParams->numChirpPerFrame;
	samplePerChirp = pRangeFFTParams->numAdcSamplePerChirp * pRangeFFTParams->numAntenna;
	bytesPerChirp =  samplePerChirp * sizeof(cplx16_t);
	buffer_in_ping = pRangeFFTParams->pPing_in;
	buffer_in_pong = pRangeFFTParams->pPang_in;
	buffer_out_ping = pRangeFFTParams->pPing_out;
	buffer_out_pong = pRangeFFTParams->pPang_out;



	//initiate a DMA transfer
	//edmaInitiateXfer((uint32_t *)ptrInCur, (uint32_t *)pDataIn, bytesPerChirp, 1, 1, bytesPerChirp, bytesPerChirp, 1, 1, pRangeFFTParams->dma_param[0]);

	for(i_lines = 0; i_lines < numChirpPerFrame; i_lines ++)
	{

		if (0 == i_lines)
		{
			status = edma_2DS_transfer( pRangeFFTParams->dma_param[0],
			                            (uint32_t)buffer_in_ping,  //dst
			                            (uint32_t)pDataIn,       //src
										bytesPerChirp,           //acnt
			                            1,                       //bcnt
										bytesPerChirp,           //srcBIdx
										bytesPerChirp,           //dstBIdx
			                            EDMA_BLOCKING            //wait till the finish of the TRX
			                           );

			//schedule the next block
			status = edma_2DS_transfer( pRangeFFTParams->dma_param[0],
										(uint32_t)buffer_in_pong,  //dst
										(uint32_t)(pDataIn + samplePerChirp),       //src
										bytesPerChirp,           //acnt
										1,                       //bcnt
										bytesPerChirp,           //srcBIdx
										bytesPerChirp,           //dstBIdx
										EDMA_NONBLOCKING            //wait till the finish of the TRX
									   );

			 //prepare for the buffer pointers
			buffer_in  = buffer_in_ping;
			buffer_out = buffer_out_pong;

		}
		else
		{
			 if(i_lines < (numChirpPerFrame - 1))
			 {
				 status = edma_2DS_transfer( pRangeFFTParams->dma_param[0],
											(uint32_t)buffer_in,  //dst
											(uint32_t)(pDataIn+ samplePerChirp * (i_lines + 1)),       //src
											bytesPerChirp,           //acnt
											1,                       //bcnt
											bytesPerChirp,           //srcBIdx
											bytesPerChirp,           //dstBIdx
											EDMA_NONBLOCKING            //wait till the finish of the TRX
										   );


			 }
			 //switch between input ping and pong for the current round
			 if(buffer_in == buffer_in_ping)
			 {
				 buffer_in  = buffer_in_pong;
				 buffer_out = buffer_out_pong;

			 }
			 else
			 {
				 buffer_in  = buffer_in_ping;
				 buffer_out = buffer_out_ping;
			 }

		}

		// chirp processing start here
		pRangeFFTParams->lineIndex = i_lines;

		//use the FFT buffer to store the chirp value after Preprocessing for bit extension and DC removal
		//computeRangeFFTChirpPreprocessing(handle, pDataIn, (cplx16_t *)pRangeFFTParams->pChirpPreprocessed);
		computeRangeFFTChirpPreprocessing(handle, buffer_in, (cplx16_t *)pRangeFFTParams->pChirpPreprocessed);

		/*range FFT for handling 4 antennas!*/
		computeRangeFFTChirpRunFloat(handle, (cplx16_t *)pRangeFFTParams->pChirpPreprocessed, buffer_out);

		//DMA writing
		if(i_lines != 0)
		{
			status |= EDMA3_DRV_waitAndClearTcc(pRangeFFTParams->dma_param[1]->hEdma, pRangeFFTParams->dma_param[1]->tccId);

		}


		status = edma_3DS_transfer( pRangeFFTParams->dma_param[1],
									(uint32_t)(pDataOut + i_lines),    //dst
									(uint32_t)(buffer_out),            //src
									sizeof(cplxf_t),                  //acnt
									pRangeFFTParams->numAntenna,       //bcnt
									pRangeFFTParams->rangeFFTSize,     //ccnt
									sizeof(cplxf_t),                  //srcBIdx
									pRangeFFTParams->numAntenna * sizeof(cplxf_t),                  //srcCIdx
									pRangeFFTParams->numChirpPerFrame *  sizeof(cplxf_t),           //dstBIdx
									pRangeFFTParams->numAntenna * pRangeFFTParams->numChirpPerFrame *  sizeof(cplxf_t),           //dstCIdx
									EDMA_NONBLOCKING            //wait till the finish of the TRX
								   );
		//check the status of input edma obj
		if(i_lines < (numChirpPerFrame - 1))
		{
			// input edma: make sure that the next block is ready
			status |= EDMA3_DRV_waitAndClearTcc(pRangeFFTParams->dma_param[0]->hEdma, pRangeFFTParams->dma_param[0]->tccId);
		}
		else
		{
			//last block: move out the last-block data in the blocking manner and no need to check input edma obj for this case
		    //wait till all output is finished
			status |= EDMA3_DRV_waitAndClearTcc(pRangeFFTParams->dma_param[1]->hEdma, pRangeFFTParams->dma_param[1]->tccId);
		}



		//DMA out result
		//edmaInitiateXfer((uint32_t *)(pDataOut + i_lines), (uint32_t *)ptrOutNext, bytesPerFFTChirp, 1, 1, bytesPerFFTChirp,
				         //pRangeFFTParams->numChirpPerFrame * pRangeFFTParams->numAntenna * sizeof(cplx16_t), 1, 1, pRangeFFTParams->dma_param[1]);


	}



	return RANGEFFT_OK;
}

int32_t computeRangeFFTChirpRunFloat (void *handle, cplx16_t *restrict pDataIn, cplxf_t *restrict pDataOut)
{
	uint16_t i_data, i_sample, numEle, numEle0, numFFTCellKeep, rangeFFTSize, dif_FFTSize_NumSample, numAdcSamplePerChirp, rangeWinSize, numChirpPerFrame, i_win, DopplerWinSize, lineIndex, coeffIndex;
	float *restrict pRangeWindow, *restrict pDopplerWindow, coeffDopplerWindow;
	int64_t *restrict pDataIn_int64, *restrict pDataInEnd;
	float *restrict pDataBufIn0, *restrict pDataBufOut0, *restrict pDataBufIn1, *restrict pDataBufOut1;
	float *restrict pDataBufIn2, *restrict pDataBufOut2, *restrict pDataBufIn3, *restrict pDataBufOut3;
	__float2_t coeff, product, val, val0;
	float *restrict pDataBufInEnd0, *restrict pDataBufInEnd1, *restrict pDataBufInEnd2, *restrict pDataBufInEnd3;
	//uint16_t numAntenna;
	__float2_t *restrict pDataOut_double;
	int32_t offset;
	float rangeWindowSfhit;

	rangeFFTParaInstance_t *pRangeFFTParams = (rangeFFTParaInstance_t*)handle;

#ifdef PROFILE_ON
	int64_t       t_start,  tsc_overhead, CPUCycles;
	startClock();
	t_start = ranClock();
	tsc_overhead = ranClock() - t_start;
#endif
	pDataOut_double = (__float2_t *)pDataOut;
	pDataIn_int64 = (int64_t *)pDataIn;
	//numAntenna = pRangeFFTParams->numAntenna;

	rangeFFTSize = pRangeFFTParams->rangeFFTSize;
	numAdcSamplePerChirp = pRangeFFTParams->numAdcSamplePerChirp;
	numChirpPerFrame = pRangeFFTParams->numChirpPerFrame;
	rangeWinSize = pRangeFFTParams->rangeWinSize;
	//process every 8 samples, so >>3
	numEle0 = numAdcSamplePerChirp-(rangeWinSize<<1);
	numEle = numEle0>>3;

	//numAdcSamplePerChirp-(rangeWinSize<<1) might be integer times of 8
	//if not devidable by 8, always process 1 more round
	if (numEle << 3 != numEle0)
		numEle += 1;


	numFFTCellKeep = pRangeFFTParams->endRangeCell - pRangeFFTParams->startRangeCell + 1;
	//process every 4 samples, so >>2, numFFTCellKeep must be interger times of 4
	numFFTCellKeep >>= 2;

	pDataBufIn0 = pRangeFFTParams->pDataBufIn0;
	pDataBufOut0 = pRangeFFTParams->pDataBufOut0;
	pDataBufIn1 = pRangeFFTParams->pDataBufIn1;
	pDataBufOut1 = pRangeFFTParams->pDataBufOut1;
	pDataBufIn2 = pRangeFFTParams->pDataBufIn2;
	pDataBufOut2 = pRangeFFTParams->pDataBufOut2;
	pDataBufIn3 = pRangeFFTParams->pDataBufIn3;
	pDataBufOut3 = pRangeFFTParams->pDataBufOut3;

	dif_FFTSize_NumSample = rangeFFTSize-numAdcSamplePerChirp;

	offset = (numAdcSamplePerChirp<<1);

	if (dif_FFTSize_NumSample >0)
	{
		/*if fft size is different from number of samples in chirp, zero padding for FFT*/
		memset(pDataBufIn0 + offset, 0, sizeof(float)*2*(dif_FFTSize_NumSample));
		memset(pDataBufIn1 + offset, 0, sizeof(float)*2*(dif_FFTSize_NumSample));
		memset(pDataBufIn2 + offset, 0, sizeof(float)*2*(dif_FFTSize_NumSample));
		memset(pDataBufIn3 + offset, 0, sizeof(float)*2*(dif_FFTSize_NumSample));
	}

	/*convert data to float point*/
	pDataIn_int64 += (rangeWinSize << 1); /*start from the middle, assuming 4 RX channels!!!!*/
	pDataBufIn0 += (rangeWinSize << 1);/*x2 considering complex number*/
	pDataBufIn1 += (rangeWinSize << 1);
	pDataBufIn2 += (rangeWinSize << 1);
	pDataBufIn3 += (rangeWinSize << 1);

#ifdef PROFILE_ON
	t_start = ranClock();
#endif

	for(i_data = 0; i_data < numEle; i_data ++)
	{

		#pragma UNROLL(8)
		for(i_sample = 0; i_sample < 8; i_sample ++)
		{

			//for antenna0
			val0 = _dinthsp(_loll(*pDataIn_int64));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif

			(*pDataBufIn0) = _lof2(val);
			pDataBufIn0 ++;
			(*pDataBufIn0) = _hif2(val);
			pDataBufIn0 ++;

			//for antenna1
			val0 = _dinthsp(_hill(*pDataIn_int64));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif

			(*pDataBufIn1) = _lof2(val);
			pDataBufIn1 ++;
			(*pDataBufIn1) = _hif2(val);
			pDataBufIn1 ++;
			pDataIn_int64 ++;
			//for antenna2
			val0 = _dinthsp(_loll(*pDataIn_int64));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif

			(*pDataBufIn2) = _lof2(val);
			pDataBufIn2 ++;
			(*pDataBufIn2) = _hif2(val);
			pDataBufIn2 ++;
			//for antenna3
			val0 = _dinthsp(_hill(*pDataIn_int64));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif

			(*pDataBufIn3) = _lof2(val);
			pDataBufIn3 ++;
			(*pDataBufIn3) = _hif2(val);
			pDataBufIn3 ++;
			pDataIn_int64 ++;



		}
	}




	/*apply range FFT window*/
	pDataIn_int64 = (int64_t *)pDataIn;
	//pDataInEnd = (int64_t *)pDataIn + (rangeFFTSize<<1) -1;
	pDataInEnd = (int64_t *)pDataIn + (numAdcSamplePerChirp<<1) -1;

	pRangeWindow = pRangeFFTParams->pRangeWindow;
	pDataBufIn0 = pRangeFFTParams->pDataBufIn0;
	pDataBufIn1 = pRangeFFTParams->pDataBufIn1;
	pDataBufIn2 = pRangeFFTParams->pDataBufIn2;
	pDataBufIn3 = pRangeFFTParams->pDataBufIn3;
	/*point to the end of the input buffer*/
	//pDataBufInEnd0 = pRangeFFTParams->pDataBufIn0 + rangeFFTSize -1;
	//pDataBufInEnd1 = pRangeFFTParams->pDataBufIn1 + rangeFFTSize -1;
	//pDataBufInEnd2 = pRangeFFTParams->pDataBufIn2 + rangeFFTSize -1;
	//pDataBufInEnd3 = pRangeFFTParams->pDataBufIn3 + rangeFFTSize -1;
	offset = ((numAdcSamplePerChirp -1)<<1) + 1;
	pDataBufInEnd0 = pRangeFFTParams->pDataBufIn0 + offset;
	pDataBufInEnd1 = pRangeFFTParams->pDataBufIn1 + offset;
	pDataBufInEnd2 = pRangeFFTParams->pDataBufIn2 + offset;
	pDataBufInEnd3 = pRangeFFTParams->pDataBufIn3 + offset;


	for(i_win = 0; i_win < (rangeWinSize>>2); i_win ++)
	{

		//#pragma UNROLL(4)
		for(i_sample = 0; i_sample < 4; i_sample ++)
		{
			rangeWindowSfhit = *pRangeWindow;
			//rangeWindowSfhit = *pRangeWindow;
			coeff=_ftof2(rangeWindowSfhit,rangeWindowSfhit);

			//for antenna 0
			/*for the left part*/
			val0 = _dinthsp(_loll(*pDataIn_int64));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif

			product =_dmpysp(val, coeff);
			(*pDataBufIn0) = _lof2(product);
			pDataBufIn0 ++;
			(*pDataBufIn0) = _hif2(product);
			pDataBufIn0 ++;
			/*for the right part*/
			val0 = _dinthsp(_loll(*pDataInEnd));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif

			product =_dmpysp(val, coeff);
			(*pDataBufInEnd2) = _hif2(product);
			pDataBufInEnd2 --;
			(*pDataBufInEnd2) = _lof2(product);
			pDataBufInEnd2 --;

			//for antenna 1
			/*for the left part*/
			val0 = _dinthsp(_hill(*pDataIn_int64));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif

			product=_dmpysp(val, coeff);
			(*pDataBufIn1) = _lof2(product);
			pDataBufIn1 ++;
			(*pDataBufIn1) = _hif2(product);
			pDataBufIn1 ++;
			/*for the right part*/
			val0 = _dinthsp(_hill(*pDataInEnd));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif

			product =_dmpysp(val, coeff);
			(*pDataBufInEnd3) = _hif2(product);
			pDataBufInEnd3 --;
			(*pDataBufInEnd3) = _lof2(product);
			pDataBufInEnd3 --;
			pDataIn_int64 ++;
			pDataInEnd --;

			//for antenna 2
			/*for the left part*/
			val0 = _dinthsp(_loll(*pDataIn_int64));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif
			product=_dmpysp(val, coeff);
			(*pDataBufIn2) = _lof2(product);
			pDataBufIn2 ++;
			(*pDataBufIn2) = _hif2(product);
			pDataBufIn2 ++;
			/*for the right part*/
			val0 = _dinthsp(_loll(*pDataInEnd));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif

			product =_dmpysp(val, coeff);
			(*pDataBufInEnd0) = _hif2(product);
			pDataBufInEnd0 --;
			(*pDataBufInEnd0) = _lof2(product);
			pDataBufInEnd0 --;

			//for antenna 3
			/*for the left part*/
			val0 = _dinthsp(_hill(*pDataIn_int64));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif
			product=_dmpysp(val, coeff);
			(*pDataBufIn3) = _lof2(product);
			pDataBufIn3 ++;
			(*pDataBufIn3) = _hif2(product);
			pDataBufIn3 ++;
			/*for the right part*/
			val0 = _dinthsp(_hill(*pDataInEnd));
			//adc data comes in as Q and I order, need to swap the low and high part
#ifdef QI_INPUT
			val = _ftof2(_lof2(val0), _hif2(val0));
#else
			val = val0;
#endif

			product =_dmpysp(val, coeff);
			(*pDataBufInEnd1) = _hif2(product);
			pDataBufInEnd1 --;
			(*pDataBufInEnd1) = _lof2(product);
			pDataBufInEnd1 --;
			pDataIn_int64 ++;
			pDataInEnd --;

			pRangeWindow ++;


		}

	}

#ifdef PROFILE_ON
	CPUCycles = ranClock() - t_start - tsc_overhead;
	printf("cycles to do frequency shift and windowing: %ld \n", CPUCycles);
#endif

	/*perform range FFT*/
	pDataBufIn0 = pRangeFFTParams->pDataBufIn0;
	pDataBufIn1 = pRangeFFTParams->pDataBufIn1;
	pDataBufIn2 = pRangeFFTParams->pDataBufIn2;
	pDataBufIn3 = pRangeFFTParams->pDataBufIn3;
#ifdef ENABLE_INTERRUPT_PROTECTION
	Hwi_disable(); // protect against interrupt for fft_sp_cmplx_notwid_br
#endif
	//fft_sp_cmplx_notwid_br((double *)pDataBufIn0,rangeFFTSize,(double *)pDataBufOut0, (pRangeFFTParams->pCoeffThales));
	//fft_sp_cmplx_notwid_br((double *)pDataBufIn1,rangeFFTSize,(double *)pDataBufOut1, (pRangeFFTParams->pCoeffThales));
	//fft_sp_cmplx_notwid_br((double *)pDataBufIn2,rangeFFTSize,(double *)pDataBufOut2, (pRangeFFTParams->pCoeffThales));
	//fft_sp_cmplx_notwid_br((double *)pDataBufIn3,rangeFFTSize,(double *)pDataBufOut3, (pRangeFFTParams->pCoeffThales));
#ifdef _WIN32
	//memset(pDataBufOut0, 0,4096);
	DSPF_sp_fftSPxSP_opt(rangeFFTSize, (float *)pDataBufIn0, (pRangeFFTParams->twiddleCoeff), (float *)pDataBufOut0, pRangeFFTParams->brev, 4, 0, rangeFFTSize);
	DSPF_sp_fftSPxSP_opt(rangeFFTSize, (float *)pDataBufIn1, (pRangeFFTParams->twiddleCoeff), (float *)pDataBufOut1, pRangeFFTParams->brev, 4, 0, rangeFFTSize);
	DSPF_sp_fftSPxSP_opt(rangeFFTSize, (float *)pDataBufIn2, (pRangeFFTParams->twiddleCoeff), (float *)pDataBufOut2, pRangeFFTParams->brev, 4, 0, rangeFFTSize);
	DSPF_sp_fftSPxSP_opt(rangeFFTSize, (float *)pDataBufIn3, (pRangeFFTParams->twiddleCoeff), (float *)pDataBufOut3, pRangeFFTParams->brev, 4, 0, rangeFFTSize);
#else
#ifdef PROFILE_ON
	t_start = ranClock();
#endif
	DSPF_sp_fftSPxSP(rangeFFTSize, (float *)pDataBufIn0, (pRangeFFTParams->twiddleCoeff), (float *)pDataBufOut0, pRangeFFTParams->brev, 4, 0, rangeFFTSize);
#ifdef PROFILE_ON
	CPUCycles = ranClock() - t_start - tsc_overhead;
	printf("cycles for one chirp range FFT: %ld \n", CPUCycles);
#endif
	DSPF_sp_fftSPxSP(rangeFFTSize, (float *)pDataBufIn1, (pRangeFFTParams->twiddleCoeff), (float *)pDataBufOut1, pRangeFFTParams->brev, 4, 0, rangeFFTSize);
	DSPF_sp_fftSPxSP(rangeFFTSize, (float *)pDataBufIn2, (pRangeFFTParams->twiddleCoeff), (float *)pDataBufOut2, pRangeFFTParams->brev, 4, 0, rangeFFTSize);
	DSPF_sp_fftSPxSP(rangeFFTSize, (float *)pDataBufIn3, (pRangeFFTParams->twiddleCoeff), (float *)pDataBufOut3, pRangeFFTParams->brev, 4, 0, rangeFFTSize);
#endif
#ifdef ENABLE_INTERRUPT_PROTECTION
	Hwi_enable();
#endif


#ifdef PROFILE_ON
	t_start = ranClock();
#endif
	/*apply Doppler window for the first and last DopplerWinSize lines according to lineIndex*/
	lineIndex = pRangeFFTParams->lineIndex;
	DopplerWinSize = pRangeFFTParams->DopplerWinSize;
	if ((lineIndex < DopplerWinSize) | (lineIndex > numChirpPerFrame-1-DopplerWinSize))
	{
		pDopplerWindow = pRangeFFTParams->pDopplerWindow;
		coeffIndex= (lineIndex>DopplerWinSize)? (numChirpPerFrame-1-lineIndex) : (lineIndex);
		coeffDopplerWindow = pDopplerWindow[coeffIndex];
		coeff=_ftof2(coeffDopplerWindow,coeffDopplerWindow);
	}
	else
	{
		/*apply scaling factor for other bins*/
		coeffDopplerWindow = 1.0;
		coeff = _ftof2(coeffDopplerWindow,coeffDopplerWindow);
	}
	/*only keep range cells within [startRangeCell endRangeCell]*/
	pDataBufOut0 += (pRangeFFTParams->startRangeCell<<1);
	pDataBufOut1 += (pRangeFFTParams->startRangeCell<<1);
	pDataBufOut2 += (pRangeFFTParams->startRangeCell<<1);
	pDataBufOut3 += (pRangeFFTParams->startRangeCell<<1);
	for(i_win = 0; i_win < numFFTCellKeep; i_win ++)
	{
		#pragma UNROLL(4)
		for(i_sample = 0; i_sample < 4; i_sample ++)
		{
			//for antenna0
			val = _ftof2(*pDataBufOut0,*(pDataBufOut0+1)); /*check this line!!!!!!!!!!!!!!*/
			*pDataOut_double  = _dmpysp(val, coeff);
			//pDataOut_double += pRangeFFTParams->numChirpPerFrame;
			pDataOut_double ++;
			pDataBufOut0 += 2;



			//for antenna1
			val = _ftof2(*pDataBufOut1, *(pDataBufOut1+1));
			*pDataOut_double = _dmpysp(val, coeff);
			//pDataOut_double += pRangeFFTParams->numChirpPerFrame;
			pDataOut_double ++;
			pDataBufOut1 += 2;


			//for antenna2
			val = _ftof2(*pDataBufOut2, *(pDataBufOut2+1));
			*pDataOut_double = _dmpysp(val, coeff);
			//pDataOut_double += pRangeFFTParams->numChirpPerFrame;
			pDataOut_double ++;
			pDataBufOut2 += 2;


			//for antenna3
			val = _ftof2(*pDataBufOut3, *(pDataBufOut3+1));
			*pDataOut_double = _dmpysp(val, coeff);
			//pDataOut_double += pRangeFFTParams->numChirpPerFrame;
			pDataOut_double ++;
			pDataBufOut3 += 2;


		}
	}
#ifdef PROFILE_ON
	CPUCycles = ranClock() - t_start - tsc_overhead;
	printf("cycles for save range FFT out and transpose: %ld \n", CPUCycles);
#endif
	return RANGEFFT_OK;

}





int32_t computeRangeFFTDelete(void *handle)
{
	uint8_t complex = 2;
	rangeFFTParaInstance_t *inst = (rangeFFTParaInstance_t*)handle;

	//radarOsal_memFree(inst->scrachMem, inst->scratchMemSize);
	radarOsal_memFree(inst->twiddleCoeff, 2 * complex * inst->rangeFFTSize*sizeof(float));
	radarOsal_memFree(handle, sizeof(rangeFFTParaInstance_t));
	return RANGEFFT_OK;
}



int32_t computeRangeFFTDebugQuery(void *handle)
{
	rangeFFTParaInstance_t *inst =  (rangeFFTParaInstance_t *)handle;

	uint32_t numSample;

	numSample= inst->numAntenna*inst->rangeFFTSize*inst->numChirpPerFrame*sizeof(cplxf_t);

	return numSample;

}
