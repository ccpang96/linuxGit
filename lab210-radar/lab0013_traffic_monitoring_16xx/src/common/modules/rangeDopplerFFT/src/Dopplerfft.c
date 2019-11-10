/**
 *   @file  dopplerfft.h
 *
 *   @brief
 *      Doppler FFT
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
/**
 *  @file   DopplerFFT.c
 *  @brief  Doppler FFT implementation
 *
 */

#include <string.h>
#include <modules/rangeDopplerFFT/api/Dopplerfft.h>
#include <modules/utilities/radarOsal_malloc.h>
#include <modules/fft/src/gen_twiddle_fftf.h>
//#include "fft/src/fft_sp_cmplx_notwid_br.h"

#ifdef ENABLE_INTERRUPT_PROTECTION
#include <ti/sysbios/hal/Hwi.h>
#endif

/**
 * \brief  Doppler FFT module parameters
 *
 *  \sa
 *
 */
typedef struct _DopplerFFTParaInstance_
{

	uint16_t  DopplerLineLen; /**< Doppler line length, could be different from Doppler FFT size. */
	uint16_t  DopplerFFTSize; /**< Doppler FFT size. */
	uint16_t  numDopplerLines; /**< number of Doppler lines. */
	//float    *pCoeffThales; /**< pointer to coeff table used for Thales. */
	float *twiddleCoeff; /**< pointer to twiddle factor. */
	unsigned char *brev; /**< brev for fft. */
	float *pDataInBuf; /**< in buffer to store one Doppler line input with floating type. */
	float *pDataOutBuf;/**< out buffer to store one Doppler line input with floating type. */
	uint16_t  numAntenna; /**< number of antennas. */
	float scale; /**< scale factor based on FFT size. */

	cplxf_t *pPing_in; /**< ping/pnag buffer for EDMA in/out. */
	cplxf_t *pPang_in;
	cplxf_t *pPing_out;
	cplxf_t *pPang_out;

	char *scrachMem;
	uint32_t scratchMemSize;
	EDMA_OBJ_t *dma_param[EDMA_CHANNELS];/**DMA channel information*/


}DopplerFFTParaInstance_t;

extern unsigned char brev[];
extern const float scaleFactor[];
extern const float relaxFactor;
extern char start_FFT_table_index;
extern char end_FFT_table_index;

void computeFFTOneLineFloat(void *handle, cplxf_t *restrict pBufferIn,  cplxf_t *restrict pBufferOut);

void *computeDopplerFFTCreate(DopplerFFTParaConfig_t * pDopplerFFTParams)
{
	DopplerFFTParaInstance_t *inst;
	uint32_t memoryUsed = 0;
	uint32_t fftSize, fftIndex, scratchMemSize, cnt;
	char *scratchMem, complex = 2;

	memoryUsed += sizeof(DopplerFFTParaInstance_t);
	inst = (DopplerFFTParaInstance_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(DopplerFFTParaInstance_t), 8);

	//carry the parameters from configuration parameters
	memcpy(inst, pDopplerFFTParams, sizeof(DopplerFFTParaInstance_t));

	inst->DopplerLineLen = pDopplerFFTParams->DopplerLineLen;
	fftSize = inst->DopplerLineLen;
	fftIndex = 0;
	while(1)
	{
		if(fftSize == 1) break;
		fftSize = fftSize >> 1;
		fftIndex++;
	}
	fftSize = (1<<fftIndex);
	if(fftSize < inst->DopplerLineLen)
	{
		fftIndex++;
		fftSize = fftSize << 1;
	}



	inst->DopplerFFTSize = fftSize;
	inst->numDopplerLines = pDopplerFFTParams->numDopplerLines;
	inst->brev = &(brev[0]);
	inst->numAntenna = pDopplerFFTParams->numAntenna;
	inst->scale = scaleFactor[fftIndex-start_FFT_table_index] * relaxFactor;

	// scratch memory is used for pDataBufIn
	//need to add more to sratchMem if other is needed, for example ping and pang buffer...
	scratchMemSize = complex * fftSize * sizeof(float);
	// scratch memory is used for pDataBufOut
	scratchMemSize = scratchMemSize << 1;
	//request scratch memory for input ping/pang
	scratchMemSize += inst->DopplerLineLen * sizeof(cplxf_t) * inst->numAntenna * 2;
	//request scratch memory for output ping/pang
	scratchMemSize += inst->DopplerFFTSize * sizeof(cplxf_t) * inst->numAntenna * 2;
	//request scratch memory
	scratchMem = radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 1, scratchMemSize, 8);
	inst->scrachMem = scratchMem;
	inst->scratchMemSize = scratchMemSize;
	memoryUsed += scratchMemSize;
    //distribute the sractch memory
	inst->pDataInBuf = (float *) (scratchMem);
	inst->pDataOutBuf = (float *) (((char *) inst->pDataInBuf) + fftSize*complex*sizeof(float));
	inst->pPing_in = (cplxf_t *)(((char *) inst->pDataOutBuf) + fftSize*complex*sizeof(float));
	inst->pPang_in = (cplxf_t *)(inst->pPing_in + inst->DopplerLineLen * inst->numAntenna );
	inst->pPing_out = (cplxf_t *)(inst->pPang_in + inst->DopplerLineLen * inst->numAntenna );
	inst->pPang_out = (cplxf_t *)(inst->pPing_out + inst->DopplerFFTSize * inst->numAntenna );


	//request L2 memory for twiddle factors, NOT scratch
	inst->twiddleCoeff = radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * complex * fftSize*sizeof(float), 8);
	memoryUsed += 2 * complex * fftSize *sizeof(float);
    // generate twiddle factor
	gen_twiddle_fftSP (inst->twiddleCoeff, inst->DopplerFFTSize);

	for (cnt = 0; cnt < EDMA_CHANNELS; cnt ++)
	{
		inst->dma_param[cnt] = pDopplerFFTParams->dma_param[cnt];

	}

	return (void *) inst;
}

void computeFFTOneLineFloat(void *handle, cplxf_t *restrict pBufferIn,  cplxf_t *restrict pBufferOut)
{
	uint16_t i_cell, DopplerFFTSize, DopplerLineLen, i_an;
	float *restrict pDataInBuf, *restrict pDataOutBuf;
	DopplerFFTParaInstance_t *pDopplerFFTParams = (DopplerFFTParaInstance_t *)handle;

	DopplerFFTSize = pDopplerFFTParams->DopplerFFTSize;
	DopplerLineLen = pDopplerFFTParams->DopplerLineLen;

	for (i_an = 0; i_an < pDopplerFFTParams->numAntenna; i_an ++)
	{
		/*convert data to float point*/
		pDataInBuf = pDopplerFFTParams->pDataInBuf;
		//pDataOutBuf = pDopplerFFTParams->pDataOutBuf;
		pDataOutBuf = (float*)pBufferOut;
		for(i_cell = 0; i_cell < DopplerLineLen; i_cell ++)
		{
			(*pDataInBuf) = (float)pBufferIn->real;
			pDataInBuf ++;
			(*pDataInBuf) = (float)pBufferIn->imag;
			pDataInBuf ++;
			pBufferIn ++;
		}

		pDataInBuf = pDopplerFFTParams->pDataInBuf;
#ifdef _WIN32
		DSPF_sp_fftSPxSP_opt(DopplerFFTSize,(float *)pDataInBuf, pDopplerFFTParams->twiddleCoeff, (float *)pDataOutBuf, pDopplerFFTParams->brev , 2, 0, DopplerFFTSize);
#else
		DSPF_sp_fftSPxSP(DopplerFFTSize,(float *)pDataInBuf, pDopplerFFTParams->twiddleCoeff, (float *)pDataOutBuf, pDopplerFFTParams->brev , 2, 0, DopplerFFTSize);
#endif





		//advance the output buffer by Doppler FFT size to prepare for next antenna
		pBufferOut += DopplerFFTSize;
	}



}

/*void computeFFTOneLineFloat(void *handle, cplxf_t *restrict pBufferIn,  cplxf_t *restrict pBufferOut)
{
	uint16_t i_cell, DopplerFFTSize, DopplerFFTSize2, DopplerLineLen, i_an;
	float *restrict pDataInBuf, *restrict pDataOutBuf, *pDataOutF;
	__float2_t *restrict pDataOutFloat;
	__float2_t val;
	DopplerFFTParaInstance_t *pDopplerFFTParams = (DopplerFFTParaInstance_t *)handle;

	DopplerFFTSize = pDopplerFFTParams->DopplerFFTSize;
	DopplerFFTSize2 = DopplerFFTSize >> 1; //half the Doppler FFT size
	DopplerLineLen = pDopplerFFTParams->DopplerLineLen;

	for (i_an = 0; i_an < pDopplerFFTParams->numAntenna; i_an ++)
	{

		pDataInBuf = pDopplerFFTParams->pDataInBuf;
		pDataOutBuf = pDopplerFFTParams->pDataOutBuf;
		for(i_cell = 0; i_cell < DopplerLineLen; i_cell ++)
		{
			(*pDataInBuf) = (float)pBufferIn->real;
			pDataInBuf ++;
			(*pDataInBuf) = (float)pBufferIn->imag;
			pDataInBuf ++;
			pBufferIn ++;
		}

		pDataInBuf = pDopplerFFTParams->pDataInBuf;

		DSPF_sp_fftSPxSP(DopplerFFTSize,(float *)pDataInBuf, pDopplerFFTParams->twiddleCoeff, (float *)pDataOutBuf, pDopplerFFTParams->brev , 2, 0, DopplerFFTSize);

		//write the first half of FFT output into the second half for fftshift
		pDataOutFloat = (__float2_t *)pBufferOut + DopplerFFTSize2;
		pDataOutF = pDataOutBuf;
		//#pragma UNROLL(4)
		for (i_cell = 0; i_cell < DopplerFFTSize2;  i_cell++)
		{

#ifdef QI_INPUT
			val = _ftof2(*pDataOutF, *(pDataOutF+1));

#else
			val = _ftof2(*(pDataOutF+1),*pDataOutF);
#endif
			*pDataOutFloat = val;
			pDataOutFloat ++;
			pDataOutF += 2;

		}

		pDataOutFloat = (__float2_t *)pBufferOut;
		//#pragma UNROLL(4)
		for (i_cell = 0; i_cell < DopplerFFTSize2;  i_cell++)
		{
#ifdef QI_INPUT
			val = _ftof2(*pDataOutF,*(pDataOutF+1));
#else
			val = _ftof2(*(pDataOutF+1),*pDataOutF);
#endif
			*pDataOutFloat = val;
			pDataOutFloat ++;
			pDataOutF += 2;

		}

		//advance the output buffer by Doppler FFT size to prepare for next antenna
		pBufferOut += DopplerFFTSize;
	}



}*/

int32_t computeDopplerFFTRunFloat (void *handle, cplxf_t *restrict pDataIn,  cplxf_t *restrict pDataOut)
{

	uint16_t numDopplerLines;
	cplxf_t *buffer_in_ping, *buffer_in_pong,*buffer_out_ping,*buffer_out_pong, *buffer_in, *buffer_out;
	uint32_t status, lineSize, samplePerLine, i_line;

	DopplerFFTParaInstance_t *pDopplerFFTParams = (DopplerFFTParaInstance_t *)handle;

	/*total number of lines considering num of antennas*/
	numDopplerLines = pDopplerFFTParams->numDopplerLines;
	samplePerLine = pDopplerFFTParams->DopplerLineLen * pDopplerFFTParams->numAntenna;
	lineSize =  samplePerLine * sizeof(cplxf_t);

	buffer_in_ping = pDopplerFFTParams->pPing_in;
	buffer_in_pong = pDopplerFFTParams->pPang_in;
	buffer_out_ping = pDopplerFFTParams->pPing_out;
	buffer_out_pong = pDopplerFFTParams->pPang_out;

	/*Perform Doppler FFT for each line*/
	for(i_line = 0; i_line < numDopplerLines; i_line ++)
	{
		if (0 == i_line)
		{
			status = edma_1DS_transfer( pDopplerFFTParams->dma_param[0],
										(uint32_t)buffer_in_ping,  //dst
										(uint32_t)pDataIn,         //src
										lineSize,                  //acnt
										EDMA_BLOCKING             //wait till the finish of the TRX
									   );

			//schedule the next block
			status = edma_1DS_transfer( pDopplerFFTParams->dma_param[0],
										(uint32_t)buffer_in_pong,  //dst
										(uint32_t)(pDataIn +  samplePerLine),     //src
										lineSize,                  //acnt
										EDMA_NONBLOCKING            //wait till the finish of the TRX
									   );

			 //prepare for the buffer pointers
			buffer_in  = buffer_in_ping;
			buffer_out = buffer_out_pong;

		}
		else
		{
			 if(i_line < (numDopplerLines - 1))
			 {
				 status = edma_1DS_transfer( pDopplerFFTParams->dma_param[0],
											(uint32_t)buffer_in,  //dst
											(uint32_t)(pDataIn + samplePerLine * (i_line+1)),         //src
											lineSize,                  //acnt
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

		//actual FFT processing start from here
		computeFFTOneLineFloat(handle, buffer_in, buffer_out);

		//DMA writing
		if(i_line != 0)
		{
			status |= EDMA3_DRV_waitAndClearTcc(pDopplerFFTParams->dma_param[1]->hEdma, pDopplerFFTParams->dma_param[1]->tccId);

		}


		status = edma_1DS_transfer( pDopplerFFTParams->dma_param[1],
									(uint32_t)(pDataOut + samplePerLine * i_line),    //dst
									(uint32_t)(buffer_out),                            //src
									lineSize,                                          //acnt
									EDMA_NONBLOCKING                                   //wait till the finish of the TRX
								   );
		//check the status of input edma obj
		if(i_line < (numDopplerLines - 1))
		{
			// input edma: make sure that the next block is ready
			status |= EDMA3_DRV_waitAndClearTcc(pDopplerFFTParams->dma_param[0]->hEdma, pDopplerFFTParams->dma_param[0]->tccId);
		}
		else
		{
			//last block: move out the last-block data in the blocking manner and no need to check input edma obj for this case
			//wait till all output is finished
			status |= EDMA3_DRV_waitAndClearTcc(pDopplerFFTParams->dma_param[1]->hEdma, pDopplerFFTParams->dma_param[1]->tccId);
		}

	}
	return DOPPLERFFT_OK;

}


int32_t computeDopplerFFTDelete(void *handle)
{
	uint8_t complex = 2;
	DopplerFFTParaInstance_t *inst = (DopplerFFTParaInstance_t*)handle;

	//radarOsal_memFree(inst->scrachMem, inst->scratchMemSize);
	radarOsal_memFree(inst->twiddleCoeff, 2 * complex * inst->DopplerFFTSize*sizeof(float));
	radarOsal_memFree(handle, sizeof(DopplerFFTParaInstance_t));
	return DOPPLERFFT_OK;
}


int32_t computeDopplerFFTDebugQuery(void *handle)
{
	DopplerFFTParaInstance_t *inst =  (DopplerFFTParaInstance_t *)handle;

	uint32_t numSample;

	numSample= inst->numAntenna*inst->DopplerFFTSize*inst->numDopplerLines*sizeof(cplxf_t);

	return numSample;

}
