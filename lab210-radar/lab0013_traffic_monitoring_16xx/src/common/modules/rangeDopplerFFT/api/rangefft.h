/**
 *   @file  rangefft.h
 *
 *   @brief
 *      Header file for range FFT
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

#ifndef _RADARDEMO_RANGEFFT_H
#define _RADARDEMO_RANGEFFT_H

/**
 *  @file   rangefft.h
 *  @brief  range FFT header file
 *
 */

//#include <types/types.h>
#include <swpform.h>

#ifdef _TMS320C6X
#ifndef _TMS320C6600 //C674x
#include "dsplib674x.h"
#else
#ifdef CCS
#include <dsplib.h>
#else
#include <ti/dsplib/dsplib.h>
#endif
#endif
#endif

#include <modules/edma/api/edma.h>

#ifdef _WIN32
#include <modules/fft/src/DSPF_sp_fftSPxSP_opt.h>
#endif

//#include <chains/api/RadarDSPCommonDefs.h>

/**
 * \brief  range FFT module parameters
 *
 *  \sa
 *
 */
typedef struct _rangeFFTPara_
{

	float  *pRangeWindow; /**< pointer to pre-calcuted window coeff for range FFT. */
	uint16_t  rangeWinSize; /**< range window size. */
	float     *pDopplerWindow;/**< pointer to pre-calcuted window coeff for Doppler FFT. */
	uint16_t  DopplerWinSize; /**< Doppler window size. */
	uint16_t numAdcSamplePerChirp;/**< number of adc samples per chirp. */
	uint16_t  rangeFFTSize;/**< range FFT size. */
	uint16_t  numChirpPerFrame; /**< number of chirps per frame. */
	uint16_t  startRangeCell; /**< starting index to keep after range FFT. */
	uint16_t  endRangeCell; /**< index of last cell to keep. */
	//float    *pCoeffThales; /**< pointer to coeff table used for Thales. */
	uint16_t  numAntenna; /**< number of antennas. */
	EDMA_OBJ_t *dma_param[EDMA_CHANNELS];/**DMA channel information*/
	uint16_t fftIndex;

}rangeFFTParaConfig_t;


/**
 * \brief  range FFT module error code
 *
 *  \sa
 *
 */
typedef enum
{
	RANGEFFT_OK = 0,
	RANGEFFT_ERROR_MEMORY_ALLOC_FAILED,
	RANGEFFT_ERROR_NOT_SUPPORTED
} rangeFFTErrorCodes;

/**
 *  @brief      create an instance of computeRangeFFT module. *
 *
 *  @param[in]  pRangeFFTParams      Pointer to parameter structure *
 *
 *  @remarks Return to the pointer of the created instance
 */
void *computeRangeFFTCreate(rangeFFTParaConfig_t * pRangeFFTParams);

/**
 *  @brief      Perform range FFT for one frame of input.
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  pDataIn              Pointer to the input Data
 *  @param[out] pDataOut             Pointer to the output Data
 *
 *  @remarks
 */
int32_t computeRangeFFTFrameRun (void *handle, cplx16_t *restrict pDataIn, cplxf_t *restrict pDataOut);


/**
 *  @brief      Perform preprocessing for a chirp including sign extention from 14 bits to 16 bits signed, and remove DC
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  pDataIn              Pointer to the input Data
 *  @param[out] pDataOut             Pointer to the output Data
 *
 *  @remarks
 */
int32_t computeRangeFFTChirpPreprocessing (void *handle, cplx16_t *restrict pDataIn, cplx16_t *restrict pDataOut);


/**
 *  @brief      Perform range FFT for one frame of input, with FFT output as complex floating point
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  pDataIn              Pointer to the input Data
 *  @param[out] pDataOut             Pointer to the output Data
 *
 *  @remarks
 */
int32_t computeRangeFFTChirpRunFloat (void *handle, cplx16_t *restrict pDataIn, cplxf_t *restrict pDataOut);


/**
 *  @brief      Delete the resources of range FFT instance.
 *
 *  @param[in]  handle      Pointer to instance handler.
 *
 */
int32_t computeRangeFFTDelete(void *handle);

/**
 *  @brief      Return the debug data size.
 *
 *  @param[in]  handle      Pointer to instance handler.
 *
 */
int32_t computeRangeFFTDebugQuery(void *handle);

#endif
