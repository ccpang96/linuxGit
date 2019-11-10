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
#ifndef _RADARDEMO_DOPPLERFFT_H
#define _RADARDEMO_DOPPLERFFT_H


/**
 *  @file   DopplerFFT.h
 *  @brief  Doppler FFT header file
 *
 */

//#include <types/types.h>
#include <swpform.h>
//#include <c6x.h>
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

/**
 * \brief  Doppler FFT module parameters
 *
 *  \sa
 *
 */
typedef struct _DopplerFFTPara_
{

	uint16_t  DopplerLineLen; /**< Doppler line length, could be different from Doppler FFT size. */
	uint16_t  numDopplerLines; /**< number of Doppler lines. */
	//float    *pCoeffThales; /**< pointer to coeff table used for Thales. */
	uint16_t  numAntenna; /**< number of antennas. */
	EDMA_OBJ_t *dma_param[EDMA_CHANNELS];/**DMA channel information*/


}DopplerFFTParaConfig_t;

typedef enum
{
	DOPPLERFFT_OK = 0,
	DOPPLERFFT_ERROR_MEMORY_ALLOC_FAILED,
	DOPPLERFFT_ERROR_NOT_SUPPORTED
} DopplerFFTErrorCodes;


/**
 *  @brief      create an instance of computeDopplerFFT module. *
 *
 *  @param[in]  pRangeFFTParams      Pointer to parameter structure *
 *
 *  @remarks Return to the pointer of the created instance
 */
void *computeDopplerFFTCreate(DopplerFFTParaConfig_t * pDopplerFFTParams);

/**
 *  @brief      Performs Doppler FFT. The input memory will be re-used to store the output to save memory,
 *
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  pDataIn              Pointer to the input Data
 *  @param[out] pDataOut             Pointer to the same address as the input data memory
 *
 *  @remarks
 */

int32_t computeDopplerFFTRunFloat (void *handle, cplxf_t *restrict pDataIn,  cplxf_t *restrict pDataOut);

/**
 *  @brief      Delete the resources of Doppler FFT instance.
 *
 *  @param[in]  handle      Pointer to instance handler.
 *
 */
int32_t computeDopplerFFTDelete(void *handle);

/**
 *  @brief      Return the debug data size.
 *
 *  @param[in]  handle      Pointer to instance handler.
 *
 */
int32_t computeDopplerFFTDebugQuery(void *handle);

#endif
