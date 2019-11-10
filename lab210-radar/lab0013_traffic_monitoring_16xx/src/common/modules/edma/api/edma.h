/**
 *   @file  edma.h
 *
 *   @brief
 *      EDMA functions for TDA3
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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
#ifndef RADARDEMO_EDMA_H_
#define RADARDEMO_EDMA_H_

#if defined(_WIN32) || defined(CCS)
#include <string.h>
#else
#include <src/utils_common/include/utils_dma.h>
#endif


#include <swpform.h>

#define EDMA_BLOCKING                1 //TRUE
#define EDMA_NONBLOCKING             0 //FALSE
#define EDMA_CHANNELS                2

#if defined(_WIN32) || defined(CCS)
#define EDMA_SUCCESS                 1;
#define EDMA3_DRV_SOK                1;
typedef uint32_t EDMA3_DRV_Handle;
typedef uint32_t EDMA3_DRV_PaRAMRegs;
typedef uint8_t EDMA3_DRV_Result ;
#endif


typedef struct
{
    uint32_t edmaChId;
    /**< EDMA CH ID that is used for this EDMA */

    uint32_t tccId;
    /**< EDMA TCC ID that is used for this EDMA */

    //zz: the six bit TCC (transfer complete code)
    uint8_t tcc; // TCC in the opt for this edma instance

    EDMA3_DRV_Handle hEdma;
    /**< Handle to EDMA controller associated with this logical DMA channel */

    EDMA3_DRV_PaRAMRegs *pParamSet;
    /**< Pointer to physical area of PaRAM for this channel */

} EDMA_OBJ_t;

#if defined(_WIN32) || defined(CCS)
extern EDMA3_DRV_Result EDMA3_DRV_waitAndClearTcc(EDMA3_DRV_Handle hEdma, uint32_t tccId);
#endif


typedef struct
{
	EDMA_OBJ_t edma_chn;
	EDMA_OBJ_t edma_link_ping;
	EDMA_OBJ_t edma_link_pong;

} EDMA_PINGPONG_t;

//Declaration

extern EDMA3_DRV_Result edma_chn_create(EDMA_OBJ_t * edma_obj_p);

extern EDMA3_DRV_Result edma_chn_release(EDMA_OBJ_t * edma_obj_p);

extern EDMA3_DRV_Result edma_1DS_transfer(EDMA_OBJ_t *  edma_obj_p,
                                          uint32_t  dst_address,
                                          uint32_t  src_address,
                                          uint32_t  cnt_element,
                                          uint8_t flag_blocking
                                         );

extern EDMA3_DRV_Result edma_2DS_transfer(EDMA_OBJ_t *  edma_obj_p,
                                          uint32_t  address_dst,
                                          uint32_t  address_src,
                                          uint32_t  acnt_element,
                                          uint32_t  bcnt_element,
                                          uint32_t  srcBIdx,
                                          uint32_t  desBIdx,
                                          uint8_t flag_firstTRX
                                         );

extern EDMA3_DRV_Result edma_3DS_transfer(EDMA_OBJ_t  * edma_obj_p,
                                          uint32_t  address_dst,
                                          uint32_t  address_src,
                                          uint32_t  acnt_element,
                                          uint32_t  bcnt_element,
                                          uint32_t  ccnt_element,
                                          uint32_t  srcBIdx,
                                          uint32_t  srcCIdx,
                                          uint32_t  desBIdx,
                                          uint32_t  desCIdx,
                                          uint8_t flag_blocking  // if TRUE, wait till the finish of the TRX
                                         );


#endif /* RADARDEMO_EDMA_H_ */
