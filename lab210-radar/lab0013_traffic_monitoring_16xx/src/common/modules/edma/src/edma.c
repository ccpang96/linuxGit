/**
 *   @file  edma.c
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


#include <modules/edma/api/edma.h>

//extern volatile short irqRaised1 = 0;
//
//extern void callback1 (uint32_t tcc, EDMA3_RM_TccStatus status, void *appData);

#if defined(_WIN32) || defined(CCS)


EDMA3_DRV_Result EDMA3_DRV_waitAndClearTcc(EDMA3_DRV_Handle hEdma, uint32_t tccId)
{
	return EDMA_SUCCESS;
}


EDMA3_DRV_Result edma_chn_create(EDMA_OBJ_t * edma_obj_p)
{
	return EDMA_SUCCESS;

}

EDMA3_DRV_Result edma_1DS_transfer(EDMA_OBJ_t *   edma_obj_p,
		                           uint32_t   address_dst,
		                           uint32_t   address_src,
		                           uint32_t   cnt_element,
		                           uint8_t  flag_blocking
		                          )
{
	memcpy((uint32_t*)address_dst, (uint32_t*)address_src, cnt_element);

	return EDMA_SUCCESS;
}

EDMA3_DRV_Result edma_2DS_transfer(EDMA_OBJ_t *  edma_obj_p,
		                           uint32_t  address_dst,
		                           uint32_t  address_src,
		                           uint32_t  acnt_element,
		                           uint32_t  bcnt_element,
		                           uint32_t  srcBIdx,
		                           uint32_t  desBIdx,
		                           uint8_t flag_blocking  // if TRUE, wait till the finish of the TRX
		                          )
{
	uint32_t i;

	for (i = 0; i < bcnt_element; i++)
	{
		memcpy((uint32_t*)address_dst, (uint32_t*)address_src, acnt_element);
		address_src += srcBIdx;
		address_dst += desBIdx;

	}	

	return EDMA_SUCCESS;
}


EDMA3_DRV_Result edma_3DS_transfer(EDMA_OBJ_t  * edma_obj_p,
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
                                  )
{
	uint32_t i, j;

	for (i = 0; i < ccnt_element; i++)
	{
		for (j = 0; j < bcnt_element; j++)
		{
			memcpy((uint32_t*)address_dst, (uint32_t*)address_src, acnt_element);
			address_src += srcBIdx;
			address_dst += desBIdx;

		}

	}

	return EDMA_SUCCESS;

}
#else

//prepare the edma channel
EDMA3_DRV_Result edma_chn_create(EDMA_OBJ_t * edma_obj_p)
{
	UInt32 paramPhyAddr;
	EDMA3_DRV_Result edma3Result    = EDMA3_DRV_SOK;

	edma_obj_p->hEdma               = Utils_dmaGetEdma3Hndl(UTILS_DMA_LOCAL_EDMA_INST_ID);

    ///////////////////////////////////////////////////////////////////////
    Vps_printf("z: edma_chn_create: edma_obj_p->hEdma = %d",
                   edma_obj_p->hEdma);
    ///////////////////////////////////////////////////////////////////////

	UTILS_assert(edma_obj_p->hEdma != NULL);

	edma_obj_p->tccId               = EDMA3_DRV_TCC_ANY;
	edma_obj_p->edmaChId            = EDMA3_DRV_DMA_CHANNEL_ANY;

    edma3Result                     = EDMA3_DRV_requestChannel( edma_obj_p->hEdma,
                                                               (uint32_t*)(&(edma_obj_p->edmaChId)),
                                                               (uint32_t*)(&(edma_obj_p->tccId)),
                                                               (EDMA3_RM_EventQueue)0,
                                                                NULL, NULL
                                                              );
                                                               //(void *)(&(edma_obj_p)));
    UTILS_assert (edma3Result == EDMA3_DRV_SOK);

//    edma_obj_p->tcc = (uint8_t)(edma_obj_p->tccId) & 0x3F;
//
////    edma3Result = EDMA3_DRV_clearErrorBits(edma_obj_p->hEdma,
////    		                               edma_obj_p->edmaChId);
////
////    UTILS_assert (edma3Result == EDMA3_DRV_SOK);

    edma3Result = EDMA3_DRV_getPaRAMPhyAddr(edma_obj_p->hEdma,
    		                                edma_obj_p->edmaChId,
                                            &paramPhyAddr);


    edma_obj_p->pParamSet = (EDMA3_DRV_PaRAMRegs *)paramPhyAddr;


    //zz: test section
    //////////////////////////////////////////////////////////////////////////////////////
    Vps_printf("z: edma_chn_create: allocated LINK (TCC) = %d (%d), pRaramSet = 0x%p\n",
        	    edma_obj_p->edmaChId, edma_obj_p->tccId,  edma_obj_p->pParamSet);
    //////////////////////////////////////////////////////////////////////////////////////

    return edma3Result;
}




//zz: edma 1D simple (continuous frame)
EDMA3_DRV_Result edma_1DS_transfer(EDMA_OBJ_t *   edma_obj_p,
		                           uint32_t   address_dst,
		                           uint32_t   address_src,
		                           uint32_t   cnt_element,
		                           uint8_t  flag_blocking
		                          )
{
	uint32_t     opt;
	//unsigned short   tccStatus = ((unsigned short)edma_obj_p->tccId) & 0x3F;
	EDMA3_DRV_Result status    = EDMA3_DRV_SOK;

	opt = (((edma_obj_p->tccId            << EDMA3_CCRL_OPT_TCC_SHIFT) & EDMA3_CCRL_OPT_TCC_MASK) |
		   (EDMA3_CCRL_OPT_TCINTEN_ENABLE << EDMA3_CCRL_OPT_TCINTEN_SHIFT)                        |
		   (EDMA3_CCRL_OPT_SYNCDIM_ABSYNC << EDMA3_CCRL_OPT_SYNCDIM_SHIFT)
		   );

	edma_obj_p->pParamSet->opt      = opt;
	edma_obj_p->pParamSet->destAddr = address_dst;
	edma_obj_p->pParamSet->srcAddr  = address_src;
	edma_obj_p->pParamSet->aCnt     = cnt_element;
	edma_obj_p->pParamSet->bCnt     = 1;
	edma_obj_p->pParamSet->cCnt     = 1;
	edma_obj_p->pParamSet->linkAddr = 0xFFFF;

	//zz: in the case of first TRX, since there is no previous TRX, so the returned tccStatus is 0 (FALSE)
	//    because there is no previous TRX, so the TCC is not written into the INT register for comparison

	//if(!flag_firstTRX) // check whether the previous TRX is completed
	//{
	    //Vps_printf("z: edma_1DS_transfer #1: status = %d, tccStatus = %d",
		//     	    status, tccStatus);

	//status = EDMA3_DRV_checkAndClearTcc(edma_obj_p->hEdma,
    //	                                edma_obj_p->tccId,
    //                                   &tccStatus);

     //   UTILS_assert((status == EDMA3_DRV_SOK) && (tccStatus == TRUE));

	//UTILS_assert(status == EDMA3_DRV_SOK);
		//status = EDMA3_DRV_waitAndClearTcc(edma_obj_p->hEdma, edma_obj_p->tccId);

        //Vps_printf("z: edma_1DS_transfer #2: status = %d, tccStatus = %d",
        //			    status, tccStatus);
	//}


    /*
	status |= EDMA3_DRV_checkAndClearTcc(edma_obj_p->hEdma,
		                                 edma_obj_p->tccId,
                                         &tccStatus);

	status |= EDMA3_DRV_clearErrorBits (edma_obj_p->hEdma,
			                            edma_obj_p->edmaChId);
	*/

    //Vps_printf("z: gpyramid: tp#e.1");

	status = EDMA3_DRV_enableTransfer(edma_obj_p->hEdma,
			                          edma_obj_p->edmaChId,
							          EDMA3_DRV_TRIG_MODE_MANUAL);

    //Vps_printf("z: gpyramid: tp#e.2");

	if(flag_blocking)
	{
	    //Vps_printf("z: edma_1DS: blocking");

	    status = EDMA3_DRV_waitAndClearTcc(edma_obj_p->hEdma, edma_obj_p->tccId);

	    //Vps_printf("z: edma_1DS: clear");
	}


	UTILS_assert(status == EDMA3_DRV_SOK);

	return status;
}



//zz: edma 2D simple
EDMA3_DRV_Result edma_2DS_transfer(EDMA_OBJ_t *  edma_obj_p,
		                           uint32_t  address_dst,
		                           uint32_t  address_src,
		                           uint32_t  acnt_element,
		                           uint32_t  bcnt_element,
		                           uint32_t  srcBIdx,
		                           uint32_t  desBIdx,
		                           uint8_t flag_blocking  // if TRUE, wait till the finish of the TRX
		                          )
{
	uint32_t     opt;
	//unsigned short   tccStatus = ((unsigned short)edma_obj_p->tccId) & 0x3F;
	EDMA3_DRV_Result status = EDMA3_DRV_SOK;

	opt = (((edma_obj_p->tccId            << EDMA3_CCRL_OPT_TCC_SHIFT) & EDMA3_CCRL_OPT_TCC_MASK) |  // TCC ID
		   (EDMA3_CCRL_OPT_TCINTEN_ENABLE << EDMA3_CCRL_OPT_TCINTEN_SHIFT)                        |  // enable TCINT to write TCC
		   (EDMA3_CCRL_OPT_SYNCDIM_ABSYNC << EDMA3_CCRL_OPT_SYNCDIM_SHIFT)                           // AB-sync
		   );

	edma_obj_p->pParamSet->opt      = opt;
	edma_obj_p->pParamSet->srcAddr  = address_src;
	edma_obj_p->pParamSet->aCnt     = acnt_element;
	edma_obj_p->pParamSet->bCnt     = bcnt_element;
	edma_obj_p->pParamSet->destAddr = address_dst;
	edma_obj_p->pParamSet->srcBIdx  = srcBIdx;
	edma_obj_p->pParamSet->destBIdx = desBIdx;
	edma_obj_p->pParamSet->linkAddr = 0xFFFF;   // null link
	edma_obj_p->pParamSet->cCnt     = 1;


	//if(!flag_firstTRX)
	//{
	//	status |= EDMA3_DRV_waitAndClearTcc(edma_obj_p->hEdma, edma_obj_p->tccId);
	//}

	//status = EDMA3_DRV_clearErrorBits (edma_obj_p->hEdma, edma_obj_p->edmaChId);

	//UTILS_assert(status == EDMA3_DRV_SOK);

    status = EDMA3_DRV_enableTransfer(edma_obj_p->hEdma, edma_obj_p->edmaChId, EDMA3_DRV_TRIG_MODE_MANUAL);

	if(flag_blocking)
	{
		//Vps_printf("z: edma_2DS: blocking");

		 status |= EDMA3_DRV_waitAndClearTcc(edma_obj_p->hEdma, edma_obj_p->tccId);

	    //Vps_printf("z: edma_2DS: clear");

	}

    if(status != EDMA3_DRV_SOK)
    {
    	UTILS_assert(0);
    }

	return status;
}





//zz: edma 3D simple
EDMA3_DRV_Result edma_3DS_transfer(EDMA_OBJ_t  * edma_obj_p,
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
                                  )
{
    uint32_t     opt;
    //unsigned short   tccStatus = ((unsigned short)edma_obj_p->tccId) & 0x3F;
    EDMA3_DRV_Result status = EDMA3_DRV_SOK;

    opt = (((edma_obj_p->tccId            << EDMA3_CCRL_OPT_TCC_SHIFT) & EDMA3_CCRL_OPT_TCC_MASK) |  // TCC ID
           (EDMA3_CCRL_OPT_TCINTEN_ENABLE << EDMA3_CCRL_OPT_TCINTEN_SHIFT)                        |  // enable TCINT to write TCC
           (EDMA3_CCRL_OPT_ITCCHEN_ENABLE << EDMA3_CCRL_OPT_ITCCHEN_SHIFT)                        |  // enable ITCCHEN
           (EDMA3_CCRL_OPT_SYNCDIM_ABSYNC << EDMA3_CCRL_OPT_SYNCDIM_SHIFT)                           // AB-sync
          );

    edma_obj_p->pParamSet->opt      = opt;
    edma_obj_p->pParamSet->srcAddr  = address_src;
    edma_obj_p->pParamSet->aCnt     = acnt_element;
    edma_obj_p->pParamSet->bCnt     = bcnt_element;
    edma_obj_p->pParamSet->cCnt     = ccnt_element;
    edma_obj_p->pParamSet->destAddr = address_dst;
    edma_obj_p->pParamSet->srcBIdx  = srcBIdx;
    edma_obj_p->pParamSet->srcCIdx  = srcCIdx;
    edma_obj_p->pParamSet->destBIdx = desBIdx;
    edma_obj_p->pParamSet->destCIdx = desCIdx;
    edma_obj_p->pParamSet->linkAddr = 0xFFFF;   // null link

    status = EDMA3_DRV_enableTransfer(edma_obj_p->hEdma, edma_obj_p->edmaChId, EDMA3_DRV_TRIG_MODE_MANUAL);

    if(flag_blocking)
    {
        status |= EDMA3_DRV_waitAndClearTcc(edma_obj_p->hEdma, edma_obj_p->tccId);
    }

//    if(status != EDMA3_DRV_SOK)
//    {
//        UTILS_assert(0);
//    }

    return status;
}
#endif
