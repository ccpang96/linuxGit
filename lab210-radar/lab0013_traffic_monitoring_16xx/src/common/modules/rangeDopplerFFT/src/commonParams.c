/**
 *   @file  commonParams.c
 *
 *   @brief
 *      Common parameters for range and Doppler FFT
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
#ifndef _RADARDEMO_FFTS_COMMONPARAMS_H
#define _RADARDEMO_FFTS_COMMONPARAMS_H

/**
 *  @file   commonParams.h
 *  @brief  common parameters shared in FFTs
 *
 */

#include <swpform.h>

/* NOTE:
 * 1) The section ".L2heap" is currently mapped to L2SRAM.
 * 2) The constant UTILS_MEM_HEAP_L2_SIZE in utils_mem_cfg.h has been adjusted
 *    to take this into account. Currently 1 KB is accounted for the radar
 *    variables mapped to L2SRAM.
 * 2) If the total size of the radar variables exceed 1 KB then make sure to
 *    adjust the constant defined in (2). Otherwise it will lead to corruption.
 */
/*---------constant   tables-----------*/
#pragma DATA_ALIGN(brev, 64);
#pragma DATA_SECTION (brev, ".L2heap");
unsigned char brev[64] = {
    0x0, 0x20, 0x10, 0x30, 0x8, 0x28, 0x18, 0x38,
    0x4, 0x24, 0x14, 0x34, 0xc, 0x2c, 0x1c, 0x3c,
    0x2, 0x22, 0x12, 0x32, 0xa, 0x2a, 0x1a, 0x3a,
    0x6, 0x26, 0x16, 0x36, 0xe, 0x2e, 0x1e, 0x3e,
    0x1, 0x21, 0x11, 0x31, 0x9, 0x29, 0x19, 0x39,
    0x5, 0x25, 0x15, 0x35, 0xd, 0x2d, 0x1d, 0x3d,
    0x3, 0x23, 0x13, 0x33, 0xb, 0x2b, 0x1b, 0x3b,
    0x7, 0x27, 0x17, 0x37, 0xf, 0x2f, 0x1f, 0x3f
};

/*Scaling factor for differnt FFT size 16 32 64 128 256 512 1024 2048*/
#pragma DATA_ALIGN(scaleFactor, 64);
#pragma DATA_SECTION (scaleFactor, ".L2heap");
/*const float scaleFactor[8] = {0.0078125,   0.00390625,   0.001953125,  0.0009765625,
		0.00048828125,  0.000244140625, 0.0001220703125,  0.00006103515625};*/

const float scaleFactor[8] = { 0.0625, 0.03125,  0.015625, 0.0078125,
		0.00390625, 0.001953125, 0.0009765625,  0.00048828125};
const float relaxFactor = 4.0;

const char start_FFT_table_index = 4;
const char end_FFT_table_index = 11;

#endif

