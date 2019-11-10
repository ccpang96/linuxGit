/**
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 *  @file   gen_twiddle_fftf.c
 *  @brief  implementation of functions to generate single precision twiddle factors
 *
 *
 */

#include <math.h>
#include "gen_twiddle_fftf.h"

#ifndef PI
# ifdef M_PI
#  define PI M_PI
# else
#  define PI 3.14159265358979323846
# endif
#endif



#ifdef _LITTLE_ENDIAN
int gen_twiddle_fftSP(float *w, int n)
{
	int i, j, k, s=0, t;

	    for (j = 1, k = 0; j < n >> 2; j = j << 2, s++) {
	        for (i = t=0; i < n >> 2; i += j, t++) {
	            w[k +  5] =  cos(6.0 * PI * i / n);
	            w[k +  4] =  sin(6.0 * PI * i / n);

	            w[k +  3] =  cos(4.0 * PI * i / n);
	            w[k +  2] =  sin(4.0 * PI * i / n);

	            w[k +  1] =  cos(2.0 * PI * i / n);
	            w[k +  0] =  sin(2.0 * PI * i / n);

	            k += 6;
	        }
	    }
	    return k;
}
#else
int gen_twiddle_fftf(float *w, int n)
{

}
#endif



#ifdef _LITTLE_ENDIAN
int gen_twiddle_ifftSP(float *w, int n)
{
	int i, j, k, s=0, t;

	    for (j = 1, k = 0; j < n >> 2; j = j << 2, s++) {
	        for (i = t=0; i < n >> 2; i += j, t++) {
	            w[k +  5] =  cos(6.0 * PI * i / n);
	            w[k +  4] =  -sin(6.0 * PI * i / n);

	            w[k +  3] =  cos(4.0 * PI * i / n);
	            w[k +  2] =  -sin(4.0 * PI * i / n);

	            w[k +  1] =  cos(2.0 * PI * i / n);
	            w[k +  0] =  -sin(2.0 * PI * i / n);

	            k += 6;
	        }
	    }
	    return k;
}
#else
int gen_twiddle_fftf(float *w, int n)
{

}
#endif
