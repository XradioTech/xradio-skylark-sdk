/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#include <stdio.h>
#include <math.h>

#include "fft.h"


#define N			10			// fixed N = 10, for 1024 points FFT
#define FFT_Points	(1<<N)
//#define Fs			16000		// sample freqency = 16kHz
#define Fin			1000		// input signal frequency = 1kHz
#define WN_Coffe	31			//

extern const COMPLEX WN[];
extern const __u32 Bh[];

COMPLEX dbuf[FFT_Points];

COMPLEX complex_add( COMPLEX x1, COMPLEX x2)
{
	COMPLEX x;
	x.Re = x1.Re + x2.Re;
	x.Im = x1.Im + x2.Im;

	return x;
}
COMPLEX complex_sub( COMPLEX x1, COMPLEX x2)
{
	COMPLEX x;
	x.Re = x1.Re - x2.Re;
	x.Im = x1.Im - x2.Im;

	return x;
}
COMPLEX complex_mul( COMPLEX x1, COMPLEX x2)
{
	COMPLEX x;
	x.Re = (__s32)(((__s64)x1.Re * (__s64)x2.Re)>>WN_Coffe) - (__s32)(((__s64)x1.Im * (__s64)x2.Im)>>WN_Coffe);
	x.Im = (__s32)(((__s64)x1.Re * (__s64)x2.Im)>>WN_Coffe) + (__s32)(((__s64)x1.Im * (__s64)x2.Re)>>WN_Coffe);

	return x;
}

FFT_RESULT Cooley_Tukey_FFT(__s32 *data, __u32 fs)
{
	__u32 i, j, k, index=0;
	COMPLEX temp1, temp2, temp3;
	FFT_RESULT ret;
	__u32 sig_index, sig_ibw_half, max_index = 0;
	double sig_power = 0, noise_power = 0, temp_power, max_power = 0;


	for(i = 0; i < FFT_Points; i++)
	{
		index = 0;
		for(j = 0; j < N; j++)
		{
			index += ((i>>j)&0x1)<<(N-j-1);
		}
		dbuf[i].Re = (__s32)(((__s64)(data[index]<<16) * (__s64)Bh[index]) >> 31);
		dbuf[i].Im = 0;
	}

	if(debug_print_en)
	{
		for(i = 0; i < FFT_Points; i++)
		{
			printf("dbuf[%d].Re = %d, dbuf[%d].Im = %d.\n", i, dbuf[i].Re, i, dbuf[i].Im);
		}
	}

	for(i = 0; i < N; i++)
	{
		for(j = 0; j < (1<<(N-i-1)); j++)
		{
			for(k = 0; k < (1<<i); k++)
			{
				temp1 = complex_mul(dbuf[(1<<i) + j*(1<<(i+1)) + k], WN[k*(1<<(N-1-i))]);
				temp2 = complex_add(dbuf[j*(1<<(i+1)) + k], temp1);		//Butterfly group index interval = 2^i
				temp3 = complex_sub(dbuf[j*(1<<(i+1)) + k], temp1);

				dbuf[j*(1<<(i+1)) + k] = temp2;
				dbuf[(1<<i) + j*(1<<(i+1)) + k] = temp3;
			}
		}
	}

	if(debug_print_en)
	{
		printf("Data after FFT:\n");
		for(i = 0; i < FFT_Points; i++)
		{
			printf("dbuf_z[%d].Re = %d, FFT_Data[%d].Im = %d.\n", i, dbuf[i].Re, i, dbuf[i].Im);
		}
		for(i = 0; i < FFT_Points; i++)
		{
			printf("dbuf_z_magnitude[%d] = %f\n", i, sqrt(((__s64)dbuf[i].Re * (__s64)dbuf[i].Re) + ((__s64)dbuf[i].Im * (__s64)dbuf[i].Im)));
		}
		for(i = 0; i < FFT_Points; i++)
		{
			printf("dbuf_z_dB[%d] = %f\n", i, 20*log10(sqrt(((__s64)dbuf[i].Re * (__s64)dbuf[i].Re) + ((__s64)dbuf[i].Im * (__s64)dbuf[i].Im))));
		}
	}

	sig_index = floor((Fin*FFT_Points)/fs);
	sig_ibw_half = floor(sig_index/4);
	printf("sig_index = %d, sig_ibw_half = %d\n",sig_index, sig_ibw_half);
	sig_power = 0; max_power = 0;
	for(i = sig_index - sig_ibw_half; i < sig_index + sig_ibw_half; i++)
	{
		temp_power = sqrt((__s64)(dbuf[i].Re * (__s64)dbuf[i].Re) + (__s64)(dbuf[i].Im * (__s64)dbuf[i].Im));
		sig_power += temp_power;

		if(temp_power > max_power)
		{
			max_power = temp_power;
			max_index = i;
		}
	}
	ret.sig_power = 20*log10(sig_power);

	printf("sig index = %d\n",max_index);
	ret.sig_freq = (max_index+1)*fs/FFT_Points;

	noise_power = 0;
	for(i = 0; i < sig_index - sig_ibw_half; i++)
	{
		temp_power = sqrt((__s64)(dbuf[i].Re * (__s64)dbuf[i].Re) + (__s64)(dbuf[i].Im * (__s64)dbuf[i].Im));
		noise_power += temp_power;
	}
	for(i = sig_index + sig_ibw_half; i < FFT_Points/2; i++)
	{
		temp_power = sqrt((__s64)(dbuf[i].Re * (__s64)dbuf[i].Re) + (__s64)(dbuf[i].Im * (__s64)dbuf[i].Im));
		noise_power += temp_power;
	}
	ret.noise_power = 20*log10(noise_power);

	sig_power = 0; max_power = 0;
	for(i = sig_index*2 - sig_ibw_half; i < sig_index*2 + sig_ibw_half; i++)
	{
		temp_power = sqrt((__s64)(dbuf[i].Re * (__s64)dbuf[i].Re) + (__s64)(dbuf[i].Im * (__s64)dbuf[i].Im));
		sig_power += temp_power;

		if(temp_power > max_power)
		{
			max_power = temp_power;
			max_index = i;
		}
	}
	ret.Harm2nd_power = 20*log10(sig_power);
	printf("Harm2nd index = %d, F_Harm2nd = %f kHz\n",max_index, (double)(max_index+1)*fs/FFT_Points/1000);

	sig_power = 0; max_power = 0;
	for(i = sig_index*3 - sig_ibw_half; i < sig_index*3 + sig_ibw_half; i++)
	{
		temp_power = sqrt((__s64)(dbuf[i].Re * (__s64)dbuf[i].Re) + (__s64)(dbuf[i].Im * (__s64)dbuf[i].Im));
		sig_power += temp_power;

		if(temp_power > max_power)
		{
			max_power = temp_power;
			max_index = i;
		}
	}
	ret.Harm3th_power = 20*log10(sig_power);
	printf("Harm3th index = %d, F_Harm3th = %f kHz\n",max_index, (double)(max_index+1)*fs/FFT_Points/1000);

	return ret;
}

