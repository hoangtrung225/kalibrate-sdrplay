/*
 * Copyright (c) 2010, Joshua Lackey
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     *  Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *     *  Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

enum {
	BI_NOT_DEFINED,
	GSM_850,
	GSM_R_900,
	GSM_900,
	GSM_E_900,
	DCS_1800,
	PCS_1900
};

const char *bi_to_str(int bi);
int str_to_bi(char *s);
double arfcn_to_freq(int n, int *bi = 0);
int freq_to_arfcn(double freq, int *bi = 0);
int first_chan(int bi);
int next_chan(int chan, int bi);

enum {
	VIETTEL = 0, 
	MOBIFONE = 1, 
	VINAPHONE = 2, 
	VIETNAMOBILE = 3, 
	GMOBILE = 4
};

static double vn_arfcn_up_GSM_900[] = {880, 890, 898.5, 906.7, 914.9};
// GSM 900 band VIETNAMOBILE = 0, VINAPHONE = 1, VIETTEL = 2, MOBIFONE = 3
static int vn_mnc_GSM_900[] = {2, 3, 1, 0, -1};
static double vn_arfcn_down_GSM_900[] = {925, 935, 943.5, 951.7, 959.9};

static double vn_arfcn_up_DCS_1800[] = {1710, 1730, 1750, 1770, 1785};
// DCS 1800 band VINAPHONE = 0, MOBIFONE = 1, VIETTEL = 2, GMOBILE = 3
static int vn_mnc_DCS_1800[] = {2, 1, 0, -1, 3};
static double vn_arfcn_down_DCS_1800[] = {1805, 1825, 1845, 1865, 1880};
