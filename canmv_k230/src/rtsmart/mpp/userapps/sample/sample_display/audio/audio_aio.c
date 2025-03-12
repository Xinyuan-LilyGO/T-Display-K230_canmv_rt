/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include<stdio.h>
#include<pthread.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "mpi_sys_api.h"

#include "audio_aio.h"
#include "audio_sample.h"

static pthread_t g_pthread_handle;
static k_u32     g_sample_rate = 44100;//44100;
static k_audio_bit_width g_bit_width = KD_AUDIO_BIT_WIDTH_16;
static k_bool    g_enable_audio_codec = K_TRUE;
static k_i2s_work_mode g_i2s_work_mode = K_STANDARD_MODE;
//static k_bool    g_enable_audio3a = K_FALSE;
static char g_wav_name[256];
//static k_u32     g_channel_count = 2;
//static k_i2s_in_mono_channel  g_mono_channel = KD_I2S_IN_MONO_RIGHT_CHANNEL;//mono channel use mic input

static void *sample_thread_fn(void *arg)
{
        printf("sample ao i2s module\n");
        audio_sample_send_ao_data(g_wav_name,0, 0,g_sample_rate,g_bit_width, g_i2s_work_mode);
        return NULL;
}


void audio_ao_play()
{
    memcpy(g_wav_name,"44k_16.wav",strlen("44k_16.wav")+1); 
    audio_sample_enable_audio_codec(g_enable_audio_codec); 
    pthread_create(&g_pthread_handle, NULL, sample_thread_fn, NULL);
        /*printf("enter q key to exit\n");
        while(getchar() != 'q')
        {
            usleep(100*1000);
        }
    audio_sample_exit();
    pthread_join(g_pthread_handle, NULL);

    printf("destroy vb block \n");
    //audio_sample_vb_destroy();
    printf("sample done\n");*/

    return;
}

void audio_ao_pause()
{
    audio_sample_exit();
    pthread_join(g_pthread_handle, NULL);

    printf("destroy vb block \n");
    //audio_sample_vb_destroy();
    printf("sample done\n");

    return;
}









