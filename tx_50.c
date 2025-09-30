#include "lime/LimeSuite.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

int main(void)
{
    lms_device_t* dev = NULL;
    lms_stream_t txs;

    // Find and open device
    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) { fprintf(stderr, "No LimeSDR found\n"); return -1; }
    if (LMS_Open(&dev, list[0], NULL)) { fprintf(stderr, "LMS_Open failed\n"); return -1; }

    // Init and enable TX channel A (ch=0)
    LMS_Init(dev);
    LMS_EnableChannel(dev, LMS_CH_TX, 0, true);

    // Set sample rate (5 Msps is enough here)
    LMS_SetSampleRate(dev, 5e6, 1);

    // Set LO to 50 MHz
    LMS_SetLOFrequency(dev, LMS_CH_TX, 0, 50e6);

    // Calibrate TX
    LMS_Calibrate(dev, LMS_CH_TX, 0, 5e6, 0);

    // Setup TX stream
    memset(&txs, 0, sizeof(txs));
    txs.channel = 0;
    txs.isTx = true;
    txs.fifoSize = 1<<16;
    txs.dataFmt = lms_stream_t::LMS_FMT_I16;
    LMS_SetupStream(dev, &txs);
    LMS_StartStream(&txs);

    // Prepare constant buffer (I=70% FS, Q=0)
    const size_t N = 4096;
    int16_t* buf = malloc(2*N*sizeof(int16_t));
    for (size_t i=0;i<N;i++){ buf[2*i]= (int16_t)(0.7*32767); buf[2*i+1]=0; }

    printf("Transmitting carrier at 50 MHz... Press Ctrl+C to stop\n");
    while (1) {
        lms_stream_meta_t meta = {0};
        LMS_SendStream(&txs, buf, N, &meta, 1000);
    }

    // Cleanup (unreachable here, but for completeness)
    free(buf);
    LMS_StopStream(&txs);
    LMS_DestroyStream(dev, &txs);
    LMS_Close(dev);
    return 0;
}
