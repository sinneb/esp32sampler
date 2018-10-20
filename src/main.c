//------------------------------------------------------------------------------
//  main.c
//
//  SGTL5000 Audio Codec Driver
//
//  Cooper Baker - Complex Arts - (c) 10/19/2018 - All Rights Reserved
//------------------------------------------------------------------------------


#ifdef __cplusplus
extern "C" {
#endif


//------------------------------------------------------------------------------
// inlcudes
//------------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sgtl5000.h"


//------------------------------------------------------------------------------
// prototypes
//------------------------------------------------------------------------------
void audio_task( void *pvParameter );
void app_main( void );


//------------------------------------------------------------------------------
// audio_task - audio processing task
//------------------------------------------------------------------------------
void audio_task( void *pvParameter )
{
    // audio signal buffer
    signed short sig[ 256 ];

    // loop iterator
    uint8_t i;

    // i2s_write bytes
    uint16_t bytes;

    // oscillator values
    float hertz = 1000;
    float phase = 0;
    float phase_inc = hertz * ( 1.0 / (float)SGTL5000_SAMPLERATE );

    // audio task loop
    while( 1 )
    {
        // iterate through the signal frame
        for( i = 0 ; i < 128 ; i++ )
        {
            // increment and wrap phase
            phase += phase_inc;
            phase  = phase > 1.0 ? phase - 1.0 : phase;

            // make a sine wave in the left channel
            sig[ i * 2 + 1 ] = sin( phase * 2 * 3.141592 ) * 32768;  // left (i2s buffer is interleaved)
            sig[ i * 2     ] = 0;                                    // right
        }

        // write signal buffer to i2s
        i2s_write( I2S_NUM, ( const char* )sig, 512, &bytes, portMAX_DELAY );
    }
}


//------------------------------------------------------------------------------
// app_main - start of execution
//------------------------------------------------------------------------------
void app_main()
{
    // wait for complete boot
    ets_delay_us( 300000 );

    // initialize sgtl5000 i2s interface
    sgtl5000_i2s_init();

    // let the mclk flow
    ets_delay_us( 1000 );

    // initialize sgtl5000 i2c interface
    sgtl5000_i2c_init ();

    // initialize sgtl5000 for audio
    sgtl5000_init();

    // create a task to fill dma buffers with audio
    xTaskCreate( &audio_task, "audio_task", 4096, NULL, 8, NULL );
}


#ifdef __cplusplus
} // extern "C"
#endif


//------------------------------------------------------------------------------
// eof
//------------------------------------------------------------------------------
