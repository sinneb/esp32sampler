
// https://sinneb.github.io
// i2s tech demo
// esp32 (ESP-WROVER-KIT with ESP32-WROVER -> 4MB flash, 4MB psram) & SGTL5000 (teensy audioboard)
// samplerate = 32000
// bitdepth = 16bit

// Demo will record for ~30 seconds from the line in,
// and than switch to playback from the recorded buffer

// buffer is allocated as 900000 bytes. samplerate 32000 -> 28.125 seconds.

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sgtl5000.h"
#include "esp_log.h"

#define CHANNELS 2

// hardware timers
hw_timer_t * audio_timer = NULL;
hw_timer_t * ppq_timer = NULL;

bool getAudio = true;
bool prepAudio = true;
size_t bytes;
long cpuload = 0;

// allocate i2s read/write buffers
signed short i2s_out[ DMA_BUFFER_LENGTH / CHANNELS ];
signed short i2s_in[ DMA_BUFFER_LENGTH / CHANNELS ];
signed short i2s_mixbuffer [DMA_BUFFER_LENGTH / CHANNELS];
//signed short* i2s_in = (signed short*) calloc(256, sizeof(signed short));

// recordbuffer 1
signed short * recbuffer1 = 0;
long recbuffer1_offset = 0;
bool recbuffer1_complete = false;

// calculate number of frames (i.e. samples per channel)
const uint16_t frames = ( DMA_BUFFER_LENGTH / CHANNELS ) / ( SGTL5000_BITSPERSAMPLE / 8 );

// allocate dsp i/o buffers
float in1 [ frames ];
float in2 [ frames ];
float out1[ frames ];
float out2[ frames ];

void onPpq_Timer() {
    // blink buildin LED
    digitalWrite(4,!digitalRead(4));
}

void onAudio_Timer() {
    // start audio processing in main loop
    getAudio = true;
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 sampler start");
    // print free memory
    Serial.println(ESP.getFreeHeap());

    // init recbuffers in esp32 psram
    recbuffer1 = (signed short*)ps_malloc(900000);

    // print free memory
    Serial.println(ESP.getFreeHeap());

    // wait for complete boot
    ets_delay_us( 300000 );

    // initialize sgtl5000 i2s interface
    int i2sinit = sgtl5000_i2s_init();

    // let the mclk flow
    ets_delay_us( 1000 );

    // initialize sgtl5000 i2c interface
    sgtl5000_i2c_init ();

    // initialize sgtl5000 for audio
    sgtl5000_audio_init();

    // init and start audio callback timer
    // the callback prepares the audio mixbuffer and  
    // prescaler 80 -> AlarmWrite in microseconds
    // 1000000 / 8000 = 125 callbacks per second
    // samplerate = 32000
    // we are writing 256 frames in every i2s callback ->  125 * 256 = 32000
    audio_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(audio_timer, &onAudio_Timer, true);
    timerAlarmWrite(audio_timer, 8000, true);
    timerAlarmEnable(audio_timer);

    // init and start ppq callback timer
    ppq_timer = timerBegin(3, 80, true);
    timerAttachInterrupt(ppq_timer, &onPpq_Timer, true);
    timerAlarmWrite(ppq_timer, 1000000, true);
    timerAlarmEnable(ppq_timer);

    // headphones volume
    sgtl5000_set_headphone_volume(60,60);

}

void loop() {
    // the prepare audio main loop function
    if(prepAudio) {
        // read from i2s and copy to mixbuffer
        i2s_read( ( i2s_port_t )I2S_NUM, ( char* )i2s_in, DMA_BUFFER_LENGTH, &bytes, portMAX_DELAY );
        memcpy(i2s_mixbuffer, i2s_in, 512 * sizeof *i2s_out);

        // Recording the i2s read to the recbuffer
        // i+2 -> process single audio channel
        for(int i=0;i<512;i=i+2){
            // are we still filling the buffer?
            if(!recbuffer1_complete) {
                // feed single channel (mono) signal into recording buffer 
                recbuffer1[(i/2)+recbuffer1_offset] = i2s_mixbuffer[i];
            } else {
                // playback from buffer
                // dual mono out
                i2s_mixbuffer[i] = recbuffer1[(i/2)+recbuffer1_offset];
                i2s_mixbuffer[i+1] = recbuffer1[(i/2)+recbuffer1_offset];
            }
            
        }
        
        // next frame
        recbuffer1_offset+=256;

        // for demo purposes, after ~30 seconds of line-in recording
        // switch to playback from recorded audio
        if(recbuffer1_offset>800000 && !recbuffer1_complete) {
            Serial.println(millis());
            Serial.println("--------------------> buf 2 full");
            recbuffer1_offset=0;
            recbuffer1_complete = true;
        } 

        // reset prepAudio
        prepAudio = false;
        // heavy cpuload?
        if(cpuload<50000)Serial.println(cpuload);
        cpuload=0;
    }

    // send prepared audioframe to i2s device using DMA 
    if(getAudio) {
        memcpy(i2s_out,i2s_mixbuffer,512 * sizeof *i2s_out);
        i2s_write( ( i2s_port_t )I2S_NUM, ( const char* )i2s_out, DMA_BUFFER_LENGTH, &bytes, portMAX_DELAY   );
        getAudio = false;
        // lets prepare the next audio frame next
        prepAudio = true;
    }
    cpuload++;
}