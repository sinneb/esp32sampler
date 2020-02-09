# esp32sampler

Demonstration of a first working version of a sampler application for the ESP32 WROVER board. This version will sample 30 seconds of single channel (left / right) audio with a samplerate of 32000. Then, the demo will automatically switch to looped playback of the sampled content. 

I'm using the [ESP-WROVER-KIT](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/get-started-wrover-kit.html) from espressif and the [Teensy audio board](https://www.pjrc.com/store/teensy3_audio.html) with the SGTL5000 codec.


# Hookup guide

![hookup image](https://sinneb.github.io/uploads/wrover_sgtl5000.jpg)

Most pins are defined in the sgtl5000 header file. 

| teensy    | wrover-kit |
|-----------|------------|
| i2c-sda   | 5          |
| i2c-scl   | 19         |
| i2s-rx    | 13         |
| i2s-tx    | 2          |
| i2s-LRCLK | 25         |
| i2s-BCLK  | 26         |
| i2s-MCLK  | 0 (fixed)  |
