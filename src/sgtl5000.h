//------------------------------------------------------------------------------
//  sgtl5000.h
//
//  SGTL5000 Audio Codec Driver
//
//  Cooper Baker - Complex Arts - (c) 10/16/2018 - All Rights Reserved
//------------------------------------------------------------------------------


#ifndef _SGTL5000_H_
#define _SGTL5000_H_


#ifdef __cplusplus
extern "C" {
#endif


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdio.h>
#include <esp_types.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "soc/soc.h"


//------------------------------------------------------------------------------
// sgtl5000 register addresses
//------------------------------------------------------------------------------
#define         SGTL5000_CHIP_ID                        0x0000
#define         SGTL5000_CHIP_DIG_POWER                 0x0002
#define         SGTL5000_CHIP_CLK_CTRL                  0x0004
#define         SGTL5000_CHIP_I2S_CTRL                  0x0006
#define         SGTL5000_CHIP_SSS_CTRL                  0x000A
#define         SGTL5000_CHIP_ADCDAC_CTRL               0x000E
#define         SGTL5000_CHIP_DAC_VOL                   0x0010
#define         SGTL5000_CHIP_PAD_STRENGTH              0x0014
#define         SGTL5000_CHIP_ANA_ADC_CTRL              0x0020
#define         SGTL5000_CHIP_ANA_HP_CTRL               0x0022
#define         SGTL5000_CHIP_ANA_CTRL                  0x0024
#define         SGTL5000_CHIP_LINREG_CTRL               0x0026
#define         SGTL5000_CHIP_REF_CTRL                  0x0028
#define         SGTL5000_CHIP_MIC_CTRL                  0x002A
#define         SGTL5000_CHIP_LINE_OUT_CTRL             0x002C
#define         SGTL5000_CHIP_LINE_OUT_VOL              0x002E
#define         SGTL5000_CHIP_ANA_POWER                 0x0030
#define         SGTL5000_CHIP_PLL_CTRL                  0x0032
#define         SGTL5000_CHIP_CLK_TOP_CTRL              0x0034
#define         SGTL5000_SHIP_ANA_STATUS                0x0036
#define         SGTL5000_CHIP_ANA_TEST1                 0x0038
#define         SGTL5000_CHIP_ANA_TEST2                 0x003A
#define         SGTL5000_CHIP_SHORT_CTRL                0x003C
#define         SGTL5000_DAP_CONTROL                    0x0100
#define         SGTL5000_DAP_PEQ                        0x0102
#define         SGTL5000_DAP_BASS_ENHANCE               0x0104
#define         SGTL5000_DAP_BASS_ENHANCE_CTRL          0x0106
#define         SGTL5000_DAP_AUDIO_EQ                   0x0108
#define         SGTL5000_DAP_SGTL_SURROUND              0x010A
#define         SGTL5000_DAP_FILTER_COEF_ACCESS         0x010C
#define         SGTL5000_DAP_COEF_WR_B0_MSB             0x010E
#define         SGTL5000_DAP_COEF_WR_B0_LSB             0x0110
#define         SGTL5000_DAP_AUDIO_EQ_BASS_BAND0        0x0116
#define         SGTL5000_DAP_AUDIO_EQ_BAND1             0x0118
#define         SGTL5000_DAP_AUDIO_EQ_BAND2             0x011A
#define         SGTL5000_DAP_AUDIO_EQ_BAND3             0x011C
#define         SGTL5000_DAP_AUDIO_EQ_TREBLE_BAND4      0x011E
#define         SGTL5000_DAP_MAIN_CHAN                  0x0120
#define         SGTL5000_DAP_MIX_CHAN                   0x0122
#define         SGTL5000_DAP_AVC_CTRL                   0x0124
#define         SGTL5000_DAP_AVC_THRESHOLD              0x0126
#define         SGTL5000_DAP_AVC_ATTACK                 0x0128
#define         SGTL5000_DAP_AVC_DECAY                  0x012A
#define         SGTL5000_DAP_COEF_WR_B1_MSB             0x012C
#define         SGTL5000_DAP_COEF_WR_B1_LSB             0x012E
#define         SGTL5000_DAP_COEF_WR_B2_MSB             0x0130
#define         SGTL5000_DAP_COEF_WR_B2_LSB             0x0132
#define         SGTL5000_DAP_COEF_WR_A1_MSB             0x0134
#define         SGTL5000_DAP_COEF_WR_A1_LSB             0x0136
#define         SGTL5000_DAP_COEF_WR_A2_MSB             0x0138
#define         SGTL5000_DAP_COEF_WR_A2_LSB             0x013A


//------------------------------------------------------------------------------
// Board Implementation
//------------------------------------------------------------------------------

// Power rails in millivolts
//------------------------------------------------------------------------------
#define         SGTL5000_VDDA                   1800
#define         SGTL5000_VDDD                   1800
#define         SGTL5000_VDDIO                  3300
#define         SGTL5000_EXT_VDDD               1


// GPIO pins for I2C
//------------------------------------------------------------------------------
#define         SGTL5000_I2C_SCL_IO             19
#define         SGTL5000_I2C_SDA_IO             5
#define         SGTL5000_I2C_ADDR               0x0A                // codec slave address

// GPIO pins for I2S
//------------------------------------------------------------------------------
#define         SGTL5000_MCLK                   0                   // NOTE: Do not change MCLK pad
#define         SGTL5000_LRCLK                  25
#define         SGTL5000_BCK                    26
#define         SGTL5000_DOUT                   2
#define         SGTL5000_DIN                    13


//------------------------------------------------------------------------------
// I2C Settings
//------------------------------------------------------------------------------
#define         SGTL5000_I2C_NUM                1                   // i2c module number
#define         SGTL5000_I2C_FREQ_HZ            100000              // master clock frequency (Hz)
#define         SGTL5000_SAMPLERATE             32000               // audio sample rate
#define         SGTL5000_BITSPERSAMPLE          16                  // audio bit depth
#define         WRITE_BIT                       I2C_MASTER_WRITE    // i2c master write
#define         READ_BIT                        I2C_MASTER_READ     // i2c master read
#define         ACK_CHECK_EN                    0x1                 // i2c master will check ack from slave
#define         ACK_CHECK_DIS                   0x0                 // i2c master will not check ack from slave
#define         ACK_VAL                         0x1                 // i2c ack value
#define         NACK_VAL                        0x0                 // i2c nack value
#define         DMA_BUFFER_LENGTH               1024                 // bytes in each dma buffer
#define         DMA_BUFFER_COUNT                4                  // number of dma buffers
#define         I2C_MASTER_TX_BUF_DISABLE       0                   // i2c master do not need buffer
#define         I2C_MASTER_RX_BUF_DISABLE       0                   // i2c master do not need buffer
#define         I2S_NUM                         (0)                 // NOTE: Do not change i2s
                                                                    // num right now because it
                                                                    // will affect MCLK output!


//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------
esp_err_t       sgtl5000_write_reg              ( i2c_port_t i2c_num, uint16_t reg_addr, uint16_t  reg_val );
esp_err_t       sgtl5000_read_reg               ( i2c_port_t i2c_num, uint16_t reg_addr, uint16_t *reg_val );

esp_err_t       sgtl5000_pin_drive_strength     ( uint8_t i2c_strength, uint8_t i2s_strength );

esp_err_t       sgtl5000_set_digital_volume     ( int8_t left_vol, int8_t right_vol );
esp_err_t       sgtl5000_set_headphone_volume   ( int8_t left_vol, int8_t right_vol );

esp_err_t       sgtl5000_set_surround_sound     ( uint8_t  surround    );
esp_err_t       sgtl5000_set_mic_resistor       ( uint8_t  bias        );
esp_err_t       sgtl5000_set_mic_voltage        ( uint16_t voltage     );
esp_err_t       sgtl5000_set_mic_gain           ( uint8_t  gain        );
esp_err_t       sgtl5000_set_ref                ( uint16_t vag_voltage );

esp_err_t       sgtl5000_unmute_headphone       ( void );
esp_err_t       sgtl5000_mute_headphone         ( void );
esp_err_t       sgtl5000_check_module           ( void );

void            sgtl5000_i2c_init               ( void );
int            sgtl5000_i2s_init               ( void );
esp_err_t       sgtl5000_audio_init             ( void );


#ifdef __cplusplus
}
#endif
#endif


//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------