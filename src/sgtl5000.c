//------------------------------------------------------------------------------
//  sgtl5000.c
//
//  SGTL5000 Audio Codec Driver
//
//  Cooper Baker - Complex Arts - (c) 10/16/2018 - All Rights Reserved
//------------------------------------------------------------------------------


#ifdef __cplusplus
extern "C" {
#endif


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <string.h>
#include "sgtl5000.h"
#include "soc/rtc.h"
#include "soc/soc.h"
#include "esp_log.h"


//------------------------------------------------------------------------------
// variables
//------------------------------------------------------------------------------

// id for esp_log functions
static const char *ID = "SGTL5000";

// i2s configuration structure
static i2s_config_t sgtl5000_i2s_config =
{
    .mode                   = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,      // master tx rx
    .sample_rate            = SGTL5000_SAMPLERATE,                              // default: 48kHz
    .bits_per_sample        = SGTL5000_BITSPERSAMPLE,                           // 16 bits per channel
    .channel_format         = I2S_CHANNEL_FMT_RIGHT_LEFT,                       // 2 channels
    .communication_format   = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,    // communication format
    .dma_buf_count          = DMA_BUFFER_COUNT,                                 // number of dma buffers
    .dma_buf_len            = DMA_BUFFER_LENGTH,                                // size of dma buffer in bytes
    .intr_alloc_flags       = ESP_INTR_FLAG_LEVEL1                              // interrupt level 1
};

// i2s_pin_config_t - i2s pinconfiguration structure
static i2s_pin_config_t sgtl5000_pin_config =
{
    .bck_io_num             = SGTL5000_BCK,
    .ws_io_num              = SGTL5000_LRCLK,
    .data_out_num           = SGTL5000_DOUT,
    .data_in_num            = SGTL5000_DIN
};


//------------------------------------------------------------------------------
// sgtl5000_configure_stream - sets sample rate and bit depth
//
// arg_sample_rate: 48000, 8000, etc
// arg_bits_per_sample: 16, 24, etc
//------------------------------------------------------------------------------
esp_err_t sgtl5000_configure_stream( uint32_t arg_sample_rate, uint32_t arg_bits_per_sample )
{
    // NOTE: To save power, MCLK is always minimum possible, i.e. 256*Fs
    sgtl5000_i2s_config.sample_rate     = arg_sample_rate;
    sgtl5000_i2s_config.bits_per_sample = arg_bits_per_sample;

    return i2s_driver_install( I2S_NUM, &sgtl5000_i2s_config, 0, NULL );
}


//------------------------------------------------------------------------------
// sgtl5000_set_surround_sound - surround sound on/off
//
// 0: Surround OFF
// 1-8: Surround effect level
//------------------------------------------------------------------------------
esp_err_t sgtl5000_set_surround_sound( uint8_t surround )
{
    // no surround
    if( surround == 0 )
    {
        if( sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_DAP_SGTL_SURROUND, 0x0000 ) == 0 )
        {
            return ESP_OK;
        }
        else
        {
            return ESP_FAIL;
        }
    }

    // clip surround max to 7
    if( surround > 7 )
    {
        surround = 7;
    }

    if( sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_DAP_SGTL_SURROUND, 0x3 | ( surround << 4 ) ) == 0 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }

}


//------------------------------------------------------------------------------
// sgtl5000_set_mic_resistor - sets mic bias resistor
//
// 0: hi-Z, 1: 2K, 2: 4K, 3: 8K
//------------------------------------------------------------------------------
esp_err_t sgtl5000_set_mic_resistor( uint8_t bias )
{
    uint16_t val;

    // clip bias value
    if( bias > 0x03 )
    {
        bias = 3;
    }

    sgtl5000_read_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_MIC_CTRL, &val );

    val &= 0xFCFF;
    val |= ( bias << 8 );

    if( sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_MIC_CTRL, val ) == 0 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_set_mic_voltage - sets mic bias voltage
//
// Range: 1250mV to 3000mV in 250mV steps
//------------------------------------------------------------------------------
esp_err_t sgtl5000_set_mic_voltage( uint16_t voltage )
{
    uint16_t val;

    if( voltage < 1250 )
    {
        voltage = 1250;
    }
    else if( voltage > 3000 )
    {
        voltage = 3000;
    }

    if( voltage > SGTL5000_VDDA - 200 )
    {
        return ESP_FAIL;
    }

    voltage -= 1250;
    voltage /= 250;

    sgtl5000_read_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_MIC_CTRL, &val );

    val &= 0xFF8F;
    val |= (voltage << 4);

    if( sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_MIC_CTRL, val ) == 0 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_set_mic_gain - sets microphone gain in 10dB steps
//
// Range: 0 to 3
// 0: 0dB, 1: +20dB... +40dB (?)
//------------------------------------------------------------------------------
esp_err_t sgtl5000_set_mic_gain( uint8_t gain )
{
    uint16_t val;

    if( gain > 3 )
    {
        gain = 3;
    }

    sgtl5000_read_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_MIC_CTRL, &val );

    val &= 0xFFFC;
    val |= gain;

    if( sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_MIC_CTRL, val ) == 0 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_mute_headphone - mutes the headphone output
//------------------------------------------------------------------------------
esp_err_t sgtl5000_mute_headphone( void )
{
    uint16_t val;

    sgtl5000_read_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ANA_CTRL, &val );

    val |= 0x0010;

    if( sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ANA_CTRL, val ) == 0 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_unmute_headphone - unmutes headphone output
//------------------------------------------------------------------------------
esp_err_t sgtl5000_unmute_headphone( void )
{
    uint16_t val;

    sgtl5000_read_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ANA_CTRL, &val );

    val &= 0xFFEF;

    if( sgtl5000_write_reg (SGTL5000_I2C_NUM, SGTL5000_CHIP_ANA_CTRL, val ) == 0 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_set_ref - sets dac and adc reference voltage (VAG)
//
// Range: 800 mV to 1575 mV
//------------------------------------------------------------------------------
esp_err_t sgtl5000_set_ref( uint16_t vag_voltage )
{
    uint16_t val;

    if( vag_voltage < 800 )
    {
        vag_voltage = 800;
    }
    else if( vag_voltage > 1575 )
    {
        vag_voltage = 1575;
    }

    vag_voltage /= 25;
    vag_voltage -= 32;
    vag_voltage  = ( vag_voltage & 0x001F ) << 4;

    sgtl5000_read_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_REF_CTRL, &val );

    val &= 0xFE0F;
    val |= vag_voltage;

    if( sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_REF_CTRL, val ) == 0 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_check_module - checks if the module is connected and initialized
//
// Call this before any other operation, and after setting up
// I2C and I2S interfaces.
//
// Note: MCLK must be active for this check to return ESP_OK
//------------------------------------------------------------------------------
esp_err_t sgtl5000_check_module( void )
{
    uint16_t val;

    sgtl5000_read_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ID, &val );

    if( ( val & 0xFF00 ) == 0xAA00 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_set_digital_volume - sets digital volume of dac output
//
// Leave this value at 0dB if HP volume can be used to control output
//
// Volume: 0 dB to -90 dB
// Recommended (default): 0 dB
//------------------------------------------------------------------------------
esp_err_t sgtl5000_set_digital_volume( int8_t left_vol, int8_t right_vol )
{
    int8_t left, right;

    left  = ( -2 * left_vol  ) + 0x3C;
    right = ( -2 * right_vol ) + 0x3C;

    if( sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_DAC_VOL, ( right << 8 ) | left ) == 0 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_set_headphone_volume - sets headphone volume
//
// Leave this value at 0dB for best audio quality, with DAC volume = 0 dB
//
// For 32 ohm headset in capless mode, -17dB is a good initial value
// Volume: +12 dB to -50 dB
// Recommended (default): 0 dB
//------------------------------------------------------------------------------
esp_err_t sgtl5000_set_headphone_volume( int8_t left_vol, int8_t right_vol )
{
    int8_t left, right;

    left  = ( ( -2 * left_vol  ) + 0x18 ) & 0x7F;
    right = ( ( -2 * right_vol ) + 0x18 ) & 0x7F;

    if( sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ANA_HP_CTRL, ( right << 8 ) | left ) == 0 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_pin_drive_strength - sets digital pad drive strength
//
// Setting high drive strength causes more noise and more power rail noise.
//
// Pad drive strength:
// 0 = disable
// 1 = low
// 2 = medium
// 3 = high
//------------------------------------------------------------------------------
esp_err_t sgtl5000_pin_drive_strength( uint8_t i2c_strength, uint8_t i2s_strength )
{
    uint16_t strength;

    i2c_strength &= 0x03;
    i2s_strength &= 0x03;

    strength  =   i2c_strength | ( i2c_strength << 2 );
    strength |= ( i2s_strength << 4 ) | ( i2s_strength << 6 ) | ( i2s_strength << 8 );

    if( sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ANA_HP_CTRL, strength ) == 0 )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_audio_init - initialize the chip for audio
//------------------------------------------------------------------------------
esp_err_t sgtl5000_audio_init( void )
{
    uint8_t  err;
    uint16_t val;

    // Read chip ID
    err = sgtl5000_read_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ID, &val );
    ESP_LOGI( ID, "Chip ID: %d", val );
    if( err ){ ESP_LOGE( ID, "Chip ID Error - Code: %d", err ); }

    // Digital power control - Enable I2S data in and DAC
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_DIG_POWER, 0x0021);
    if( err ){ ESP_LOGE( ID, "I2S/DAC Power Up Error - Code: %d", err ); }

    // 48kHz sample rate, with CLKM = 256*Fs = 12.288000 MHz
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_CLK_CTRL, 0x0008);
    if( err ){ ESP_LOGE( ID, "Sample Rate Error - Code: %d", err ); }

    // I2S mode = , LRALIGN = 0, LRPOL = 0
    // 32*Fs is SCLK rate, 16 bits/sample, I2S is slave, no PLL used
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_I2S_CTRL, 0x0130 );
    if( err ){ ESP_LOGE( ID, "I2S Configuration Error - Code: %d", err ); }

    // I2S in -> DAC output, rest left at default
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_SSS_CTRL, 0x0010 );
    if( err ){ ESP_LOGE( ID, "I2S to DAC Connection Error - Code: %d", err ); }

    // Unmute DAC, no volume ramp enabled
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ADCDAC_CTRL, 0x0000 );
    if( err ){ ESP_LOGE( ID, "DAC Unmute Error - Code: %d", err ); }

    // DAC volume is 0dB for both channels
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_DAC_VOL, 0x3C3C );
    if( err ){ ESP_LOGE( ID, "DAC Volume Error - Code: %d", err ); }

    // Moderate drive strength (4mA) for all pads
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_PAD_STRENGTH, 0x02AA );
    if( err ){ ESP_LOGE( ID, "Drive Strength Error - Code: %d", err ); }

    // 0dB headphone volume
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ANA_HP_CTRL, 0x3C3C );
    if( err ){ ESP_LOGE( ID, "Headphone Volume Error - Code: %d", err ); }

    // Unmute HP, ZCD disabled, rest mute
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ANA_CTRL, 0x0101 );
    if( err ){ ESP_LOGE( ID, "Headphone Unmute Error - Code: %d", err ); }

    // VAG_VAL = 0.8V + 100mV = 0.9V
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_REF_CTRL, 0x0040 );
    if( err ){ ESP_LOGE( ID, "Analog Voltage Error - Code: %d", err ); }

    // Capless HP and DAC on
    // Stereo DAC with external VDDD source
    err = sgtl5000_write_reg( SGTL5000_I2C_NUM, SGTL5000_CHIP_ANA_POWER, 0x40FC );
    if( err ){ ESP_LOGE( ID, "Analog Power Error - Code: %d", err ); }

    if( err == 0 )
    {
        ESP_LOGI( ID, "Initialization Complete" );
        return ESP_OK;
    }
    else
    {
        ESP_LOGE( ID, "Initialization Error" );
        return ESP_FAIL;
    }
}


//------------------------------------------------------------------------------
// sgtl5000_write_reg - writes to a register of the chip
//
// Write operation:
//
// • Start condition
// • Device address with the R/W bit cleared to indicate write
// • Send two bytes for the 16 bit register address (most significant byte first)
// • Send two bytes for the 16 bits of data to be written to the register (most significant byte first)
// • Stop condition
//------------------------------------------------------------------------------
esp_err_t sgtl5000_write_reg( i2c_port_t i2c_num, uint16_t reg_addr, uint16_t reg_val )
{
    // data words
    uint8_t dwr[4];

    // create i2c command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // start
    i2c_master_start(cmd);

    // address & write bit
    i2c_master_write_byte( cmd, ( SGTL5000_I2C_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN );

    // msbs & lsbs for address & data
    dwr[ 0 ] = ( reg_addr >> 8 ) & 0xFF;    // MSB for reg address
    dwr[ 1 ] = ( reg_addr & 0xFF );         // LSB for reg address
    dwr[ 2 ] = ( reg_val >> 8 ) & 0xFF;     // MSB for reg data
    dwr[ 3 ] = ( reg_val & 0xFF );          // LSB for reg data

    // set msbs & lsbs
    i2c_master_write( cmd, dwr, 4, ACK_CHECK_EN );

    // stop
    i2c_master_stop( cmd );

    // execute command and return status
    esp_err_t ret = i2c_master_cmd_begin( i2c_num, cmd, 1000 / portTICK_RATE_MS );

    // delete i2c command link
    i2c_cmd_link_delete(cmd);

    // return command status
    return ret;
}


//------------------------------------------------------------------------------
// sgtl5000_read_reg - read a register from the chip
//
// Read operation:
//
// • Start condition
// • Device address with the R/W bit cleared to indicate write
// • Send two bytes for the 16 bit register address (most significant byte first)
// • Stop Condition followed by start condition (or a single restart condition)
// • Device address with the R/W bit set to indicate read
// • Read two bytes from the addressed register (most significant byte first)
// • Stop condition
//------------------------------------------------------------------------------
esp_err_t sgtl5000_read_reg( i2c_port_t i2c_num, uint16_t reg_addr, uint16_t *reg_val )
{
    // this will cause a warning, please ignore :(
    uint8_t *byte_val = ( uint8_t* )reg_val;

    // return value
    esp_err_t ret;

    // create i2c command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // start
    i2c_master_start( cmd );

    // set address, write bit, and msb/lsb of register address
    i2c_master_write_byte( cmd, ( SGTL5000_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN );  // Address + Write bit
    i2c_master_write_byte( cmd, ( reg_addr >> 8) & 0xFF, ACK_CHECK_EN );                // MSB for reg address
    i2c_master_write_byte( cmd, ( reg_addr & 0xFF ), ACK_CHECK_EN );                    // LSB for reg address

    // restart
    i2c_master_start( cmd );

    // set address, read bit, and msb/lsb of register address
    i2c_master_write_byte( cmd, ( SGTL5000_I2C_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN );  // Address + read
    i2c_master_read( cmd, byte_val + 1, 1, ACK_VAL );                                   // MSB for reg data
    i2c_master_read_byte( cmd, byte_val, NACK_VAL );                                    // LSB for reg data

    // stop
    i2c_master_stop( cmd );

    // execute and return status - returns 0 for no error
    ret = i2c_master_cmd_begin( i2c_num, cmd, 1000 / portTICK_RATE_MS );                // Execute and return status, should return 0

    // delete i2c command link
    i2c_cmd_link_delete( cmd );                                                         // Delete i2c command link

    // return command status
    return ret;
}

 //------------------------------------------------------------------------------
 // sgtl5000_i2c_init - i2c master initialization
 //------------------------------------------------------------------------------
void sgtl5000_i2c_init()
{
    int i2c_master_port = SGTL5000_I2C_NUM;

    i2c_config_t conf;

    conf.mode               = I2C_MODE_MASTER;
    conf.sda_io_num         = SGTL5000_I2C_SDA_IO;
    conf.sda_pullup_en      = GPIO_PULLUP_ENABLE;
    conf.scl_io_num         = SGTL5000_I2C_SCL_IO;
    conf.scl_pullup_en      = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed   = SGTL5000_I2C_FREQ_HZ;

    i2c_param_config( i2c_master_port, &conf );
    i2c_driver_install( i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0 );
}

//------------------------------------------------------------------------------
// sgtl5000_i2s_init - initialize i2s bus for this chip
//------------------------------------------------------------------------------
void sgtl5000_i2s_init()
{
    i2s_driver_install( I2S_NUM, &sgtl5000_i2s_config, 0 , NULL );
    i2s_set_pin( I2S_NUM, &sgtl5000_pin_config );

    // Enable MCLK output
    WRITE_PERI_REG( PIN_CTRL, READ_PERI_REG( PIN_CTRL ) & 0xFFFFFFF0 );
    PIN_FUNC_SELECT( PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1 );
}


#ifdef __cplusplus
} // extern "C"
#endif


//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
