#ifndef GPIO_EXP_I2C_H
#define GPIO_EXP_I2C_H

// GPIO Expander registers
#define EXPANDER_REGISTER_IODIRA        0x00
#define EXPANDER_REGISTER_IODIRB        0x01
#define EXPANDER_REGISTER_GPINTENA      0x04
#define EXPANDER_REGISTER_GPINTENB      0x05
#define EXPANDER_REGISTER_IOCON         0x0A
#define EXPANDER_REGISTER_GPPUA         0x0C
#define EXPANDER_REGISTER_GPPUB         0x0D
#define EXPANDER_REGISTER_GPIOA         0x12
#define EXPANDER_REGISTER_GPIOB         0x13
#define EXPANDER_REGISTER_OLATA         0x14
#define EXPANDER_REGISTER_OLATB         0x15

// LED Driver registers
#define LED_DRIVER_REGISTER_DAC         0x00

// ADC registers
#define ADC_REGISTER_CONVERSION      	0x07
#define ADC_REGISTER_CONF      			0x00
#define ADC_REGISTER_ADVANCED_CONF      0x0B
#define ADC_REGISTER_IN0        		0x20
#define ADC_REGISTER_IN1        		0x21
#define ADC_REGISTER_IN2        		0x22
#define ADC_REGISTER_IN3        		0x23
#define ADC_REGISTER_IN4        		0x24
#define ADC_REGISTER_IN5        		0x25
#define ADC_REGISTER_IN6        		0x26
#define ADC_REGISTER_IN7        		0x27

// GPIO pins
enum gpio_exp_pins
{
    GSM1_SLEEP_CTRL = 0,
    GSM2_SLEEP_CTRL,
    CM_FAN2,
    CM_AUDIO_EN,
    CM_SERVO_EN,
    POLE_LIGTH_CTRL,
    CM_BP3000_EN,
    OUT_SPARE,
    CM_CAM0_PWEN,
    CM_CAM0_RES,
    CM_CAM1_PWEN,
    CM_CAM1_RES,
    CM_CAM2_PWEN,
    CM_CAM2_RES,
    CM_CAM3_PWEN,
    CM_CAM3_RES
};

#endif
