#ifndef __BLDCM_CONTROL_H_
#define __BLDCM_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "stm32f4xx.h"  // Device header
#include "tim.h"
#include "foc_utils.h"

#define BLDCM_ENABLE_SD()     HAL_GPIO_WritePin(MOTOR_SD_GPIO_Port, MOTOR_SD_Pin, GPIO_PIN_SET)
#define BLDCM_DISABLE_SD()    HAL_GPIO_WritePin(MOTOR_SD_GPIO_Port, MOTOR_SD_Pin, GPIO_PIN_RESET)

#define SPEED_FILTER_NUM      10

// ADC conversion
#define VREF                  3.3f
#define GET_ADC_VDC_VAL(val)  ((float)val / 4096.0f * VREF)
#define GET_VBUS_VAL(val)     (((float)val - 1.24f) * 37.0f)
#define GET_ADC_CURR_VAL(val) (((float)val) / (float)8.0 / (float)0.02 * (float)1000.0)

#define PWM1_DUTY(duty)       __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty)
#define PWM2_DUTY(duty)       __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty)
#define PWM3_DUTY(duty)       __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty)

#define CLOSE_PWM1_LOWER \
    HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_PORT, MOTOR_OCNPWM1_PIN, GPIO_PIN_RESET)

typedef enum {
    MOTOR_FWD = 0,
    MOTOR_REV,
} motor_dir_t;

typedef struct
{
    uint8_t enable_flag;
    motor_dir_t direction;
    float speed;
    uint16_t duty;
    int32_t speed_group[SPEED_FILTER_NUM];
} motor_rotate_t;

typedef struct
{
    float v_bus;
    float temp;
    int32_t u_curr;
    int32_t v_curr;
    int32_t w_curr;
} adc_value;




void loop(void);

void PrintMotorInformation();

void BLDCM_Enable(void);

void BLDCM_Disable(void);

uint8_t GetHallState(void);

void UpdateMotorSpeed(uint8_t dir_in, uint32_t time);

void UpdateSpeedDir(uint8_t dir_in);

uint16_t GetBLDCMPulse(void);

adc_value GetADCValue(void);

PhaseCurrent_s GetPhaseCurrents(void);

#ifdef __cplusplus
}
#endif

#endif
