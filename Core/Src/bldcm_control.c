/**
 * @file bldc_control.c
 * @author mujianhua
 */

#include "bldcm_control.h"

#include "adc.h"
#include "foc.h"
#include "foc_utils.h"
#include "gpio.h"
#include "main.h"
#include "stm32f4xx.h"  // Device header
#include "tim.h"

#include <stdio.h>

static motor_rotate_t motor_drive = {0};

adc_value adc_value_;

uint16_t bldcm_pulse = 0;

void loop(void)
{
    uint8_t flag = 0;
    if (Key_Scan(KEY1_GPIO_Port, KEY1_Pin) == KEY_ON) {
        printf("enable the motor\n");
        BLDCM_Enable();
    }
    if (Key_Scan(KEY2_GPIO_Port, KEY2_Pin) == KEY_ON) {
        printf("disable the motor\n");
        BLDCM_Disable();
    }
#if 1
    if (HAL_GetTick() % 1000 == 0 && flag == 0) {
        flag = 1;
        PrintMotorInformation();
    } else if (HAL_GetTick() % 1000 != 0 && flag == 1) {
        flag = 0;
    }
#endif
}

void PrintMotorInformation()
{
    printf(
      "%f, %f V, %d mA, %d mA, %d mA\n", adc_value_.temp, adc_value_.v_bus, adc_value_.u_curr,
      adc_value_.v_curr, adc_value_.w_curr);
}

void BLDCM_Enable(void)
{
    BLDCM_ENABLE_SD();  // driver plate MOTOR SD enable.
    // HALL
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_TRIGGER);  // Enable HALL Trigger IRQ
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
    HAL_TIMEx_HallSensor_Start(&htim5);  // todo:...
    HAL_TIM_TriggerCallback(NULL);
    // PWM
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    // ADC
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);               // ADC TIM
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&adc_buffer, 5);  // Open ADC_DMA
    motor_drive.enable_flag = 1;
}

void BLDCM_Disable(void)
{
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_TRIGGER);
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
    HAL_TIMEx_HallSensor_Stop(&htim5);
    // stop PWM
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    motor_drive.enable_flag = 0;
}

uint8_t GetHallState(void)
{
    uint8_t state = 0;
#if 1
    if (HAL_GPIO_ReadPin(HALL_INPUT1_GPIO_Port, HALL_INPUT1_Pin) != GPIO_PIN_RESET) {
        state |= 0x01U;
    }
    if (HAL_GPIO_ReadPin(HALL_INPUT2_GPIO_Port, HALL_INPUT2_Pin) != GPIO_PIN_RESET) {
        state |= 0x02U;
    }
    if (HAL_GPIO_ReadPin(HALL_INPUT3_GPIO_Port, HALL_INPUT3_Pin) != GPIO_PIN_RESET) {
        state |= 0x04U;
    }
#else
    state = (GPIOH->IDR >> 10) & 7;
#endif
    return state;
}

static uint8_t count = 0;
void UpdateMotorSpeed(uint8_t dir_in, uint32_t time)
{
    int speed_temp = 0;
    static int flag = 0;
    float f = 0;
    if (time == 0)
        motor_drive.speed_group[count++] = 0;
    else {
        f = (1.0f / (84000000.0f / 128) * time);
        f = (1.0f / 12.0f) / (f / 60.0f);
        motor_drive.speed_group[count++] = f;
    }
    UpdateSpeedDir(dir_in);
    //	motor_drive.speed = motor_drive.speed_group[count-1];
    if (count >= SPEED_FILTER_NUM) {
        flag = 1;
        count = 0;
    }
    //	return ;
    speed_temp = 0;

    if (flag) {
        for (uint8_t c = 0; c < SPEED_FILTER_NUM; c++) {
            speed_temp += motor_drive.speed_group[c];
        }
        motor_drive.speed = speed_temp / SPEED_FILTER_NUM;
    } else {
        for (uint8_t c = 0; c < count; c++) {
            speed_temp += motor_drive.speed_group[c];
        }
        motor_drive.speed = speed_temp / count;
    }
}

void UpdateSpeedDir(uint8_t dir_in)
{
    uint8_t step[6] = {1, 3, 2, 6, 4, 5};

    static uint8_t num_old = 0;
    uint8_t step_loc = 0;  // 记录当前霍尔位置
    int8_t dir = 1;
    for (step_loc = 0; step_loc < 6; step_loc++) {
        if (step[step_loc] == dir_in) {
            break;
        }
    }
    // 端点处理
    if (step_loc == 0) {
        if (num_old == 1) {
            dir = 1;
        } else if (num_old == 5) {
            dir = -1;
        }
    } else if (step_loc == 5) {
        if (num_old == 0) {
            dir = 1;
        } else if (num_old == 4) {
            dir = -1;
        }
    } else if (step_loc > num_old) {
        dir = -1;
    } else if (step_loc < num_old) {
        dir = 1;
    }
    if (dir == 1)
        motor_drive.direction = MOTOR_FWD;
    else
        motor_drive.direction = MOTOR_REV;
    num_old = step_loc;
    //  motor_drive.speed *= dir;;
    motor_drive.speed_group[count - 1] *= dir;
}

uint16_t GetBLDCMPulse(void) { return bldcm_pulse; }

adc_value GetADCValue(void)
{
    float vdc_u = GET_ADC_VDC_VAL(adc_buffer[2]);
    adc_value_.u_curr = GET_ADC_CURR_VAL(vdc_u);
    return adc_value_;
}

PhaseCurrent_s GetPhaseCurrents(void)
{
    PhaseCurrent_s phase_current;
    return phase_current;
}

/****************************************** IRQ Callback****************************************/
/**
 * @brief Get current signal from ADC_DMA.
 */
// TODO: test ADC value.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
    HAL_ADC_Stop_DMA(hadc);

    adc_value_.temp = adc_buffer[0];
    adc_value_.v_bus = GET_VBUS_VAL(GET_ADC_VDC_VAL(adc_buffer[1]));
    adc_value_.u_curr = GET_ADC_CURR_VAL(GET_ADC_VDC_VAL(adc_buffer[2]));
    adc_value_.v_curr = GET_ADC_CURR_VAL(GET_ADC_VDC_VAL(adc_buffer[3]));
    adc_value_.w_curr = GET_ADC_CURR_VAL(GET_ADC_VDC_VAL(adc_buffer[4]));
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&adc_buffer, 5);
}

/**
 * @brief get hall signal.
 */
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef * htim)
{
    uint8_t step = 0;
    step = GetHallState();
#if 0
    printf("trigger IRQ.\n");
	printf("hall: %d", step);
#endif
    if (htim == &htim5) {
        UpdateMotorSpeed(step, __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1));
    }
    uint16_t bldcm_pulse = 2000;
    switch (step) {
        case 1: /* U+ W- */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_RESET);

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_RESET);

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, bldcm_pulse);
            HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_SET);
            break;

        case 2:                                               /* V+ U- */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);  // 通道 3 配置丿 0
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_RESET);  // 关闭下桥臿

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // 通道 1 配置丿 0
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_RESET);  // 关闭下桥臿

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, bldcm_pulse);  // 通道 2
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_SET);  // 弿启下桥臂

            break;

        case 3:                                               /* V+ W- */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // 通道 1 配置丿 0
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_RESET);  // 关闭下桥臿

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);  // 通道 3 配置丿 0
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_RESET);  // 关闭下桥臿

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, bldcm_pulse);  // 通道 2
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_SET);  // 弿启下桥臂
            break;

        case 4:                                               /* W+ V- */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // 通道 1 配置丿 0
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_RESET);  // 关闭下桥臿

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // 通道 2 配置丿 0
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_RESET);  // 关闭下桥臿

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, bldcm_pulse);  // 通道 3
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_SET);  // 弿启下桥臂
            break;

        case 5:                                               /* U+  V -*/
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);  // 通道 3 配置丿 0
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_RESET);  // 关闭下桥臿

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // 通道 2 配置丿 0
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_RESET);  // 关闭下桥臿

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, bldcm_pulse);  // 通道 1
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_SET);  // 弿启下桥臂
            break;

        case 6:                                               /* W+ U- */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // 通道 2 配置丿 0
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_RESET);  // 关闭下桥臿

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // 通道 1 配置丿 0
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_RESET);  // 关闭下桥臿

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, bldcm_pulse);  // 通道 3
            HAL_GPIO_WritePin(
              MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_SET);  // 弿启下桥臂
            break;
    }
    HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM);  // 软件产生换相事件，此时才将配置写兿
}

/**
 * @brief Preventing motor blockage.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
#if 0
    printf("time IRQ.\n");
#endif
    // GetHallState();
}
/*********************************************************************************************/