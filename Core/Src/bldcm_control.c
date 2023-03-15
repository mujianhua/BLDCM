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

// seq 1 > 5 > 4 > 6 > 2 > 3 > 1     000 001 010 011 100 101 110 111
const int8_t ELECTRIC_SECTORS[8] = {-1, 0, 4, 5, 2, 1, 3, -1};
volatile int8_t electric_sector;

int16_t adc_buffer[4] = {0};
static motor_rotate_t motor_drive = {0};
Direction__ direction;

adc_value adc_value_;
uint16_t bldcm_pulse = 0;
uint8_t shaft_electrical_count = 0;

void init(void)
{
    // ADC
    HAL_ADCEx_InjectedStart(&hadc3);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOC);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // ADC TIM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2000);
    uint8_t ii = 20;
    while (ii--) {
        GetADCValue();
        HAL_Delay(1);
    }
    // Drive plate.
    BLDCM_ENABLE_SD();  // driver plate MOTOR SD enable.
    initFOC();
}

uint8_t flag = 0;
void loop(void)
{
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
        GetADCValue();
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
    //    printf("motor shaft angle: %f\n", motor_drive.shaft_angle);
}

void BLDCM_Enable(void)
{
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
    // FLAG
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
        f = (1.0f / (84000000.0f / HALL_TIM_PSC + 1) * time);
        f = (1.0f / 24.0f) / (f / 60.0f);
        motor_drive.speed_group[count++] = f;
    }
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
    UpdateMotorAngle(time);
}

void UpdateMotorAngle(uint32_t time)
{
    float add_angle = motor_drive.speed * _PI_30 * (1.0f / (84000000.0f / HALL_TIM_PSC + 1) * time);
    motor_drive.shaft_angle = _normalizeAngle(
      add_angle + motor_drive.electrical_angle / POLE_OF_PAIRS + _PI_2 * shaft_electrical_count);
}

uint16_t GetBLDCMPulse(void) { return bldcm_pulse; }

adc_value * GetADCValue(void)
{
    static uint8_t flag = 0;
    static uint16_t adc_offest[4] = {0};
    if (flag < 17) {
        for (uint8_t i = 0; i < 4; i++) {
            adc_offest[i] = adc_buffer[i];
        }
        flag++;
    }
    for (uint8_t i = 0; i < 4; i++) {
        if (adc_buffer[i] > adc_offest[i]) {
            adc_buffer[i] -= adc_offest[i];
        } else {
            adc_buffer[i] = 0;
        }
    }

    adc_value_.u_curr = GET_ADC_CURR_VAL(GET_ADC_VDC_VAL(adc_buffer[0]));
    adc_value_.v_curr = GET_ADC_CURR_VAL(GET_ADC_VDC_VAL(adc_buffer[1]));
    adc_value_.w_curr = GET_ADC_CURR_VAL(GET_ADC_VDC_VAL(adc_buffer[2]));
    adc_value_.v_bus = GET_VBUS_VAL(GET_ADC_VDC_VAL(adc_buffer[3]));

    return &adc_value_;
}

PhaseCurrent_s GetPhaseCurrents(void)
{
    PhaseCurrent_s phase_current;
    return phase_current;
}

/**************************************** IRQ Callback ****************************************/
/**
 * @brief Get current signal from ADC_DMA.
 */
// TODO: test ADC value.
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
// {
//     HAL_ADC_Stop_DMA(hadc);
//     HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&adc_buffer, 5);
// }

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef * hadc)
{
    adc_buffer[0] = hadc->Instance->JDR1;
    adc_buffer[1] = hadc->Instance->JDR2;
    adc_buffer[2] = hadc->Instance->JDR3;
    adc_buffer[3] = hadc->Instance->JDR4;
}

/**
 * @brief get hall signal.
 */
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef * htim)
{
    uint8_t hall_state = 0;
    hall_state = GetHallState();
    int8_t new_electric_sector = ELECTRIC_SECTORS[hall_state];
    if (new_electric_sector - electric_sector > 3) {
        direction = CCW;
        motor_drive.electric_rotations += direction;
    } else if (new_electric_sector - electric_sector < (-3)) {
        direction = CW;
        motor_drive.electric_rotations += direction;
    } else {
        direction = (new_electric_sector > electric_sector) ? CW : CCW;
    }
    electric_sector = new_electric_sector;
    motor_drive.shaft_angle = _normalizeAngle(
      (float)(motor_drive.electric_rotations * 6 + electric_sector) / (float)CPR * _2PI);

#if PRINT_HALL_INFORMATION
    printf("shaft angle: %f\n", motor_drive.shaft_angle);
    printf("hall: %d, sector: %d, Direction: %d\n", hall_state, electric_sector, direction);
#endif

    if (htim == &htim5) {
        // UpdateMotorSpeed(hall_state, __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1));
    }
    uint16_t bldcm_pulse = 0;
    switch (hall_state) {
        case 1: /* U+ W- */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_RESET);

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_RESET);

            // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, bldcm_pulse);
            PWM1_DUTY(bldcm_pulse);
            HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_SET);
            motor_drive.electrical_angle = _11PI_6;
            shaft_electrical_count++;
            shaft_electrical_count = _constrain(shaft_electrical_count, 0, 3);
            break;

        case 2: /* V+ U- */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_RESET);

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_RESET);

            // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, bldcm_pulse);
            PWM2_DUTY(bldcm_pulse);
            HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_SET);
            motor_drive.electrical_angle = _PI_2;
            break;

        case 3: /* V+ W- */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_RESET);

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_RESET);

            // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, bldcm_pulse);
            PWM2_DUTY(bldcm_pulse);
            HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_SET);
            motor_drive.electrical_angle = _PI_6;
            break;

        case 4: /* W+ V- */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_RESET);

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_RESET);

            // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, bldcm_pulse);
            PWM3_DUTY(bldcm_pulse);
            HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_SET);
            motor_drive.electrical_angle = _7PI_6;
            break;

        case 5: /* U+  V-*/
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_RESET);

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_RESET);

            // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, bldcm_pulse);
            PWM1_DUTY(bldcm_pulse);
            HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_SET);
            motor_drive.electrical_angle = _3PI_2;
            break;

        case 6: /* W+ U- */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM2_GPIO_Port, MOTOR_OCNPWM2_Pin, GPIO_PIN_RESET);

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
            HAL_GPIO_WritePin(MOTOR_OCNPWM3_GPIO_Port, MOTOR_OCNPWM3_Pin, GPIO_PIN_RESET);

            // __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, bldcm_pulse);
            PWM3_DUTY(bldcm_pulse);
            HAL_GPIO_WritePin(MOTOR_OCNPWM1_GPIO_Port, MOTOR_OCNPWM1_Pin, GPIO_PIN_SET);
            motor_drive.electrical_angle = _5PI_6;
            break;
    }
    HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM);
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