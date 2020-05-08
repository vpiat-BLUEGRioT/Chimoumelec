/*
******************************************************************************
* @file			drv_pwm.c
* @brief 		Driver for PWM control
* @version              V1
* @date 		08/05/2020
* @author               Valentin PIAT
******************************************************************************
*/
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "drv_pwm.h"
#include "nrf_log.h"
#include "nrfx_pwm.h"

/*
******************************************************************************
* Define
******************************************************************************
*/
#define PWM_TOP_VALUE           2500
/*
******************************************************************************
* Local Variable
******************************************************************************
*/
static nrfx_pwm_t m_pwm_inst = NRFX_PWM_INSTANCE(0);
static uint16_t pwm_value;
/*
******************************************************************************
* Local Function
******************************************************************************
*/

static void drv_pwm_interrupt_handler(nrfx_pwm_evt_type_t event)
{
    // NOTHING TODO
}

/*
******************************************************************************
* Global Function
******************************************************************************
*/
void drv_pwm_increase_dutycycle(void)
{
    pwm_value = pwm_value > PWM_TOP_VALUE ? PWM_TOP_VALUE : pwm_value + 10;
    NRF_LOG_INFO("pwm_value %d", pwm_value);
}

void drv_pwm_decrease_dutycycle(void)
{
    pwm_value = pwm_value < 10 ? DRV_PWM_0 : pwm_value - 10;
    NRF_LOG_INFO("pwm_value %d", pwm_value);
}


void drv_pwm_change_dutycycle(drv_pwm_dutycycle_t dutycycle)
{
    switch(dutycycle)
    {
        case DRV_PWM_0:
            pwm_value = 0;
            break;
        case DRV_PWM_10:
            pwm_value = PWM_TOP_VALUE / 10;
            break;
        case DRV_PWM_20:
            pwm_value = PWM_TOP_VALUE / 5;
            break;
        case DRV_PWM_30:
            pwm_value = PWM_TOP_VALUE * 30 /100;
            break;
        case DRV_PWM_40:
            pwm_value = PWM_TOP_VALUE * 40 /100;
            break;
        case DRV_PWM_50:
            pwm_value = PWM_TOP_VALUE / 2;
            break;
        case DRV_PWM_60:
            pwm_value = PWM_TOP_VALUE * 60 /100;
            break;
        case DRV_PWM_70:
            pwm_value = PWM_TOP_VALUE * 70 /100;
            break;
        case DRV_PWM_80:
            pwm_value = PWM_TOP_VALUE * 80 /100;
            break;
        case DRV_PWM_90:
            pwm_value = PWM_TOP_VALUE * 90 /100;
            break;
        case DRV_PWM_100:
            pwm_value = PWM_TOP_VALUE;
            break;
        default:
            NRF_LOG_ERROR("PWM Bad Duty Cycle");
            break;
    }
}

uint32_t drv_pwm_init(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    nrfx_pwm_config_t const pwm_config =
    {
        .output_pins =
        {
            13,
            4,
            NRFX_PWM_PIN_NOT_USED,
            NRFX_PWM_PIN_NOT_USED,
        },
        .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .base_clock   = NRF_PWM_CLK_250kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = PWM_TOP_VALUE,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

    err_code = nrfx_pwm_init(&m_pwm_inst, &pwm_config, drv_pwm_interrupt_handler);
    if(err_code != NRF_SUCCESS)
    {
        return NRF_SUCCESS;
    }

    pwm_value = PWM_TOP_VALUE;
    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = &pwm_value,
        .length          = 1,
        .repeats         = 0,
        .end_delay       = 0
    };

    err_code = nrfx_pwm_simple_playback(&m_pwm_inst, &seq, 1, NRFX_PWM_FLAG_LOOP);

    return err_code;
}