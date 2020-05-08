/******************************************************************************
 * @file    main.c
 * @author  BLUEGRIoT
 * @version V0
 * @date    16/01/2020
 * @brief   Wavely Project
 *
 *****************************************************************************/
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
// nRF
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_gpiote.h"
#include "boards.h"
#include "bsp.h"

// logs
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Drivers
#include "drv_pwm.h"

// Services
#include "srv_pompe.h"

// Bluetooth
#include "ble_main.h"
#include "ble_dfu.h"

/*
******************************************************************************
* Define
******************************************************************************
*/
#define SCHED_QUEUE_SIZE 32                                   /**< Maximum number of events in the scheduler queue. */
#define SCHED_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum app_scheduler event size. */

/*
******************************************************************************
* Local Variable
******************************************************************************
*/
NRF_SECTION_DEF(rtt, uint8_t);

/*
******************************************************************************
* Function
******************************************************************************
*/
static void bsp_evt_handler(bsp_event_t evt)
{
    static drv_pwm_dutycycle_t dutycycle = DRV_PWM_0;

    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            drv_pwm_increase_dutycycle();
            break;

        case BSP_EVENT_KEY_1:
            drv_pwm_decrease_dutycycle();
            break;
        
        case BSP_EVENT_KEY_2:

            break;

        case BSP_EVENT_KEY_3:

            break;

        default:
            return;
    }
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t* p_file_name) {
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
    ret_code_t err_code;

    // Init power module
    err_code = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(err_code);

    // Init Power management
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Scheduler - Clock System - Timer
 */
static void timer_init(void) {
    ret_code_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);

    // Init Scheduler
    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing hardware
 */
static void hardware_init (void)
{
    ret_code_t err_code;

    // Initialize GPIOTE module
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_buttons_enable();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void) {
    // To share log between app and bootloader
    memset(NRF_SECTION_START_ADDR(rtt), 0, NRF_SECTION_LENGTH(rtt));

    // Initialize logs.
    log_init();

    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    //ble_dfu_async_svci_init();

    // Initialize timers
    timer_init();

    // Initialize power manager
    power_management_init();

    // Initialize chip.
    hardware_init();

    // Initialize BLE
    //BLE_Init();

    // Initalize service
    srv_pompe_init();

    NRF_LOG_INFO("Chifoumelec Start");

    for (;;) 
    {
        app_sched_execute();

        
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}