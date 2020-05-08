/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/* Attention!
 *  To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#include "ble_dfu.h"
#include <string.h>
#include "ble_hci.h"
#include "sdk_macros.h"
#include "ble_srv_common.h"
#include "nrf_nvic.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_log.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_bootloader_info.h"
#include "nrf_svci_async_function.h"
#include "nrf_pwr_mgmt.h"
#include "peer_manager.h"
#include "gatts_cache_manager.h"
#include "peer_id.h"

#define MAX_CTRL_POINT_RESP_PARAM_LEN   3                           /**< Max length of the responses. */
#define NRF_DFU_ADV_NAME_MAX_LENGTH     (20)

#define BLE_DFU_SERVICE_UUID            0xFE59                      /**< The 16-bit UUID of the Secure DFU Service. */

static ble_dfu_t             m_dfu;                      /**< Structure holding information about the Buttonless Secure DFU Service. */
static nrf_dfu_adv_name_t   m_adv_name;

void ble_dfu_on_sys_evt(uint32_t sys_evt, void * p_context);
void ble_dfu_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
uint32_t nrf_dfu_svci_vector_table_set(void);
uint32_t nrf_dfu_svci_vector_table_unset(void);

/**@brief Define functions for async interface to set new advertisement name for DFU mode.  */
NRF_SVCI_ASYNC_FUNC_DEFINE(NRF_DFU_SVCI_SET_ADV_NAME, nrf_dfu_set_adv_name, nrf_dfu_adv_name_t);

NRF_SDH_SOC_OBSERVER(m_dfu_soc_obs, BLE_DFU_SOC_OBSERVER_PRIO, ble_dfu_on_sys_evt, NULL);
NRF_SDH_BLE_OBSERVER(m_dfus_obs, BLE_DFU_BLE_OBSERVER_PRIO, ble_dfu_on_ble_evt, &m_dfu);

static uint32_t ble_dfu_resp_send(ble_dfu_op_code_t op_code, ble_dfu_rsp_code_t rsp_code)
{
    // Send indication
    uint32_t                err_code;
    const uint16_t          len = MAX_CTRL_POINT_RESP_PARAM_LEN;
    uint16_t                hvx_len;
    uint8_t                 hvx_data[MAX_CTRL_POINT_RESP_PARAM_LEN];
    ble_gatts_hvx_params_t  hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_len     = len;
    hvx_data[0] = DFU_OP_RESPONSE_CODE;
    hvx_data[1] = (uint8_t)op_code;
    hvx_data[2] = (uint8_t)rsp_code;

    hvx_params.handle = m_dfu.control_point_char.value_handle;
    hvx_params.type   = BLE_GATT_HVX_INDICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = hvx_data;

    err_code = sd_ble_gatts_hvx(m_dfu.conn_handle, &hvx_params);
    if ((err_code == NRF_SUCCESS) && (hvx_len != len))
    {
        err_code = NRF_ERROR_DATA_SIZE;
    }

    return err_code;
}


/**@brief Function for setting an advertisement name.
 *
 * @param[in]   adv_name    The new advertisement name.
 *
 * @retval NRF_SUCCESS      Advertisement name was successfully set.
 * @retval DFU_RSP_BUSY     Advertisement name was not set because of an ongoing operation.
 * @retval Any other errors from the SVCI interface call.
 */
static uint32_t set_adv_name(nrf_dfu_adv_name_t * p_adv_name)
{
    uint32_t err_code;

    if (m_dfu.is_waiting_for_svci)
    {
        return DFU_RSP_BUSY;
    }

    err_code = nrf_dfu_set_adv_name(p_adv_name);
    if (err_code == NRF_SUCCESS)
    {
        // The request was accepted.
        m_dfu.is_waiting_for_svci = true;
    }
    else if (err_code == NRF_ERROR_FORBIDDEN)
    {
        NRF_LOG_ERROR("The bootloader has write protected its settings page. This prohibits setting the advertising name. "\
                      "The bootloader must be compiled with NRF_BL_SETTINGS_PAGE_PROTECT=0 to allow setting the advertising name.");
    }

    return err_code;
}


/**@brief Function for entering the bootloader.
 */
static uint32_t enter_bootloader()
{
    uint32_t err_code;

    if (m_dfu.is_waiting_for_svci)
    {
        // We have an ongoing async operation. Entering bootloader mode is not possible at this time.
        err_code = ble_dfu_resp_send(DFU_OP_ENTER_BOOTLOADER, DFU_RSP_BUSY);
        APP_ERROR_CHECK(err_code);

        return NRF_SUCCESS;
    }

    // Set the flag indicating that we expect DFU mode.
    // This will be handled on acknowledgement of the characteristic indication.
    m_dfu.is_waiting_for_reset = true;

    err_code = ble_dfu_resp_send(DFU_OP_ENTER_BOOTLOADER, DFU_RSP_SUCCESS);
    if (err_code != NRF_SUCCESS)
    {
        m_dfu.is_waiting_for_reset = false;
    }

    return err_code;
}

static uint32_t ble_dfu_bootloader_start(void)
{
    uint32_t err_code;

    err_code = sd_power_gpregret_clr(0, 0xffffffff);
    VERIFY_SUCCESS(err_code);

    err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
    VERIFY_SUCCESS(err_code);

    NRF_LOG_INFO("Device will enter bootloader mode.");

    // Signal that DFU mode is to be enter to the power management module
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);

    return NRF_SUCCESS;
}

/**@brief Write authorization request event handler.
 *
 * @details The write authorization request event handler is called when writing to the control point.
 *
 * @param[in]   p_ble_evt Event received from the BLE stack.
 */
static void on_rw_authorize_req(ble_evt_t const * p_ble_evt)
{
    ble_gatts_rw_authorize_reply_params_t write_authorize_reply;
    uint8_t cccd_val[2];
    ble_gatts_value_t gatts_value;
    ble_dfu_op_code_t op_code;
    ble_dfu_rsp_code_t rsp_code = DFU_RSP_OPERATION_FAILED;
    uint8_t adv_name_length = 0;
    uint32_t err_code;

    if (p_ble_evt->evt.gatts_evt.conn_handle != m_dfu.conn_handle)
    {
        return;
    }

    const ble_gatts_evt_rw_authorize_request_t * p_auth_req = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    
    // Verify the authorization
    if (
        (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)                            &&
        (p_auth_req->request.write.handle == m_dfu.control_point_char.value_handle)     &&
        (p_auth_req->request.write.op != BLE_GATTS_OP_PREP_WRITE_REQ)                   &&
        (p_auth_req->request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)               &&
        (p_auth_req->request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)
       )
    {
        
        memset(&write_authorize_reply, 0, sizeof(write_authorize_reply));
        write_authorize_reply.type   = BLE_GATTS_AUTHORIZE_TYPE_WRITE;

        gatts_value.p_value = cccd_val;
        gatts_value.len = 2;
        gatts_value.offset = 0;
        err_code = sd_ble_gatts_value_get(m_dfu.conn_handle, m_dfu.control_point_char.cccd_handle, &gatts_value);
        if (err_code == NRF_SUCCESS && ble_srv_is_indication_enabled(cccd_val))
        {
            write_authorize_reply.params.write.update      = 1;
            write_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        }
        else
        {
            write_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_CPS_CCCD_CONFIG_ERROR;
        }

        // Authorize the write request
        do {
            err_code = sd_ble_gatts_rw_authorize_reply(m_dfu.conn_handle, &write_authorize_reply);
        } while (err_code == NRF_ERROR_BUSY);


        if (write_authorize_reply.params.write.gatt_status != BLE_GATT_STATUS_SUCCESS)
        {
            return;
        }

        // Forward the write event to the Buttonless DFU module.
        op_code = p_auth_req->request.write.data[0];
        switch (op_code)
        {
            case DFU_OP_ENTER_BOOTLOADER:
                err_code = enter_bootloader();
                if (err_code == NRF_SUCCESS)
                {
                    rsp_code = DFU_RSP_SUCCESS;
                }
                else if (err_code == NRF_ERROR_BUSY)
                {
                    rsp_code = DFU_RSP_BUSY;
                }
                break;

            case DFU_OP_SET_ADV_NAME:
                adv_name_length = p_auth_req->request.write.data[1];
                if((adv_name_length > NRF_DFU_ADV_NAME_MAX_LENGTH) || (adv_name_length == 0))
                {
                    // New advertisement name too short or too long.
                    rsp_code = DFU_RSP_ADV_NAME_INVALID;
                }
                else
                {
                    memcpy(m_adv_name.name, &p_auth_req->request.write.data[2], adv_name_length);
                    m_adv_name.len = adv_name_length;
                    err_code = set_adv_name(&m_adv_name);
                    if (err_code == NRF_SUCCESS)
                    {
                        rsp_code = DFU_RSP_SUCCESS;
                    }
                }
                break;

            default:
                rsp_code = DFU_RSP_OP_CODE_NOT_SUPPORTED;
                break;
        }

        // Report back in case of error
        if (rsp_code != DFU_RSP_SUCCESS)
        {
            err_code = ble_dfu_resp_send(op_code, rsp_code);
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**@brief Function for handling the HVC events.
 *
 * @details Handles HVC events from the BLE stack.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_hvc(ble_evt_t const * p_ble_evt)
{
    uint32_t err_code;
    ble_gatts_evt_hvc_t const * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == m_dfu.control_point_char.value_handle)
    {
        // Enter bootloader if we were waiting for reset after hvc indication confimation.
        if (m_dfu.is_waiting_for_reset)
        {
            err_code = ble_dfu_bootloader_start();
            APP_ERROR_CHECK(err_code);
        }
    }
}


void ble_dfu_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    VERIFY_PARAM_NOT_NULL_VOID(p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_dfu.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (m_dfu.conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
            {
                 m_dfu.conn_handle = BLE_CONN_HANDLE_INVALID;
            }
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_req(p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            on_hvc(p_ble_evt);
            break;

        default:
            // no implementation
            break;
    }
}

void ble_dfu_on_sys_evt(uint32_t sys_evt, void * p_context)
{
    uint32_t err_code;

    if (!nrf_dfu_set_adv_name_is_initialized())
    {
        return;
    }

    err_code = nrf_dfu_set_adv_name_on_sys_evt(sys_evt);
    if (err_code == NRF_ERROR_INVALID_STATE)
    {
        // The system event is not from an operation started by buttonless DFU.
        // No action is taken, and nothing is reported.
    }
    else if (err_code == NRF_SUCCESS)
    {
        // The async operation is finished.
        // Set the flag indicating that we are waiting for indication response
        // to activate the reset.
        m_dfu.is_waiting_for_svci = false;

        // Report back the positive response
        err_code = ble_dfu_resp_send(DFU_OP_SET_ADV_NAME, DFU_RSP_SUCCESS);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // Invalid error code reported back.
        m_dfu.is_waiting_for_svci = false;

        err_code = ble_dfu_resp_send(DFU_OP_SET_ADV_NAME, DFU_RSP_BUSY);
        APP_ERROR_CHECK(err_code);

        // Report the failure to enter DFU mode
        //mp_dfu->evt_handler(BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED);
    }
}

void ble_dfu_async_svci_init(void)
{
    uint32_t ret_val;

    ret_val = nrf_dfu_svci_vector_table_set();
    APP_ERROR_CHECK(ret_val);

    ret_val = nrf_dfu_set_adv_name_init();
    APP_ERROR_CHECK(ret_val);

    ret_val = nrf_dfu_svci_vector_table_unset();
    APP_ERROR_CHECK(ret_val);
}

uint32_t ble_dfu_init(void)
{
    uint32_t        err_code;
    ble_uuid_t      service_uuid;
    ble_uuid128_t   nordic_base_uuid = BLE_NORDIC_VENDOR_BASE_UUID;
    ble_add_char_params_t add_char_params;
    uint8_t init_value;

    // Initialize the service structure.
    m_dfu.conn_handle                  = BLE_CONN_HANDLE_INVALID;
    m_dfu.is_waiting_for_reset         = false;

    BLE_UUID_BLE_ASSIGN(service_uuid, BLE_DFU_SERVICE_UUID);

    // Add the DFU service declaration.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &(m_dfu.service_handle));
    VERIFY_SUCCESS(err_code);

    // Add vendor specific base UUID to use with the Buttonless DFU characteristic.
    err_code = sd_ble_uuid_vs_add(&nordic_base_uuid, &m_dfu.uuid_type);
    VERIFY_SUCCESS(err_code);

    // Add the Buttonless DFU Characteristic (without bonds).
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = BLE_DFU_CTRL_POINT_CHAR_UUID;
    add_char_params.uuid_type           = m_dfu.uuid_type;
    add_char_params.char_props.indicate = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.is_defered_write    = true;
    add_char_params.is_var_len          = true;
    add_char_params.max_len             = BLE_GATT_ATT_MTU_DEFAULT;

    add_char_params.cccd_write_access = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.read_access       = SEC_OPEN;

    err_code = characteristic_add(m_dfu.service_handle, &add_char_params, &m_dfu.control_point_char);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    init_value = 1;
    // Add the firmware version 
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = BLE_DFU_FIRMWARE_VERSION_CHAR_UUID;
    add_char_params.uuid_type           = m_dfu.uuid_type;
    add_char_params.init_len            = 1;
    add_char_params.max_len             = 1;
    add_char_params.p_init_value        = &init_value;
    add_char_params.char_props.read     = 1;
    add_char_params.read_access         = SEC_OPEN;

    return characteristic_add(m_dfu.service_handle, &add_char_params, &m_dfu.firmware_version_char);
}
