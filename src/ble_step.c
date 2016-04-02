/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

#include "ble_step.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"

#include "SEGGER_RTT.h"



#define INVALID_BATTERY_LEVEL 255

static uint8_t mac_address [BLE_GAP_ADDR_LEN];
static ble_gatts_char_handles_t mac_address_handles;
static ble_srv_security_mode_t 	mac_address_attr_md; 

static ble_gatts_char_handles_t target_handles;
static ble_srv_security_mode_t 	target_attr_md;




//read mac address
static void get_mac_addr(uint8_t *p_mac_addr)
{
		uint32_t error_code;
		ble_gap_addr_t *p_mac_addr_t = (ble_gap_addr_t*)malloc(sizeof(ble_gap_addr_t));
		error_code = sd_ble_gap_address_get(p_mac_addr_t);
		//APP_ERROR_CHECK(error_code);
		uint8_t *d = p_mac_addr_t->addr;
		for ( uint8_t i = 6; i >0;)
		{	
			i--;
			p_mac_addr[5-i]= d[i];
		}
		free(p_mac_addr_t);
		p_mac_addr_t = NULL;

		SEGGER_RTT_printf(0,"mac=0x%x",p_mac_addr[0]);
			
}


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_step_t * p_step, ble_evt_t * p_ble_evt)
{
    p_step->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_step_t * p_step, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_step->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_step_t * p_step, ble_evt_t * p_ble_evt)
{
    if (p_step->is_notification_supported)
    {
    	
		
        ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

		SEGGER_RTT_printf(0,"on_write:len=%d\r\n",p_evt_write->len);

        if (
            (p_evt_write->handle == p_step->step_count_handles.cccd_handle)
            &&
            (p_evt_write->len == 2)
           )
        {
            // CCCD written, call application event handler
            if (p_step->evt_handler != NULL)
            {
                ble_step_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = BLE_STEP_EVT_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = BLE_STEP_EVT_NOTIFICATION_DISABLED;
                }

                p_step->evt_handler(p_step, &evt);
            }
        }
    }
}


void ble_step_on_ble_evt(ble_step_t * p_step, ble_evt_t * p_ble_evt)
{
    if (p_step == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_step, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_step, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_step, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_bas        Battery Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t step_count_char_add(ble_step_t * p_step, const ble_step_init_t * p_step_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_step_count;
    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
    uint8_t             init_len;

    // Add Battery Level characteristic
    if (p_step->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        cccd_md.write_perm = p_step_init->step_count_char_attr_md.cccd_write_perm;
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = (p_step->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_step->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_WECHAT_CURRENT_PEDOMETER_MEASUREMENT);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_step_init->step_count_char_attr_md.read_perm;
    attr_md.write_perm = p_step_init->step_count_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    initial_step_count = p_step_init->initial_step_count;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 7;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   =7;
    attr_char_value.p_value   = &initial_step_count;

    err_code = sd_ble_gatts_characteristic_add(p_step->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_step->step_count_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    if (p_step_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

        memset(&attr_md, 0, sizeof(attr_md));

        attr_md.read_perm = p_step_init->step_count_report_read_perm;
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen    = 0;
        
        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_step_init->p_report_ref);
        
        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid    = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len  = init_len;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len   = attr_char_value.init_len;
        attr_char_value.p_value   = encoded_report_ref;

        err_code = sd_ble_gatts_descriptor_add(p_step->step_count_handles.value_handle,
                                               &attr_char_value,
                                               &p_step->report_ref_handle);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {
        p_step->report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}

//eric-han:for wehcat fec9
static uint32_t read_char_add(uint16_t  uuid,uint8_t len,uint8_t * p_value,
                         const ble_srv_security_mode_t * step_attr_md,
                         ble_gatts_char_handles_t      * p_handles,ble_step_t * p_step)
{
    ble_uuid_t          ble_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
	uint8_t temp=100;

    //APP_ERROR_CHECK_BOOL(p_char_value != NULL);
    //APP_ERROR_CHECK_BOOL(char_len > 0);

    // The ble_gatts_char_md_t structure uses bit fields. So we reset the memory to zero.
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, uuid);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = step_attr_md->read_perm;
    attr_md.write_perm = step_attr_md->write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = len;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = len;
    attr_char_value.p_value   = p_value;

    return sd_ble_gatts_characteristic_add(p_step->service_handle, &char_md, &attr_char_value, p_handles);
}

//------------------------------------------------------------
//eric-han:for wehcat fea2
static uint32_t indicate_char_add(uint16_t  uuid,uint8_t len,uint8_t * p_value,
                         const ble_srv_security_mode_t * step_attr_md,
                         ble_gatts_char_handles_t      * p_handles,ble_step_t * p_step)
{
    ble_uuid_t          ble_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
	uint8_t temp=100;

    memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.write = 1;
	char_md.char_props.indicate= 1;
    char_md.char_props.read  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, uuid);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = step_attr_md->read_perm;
    attr_md.write_perm = step_attr_md->write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = len;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = len;
    attr_char_value.p_value   = p_value;

    return sd_ble_gatts_characteristic_add(p_step->service_handle, &char_md, &attr_char_value, p_handles);
}


//--------------------------------------------------
uint32_t ble_step_init(ble_step_t * p_step, const ble_step_init_t * p_step_init)
{
    if (p_step == NULL || p_step_init == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
	uint8_t target[4]={0};
	
    // Initialize service structure
    p_step->evt_handler               = p_step_init->evt_handler;
    p_step->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_step->is_notification_supported = p_step_init->support_notification;
    p_step->step_count_last        = INVALID_BATTERY_LEVEL;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_WECHAT_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_step->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	get_mac_addr(mac_address);

    //mac address Characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&mac_address_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&mac_address_attr_md.write_perm);
    read_char_add(BLE_UUID_WECHAT_MACADD,BLE_GAP_ADDR_LEN,mac_address,
                            &mac_address_attr_md,
                            &mac_address_handles,p_step);
	//target Characteristic
	target[0]=1;
	target[1]=0x10;
	target[2]=0x27;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&target_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&target_attr_md.write_perm);
    indicate_char_add(BLE_UUID_WECHAT_TARGET,4,target,
                            &target_attr_md,
                            &target_handles,p_step);
    // Add battery level characteristic
    return step_count_char_add(p_step, p_step_init);
}


uint32_t ble_step_count_update(ble_step_t * p_step, uint8_t* step_count)
{
	 if (p_step == NULL)
    {
        return NRF_ERROR_NULL;
    }
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    if (step_count[1] != p_step->step_count_last)
    { 
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     =7;// sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value =step_count;// &step_count;

        // Update database.
        err_code = sd_ble_gatts_value_set(p_step->conn_handle,
                                          p_step->step_count_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new battery value.
            p_step->step_count_last = step_count[1];
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_step->conn_handle != BLE_CONN_HANDLE_INVALID) && p_step->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_step->step_count_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_step->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;

}
//---------------------------------------------------------------------------------
uint32_t ble_wechat_target_update(ble_step_t * p_step, uint8_t* target)
{
	 if (p_step == NULL)
    {
        return NRF_ERROR_NULL;
    }
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
    { 
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     =4;// sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value =target;// &step_count;
        // Update database.
        err_code = sd_ble_gatts_value_set(p_step->conn_handle,
                                          target_handles.value_handle,
                                          &gatts_value);
       
        // Send value if connected and notifying.
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = target_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_INDICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_step->conn_handle, &hvx_params);
        }
    }
			//SEGGER_RTT_printf(0,"0x%x,0x%x\r\n",target[0],target[1]);

    return err_code;
}

