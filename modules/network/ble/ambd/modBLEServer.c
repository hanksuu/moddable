/*
 * Copyright (c) 2016-2018  Moddable Tech, Inc.
 *
 *   This file is part of the Moddable SDK Runtime.
 *
 *   The Moddable SDK Runtime is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   The Moddable SDK Runtime is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with the Moddable SDK Runtime.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "xsmc.h"
#include "xsHost.h"
#include "mc.xs.h"
#include "modBLE.h"

#include <platform_opts_bt.h>
#include <os_sched.h>
#include <string.h>
#include <trace_app.h>
#include <gap.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <profile_server.h>
#include <gap_msg.h>
#include <bas.h>
#include <bte.h>
#include <gap_config.h>
#include <bt_flags.h>
#include <stdio.h>
#include "wifi_constants.h"
#include "FreeRTOS.h"
#include <wifi/wifi_conf.h>
#include "task.h"
#include "rtk_coex.h"
#include <os_msg.h>
#include <os_task.h>
#include <gap.h>
#include <gap_le.h>
#include <app_msg.h>
#include <basic_types.h>
#include <gap_conn_le.h>
#include "platform_stdlib.h"
#include <profile_server.h>
//#include <ble_peripheral_tmp/moddable_ble_service.c>
#include "mc.bleservices.c"

/*================= Moddable =================*/
typedef struct {
	xsMachine	*the;
	xsSlot		obj;

	// server
	//gatt_if_t gatts_if;
	//uint8_t *advertisingData;
	//uint8_t *scanResponseData;
	//ble_adv_params_t adv_params;
	//uint8_t adv_config_done;
	//uint8_t deviceNameSet;

	// services
	uint8_t deployServices;
	//uint16_t handles[service_count][max_attribute_count];

	// security
	//uint8_t encryption;
	//uint8_t bonding;
	//uint8_t mitm;
	//uint8_t iocap;

	// connection
	//rtk_bd_addr_t remote_bda;
	uint8_t conn_id;
	//uint16_t app_id;
} modBLERecord, *modBLE;

static modBLE gBLE = NULL;
static char_name_table *handleToCharName(uint8_t service_index, uint8_t handle);
static void addressToBuffer(uint8_t *address, uint8_t *buffer);
static void bufferToAddress(uint8_t *buffer, uint8_t *address);

/*================= Moddable end =============*/
//================= RTK start ============*/
#define APP_MAX_LINKS                               1

#define DEFAULT_ADVERTISING_INTERVAL_MIN            352 //220ms
#define DEFAULT_ADVERTISING_INTERVAL_MAX            384 //240ms

#define APP_TASK_PRIORITY                           2         //!< Task priorities
#define APP_TASK_STACK_SIZE                         256 * 10   //!< Task stack size
#define MAX_NUMBER_OF_GAP_MESSAGE                   0x20      //!< GAP message queue size
#define MAX_NUMBER_OF_IO_MESSAGE                    0x20      //!< IO message queue size
#define MAX_NUMBER_OF_EVENT_MESSAGE                 (MAX_NUMBER_OF_GAP_MESSAGE + MAX_NUMBER_OF_IO_MESSAGE)    //!< Event message queue size

// Service define
#define SIMP_READ_V1                                1
#define SIMP_WRITE_V2                               1
#define SIMP_NOTIFY_INDICATE_V3_ENABLE              1
#define SIMP_NOTIFY_INDICATE_V3_DISABLE             2
#define SIMP_NOTIFY_INDICATE_V4_ENABLE              3
#define SIMP_NOTIFY_INDICATE_V4_DISABLE             4
#define SIMP_READ_V1_MAX_LEN                        300

#define SIMP_BLE_SERVICE0_CHAR0_HANDLE              0x02
#define SIMP_BLE_SERVICE0_CHAR1_HANDLE              (SIMP_BLE_SERVICE0_CHAR0_HANDLE + 0x2)
#define SIMP_BLE_SERVICE0_CHAR2_HANDLE              (SIMP_BLE_SERVICE0_CHAR1_HANDLE + 0x2)
#define SIMP_BLE_SERVICE0_CHAR3_HANDLE              (SIMP_BLE_SERVICE0_CHAR2_HANDLE + 0x2)
#define SIMP_BLE_SERVICE_CHAR_NOTIFY_CCCD_HANDLE    (SIMP_BLE_SERVICE0_CHAR3_HANDLE + 0x1)

/**<  Value of simple read characteristic. */
static uint8_t simple_char_read_value[SIMP_READ_V1_MAX_LEN];
static uint16_t simple_char_read_len = 1;
static P_FUN_SERVER_GENERAL_CB pfn_simp_ble_service_cb = NULL;

extern void wifi_btcoex_set_bt_on(void);
extern bool bt_trace_uninit(void);

T_SERVER_ID rtk_service_id;
T_SERVER_ID rtk_srv_id;

T_GAP_DEV_STATE gap_dev_state = {0, 0, 0, 0, 0};                 /**< GAP device state     */
T_GAP_CONN_STATE gap_conn_state = GAP_CONN_STATE_DISCONNECTED;   /**< GAP connection state */
uint8_t  device_name[GAP_DEVICE_NAME_LEN] = "Moddable";

void *app_task_handle = NULL;
void *evt_queue_handle = NULL;
void *io_queue_handle = NULL;

static const uint8_t scan_rsp_data[] =
{
    0x03,                             /* length */
    GAP_ADTYPE_APPEARANCE,            /* type="Appearance" */
    LO_WORD(GAP_GATT_APPEARANCE_UNKNOWN),
    HI_WORD(GAP_GATT_APPEARANCE_UNKNOWN),
};

static const uint8_t adv_data[] =
{
    /* Flags */
    0x02,             /* length */
    GAP_ADTYPE_FLAGS, /* type="Flags" */
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    /* Service */
    0x03,             /* length */
    GAP_ADTYPE_16BIT_COMPLETE,
    LO_WORD(GATT_SERVICE0_UUID),
    HI_WORD(GATT_SERVICE0_UUID),
    /* Local name */
    0x0F,             /* length */
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'B', 'L', 'E', '_', 'P', 'E', 'R', 'I', 'P', 'H', 'E', 'R', 'A', 'L',
};

typedef enum
{
    SIMPLE_BLE_SERVICE_PARAM_V1_READ_CHAR_VAL = 0x01,
} T_SIMP_PARAM_TYPE;

typedef struct
{
    uint8_t opcode;
    T_WRITE_TYPE write_type;
    uint16_t len;
    uint8_t *p_value;
} TSIMP_WRITE_MSG;

typedef union
{
    uint8_t notification_indification_index;
    uint8_t read_value_index;
    TSIMP_WRITE_MSG write;
} TSIMP_UPSTREAM_MSG_DATA;

typedef struct
{
    uint8_t                 conn_id;
    T_SERVICE_CALLBACK_TYPE msg_type;
    TSIMP_UPSTREAM_MSG_DATA msg_data;
} TSIMP_CALLBACK_DATA;

void rtk_bt_stack_config_init(void);
void rtk_app_le_profile_init(void);
void rtk_app_le_gap_init(void);
void rtk_app_task_init(void);
void rtk_app_task_deinit(void);
bool rtk_ble_service_send_v3_notify(uint8_t conn_id, T_SERVER_ID service_id, void *p_value, uint16_t length);

void xs_ble_server_initialize(xsMachine *the)
{
	T_GAP_DEV_STATE new_state;

	if (NULL != gBLE)
		xsUnknownError("BLE already initialized");

	gBLE = (modBLE)c_calloc(sizeof(modBLERecord), 1);
	if (!gBLE)
		xsUnknownError("no memory");

	xsmcSetHostData(xsThis, gBLE);
	gBLE->the = the;
	gBLE->obj = xsThis;
	xsRemember(gBLE->obj);

	xsmcVars(1);
	if (xsmcHas(xsArg(0), xsID_deployServices)) {
		xsmcGet(xsVar(0), xsArg(0), xsID_deployServices);
		gBLE->deployServices = xsmcToBoolean(xsVar(0));
	}
	else
		gBLE->deployServices = true;

	/*Wait WIFI init complete*/
	while(!(wifi_is_up(RTW_STA_INTERFACE) || wifi_is_up(RTW_AP_INTERFACE))) {
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

	//judge BLE central is already on
	le_get_gap_param(GAP_PARAM_DEV_STATE , &new_state);
	if (new_state.gap_init_state == GAP_INIT_STATE_STACK_READY) {
		printf("[BLE Peripheral]BT Stack already on\n\r");
		return;
	}
	else{
		bt_trace_init();
	    rtk_bt_stack_config_init();
	    bte_init();
	    le_gap_init(APP_MAX_LINKS);
	    rtk_app_le_gap_init();
	    rtk_app_le_profile_init();
	    rtk_app_task_init();
	}

	bt_coex_init();

	/*Wait BT init complete*/
	do {
		vTaskDelay(100 / portTICK_RATE_MS);
		le_get_gap_param(GAP_PARAM_DEV_STATE , &new_state);
	}while(new_state.gap_init_state != GAP_INIT_STATE_STACK_READY);

	/*Start BT WIFI coexistence*/
	wifi_btcoex_set_bt_on();

}

void xs_ble_server_close(xsMachine *the)
{
	modBLE ble = gBLE;
	if (!ble) return;

	gBLE = NULL;
	xsForget(ble->obj);
	xs_ble_server_destructor(ble);
}

void xs_ble_server_destructor(void *data)
{
	rtk_app_task_deinit();

	T_GAP_DEV_STATE state;
	le_get_gap_param(GAP_PARAM_DEV_STATE , &state);
	if (state.gap_init_state != GAP_INIT_STATE_STACK_READY) {
		printf("[BLE Peripheral]BT Stack is not running\n\r");
	}
#if F_BT_DEINIT
	else {
		bte_deinit();
		bt_trace_uninit();
		memset(&gap_dev_state, 0, sizeof(T_GAP_DEV_STATE));
		printf("[BLE Peripheral]BT Stack deinitalized\n\r");
	}
#endif
}

void xs_ble_server_disconnect(xsMachine *the)
{
	modBLE ble = gBLE;
	if (!ble) return;

	gBLE = NULL;
	xsForget(ble->obj);
	xs_ble_server_destructor(ble);
}

void xs_ble_server_get_local_address(xsMachine *the)
{
	uint8_t bt_addr[6],buffer[6];
	gap_get_param(GAP_PARAM_BD_ADDR, bt_addr);
	addressToBuffer(bt_addr, buffer);
	xsmcSetArrayBuffer(xsResult, (void*)buffer, 6);
}

void xs_ble_server_set_device_name(xsMachine *the)
{
	memcpy(device_name,xsmcToString(xsArg(0)),c_strlen(xsmcToString(xsArg(0))));
	le_set_gap_param(GAP_PARAM_DEVICE_NAME, GAP_DEVICE_NAME_LEN, device_name);
}

void xs_ble_server_start_advertising(xsMachine *the)
{
	le_adv_start();
}

void xs_ble_server_stop_advertising(xsMachine *the)
{
	le_adv_stop();
}

void xs_ble_server_characteristic_notify_value(xsMachine *the)
{
	uint16_t handle = xsmcToInteger(xsArg(0));
	uint16_t notify = xsmcToInteger(xsArg(1));
	uint8_t *value = xsmcToArrayBuffer(xsArg(2));
	uint32_t length = xsmcGetArrayBufferLength(xsArg(2));

	rtk_ble_service_send_v3_notify(gBLE->conn_id, rtk_srv_id, value, length);
}

void xs_ble_server_set_security_parameters(xsMachine *the)
{
	printf("xs_ble_server_set_security_parameters unsupport\n");
}

void xs_ble_server_passkey_input(xsMachine *the)
{
	printf("xs_ble_server_passkey_input unsupport\n");
}

void xs_ble_server_passkey_reply(xsMachine *the)
{
	printf("xs_ble_server_passkey_reply unsupport\n");
}

void xs_ble_server_get_service_attributes(xsMachine *the)
{
	printf("xs_ble_server_get_service_attributes unsupport\n");
}

void xs_ble_server_deploy(xsMachine *the)
{
	printf("xs_ble_server_deploy unsupport\n");
}

static char_name_table *handleToCharName(uint8_t service_id, uint8_t handle)
{
	for (uint16_t i = 0; i < service_count; ++i)
	{
		if (rtk_srv_id == service_id)
		{
			for (uint16_t k = 0; k < char_name_count; ++k)
			{
				if (char_names[k].service_index == i && char_names[k].att_index == handle)
					return &char_names[k];
			}
		}
	}
	return NULL;
}

static void addressToBuffer(uint8_t *address, uint8_t *buffer)
{
	buffer[0] = address[5];
	buffer[1] = address[4];
	buffer[2] = address[3];
	buffer[3] = address[2];
	buffer[4] = address[1];
	buffer[5] = address[0];
}

static void bufferToAddress(uint8_t *buffer, uint8_t *address)
{
	address[0] = buffer[5];
	address[1] = buffer[4];
	address[2] = buffer[3];
	address[3] = buffer[2];
	address[4] = buffer[1];
	address[5] = buffer[0];
}

bool rtk_ble_service_set_parameter(T_SIMP_PARAM_TYPE param_type, uint16_t len, void *p_value)
{
    bool ret = true;

    switch (param_type)
    {
    default:
        ret = false;
        break;

    case SIMPLE_BLE_SERVICE_PARAM_V1_READ_CHAR_VAL:
        if (len <= SIMP_READ_V1_MAX_LEN)
        {
            memcpy(simple_char_read_value, p_value, len);
            simple_char_read_len = len;
        }
        else
        {
            ret = false;
        }
        break;
    }
    if (!ret)
    {
        APP_PRINT_ERROR0("rtk_ble_service_set_parameter failed");
    }

    return ret;
}

T_APP_RESULT  rtk_ble_service_attr_read_cb(uint8_t conn_id, T_SERVER_ID service_id,
                                            uint16_t attrib_index, uint16_t offset, uint16_t *p_length, uint8_t **pp_value)
{
    (void)offset;
    T_APP_RESULT  cause  = APP_RESULT_SUCCESS;
    uint8_t uuid_buffer[2];
    char_name_table *char_names;

    switch (attrib_index)
    {
    default:
        APP_PRINT_ERROR1("rtk_ble_service_attr_read_cb, Attr not found, index %d", attrib_index);
        cause = APP_RESULT_ATTR_NOT_FOUND;
        break;
    case SIMP_BLE_SERVICE0_CHAR3_HANDLE:
        {
            TSIMP_CALLBACK_DATA callback_data;
            callback_data.msg_type = SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE;
            callback_data.msg_data.read_value_index = SIMP_READ_V1;
            callback_data.conn_id = conn_id;
            if (pfn_simp_ble_service_cb)
            {
                pfn_simp_ble_service_cb(service_id, (void *)&callback_data);
            }

            char_names = handleToCharName(service_id,attrib_index);
            uuid_buffer[0] = LO_WORD(GATT_SERVICE0_CHR3_UUID);
            uuid_buffer[1] = HI_WORD(GATT_SERVICE0_CHR3_UUID);

            xsBeginHost(gBLE->the);
            xsmcVars(5);

            xsVar(0) = xsmcNewObject();
            xsmcSetArrayBuffer(xsVar(1), uuid_buffer, 2);
            xsmcSetInteger(xsVar(2), attrib_index);
            xsmcSet(xsVar(0), xsID_uuid, xsVar(1));
            xsmcSet(xsVar(0), xsID_handle, xsVar(2));

            xsmcSetString(xsVar(3), (char*)char_names->name);
            xsmcSet(xsVar(0), xsID_name, xsVar(3));
            xsmcSetString(xsVar(4), (char*)char_names->type);
            xsmcSet(xsVar(0), xsID_type, xsVar(4));
            xsResult = xsCall2(gBLE->obj, xsID_callback, xsString("onCharacteristicRead"), xsVar(0));
            if (xsUndefinedType != xsmcTypeOf(xsResult))
            {
                *pp_value = xsmcToArrayBuffer(xsResult);
                *p_length = xsmcGetArrayBufferLength(xsResult);
            }
            xsEndHost(gBLE->the);
        }
        break;
    }

    return (cause);
}

void rtk_write_post_callback(uint8_t conn_id, T_SERVER_ID service_id, uint16_t attrib_index,
                                uint16_t length, uint8_t *p_value)
{
    (void)p_value;
    printf("rtk_write_post_callback: conn_id %d, service_id %d, attrib_index 0x%x, length %d", conn_id, service_id, attrib_index, length);
}

T_APP_RESULT rtk_ble_service_attr_write_cb(uint8_t conn_id, T_SERVER_ID service_id,
                                            uint16_t attrib_index, T_WRITE_TYPE write_type, uint16_t length, uint8_t *p_value,
                                            P_FUN_WRITE_IND_POST_PROC *p_write_ind_post_proc)
{
    TSIMP_CALLBACK_DATA callback_data;
    T_APP_RESULT  cause = APP_RESULT_SUCCESS;
    uint8_t uuid_buffer[2];
    char_name_table *char_names;

    if (!gBLE) return;
    xsBeginHost(gBLE->the);

    //printf("rtk_ble_service_attr_write_cb write_type = 0x%x\n", write_type);
    //*p_write_ind_post_proc = rtk_write_post_callback;
    if (SIMP_BLE_SERVICE0_CHAR0_HANDLE == attrib_index ||
        SIMP_BLE_SERVICE0_CHAR1_HANDLE == attrib_index ||
        SIMP_BLE_SERVICE0_CHAR2_HANDLE == attrib_index)
    {
        /* Make sure written value size is valid. */
        if (p_value == NULL)
        {
            cause  = APP_RESULT_INVALID_VALUE_SIZE;
        }
        else
        {
            /* Notify Application. */
            callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
            callback_data.conn_id  = conn_id;
            callback_data.msg_data.write.opcode = SIMP_WRITE_V2;
            callback_data.msg_data.write.write_type = write_type;
            callback_data.msg_data.write.len = length;
            callback_data.msg_data.write.p_value = p_value;

            if (pfn_simp_ble_service_cb)
            {
                pfn_simp_ble_service_cb(service_id, (void *)&callback_data);
            }

            if(SIMP_BLE_SERVICE0_CHAR0_HANDLE == attrib_index)
            {
                uuid_buffer[0] = LO_WORD(GATT_SERVICE0_CHR0_UUID);
                uuid_buffer[1] = HI_WORD(GATT_SERVICE0_CHR0_UUID);
            }
            else if(SIMP_BLE_SERVICE0_CHAR1_HANDLE == attrib_index)
            {
                uuid_buffer[0] = LO_WORD(GATT_SERVICE0_CHR1_UUID);
                uuid_buffer[1] = HI_WORD(GATT_SERVICE0_CHR1_UUID);

            }
            else if(SIMP_BLE_SERVICE0_CHAR2_HANDLE == attrib_index)
            {
                uuid_buffer[0] = LO_WORD(GATT_SERVICE0_CHR2_UUID);
                uuid_buffer[1] = HI_WORD(GATT_SERVICE0_CHR2_UUID);
            }

            char_names = handleToCharName(service_id,attrib_index);

            xsmcVars(6);
            xsVar(0) = xsmcNewObject();
            xsmcSetArrayBuffer(xsVar(1), uuid_buffer, 2);
            xsmcSet(xsVar(0), xsID_uuid, xsVar(1));

            if (char_names) {
                xsmcSetString(xsVar(2), (char*)char_names->name);
                xsmcSet(xsVar(0), xsID_name, xsVar(2));
                xsmcSetString(xsVar(3), (char*)char_names->type);
                xsmcSet(xsVar(0), xsID_type, xsVar(3));
            }

            xsmcSetInteger(xsVar(2), attrib_index);
            xsmcSet(xsVar(0), xsID_handle, xsVar(2));
            xsmcSetArrayBuffer(xsVar(3), p_value, length);
            xsmcSet(xsVar(0), xsID_value, xsVar(3));
            xsCall2(gBLE->obj, xsID_callback, xsString("onCharacteristicWritten"), xsVar(0));

        }
    }
    else
    {
        printf("rtk_ble_service_attr_write_cb Error: attrib_index 0x%x, length %d\n", attrib_index, length);
        cause = APP_RESULT_ATTR_NOT_FOUND;
    }

    xsEndHost(gBLE->the);
    return cause;
}

bool rtk_ble_service_send_v3_notify(uint8_t conn_id, T_SERVER_ID service_id, void *p_value,
                                     uint16_t length)
{
    APP_PRINT_INFO0("rtk_ble_service_send_v3_notify");
    // send notification to client
    return server_send_data(conn_id, service_id, SIMP_BLE_SERVICE0_CHAR3_HANDLE, p_value,
                            length,
                            GATT_PDU_TYPE_ANY);
}

void rtk_ble_service_cccd_update_cb(uint8_t conn_id, T_SERVER_ID service_id, uint16_t index,
                                     uint16_t cccbits)
{
    TSIMP_CALLBACK_DATA callback_data;
    bool is_handled = false;
    callback_data.conn_id = conn_id;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
	uint8_t uuid_buffer[2];
	uint8_t notify = 0xFF;
	char_name_table *char_names;

	if (!gBLE) return;
	xsBeginHost(gBLE->the);

    printf("simp_ble_service_cccd_update_cb: index = %d, cccbits 0x%x", index, cccbits);
    switch (index)
    {
    case SIMP_BLE_SERVICE_CHAR_NOTIFY_CCCD_HANDLE:
        {
            if (cccbits & GATT_CLIENT_CHAR_CONFIG_NOTIFY)
            {
                // Enable Notification
                callback_data.msg_data.notification_indification_index = SIMP_NOTIFY_INDICATE_V3_ENABLE;
                notify = SIMP_NOTIFY_INDICATE_V3_ENABLE;
            }
            else
            {
                // Disable Notification
                callback_data.msg_data.notification_indification_index = SIMP_NOTIFY_INDICATE_V3_DISABLE;
                notify = SIMP_NOTIFY_INDICATE_V3_DISABLE;
            }
            is_handled =  true;
        }
        break;
    default:
        break;
    }
    /* Notify Application. */
    if (pfn_simp_ble_service_cb && (is_handled == true))
    {
        pfn_simp_ble_service_cb(service_id, (void *)&callback_data);
    }

    char_names = handleToCharName(service_id,index-1);
    uuid_buffer[0] = 0x04;
    uuid_buffer[1] = 0xFF;

    xsmcVars(6);
    xsVar(0) = xsmcNewObject();
    xsmcSetArrayBuffer(xsVar(1), uuid_buffer, 2);
    xsmcSet(xsVar(0), xsID_uuid, xsVar(1));

    if (char_names) {
        xsmcSetString(xsVar(2), (char*)char_names->name);
        xsmcSet(xsVar(0), xsID_name, xsVar(2));
        xsmcSetString(xsVar(3), (char*)char_names->type);
        xsmcSet(xsVar(0), xsID_type, xsVar(3));
    }

    xsmcSetInteger(xsVar(4), index-1);
    xsmcSetInteger(xsVar(5), notify);
    xsmcSet(xsVar(0), xsID_handle, xsVar(4));
    xsmcSet(xsVar(0), xsID_notify, xsVar(5));
    xsCall2(gBLE->obj, xsID_callback, xsString(0 == notify ? "onCharacteristicNotifyDisabled" : "onCharacteristicNotifyEnabled"), xsVar(0));

    xsEndHost(gBLE->the);
}

const T_FUN_GATT_SERVICE_CBS rtk_ble_service_cbs =
{
    rtk_ble_service_attr_read_cb,  // Read callback function pointer
    rtk_ble_service_attr_write_cb, // Write callback function pointer
    rtk_ble_service_cccd_update_cb // CCCD update callback function pointer
};

T_SERVER_ID rtk_ble_service_add_service(void *p_func)
{
    if (false == server_add_service(&rtk_service_id,
                                    (uint8_t *)modd_ble_service_tbl,
                                    sizeof(modd_ble_service_tbl),
                                    rtk_ble_service_cbs))
    {
        APP_PRINT_ERROR0("rtk_ble_service_add_service: fail");
        rtk_service_id = 0xff;
        return rtk_service_id;
    }

    pfn_simp_ble_service_cb = (P_FUN_SERVER_GENERAL_CB)p_func;
    return rtk_service_id;
}

void rtk_app_handle_io_msg(T_IO_MSG io_msg)
{
    uint16_t msg_type = io_msg.type;

    switch (msg_type)
    {
    case IO_MSG_TYPE_BT_STATUS:
        {
            rtk_app_handle_gap_msg(&io_msg);
        }
        break;
    case IO_MSG_TYPE_AT_CMD:
        {
            uint16_t subtype = io_msg.subtype;
            void *arg = io_msg.u.buf;
            printf("WARNING: skip ble_peripheral_app_handle_at_cmd\n");
            //ble_peripheral_app_handle_at_cmd(subtype, arg);
        }
        break;
    case IO_MSG_TYPE_QDECODE:
        {
            if (io_msg.subtype == 0) {
                le_adv_stop();
            } else if (io_msg.subtype == 1) {
                le_adv_start();
            }
        }
        break;
    default:
        break;
    }
}

void rtk_app_handle_dev_state_evt(T_GAP_DEV_STATE new_state, uint16_t cause)
{
    APP_PRINT_INFO3("rtk_app_handle_dev_state_evt: init state %d, adv state %d, cause 0x%x",
                    new_state.gap_init_state, new_state.gap_adv_state, cause);
    if (gap_dev_state.gap_init_state != new_state.gap_init_state)
    {
        if (new_state.gap_init_state == GAP_INIT_STATE_STACK_READY)
        {
            APP_PRINT_INFO0("GAP stack ready");
			printf("\n\r[BLE peripheral] GAP stack ready\n\r");
			if (!gBLE) return;	// Stack is ready
			xsBeginHost(gBLE->the);
			xsCall1(gBLE->obj, xsID_callback,xsString("onReady"));
			xsEndHost(gBLE->the);
            /*stack ready*/
            //le_adv_start();
        }
    }

    if (gap_dev_state.gap_adv_state != new_state.gap_adv_state)
    {
        if (new_state.gap_adv_state == GAP_ADV_STATE_IDLE)
        {
            if (new_state.gap_adv_sub_state == GAP_ADV_TO_IDLE_CAUSE_CONN)
            {
				printf("\n\rGAP adv stoped: because connection created\n\r");
            }
            else
            {
				printf("\n\rGAP adv stopped\n\r");
            }
        }
        else if (new_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING)
        {
			printf("\n\rGAP adv start\n\r");
        }
    }

    gap_dev_state = new_state;
}

void rtk_app_handle_conn_state_evt(uint8_t conn_id, T_GAP_CONN_STATE new_state, uint16_t disc_cause)
{
    APP_PRINT_INFO4("rtk_app_handle_conn_state_evt: conn_id %d old_state %d new_state %d, disc_cause 0x%x",
                    conn_id, gap_conn_state, new_state, disc_cause);
    switch (new_state)
    {
    case GAP_CONN_STATE_DISCONNECTED:
        {
            if ((disc_cause != (HCI_ERR | HCI_ERR_REMOTE_USER_TERMINATE))
                && (disc_cause != (HCI_ERR | HCI_ERR_LOCAL_HOST_TERMINATE)))
            {
                APP_PRINT_ERROR1("rtk_app_handle_conn_state_evt: connection lost cause 0x%x", disc_cause);
            }
			printf("\n\r[BLE peripheral] BT Disconnected, start ADV\n\r");
            le_adv_start();
        }
        break;

    case GAP_CONN_STATE_CONNECTED:
        {
            uint16_t conn_interval;
            uint16_t conn_latency;
            uint16_t conn_supervision_timeout;
            uint8_t  remote_bd[6];
            T_GAP_REMOTE_ADDR_TYPE remote_bd_type;

            le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_latency, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
            le_get_conn_addr(conn_id, remote_bd, (void *)&remote_bd_type);
            APP_PRINT_INFO5("GAP_CONN_STATE_CONNECTED:remote_bd %s, remote_addr_type %d, conn_interval 0x%x, conn_latency 0x%x, conn_supervision_timeout 0x%x",
                            TRACE_BDADDR(remote_bd), remote_bd_type,
                            conn_interval, conn_latency, conn_supervision_timeout);
			printf("\n\r[BLE peripheral] BT Connected\n\r");
        }
        break;

    default:
        break;
    }
    gap_conn_state = new_state;
}

void rtk_app_handle_authen_state_evt(uint8_t conn_id, uint8_t new_state, uint16_t cause)
{
    APP_PRINT_INFO2("rtk_app_handle_authen_state_evt:conn_id %d, cause 0x%x", conn_id, cause);

    switch (new_state)
    {
    case GAP_AUTHEN_STATE_STARTED:
        {
            APP_PRINT_INFO0("rtk_app_handle_authen_state_evt: GAP_AUTHEN_STATE_STARTED");
        }
        break;

    case GAP_AUTHEN_STATE_COMPLETE:
        {
            if (cause == GAP_SUCCESS)
            {
                printf("Pair success\r\n");
                APP_PRINT_INFO0("rtk_app_handle_authen_state_evt: GAP_AUTHEN_STATE_COMPLETE pair success");

            }
            else
            {
                printf("Pair failed: cause 0x%x\r\n", cause);
                APP_PRINT_INFO0("rtk_app_handle_authen_state_evt: GAP_AUTHEN_STATE_COMPLETE pair failed");
            }
        }
        break;

    default:
        {
            APP_PRINT_ERROR1("rtk_app_handle_authen_state_evt: unknown newstate %d", new_state);
        }
        break;
    }
}

void rtk_app_handle_conn_mtu_info_evt(uint8_t conn_id, uint16_t mtu_size)
{
    APP_PRINT_INFO2("rtk_app_handle_conn_mtu_info_evt: conn_id %d, mtu_size %d", conn_id, mtu_size);
}

void rtk_app_handle_conn_param_update_evt(uint8_t conn_id, uint8_t status, uint16_t cause)
{
    switch (status)
    {
    case GAP_CONN_PARAM_UPDATE_STATUS_SUCCESS:
        {
            uint16_t conn_interval;
            uint16_t conn_slave_latency;
            uint16_t conn_supervision_timeout;

            le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_slave_latency, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
            printf("rtk_app_handle_conn_param_update_evt update success:conn_interval 0x%x, conn_slave_latency 0x%x, conn_supervision_timeout 0x%x\r\n",
                            conn_interval, conn_slave_latency, conn_supervision_timeout);
        }
        break;

    case GAP_CONN_PARAM_UPDATE_STATUS_FAIL:
        {
		    printf("rtk_app_handle_conn_param_update_evt update failed: cause 0x%x", cause);
        }
        break;

    case GAP_CONN_PARAM_UPDATE_STATUS_PENDING:
        {
		    printf("\n\rble_central_app_handle_conn_param_update_evt update pending: conn_id %d\r\n", conn_id);
        }
        break;

    default:
        break;
    }
}

void rtk_app_handle_gap_msg(T_IO_MSG *p_gap_msg)
{
    T_LE_GAP_MSG gap_msg;
    uint8_t conn_id;
    memcpy(&gap_msg, &p_gap_msg->u.param, sizeof(p_gap_msg->u.param));

    APP_PRINT_TRACE1("rtk_app_handle_gap_msg: subtype %d", p_gap_msg->subtype);
    switch (p_gap_msg->subtype)
    {
    case GAP_MSG_LE_DEV_STATE_CHANGE:
        {
            rtk_app_handle_dev_state_evt(gap_msg.msg_data.gap_dev_state_change.new_state,
                                     gap_msg.msg_data.gap_dev_state_change.cause);
        }
        break;

    case GAP_MSG_LE_CONN_STATE_CHANGE:
        {
            rtk_app_handle_conn_state_evt(gap_msg.msg_data.gap_conn_state_change.conn_id,
                                      (T_GAP_CONN_STATE)gap_msg.msg_data.gap_conn_state_change.new_state,
                                      gap_msg.msg_data.gap_conn_state_change.disc_cause);
        }
        break;

    case GAP_MSG_LE_CONN_MTU_INFO:
        {
            rtk_app_handle_conn_mtu_info_evt(gap_msg.msg_data.gap_conn_mtu_info.conn_id,
                                         gap_msg.msg_data.gap_conn_mtu_info.mtu_size);
        }
        break;

    case GAP_MSG_LE_CONN_PARAM_UPDATE:
        {
            rtk_app_handle_conn_param_update_evt(gap_msg.msg_data.gap_conn_param_update.conn_id,
                                             gap_msg.msg_data.gap_conn_param_update.status,
                                             gap_msg.msg_data.gap_conn_param_update.cause);
        }
        break;

    case GAP_MSG_LE_AUTHEN_STATE_CHANGE:
        {
            rtk_app_handle_authen_state_evt(gap_msg.msg_data.gap_authen_state.conn_id,
                                        gap_msg.msg_data.gap_authen_state.new_state,
                                        gap_msg.msg_data.gap_authen_state.status);
        }
        break;

    case GAP_MSG_LE_BOND_JUST_WORK:
        {
            conn_id = gap_msg.msg_data.gap_bond_just_work_conf.conn_id;
            le_bond_just_work_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
            APP_PRINT_INFO0("GAP_MSG_LE_BOND_JUST_WORK");
        }
        break;

    case GAP_MSG_LE_BOND_PASSKEY_DISPLAY:
        {
            uint32_t display_value = 0;
            conn_id = gap_msg.msg_data.gap_bond_passkey_display.conn_id;
            le_bond_get_display_key(conn_id, &display_value);
            le_bond_passkey_display_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
		    printf("GAP_MSG_LE_BOND_PASSKEY_DISPLAY:passkey %d", display_value);
        }
        break;

    case GAP_MSG_LE_BOND_USER_CONFIRMATION:
        {
            uint32_t display_value = 0;
            conn_id = gap_msg.msg_data.gap_bond_user_conf.conn_id;
            le_bond_get_display_key(conn_id, &display_value);
            printf("GAP_MSG_LE_BOND_USER_CONFIRMATION: passkey %d", display_value);
            //le_bond_user_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
        }
        break;

    case GAP_MSG_LE_BOND_PASSKEY_INPUT:
        {
            //uint32_t passkey = 888888;
            conn_id = gap_msg.msg_data.gap_bond_passkey_input.conn_id;
            //le_bond_passkey_input_confirm(conn_id, passkey, GAP_CFM_CAUSE_ACCEPT);
		    printf("GAP_MSG_LE_BOND_PASSKEY_INPUT: conn_id %d", conn_id);
        }
        break;
#if F_BT_LE_SMP_OOB_SUPPORT
    case GAP_MSG_LE_BOND_OOB_INPUT:
        {
            uint8_t oob_data[GAP_OOB_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            conn_id = gap_msg.msg_data.gap_bond_oob_input.conn_id;
            APP_PRINT_INFO0("GAP_MSG_LE_BOND_OOB_INPUT");
            le_bond_set_param(GAP_PARAM_BOND_OOB_DATA, GAP_OOB_LEN, oob_data);
            le_bond_oob_input_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
        }
        break;
#endif
    default:
        APP_PRINT_ERROR1("rtk_app_handle_gap_msg: unknown subtype %d", p_gap_msg->subtype);
        break;
    }
}

T_APP_RESULT rtk_app_gap_callback(uint8_t cb_type, void *p_cb_data)
{
    T_APP_RESULT result = APP_RESULT_SUCCESS;
    T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;

    switch (cb_type)
    {
#if F_BT_LE_4_2_DATA_LEN_EXT_SUPPORT
    case GAP_MSG_LE_DATA_LEN_CHANGE_INFO:
        APP_PRINT_INFO3("GAP_MSG_LE_DATA_LEN_CHANGE_INFO: conn_id %d, tx octets 0x%x, max_tx_time 0x%x",
                        p_data->p_le_data_len_change_info->conn_id,
                        p_data->p_le_data_len_change_info->max_tx_octets,
                        p_data->p_le_data_len_change_info->max_tx_time);
        break;
#endif
    case GAP_MSG_LE_MODIFY_WHITE_LIST:
        APP_PRINT_INFO2("GAP_MSG_LE_MODIFY_WHITE_LIST: operation %d, cause 0x%x",
                        p_data->p_le_modify_white_list_rsp->operation,
                        p_data->p_le_modify_white_list_rsp->cause);
        break;

    default:
        APP_PRINT_ERROR1("app_gap_callback: unhandled cb_type 0x%x", cb_type);
        break;
    }
    return result;
}

T_APP_RESULT rtk_app_profile_callback(T_SERVER_ID service_id, void *p_data)
{
    T_APP_RESULT app_result = APP_RESULT_SUCCESS;
    if (service_id == SERVICE_PROFILE_GENERAL_ID)
    {
        T_SERVER_APP_CB_DATA *p_param = (T_SERVER_APP_CB_DATA *)p_data;
        switch (p_param->eventId)
        {
        case PROFILE_EVT_SRV_REG_COMPLETE:// srv register result event.
            APP_PRINT_INFO1("PROFILE_EVT_SRV_REG_COMPLETE: result %d",
                            p_param->event_data.service_reg_result);
            break;

        case PROFILE_EVT_SEND_DATA_COMPLETE:
            APP_PRINT_INFO5("PROFILE_EVT_SEND_DATA_COMPLETE: conn_id %d, cause 0x%x, service_id %d, attrib_idx 0x%x, credits %d",
                            p_param->event_data.send_data_result.conn_id,
                            p_param->event_data.send_data_result.cause,
                            p_param->event_data.send_data_result.service_id,
                            p_param->event_data.send_data_result.attrib_idx,
                            p_param->event_data.send_data_result.credits);
            if (p_param->event_data.send_data_result.cause == GAP_SUCCESS)
            {
                APP_PRINT_INFO0("PROFILE_EVT_SEND_DATA_COMPLETE success");
            }
            else
            {
                APP_PRINT_ERROR0("PROFILE_EVT_SEND_DATA_COMPLETE failed");
            }
            break;

        default:
            break;
        }
    }
    else  if (service_id == rtk_srv_id)
    {
        TSIMP_CALLBACK_DATA *p_simp_cb_data = (TSIMP_CALLBACK_DATA *)p_data;
        switch (p_simp_cb_data->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
            {
                switch (p_simp_cb_data->msg_data.notification_indification_index)
                {
                case SIMP_NOTIFY_INDICATE_V3_ENABLE:
                    {
                        APP_PRINT_INFO0("SIMP_NOTIFY_INDICATE_V3_ENABLE");
                    }
                    break;

                case SIMP_NOTIFY_INDICATE_V3_DISABLE:
                    {
                        APP_PRINT_INFO0("SIMP_NOTIFY_INDICATE_V3_DISABLE");
                    }
                    break;
                case SIMP_NOTIFY_INDICATE_V4_ENABLE:
                    {
                        APP_PRINT_INFO0("SIMP_NOTIFY_INDICATE_V4_ENABLE");
                    }
                    break;
                case SIMP_NOTIFY_INDICATE_V4_DISABLE:
                    {
                        APP_PRINT_INFO0("SIMP_NOTIFY_INDICATE_V4_DISABLE");
                    }
                    break;
                default:
                    break;
                }
            }
            break;

        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
            {
                if (p_simp_cb_data->msg_data.read_value_index == SIMP_READ_V1)
                {
                    uint8_t value[2] = {0x01, 0x02};
                    APP_PRINT_INFO0("SIMP_READ_V1");
                    rtk_ble_service_set_parameter(SIMPLE_BLE_SERVICE_PARAM_V1_READ_CHAR_VAL, 2, &value);
                }
            }
            break;
        case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
            {
                switch (p_simp_cb_data->msg_data.write.opcode)
                {
                case SIMP_WRITE_V2:
                    {
                        APP_PRINT_INFO2("SIMP_WRITE_V2: write type %d, len %d", p_simp_cb_data->msg_data.write.write_type,
                                        p_simp_cb_data->msg_data.write.len);
                    }
                    break;
                default:
                    break;
                }
            }
            break;

        default:
            break;
        }
    }

    return app_result;
}

void rtk_bt_stack_config_init(void)
{
    gap_config_max_le_link_num(APP_MAX_LINKS);
    gap_config_max_le_paired_device(APP_MAX_LINKS);
}

void rtk_app_le_profile_init(void)
{
    server_init(1);
    rtk_srv_id = rtk_ble_service_add_service((void *)rtk_app_profile_callback);
    server_register_app_cb(rtk_app_profile_callback);
}

void rtk_app_le_gap_init(void)
{
    /* Device name and device appearance */
    //uint8_t  device_name[GAP_DEVICE_NAME_LEN] = "Moddable Device";
    //uint8_t  device_name[GAP_DEVICE_NAME_LEN] = "Hank_PERIPHERAL";
    uint16_t appearance = GAP_GATT_APPEARANCE_UNKNOWN;
    uint8_t  slave_init_mtu_req = false;

    /* Advertising parameters */
    uint8_t  adv_evt_type = GAP_ADTYPE_ADV_IND;
    uint8_t  adv_direct_type = GAP_REMOTE_ADDR_LE_PUBLIC;
    uint8_t  adv_direct_addr[GAP_BD_ADDR_LEN] = {0};
    uint8_t  adv_chann_map = GAP_ADVCHAN_ALL;
    uint8_t  adv_filter_policy = GAP_ADV_FILTER_ANY;
    uint16_t adv_int_min = DEFAULT_ADVERTISING_INTERVAL_MIN;
    uint16_t adv_int_max = DEFAULT_ADVERTISING_INTERVAL_MAX;

    /* GAP Bond Manager parameters */
    uint8_t  auth_pair_mode = GAP_PAIRING_MODE_PAIRABLE;
    uint16_t auth_flags = GAP_AUTHEN_BIT_BONDING_FLAG;
    uint8_t  auth_io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
#if F_BT_LE_SMP_OOB_SUPPORT
    uint8_t  auth_oob = false;
#endif
    uint8_t  auth_use_fix_passkey = false;
    uint32_t auth_fix_passkey = 0;
    uint8_t  auth_sec_req_enable = false;
    uint16_t auth_sec_req_flags = GAP_AUTHEN_BIT_BONDING_FLAG;

    /* Set device name and device appearance */
    le_set_gap_param(GAP_PARAM_DEVICE_NAME, GAP_DEVICE_NAME_LEN, device_name);
    le_set_gap_param(GAP_PARAM_APPEARANCE, sizeof(appearance), &appearance);
    le_set_gap_param(GAP_PARAM_SLAVE_INIT_GATT_MTU_REQ, sizeof(slave_init_mtu_req),
                     &slave_init_mtu_req);

    /* Set advertising parameters */
    le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE, sizeof(adv_evt_type), &adv_evt_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR_TYPE, sizeof(adv_direct_type), &adv_direct_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR, sizeof(adv_direct_addr), adv_direct_addr);
    le_adv_set_param(GAP_PARAM_ADV_CHANNEL_MAP, sizeof(adv_chann_map), &adv_chann_map);
    le_adv_set_param(GAP_PARAM_ADV_FILTER_POLICY, sizeof(adv_filter_policy), &adv_filter_policy);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN, sizeof(adv_int_min), &adv_int_min);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX, sizeof(adv_int_max), &adv_int_max);
    le_adv_set_param(GAP_PARAM_ADV_DATA, sizeof(adv_data), (void *)adv_data);
    le_adv_set_param(GAP_PARAM_SCAN_RSP_DATA, sizeof(scan_rsp_data), (void *)scan_rsp_data);

    /* Setup the GAP Bond Manager */
    gap_set_param(GAP_PARAM_BOND_PAIRING_MODE, sizeof(auth_pair_mode), &auth_pair_mode);
    gap_set_param(GAP_PARAM_BOND_AUTHEN_REQUIREMENTS_FLAGS, sizeof(auth_flags), &auth_flags);
    gap_set_param(GAP_PARAM_BOND_IO_CAPABILITIES, sizeof(auth_io_cap), &auth_io_cap);
#if F_BT_LE_SMP_OOB_SUPPORT
    gap_set_param(GAP_PARAM_BOND_OOB_ENABLED, sizeof(auth_oob), &auth_oob);
#endif
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY, sizeof(auth_fix_passkey), &auth_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY_ENABLE, sizeof(auth_use_fix_passkey),
                      &auth_use_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_ENABLE, sizeof(auth_sec_req_enable), &auth_sec_req_enable);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_REQUIREMENT, sizeof(auth_sec_req_flags),
                      &auth_sec_req_flags);

    /* register gap message callback */
    le_register_app_cb(rtk_app_gap_callback);
}

void rtk_app_main_task(void *p_param)
{
    (void)p_param;
    uint8_t event;
    os_msg_queue_create(&io_queue_handle, MAX_NUMBER_OF_IO_MESSAGE, sizeof(T_IO_MSG));
    os_msg_queue_create(&evt_queue_handle, MAX_NUMBER_OF_EVENT_MESSAGE, sizeof(uint8_t));

    gap_start_bt_stack(evt_queue_handle, io_queue_handle, MAX_NUMBER_OF_GAP_MESSAGE);

    while (true)
    {
        if (os_msg_recv(evt_queue_handle, &event, 0xFFFFFFFF) == true)
        {
            if (event == EVENT_IO_TO_APP)
            {
                T_IO_MSG io_msg;
                if (os_msg_recv(io_queue_handle, &io_msg, 0) == true)
                {
                    rtk_app_handle_io_msg(io_msg);
                }
            }
            else
            {
                gap_handle_msg(event);
            }
        }
    }
}

void rtk_app_task_init(void)
{
    os_task_create(&app_task_handle, "app", rtk_app_main_task, 0, APP_TASK_STACK_SIZE,
                   APP_TASK_PRIORITY);
}

void rtk_app_task_deinit(void)
{
	if (io_queue_handle) {
		os_msg_queue_delete(io_queue_handle);
	}
	if (evt_queue_handle) {
		os_msg_queue_delete(evt_queue_handle);
	}
	if (app_task_handle) {
		os_task_delete(app_task_handle);
	}
	io_queue_handle = NULL;
	evt_queue_handle = NULL;
	app_task_handle = NULL;

	gap_dev_state.gap_init_state = 0;
	gap_dev_state.gap_adv_sub_state = 0;
	gap_dev_state.gap_adv_state = 0;
	gap_dev_state.gap_scan_state = 0;
	gap_dev_state.gap_conn_state = 0;
}


