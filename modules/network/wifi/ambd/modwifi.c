/*
 * Copyright (c) 2016-2020  Moddable Tech, Inc.
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

#include "xs.h"
#include "xsmc.h"
#include "xsHost.h"

#include "mc.xs.h"          // for xsID_ values

#include "wifi_conf.h"
#include "wlan_intf.h"
#include "wifi_constants.h"
#include "lwip_netconf.h"
#include <wifi/wifi_conf.h>
#include <wifi/wifi_util.h>


/*NETMASK*/
#ifndef NETMASK_ADDR0
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0
#endif

/*Gateway Address*/
#ifndef GW_ADDR0
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   1
#define GW_ADDR3   1
#endif

extern struct netif xnetif[NET_IF_NUM];

static int gMode = 0;
static void initWiFi(void);
static rtw_result_t app_scan_result_handler( rtw_scan_handler_result_t* malloced_scan_result );
static rtw_network_info_t wifi = {0};
static rtw_ap_info_t ap = {0};
static unsigned char password[65] = {0};
static int mode = 0;
static int gWiFiStatus = -1;

struct wifiScanRecord {
    xsSlot          callback;
    xsMachine       *the;
};
typedef struct wifiScanRecord wifiScanRecord;

static wifiScanRecord *gScan;

typedef struct xsWiFiRecord xsWiFiRecord;
typedef xsWiFiRecord *xsWiFi;

struct xsWiFiRecord {
    xsWiFi              next;
    xsMachine           *the;
    xsSlot              obj;
    uint8_t             haveCallback;
};

static xsWiFi gWiFi;


void xs_wifi_set_mode(xsMachine *the)
{
    int mode = xsmcToInteger(xsArg(0));

    initWiFi();
    if (RTW_MODE_STA == mode)
    {
        wifi_set_mode(RTW_MODE_STA);
        gMode = RTW_MODE_STA;
    }
    else if (RTW_MODE_AP == mode)
    {
        wifi_set_mode(RTW_MODE_AP);
        gMode = RTW_MODE_AP;
    }
    else if (RTW_MODE_STA_AP == mode)
    {
        wifi_set_mode(RTW_MODE_STA_AP);
        gMode = RTW_MODE_STA_AP;
    }
    else
    {
        xsUnknownError("invalid mode");
    }
}

void xs_wifi_get_mode(xsMachine *the)
{
    initWiFi();
	int mode = 0;
	u8 *ifname[1] = {(u8*)WLAN0_NAME};

	if(wext_get_mode(ifname, &mode) < 0)
		return;

	switch(mode) {
		case IW_MODE_MASTER:
			xsmcSetInteger(xsResult, 2); // RTW_MODE_AP;
			break;
		case IW_MODE_INFRA:
		default:
			xsmcSetInteger(xsResult, 1); // RTW_MODE_STA;
			break;
	}
}

void xs_wifi_scan(xsMachine *the)
{
    int ret = RTW_SUCCESS;

    initWiFi();

    if (gScan)
        xsUnknownError("already scanning");

    gScan = (wifiScanRecord *)c_calloc(1, sizeof(wifiScanRecord));
    if (NULL == gScan)
        xsUnknownError("out of memory");
    gScan->callback = xsArg(1);
    gScan->the = the;
    xsRemember(gScan->callback);

    if (xsmcArgc) {
        xsmcVars(1);

        if (xsmcHas(xsArg(0), xsID_hidden)) {
            xsmcGet(xsVar(0), xsArg(0), xsID_hidden);
            //config.show_hidden = xsmcTest(xsVar(0));
        }

        if (xsmcHas(xsArg(0), xsID_channel)) {
            xsmcGet(xsVar(0), xsArg(0), xsID_channel);
            //config.channel = xsmcToInteger(xsVar(0));
        }
    }

    if((ret = wifi_scan_networks(app_scan_result_handler, NULL)) != RTW_SUCCESS)
    {
        xsForget(gScan->callback);
        c_free(gScan);
        gScan = NULL;
        xsUnknownError("scan request failed");
    }
}

static rtw_result_t app_scan_result_handler( rtw_scan_handler_result_t* malloced_scan_result )
{
    static int ApNum = 0;
    xsBeginHost(gScan->the);
    if (malloced_scan_result->scan_complete != RTW_TRUE)
    {
        rtw_scan_result_t* record = &malloced_scan_result->ap_details;
        record->SSID.val[record->SSID.len] = 0; /* Ensure the SSID is null terminated */

        xsmcVars(2);
        xsTry {
            xsmcSetNewObject(xsVar(1));

            xsmcSetString(xsVar(0), record->SSID.val);
            xsmcSet(xsVar(1), xsID_ssid, xsVar(0));

            xsmcSetInteger(xsVar(0), record->signal_strength);
            xsmcSet(xsVar(1), xsID_rssi, xsVar(0));

            xsmcSetInteger(xsVar(0), record->channel);
            xsmcSet(xsVar(1), xsID_channel, xsVar(0));

            xsmcSetBoolean(xsVar(0), 0);
            xsmcSet(xsVar(1), xsID_hidden, xsVar(0));

            xsmcSetArrayBuffer(xsVar(0), record->BSSID.octet, sizeof(record->BSSID.octet));
            xsmcSet(xsVar(1), xsID_bssid, xsVar(0));

            if (RTW_SECURITY_OPEN == record->security)
                xsmcSetString(xsVar(0), "none");
            else if (RTW_SECURITY_WEP_PSK == record->security)
                xsmcSetString(xsVar(0), "wep");
            else if (RTW_SECURITY_WPA_TKIP_PSK == record->security)
                xsmcSetString(xsVar(0), "wpa_tkip");
            else if (RTW_SECURITY_WPA_AES_PSK == record->security)
                xsmcSetString(xsVar(0), "wpa2_psk");
            else if (RTW_SECURITY_WPA2_AES_PSK == record->security)
                xsmcSetString(xsVar(0), "wpa2_aes");
            else if (RTW_SECURITY_WPA2_TKIP_PSK == record->security)
                xsmcSetString(xsVar(0), "wpa2_tkip");
            else if (RTW_SECURITY_WPA2_MIXED_PSK == record->security)
                xsmcSetString(xsVar(0), "wpa2_mixed");
            else if (RTW_SECURITY_WPA_WPA2_MIXED == record->security)
                xsmcSetString(xsVar(0), "wpa_wpa2_aes");
            else
                xsmcSetString(xsVar(0), "unknow");

            xsmcSet(xsVar(1), xsID_authentication, xsVar(0));

            xsCallFunction1(gScan->callback, xsGlobal, xsVar(1));
        }
        xsCatch {
        }
        ++ApNum;
    }
    else
    {
        ApNum = 0;
    }

    xsCallFunction1(gScan->callback, xsGlobal, xsNull);     // end of scan
    xsForget(gScan->callback);
    xsEndHost(gScan->the);

    return RTW_SUCCESS;
}

static void init_wifi_struct(void)
{
	memset(wifi.ssid.val, 0, sizeof(wifi.ssid.val));
	memset(wifi.bssid.octet, 0, ETH_ALEN);
	memset(password, 0, sizeof(password));
	wifi.ssid.len = 0;
	wifi.password = NULL;
	wifi.password_len = 0;
	wifi.key_id = -1;
	memset(ap.ssid.val, 0, sizeof(ap.ssid.val));
	ap.ssid.len = 0;
	ap.password = NULL;
	ap.password_len = 0;
	ap.channel = 1;
}

static void wifiEventPending(void *the, void *refcon, uint8_t *message, uint16_t messageLength)
{
	xsWiFi wifi = refcon;
	const char *msg;

	switch (gWiFiStatus) {
		case RTW_SUCCESS:
			msg = "connect";
			break;
		default:
			msg = "disconnect";
			break;
	}

	xsBeginHost(the);
		xsCall1(wifi->obj, xsID_callback, xsString(msg));
	xsEndHost(the);
}

void wlan_callback_handler(void)
{
 	xsWiFi walker;
	for (walker = gWiFi; NULL != walker; walker = walker->next)
	{
		modMessagePostToMachine(walker->the, NULL, 0, wifiEventPending, walker);
	}
}

void xs_wifi_connect(xsMachine *the)
{
	char *str;
	int argc = xsmcArgc;
	int ret;
	int channel;
	int assoc_by_bssid = 0;
	int pscan_config;

	if (0 == argc)
		return;

	initWiFi();

	init_wifi_struct();

	xsmcVars(2);
	xsmcGet(xsVar(0), xsArg(0), xsID_ssid);
	if (!xsmcTest(xsVar(0)))
		xsUnknownError("ssid required");
	str = xsmcToString(xsVar(0));
	strcpy((char *)wifi.ssid.val, str);
	wifi.ssid.len = strlen(str);

	xsmcGet(xsVar(0), xsArg(0), xsID_password);
	if (xsmcTest(xsVar(0))) {
		str = xsmcToString(xsVar(0));
		strcpy((char *)password, str);
		wifi.password = password;
		wifi.password_len = strlen(str);
	}

	if (xsmcHas(xsArg(0), xsID_bssid)) {
		xsmcGet(xsVar(0), xsArg(0), xsID_bssid);

		if (sizeof(wifi.bssid.octet) != xsmcGetArrayBufferLength(xsVar(0)))
			xsUnknownError("bssid must be 6 bytes");
		xsmcGetArrayBufferData(xsVar(0), 0, wifi.bssid.octet, sizeof(wifi.bssid.octet));
		assoc_by_bssid = 1;
	}

	if(wifi.password != NULL){
		if((wifi.key_id >= 0)&&(wifi.key_id <= 3)) {
			wifi.security_type = RTW_SECURITY_WEP_PSK;
		}
		else{
			wifi.security_type = RTW_SECURITY_WPA2_AES_PSK;
		}
	}
	else{
		wifi.security_type = RTW_SECURITY_OPEN;
	}

	if (xsmcHas(xsArg(0), xsID_channel)) {
		xsmcGet(xsVar(0), xsArg(0), xsID_channel);
		channel = xsmcToInteger(xsVar(0));
		if ((channel < 1) || (channel > 13))
			xsUnknownError("invalid channel");
		pscan_config = PSCAN_ENABLE;
		wifi_set_pscan_chan(&channel, &pscan_config, 1);
	}

	//Check if in AP mode
	wext_get_mode(WLAN0_NAME, &mode);
	if(mode == IW_MODE_MASTER) {
		dhcps_deinit();

		if (wifi_set_mode(RTW_MODE_STA) < 0)
		{
			printf("\n\rERROR: Wifi on failed!");
			return;
		}
	}

	if(assoc_by_bssid){
		printf("\n\rJoining BSS by BSSID "MAC_FMT" ...\n\r", MAC_ARG(wifi.bssid.octet));
		ret = wifi_connect_bssid(wifi.bssid.octet, (char*)wifi.ssid.val, wifi.security_type, (char*)wifi.password,
						ETH_ALEN, wifi.ssid.len, wifi.password_len, wifi.key_id, NULL);
	} else {
		printf("\n\rJoining BSS by SSID %s...\n\r", (char*)wifi.ssid.val);
		ret = wifi_connect((char*)wifi.ssid.val, wifi.security_type, (char*)wifi.password, wifi.ssid.len,
						wifi.password_len, wifi.key_id, NULL);
	}

	if(ret != RTW_SUCCESS){
		gWiFiStatus = ret;
	}
	else
	{
		gWiFiStatus = RTW_SUCCESS;
	}

	/* Start DHCPClient */
	LwIP_DHCP(0, 0/*DHCP_START*/);
}

void xs_wifi_destructor(void *data)
{
    xsWiFi wifi = data;

    if (wifi) {
        if (wifi == gWiFi)
            gWiFi = wifi->next;
        else {
            xsWiFi walker;
            for (walker = gWiFi; walker->next != wifi; walker = walker->next)
                ;
            walker->next = wifi->next;
        }

        c_free(wifi);
    }
}

void xs_wifi_constructor(xsMachine *the)
{
    int argc = xsmcArgc;

    if (1 == argc)
        xsCall1(xsThis, xsID_build, xsArg(0));
    else if (2 == argc)
        xsCall2(xsThis, xsID_build, xsArg(0), xsArg(1));
}

void xs_wifi_close(xsMachine *the)
{
    xsWiFi wifi = xsmcGetHostData(xsThis);
    if (wifi) {
        if (wifi->haveCallback)
            xsForget(wifi->obj);
    }
    xs_wifi_destructor(wifi);
    xsmcSetHostData(xsThis, NULL);
}

void xs_wifi_set_onNotify(xsMachine *the)
{
	xsWiFi wifi = xsmcGetHostData(xsThis);
	if (NULL == wifi) {
		wifi = c_calloc(1, sizeof(xsWiFiRecord));
		if (!wifi)
			xsUnknownError("out of memory");
		xsmcSetHostData(xsThis, wifi);
		wifi->the = the;
		wifi->obj = xsThis;
	}
	else if (wifi->haveCallback) {
		xsmcDelete(xsThis, xsID_callback);
		wifi->haveCallback = FALSE;
		xsForget(wifi->obj);
	}

	if (!xsmcTest(xsArg(0)))
		return;

	wifi->haveCallback = TRUE;

	xsRemember(wifi->obj);

	wifi->next = gWiFi;
	gWiFi = wifi;

	xsmcSet(xsThis, xsID_callback, xsArg(0));

	wlan_callback_handler();
}

void initWiFi(void)
{
    //LwIP_Init();
    //wifi_on(RTW_MODE_STA);
    //printf("%s \n",__FUNCTION__);
}

void xs_wifi_accessPoint(xsMachine *the)
{
    char *str;
    int timeout = 20;
    int ret = RTW_SUCCESS;
    struct ip_addr ipaddr;
    struct ip_addr netmask;
    struct ip_addr gw;
    struct netif * pnetif = &xnetif[0];

    initWiFi();
    init_wifi_struct();

    xsmcVars(2);

    xsmcGet(xsVar(0), xsArg(0), xsID_ssid);
    str = xsmcToString(xsVar(0));
    ap.ssid.len = c_strlen(str);
    if (ap.ssid.len > 32)
       xsUnknownError("ssid too long - 32 bytes max");
    c_strcpy(ap.ssid.val, str);

    ap.security_type = RTW_SECURITY_OPEN;
    if (xsmcHas(xsArg(0), xsID_password)) {
        xsmcGet(xsVar(0), xsArg(0), xsID_password);
        str = xsmcToString(xsVar(0));
        ap.password_len = c_strlen(str);
        if (c_strlen(str) > 64)
            xsUnknownError("password too long - 64 bytes max");
        if (c_strlen(str) < 8)
            xsUnknownError("password too short - 8 bytes min");
        if (!c_isEmpty(str)) {
            c_strcpy(password, str);
            ap.password = password;
            ap.security_type = RTW_SECURITY_WPA2_AES_PSK;
        }
    }

    ap.channel = 1;
    if (xsmcHas(xsArg(0), xsID_channel)) {
        xsmcGet(xsVar(0), xsArg(0), xsID_channel);
        ap.channel = xsmcToInteger(xsVar(0));
        if ((ap.channel < 1) || (ap.channel > 13))
            xsUnknownError("invalid channel");
    }

    dhcps_deinit();
    IP4_ADDR(ip_2_ip4(&ipaddr), GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
    IP4_ADDR(ip_2_ip4(&netmask), NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
    IP4_ADDR(ip_2_ip4(&gw), GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
    netif_set_addr(pnetif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask),ip_2_ip4(&gw));

    wifi_off();
    vTaskDelay(20);
    if (wifi_on(RTW_MODE_AP) < 0){
        xsUnknownError("wifi_on failed");
    }

    if((ret = wifi_start_ap((char*)ap.ssid.val, ap.security_type, (char*)ap.password, ap.ssid.len, ap.password_len, ap.channel) )< 0) {
        xsUnknownError("wifi_start_ap failed");
    }

    while(1) {
        char essid[33];
        if(wext_get_ssid(WLAN0_NAME, (unsigned char *) essid) > 0) {
            if(strcmp((const char *) essid, (const char *)ap.ssid.val) == 0) {
                printf("%s started\n", ap.ssid.val);
                break;
            }
        }

        if(timeout == 0) {
            xsUnknownError("ERROR: Start AP timeout!\n");
            break;
        }

        vTaskDelay(1 * 1000/*configTICK_RATE_HZ*/);
        timeout --;
    }
    dhcps_init(pnetif);

}
