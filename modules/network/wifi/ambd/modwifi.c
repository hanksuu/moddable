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

static int gMode = 0;
static void initWiFi(void);
static rtw_result_t app_scan_result_handler( rtw_scan_handler_result_t* malloced_scan_result );

struct wifiScanRecord {
    xsSlot          callback;
    xsMachine       *the;
};
typedef struct wifiScanRecord wifiScanRecord;

static wifiScanRecord *gScan;

void xs_wifi_set_mode(xsMachine *the)
{
    int mode = xsmcToInteger(xsArg(0));

    initWiFi();
    if (1 == mode)
    {
        wifi_set_mode(RTW_MODE_STA);
        gMode = 1;
    }
    else if (2 == mode)
    {
        wifi_set_mode(RTW_MODE_AP);
        gMode = 2;
    }
    else if (3 == mode)
    {
        wifi_set_mode(RTW_MODE_STA_AP);
        gMode = 3;
    }
    else
    {
        xsUnknownError("invalid mode");
    }
}

void xs_wifi_get_mode(xsMachine *the)
{
    initWiFi();

    if (1 == gMode)
        xsmcSetInteger(xsResult, 1);
    else if (2 == gMode)
        xsmcSetInteger(xsResult, 2);
    else if (3 == gMode)
        xsmcSetInteger(xsResult, 3);
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

void xs_wifi_connect(xsMachine *the)
{
#if 0
    wifi_config_t config;
    char *str;
    int argc = xsmcArgc;
    wifi_mode_t mode;
    int channel;

    if (gWiFiState > 1) {
        gWiFiState = 2;
        gDisconnectReason = 0;
        esp_wifi_disconnect();
    }

    if (0 == argc)
        return;

    initWiFi();

    c_memset(&config, 0, sizeof(config));

    config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    xsmcVars(2);
    xsmcGet(xsVar(0), xsArg(0), xsID_ssid);
    if (!xsmcTest(xsVar(0)))
        xsUnknownError("ssid required");
    str = xsmcToString(xsVar(0));
    if (espStrLen(str) > (sizeof(config.sta.ssid) - 1))
        xsUnknownError("ssid too long - 32 bytes max");
    espMemCpy(config.sta.ssid, str, espStrLen(str));

    xsmcGet(xsVar(0), xsArg(0), xsID_password);
    if (xsmcTest(xsVar(0))) {
        str = xsmcToString(xsVar(0));
        if (espStrLen(str) > (sizeof(config.sta.password) - 1))
            xsUnknownError("password too long - 64 bytes max");
        espMemCpy(config.sta.password, str, espStrLen(str));
    }

    if (xsmcHas(xsArg(0), xsID_bssid)) {
        xsmcGet(xsVar(0), xsArg(0), xsID_bssid);
        if (sizeof(config.sta.bssid) != xsmcGetArrayBufferLength(xsVar(0)))
            xsUnknownError("bssid must be 6 bytes");
        xsmcGetArrayBufferData(xsVar(0), 0, config.sta.bssid, sizeof(config.sta.bssid));
        config.sta.bssid_set = 1;
    }

    if (xsmcHas(xsArg(0), xsID_channel)) {
        xsmcGet(xsVar(0), xsArg(0), xsID_channel);
        channel = xsmcToInteger(xsVar(0));
        if ((channel < 1) || (channel > 13))
            xsUnknownError("invalid channel");
        config.sta.channel = channel;
    }

    esp_wifi_get_mode(&mode);
    if ((WIFI_MODE_STA != mode) && (WIFI_MODE_APSTA != mode))
        esp_wifi_set_mode(WIFI_MODE_STA);

    esp_wifi_set_config(WIFI_IF_STA, &config);

    gWiFiConnectRetryRemaining = MODDEF_WIFI_ESP32_CONNECT_RETRIES;
    if (0 != esp_wifi_connect())
        xsUnknownError("esp_wifi_connect failed");
#else
    printf("%s, need to implement\n",__FUNCTION__);
#endif
}

typedef struct xsWiFiRecord xsWiFiRecord;
typedef xsWiFiRecord *xsWiFi;

struct xsWiFiRecord {
    xsWiFi              next;
    xsMachine           *the;
    xsSlot              obj;
    uint8_t             haveCallback;
};

static xsWiFi gWiFi;

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
#if 0
    xsWiFi wifi = xsmcGetHostData(xsThis);
    //uint32_t deviceId = qca4020_wlan_get_active_device();

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
        wifi->haveCallback = false;
        xsForget(wifi->obj);
    }

    if (!xsmcTest(xsArg(0)))
        return;

    wifi->haveCallback = true;

    xsRemember(wifi->obj);

    wifi->next = gWiFi;
    gWiFi = wifi;

    xsmcSet(xsThis, xsID_callback, xsArg(0));
#else
    printf("%s, need to implement\n",__FUNCTION__);
#endif
}

void initWiFi(void)
{
    //LwIP_Init();
    //wifi_on(RTW_MODE_STA);
    //printf("%s \n",__FUNCTION__);
}

void xs_wifi_accessPoint(xsMachine *the)
{
#if 0
    wifi_mode_t mode;
    wifi_config_t config;
    wifi_ap_config_t *ap;
    tcpip_adapter_ip_info_t info;
    char *str;
    uint8_t station = 0;

    initWiFi();

    c_memset(&config, 0, sizeof(config));
    ap = &config.ap;

    xsmcVars(2);

    xsmcGet(xsVar(0), xsArg(0), xsID_ssid);
    str = xsmcToString(xsVar(0));
    ap->ssid_len = c_strlen(str);
    if (ap->ssid_len > (sizeof(ap->ssid) - 1))
        xsUnknownError("ssid too long - 32 bytes max");
    c_memcpy(ap->ssid, str, ap->ssid_len);

    ap->authmode = WIFI_AUTH_OPEN;
    if (xsmcHas(xsArg(0), xsID_password)) {
        xsmcGet(xsVar(0), xsArg(0), xsID_password);
        str = xsmcToString(xsVar(0));
        if (c_strlen(str) > (sizeof(ap->password) - 1))
            xsUnknownError("password too long - 64 bytes max");
        if (c_strlen(str) < 8)
            xsUnknownError("password too short - 8 bytes min");
        if (!c_isEmpty(str)) {
            c_memcpy(ap->password, str, c_strlen(str));
            ap->authmode = WIFI_AUTH_WPA_WPA2_PSK;
        }
    }

    ap->channel = 1;
    if (xsmcHas(xsArg(0), xsID_channel)) {
        xsmcGet(xsVar(0), xsArg(0), xsID_channel);
        ap->channel = xsmcToInteger(xsVar(0));
        if ((ap->channel < 1) || (ap->channel > 13))
            xsUnknownError("invalid channel");
    }

    ap->ssid_hidden = 0;
    if (xsmcHas(xsArg(0), xsID_hidden)) {
        xsmcGet(xsVar(0), xsArg(0), xsID_hidden);
        ap->ssid_hidden = xsmcTest(xsVar(0));
    }

    ap->max_connection = 4;
    if (xsmcHas(xsArg(0), xsID_max)) {
        xsmcGet(xsVar(0), xsArg(0), xsID_max);
        ap->max_connection = xsmcToInteger(xsVar(0));
    }

    ap->beacon_interval = 100;
    if (xsmcHas(xsArg(0), xsID_interval)) {
        xsmcGet(xsVar(0), xsArg(0), xsID_interval);
        ap->beacon_interval = xsmcToInteger(xsVar(0));
    }

    if (xsmcHas(xsArg(0), xsID_station)) {
        xsmcGet(xsVar(0), xsArg(0), xsID_station);
        station = xsmcToBoolean(xsVar(0));
    }

    esp_wifi_get_mode(&mode);
    if ((WIFI_MODE_AP != mode) && (WIFI_MODE_APSTA != mode)) {
        if (ESP_OK != esp_wifi_set_mode(station ? WIFI_MODE_APSTA : WIFI_MODE_AP))
            xsUnknownError("esp_wifi_set_mode failed");
    }

    if (ESP_OK != esp_wifi_set_config(ESP_IF_WIFI_AP, &config))
        xsUnknownError("esp_wifi_set_config failed");
    if (ESP_OK != esp_wifi_start())
        xsUnknownError("esp_wifi_start failed");
    if (ESP_OK != tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &info))
        xsUnknownError("tcpip_adapter_get_ip_info failed");
    if (0 == info.ip.addr)
        xsUnknownError("IP config bad when starting Wi-Fi AP!");
#else
    printf("%s, need to implement\n",__FUNCTION__);
#endif
}
