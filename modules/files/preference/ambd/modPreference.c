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

#include "xsmc.h"
#include "xsHost.h"

#include "dct.h"

#define kPreferencesBeginAddr  0x1F0000     /*!< Preferences begin address of flash, ex: 0x100000 = 1M */
#define kPreferencesKeySize	   32           /*!< max size of the variable name */
#define kPreferencesValueSize  64           /*!< max size of the variable value */
#define kPreferencesMagic      0x81958711
#define kPreferencesDomainNum  2            /*!< max number of module */

#if kPreferencesDomainNum > 2
#error "defult max DCT size is 8k(change kPreferencesBeginAddr in platform_opts.h), module size is 4k, max module number is 2; if backup enabled, the total module number is 2 + 1*2 = 4, the size is 16k; if wear leveling enabled, the total module number is 2 + 2*2 + 3*2 = 12, the size is 48k"
#endif

enum {
	kPrefsTypeBoolean = 1,
	kPrefsTypeInteger = 2,
	kPrefsTypeString = 3,
	kPrefsTypeBuffer = 4
};

static int32_t initPref(char *domain);
static int32_t setPref(char *domain, char *key, uint8_t type, uint8_t *value, uint16_t byteCount);

void xs_preference_set(xsMachine *the)
{
	uint8_t success;
	uint8_t boolean;
	int32_t integer;
	double dbl;
	char *str;

	if ((c_strlen(xsmcToString(xsArg(0))) > 31) || (c_strlen(xsmcToString(xsArg(1))) > 31))
		xsUnknownError("too long");

	switch (xsmcTypeOf(xsArg(2))) {
		case xsBooleanType:
			boolean = xsmcToBoolean(xsArg(2));
			success = setPref(xsmcToString(xsArg(0)), xsmcToString(xsArg(1)), kPrefsTypeBoolean, (uint8_t *)&boolean, 1);
			break;

		case xsIntegerType:
			integer = xsmcToInteger(xsArg(2));
			success = setPref(xsmcToString(xsArg(0)), xsmcToString(xsArg(1)), kPrefsTypeInteger, (uint8_t *)&integer, sizeof(integer));
			break;

		case xsNumberType:
			dbl = xsmcToNumber(xsArg(2));
			integer = (int32_t)dbl;
			if (dbl != integer)
				xsUnknownError("float unsupported");
			success = setPref(xsmcToString(xsArg(0)), xsmcToString(xsArg(1)), kPrefsTypeInteger, (uint8_t *)&integer, sizeof(integer));
			break;

		case xsStringType:
			str = xsmcToString(xsArg(2));
			success = setPref(xsmcToString(xsArg(0)), xsmcToString(xsArg(1)), kPrefsTypeString, (uint8_t *)str, c_strlen(str) + 1);
			break;

		case xsReferenceType:
			if (xsmcIsInstanceOf(xsArg(2), xsArrayBufferPrototype))
				success = setPref(xsmcToString(xsArg(0)), xsmcToString(xsArg(1)), kPrefsTypeBuffer, xsmcToArrayBuffer(xsArg(2)), xsmcGetArrayBufferLength(xsArg(2)));
			else
				goto unknown;
			break;

		unknown:
		default:
			xsUnknownError("unsupported type");
	}

	if (!success)
		xsUnknownError("can't save prefs");
}

void xs_preference_get(xsMachine *the)
{
	dct_handle_t handle;
	int32_t ret = -1;
	uint16_t DataLen = 0;
	char *pref = NULL;
	char Data[64];
	char *domain = xsmcToString(xsArg(0));
	char *key = xsmcToString(xsArg(1));

	ret = dct_open_module(&handle, domain);
	if (ret != DCT_SUCCESS){
		printf("dct_open_module failed\n"); // most likely that domain doesn't exist yet
		return;
	}

	c_memset(Data, 0, sizeof(Data));
	DataLen = sizeof(Data);
	ret = dct_get_variable_new(&handle,key,Data,&DataLen);
	if (ret != DCT_SUCCESS){
		ret = DCT_SUCCESS;
		return;
	}

	pref = Data;
	if (kPreferencesMagic != c_read32(pref)) {
		ret = -1;
		goto bail;
	}

	pref += sizeof(kPreferencesMagic);

	switch (*pref++) {
		case kPrefsTypeBoolean:
			xsmcSetBoolean(xsResult, *pref);
			break;
		case kPrefsTypeInteger:
			xsmcSetInteger(xsResult, c_read32(pref));
			break;
		case kPrefsTypeString:
			xsmcSetString(xsResult, (char*)pref);
			break;
		case kPrefsTypeBuffer:
			xsmcSetArrayBuffer(xsResult, pref + 2, c_read16(pref));
			break;
	}

bail:
	dct_close_module(&handle);
	if (DCT_SUCCESS != ret)
		xsUnknownError("can't read pref");
}

void xs_preference_delete(xsMachine *the)
{
	dct_handle_t handle;
	int32_t ret = -1;
	char *domain = xsmcToString(xsArg(0));
	char *key = xsmcToString(xsArg(1));

	ret = dct_open_module(&handle, domain);
	if (ret != DCT_SUCCESS){
		printf("preference open failed\n"); // most likely that domain doesn't exist yet
		return;
	}

	ret = dct_delete_variable(&handle, key);
	//if(ret == DCT_ERR_NOT_FIND || ret == DCT_SUCCESS)
	//	printf("preference delete %s success.\n", key);
	//else
	//	printf("preference delete failed");

	dct_close_module(&handle);
}

void xs_preference_keys(xsMachine *the)
{
	xsUnknownError("unimplemented");
}

int32_t setPref(char *domain, char *key, uint8_t type, uint8_t *value, uint16_t byteCount)
{
	dct_handle_t handle;
	int32_t ret = -1;
	uint32_t prefSize;
	char buffer[kPreferencesValueSize];
	char *pref = buffer;

	if (byteCount > 63)
		return 0;

	ret = initPref(domain);
	if (DCT_SUCCESS != ret)
		goto bail;

	ret = dct_open_module(&handle, domain);
	if (DCT_SUCCESS != ret)
		goto bail;

	// build pref entry
	prefSize = sizeof(kPreferencesMagic) + 1 + byteCount + ((kPrefsTypeBuffer == type) ? 2 : 0);
	if (prefSize > sizeof(buffer)) {
		ret = DCT_ERR_INVALID;
		goto bail;
	}

	c_memset(buffer, 0, prefSize);
	pref[0] = (uint8_t)(kPreferencesMagic & 0xFF);
	pref[1] = (uint8_t)((kPreferencesMagic >> 8) & 0xFF);
	pref[2] = (uint8_t)((kPreferencesMagic >> 16) & 0xFF);
	pref[3] = (uint8_t)((kPreferencesMagic >> 24) & 0xFF);
	pref += sizeof(kPreferencesMagic);
	*pref++ = type;
	if (kPrefsTypeBuffer == type) {
		pref[0] = (uint8_t)byteCount;
		pref[1] = (uint8_t)(byteCount >> 8);
		pref += 2;
	}

	c_memcpy(pref, value, byteCount);

	ret = dct_set_variable_new(&handle,key,buffer,prefSize);
	if (DCT_SUCCESS != ret)
		goto bail;

bail:
	dct_close_module(&handle);
	return (DCT_SUCCESS == ret ? 1 : 0);

}

int32_t initPref(char *domain)
{
	int32_t ret = -1;
	ret = dct_init(kPreferencesBeginAddr, kPreferencesDomainNum, kPreferencesKeySize, kPreferencesValueSize, 1, 0);
	ret = dct_register_module(domain);

	return ret;
}
