/*
 * Copyright (c) 2018-2019  Moddable Tech, Inc.
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

#define __XS6PLATFORMMINIMAL__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "xs.h"
#include "xsHost.h"
#include "xsPlatform.h"

#include "xsmain.h"
#include "modTimer.h"
#include "xsPlatform.h"
#include "modInstrumentation.h"

#define xsMain_THREAD_PRIORITY		(2)
#define xsMain_THREAD_STACK_SIZE	(1024 * 20)

extern void mc_setup(xsMachine *the);

static xsMachine *gThe;		// the main XS virtual machine running

void setup(void)
{
    gThe = ESP_cloneMachine(0, 0, 0, NULL);
    mc_setup(gThe);
}

void loop_task(void *pvParameter)
{
    setup();

    if (!gThe)
    {
        printf("gThe == NULL %s, %d\n",__FUNCTION__,__LINE__);
        return;
    }

	while (true) 
    {
		modTimersExecute();
		modMessageService(gThe, modTimersNext());
	}
}

void xs_start() {

	//#if 0 == CONFIG_LOG_DEFAULT_LEVEL
	//	#define kStack (((8 * 1024) + XT_STACK_EXTRA_CLIB) / sizeof(StackType_t))
	//#else
	//	#define kStack (((10 * 1024) + XT_STACK_EXTRA_CLIB) / sizeof(StackType_t))
	//#endif
	#define kStack 10*1024
    xTaskCreate(loop_task, "xsmain", kStack, NULL, 1, NULL);
}

