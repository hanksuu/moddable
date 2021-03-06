/*
 * Copyright (c) 2017-2020  Moddable Tech, Inc.
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
#include "xsHost.h"

#include "modGPIO.h"

/*
	gpio
*/

#define kUninitializedPin (255)

int modGPIOInit(modGPIOConfiguration config, const char *port, uint8_t pin, uint32_t mode)
{
	int result;

	gpio_init(&config->gpio, pin);

	config->pin = pin;
	result = modGPIOSetMode(config, mode);
	if (result) {
		config->pin = kUninitializedPin;
		return result;
	}

	return 0;
}

void modGPIOUninit(modGPIOConfiguration config)
{
	config->pin = kUninitializedPin;
}

int modGPIOSetMode(modGPIOConfiguration config, uint32_t mode)
{
	switch (mode) {
		case kModGPIOOutput:
		case kModGPIOOutputOpenDrain:
			gpio_dir(&config->gpio,PIN_OUTPUT);
			gpio_mode(&config->gpio, PullNone);
			break;

		case kModGPIOInput:
		case kModGPIOInputPullUp:
		case kModGPIOInputPullDown:
		//case kModGPIOInputPullUpDown:
			gpio_dir(&config->gpio, PIN_INPUT);

			if (kModGPIOInputPullUp == mode)
				gpio_mode(&config->gpio, PullUp);
			else if (kModGPIOInputPullDown == mode)
				gpio_mode(&config->gpio, PullDown);
			break;

		default:
			return -1;
	}

	return 0;
}

uint8_t modGPIORead(modGPIOConfiguration config)
{
    return gpio_read(&config->gpio);
}

void modGPIOWrite(modGPIOConfiguration config, uint8_t value)
{
    gpio_write(&config->gpio, value ? 1 : 0);
}
