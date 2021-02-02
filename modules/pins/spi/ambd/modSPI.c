/*
 * Copyright (c) 2016-2019  Moddable Tech, Inc.
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

#include "xsHost.h"

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "modSPI.h"
#include "mc.defines.h"

#define IRAM_ATTR

#ifndef MODDEF_SPI_MISO_PIN
	#define MODDEF_SPI_MISO_PIN	PB_5
#endif
#ifndef MODDEF_SPI_MOSI_PIN
	#define MODDEF_SPI_MOSI_PIN	PB_4
#endif
#ifndef MODDEF_SPI_SCK_PIN
	#define MODDEF_SPI_SCK_PIN	PB_6
#endif

typedef uint16_t (*modSPIBufferLoader)(uint8_t *data, uint16_t bytes, uint16_t *bitsOut);

static uint16_t modSpiLoadBufferAsIs(uint8_t *data, uint16_t bytes, uint16_t *bitsOut);
static uint16_t modSpiLoadBufferSwap16(uint8_t *data, uint16_t bytes, uint16_t *bitsOut);
static uint16_t modSpiLoadBufferRGB565LEtoRGB444(uint8_t *data, uint16_t bytes, uint16_t *bitsOut);
static uint16_t modSpiLoadBufferRGB332To16BE(uint8_t *data, uint16_t bytes, uint16_t *bitsOut);
static uint16_t modSpiLoadBufferGray256To16BE(uint8_t *data, uint16_t bytes, uint16_t *bitsOut);
static uint16_t modSpiLoadBufferGray16To16BE(uint8_t *data, uint16_t bytes, uint16_t *bitsOut);
//static uint16_t modSpiLoadBufferCLUT16To16BE(uint8_t *data, uint16_t bytes, uint16_t *bitsOut);

static modSPIConfiguration gConfig;
static uint8_t *gSPIData;
static volatile int16_t gSPIDataCount = -1;
static modSPIBufferLoader gSPIBufferLoader;
static uint16_t *gCLUT16;
static uint8_t gSPIInited;

#define DataFrameSize	8
#define SPI_BUFFER_SIZE (512) // SPI_BUFFER_SIZE can be larger than 64... tested up to 512.

spi_t spi_master;

static uint32_t gSPITxBuffer[SPI_BUFFER_SIZE];
static uint32_t gSPIRxBuffer[SPI_BUFFER_SIZE];

// queue up the next one
void postTransfer(void *refcon)
{
	modSPIConfiguration config = (modSPIConfiguration)refcon;
	uint16_t loaded, bitsOut;
	int i=0;

	if (gSPIDataCount <= 0) {
		gSPIDataCount = -1;
		gSPIData = NULL;
		return;
	}

	loaded = (gSPIBufferLoader)(gSPIData, (gSPIDataCount <= SPI_BUFFER_SIZE) ? gSPIDataCount : SPI_BUFFER_SIZE, &bitsOut);
	gSPIDataCount -= loaded;
	gSPIData += loaded;

	for(i = 0 ; i < loaded/4; i++)
	{
		spi_master_write(&spi_master, (0xff & gSPITxBuffer[i]));
		spi_master_write(&spi_master, (0xff00 & gSPITxBuffer[i])>> 8);
		spi_master_write(&spi_master, (0xff0000 & gSPITxBuffer[i]) >> 16);
		spi_master_write(&spi_master, (0xff000000 & gSPITxBuffer[i]) >> 24);
	}
}

void modSPIInit(modSPIConfiguration config)
{
	if (!gSPIInited) {
		spi_master.spi_idx = MBED_SPI1;
		spi_init(&spi_master, MODDEF_SPI_MOSI_PIN, MODDEF_SPI_MISO_PIN, MODDEF_SPI_SCK_PIN, PB_7);
		spi_format(&spi_master, DataFrameSize, 0, 0);
		spi_frequency(&spi_master, config->hz);

		_memset(gSPITxBuffer, 0, SPI_BUFFER_SIZE);
		_memset(gSPIRxBuffer, 0, SPI_BUFFER_SIZE);

		gSPIInited = true;
	}

	if (NULL == gConfig) {
		gSPIData = NULL;
		gSPIDataCount = -1;
		gConfig = config;
	}
	else
		modSPIFlush();
}

void modSPIUninit(modSPIConfiguration config)
{
	if (config == gConfig)
		modSPIActivateConfiguration(NULL);

	gSPIInited -= 1;
	if (gSPIInited)
		return;

	spi_free(&spi_master);
}

void modSPIActivateConfiguration(modSPIConfiguration config)
{
	modSPIFlush();

	if (config == gConfig)
		return;

	if (gConfig)
		(gConfig->doChipSelect)(0, gConfig);

	gConfig = config;
	if (gConfig)
		(gConfig->doChipSelect)(1, gConfig);
}

// data must be long aligned
uint16_t IRAM_ATTR modSpiLoadBufferAsIs(uint8_t *data, uint16_t bytes, uint16_t *bitsOut)
{
	uint32_t *from = (uint32_t *)data, *to = gSPITxBuffer;
	uint8_t longs;
	uint16_t result = bytes;

	*bitsOut = bytes << 3;

	while (bytes >= 64) {
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		*to++ = *from++;
		bytes -= 64;
	}

	longs = (bytes + 3) >> 2;
	while (longs--)
		*to++ = *from++;

	return result;
}

uint16_t IRAM_ATTR modSpiLoadBufferSwap16(uint8_t *data, uint16_t bytes, uint16_t *bitsOut)
{
	uint32_t *from = (uint32_t *)data;
	uint32_t *to = gSPITxBuffer;
	const uint32_t mask = 0x00ff00ff;
	uint32_t twoPixels;
	uint16_t result = bytes;
	uint8_t longs;

	*bitsOut = bytes << 3;

	while (bytes >= 64) {
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
		bytes -= 64;
	}

	longs = (bytes + 3) >> 2;
	while (longs--) {
		twoPixels = *from++; *to++ = ((twoPixels & mask) << 8) | ((twoPixels >> 8) & mask);
	}

	return result;

}

uint16_t IRAM_ATTR modSpiLoadBufferRGB565LEtoRGB444(uint8_t *data, uint16_t bytes, uint16_t *bitsOut)
{
	uint16_t *from = (uint16_t *)data;
	uint8_t *to = (uint8_t *)gSPITxBuffer;
	uint16_t remain;

	if (bytes > SPI_BUFFER_SIZE)
		bytes = SPI_BUFFER_SIZE;

	*bitsOut = (bytes >> 1) * 12;

	remain = bytes >> 2;
	while (remain--) {
		uint8_t r, g, b;
		uint16_t rgb565 = *from++;
		r = rgb565 >> 12, g = (rgb565 >> 7) & 0x0F, b = (rgb565 >> 1) & 0x0F;
		*to++ = (r << 4) | g;

		rgb565 = *from++;
		r = rgb565 >> 12;
		*to++ = (b << 4) | r;
		g = (rgb565 >> 7) & 0x0F, b = (rgb565 >> 1) & 0x0F;
		*to++ = (g << 4) | b;
	}

	if (bytes & 2) {
		uint8_t r, g, b;
		uint16_t rgb565 = *from;
		r = rgb565 >> 12, g = (rgb565 >> 7) & 0x0F, b = (rgb565 >> 1) & 0x0F;
		*to++ = (r << 4) | g;
		*to = b << 4;
	}

	return bytes;		// input bytes consumed
}

uint16_t IRAM_ATTR modSpiLoadBufferRGB332To16BE(uint8_t *data, uint16_t bytes, uint16_t *bitsOut)
{
	uint8_t *from = (uint8_t *)data;
	uint32_t *to = gSPITxBuffer;
	uint16_t remain;

	if (bytes > (SPI_BUFFER_SIZE >> 1))
		bytes = SPI_BUFFER_SIZE >> 1;

	*bitsOut = bytes << 4;		// output bits are double input bits

	remain = bytes;
	while (remain >= 2) {
		uint8_t rgb332, r, g, b;
		uint16_t pixela, pixelb;

		rgb332 = *from++;
		r = rgb332 >> 5, g = (rgb332 >> 2) & 0x07, b = rgb332 & 0x03;
		r = (r << 2) | (r >> 2), g |= g << 3, b = (b << 3) | (b << 1) | (b >> 1);
		pixela = (r << 11) | (g << 5) | b;
		pixela = (pixela >> 8) | (pixela << 8);

		rgb332 = *from++;
		r = rgb332 >> 5, g = (rgb332 >> 2) & 0x07, b = rgb332 & 0x03;
		r = (r << 2) | (r >> 2), g |= g << 3, b = (b << 3) | (b << 1) | (b >> 1);
		pixelb = (r << 11) | (g << 5) | b;
		pixelb = (pixelb >> 8) | (pixelb << 8);

		*to++ = (pixelb << 16) | pixela;
		remain -= 2;
	}

	if (remain) {
		uint16_t pixela;
		uint8_t rgb332 = *from++, r, g, b;
		r = rgb332 >> 5, g = (rgb332 >> 2) & 0x07, b = rgb332 & 0x03;
		r = (r << 2) | (r >> 2), g |= g << 3, b = (b << 3) | (b << 1) | (b >> 1);
		pixela = (r << 11) | (g << 5) | b;
		pixela = (pixela >> 8) | (pixela << 8);
		*to++ = (0 << 16) | pixela;
	}

	return bytes;		// input bytes consumed
}

uint16_t IRAM_ATTR modSpiLoadBufferGray256To16BE(uint8_t *data, uint16_t bytes, uint16_t *bitsOut)
{
	uint8_t *from = (uint8_t *)data;
	uint32_t *to = gSPITxBuffer;
	uint16_t remain;

	if (bytes > (SPI_BUFFER_SIZE >> 1))
		bytes = SPI_BUFFER_SIZE >> 1;

	*bitsOut = bytes << 4;		// output bits are double input bits

	remain = bytes;
	while (remain >= 2) {
		uint8_t gray;
		uint16_t pixela, pixelb;

		gray = *from++;
		pixela = ((gray >> 3) << 11) | ((gray >> 2) << 5) | (gray >> 3);
		pixela = (pixela >> 8) | (pixela << 8);

		gray = *from++;
		pixelb = ((gray >> 3) << 11) | ((gray >> 2) << 5) | (gray >> 3);
		pixelb = (pixelb >> 8) | (pixelb << 8);

		*to++ = (pixelb << 16) | pixela;
		remain -= 2;
	}

	if (remain) {
		uint8_t gray = *from++;
		uint16_t pixela = ((gray >> 3) << 11) | ((gray >> 2) << 5) | (gray >> 3);
		pixela = (pixela >> 8) | (pixela << 8);
		*to++ = (0 << 16) | pixela;
	}

	return bytes;		// input bytes consumed
}

uint16_t IRAM_ATTR modSpiLoadBufferGray16To16BE(uint8_t *data, uint16_t bytes, uint16_t *bitsOut)
{
	uint8_t *from = (uint8_t *)data;
	uint32_t *to = gSPITxBuffer;
	uint16_t i;

	if (bytes > (SPI_BUFFER_SIZE >> 2))
		bytes = SPI_BUFFER_SIZE >> 2;
	*bitsOut = bytes << 5;		// output bits are quadruple input bits

	for (i = 0; i < bytes; i++) {
		uint8_t gray;
		uint8_t twoPixels = *from++;
		uint16_t pixela, pixelb;

		gray = (twoPixels >> 2) & 0x3C;
		gray |= gray >> 4;				// 6 bits of gray
		pixela = gray >> 1;				// 5 bita of gray
		pixela |= (pixela << 11) | (gray << 5);
		pixela = (pixela >> 8) | (pixela << 8);

		gray = (twoPixels << 2) & 0x3C;
		gray |= gray >> 4;
		pixelb = gray >> 1;
		pixelb |= (pixelb << 11) | (gray << 5);
		pixelb = (pixelb >> 8) | (pixelb << 8);

		*to++ = (pixelb << 16) | pixela;
	}

	return bytes;		// input bytes consumed
}

uint16_t IRAM_ATTR modSpiLoadBufferCLUT16To16BE(uint8_t *data, uint16_t bytes, uint16_t *bitsOut)
{
	uint16_t *clut16_t = gCLUT16;
	uint8_t *from = (uint8_t *)data;
	uint32_t *to = gSPITxBuffer;
	uint16_t i;

	if (bytes > (SPI_BUFFER_SIZE >> 2))
		bytes = SPI_BUFFER_SIZE >> 2;
	*bitsOut = bytes << 5;		// output bits are quadruple input bits

	for (i = 0; i < bytes; i++) {
		uint16_t pixelA, pixelB;
		uint8_t twoPixels = *from++;

		pixelA = clut16_t[twoPixels >> 4];
		pixelA = (pixelA >> 8) | (pixelA << 8);

		pixelB = clut16_t[twoPixels & 0x0F];
		pixelB = (pixelB >> 8) | (pixelB << 8);
		*to++ = (pixelB << 16) | pixelA;
	}

	return bytes;		// input bytes consumed
}

// N.B. callers assume this funtion is synchronous
void modSPITxRx(modSPIConfiguration config, uint8_t *data, uint16_t count)
{
	int i;
	modSPIActivateConfiguration(config);

	for(i = 0 ; i < count; i++)
	{
		data[i] = spi_master_write(&spi_master, data[i]);
	}
}

void modSPIFlush(void)
{
	//while (gSPIDataCount >= 0)
	//	;
}

static void modSPITxCommon(modSPIConfiguration config, uint8_t *data, uint16_t count, modSPIBufferLoader loader)
{
	modSPIActivateConfiguration(config);

	gSPIBufferLoader = loader;
	gSPIData = data;
	gSPIDataCount = count;

	postTransfer(config);

	if (config->sync)
		modSPIFlush();
}

void modSPITx(modSPIConfiguration config, uint8_t *data, uint16_t count)
{
	modSPITxCommon(config, data, count, modSpiLoadBufferAsIs);
}

void modSPITxSwap16(modSPIConfiguration config, uint8_t *data, uint16_t count)
{
	modSPITxCommon(config, data, count, modSpiLoadBufferSwap16);
}

void modSPITxRGB565LEtoRGB444(modSPIConfiguration config, uint8_t *data, uint16_t count)
{
	modSPITxCommon(config, data, count, modSpiLoadBufferRGB565LEtoRGB444);
}

void modSPITxRGB332To16BE(modSPIConfiguration config, uint8_t *data, uint16_t count)
{
	modSPITxCommon(config, data, count, modSpiLoadBufferRGB332To16BE);
}

void modSPITxGray256To16BE(modSPIConfiguration config, uint8_t *data, uint16_t count)
{
	modSPITxCommon(config, data, count, modSpiLoadBufferGray256To16BE);
}

void modSPITxGray16To16BE(modSPIConfiguration config, uint8_t *data, uint16_t count)
{
	modSPITxCommon(config, data, count, modSpiLoadBufferGray16To16BE);
}

void modSPITxCLUT16To16BE(modSPIConfiguration config, uint8_t *data, uint16_t count, uint16_t *colors)
{
	gCLUT16 = colors;
	modSPITxCommon(config, data, count, modSpiLoadBufferCLUT16To16BE);
}

