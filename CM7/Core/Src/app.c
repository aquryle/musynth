/**
 * @file	app.c
 * @brief
 * @date	2025/11/23
 * @author	sata
 * @version	1.00
*/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "wave.h"

static void initialize(void);
static void deinitialize(void);



static void initialize(void)
{
	volatile uint8_t i = 0;
	i++;
}

static void deinitialize(void)
{
	volatile uint8_t i = 0;
	i++;
}

void app_exec(void)
{
	initialize();
	wave_gen(0);
	deinitialize();
}
