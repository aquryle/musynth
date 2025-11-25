/***************************************************************************//**
 * @file	wave.c
 * @brief
 * @date	2025/11/23
 * @author	sata
 * @version	1.00
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


__attribute__((section(".wave_buffer")))
volatile uint8_t WaveBuffer[0x1000] = {0};


void wave_gen(uint8_t note)
{
	for (size_t i = 0; i < 0x100; i++) {
		WaveBuffer[i] = (uint8_t)(i + note);
	}
}

