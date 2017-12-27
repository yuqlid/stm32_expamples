/*
 * AS5048A.c
 *
 *  Created on: 2017/12/27
 *      Author: kusakabe
 */

#include <stdint.h>

uint16_t parity(uint16_t x){

	uint16_t parity = 0;

	while(x != 0){
		parity ^= x;
		x >>= 1;
	}

	return (parity & 0x1);
}

void AS5048A_Write(uint16_t address, uint16_t data){
	if (parity(address & 0x3FFF) == 1) address = address | 0x8000; // set parity bit
	if (parity(data & 0x3FFF) == 1) data = data | 0x8000; // set parity bit
}
