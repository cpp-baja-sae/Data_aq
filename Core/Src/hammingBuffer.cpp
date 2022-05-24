#include <hammingBuffer.hpp>
#include "main.h"

const uint64_t MASKS[7] = {
		0b1101101010110101010101010110101010101010101010101010101010000001,
		0b1011011001101100110011001101100110011001100110011001100110000010,
		0b0111000111100011110000111100011110000111100001111000011110000100,
		0b0000111111100000001111111100000001111111100000000111111110001000,
		0b0000000000011111111111111100000000000000011111111111111110010000,
		0b0000000000000000000000000011111111111111111111111111111110100000,
		0b1111111111111111111111111111111111111111111111111111111111111111 };

uint64_t parity(uint64_t of) {
	uint32_t of2 = (uint32_t) ((of >> 32) ^ of);
	of2 = (of2 >> 16) ^ of2;
	of2 = (of2 >> 8) ^ of2;
	of2 = (of2 >> 4) ^ of2;
	of2 = (of2 >> 2) ^ of2;
	of2 = (of2 >> 1) ^ of2;
	return (of2 & 0x1) == 1;
}

uint64_t encode(uint64_t data) {
	// Explicitly unrolling this because I don't trust the compiler to do it for
	// me, I think it's permanently set to debug mode.
	data |= parity(data & MASKS[0]) ? 1 << 0 : 0;
	data |= parity(data & MASKS[1]) ? 1 << 1 : 0;
	data |= parity(data & MASKS[2]) ? 1 << 2 : 0;
	data |= parity(data & MASKS[3]) ? 1 << 3 : 0;
	data |= parity(data & MASKS[4]) ? 1 << 4 : 0;
	data |= parity(data & MASKS[5]) ? 1 << 5 : 0;
	data |= parity(data & MASKS[6]) ? 1 << 6 : 0;
	return data;
}

void HammingBuffer::append_7_bytes(uint8_t *src) {
	// TODO: Make this atomic
	// https://stackoverflow.com/questions/71626597/what-are-the-various-ways-to-disable-and-re-enable-interrupts-in-stm32-microcont/71626598#71626598
	uint64_t data = 0;
	data |= ((uint64_t) *(src + 0)) << 56;
	data |= ((uint64_t) *(src + 1)) << 48;
	data |= ((uint64_t) *(src + 2)) << 40;
	data |= ((uint64_t) *(src + 3)) << 32;
	data |= ((uint64_t) *(src + 4)) << 24;
	data |= ((uint64_t) *(src + 5)) << 16;
	data |= ((uint64_t) *(src + 6)) << 8;
	// Clear last byte as it will be garbage.
	data &= ~0xFF;
	data = encode(data);
	buffer[next_write] = data;
	next_write += 1;
	read_end += 1;
	// Detect buffer overflows.
	if (next_write == read_start)
		Error_Handler();
}

void HammingBuffer::append_multiple_bytes(uint8_t *src, uint16_t len) {
	if (len % 7 != 0)
		Error_Handler();
	for (uint16_t offset = 0; offset < len; offset += 7) {
		append_7_bytes(src + offset);
	}
}

bool HammingBuffer::drain_8_bytes(uint64_t *target) {
	// TODO: Make this atomic
	// https://stackoverflow.com/questions/71626597/what-are-the-various-ways-to-disable-and-re-enable-interrupts-in-stm32-microcont/71626598#71626598
	if (read_start == read_end) {
		return false;
	} else {
		*target = buffer[read_start];
		read_start = (read_start + 1) % BUFFER_SIZE;
		return true;
	}
}
