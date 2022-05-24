#include "main.h"
#include "Sd2Card.h"
#include "sdCardTask.h"
#include "hammingBuffer.hpp"
#include "cmsis_os.h"

const uint8_t staticData[512] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0 };
HammingBuffer hammingBuffer;
uint8_t outputBuffer[4096] = { 0 };
uint16_t nextOutputBufferIndex = 0;
uint32_t nextSdCardAddress = 0x8000;
static bool initialized = false;

extern "C" void appendDataFrame(void *data, uint16_t len) {
	if (initialized) {
		hammingBuffer.append_multiple_bytes((uint8_t*) data, len);
	}
}

void sanityCheck(Sd2Card &card) {
	if (!card.writeStart(0x8000, 0x40))
		Error_Handler();
	for (int i = 0; i < 0x40; i++) {
		if (!card.writeData(staticData))
			Error_Handler();
	}
	if (!card.writeStop())
		Error_Handler();
	uint8_t buffer[512] = { 1 };
	for (int i = 0; i < 0x40; i++) {
		if (!card.readBlock(0x8000 + i, &buffer[0])) {
			auto cause = card.errorCode();
			auto cause2 = card.errorData();
			Error_Handler();
		}
		for (int b = 0; b < 512; b++) {
			if (buffer[b] != staticData[b]) {
				Error_Handler();
			}
		}
	}
}

extern "C" void sdCardTask(void *argument) {
//	hammingBuffer = HammingBuffer();
	Sd2Card card = Sd2Card();
	if (!card.init()) {
		auto cause = card.errorCode();
		auto causeData = card.errorData();
		Error_Handler();
	}
	sanityCheck(card);
	initialized = true;
	while (true) {
		// Wait for the interrupt to say it's our time to resume again.
		ulTaskNotifyTake(pdTRUE, 1000);
//		bool ready = false;
//		while (hammingBuffer.drain_8_bytes(
//				(uint64_t*) &outputBuffer[nextOutputBufferIndex])) {
//			nextOutputBufferIndex = (nextOutputBufferIndex + 8) % 4096;
//			ready |= nextOutputBufferIndex == 0;
//		}
//		if (ready) {
//			if (!card.writeStart(nextSdCardAddress, 4096 / 512))
//				Error_Handler();
//			nextSdCardAddress += 4096 / 512;
//			for (int i = 0; i < 4096; i += 512) {
//				if (!card.writeData(&outputBuffer[i]))
//					Error_Handler();
//			}
//			if (!card.writeStop())
//				Error_Handler();
//		}
	}
}
