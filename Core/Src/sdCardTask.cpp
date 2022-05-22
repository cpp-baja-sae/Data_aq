#include "main.h"
#include "Sd2Card.h"
#include "sdCardTask.h"
#include "cmsis_os.h"

const uint8_t staticData[512] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0 };

void sanityCheck(Sd2Card &card) {
	if (!card.init()) Error_Handler();
	if (!card.writeStart(0x8000, 0x40)) Error_Handler();
	for (int i = 0; i < 0x40; i++) {
		if (!card.writeData(staticData)) Error_Handler();
	}
	if (!card.writeStop()) Error_Handler();
}

extern "C" void sdCardTask(void *argument) {
	Sd2Card card = Sd2Card();
	sanityCheck(card);
	while (true) {
		// Wait for the interrupt to say it's our time to resume again.
		ulTaskNotifyTake(pdTRUE, 1000);
	}
//	HammingBuffer buf = HammingBuffer();
//	uint64_t value = 0;
//	buf.append_7_bytes((uint8_t*) &value);
}
