#include <stdint.h>

#define BUFFER_SIZE 2048

class HammingBuffer {
public:
	void append_7_bytes(uint8_t *src);
	// len must be a multiple of 7.
	void append_multiple_bytes(uint8_t *src, uint16_t len);
	// Returns true if 8 bytes were successfully written to target.
	bool drain_8_bytes(uint64_t *target);
private:
	uint64_t buffer[BUFFER_SIZE];
	uint16_t next_write = 0;
	uint16_t read_start = 0;
	uint16_t read_end = 0;
};
