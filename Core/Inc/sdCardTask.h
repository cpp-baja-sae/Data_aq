#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// len must be a multiple of 7.
void appendDataFrame(void *data, uint16_t len);
void sdCardTask(void*);

#ifdef __cplusplus
}
#endif
