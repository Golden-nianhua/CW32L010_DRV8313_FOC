#ifndef RTT_H
#define RTT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Called from the main loop. Drops the entire message if the host falls behind. */
void Rtt_Write(unsigned channel, const void *data, size_t length);
bool Rtt_ReadByte(char *byte);

void Rtt_AudioBegin(void);
void Rtt_AudioEnd(void);
uint32_t Rtt_AudioAvailable(void);
bool Rtt_ReadAudio(int16_t *sample, bool mulaw);
#endif
