#ifndef RTT_H
#define RTT_H

#include <stdbool.h>
#include <stddef.h>

/* Called from the main loop. Drops the entire message if the host falls behind. */
void Rtt_Write(unsigned channel, const void *data, size_t length);
bool Rtt_ReadByte(char *byte);

#endif
