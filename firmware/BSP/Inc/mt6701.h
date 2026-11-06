#ifndef MT6701_H
#define MT6701_H

#include <stdbool.h>
#include <stdint.h>

#define MT6701_RESOLUTION 16384u

void MT6701_Init(void);
typedef enum { MT6701_BUSY, MT6701_READY, MT6701_ERROR } MT6701_ReadResult;
/* Begin one transaction, then poll without waiting until READY or ERROR. */
void MT6701_BeginRead(void);
MT6701_ReadResult MT6701_PollAngle(uint16_t *angle);
bool MT6701_ReadAngle(uint16_t *angle);

#endif
