#ifndef BSP_H
#define BSP_H

#include <stdbool.h>

void BSP_Init(void);
bool BSP_StopRequested(void);
bool BSP_StopPressed(void);

#endif
