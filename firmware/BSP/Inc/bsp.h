#ifndef BSP_H
#define BSP_H

#include <stdbool.h>
#include <stdint.h>

void BSP_Init(void);
void BSP_UpdateSupply(void);
uint16_t BSP_BusVoltage(void);
bool BSP_StopRequested(void);
bool BSP_StopPressed(void);

#endif
