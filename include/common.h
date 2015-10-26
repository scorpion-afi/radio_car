#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "queue.h"

#define malloc pvPortMalloc
#define free   vPortFree

extern void hardware_fail( void );

#endif // COMMON_H
