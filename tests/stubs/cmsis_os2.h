#pragma once
#include <cstdint>
enum osPriority_t { osPriorityHigh=40 }; using osThreadId_t=void*;
struct osThreadAttr_t {const char* name; uint32_t attr_bits=0;void* cb_mem=nullptr;uint32_t cb_size=0;void* stack_mem=nullptr;uint32_t stack_size;osPriority_t priority;uint32_t tz_module=0,reserved=0;};
uint32_t osKernelGetTickCount(); int osDelayUntil(uint32_t);osThreadId_t osThreadNew(void(*)(void*),void*,const osThreadAttr_t*);void osThreadExit();

constexpr int osOK=0;
