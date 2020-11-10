
#pragma once

#include "app_config.h"
#include "compiler.h"
#include "types.h"
#include "bit.h"
#if CHIP_TYPE == MCU_CORE_8266
#include "register_8266.h"
#include "gpio_default_8266.h"
#include "gpio.h"
#elif CHIP_TYPE == MCU_CORE_8267
#include "register_8267.h"
#include "gpio_default_8267.h"
#include "gpio.h"
#else
#include "register_8258.h"
#include "gpio_8258.h"
//#include "gpio_default_8258.h"
#endif


#define USE_EXT_FLASH 0
#define CS_EXT_FLASH GPIO_PB5



