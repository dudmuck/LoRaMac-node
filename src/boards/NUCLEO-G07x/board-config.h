#if defined(STM32G071xx) || defined(STM32G070xx)
    #include "board-config-nucleo64.h"
#elif defined (STM32G031xx)
    #include "board-config-nucleo32.h"
#else
    #error "board-config for MCU family"
#endif
