#ifndef ARM_UTILS_H
#define ARM_UTILS_H

#define __DSB do {        \
    asm volatile ("dsb"); \
} while(0)

#define __ISB do {        \
    asm volatile ("isb"); \
} while(0)

#define __WFI do {        \
    asm volatile ("wfi"); \
} while(0)

#define __WFE do {        \
    asm volatile ("wfe"); \
} while(0)

#endif