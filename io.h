#ifndef IO_H
#define IO_H

static inline uint32_t reg_r32(uint32_t addr)
{
    volatile uint32_t *p = (uint32_t *)addr;
    /* TODO: DSB? ISB? */
    return *p;
}

static inline uint32_t reg_r32_mask(uint32_t addr, uint32_t mask)
{
    return reg_r32(addr) & mask;
}

static inline void reg_w32(uint32_t addr, uint32_t val)
{
    volatile uint32_t *p = (uint32_t *)addr;
    *p = val;
}

static inline void reg_clear_bit(uint32_t addr, uint32_t bit)
{
    uint32_t val = reg_r32(addr);
    val &= ~bit;
    reg_w32(addr, val);
}

static inline void reg_set_bit(uint32_t addr, uint32_t bit)
{
    uint32_t val = reg_r32(addr);
    val |= bit;
    reg_w32(addr, val);
}

#endif
