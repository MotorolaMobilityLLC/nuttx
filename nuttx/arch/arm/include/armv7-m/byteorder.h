/**
 * Copyright (c) 2015 Google, Inc.
 * @author: Perry Hung
 */

#ifndef  _BYTEORDER_H_
#define  _BYTEORDER_H_

#ifdef CONFIG_ENDIAN_BIG
#error "big-endian unsupported"
#endif

static inline uint32_t __swap32(uint32_t value) {
    uint32_t result;
    asm volatile("rev %[result], %[value]" :
                 [result] "=r" (result) :
                 [value] "r" (value));
    return result;
}

static inline uint16_t __swap16(uint16_t value) {
    uint16_t result;
    asm volatile("rev16 %[result], %[value]" :
                 [result] "=r" (result) :
                 [value] "r" (value));
    return result;
};

#define be32_to_cpu(v) __swap32(v)
#define cpu_to_be32(v) __swap32(v)
#define be16_to_cpu(v) __swap16(v)
#define cpu_to_be16(v) __swap16(v)
#define le32_to_cpu(v) (v)
#define cpu_to_le32(v) (v)
#define le16_to_cpu(v) (uint16_t)(v)
#define cpu_to_le16(v) (uint16_t)(v)

#endif


