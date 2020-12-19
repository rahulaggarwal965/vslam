#ifndef __VSLAM_INTERNAL_H__
#define __VSLAM_INTERNAL_H__

#include <cstddef>
#include <stdint.h>
#include <limits.h>
#include <float.h>

typedef int8_t  s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef size_t memory_index; //use for going into memory


#define u32_max ((u32) - 1)

#define f32_maximum FLT_MAX

typedef float  f32;
typedef double f64;

/* #define internal static */
#define internal_function static
#define local_persist static
#define global_variable static

#endif
