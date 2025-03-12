#ifndef _K230__MATH_H_
#define _K230__MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief        Compute the square root of a float data
 * @param[in]    in   input value
 * @return       out  out value
 */
static __inline __attribute__ ((__always_inline__))
float k230_sqrtf(float in)
{
    register float x = in;
    __asm volatile ("fsqrt.s %0, %0;":"+f" (x)
                    ::);
    return x;
}

/**
 * @brief        Compute the exp of a float data
 * @param[in]    in   input value
 * @return       out  out value
 */
float k230_expf(float x);

/**
 * @brief        Return the absolute value of x
 * @param[in]    in   input value
 * @return       out  out value
 */
static __inline __attribute__ ((__always_inline__))
float k230_fabsf(float in)
{
    register float x = in;
    __asm volatile ("fabs.s %0, %1;":"=f" (x)
                    :"f"(in)
                    :);
    return x;
}

/**
 * @brief        Return the smaller of two floating point values
 * @param[in]    in   input value
 * @return       out  out value
 */
static __inline __attribute__ ((__always_inline__))
float k230_fminf(float value1, float value2)
{
    register float x;
    __asm volatile ("fmin.s %0, %1, %2;":"=f" (x)
                    :"f"(value1), "f"(value2)
                    :);
    return x;
}

/**
 * @brief        Return the larger of two floating point values
 * @param[in]    in   input value
 * @return       out  out value
 */
static __inline __attribute__ ((__always_inline__))
float k230_fmaxf(float value1, float value2)
{
    register float x;
    __asm volatile ("fmax.s %0, %1, %2;":"=f" (x)
                    :"f"(value1), "f"(value2)
                    :);
    return x;
}

/**
 * @brief        Returns the largest integer value less than or equal to input value
 * @param[in]    in   input value
 * @return       out  out value
 */
/*
 * rne: 最近舍入，朝向偶数方向（round to nearest, ties to even）
 * rne: 最近舍入，朝向偶数方向（round to nearest, ties to even）
 * rtz: 朝零舍入（round towards zero）
 * rdn: 向下舍入（round down）
 * rup: 向上舍入（round up）
 * rmm: 最近舍入，朝向最大幅度方向（round to nearest, ties to max magnitude）
 * dyn: 动态舍入模式（dynamic rounding mode）
 */
static __inline __attribute__ ((__always_inline__))
float k230_floorf(float in)
{
    register float x = in;
    __asm volatile ("fcvt.w.s t0, %0, rdn;" "fcvt.s.w %0, t0;":"+f" (x)
                    ::"t0");
    return x;
}

/**
 * @brief        Returns cycles number since the CPU reset
 * @return       out cycles number
 */
static __inline __attribute__ ((__always_inline__))
uint64_t k230_get_cycles()
{
    register uint64_t x;
    __asm volatile ("rdcycle %0;":"=r" (x)
                    ::);


    return x;
}

static __inline __attribute__ ((__always_inline__))
uint64_t k230_get_time_cycles()
{
    register uint64_t x;
    // __asm volatile ("RDINSTRET %0;":"=r" (x)
                    // ::);
    __asm volatile ("RDTIME %0;":"=r" (x)::);
    return x;
}

#ifdef __cplusplus
}
#endif

#endif                          // _K230__MATH_H_
