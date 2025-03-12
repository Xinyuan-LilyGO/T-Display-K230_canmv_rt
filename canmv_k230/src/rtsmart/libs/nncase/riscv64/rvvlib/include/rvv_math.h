#ifndef _K230__RVV_MATH__H_
#define _K230__RVV_MATH__H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief        Calculate the vector-scalar product and add to the result
 *                    y = a*x + y
 * @param[in]    n   element number of vector x and y
 * @param[in]    a   scalar a
 * @param[in]    x   vector x
 * @param[in]    y   vector y
 * @return       NULL
 */
void saxpy_vec(size_t n, const float a, const float *x, float *y);
void saxpy_golden(size_t n, const float a, const float *x, float *y);


/**
 * @brief        Computes a matrix-matrix product with matries and add the result to another matrix
 *                    c += a*b
 * @param[in]    m     Specifies the number of rows of the matrix a and  of the matrix
 * @param[in]    n     Specifies the number of columns of the matrix b and the number of columns of the matrix c
 * @param[in]    k     the number of columns of the matrix a and the number of rows of the matrix b 
 * @param[in]    a     marix a
 * @param[in]    lda   the leading dimension of a as declared in the calling (sub)program
 * @param[in]    b     marix b
 * @param[in]    ldb   the leading dimension of b as declared in the calling (sub)program
 * @param[in]    c     marix c
 * @param[in]    ldc   the leading dimension of b as declared in the calling (sub)program
 * @return       NULL
 */
void sgemm_vec(size_t size_m, size_t size_n, size_t size_k, const float *a, size_t lda, // m * k matrix
               const float *b, size_t ldb,  // k * n matrix
               float *c, size_t ldc // m * n matrix
    );
void sgemm_golden(size_t size_m, size_t size_n, size_t size_k, const float *a, size_t lda,  // m * k matrix
                  const float *b, size_t ldb,   // k * n matrix
                  float *c, size_t ldc  // m * n matrix
    );


/**
 * @brief        Calculate the expf of vector x
 *                    y = expf(x)
 * @param[in]    n   element number of vector x and y
 * @param[in]    x   vector x
 * @param[out]   y   vector y
 * @return       NULL
 */
void expf_vec(size_t n, const float *x, float *y);
void expf_golden(size_t n, const float *x, float *y);


/**
 * @brief        Calculate the max value index of  vector x
 * @param[in]    n   element number of vector x
 * @param[in]    x   vector x
 * @return       The max value index
 */
size_t maxindex_vec(size_t n, float* x);
size_t maxindex_golden(size_t n, float* x);


/**
 * @brief        Calculate the nozero data mean of vector x
 * @param[in]    n   element number of vector in
 * @param[in]    x   vector x
 * @return       The nozero data mean of vector x
 */
int nozero_mean_vec(int n, int* x);
int nozero_mean_golden(int n, int* x);


/**
 * @brief        normalize
 */
void normalize_vec(int n, float* x, float* y);
void normalize_golden(int n, float* x, float* y);


/**
 * @brief        cosin distance
 */
float cosin_distance_vec(int n, float* x, float* y);
float cosin_distance_golden(int n, float* x, float* y);


/**
 * @brief        Calculate the softmax of vector x
 * @param[in]    n   element number of vector x
 * @param[in]    x   vector x
 * @param[out]   y   vector y
 * @return       NULL
 */
void softmax_vec(int n, float* x, float* y);
void softmax_golden(int n, float* x, float* y);


void softmax2_vec(int n, float* x, float* y);
void softmax2_golden(int n, float* x, float* y);

/**
 * @brief        Calculate the sigmoid of vector x
 * @param[in]    n   element number of vector x
 * @param[in]    x   vector x
 * @param[out]   y   vector y
 * @return       NULL
 */
void sigmoid_vec(int n, float* x, float* y);
void sigmoid_golden(int n, float* x, float* y);

void softmax_2group_golden(int n, float* x1, float* x2, float* dx1, float* dx2);
void softmax_2group_vec(int n, float* x1, float* x2, float* dx1, float* dx2);

#ifdef __cplusplus
}
#endif

#endif                          // _K230__RVV_MATH__H_
