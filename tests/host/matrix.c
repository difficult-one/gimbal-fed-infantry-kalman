/* 仅为主机运行提供浮点矩阵后端；被测 KF/底盘控制代码使用真实生产源码。
 * ARM 固件仍链接工程自带的 CMSIS DSP 静态库。 */
#include "arm_math.h"
#include <math.h>
void arm_mat_init_f32(arm_matrix_instance_f32 *m, uint16_t r, uint16_t c, float *p)
{ m->numRows = r; m->numCols = c; m->pData = p; }
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *o)
{
    if (a->numCols != b->numRows || o->numRows != a->numRows || o->numCols != b->numCols) return ARM_MATH_SIZE_MISMATCH;
    for (unsigned r = 0; r < o->numRows; ++r)
        for (unsigned c = 0; c < o->numCols; ++c)
        {
            float v = 0;
            for (unsigned k = 0; k < a->numCols; ++k) v += a->pData[r*a->numCols+k] * b->pData[k*b->numCols+c];
            o->pData[r*o->numCols+c] = v;
        }
    return ARM_MATH_SUCCESS;
}
static arm_status Add(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *o, float sign)
{
    if (a->numRows != b->numRows || a->numCols != b->numCols || a->numRows != o->numRows || a->numCols != o->numCols) return ARM_MATH_SIZE_MISMATCH;
    for (unsigned i = 0; i < a->numRows*a->numCols; ++i) o->pData[i] = a->pData[i] + sign*b->pData[i];
    return ARM_MATH_SUCCESS;
}
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *o) { return Add(a,b,o,1); }
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *o) { return Add(a,b,o,-1); }
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *o)
{
    if (a->numRows != o->numCols || a->numCols != o->numRows) return ARM_MATH_SIZE_MISMATCH;
    for (unsigned r=0;r<a->numRows;++r) for(unsigned c=0;c<a->numCols;++c) o->pData[c*o->numCols+r]=a->pData[r*a->numCols+c];
    return ARM_MATH_SUCCESS;
}
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *o)
{
    unsigned n=a->numRows;
    if (n>4 || n!=a->numCols || n!=o->numRows || n!=o->numCols) return ARM_MATH_SIZE_MISMATCH;
    float t[4][8]={{0}};
    for(unsigned r=0;r<n;++r) { for(unsigned c=0;c<n;++c) t[r][c]=a->pData[r*n+c]; t[r][r+n]=1; }
    for(unsigned c=0;c<n;++c)
    {
        unsigned p=c;
        for(unsigned r=c+1;r<n;++r) if(fabsf(t[r][c])>fabsf(t[p][c])) p=r;
        if(fabsf(t[p][c])<1e-20f) return ARM_MATH_SINGULAR;
        for(unsigned k=0;k<2*n;++k) { float v=t[c][k]; t[c][k]=t[p][k]; t[p][k]=v; }
        float d=t[c][c]; for(unsigned k=0;k<2*n;++k) t[c][k]/=d;
        for(unsigned r=0;r<n;++r) if(r!=c) { d=t[r][c]; for(unsigned k=0;k<2*n;++k) t[r][k]-=d*t[c][k]; }
    }
    for(unsigned r=0;r<n;++r) for(unsigned c=0;c<n;++c) o->pData[r*n+c]=t[r][c+n];
    return ARM_MATH_SUCCESS;
}
