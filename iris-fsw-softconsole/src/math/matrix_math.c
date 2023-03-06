#include "math/matrix_math.h"

void matmul(int m, int n, int p, float A[m][n], float B[n][p], float C[m][p])
{
    int i,j,k;
    for(i=0; i < m; i++){
        for(j=0; j < p; j++){
            C[i][j] = 0;
            for(k=0; k < n; k++){
                C[i][j] += A[i][k]*B[k][j];
            }
        }
    }
}

void dot_prod(float *a, float *b, float *c)
{
    *c = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void cross_prod(float *a, float *b, float *c)
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

void transpose(int m, int n, float A[][n], float A_T[][m])
{
    int i,j;
    for(i=0; i < m; i++){
        for(j=0; j < n; j++){
            A_T[j][i] = A[i][j];
        }
    }
}


void mat_divide(int m, int n, float A[][n], float div)
{
    int i,j;
    for(i=0; i < m; i++){
        for(j=0; j < n; j++){
            A[i][j] /= div;
        }
    }
}
