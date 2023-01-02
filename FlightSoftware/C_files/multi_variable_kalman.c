/*
Task:
    - Recreate 06-Multivariable-Kalman-Filter from Kalman and Bayesian Filters in Python
    - Use 6dof sim to simulate physics, and kalman_filter.c to filter measurment data from python
Notes:
    Currently, going to statically allocate a known size matrix (row-column form starting at 0) rather than
    generalizing matric allocation using calloc().

    *Functions assume the lengths and formats as listed bellow* 
*/

#include <stdio.h>

// Macro Functions
#define rowcol2matrix(i,j,num_col) i * num_col + j

/*
Macro vabriables

In this example 
    x = 2x1 matrix (double[2]) [x] 
    u = NULL
    P = 2x2 matrix (double[4])
    F = 2x2 matrix (double[4])
    B = NULL
    H = 1x2 matrix (double[2])
    R = 1x0 matrix (double[1])
    Q = 2x2 matrix (double[4])
*/

//x * ADD ROWS*
#define x_num_col   1
#define x_arrlen    2

//P
#define P_num_col   2
#define P_arrlength 4

//F
#define F_num_col   2
#define F_arrlen    4

//H
#define H_num_col   2
#define H_arrlength 1

//R
#define R_num_col   1
#define R_arrlen    1

//Q
#define Q_num_col   2
#define Q_arrlen    4

typedef unsigned int u_int;

void matprint(u_int num_row, u_int num_col, double matrix[num_row*num_col]){
    u_int i,j;
    printf("-\n");
    for (i = 0; i < num_row; i++){
        printf("[");
        for (j = 0; j < num_col; j++){
            printf(" %lf ",matrix[rowcol2matrix(i,j,num_col)]);
        }
        printf("]\n");
    }
    printf("-\n");
}

void mattranspose(u_int num_row, u_int num_col, double matrix[num_row*num_col], 
                  double result[num_row*num_col]){
    /*
    Returns :
        result (double[num_row*num_col])   = The transpose of the input matrix 
    Inputs:
        num_row (u_int)                    = numver of rows in "matrix"
        num_col (u_int)                    = number of collumns in "matrix"
        matrix  (double[num_rows*num_col]) = Input matrix
    */
    u_int i,j; 
     for (i = 0; i < num_row; i++){
        for (j = 0; j < num_col; j++){
            result[rowcol2matrix(j,i,num_col)] = matrix[rowcol2matrix(i,j,num_col)];
        }
    }
}

void matmul(u_int num_row1, u_int num_col1, double matrix1[num_row1*num_col1], 
            u_int num_row2, u_int num_col2, double matrix2[num_row2*num_col2],
            double result[num_row1*num_col2]){
    u_int i,j;
}

/*
===========
Prediction Functions

Notes:
    - In this example all matrixs are small enough to not require rowcol2matrix in alot of cases
    - Chose not to generalize functions until I can figure out malloc workaround (this ultimiatly needs)
      to run on a RTS system (microcontroller with a state machine)
===========
*/

void km_predict(double x_est[x_arrlen], double F[F_arrlen],double Q[Q_arrlen]){
    /*
    Returns:
        x_pred (double[2]) = Kalman prediction 
    */
    double x_pred[x_arrlen] = {(F[0] * x_pred[0] + F[2]*x_pred[0]),(F[1] * x_pred[1] + F[3]*x_pred[1])};
    matprint((x_arrlen/x_num_col),x_num_col,x_pred);
    /*
    new_P = FPF^T + Q
    */
    double TF[F_arrlen];
    mattranspose(F_arrlen/F_num_col,F_num_col,F,TF);
    matprint((F_arrlen/F_num_col),F_num_col,TF);
}


int main(){
    double test_x[x_arrlen] = {1,
                               1};

    double test_F[F_arrlen] = {1,2,
                               3,4};

    double test_Q[Q_arrlen] = {0,0,
                               0,0};

    km_predict(test_x,test_F,test_Q);
}