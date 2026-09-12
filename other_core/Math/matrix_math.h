#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H


#include "ifr_lib.h"

#define EPS 1e-8f

/*****************************************************************************************
 * 矩阵数学库
 * 
 ****************************************************************************************/



 class matrix_math
{
public:
    // 构造：传入 行、列
    matrix_math(uint8_t a_h, uint8_t a_l);
    matrix_math() : h_count(0), l_count(0), matrix(NULL) {}


    // 拷贝构造函数（深拷贝）
    matrix_math(const matrix_math& other);

    // 重载赋值运算符（必须加！防止崩溃）
    matrix_math& operator=(const matrix_math& other);

    // 析构：释放内存
    ~matrix_math();


    // 设置矩阵值,i是行索引，j是列索引，value是值
    void set_value(uint8_t i, uint8_t j, float value);

    void set_data_arr(const float data[]);

    // 获取矩阵值
    float get_value(uint8_t i, uint8_t j) const;

    // 设置成单位矩阵
    void set_unit_matrix();




public:
    uint8_t h_count;    // 行数
    uint8_t l_count;    // 列数

protected:
    float* matrix;   // 动态数组指针
};



matrix_math DH_Matrix(float a_alpha, float a, float d, float O_theta);


// 矩阵乘法
matrix_math matrix_multiplication(const matrix_math &a, const matrix_math &b);


    // 矩阵转置
matrix_math matrix_transpose(const matrix_math &input_matrix);


matrix_math matrix_inverse(const matrix_math& A);















#endif // MATRIX_MATH_H