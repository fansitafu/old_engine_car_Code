#include "matrix_math.h"



matrix_math :: matrix_math(uint8_t a_h, uint8_t a_l) : h_count(a_h), l_count(a_l)
{
    // 动态开辟一维数组（模拟二维矩阵）
    matrix = new float[h_count * l_count]();

}


matrix_math :: matrix_math(const matrix_math& other): h_count(other.h_count), l_count(other.l_count)
{
    // 开辟新内存
    matrix = new float[h_count * l_count];
    
    // 把数据完整拷贝过来（不是只拷贝指针）
    for(int i=0; i<h_count*l_count; i++){
        matrix[i] = other.matrix[i];
    }
}




matrix_math& matrix_math::operator=(const matrix_math& other)
{
		//防止自己给自己赋值 A = A,作用：避免自己删自己内存，导致崩溃。
    if (this == &other)
        return *this;

    // 释放旧内存
    if (matrix != NULL)
    {
        delete[] matrix;
        matrix = NULL;
    }

    h_count = other.h_count;
    l_count = other.l_count;

    if (h_count == 0 || l_count == 0)
    {
        matrix = NULL;
        return *this;
    }

    matrix = new float[h_count * l_count];

    for (int i = 0; i < h_count * l_count; i++)
    {
        matrix[i] = other.matrix[i];
    }

    return *this;		//不写 return *this 编译能过,单个赋值（B=A）正常,连续赋值（A=B=C）直接报错
}




matrix_math :: ~matrix_math()
{
    if (matrix != NULL)
    {
        delete[] matrix;
        matrix = NULL;
    }
}

/*
 * @功能        : 设置矩阵值
 * @参数1       : i：行索引
 * @参数2       : j：列索引
 * @参数3       : value：要设置的值
 * @返回值      : 无
 * @概述        : 根据输入的索引和值设置矩阵中的元素
 */
void matrix_math :: set_value(uint8_t i, uint8_t j, float value)
{
    matrix[i * l_count + j] = value;
}

void matrix_math :: set_data_arr(const float data[])
{

    for (uint8_t i = 0; i < h_count * l_count; i++)
    {
        matrix[i] = data[i];
    }
}

void matrix_math :: set_unit_matrix()
{
    if (h_count != l_count)
    {
        // 不是方阵，无法设置为单位矩阵
        return;
    }
    for (uint8_t i = 0; i < h_count; i++)
    {
        for (uint8_t j = 0; j < l_count; j++)
        {
            if (i == j)
            {
                matrix[i * l_count + j] = 1.0f;
            }
            else
            {
                matrix[i * l_count + j] = 0.0f;
            }
        }
    }
}

// 获取矩阵值
float matrix_math :: get_value(uint8_t i, uint8_t j) const
{
    return matrix[i * l_count + j];
}



float T_watch2[4][4];

matrix_math matrix_multiplication(const matrix_math &a, const matrix_math &b)
{
 
//					for (uint8_t i = 0; i < 4; ++i)
//					{
//							for (uint8_t j = 0; j < 4; ++j)
//							{
//									T_watch2[i][j] = a.get_value(i, j);
//							}
//					}

    if (a.l_count != b.h_count)
    {
        // 维度不匹配，无法相乘
        return matrix_math(1, 1);
    }
    matrix_math result(a.h_count, b.l_count);
    for (uint8_t i = 0; i < a.h_count; ++i)
    {
        for (uint8_t j = 0; j < b.l_count; ++j)
        {
            float sum = 0.0f;
            for (uint8_t k = 0; k < a.l_count; ++k)
            {
                sum += a.get_value(i, k) * b.get_value(k, j);
            }
            result.set_value(i, j, sum);
        }
    }


    return result;
}





matrix_math matrix_transpose(const matrix_math &input_matrix)
{
    matrix_math result(input_matrix.l_count, input_matrix.h_count); // 转置后行列数交换
    for (uint8_t i = 0; i < input_matrix.h_count; ++i)
    {
        for (uint8_t j = 0; j < input_matrix.l_count; ++j)
        {
            result.set_value(j, i, input_matrix.get_value(i, j)); // 转置：行列交换
        }
    }
    return result;
}





/**
 * @功能        : DH矩阵计算函数
 * @参数1       : alpha：连杆扭转角（单位：弧度） 
 * @参数2       : a：连杆长度（单位：mm）
 * @参数3       : d：连杆偏距（单位：mm）
 * @参数4       : theta：关节角度（单位：弧度）
 * @返回值      : 4x4的DH变换矩阵
 * @概述        : 根据输入的DH参数计算对应的变换矩阵，适用于机械臂运动学分析
 */
matrix_math DH_Matrix(float a_alpha, float a, float d, float O_theta)
{
    
    matrix_math result(4, 4);
    result.set_value(0, 0, cos(O_theta));
    result.set_value(0, 1, -sin(O_theta));
    result.set_value(0, 2, 0);
    result.set_value(0, 3, a);
    result.set_value(1, 0, sin(O_theta) * cos(a_alpha));
    result.set_value(1, 1, cos(O_theta) * cos(a_alpha));
    result.set_value(1, 2, -sin(a_alpha));
    result.set_value(1, 3, -sin(a_alpha) * d);
    result.set_value(2, 0, sin(O_theta) * sin(a_alpha));
    result.set_value(2, 1, cos(O_theta) * sin(a_alpha));
    result.set_value(2, 2, cos(a_alpha));
    result.set_value(2, 3, cos(a_alpha) * d);
    result.set_value(3, 0, 0);
    result.set_value(3, 1, 0);
    result.set_value(3, 2, 0);
    result.set_value(3, 3, 1);
    return result;

}


//一定是先绕z轴转，再绕y轴转，最后绕x轴转





// 独立函数：传入 matrix_math 矩阵，返回逆矩阵
matrix_math matrix_inverse(const matrix_math& A)
{
    // 1. 必须是方阵
    if (A.h_count != A.l_count)
    {
        return matrix_math(); // 非方阵返回空矩阵
    }

    int n = A.h_count;

    // 2. 创建增广矩阵 [A | E]
    float aug[16][32] = {0};
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            aug[i][j] = A.get_value(i, j);
        }
        aug[i][n + i] = 1.0f;
    }

    // 3. 高斯-约旦消元
    for (int col = 0; col < n; col++)
    {
        // 选主元
        int pivot = col;
        for (int row = col; row < n; row++)
        {
            if (fabs(aug[row][col]) > fabs(aug[pivot][col]))
            {
                pivot = row;
            }
        }

        // 交换主元行
        for (int c = col; c < 2 * n; c++)
        {
            float temp = aug[col][c];
            aug[col][c] = aug[pivot][c];
            aug[pivot][c] = temp;
        }

        // 主元太小 → 奇异矩阵，无逆
        if (fabs(aug[col][col]) < EPS)
        {
            return matrix_math();
        }

        // 主元归一化
        float div = aug[col][col];
        for (int c = col; c < 2 * n; c++)
        {
            aug[col][c] /= div;
        }

        // 消去其他行
        for (int row = 0; row < n; row++)
        {
            if (row != col && fabs(aug[row][col]) > EPS)
            {
                float factor = aug[row][col];
                for (int c = col; c < 2 * n; c++)
                {
                    aug[row][c] -= factor * aug[col][c];
                }
            }
        }
    }

    // 4. 提取逆矩阵
    matrix_math inv(n, n);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            inv.set_value(i, j, aug[i][n + j]);
        }
    }

    return inv;
}







