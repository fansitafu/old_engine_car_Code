#ifndef TRAJECTORY_PLANNING_H
#define TRAJECTORY_PLANNING_H



#define planning_way_def_velocity 1
#define planning_way_ndef_velocity 2

#include "Math_calculate.h"

typedef struct
{

    float point;     // 位置
    float velocity;  // 速度
    float point_to_next_time;      // 到下一个点的时间

} trajectory_point_t;












class trajectory_planning_way_1
{
    

public:
    trajectory_planning_way_1(trajectory_point_t trajectory_points[], int num_points, int planning_way); // 构造函数，接受轨迹点数组和点的数量
    trajectory_planning_way_1(float initial_velocity,  float starting_point, float ending_point, float continue_time);
    ~trajectory_planning_way_1(){};


    void account_arguments_for_def_velocity();
    void account_arguments_for_ndef_velocity();





protected:
    trajectory_point_t *trajectory_points;
    int num_points;
    int planning_way;
    float a0[100];
    float a1[100];
    float a2[100];
    float a3[100];







};















































#endif // TRAJECTORY_PLANNING_H