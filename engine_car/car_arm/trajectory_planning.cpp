#include "trajectory_planning.h"




trajectory_planning_way_1 :: trajectory_planning_way_1(trajectory_point_t trajectory_points[], int num_points, int planning_way)
{

    this->trajectory_points = trajectory_points;
    this->num_points = num_points;
    this->planning_way = planning_way;


    if(planning_way == 1)
    {
        account_arguments_for_def_velocity();

    }
    else if(planning_way == 2)
    {
        account_arguments_for_ndef_velocity();
    }








}









void trajectory_planning_way_1 :: account_arguments_for_def_velocity()
{
    
    for(int i = 0; i < num_points-1; i++)
    {
        a0[i] = trajectory_points[i].point;
        a1[i] = trajectory_points[i].velocity;
        a2[i] = (3 / square(trajectory_points[i].point_to_next_time)) * (trajectory_points[i+1].point - trajectory_points[i].point) - (2 / trajectory_points[i].point_to_next_time * trajectory_points[i].velocity) - (1 / trajectory_points[i].point_to_next_time * trajectory_points[i + 1].velocity);
        a3[i] = (-2 / (square(trajectory_points[i].point_to_next_time) * trajectory_points[i].point_to_next_time)) * (trajectory_points[i + 1].point - trajectory_points[i].point) + (1 / square(trajectory_points[i].point_to_next_time) ) * (trajectory_points[i].velocity + trajectory_points[i + 1].velocity);

    }

}





void trajectory_planning_way_1 :: account_arguments_for_ndef_velocity()
{





    
}









