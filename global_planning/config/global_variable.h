
#ifndef _GLOBAL_VARIABLE_H
#define _GLOBAL_VARIABLE_H

//#include "../config/read_config.h"

#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include <algorithm>
using namespace Eigen;
using namespace std;



/*************1. 壁面数量*******************/
extern int TOTAL_NUM_OF_WALL; 

/*************2. 吸盘半径*******************/
extern double SUCTION_CUP_RADIUS;

/*************3. 壁面数据*******************/
// 壁面数据(unit : m)
extern MatrixXd WALL_BORDER_1;
extern MatrixXd WALL_BORDER_2;
extern MatrixXd WALL_BORDER_3;
extern MatrixXd WALL_BORDER_4;
extern vector<MatrixXd> WALL_BORDER; 

// 收缩后的壁面数据(unit : m)
extern MatrixXd SHRINK_WALL_BORDER_1;
extern MatrixXd SHRINK_WALL_BORDER_2;
extern MatrixXd SHRINK_WALL_BORDER_3;
extern MatrixXd SHRINK_WALL_BORDER_4;
extern vector<MatrixXd> SHRINK_WALL_BORDER;

// 全局规划时，对壁面进行法向量方向的抬升(unit : m) 
// extern vector<MatrixXd> uplift_normal_vector_shirink_wall_border;

// 壁面法向量(行向量)
extern RowVector3d NORMAL_VECTOR_WALL_1, NORMAL_VECTOR_WALL_2, NORMAL_VECTOR_WALL_3, NORMAL_VECTOR_WALL_4;
extern vector<RowVector3d> NORMAL_VECTOR_WALL;

// 壁面全局规划的起点和终点
extern MatrixXd START_POINT;
extern MatrixXd END_POINT;

// 壁面规划的起始壁面和终点壁面
extern vector<int> START_AND_END_WALL;  // 第一个为起始壁面序号(第一个序号为1)，第二个为终壁面


// 壁面可过渡性邻接矩阵
extern MatrixXd transition_possibility_adjacency_matrix; 


/*************4. 机器人的配置*******************/

extern vector<double> ROBOT_CLIMBOT_5D_LENGTH;  // 5DOF机器人的4个连杆长度(unit: m) -- 必须要初始化
extern vector<int> ROBOT_JOINT_MIN_ANGLE;       // 5DOF机器人的5个关节的最小限角 (unit : deg)
extern vector<int> ROBOT_JOINT_MAX_ANGLE;       // 5DOF机器人的5个关节的最大限角 (unit : deg)

/*************5. 障碍物的数据*******************/
extern int OBSTACLE_NUM;                        // 障碍物的数量
extern int OBSTACLE_PLACE_WALL_1;               // 障碍物1所在的壁面
extern MatrixXd OBSTACLE_BORDER_1;        // 障碍物1在壁面上的位置
extern pair<int, MatrixXd> OBSTACLE_1;          // 障碍物1的壁面和位置的pair(first为壁面, second 为壁面位置)


#endif                              


