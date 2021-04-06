#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "bits/stdc++.h"
#include <sstream>
#include <cstdlib>
#include <string>
#include <fstream>

using namespace Eigen;
using namespace std;

/*************1. 壁面数量*******************/
 int TOTAL_NUM_OF_WALL; 

/*************2. 吸盘半径*******************/
 double SUCTION_CUP_RADIUS;

/*************3. 壁面数据*******************/
// 壁面数据(unit : m)
 MatrixXd WALL_BORDER_1(4,3);
 MatrixXd WALL_BORDER_2(4,3);
 MatrixXd WALL_BORDER_3(4,3);
 MatrixXd WALL_BORDER_4(4,3);
 vector<MatrixXd> WALL_BORDER;

// 收缩后的壁面数据(unit : m)
 MatrixXd SHRINK_WALL_BORDER_1(4,3);
 MatrixXd SHRINK_WALL_BORDER_2(4,3);
 MatrixXd SHRINK_WALL_BORDER_3(4,3);
 MatrixXd SHRINK_WALL_BORDER_4(4,3);
 vector<MatrixXd> SHRINK_WALL_BORDER(4, MatrixXd::Zero(4,3)); // (3, MatrixXd::Zero(4,3))

// 全局规划时，对壁面进行法向量方向的抬升(unit : m) 
//  vector<MatrixXd> uplift_normal_vector_shirink_wall_border(3, MatrixXd::Zero(4,3));

// 壁面法向量(行向量)
 RowVector3d NORMAL_VECTOR_WALL_1, NORMAL_VECTOR_WALL_2, NORMAL_VECTOR_WALL_3, NORMAL_VECTOR_WALL_4;
 vector<RowVector3d> NORMAL_VECTOR_WALL(4, RowVector3d::Zero(1,3));//

// 壁面全局规划的起点和终点
 MatrixXd START_POINT(1,3);
 MatrixXd END_POINT(1,3);

// 壁面规划的起始壁面和终点壁面
 vector<int> START_AND_END_WALL(2, 0);  // 第一个为起始壁面序号(第一个序号为0)，第二个为终壁面


// 壁面可过渡性邻接矩阵 -----------初始化空间
 MatrixXd transition_possibility_adjacency_matrix(MatrixXd::Zero(4,4));


/*************4. 机器人的配置*******************/

 vector<double> ROBOT_CLIMBOT_5D_LENGTH(6, 0);  // 5DOF机器人的4个连杆长度，前后两个为吸附模块的直立高度(unit: m) -- 必须要初始化
 vector<int> ROBOT_JOINT_MIN_ANGLE(5, 0);       // 5DOF机器人的5个关节的最小限角 (unit : deg)
 vector<int> ROBOT_JOINT_MAX_ANGLE(5, 0);       // 5DOF机器人的5个关节的最大限角 (unit : deg)

/*************5. 障碍物的数据*******************/
 int OBSTACLE_NUM;                        // 障碍物的数量
 int OBSTACLE_PLACE_WALL_1;               // 障碍物1所在的壁面
 MatrixXd OBSTACLE_BORDER_1(4, 2);        // 障碍物1在壁面上的位置
 pair<int, MatrixXd> OBSTACLE_1(0, MatrixXd::Zero(4,2));          // 障碍物1的壁面和位置的pair(first为壁面, second 为壁面位置)



//*1/ 内存访问越界的问题
//  - 以vector 为例：
//      - 当对于vector<>，最开始的操作就是push_back的时候，无需定义内存空间大小，因为它会自动扩容
//      - 而当还没定义空间大小初始化的时候，就去访问(如读取数值或写入数值)，就会造成无法找到内存的错误。需要提前对vector进行初始化内存空间大小。