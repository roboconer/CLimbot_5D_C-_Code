
#ifndef _GLOBAL_PLANNING_H
#define _GLOBAL_PLANNING_H

#include "../../config/global_variable.h"


// function-1 : 可过渡性分析[消除姿态可达约束 + 位置可达约束判断]
// INPUT:  vector<RowVector3d> NORMAL_VECTOR_WALL: 壁面法向量; vector<MatrixXd> SHRINK_WALL_BORDER: 收缩后的壁面数据
// OUTPUT: 无
// 将可过渡性分析得到的邻接矩阵存入全局规划变量: MatrixXd transition_possibility_adjacency_matrix
void transition_possibility_analysis(vector<RowVector3d>& NORMAL_VECTOR_WALL, vector<MatrixXd>& SHRINK_WALL_BORDER);

// function-2 : 消除姿态可达约束[往壁面法向量抬升]
// INPUT:  vector<RowVector3d> NORMAL_VECTOR_WALL: 壁面法向量; vector<MatrixXd> SHRINK_WALL_BORDER: 收缩后的壁面数据
// OUTPUT: 无
void uplift_normal_vector_shirink_wall_fun(vector<MatrixXd>& uplift_normal_vector_shirink_wall_border, vector<RowVector3d>& NORMAL_VECTOR_WALL, vector<MatrixXd>& SHRINK_WALL_BORDER);

// function-3 : 对壁面进行位置约束判断[递归遍历每两个壁面，有最近点对建立的机器人工作空间满足时，两个壁面可过渡，邻接矩阵单元为最近的距离；不可过渡时，单元值为INT_MAX]
// INPUT: int start: 起始的壁面序号(0～(N-1)); vector<MatrixXd> uplift_normal_vector_shirink_wall_border: 消除姿态可达约束后的壁面数据
// OUTPUT: 无
void transition_possibility_adjacency_matrix_fun(int start, vector<MatrixXd>& uplift_normal_vector_shirink_wall_border);

// function-4 : 可达工作空间判断[当两最近点小于2*R_l2,则说明位置可达]
// INPUT:  pair<double, Eigen::MatrixXd> line_line_distance: 两线段的最近点对及最近距离;
// OUTPUT: bool == true: 位置可达;  bool == false: 位置不可达
bool judgment_of_robot_reachable_workspace(pair<double, Eigen::MatrixXd>& line_line_distance);

// function-5 : 对邻接矩阵进行DFS搜索，获得可能的路径壁面序列
// INPUT:  int* visited: 一个存储访问过的节点数组
//         MatrixXd transition_possibility_adjacency_matrix: 邻接矩阵;
//         int start: 起始壁面序号(0～(N-1)); int end: 终点壁面序号(0～(N-1));
//         vector<int> a_pathway: 存储的其中可能的路径; vector<vector<int>> all_pathway: 存储全部的可能路径
// OUTPUT: 无
void WALL_SEARCH_DFS(int* visited, Eigen::MatrixXd &transition_possibility_adjacency_matrix,int start,int end,vector<int> a_pathway,vector<vector<int>> &all_pathway);



#endif



