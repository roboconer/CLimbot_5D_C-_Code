#ifndef _DISTANCE_COMPUTE_H
#define _DISTANCE_COMPUTE_H

#include "../../config/global_variable.h"


// function_1 : 线段与面求距
// INPUT：  MatrixXd line(2,3)--两行为线段的两个端点, MatrixXd surface(n, 3) -- n行是面的n个端点
// OUTPUT： 返回最短距离与相应的最近点对, 矩阵的第一行为线段上的点，第二行为面上的点
pair<double, Eigen::MatrixXd> line_to_surface_distance(Eigen::MatrixXd& line, Eigen::MatrixXd& surface);

// function-2 : 点与面的求距
// INPUT：  RowVector3d point(1,3) -- 测距的点， MatrixXd surface(n, 3) -- n行是面的n个端点
// OUTPUT： 返回最短距离与相应的面上最近点
pair<double, Eigen::RowVector3d> point_to_surface_distance(Eigen::RowVector3d& point, MatrixXd& surface);

// function-3 : 线段与线段的求距
// INPUT:  MatrixXd line1(2,3) -- 线段1，两行 为两个端点, MatrixXd line2(2,3) -- 线段2，两行 为两个端点
// OUTPUT: 返回最短距离与相应的最近点对，矩阵的第一行为线段1上的点，第二行为线段2上的点
// 参考：[空间中2条线段的最短距离](https://download.csdn.net/download/liying426/5058179?utm_medium=distribute.pc_relevant_download.none-task-download-baidujs-2.nonecase&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-baidujs-2.nonecase)
// 注意上面文档中，关于两线段平行的最短距离是错的. 线段平行时，最短距离点对其中一个一定是4个端点的其中一个，所以遍历4个端点到另一线段的最短距离就行
pair<double, Eigen::MatrixXd> line_to_line_distance(MatrixXd& line1, MatrixXd& line2);

// function-4 : 点与线段的求距
// INPUT:  RowVector3d point(1,3) --测距点, MatrixXd line(2,3) -- 两行为线段的两端点
// OUTPUT: 返回最短距离与线段的最近点
pair<double, Eigen::RowVector3d> point_to_line_distance(Eigen::RowVector3d& point, MatrixXd& line);

// function-5 : 点在面上的投影点
// INPUT:  RowVector3d point(1,3) -- 测试的点， MatrixXd surface(n, 3) -- n行是面的n个端点 
// OUTPUT: 返回点在平面上的投影点是否在面内(true -> 在面内；false -> 在面外) , 以及投影点
pair<bool, RowVector3d> mapping_point_on_surface(RowVector3d& point, MatrixXd& surface);

// function-6 : 点是否在面内
// INPUT:  RowVector3d point(1,3) -- 测试的点， MatrixXd surface(n, 3) -- n行是面的n个端点 
// OUTPUT: 返回点是否在面内(true -> 在面内；false -> 在面外)
bool point_whether_in_surface(RowVector3d& point, MatrixXd& surface);

// function-7 : 将长方体分解为5个面和12条边
// INPUT:  MatrixXd obstacle(8,3) -- 长方体的8个端点 
// OUTPUT: 返回 MatrixXd obstacle(5,12) -- 5个面，每行连续放着4*3 [4个点]
MatrixXd tran_obstacle_to_five_surface(MatrixXd& obstacle);
#endif