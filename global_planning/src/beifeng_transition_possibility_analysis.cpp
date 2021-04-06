//#include "../config/read_config.h"
#include "global_planning.h"
#include "../config/global_variable.h"
#include "../config/read_config.h"
#include "distance_compute/distance_compute.h"


void transition_possibility_analysis(vector<RowVector3d>& NORMAL_VECTOR_WALL, vector<MatrixXd>& SHRINK_WALL_BORDER);
void uplift_normal_vector_shrink_wall_fun(vector<MatrixXd>& uplift_normal_vector_shrink_wall_border, vector<RowVector3d>& NORMAL_VECTOR_WALL, vector<MatrixXd>& SHRINK_WALL_BORDER);
void transition_possibility_adjacency_matrix_fun(int start, vector<MatrixXd>& uplift_normal_vector_wall_border);
bool judgment_of_robot_reachable_workspace(Eigen::MatrixXd& tested_wall, Eigen::RowVector3d& line_line_nearest_point, Eigen::RowVector3d& two_wall_normal_vector_cross);

pair<bool, Eigen::RowVector3d> line_to_surface(Eigen::RowVector3d& line_nearest_point, Eigen::MatrixXd& line, Eigen::RowVector3d& two_wall_normal_vector_cross);

//********************1. 可过渡性分析***********************//
void transition_possibility_analysis(vector<RowVector3d>& NORMAL_VECTOR_WALL, vector<MatrixXd>& SHRINK_WALL_BORDER) {

    // 全局规划时，对壁面进行法向量方向的抬升(unit : m)
    vector<MatrixXd> uplift_normal_vector_shrink_wall_border;
    uplift_normal_vector_shrink_wall_fun(uplift_normal_vector_shrink_wall_border, NORMAL_VECTOR_WALL, SHRINK_WALL_BORDER);
    
    // 对壁面进行位置约束判断
    transition_possibility_adjacency_matrix_fun(0, uplift_normal_vector_shrink_wall_border);
}

//********************1.1 对壁面在壁面法向量方向抬升，以消除姿态可达约束***********************//
void uplift_normal_vector_shrink_wall_fun(vector<MatrixXd>& uplift_normal_vector_shrink_wall_border, vector<RowVector3d>& NORMAL_VECTOR_WALL, vector<MatrixXd>& SHRINK_WALL_BORDER) {

    // 对每个壁面进行遍历，在壁面法向量方向抬升该壁面，以消除姿态可达约束
    for (int i = 0; i < SHRINK_WALL_BORDER.size(); i++) {
        // 每个壁面的顶点数
        int i_SHRINK_WALL_BORDER_point_nums = SHRINK_WALL_BORDER[i].rows();
        // 建立临时的壁面法向方向增量矩阵
        Eigen::MatrixXd normal_increment_matrix(i_SHRINK_WALL_BORDER_point_nums, 3);

        // 壁面法向方向增量矩阵的每一行都是该壁面的单位法向量
        for (int j = 0; j < i_SHRINK_WALL_BORDER_point_nums; j++) {
            normal_increment_matrix.row(j) << NORMAL_VECTOR_WALL[i];
        }
        
        // 壁面法向方向增量矩阵*抬升高度
        normal_increment_matrix << normal_increment_matrix * (ROBOT_CLIMBOT_5D_LENGTH[0] + ROBOT_CLIMBOT_5D_LENGTH[1]);

        // 将收缩后的壁面数据 加上 壁面法向方向增量矩阵,push进抬升后的壁面数据
        uplift_normal_vector_shrink_wall_border.push_back(normal_increment_matrix + SHRINK_WALL_BORDER[i]);

    }

    for (int i = 0; i < SHRINK_WALL_BORDER.size(); i++)
        std::cout << "uplift_normal_vector_shrink_wall_border:[ "<< i << " ]" << uplift_normal_vector_shrink_wall_border[i] << endl;
}



//********************1.2 对壁面进行位置约束判断***********************//
// start 从 0 开始, [n*(n-1)]/2 次递归
void transition_possibility_adjacency_matrix_fun(int start, vector<MatrixXd>& uplift_normal_vector_shrink_wall_border) {
        // 递归结束标志
        if (start >= uplift_normal_vector_shrink_wall_border.size()-1)  return;
       
        // 当前层任务[遍历每两个壁面]
        //*********第二个面************************************//
        for (int i = start + 1; i < uplift_normal_vector_shrink_wall_border.size(); i++) {

            // 储存每对线段的最近距离和最近点对
            pair<double, Eigen::MatrixXd> line_line_nearest_point(0, Eigen::MatrixXd::Zero(2,3));
            
            // 储存进行可过渡性判断的两个壁面序列，以便后面进行可达工作空间的判断
            vector<int> two_surface_serial_number(2, 0);
            
            // 两个面的两个线段
            Eigen::MatrixXd surface_1_line(2,3), surface_2_line(2,3);

            // 两个壁面的法向量叉乘
            Eigen::RowVector3d two_wall_normal_vector_cross(1,3);
            two_wall_normal_vector_cross << NORMAL_VECTOR_WALL[start].cross(NORMAL_VECTOR_WALL[i]);

            // 对 uplift_normal_vector_shrink_wall_border[start]的每条边 与 uplift_normal_vector_shrink_wall_border[i]的每条边 进行遍历求最近点对
            //*********第一个面的线段遍历************************************//       
            for (int j = 0; j < uplift_normal_vector_shrink_wall_border[start].rows(); j++) {

                // uplift_normal_vector_shrink_wall_border[start]的每条边
                if (j == uplift_normal_vector_shrink_wall_border[start].rows()-1) {
                    surface_1_line.row(0) << uplift_normal_vector_shrink_wall_border[start].row(j);
                    surface_1_line.row(1) << uplift_normal_vector_shrink_wall_border[start].row(0);
                }
                else {
                    surface_1_line.row(0) << uplift_normal_vector_shrink_wall_border[start].row(j);
                    surface_1_line.row(1) << uplift_normal_vector_shrink_wall_border[start].row(j+1);
                }
                std::cout <<"surface [" << start<< "]line : " <<  surface_1_line << endl;
                // 将第一个面的序号存入 two_surface_serial_number
                two_surface_serial_number[0] = start;

            //*********第二个面的线段遍历************************************//
                for (int k = 0; k < uplift_normal_vector_shrink_wall_border[i].rows(); k++) {

                    // uplift_normal_vector_shrink_wall_border[i]的每条边
                    if (k == uplift_normal_vector_shrink_wall_border[i].rows()-1) {
                        surface_2_line.row(0) << uplift_normal_vector_shrink_wall_border[i].row(k);
                        surface_2_line.row(1) << uplift_normal_vector_shrink_wall_border[i].row(0);
                    }    
                    else {
                        surface_2_line.row(0) << uplift_normal_vector_shrink_wall_border[i].row(k);
                        surface_2_line.row(1) << uplift_normal_vector_shrink_wall_border[i].row(k+1);
                    }

                    std::cout <<"surface [" << i << "]line : " << surface_2_line << endl;

                    // 将第二个面的序号存入 two_surface_serial_number
                    two_surface_serial_number[1] = i;

                    // 两条边进行求解最近点对
                    line_line_nearest_point = line_to_line_distance(surface_1_line, surface_2_line);
                    
                    Eigen::RowVector3d line_line_point;
                    line_line_point << line_line_nearest_point.second.row(0);

                    cout << "line_line_point:  dis = " <<line_line_nearest_point.first << "line_line_point : " << line_line_nearest_point.second << endl;
                    cout << "two_wall_normal_vector_cross: " << two_wall_normal_vector_cross << endl;
                    cout << "uplift_normal_vector_shrink_wall_border[i]: " << uplift_normal_vector_shrink_wall_border[i] << endl;
                    cout << " r = " << ROBOT_CLIMBOT_5D_LENGTH[2] + ROBOT_CLIMBOT_5D_LENGTH[3] << endl;
                    // 可达工作空间判断
                    bool robot_reachable_workspace = judgment_of_robot_reachable_workspace(uplift_normal_vector_shrink_wall_border[i], line_line_point, two_wall_normal_vector_cross);
                    line_line_point << line_line_nearest_point.second.row(1);
                    // 两条边均可作为机器人可达范围，说明这两壁面可过渡
                    if (robot_reachable_workspace == true || judgment_of_robot_reachable_workspace(uplift_normal_vector_shrink_wall_border[start], line_line_point, two_wall_normal_vector_cross)) {
                        // 不可过渡的情况下，矩阵初始化已经初始化为0
                        transition_possibility_adjacency_matrix(start, i) = 1;
                        transition_possibility_adjacency_matrix(i, start) = 1;
                        // 结束当前两个面的线段遍历，进入start壁面与(i+1)壁面的遍历
                        j = uplift_normal_vector_shrink_wall_border[start].rows();
                        k = uplift_normal_vector_shrink_wall_border[i].rows();
                    }
                    else {
                        transition_possibility_adjacency_matrix(start, i) = INT_MAX;
                        transition_possibility_adjacency_matrix(i, start) = INT_MAX;
                    }                                                               
                }
            }                       
        }
        // 递归到下一层
        transition_possibility_adjacency_matrix_fun(start + 1, uplift_normal_vector_shrink_wall_border);
}


//********************1.2 - 1 判断两条线段的最近距离点对是否在机器人的可达工作空间***********************//
bool judgment_of_robot_reachable_workspace(Eigen::MatrixXd& tested_wall, Eigen::RowVector3d& line_line_nearest_point, Eigen::RowVector3d& two_wall_normal_vector_cross) {
    
    int line_point_pair_number = 0;
    Eigen::MatrixXd line_point_pair(4,3);

    int two_side_number = 0;   
    double  two_surface_cross_vector_dot_test_line_1,two_surface_cross_vector_dot_test_line_2;
   
    // test_wall的顶点数
    for (int i = 0; i < tested_wall.rows(); i++) {
        if (i == tested_wall.rows()-1) {
            two_surface_cross_vector_dot_test_line_1 = two_wall_normal_vector_cross.dot(tested_wall.row(i) - line_line_nearest_point);
            two_surface_cross_vector_dot_test_line_2 = two_wall_normal_vector_cross.dot(tested_wall.row(0) - line_line_nearest_point);
        }
        else {
                two_surface_cross_vector_dot_test_line_1 = two_wall_normal_vector_cross.dot(tested_wall.row(i) - line_line_nearest_point);
                two_surface_cross_vector_dot_test_line_2 = two_wall_normal_vector_cross.dot(tested_wall.row(i+1) - line_line_nearest_point);
        }

        if (two_surface_cross_vector_dot_test_line_1 * two_surface_cross_vector_dot_test_line_2 < 0.00001) {
            two_side_number++;
            break;
        }

        if ( ((tested_wall.row(i) - line_line_nearest_point).dot(two_wall_normal_vector_cross)) < 0.0001 ) {

            line_point_pair.row(line_point_pair_number) << tested_wall.row(i);
            line_point_pair_number++;

        }
    }
    // 储存壁面的边
    Eigen::MatrixXd test_wall_line(2,3);

    // 储存边与面的可能的四个交点
    Eigen::MatrixXd wall_line_in_another_wall_point(4,3);

    Eigen::MatrixXd interline(2,3);

    int line_wall_point_number = 0;

    if (two_side_number == 1) {
        for (int j = 0; j < tested_wall.rows(); j++) {
            // 壁面边
            if (j == tested_wall.rows()-1) {
                test_wall_line << tested_wall.row(j), tested_wall.row(0);
            }
            else 
                test_wall_line << tested_wall.row(j), tested_wall.row(j+1);

            if ( abs((test_wall_line.row(1) - test_wall_line.row(0)).dot(two_wall_normal_vector_cross) ) < 0.00001 ) {
                continue;
            }

            pair<bool, Eigen::RowVector3d> line_with_surface_intersection(false, Eigen::RowVector3d::Zero(1,3));

            cout << "line_line_nearest_point : "<< line_line_nearest_point << endl;
            cout << "test_wall_line: " << test_wall_line << endl;
            cout << "two_wall_normal_vector_cross : " << two_wall_normal_vector_cross << endl;


            line_with_surface_intersection = line_to_surface(line_line_nearest_point, test_wall_line, two_wall_normal_vector_cross);
            cout << "line_with_surface_intersection : " << endl;
            cout << line_with_surface_intersection.second << endl;

            if (line_with_surface_intersection.first == true) {
                wall_line_in_another_wall_point.row(line_wall_point_number) << line_with_surface_intersection.second;
                line_wall_point_number++;
            }
        }

        if (line_wall_point_number > 2) {
            if ( (wall_line_in_another_wall_point.row(0) - wall_line_in_another_wall_point.row(1)).norm() < 0.00001) 
                interline << wall_line_in_another_wall_point.row(0),  wall_line_in_another_wall_point.row(2);
        }
        else {
            interline << wall_line_in_another_wall_point.row(0),  wall_line_in_another_wall_point.row(1);
        }     
    }
    else if (two_side_number == 0 && line_point_pair_number == 2) {
        interline << line_point_pair.row(0), line_point_pair.row(1);
    }
    else if (two_side_number == 0 && line_point_pair_number == 1) {
        interline << line_point_pair.row(0), line_point_pair.row(0);
    }
    else {
        return false;
    }

    // 两条线的最近点 与 线在另一面的交点的线
    pair<double, Eigen::RowVector3d> point_line_distance(0, Eigen::RowVector3d::Zero(1,3));

    cout << "line_line_nearest_point : " << line_line_nearest_point << endl;
    cout << "interline : " <<interline << endl;
    point_line_distance = point_to_line_distance(line_line_nearest_point, interline);
    cout << "point_line_distance : " << point_line_distance.first << endl;
    cout << "point_line_distance : " << point_line_distance.second << endl;

    if (point_line_distance.first <= (ROBOT_CLIMBOT_5D_LENGTH[2] + ROBOT_CLIMBOT_5D_LENGTH[3]) ){
        return true;
    } 
    else 
        return false;

}

// 线与面的交点测试(如无交点，则返回false, 点为0)
pair<bool, Eigen::RowVector3d> line_to_surface(Eigen::RowVector3d& line_nearest_point, Eigen::MatrixXd& line, Eigen::RowVector3d& two_wall_normal_vector_cross) {
    
    Eigen::RowVector3d  line_vector;

    pair<bool, Eigen::RowVector3d> line_with_surface_intersection(false, Eigen::RowVector3d::Zero(1,3)); 

    line_vector << line.row(1) - line.row(0);

    double line_mapping_point_rate = two_wall_normal_vector_cross.dot(line_nearest_point - line.row(0)) / two_wall_normal_vector_cross.dot(line_vector);

    if (line_mapping_point_rate >= 0 && line_mapping_point_rate <= 1) {
        line_with_surface_intersection.first = true;
        line_with_surface_intersection.second = line.row(0) + line_vector * line_mapping_point_rate;
    }
    else {
        line_with_surface_intersection.first = false;
        line_with_surface_intersection.second << 0, 0, 0;
    }
    return line_with_surface_intersection;

}







//-------------------------------------------------------------//
int main(int argc, char* argv[]) {

    read_global_variable_config();
    //面的点绕x旋转90度
    for (int i = 0; i < WALL_BORDER.size(); i++) {
        axis_rotate_angle('x', -90, WALL_BORDER[i]);
        cout << "the "<< i << "wall: " << WALL_BORDER[i] << endl;
    }
//     WALL_BORDER_1 = WALL_BORDER[0];
//     WALL_BORDER_2 = WALL_BORDER[1];
//     WALL_BORDER_3 = WALL_BORDER[2];

    // 对壁面进行缩放
    shrink_surface(WALL_BORDER);

    transition_possibility_analysis(NORMAL_VECTOR_WALL, SHRINK_WALL_BORDER);
    cout << "transition_possibility_adjacency_matrix : " << transition_possibility_adjacency_matrix << endl;

    return 0;
//    Eigen::MatrixXd line1(2, 3), line2(2,3);
//    line1 << 0.511185,  -1.06961,  0.395674,
//    0.48553, -0.462322,  0.390206;
//    line2 << 200.345, -1.07154,  1.18129,
//    198.739, -1.09376,  1.20422;
//    pair<double, Eigen::MatrixXd> line_to_line(0 ,  Eigen::MatrixXd::Zero(2, 3));
//    line_to_line =  line_to_line_distance( line1, line2);
//    cout << line_to_line.first << endl;
//    cout << line_to_line.second << endl;
}


