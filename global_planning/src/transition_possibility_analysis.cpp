#include "../config/read_config.h"
#include "header/global_planning.h"
#include "header/base_lib_code.h"
#include "../config/global_variable.h"
#include "distance_compute/distance_compute.h"


//********************1. 可过渡性分析***********************//
void transition_possibility_analysis(vector<RowVector3d>& NORMAL_VECTOR_WALL, vector<MatrixXd>& SHRINK_WALL_BORDER) {
    
    // 全局规划时，对壁面进行法向量方向的抬升(unit : m)
    vector<MatrixXd> uplift_normal_vector_shirink_wall_border;
    uplift_normal_vector_shirink_wall_fun(uplift_normal_vector_shirink_wall_border, NORMAL_VECTOR_WALL, SHRINK_WALL_BORDER);
    //############################################//
    std::cout << "$$$$$$$$ START THE ANALYSIS POSSIABLITY. $$$$$$$$" << endl;
    for (int i = 0; i < SHRINK_WALL_BORDER.size(); i++) {
        cout << "SHRINK_WALL_BORDER["<< i << "] = " << SHRINK_WALL_BORDER[i] << endl;
    }
    for (int i = 0; i < uplift_normal_vector_shirink_wall_border.size(); i++) {
        cout << "uplift_normal_vector_shirink_wall_border["<< i << "] = " << uplift_normal_vector_shirink_wall_border[i] << endl;
    }
    // 对壁面进行位置约束判断
    transition_possibility_adjacency_matrix_fun(0, uplift_normal_vector_shirink_wall_border);

}

//********************1.1 对壁面在壁面法向量方向抬升，以消除姿态可达约束***********************//
void uplift_normal_vector_shirink_wall_fun(vector<MatrixXd>& uplift_normal_vector_shirink_wall_border, vector<RowVector3d>& NORMAL_VECTOR_WALL, vector<MatrixXd>& SHRINK_WALL_BORDER) {

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
        uplift_normal_vector_shirink_wall_border.push_back(normal_increment_matrix + SHRINK_WALL_BORDER[i]);
    }
}

//********************1.2 对壁面进行位置约束判断***********************//
// start 从 0 开始, [n*(n-1)]/2 次递归
void transition_possibility_adjacency_matrix_fun(int start, vector<MatrixXd>& uplift_normal_vector_shirink_wall_border) {
        // 递归结束标志
        if (start >= uplift_normal_vector_shirink_wall_border.size()-1)  return;
       
        // 当前层任务[遍历每两个壁面]
        //*********第二个面************************************//
        for (int i = start + 1; i < uplift_normal_vector_shirink_wall_border.size(); i++) {

            // 储存每对线段的最近距离和最近点对
            pair<double, Eigen::MatrixXd> line_line_distance(0, Eigen::MatrixXd::Zero(2,3));
            
            // 储存进行可过渡性判断的两个壁面序列，以便后面进行可达工作空间的判断
            vector<int> two_surface_serial_number(2, 0);
            
            // 两个面的两个线段
            Eigen::MatrixXd surface_1_line(2,3), surface_2_line(2,3);

            // 对 uplift_normal_vector_shirink_wall_border[start]的每条边 与 uplift_normal_vector_shirink_wall_border[i]的每条边 进行遍历求最近点对
            //*********第一个面的线段遍历************************************//       
            for (int j = 0; j < uplift_normal_vector_shirink_wall_border[start].rows(); j++) {

                // uplift_normal_vector_shirink_wall_border[start]的每条边
                if (j == uplift_normal_vector_shirink_wall_border[start].rows()-1) {
                    surface_1_line.row(0) << uplift_normal_vector_shirink_wall_border[start].row(j);
                    surface_1_line.row(1) << uplift_normal_vector_shirink_wall_border[start].row(0);
                }
                else {
                    surface_1_line.row(0) << uplift_normal_vector_shirink_wall_border[start].row(j);
                    surface_1_line.row(1) << uplift_normal_vector_shirink_wall_border[start].row(j+1);
                }
                cout <<"surface [" << start<< "]line : " <<  surface_1_line << endl;
                // 将第一个面的序号存入 two_surface_serial_number
                two_surface_serial_number[0] = start;

            //*********第二个面的线段遍历************************************//
                for (int k = 0; k < uplift_normal_vector_shirink_wall_border[i].rows(); k++) {
                    // 存储两条线的最短距离
                    double store_min_line_line_distance = INT_MAX;

                    // uplift_normal_vector_shirink_wall_border[i]的每条边
                    if (k == uplift_normal_vector_shirink_wall_border[i].rows()-1) {
                        surface_2_line.row(0) << uplift_normal_vector_shirink_wall_border[i].row(k); 
                        surface_2_line.row(1) << uplift_normal_vector_shirink_wall_border[i].row(0); 
                    }    
                    else {
                        surface_2_line.row(0) << uplift_normal_vector_shirink_wall_border[i].row(k); 
                        surface_2_line.row(1) << uplift_normal_vector_shirink_wall_border[i].row(k+1); 
                    }

                    cout <<"surface [" << i << "]line : " << surface_2_line << endl;

                    // 将第一个面的序号存入 two_surface_serial_number
                    two_surface_serial_number[1] = i;

                    // 两条边进行求解最近点对
                    line_line_distance = line_to_line_distance(surface_1_line, surface_2_line);

                    // 可达工作空间判断
                    bool robot_reachable_workspace = judgment_of_robot_reachable_workspace(line_line_distance);

                    // 两条边均可作为机器人可达范围，说明这两壁面可过渡
//                    if (robot_reachable_workspace == true) {
//                        // 不可过渡的情况下，矩阵初始化已经初始化为0
//                        transition_possibility_adjacency_matrix(start, i) = 1;
//                        transition_possibility_adjacency_matrix(i, start) = 1;
//                        std::cout << "surface[ " << start << " ]" << "and surface[ " << i << " ] can translate." << std::endl;
//                        // 结束当前两个面的线段遍历，进入start壁面与(i+1)壁面的遍历
//                        j = uplift_normal_vector_shirink_wall_border[start].rows();
//                        k = uplift_normal_vector_shirink_wall_border[i].rows();
//                    }
//                    else {
//                        transition_possibility_adjacency_matrix(start, i) = INT_MAX;
//                        transition_possibility_adjacency_matrix(i, start) = INT_MAX;
//                    }
                    if (robot_reachable_workspace == true) {
                        // 不可过渡的情况下，矩阵初始化已经初始化为0
                        // 存储最短的距离
                        if (line_line_distance.first < store_min_line_line_distance) {
                            store_min_line_line_distance = line_line_distance.first;
                        }
                        transition_possibility_adjacency_matrix(start, i) = store_min_line_line_distance;
                        transition_possibility_adjacency_matrix(i, start) = store_min_line_line_distance;

                        std::cout << "surface[ " << start << " ]" << "and surface[ " << i << " ] can translate." << std::endl;

                    }
                }
            }
            // 设置不可过渡的矩阵位置值为 INT_MAX
            if (transition_possibility_adjacency_matrix(start, i) == 0) {
                transition_possibility_adjacency_matrix(start, i) = INT_MAX;
                transition_possibility_adjacency_matrix(i, start) = INT_MAX;
            }
        }

        // 递归到下一层
        transition_possibility_adjacency_matrix_fun(start + 1, uplift_normal_vector_shirink_wall_border);
}

//********************1.2 - 1 判断两条线段的最近距离点对是否在机器人的可达工作空间***********************//
bool judgment_of_robot_reachable_workspace(pair<double, Eigen::MatrixXd>& line_line_distance) {
    // 判断两最近点的欧式距离是否小于(2L2)--小于，则说明位置可达
    if (line_line_distance.first <= (ROBOT_CLIMBOT_5D_LENGTH[2] + ROBOT_CLIMBOT_5D_LENGTH[3])) {
        return true;
    }
    else
        return false;
}

//********************2 壁面过渡序列搜索策略***********************//
void wall_transition_strategy(vector<vector<int>> &all_pathway, vector<int> &select_pathway, int strategy) {

    if (all_pathway.size() <= 0) {
        std::cout << "Have no legal path." << endl;
        return;
    }

    int wall_size = select_pathway.size();   // 储存路径的壁面数量
    int pathway_number = 0;    // 储存的壁面路径序号
    double wall_distance = INT_MAX;  // 储存路径间的最小距离

    switch(strategy) {
        case 0:         // 经过的壁面最少
            for (int i = 1; i < all_pathway.size(); i++) {
                // 若这个壁面路径的壁面数量比上一个少，则重新赋值序号
                if (all_pathway[i].size() < wall_size) {
                    wall_size = all_pathway[i].size();
                    pathway_number = i;
                }
                else {
                    wall_size = wall_size;
                    pathway_number = pathway_number;
                }
            }

        case 1:         // 经过的壁面最多
            for (int i = 1; i < all_pathway.size(); i++) {
                // 若这个壁面路径的壁面数量比上一个多，则重新赋值序号
                if (all_pathway[i].size() > wall_size) {
                    wall_size = all_pathway[i].size();
                    pathway_number = i;
                }
                else {
                    wall_size = wall_size;
                    pathway_number = pathway_number;
                }
            }

        case 2:  // 壁面间的和距离最少
            // 遍历每一个壁面可行路径
            for (int i = 0; i < all_pathway.size()-1; i++) {
                double singal_wall_distance = 0;
                // 对当前壁面可行路径进行距离求和
                for (int j = 0; j < all_pathway[i].size()-1; j++) {
                    singal_wall_distance += transition_possibility_adjacency_matrix(j, j+1);
                }

                // 若这个壁面路径的壁面距离比上一个更小，则更新序号
                if (singal_wall_distance < wall_distance) {
                    wall_distance = singal_wall_distance;
                    pathway_number = i;
                }
                else {
                    wall_distance = singal_wall_distance;
                    pathway_number = pathway_number;
                }
            }
            break;
        default :
            std::cout << "illegal wall transition strategy." << std::endl;
            break;
    }
    // 将选择好的路径序号填入
    select_pathway = all_pathway[pathway_number];

}


//********************2.1 对邻接矩阵进行深度优先搜索，获得所有可能的壁面可过渡路径序列***********************//
void WALL_SEARCH_DFS(int* visited, Eigen::MatrixXd &transition_possibility_adjacency_matrix,int start,int end,vector<int> a_pathway,vector<vector<int>> &all_pathway)
{
    a_pathway.push_back(start);//将当前节点入栈
    visited[start]=true;//标记为已经访问过
    int middle = 0;//一个索引，用来找到下一个节点

    while(a_pathway.empty()==false)//如果栈空则结束
    {
        if(start==end)//如果当前节点就是目标节点
        {
            all_pathway.push_back(a_pathway);//保存一条路径
            a_pathway.pop_back();//退栈，回溯
            visited[start]=false;//设置为没有被访问过
            break; //退出一个递归
        }

        while(middle<transition_possibility_adjacency_matrix.rows())//如果没有到最后一个(遍历所有的可能下一个节点)
        {
            if(transition_possibility_adjacency_matrix(start,middle) != 0 &&  transition_possibility_adjacency_matrix(start,middle) != INT_MAX && visited[middle]==false)
            {
                WALL_SEARCH_DFS(visited, transition_possibility_adjacency_matrix,middle,end,a_pathway,all_pathway); //递归查询
            }
            middle++;
        }
        if(middle==transition_possibility_adjacency_matrix.rows()) //当前层的递归找到最后一个，都没有合适的结果，则标记该节点为没有访问过
        {
            visited[a_pathway.at(a_pathway.size()-1)] = false;//先标记栈中最后一个元素为没有访问过
            a_pathway.pop_back();//退栈，回溯
            break;  //退出一个递归
        }
    }
}


//bool judgment_of_robot_reachable_workspace(vector<MatrixXd>& uplift_normal_vector_shirink_wall_border, pair<double, Eigen::MatrixXd>& line_line_distance, vector<int>& two_surface_serial_number) {
//        // 两个壁面的法向量的叉乘
//        Eigen::RowVector3d two_surface_normal_vector_cross;
//        // line1_base_circle_low_point: line1的最近点为base基座时，形成的工作可达圆空间的最低点
//        // line2_base_circle_low_point: line2的最近点为base基座时，形成的工作可达圆空间的最低点
//        Eigen::RowVector3d line1_base_circle_low_point,line2_base_circle_low_point;
//
//        // 两个壁面的壁面坐标系
//        Eigen::RowVector3d surface_vector_x,surface_vector_y,surface_vector_z;
//
//        Eigen::Matrix3d surface_vector_frame(3,3);
//
//        // 两个最近点对的坐标转到壁面的坐标系上
//        Eigen::Vector3d line_2_point_tran_to_surface_1, line_1_point_tran_to_surface_1;
//        Eigen::Vector3d line_1_point_tran_to_surface_2, line_2_point_tran_to_surface_2;
//
//
//        // 两个壁面的法向量的叉乘
//        two_surface_normal_vector_cross << NORMAL_VECTOR_WALL[two_surface_serial_number[0]].cross(NORMAL_VECTOR_WALL[two_surface_serial_number[1]]);
//
//        // 如果两个壁面平行(法向量叉乘为0), 机器人可达工作空间为一个球，只需判断最近点对的距离是否小于(ROBOT_CLIMBOT_5D_LENGTH[2] + ROBOT_CLIMBOT_5D_LENGTH[3])
//        if (two_surface_normal_vector_cross == Eigen::RowVector3d::Zero(1,3)) {
//
//            if (line_line_distance.first <= (ROBOT_CLIMBOT_5D_LENGTH[2] + ROBOT_CLIMBOT_5D_LENGTH[3])) {
//                return true;
//            }
//        }
//        // 两个壁面不平行
//        else {
//            // 确保了最近点对的距离问题，只需要确保最近点在面上就行
//            if (line_line_distance.first <= (ROBOT_CLIMBOT_5D_LENGTH[2] + ROBOT_CLIMBOT_5D_LENGTH[3])) {
//
//            //***以line1的最近点为base基座点, 以line2的最近点为end落足点**********//
//                surface_vector_x << (uplift_normal_vector_shirink_wall_border[two_surface_serial_number[0]].row(1) - uplift_normal_vector_shirink_wall_border[two_surface_serial_number[0]].row(0)).normalized();
//
//                surface_vector_y << (surface_vector_x.cross(NORMAL_VECTOR_WALL[two_surface_serial_number[0]])).normalized();
//
//                surface_vector_z << NORMAL_VECTOR_WALL[two_surface_serial_number[0]];
//                // 将行向量转成列向量，填入 surface_vector_frame
//                surface_vector_frame << surface_vector_x.transpose(), surface_vector_y.transpose(), surface_vector_z.transpose();
//
//                cout << "surface_vector_frame : " << surface_vector_frame << endl;
//
//                // 将两个最近点对的坐标转到壁面1的坐标系上
//                line_2_point_tran_to_surface_1 << surface_vector_frame.inverse() * line_line_distance.second.row(1).transpose();
//
//                cout << "line_2_point_tran_to_surface_1 : " << line_2_point_tran_to_surface_1 << endl;
//
//                line_1_point_tran_to_surface_1 << surface_vector_frame.inverse() * line_line_distance.second.row(0).transpose();
//
//                cout << "line_1_point_tran_to_surface_1 : " << line_1_point_tran_to_surface_1 << endl;
//
//
//                // 如果 line_2_point_tran_to_surface_1(0,0)也就是line2上的最近点转换到line1所在平面上的坐标系下的坐标表示 与 line1的最近点在line1所在平面上的坐标表示
//                // 在x轴(朝外)上没有差距距离(dx = 0)，则说明line2上的最近点在line1最近点构建的平面内
//                cout << "line_2_point_tran_to_surface_1(0,0) - line_1_point_tran_to_surface_1(0,0) : " << line_2_point_tran_to_surface_1(0,0) - line_1_point_tran_to_surface_1(0,0) << endl;
//                if (abs(line_2_point_tran_to_surface_1(0,0) - line_1_point_tran_to_surface_1(0,0)) <= 0.0001) {
//
//                    return true;
//                }
//
//            //*******切换到以line2的最近点为base基座点, 以line1的最近点为end落足点**********//
//                else {
//
//                    surface_vector_x << (uplift_normal_vector_shirink_wall_border[two_surface_serial_number[1]].row(1) - uplift_normal_vector_shirink_wall_border[two_surface_serial_number[1]].row(0)).normalized();
//
//                    surface_vector_y << (surface_vector_x.cross(NORMAL_VECTOR_WALL[two_surface_serial_number[1]])).normalized();
//
//                    surface_vector_z << NORMAL_VECTOR_WALL[two_surface_serial_number[1]];
//                    // 将行向量转成列向量，填入 surface_vector_frame
//                    surface_vector_frame << surface_vector_x.transpose(), surface_vector_y.transpose(), surface_vector_z.transpose();
//
//                    cout << "surface_vector_frame : " << surface_vector_frame << endl;
//
//                    // 将两个最近点对的坐标转到壁面1的坐标系上
//                    line_1_point_tran_to_surface_2 << surface_vector_frame.inverse() * line_line_distance.second.row(1).transpose();
//
//                    cout << "line_1_point_tran_to_surface_2 : " << line_1_point_tran_to_surface_2 << endl;
//
//                    line_2_point_tran_to_surface_2 << surface_vector_frame.inverse() * line_line_distance.second.row(0).transpose();
//
//                    cout << "line_2_point_tran_to_surface_2 : " << line_2_point_tran_to_surface_2 << endl;
//
//
//                    // 如果 line_1_point_tran_to_surface_2(0,0)也就是line1上的最近点转换到line2所在平面上的坐标系下的坐标表示 与 line2的最近点在line2所在平面上的坐标表示
//                    // 在x轴(朝外)上没有差距距离(dx = 0)，则说明line1上的最近点在line2最近点构建的平面内
//                    cout << "abs(line_1_point_tran_to_surface_2(0,0) - line_2_point_tran_to_surface_2(0,0)) : " << line_1_point_tran_to_surface_2(0,0) - line_2_point_tran_to_surface_2(0,0) << endl;
//                    cout << "abs(line_1_point_tran_to_surface_2(0,0) - line_2_point_tran_to_surface_2(0,0)) : " << line_1_point_tran_to_surface_2(1,0) - line_2_point_tran_to_surface_2(1,0) << endl;
//                    cout << "abs(line_1_point_tran_to_surface_2(0,0) - line_2_point_tran_to_surface_2(0,0)) : " << line_1_point_tran_to_surface_2(2,0) - line_2_point_tran_to_surface_2(2,0) << endl;
//                    if (abs(line_1_point_tran_to_surface_2(1,0) - line_2_point_tran_to_surface_2(1,0)) <= 0.005) {
//                        return true;
//                    }
//                    else
//                        return false;
//                }
//            }
//            else
//                return false;
//
//        }
//
//  }


//-------------------------------------------------------------//


int main(int argc, char* argv[]) {

    read_global_variable_config();                  
    //面的点绕x旋转90度
    for (int i = 0; i < WALL_BORDER.size(); i++) {
        axis_rotate_angle('x', -90, WALL_BORDER[i]); 
        cout << "the "<< i << "wall: " << WALL_BORDER[i] << endl;       
    }
     WALL_BORDER_1 = WALL_BORDER[0];
     WALL_BORDER_2 = WALL_BORDER[1];
     WALL_BORDER_3 = WALL_BORDER[2];
   
    // 对壁面进行缩放
    shrink_surface(WALL_BORDER);

    transition_possibility_analysis(NORMAL_VECTOR_WALL, SHRINK_WALL_BORDER);
    std::cout << "transition_possibility_adjacency_matrix : " << transition_possibility_adjacency_matrix << std::endl;

    int start = 1, end = 0;
    vector<int> a_pathway;
    vector<vector<int>> all_pathway;

    // 壁面序列DFS搜索时的已访问节点
    int visited[100]= {0};

    WALL_SEARCH_DFS(visited, transition_possibility_adjacency_matrix, start, end, a_pathway, all_pathway);
    for (int i = 0; i < all_pathway.size(); i++) {
        std::cout << "the [ "<< i <<" ] pathway: " << std::endl;
        for (int j = 0; j < all_pathway[i].size(); j++) {
            std::cout << all_pathway[i][j] << ", ";
        }
        std::cout << endl;
    }
    return 0;

}






