#include "../../config/global_variable.h"
#include "distance_compute.h"
//#include "../global_planning.h"
// 面与长方体的距离
// void border_obatacle_distance(MatrixXd& border, MatrixXd& obstacle) {
    
// }

double clamp(double n, double min, double max);


// 线段与面求距,返回最短距离与相应的两点,po1为线段上的点，po2为面上的点
pair<double, Eigen::MatrixXd> line_to_surface_distance( Eigen::MatrixXd& line, Eigen::MatrixXd& surface) {
    
    // pair.first 为最短距离； MatrixXd.row(0) 为线段上的点, MatrixXd.row(1) 为面上的点
    pair<double, Eigen::MatrixXd> line_surface_distance(0, MatrixXd::Zero(2,3));

    int surface_point_num = surface.rows();
//************若线段与面内有交点，点到面的距离为0**************************//
    // 壁面法向量
    Eigen::RowVector3d surface_normal_vector, line_vector;
    Eigen::RowVector3d surface_vector_1,surface_vector_2;
    surface_vector_1 << surface.row(1) - surface.row(0);
    surface_vector_2 << surface.row(2) - surface.row(0);
    surface_normal_vector << surface_vector_1.cross(surface_vector_2);
    line_vector << line.row(1) - line.row(0);
    // 先求解线段是否与面相交，相交距离为0
    double line_rate = (surface_normal_vector.dot(surface.row(0) - line.row(0)))/(surface_normal_vector.dot(line.row(1) - line.row(0)));
    
    // 线段在平面上的点
    Eigen::RowVector3d line_point_on_surface; 
    line_point_on_surface << line.row(0) + line_vector * line_rate;
    
    // 线段与面有交点,则最近点就是交点，最小距离为0
    if (line_rate >= 0 && point_whether_in_surface(line_point_on_surface, surface)) {
        line_surface_distance.second.row(0) << line_point_on_surface;
        line_surface_distance.second.row(1) << line_point_on_surface;
        line_surface_distance.first = 0;
        return line_surface_distance;
    }                     
    
//************求两个线段端点到面的最近距离**************************//
// 这种情况就是线段到面的最近点在面内(不含边界)
    // 线段的两个端点
    Eigen::RowVector3d line_point_1, line_point_2;
    // 存储面上的两个投影点
    Eigen::RowVector3d line_point_1_mapping, line_point_2_mapping;
    //两个线段端点到面的最近距离
    double line_point_1_to_surface_distance, line_point_2_to_surface_distance;
    
    line_point_1 << line.row(0);
    line_point_2 << line.row(1);
    
    pair<bool, Eigen::RowVector3d> mapping_point(true, Eigen::RowVector3d::Zero(1,3));
    // 点1在面上的投影点是否在面内
    mapping_point = mapping_point_on_surface(line_point_1, surface);
    // 是
    if (mapping_point.first == true) {
        line_point_1_to_surface_distance = (line_point_1 - mapping_point.second).norm();
        line_point_1_mapping = mapping_point.second;
    }
    else 
        line_point_1_to_surface_distance = INT_MAX;

    // 点2在面上的投影点是否在面内
    mapping_point = mapping_point_on_surface(line_point_2, surface);
    // 是
    if (mapping_point.first == true) {
        line_point_2_to_surface_distance = (line_point_1 - mapping_point.second).norm();
        line_point_2_mapping = mapping_point.second;
    }
    else 
        line_point_2_to_surface_distance = INT_MAX;

    // 取两个距离的最小值
    if (line_point_1_to_surface_distance < line_point_2_to_surface_distance) {
        line_surface_distance.first = line_point_1_to_surface_distance;
        line_surface_distance.second.row(0) = line_point_1;
        line_surface_distance.second.row(1) = line_point_1_mapping;
    }
    else {
        line_surface_distance.first = line_point_2_to_surface_distance;
        line_surface_distance.second.row(0) = line_point_2;
        line_surface_distance.second.row(1) = line_point_2_mapping;
    }

//************求线段 到 面的每个线段的距离**************************//
// 这种情况就是线段到面的最近点在面的边界
    // 面的边的两点
    Eigen::MatrixXd surface_line_point(2,3);
    // 线段到线段的距离(矩阵的第一行为line1的最近点, 矩阵的第二行为line2的最近点)
    pair<double, Eigen::MatrixXd> line_line_distance(0, Eigen::MatrixXd::Zero(2,3));
    for (int i = 0; i < surface_point_num; i++) {
        if (i == surface_point_num - 1){ 
            surface_line_point << surface.row(i) , surface.row(0);
        }
        else {
            surface_line_point << surface.row(i), surface.row(i+1);
        }
        line_line_distance = line_to_line_distance(line, surface_line_point);  // 求线段与线段间的最近距离  
        cout << "line_line_distance ::" << line_line_distance.second << endl;
        // 线段到面的最近点在面内(不含边界) 与 线段到面的每个线段的距离(边界) 选取最小的
        if (line_line_distance.first < line_surface_distance.first) {
            line_surface_distance.first = line_line_distance.first;
            line_surface_distance.second.row(0) << line_line_distance.second.row(0);
            line_surface_distance.second.row(1) << line_line_distance.second.row(1);
        }
    }
    cout << "line_surface_distance: " << line_surface_distance.second << endl;
    return line_surface_distance;
}

// 线段之间的最近距离和最近点对(MatrixXd 的第一行为line1的最近点，第二行为line2的最近点)
// 参考：[空间中2条线段的最短距离](https://download.csdn.net/download/liying426/5058179?utm_medium=distribute.pc_relevant_download.none-task-download-baidujs-2.nonecase&depth_1-utm_source=distribute.pc_relevant_download.none-task-download-baidujs-2.nonecase)
// 注意上面文档中，关于两线段平行的最短距离是错的. 线段平行时，最短距离点对其中一个一定是4个端点的其中一个，所以遍历4个端点到另一线段的最短距离就行
pair<double, Eigen::MatrixXd> line_to_line_distance(MatrixXd& line1, MatrixXd& line2) {

    pair<double, Eigen::MatrixXd> line_line_distance(0, Eigen::MatrixXd::Zero(2,3));

    // line1的A点坐标（x1,y1,z1）, line1的B点坐标（x2,y2,z2）， line2的C点坐标（x3,y3,z3）， line2的D点坐标(x4,y4,z4)
    double x1 = line1(0,0);
    double y1 = line1(0,1);
    double z1 = line1(0,2);
    double x2 = line1(1,0);
    double y2 = line1(1,1);
    double z2 = line1(1,2);
    double x3 = line2(0,0);
    double y3 = line2(0,1);
    double z3 = line2(0,2);
    double x4 = line2(1,0);
    double y4 = line2(1,1);
    double z4 = line2(1,2);
    // 最短距离对line1参数s求偏导的方程参数
    double partial_parameter_s_a = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1);
    double partial_parameter_s_b = -((x2-x1)*(x4-x3) + (y2-y1)*(y4-y3) + (z2-z1)*(z4-z3));
    double partial_parameter_s_c = -((x1-x2)*(x1-x3) + (y1-y2)*(y1-y3) + (z1-z2)*(z1-z3));
    // 最短距离对line2参数t求偏导的方程参数
    double partial_parameter_t_d = -((x2-x1)*(x4-x3) + (y2-y1)*(y4-y3) + (z2-z1)*(z4-z3));
    double partial_parameter_t_e = (x4-x3)*(x4-x3) + (y4-y3)*(y4-y3) + (z4-z3)*(z4-z3);
    double partial_parameter_t_f = -((x1-x3)*(x4-x3) + (y1-y3)*(y4-y3) + (z1-z3)*(z4-z3));

//*****************两线段平行的时候*******************************//
    if (abs(partial_parameter_s_a*partial_parameter_t_e - partial_parameter_s_b*partial_parameter_t_d) < 0.0001) {
        // 储存4个端点到另外线段的最近点和距离
        pair<double, Eigen::RowVector3d> point_line_distance_1(0, Eigen::RowVector3d::Zero(1,3));
        pair<double, Eigen::RowVector3d> point_line_distance_2(0, Eigen::RowVector3d::Zero(1,3));
        pair<double, Eigen::RowVector3d> point_line_distance_3(0, Eigen::RowVector3d::Zero(1,3));
        pair<double, Eigen::RowVector3d> point_line_distance_4(0, Eigen::RowVector3d::Zero(1,3));

        Eigen::RowVector3d test_point(1,3);

        test_point << line1.row(0);
        point_line_distance_1 = point_to_line_distance(test_point, line2);
        test_point << line1.row(1);
        point_line_distance_2 = point_to_line_distance(test_point, line2);
        test_point << line2.row(0);
        point_line_distance_3 = point_to_line_distance(test_point, line1);
        test_point << line2.row(1);
        point_line_distance_4 = point_to_line_distance(test_point, line1);

        // 将最小距离值放入line_line_distance.first
        line_line_distance.first = min( min( min(point_line_distance_1.first, point_line_distance_2.first), point_line_distance_3.first), point_line_distance_4.first);

        // 获得最近距离点对
        if (point_line_distance_1.first == line_line_distance.first) {
            line_line_distance.second.row(0) << line1.row(0);
            line_line_distance.second.row(1) << point_line_distance_1.second;
        }
        else if (point_line_distance_2.first == line_line_distance.first) {
            line_line_distance.second.row(0) << line1.row(1);
            line_line_distance.second.row(1) << point_line_distance_2.second;
        }
        else if (point_line_distance_3.first == line_line_distance.first) {
            line_line_distance.second.row(1) << line2.row(0);
            line_line_distance.second.row(0) << point_line_distance_3.second;
        }
        else if (point_line_distance_4.first == line_line_distance.first) {
            line_line_distance.second.row(1) << line2.row(1);
            line_line_distance.second.row(0) << point_line_distance_4.second;
        }
        std::cout << "line_line_distance.first : " << line_line_distance.first << std::endl;
        std::cout << "line_line_distance.second : " << line_line_distance.second << std::endl;
        return line_line_distance;
    }

    double partial_parameter_s = (partial_parameter_s_b * partial_parameter_t_f - partial_parameter_t_e * partial_parameter_s_c)/(partial_parameter_s_a * partial_parameter_t_e - partial_parameter_s_b * partial_parameter_t_d);
    double partial_parameter_t = (partial_parameter_s_a * partial_parameter_t_f - partial_parameter_t_d * partial_parameter_s_c)/(partial_parameter_s_b * partial_parameter_t_d - partial_parameter_s_a * partial_parameter_t_e);

    std::cout << "partial_parameter_s : " << partial_parameter_s << endl;
    std::cout << "partial_parameter_t : " << partial_parameter_t << endl;
    // 说明线段的最近点对都在线段上
    if (partial_parameter_s >= 0 && partial_parameter_s <= 1 && partial_parameter_t >= 0 && partial_parameter_t <= 1) {
        // line1的最近点P(line1_x, line1_y, line1_z)
        // line2的最近点Q(line2_x, line2_y, line2_z)
        // 这两点形成的线段PQ为两条线段的共垂线段
        double line1_x = x1 + partial_parameter_s*(x2 - x1);
        double line1_y = y1 + partial_parameter_s*(y2 - y1);
        double line1_z = z1 + partial_parameter_s*(z2 - z1);
        double line2_x = x3 + partial_parameter_t*(x4 - x3);
        double line2_y = y3 + partial_parameter_t*(y4 - y3);
        double line2_z = z3 + partial_parameter_t*(z4 - z3);

        line_line_distance.second.row(0) << line1_x, line1_y, line1_z;
        line_line_distance.second.row(1) << line2_x, line2_y, line2_z;

        line_line_distance.first = (line_line_distance.second.row(1) - line_line_distance.second.row(0)).norm();
        return line_line_distance;
    }
    // 说明线段的最近点对不在线段上,那必然最近线段点对其中一个一定在端点上
    else {
        // 储存4个端点到另外线段的最近点和距离
        pair<double, Eigen::RowVector3d> point_line_distance_1(0, Eigen::RowVector3d::Zero(1,3));
        pair<double, Eigen::RowVector3d> point_line_distance_2(0, Eigen::RowVector3d::Zero(1,3));
        pair<double, Eigen::RowVector3d> point_line_distance_3(0, Eigen::RowVector3d::Zero(1,3));
        pair<double, Eigen::RowVector3d> point_line_distance_4(0, Eigen::RowVector3d::Zero(1,3));

        Eigen::RowVector3d test_point(1,3);

        test_point << line1.row(0);
        point_line_distance_1 = point_to_line_distance(test_point, line2);
        cout << "point_line_distance_1 : " << point_line_distance_1.second << endl;
        test_point << line1.row(1);
        point_line_distance_2 = point_to_line_distance(test_point, line2);
        cout << "point_line_distance_2 : " << point_line_distance_2.second << endl;
        test_point << line2.row(0);
        point_line_distance_3 = point_to_line_distance(test_point, line1);
        cout << "point_line_distance_3 : " << point_line_distance_3.second << endl;
        test_point << line2.row(1);
        point_line_distance_4 = point_to_line_distance(test_point, line1);
        cout << "point_line_distance_4 : " << point_line_distance_4.second << endl;

        // 将最小距离值放入line_line_distance.first
        line_line_distance.first = min( min( min(point_line_distance_1.first, point_line_distance_2.first), point_line_distance_3.first), point_line_distance_4.first);

        // 获得最近距离点对
        if (point_line_distance_1.first == line_line_distance.first) {
            line_line_distance.second.row(0) << line1.row(0);
            line_line_distance.second.row(1) << point_line_distance_1.second;
        }
        else if (point_line_distance_2.first == line_line_distance.first) {
            line_line_distance.second.row(0) << line1.row(1);
            line_line_distance.second.row(1) << point_line_distance_2.second;
        }
        else if (point_line_distance_3.first == line_line_distance.first) {
            line_line_distance.second.row(1) << line2.row(0);
            line_line_distance.second.row(0) << point_line_distance_3.second;
        }
        else if (point_line_distance_4.first == line_line_distance.first) {
            line_line_distance.second.row(1) << line2.row(1);
            line_line_distance.second.row(0) << point_line_distance_4.second;
        }
        cout << "line_line_distance.first : " << line_line_distance.first << endl;
        cout << "line_line_distance.second : " << line_line_distance.second << endl;
        return line_line_distance;
    }


}


// 点与面求距离，返回最短距离与面上映射的点
pair<double, Eigen::RowVector3d> point_to_surface_distance(Eigen::RowVector3d& point, MatrixXd& surface) {
    
    pair<double, Eigen::RowVector3d> point_surface_distance(0, Eigen::RowVector3d::Zero(1,3));
    
    // 点在面的投影点
    pair<bool, Eigen::RowVector3d> point_mapping_point_on_surface(true, Eigen::RowVector3d::Zero(1,3));
    
    // 点在面上
    bool point_on_the_surface = point_whether_in_surface(point, surface);

    // 将面化为线
    MatrixXd surface_line(2,3);

    // 点到线的距离
    pair<double, Eigen::RowVector3d> point_line_distance(0, Eigen::RowVector3d::Zero(1,3));

    // if point is on the surface, directly return
    if (point_on_the_surface == true){
        point_surface_distance.first = 0;
        point_surface_distance.second = point;
        return point_surface_distance;
    }

    point_mapping_point_on_surface = mapping_point_on_surface(point, surface);

    // point's mapping point is on the surface 
    if (point_mapping_point_on_surface.first == true) {
        point_surface_distance.first = (point - point_mapping_point_on_surface.second).norm();
        point_surface_distance.second = point_mapping_point_on_surface.second;
    }
    else {
        point_surface_distance.first = INT_MAX;
        point_surface_distance.second = point_mapping_point_on_surface.second;
    }

    // circle every line of surface
    for (int i = 0; i < surface.rows(); i++) {
        if ( i == surface.rows()-1) 
            surface_line << surface.row(i), surface.row(0);
        else 
            surface_line << surface.row(i), surface.row(i+1);

        // point to line distance
        point_line_distance = point_to_line_distance(point, surface_line);
        
        // save the minist value of point_surface_distance
        if (point_line_distance.first < point_surface_distance.first){

            point_surface_distance.first = point_line_distance.first;
            point_surface_distance.second = point_line_distance.second;       
        }
    }

    return point_surface_distance;
    
}


// 求点到线段的最小距离，返回距离及对应的点
pair<double, Eigen::RowVector3d> point_to_line_distance(Eigen::RowVector3d& point, MatrixXd& line) {
    // return : min_distance, mapping_point
    pair<double, Eigen::RowVector3d> point_line_distance(0, Eigen::RowVector3d::Zero(1,3));

    // line vector
    Eigen::RowVector3d line_vector, point_vector_1, point_vector_2;
    line_vector << line.row(1) - line.row(0);
    
    // line norm
    double distance = 0;
    distance = (line.row(0) - line.row(1)).norm();

    // if line is very short, take the line as a point
    if (distance < 0.00001) {
        point_line_distance.first = (line.row(0) - point).norm();
        point_line_distance.second = line.row(0);
        return point_line_distance;
    }

    else {
        point_vector_1 << point - line.row(0);
        point_vector_2 << point - line.row(1);
        // if angle(point_vector_1, line_vector) >90(degee), the line.row(0) is the nearest point
        if ( line_vector.dot(point_vector_1)  <= 0.00001) {
            point_line_distance.first = point_vector_1.norm();
            point_line_distance.second = line.row(0);
            return point_line_distance;
        }
        // angle(point_vector_2, line_vector) < 90(degee), the line.row(1) is the nearest point
        if ( line_vector.dot(point_vector_2) > 0.00001) {
            point_line_distance.first = point_vector_2.norm();
            point_line_distance.second = line.row(1);
            return point_line_distance;
        }
        // get the angle of two vector
        else {
            Eigen::RowVector3d norm_vector;
            norm_vector << (line_vector.cross(point_vector_1))/distance;

            point_line_distance.first = norm_vector.norm();
            point_line_distance.second = line.row(0) + line_vector * ( line_vector.dot(point_vector_1)/line_vector.dot(line_vector));
            return point_line_distance;
        }
    }
    //return point_line_distance;
} 


// 点在多边形内的投影点
pair<bool, RowVector3d> mapping_point_on_surface(RowVector3d& point, MatrixXd& surface) {
    Eigen::RowVector3d normal_vector_in_surface;
    Eigen::RowVector3d surface_vector_1, surface_vector_2;
    pair<bool, RowVector3d> mapping_point(true, Eigen::RowVector3d::Zero(1,3));

    surface_vector_1 << surface.row(1) - surface.row(0);
    surface_vector_2 << surface.row(2) - surface.row(1);
    normal_vector_in_surface << surface_vector_1.cross(surface_vector_2);  //法向量垂直面向里(逆时针定序)

    normal_vector_in_surface << normal_vector_in_surface.normalized();
    // 点减去向量在法向量上的投影向量
    mapping_point.second << point - normal_vector_in_surface * (point - surface.row(0)).dot(normal_vector_in_surface);

/************Test whether mapping_point on the surface********************/ 
    if (!point_whether_in_surface(point, surface)) {
        cout << "The point: "<< point << "have not legal mapping_point on surface: " << surface << endl;
        // mapping point is not on the surface
        mapping_point.first =  false;
       
    } 
    return mapping_point;

}

// if the return is "TRUE" means the point is in the surface
bool point_whether_in_surface(RowVector3d& point, MatrixXd& surface) {
    // the number of surface
    int surface_point_num = surface.cols();

    double dot_value;
    // center point in surface
    Eigen::RowVector3d center_point_in_surface, normal_vector_in_surface;    
    // two vector to get dot value 
    Eigen::RowVector3d dot_vector_1;
    // two vector of surface
    Eigen::RowVector3d surface_vector_1, surface_vector_2;
    
    center_point_in_surface << surface.colwise().mean();

    // the normal vector of surface [the surface_point_num >= 3]
    surface_vector_1 << surface.row(1) - surface.row(0);
    surface_vector_2 << surface.row(2) - surface.row(0);
    
    normal_vector_in_surface << surface_vector_1.cross(surface_vector_2);

    dot_vector_1 << center_point_in_surface - point;

    dot_value = dot_vector_1.dot(normal_vector_in_surface);

/************Test whether point on the surface********************/    
    //  cos(88 degree) ==  0.034899 (<= 2 degree)
    if (abs(dot_value) >  0.034899) {
        cout << "The point: "<< point << "is not on the surface: " << surface << endl;
        return false;
    }

    for (int i = 0; i < surface_point_num; i++) {
        Eigen::RowVector3d surface_point_1, surface_point_2;
        Eigen::RowVector3d surface_vector_1, surface_vector_2;

        surface_point_1 << surface.row(i);
        if (i == surface_point_num-1) {
            surface_point_2 << surface.row(0);
        }
        else surface_point_2 << surface.row(i+1);

        surface_vector_1 << (point - surface_point_1).cross(surface_point_2 - surface_point_1);
        surface_vector_2 << (center_point_in_surface - surface_point_1).cross(surface_point_2 - surface_point_1);

        dot_value = surface_vector_1.dot(surface_vector_2);
/************Test whether point in the surface vector********************/ 
        if (dot_value < 0) {
            cout  << "The point: "<< point << "is not in the surface vector: " << surface << endl;
            return false;
        }
    }
    return true;

}



// 将转换后的5*12矩阵作为上一层函数的拷贝值[深拷贝]
MatrixXd tran_obstacle_to_five_surface(MatrixXd& obstacle) {
    MatrixXd obstacle_five_surface(5, 12);
    for (int i = 0; i < 4; i++) {
        if (i == 3)
            obstacle_five_surface.row(i) << obstacle.row(3), obstacle.row(0), obstacle.row(4), obstacle.row(7);
        else 
            obstacle_five_surface.row(i) << obstacle.row(i), obstacle.row(1+i), obstacle.row(5+i), obstacle.row(4+i); 
    }
    obstacle_five_surface.row(4) << obstacle.row(4), obstacle.row(5), obstacle.row(6), obstacle.row(7);
}


//pair<double, Eigen::MatrixXd> line_to_line_distance(MatrixXd& line1, MatrixXd& line2) {
//    pair<double, Eigen::MatrixXd> line_line_distance(0, Eigen::MatrixXd::Zero(2, 3));
//
//    Eigen::RowVector3d line1_point_1, line1_point_2, line2_point_1, line2_point_2;
//    Eigen::RowVector3d line_vector_1, line_vector_2, cross_vector;
//
//    line1_point_1 << line1.row(0);
//    line1_point_2 << line1.row(1);
//    line2_point_1 << line2.row(0);
//    line2_point_2 << line2.row(1);
//
//    line_vector_1 << line1.row(1) - line1.row(0);
//    line_vector_2 << line2.row(1) - line2.row(0);
//
//    cross_vector << line1.row(0) - line2.row(0);
//
//    double a = line_vector_1.dot(line_vector_1);
//    double e = line_vector_2.dot(line_vector_2);
//    double f = line_vector_2.dot(cross_vector);
//
//    if (a <= 0.00001 && e <= 0.00001) {
//        Eigen::RowVector3d CPtSeg1;
//        CPtSeg1 << line1.row(0) - line2.row(0);
//        line_line_distance.first = CPtSeg1.norm();
//        line_line_distance.second.row(0) << line1.row(0);
//        line_line_distance.second.row(0) << line2.row(0);
//        std::cout << "line_line_distance.first :" << line_line_distance.first << endl;
//        std::cout << "line_line_distance.second :" << line_line_distance.second << endl;
//        return line_line_distance;
//    }
//    double s, t;
//
//    if (a <= 0.00001) {
//        s = 0;
//        t = f/e;
//        t = clamp(t, 0, 1);
//    }
//    else {
//        double c = line_vector_1.dot(cross_vector);
//        if (e <= 0.00001) {
//            t = 0;
//            s = clamp(-c/a, 0, 1);
//        }
//        else {
//            double b = line_vector_1.dot(line_vector_2);
//            double denom = a*e - b*b;
//
//            if (denom != 0)
//                s = clamp((b*f - c*e)/denom, 0, 1);
//            else
//                s = 0;
//
//            t = (b*s + f)/e;
//
//            if (t < 0) {
//                t = 0;
//                s = clamp(-c/a, 0, 1);
//            }
//            else if (t > 1) {
//                t = 0;
//                s = clamp((b-c)/a, 0, 1);
//            }
//        }
//    }
//    line_line_distance.second.row(0) << line1.row(0) + line_vector_1 * s;
//    line_line_distance.second.row(1) << line2.row(0) + line_vector_2 * t;
//    line_line_distance.first = (line_line_distance.second.row(0) - line_line_distance.second.row(1) ).norm();
//    std::cout << "line_line_distance.first :" << line_line_distance.first << endl;
//    std::cout << "line_line_distance.second :" << line_line_distance.second << endl;
//    return line_line_distance;
//}
//
double clamp(double n, double min, double max) {
    double N;
    if (n < min) {
        N = min;
        return N;
    }
    else if (n > max) {
        N = max;
        return N;
    }
    else {
        N = n;
        return N;
    }
}