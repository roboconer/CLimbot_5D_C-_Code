#include "../config/global_variable.h"
#include "header/base_lib_code.h"

//1、******************壁面收缩函数
void shrink_surface(vector<MatrixXd>& WALL_BORDER) {

	for (int i = 0; i < WALL_BORDER.size(); i++) {

		int point_nums = WALL_BORDER[i].rows(); 
		// 壁面两相交向量，夹角(Vector3d默认是列向量)Eigen::RowVector3d
		Eigen::RowVector3d vec_1, vec_2, vm; 
		double vec_cos, vec_sin = 0;
		double shrink_rate = 0;

		vec_1 << (WALL_BORDER[i].row(point_nums-1) - WALL_BORDER[i].row(0)).normalized();		
		vec_2 << (WALL_BORDER[i].row(1) - WALL_BORDER[i].row(0)).normalized();

		vec_cos = vec_1.dot(vec_2);
		vec_sin = sqrt(1 - vec_cos*vec_cos);

		// 壁面法向量
		vm = vec_2.cross(vec_1);
		NORMAL_VECTOR_WALL[i] = vm;
		
		cout << "NORMAL_VECTOR_WALL["<< i << "] =" << NORMAL_VECTOR_WALL[i] << endl;
		// 收缩比例
		shrink_rate = SUCTION_CUP_RADIUS/vec_sin;
		// 收缩后的第一边
		SHRINK_WALL_BORDER[i].row(0) << WALL_BORDER[i].row(0) + vec_1*shrink_rate + vec_2*shrink_rate;

		// // 收缩后的最后一边
		vec_1 << (WALL_BORDER[i].row(0) - WALL_BORDER[i].row(point_nums-1)).normalized();
		vec_2 << (WALL_BORDER[i].row(point_nums-2) - WALL_BORDER[i].row(point_nums-1)).normalized();
		vec_cos = vec_1.dot(vec_2);
		vec_sin = sqrt(1 - vec_cos*vec_cos);
		shrink_rate = SUCTION_CUP_RADIUS/vec_sin;
		SHRINK_WALL_BORDER[i].row(point_nums-1) = WALL_BORDER[i].row(point_nums-1) + vec_1*shrink_rate + vec_2*shrink_rate;

		// 中间的边
		for (int j = 1; j < point_nums-1; j++) {
			vec_1 << (WALL_BORDER[i].row(j+1) - WALL_BORDER[i].row(j)).normalized();
			vec_2 << (WALL_BORDER[i].row(j-1) - WALL_BORDER[i].row(j)).normalized();
			vec_cos = vec_1.dot(vec_2);
			vec_sin = sqrt(1 - vec_cos*vec_cos);
			shrink_rate = SUCTION_CUP_RADIUS/vec_sin;
			SHRINK_WALL_BORDER[i].row(j) = WALL_BORDER[i].row(j) + vec_1*shrink_rate + vec_2*shrink_rate;
		}
		//cout <<"SHRINK_WALL_BORDER["<< i <<"] = " << SHRINK_WALL_BORDER[i] << endl;
	}

	// 收缩壁面后的起始点和壁面结束点
	// 对初始壁面和结束壁面求平均行向量，作为壁面起始点和壁面结束点
    START_POINT << SHRINK_WALL_BORDER[START_AND_END_WALL[0]].colwise().mean();
    END_POINT << SHRINK_WALL_BORDER[START_AND_END_WALL[1]].colwise().mean();

}
//**************************************************


//2、******************对象旋转函数
void axis_rotate_angle(char axis, int angle, MatrixXd& object) {
	
    // 旋转矩阵
	Eigen::Matrix3d rotate_matrix;

    if (axis == 'x') {
		// 旋转向量
		AngleAxisd Vx(M_PI/180*angle, Vector3d(1, 0, 0));
		rotate_matrix = Vx.matrix();       
    }
    else if (axis == 'y') {
        AngleAxisd Vy(M_PI/180*angle, Vector3d(0, 1, 0));
		rotate_matrix = Vy.matrix();
    }
    else if (axis == 'z') {
        AngleAxisd Vz(M_PI/180*angle, Vector3d(0, 0, 1));
		rotate_matrix = Vz.matrix();
    }
    else return;

    // 目标*旋转矩阵 = 旋转后的目标
   
    object = object * rotate_matrix;

}


