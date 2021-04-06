//
// Created by kiko on 2021/4/6.
//

#ifndef BASE_LIB_CODE_H
#define BASE_LIB_CODE_H

// function-1 : 壁面收缩函数[按照收缩系数，对壁面的顶点往内方向收缩]
// INPUT:  vector<MatrixXd> WALL_BORDER -- 壁面的源数据
// OUTPUT: 无
// 将收缩后的壁面数据存入全局变量：vector<MatrixXd> SHRINK_WALL_BORDER
void shrink_surface(vector<MatrixXd>& WALL_BORDER);

//function-2 : 对象旋转函数[将壁面数据按照旋转轴旋转一定的角度]
// INPUT: char axis: 旋转轴(x|y|z); int angle: 旋转角度(deg); MatrixXd object: 旋转对象(每一行都为旋转对象)
// OUTPUT: 无
void axis_rotate_angle(char axis, int angle, MatrixXd& object);


#endif //BASE_LIB_CODE_H
