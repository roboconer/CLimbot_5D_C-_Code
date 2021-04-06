#include "../config/read_config.h"
#include "../config/global_variable.h"
#include "header/global_planning.h"
#include "header/base_lib_code.h"


// 壁面数据输入
// 

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

    for (int i = 0; i < NORMAL_VECTOR_WALL.size(); i++)
    cout << "NORMAL_VECTOR_WALL: " << NORMAL_VECTOR_WALL[i] << endl;


    return 0;

}







