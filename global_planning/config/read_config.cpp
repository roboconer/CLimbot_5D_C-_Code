#include "read_config.h"
#include "../src/header/global_planning.h"  // 自己写的include文件要用" "


void read_global_variable_config() {
//int main () {

    // open the config.ini
    ifstream global_planning_config;
    /*******************NOTE**********************/
    /*这里的config.ini路径，如果是单文件编译，若路径与当前文件一致，则直接是"config.ini" */
    /*如果是cmake编译，则要根据cmake的路径，进行修改。这里的"..config.ini"，就是因为cmake路径在config.ini文件的子目录build下 */
     /*******************NOTE**********************/
    global_planning_config.open("../config/config.ini"); 

    // store the cache number(such as: wall point number)
    int cache_number;
    int cache_wall_number;
  
    // read the config.ini
    if(global_planning_config.is_open()) {  

        cout << "# Start reading the config:..." <<endl;
        string cache_string;

        // 循环读取文件的每一行
        while (getline(global_planning_config, cache_string)) {

            // if is blank line or note(#), skin this line
            if (cache_string == "" || cache_string[0] == '#') {         
                continue;
            }

            else {
                
                // read the suction cup radius
                if ( string(cache_string.begin(), cache_string.begin()+18) == "SUCTION_CUP_RADIUS") {  
                    
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = ";

                    // read the string before '#' but after '=' [data]                    
                    // use the stringstream to translate string to double
                    ss >> SUCTION_CUP_RADIUS;
                    cout << SUCTION_CUP_RADIUS  << endl;
                    continue; 
                            
                }
                // read the TOTAL_NUM_OF_WALL
                else if (string(cache_string.begin(), cache_string.begin()+17) == "TOTAL_NUM_OF_WALL") {
                    
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = ";

                    // read the string before '#' but after '=' [data]
                    ss >> TOTAL_NUM_OF_WALL;
                    cout << TOTAL_NUM_OF_WALL  << endl;

                    cache_wall_number = TOTAL_NUM_OF_WALL;   
                    continue;      
                }

            //***************************************************FIRST WALL    
                // read the WALL_BORDER_POINT_NUM
                else if (string(cache_string.begin(), cache_string.begin()+23) == "WALL_BORDER_POINT_NUM_1") {
                    
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = ";

                    // read the string before '#' but after '=' [data]
                    ss >> cache_number;
                    cout << cache_number << endl;     
                    continue;
                }
                // read the first wall data 
                else if (string(cache_string.begin(), cache_string.begin()+13) == "WALL_BORDER_1") {
                    
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = " << endl;

                    // read the string before '#' but after '=' [data]
                    // read the first line
                    ss >> WALL_BORDER_1(0,0);
                    ss >> WALL_BORDER_1(0,1);
                    ss >> WALL_BORDER_1(0,2);                    
                    cache_number--;
                    // read the last three line
                    while (cache_number--) {

                        getline(global_planning_config, cache_string);

                        stringstream ss(cache_string);
                    
                        for (int i = 0; i < 3; i++) {
                            ss >> WALL_BORDER_1(3-cache_number, i);
                        }                                 
                    }
                    // push the wall data into WALL_BORDER
                    //cout <<  "WALL_BORDER_ 1: " << WALL_BORDER_1 << endl;
                    WALL_BORDER.push_back(WALL_BORDER_1);
                    cout << WALL_BORDER[0] << endl;
                    continue;
                }

            //*************************************************SECOND WALL
                else if (string(cache_string.begin(), cache_string.begin()+23) == "WALL_BORDER_POINT_NUM_2") {
                    
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = ";

                    // read the string before '#' but after '=' [data]
                    ss >> cache_number;
                    cout << cache_number << endl;     
                    continue;
                }
                // read the first wall data 
                else if (string(cache_string.begin(), cache_string.begin()+13) == "WALL_BORDER_2") {
                    
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = " << endl;

                    // read the string before '#' but after '=' [data]
                    // read the first line
                    ss >> WALL_BORDER_2(0,0);
                    ss >> WALL_BORDER_2(0,1);
                    ss >> WALL_BORDER_2(0,2);
                    cache_number--;
                    // read the last three line
                    while (cache_number--) {

                        getline(global_planning_config, cache_string);

                        stringstream ss(cache_string);
                    
                        for (int i = 0; i < 3; i++) {
                            ss >> WALL_BORDER_2(3-cache_number, i);
                        }                                 
                    }
                    // push the wall data into WALL_BORDER
                    WALL_BORDER.push_back(WALL_BORDER_2);
                    cout << WALL_BORDER[1] << endl;
                    continue;
                }

            //*************************************************THIRD WALL
                else if (string(cache_string.begin(), cache_string.begin()+23) == "WALL_BORDER_POINT_NUM_3") {

                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name]
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = ";

                    // read the string before '#' but after '=' [data]
                    ss >> cache_number;
                    cout << cache_number << endl;
                    continue;
                }
                    // read the first wall data
                else if (string(cache_string.begin(), cache_string.begin()+13) == "WALL_BORDER_3") {

                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name]
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = " << endl;

                    // read the string before '#' but after '=' [data]
                    // read the first line
                    ss >> WALL_BORDER_3(0,0);
                    ss >> WALL_BORDER_3(0,1);
                    ss >> WALL_BORDER_3(0,2);
                    cache_number--;
                    // read the last three line
                    while (cache_number--) {

                        getline(global_planning_config, cache_string);

                        stringstream ss(cache_string);

                        for (int i = 0; i < 3; i++) {
                            ss >> WALL_BORDER_3(3-cache_number, i);
                        }
                    }
                    // push the wall data into WALL_BORDER
                    WALL_BORDER.push_back(WALL_BORDER_3);
                    cout << WALL_BORDER[2] << endl;
                    continue;
                }

            //*************************************************FOURTH WALL
                else if (string(cache_string.begin(), cache_string.begin()+23) == "WALL_BORDER_POINT_NUM_4") {
                    
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = ";

                    // read the string before '#' but after '=' [data]
                    ss >> cache_number;
                    cout << cache_number << endl;     
                    continue;
                }
                // read the first wall data 
                else if (string(cache_string.begin(), cache_string.begin()+13) == "WALL_BORDER_4") {
                    
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = " << endl;

                    // read the string before '#' but after '=' [data]
                    // read the first line
                    ss >> WALL_BORDER_4(0,0);
                    ss >> WALL_BORDER_4(0,1);
                    ss >> WALL_BORDER_4(0,2);
                    cache_number--;
                    // read the last three line
                    while (cache_number--) {

                        getline(global_planning_config, cache_string);

                        stringstream ss(cache_string);
                    
                        for (int i = 0; i < 3; i++) {
                            ss >> WALL_BORDER_4(3-cache_number, i);
                        }                                 
                    }
                    // push the wall data into WALL_BORDER
                    WALL_BORDER.push_back(WALL_BORDER_4);
                    cout << WALL_BORDER[3] << endl;
                    continue;
                }

            //*************************************************read the num of obstacle
                else if (string(cache_string.begin(), cache_string.begin()+12) == "OBSTACLE_NUM") {
                        stringstream ss(cache_string);
                        string cache_name, cache_data;

                        // read the string before '=' [name]
                        getline(ss, cache_name, '=');
                        cout << "Reading the "<< cache_name << " = ";

                        // read the string before '#' but after '=' [data]
                        ss >> OBSTACLE_NUM;
                        cout << OBSTACLE_NUM << endl;
                        continue;

                }

            //*************************************************read the first obstacle
                // read the OBSTACLE_PLACE_WALL_1
                else if (string(cache_string.begin(), cache_string.begin()+21) == "OBSTACLE_PLACE_WALL_1") {
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = ";

                    // read the string before '#' but after '=' [data]
                    ss >> OBSTACLE_PLACE_WALL_1;
                    cout << OBSTACLE_PLACE_WALL_1 << endl;

                    // push the OBSTACLE_PLACE_WALL_1 into OBSTACLE_1
                    OBSTACLE_1.first = OBSTACLE_PLACE_WALL_1;     
                    continue;    
                }
                // read the OBSTACLE_BORDER_1
                else if (string(cache_string.begin(), cache_string.begin()+17) == "OBSTACLE_BORDER_1") {
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = " << endl;

                    // read the string before '#' but after '=' [data]
                    // read the first line
                    ss >> OBSTACLE_BORDER_1(0,0);
                    ss >> OBSTACLE_BORDER_1(0,1);
                    cache_number = 3;
                    // read the last three line
                    while (cache_number--) {

                        getline(global_planning_config, cache_string);

                        stringstream ss(cache_string);
                    
                        for (int i = 0; i < 2; i++) {
                            ss >> OBSTACLE_BORDER_1(3-cache_number, i);
                        }                                 
                    }
                    // push the wall data into WALL_BORDER
                    OBSTACLE_1.second = OBSTACLE_BORDER_1;
                    cout << OBSTACLE_1.second << endl;
                    continue;
                }
                
            //*************************************************read the ROBOT Climbot_5D CONFIG
                // read the ROBOT_CLIMBOT_5D_LENGTH   
                else if (string(cache_string.begin(), cache_string.begin()+23) == "ROBOT_CLIMBOT_5D_LENGTH") {
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = " << endl;

                    // read the string before '#' but after '=' [data]
                    // read the first line
                    cache_number = 6;
                                         
                    // read and store the data
                    for (int j = 0; j < cache_number; j++) {
                        ss >> ROBOT_CLIMBOT_5D_LENGTH[j];
                         cout <<  ROBOT_CLIMBOT_5D_LENGTH[j] << " ";                              
                    }
                    cout << endl;                        
                }

                // read the ROBOT_JOINT_MIN_ANGLE
                else if (string(cache_string.begin(), cache_string.begin()+21) == "ROBOT_JOINT_MIN_ANGLE") {
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = " << endl;

                    // read the string before '#' but after '=' [data]
                    // read the first line
                    cache_number = 5;
                                         
                    // read and store the data
                    for (int j = 0; j < cache_number; j++) {
                        ss >> ROBOT_JOINT_MIN_ANGLE[j];
                         cout << ROBOT_JOINT_MIN_ANGLE[j] << " ";                              
                    }
                    cout << endl;                        
                }

                // read the ROBOT_JOINT_MAX_ANGLE
                else if (string(cache_string.begin(), cache_string.begin()+21) == "ROBOT_JOINT_MAX_ANGLE") {
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = " << endl;

                    // read the string before '#' but after '=' [data]
                    // read the first line
                    cache_number = 5;
                                         
                    // read and store the data
                    for (int j = 0; j < cache_number; j++) {
                        ss >> ROBOT_JOINT_MAX_ANGLE[j];
                         cout << ROBOT_JOINT_MAX_ANGLE[j] << " ";                              
                    }
                    cout << endl;                        
                }

            //*************************************************read the planning start and end wall
                else if (string(cache_string.begin(), cache_string.begin()+23) == "START_END_PLANNING_WALL") {
                    stringstream ss(cache_string);
                    string cache_name, cache_data;

                    // read the string before '=' [name] 
                    getline(ss, cache_name, '=');
                    cout << "Reading the "<< cache_name << " = " << endl;

                    // read the string before '#' but after '=' [data]                                         
                    // read and store the data
                    ss >> START_AND_END_WALL[0];
                    ss >> START_AND_END_WALL[1];
                    cout <<START_AND_END_WALL[0] << ","  << START_AND_END_WALL[1] << endl;                        
                }    
            
            // ERROR    
                else {
                    cout << "The config.ini may have different config with the config.cpp.\nPlease check it and try again." << endl;
                }
            
            }
            
        }
    
    }
   
   else cout << "Open the config.ini failed." << endl;
    // return 0;
}


