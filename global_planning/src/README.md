## 记录问题

### 1、在进行壁面可过渡性判断的时候，需要
    1-1 面跟面的可过渡性是根据两个面的最近距离点对，对每个点分别建立机器人可达工作空间
    1-2 点对的获得(点对其中有一点必在边线上)：分别遍历两个壁面(N.M条边)，获得 N*M种可能的点对
    1-3 对每种可能的点对，(两个点)分别建立可达工作空间，判断另外一点是否在工作空间内
    1-4 找到一个点对符合，立即退出
    两个面之间进行搜索的时间复杂度：O(N*M*2)
    面与面(K个面)之间的搜索的时间复杂度：O(K*(K-1)/2)

    这样的时间复杂度过高  

TIME : 2021/3/20

### 2、坐标系转换
- 在壁面的数据上建立的壁面坐标系R_{tran_surface_to_world} -- Matrix3d(Vx',Vy',Vz'),是该壁面坐标系在世界坐标系下的表示
- 而要求世界坐标系下的点P_{wrold}在壁面坐标系下的表示: P_{world_in_surface}
 P_{wrold} =  R_{tran_surface_to_world} * P_{world_in_surface}
- 则：P_{world_in_surface} = (R_{tran_surface_to_world})^(-1) * P_{wrold}

TIME : 2021/3/21

### 3、code review
#### (1)了解gtest，给每个cpp都建立一个gtest
#### (2) github文档在网址github后面加上'1s'，在线上编译器打开
    如:"https://github.com/roboconer/DATA_ANALYSIS/blob/main/Titanic/titanic_DecisionTree.py"
    用这个："https://github1s.com/roboconer/DATA_ANALYSIS/blob/main/Titanic/titanic_DecisionTree.py"
#### (3) 针对面向对象，进行编写，不要写成C project。

TIME ：2021/4/6
