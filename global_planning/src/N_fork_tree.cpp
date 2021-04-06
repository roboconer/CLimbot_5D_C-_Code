//
// Created by kiko on 2021/4/4.
//

#include <iostream>
#include <vector>
#include<fstream>
#include "stdlib.h"
#include "Eigen/Dense"
//#include "../config/global_variable.h"

using namespace std;

#define M 100+1

//定义树的结构
typedef struct node_t{
    int	name;					//节点名
    int		n_children;				//子节点个数
    int		level;					//记录层数
    struct  node_t** children;		//指向其自身的子节点，children一个数组，该数组中的元素时node_t*指针
}NODE;

//实现栈
typedef struct stack_t {
    NODE**  arry;	//arry为数组，其元素为NODE*指针
    int		index;	//栈顶
    int		size;   //栈的大小
}STACK;

//实现队列
typedef struct queue_t {
    NODE**  arry;	//arry为数组，其元素为NODE*指针
    int		head;	//队列的头
    int		tail;	//队列的尾
    int		num;	//队列中的元素
    int		size;	//队列的大小
}QUEUE;

//内存分配函数
void* util_malloc(int size) {
    void* ptr = malloc(size);

    if (ptr == NULL) {
        cout << "内存分配失败" << endl;
        exit(EXIT_FAILURE);
    }

    //分配成功
    return ptr;
}

//实现栈的操作##########################################################################

//栈的初始化
STACK* STACKinit(int size) //初始化
{
    STACK* sp;

    sp = (STACK*)util_malloc(sizeof(STACK));
    sp->size	= size;
    sp->index	= 0;
    sp->arry = (NODE**)util_malloc(size * sizeof(NODE*));

    return sp;
}

// 检测栈是否为空
// 如果为空返回1，否则返回0
int STACKempty(STACK* sp) {
    if (sp == NULL || sp->index <= 0)
        return 1;
    return 0;
}

//压栈操作
int STACKpush(STACK* sp, NODE* data) {
    if (sp == NULL || sp->index == sp->size) //栈没被初始化或者已满
    {
        return 0;
    }

    sp->arry[sp->index++] = data;
    return 1;
}

//弹栈操作
int STACKpop(STACK* sp, NODE** data_ptr) {
    if (sp == NULL || sp->index == 0) //栈没被初始化或者已空
    {
        return 0;
    }

    *data_ptr = sp->arry[--sp->index]; //弹栈
    return 1;
}

//将栈销毁
void STACKdestory(STACK* sp) {
    free(sp->arry);
    free(sp);
}

//实现队列的操作##########################################################################

//队列的初始化
QUEUE* QUEUEinit(int size) {
    QUEUE* qp;

    qp = (QUEUE*)util_malloc(sizeof(QUEUE));
    qp->size = size;
    qp->head = qp->tail = qp->num = 0;
    qp->arry = (NODE**)util_malloc(size * sizeof(NODE*));

    return qp;
}

//入队列
int QUEUEenqueue(QUEUE* qp, NODE* data) {
    if (qp == NULL || qp->num >= qp->size) //qp未初始化或队列内已满
    {
        return 0;
    }

    qp->arry[qp->tail] = data;// 入队，tail一直指向最后一个元素的下一个位置
    qp->tail = (qp->tail + 1) % (qp->size); //循环队列
    ++qp->num;
    return 1;
}

//出队列
int QUEUEdequeue(QUEUE* qp, NODE** data_ptr) {
    if (qp == NULL || qp->num <= 0) //qp未初始化或队列内无元素
    {
        return 0;
    }

    *data_ptr = qp->arry[qp->head]; //出队
    qp->head = (qp->head + 1) % (qp->size); //循环队列
    --qp->num;

    return 1;
}

//检测队列是否为空
int QUEUEempty(QUEUE* qp) {
    if (qp == NULL || qp->num <= 0) {
        return 1;
    }

    return 0;
}

//销毁队列
void QUEUEdestory(QUEUE* qp) {
    free(qp->arry);
    free(qp);
}

//生成多叉树节点
NODE* create_node() {
    NODE* q;

    q = (NODE*)util_malloc(sizeof(NODE));
    q->n_children	= 0;
    q->level		= -1;
    q->children		= NULL;

    return q;
}

//按节点名字查找
vector<NODE*> search_node_r(int name, NODE* head) {
    vector<NODE*> temp(4,NULL);
    int i = 0;

    if (head != NULL)
    {
        if (name == head->name) {//如果名字匹配
            temp.push_back(head);
        }
        else
        {
            for ( i = 0; i < head->n_children && temp[0]==NULL; i++) //temp未找到时继续查找
            {
                temp = search_node_r(name, head->children[i]);
            }
        }
    }

    return temp; //返回指针temp，也可能为空
}

//从文件中读取多叉树数据，并建立多叉树
//void read_file(NODE** head,char text_name[M]) {
//    NODE* temp = NULL;
//    int i = 0,num;
//    char read_data[M] = { 0 };
//    char data;
//
//    ifstream input;
//    input.open(text_name);
//
//    //读取文件
//    while (1) {
//        input >> data;
//        if (data == '(')
//        {
//            input >> data;
//            if (data == '#') break;
//            while (data != ')')
//            {
//                if (data != ',') {							//过滤文件内容
//                    read_data[i]=data;
//                    i++;
//                }
//                input >> data;
//            }
//            input >> data;
//        }
//    }
//
//    //处理读取的文件内容
//    i = 0;
//    while (read_data[i] != 0)
//    {
//        if (*head == NULL) {
//            temp = *head = create_node();		// 生成一个新节点
//            temp->name = read_data[i];	    // 赋名
//        }
//        else
//        {
//            temp = search_node_r(read_data[i], *head);	// 根据name找到节点
//        }
//        //找到节点后，对子节点进行处理
//        num = read_data[i+1]-48;
//        temp->n_children = num; //因为读入第二位是数字,即当前子节点个数
//        temp->children = (NODE**)malloc(num * sizeof(NODE*));
//        if (temp->children == NULL) //内存分配失败
//        {
//            cout << "内存分配失败" << endl;
//            exit(EXIT_FAILURE);
//        }
//
//        // 如果分配成功，则读取后面的子节点，并保存
//        for ( int n = 0; n <num; n++)
//        {
//            temp->children[n] = create_node();
//            temp->children[n]->name = read_data[i + 2 + n];
//        }
//
//        //读取下一批
//        i = i + 2 + num;
//    }
//
//    //读取完毕
//    input.close();
//
//}


// 输入数据
void read_tree_data(int root_node, NODE** head, Eigen::MatrixXi& transition_possibility_adjacency_matrix) {

    vector<NODE*> temp(transition_possibility_adjacency_matrix.rows(),NULL);

    for (int i = 0; i < transition_possibility_adjacency_matrix.rows(); i++) {
        // 建立头节点
        if (*head == NULL) {
            temp.push_back(*head = create_node());        // 生成一个新节点
            NODE* store = {NULL};
            store->name = root_node;
            temp.push_back(store);        // 赋名
        }
        else
        {
            temp = search_node_r(i, *head);	// 根据name找到节点
        }
    //找到节点后，对子节点进行处理
        // 获得每一行的有效值的个数
        int legal_row_num = 0;
        vector<int> legal_wall_serial;
        for (int j = 0; j < transition_possibility_adjacency_matrix.cols(); j++) {
            if (transition_possibility_adjacency_matrix(i, j) != INT_MAX && transition_possibility_adjacency_matrix(i, j) != 0) {
                legal_row_num++;
                legal_wall_serial.push_back(j);
            }
        }
        // 给子节点分配格式内存
        for (int k = 0; k < temp.size(); k++) {
            temp[k]->n_children = legal_row_num;
            temp[k]->children = (NODE **) malloc(legal_row_num * sizeof(NODE *));

            if (temp[k]->children == NULL) {//内存分配失败
                cout << "内存分配失败" << endl;
                exit(EXIT_FAILURE);
            }

            // 如果分配成功，则读取后面的子节点，并保存
            for (int n = 0; n < legal_row_num; n++) {
                temp[k]->children[n] = create_node();
                temp[k]->children[n]->name = legal_wall_serial[n];
            }
        }

    }
}


//输出树的深度结构
void output_tree(NODE* head) {
    NODE* p = NULL;
    QUEUE* q = NULL;
    STACK* s = NULL;
    int i = 0;

    q = QUEUEinit(100);  //初始化队列大小为100
    s = STACKinit(100);  //初始化栈的大小为100

    head->level = 0;  //设置根节点深度

    //根节点入队列
    QUEUEenqueue(q, head);

    // 对多叉树中的节点的深度值level进行赋值
    // 采用层次优先遍历方法，借助于队列
    while (QUEUEempty(q) == 0)
    {
        QUEUEdequeue(q, &p); //出队列
        for ( i = 0; i < p->n_children; i++)
        {
            p->children[i]->level = p->level + 1;
            QUEUEenqueue(q, p->children[i]);
        }
        STACKpush(s, p);
    }

    cout << "节点名称    深度" << endl;

    while (STACKempty(s) == 0)
    {
        STACKpop(s, &p);
        cout << p->name <<"            "<< p->level << endl;
    }

    QUEUEdestory(q); // 消毁队列
    STACKdestory(s); // 消毁栈
}

//输出树的结构
void output_tree_fix(NODE* head) {
    cout << head->name;
    if (head->n_children != 0)
    {
        cout << "(";
        for (int i = 0; i < head->n_children; i++)
        {
            if (i == head->n_children - 1) {
                output_tree_fix(head->children[i]);
                break;
            }
            output_tree_fix(head->children[i]);
            cout << ",";
        }
        cout << ")";
    }

}

int main()
{
    NODE* head = NULL;
    char* strBest = NULL;
    char text_name[M];

    cout << "请输入读取的目标文件:";
    //cin >> text_name;
    Eigen::MatrixXi transition_possibility_adjacency_matrix(4, 4);

    transition_possibility_adjacency_matrix << 0, 1, 1, 1,
                                               1, 0, 1, 1,
                                               1, 1, 0, 0,
                                               1, 1, 0, 0;
    //read_file(&head,"tobuy.txt");
    cout << "dsdsdsdsdsdsd" << endl;
    read_tree_data(0, &head, transition_possibility_adjacency_matrix);
    cout << "desddwqew" << endl;
    output_tree(head);

    cout << "\n输出树的结构" << endl;
    output_tree_fix(head);

    return 0;
}
