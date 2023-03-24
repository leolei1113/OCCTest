//    A      A    A
//    |      |    |
//    B--------C
//    |      |    |
//    D      D    D

#include <iostream>

using namespace std;

#define MAXV 4
#define MAXE 7

//邻接矩阵表示图
typedef struct {
    int vexs[MAXV]; //存储顶点的数组
    int arcs[MAXV][MAXV]; //存储边的二维数组
    int vexnum, arcnum; //记录顶点数和边数
}Graph;

//初始化图
void init_graph(Graph& G, int vexs[], int arcs[][MAXV], int v, int e) {
    int i, j;
    G.vexnum = v; //顶点数赋值
    G.arcnum = e; //边数赋值
    for (i = 0; i < G.vexnum; i++) {
        G.vexs[i] = vexs[i]; //存储顶点
        for (j = 0; j < G.vexnum; j++)
            G.arcs[i][j] = arcs[i][j]; //存储边
    }
}

//求欧拉路径
void get_euler_path(Graph G) 
{
    int odd_cnt = 0; //记录度数为奇数的顶点数
    int i, j, n, path_idx = 0;
    int path[MAXE]; //存储欧拉路径

    //确定度数为奇数的顶点数
    for (i = 0; i < G.vexnum; i++) 
    {
        n = 0;
        for (j = 0; j < G.vexnum; j++)
            n += G.arcs[i][j];
        if (n % 2 != 0)
            odd_cnt++;
    }

    //无欧拉路径，返回
    if (odd_cnt != 0 && odd_cnt != 2)
    {
        cout << "无欧拉路径" << endl;
        return;
    }

    //开始求欧拉路径
    i = 0; //设置起点
    while (i < G.vexnum) {
        for (j = 0; j < G.vexnum; j++)
            if (G.arcs[i][j] > 0) //找到一条边
                break;
        if (j < G.vexnum) { //有路径，继续寻找
            path[path_idx++] = i; //存储路径
            G.arcs[i][j]--; //删除此边
            G.arcs[j][i]--;
            i = j; //继续从j出发
        }
        else //无路径
            i = path[--path_idx]; //返回上一条路径
    }

    //输出欧拉路径
    for (i = 0; i < path_idx; i++) {
        cout << G.vexs[path[i]] << "->";
    }
    cout << G.vexs[path[path_idx]] << endl;
}

int main()
{
    int vexs[] = { 1,2,3,4 }; //定义顶点数组
    int arcs[][MAXV] = {
        {0,1,1,1},
        {1,0,1,0},
        {1,1,0,1},
        {1,0,1,0}
    }; //定义邻接矩阵
    Graph G;
    init_graph(G, vexs, arcs, 4, 7); //初始化图
    get_euler_path(G); //求欧拉路径
    return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
