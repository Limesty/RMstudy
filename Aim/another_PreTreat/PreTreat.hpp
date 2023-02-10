#ifndef PRE_TREAT
#define PRE_TREAT

#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>

using namespace cv;
using namespace std;


///颜色：B蓝 G绿 R红,NONE：未设置颜色
enum Color
{
    BLUE = 0,
    GREEN = 1,
    RED = 2
};

//用于预处理，将图像中的灯条凸显出来
class PreTreat
{
public:
	PreTreat();
	~PreTreat();
	void setEnemyColor(Color enemyColor);// 设置敌方颜色
	Mat run_rgb(Mat & src);//预处理函数
    Mat run_hsv(Mat & src);
    bool flag = 0; //显示预处理后的图像是否为空，1表示不为空
private:
	Color enemyColor = RED;  //敌方颜色
};


#endif // ARMOR_H
