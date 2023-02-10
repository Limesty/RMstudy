/*
这里写了两个预处理的函数，分别使用rgb通道和hsv通道
hsv进行预处理的效果测试时效果感觉要比rgb要更好一些。
*/

#include "PreTreat.hpp"

PreTreat::PreTreat()
{
}

PreTreat::~PreTreat()
{
}

void PreTreat::setEnemyColor(Color color){
    enemyColor = color;
}
static void imagePreProcess(Mat &src)//开闭运算
{
    static Mat kernel_dilate2 = getStructuringElement(MORPH_RECT, Size(5,5));
    dilate(src, src, kernel_dilate2);

    static Mat kernel_erode2 = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(src, src, kernel_erode2);

    static Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(src, src, kernel_erode);

    static Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(src, src, kernel_dilate);

}

/*
    本想用红蓝通道相减得到的差作为识别用的灰度图，因为在低曝光度下除灯条外
其余位置红蓝像素值差别不大，因此相减后就只剩下灯条处像素值不为零,但是实践中
因为灯条中心几乎为白光，红蓝光强度大致相当，导致二值化后的灯条中心有一个空
洞，因此这个方法似乎不太适用,于是就按照去年的方法简单改了一下。
*/

Mat PreTreat::run_rgb(Mat &src)//rgb通道预处理
{
    Mat color_channel;         //保存enermycolor对应颜色的通道
    Mat src_bin_light;         //保存预处理后的图象
    std::vector<Mat> channels; //存储通道拆分后的通道
    // Mat red_blue,blue_red;           //分别存储red通道减去blue通道与blue-red后的图像
    split(src, channels);
    // red_blue = channels[2] - channels[0];
    // blue_red = channels[0] - channels[2];
    // imwrite("red.jpg",red_blue);
    // imwrite("blue.jpg",blue_red);

    if (enemyColor == BLUE)
    {
        color_channel = channels[0];
    }
    else if (enemyColor == RED)
    {
        color_channel = channels[2];
    }

    int light_threshold = 200;//二值化阈值

    threshold(color_channel, src_bin_light, light_threshold, 255, THRESH_BINARY); // 二值化对应通道
    if (src_bin_light.empty())
        flag = 0;
    else
        flag = 1;
    imagePreProcess(src_bin_light); //两次开闭运算除去图像中的毛刺或空洞
    return src_bin_light;
}

Mat PreTreat::run_hsv(Mat &src)//hsv通道预处理
{
    cvtColor(src,src,COLOR_BGR2HSV);
    //设置HSV色彩空间各个分量的范围，Hue：色调，Saturation：饱和度，Value：明度
    Scalar red1_lower = Scalar(0,0,200);
    Scalar red1_upper = Scalar(25,255,255);
    Scalar red2_lower = Scalar(155,0,200);
    Scalar red2_upper = Scalar(180,255,255);
    Scalar blue_lower = Scalar(60,0,200);
    Scalar blue_upper = Scalar(120,255,255);
    Mat src_bin_light;//掩膜，储存二值化后的图像
    if (enemyColor == BLUE)
    {
        cv::inRange(src, blue_lower, blue_upper, src_bin_light);
    }
    else if (enemyColor == RED)
    {
        Mat red1,red2;
        cv::inRange(src, red1_lower, red1_upper, red1);
        cv::inRange(src, red2_lower, red2_upper, red2);
        src_bin_light = red1 + red2;
    }  
    if(src_bin_light.empty()) flag = 0;
    else flag = 1;
    imagePreProcess(src_bin_light); //两次开闭运算除去图像中的毛刺或空洞
    return src_bin_light;
}
