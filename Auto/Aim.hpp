#include <opencv4/opencv2/opencv.hpp>
#include "Mapping.hpp"
#include <vector>

using namespace std;

// 这个类主要用于哨兵的自瞄，很多地方基于我自己的
// 思考，如果有理解不正确的还望指正

class Aim
{
public:
    // 首先，我们需要将视觉相机所得到的图片进行解析，
    // 返回我们需要瞄准的一系列目标
    // 疑惑的地方：这里的信息应该有Location来解析，如果Location已经
    // 解析好了我们要瞄准的目标的话，那么我们就修改传入函数的参数，
    // 输入函数的参数就改为地图的信息即可
    vector<MAPPING::pos> get_the_attack_loc(cv::Mat input_img);

    // 需要根据当前一系列的备选的目标，根据我们写的计算打击优先度的
    // 函数，以及地图之中的一些信息，选择一个进行瞄准
    MAPPING::pos select_one_aim(vector<MAPPING::pos>, MAPPING::maptype global_map);

    // 根据传递进来的我们要打击的对象的坐标、以及输入进来的图像，我们需要计算出来
    // 我们需要瞄准的角度，由于目前还不知道角度应该用什么数据结构进行传递
    // 所以输出的参数暂且用void来代替
    void angle_calculate(vector<MAPPING::pos> ready_to_attack, cv::Mat input_img);

    // 与上面类似，我们需要将输入的图像以及待打击的目标进行解析，计算出我们的射击的距离等信息
    void lenth_calculate(vector<MAPPING::pos> ready_to_attack, cv::Mat input_img);

    // 根据传递进来的打击点的坐标，我们根据地图相应的信息来计算我们打击的优先级，这里
    // 用一个数组来进行描述，数值越高，说明优先级越高，当数值达到一定的时候，就可以进行打击
    vector<int> attack_order(vector<MAPPING::pos>, MAPPING::maptype global_map);
};