#ifndef AIM_H
#define AIM_H
#include<bits/stdc++.h>
using namespace std;
using namespace cv;
class Aim
{
	public:
		bool Strategy1, Strategy2,......;//读入选择的策略  
		bool aimornot;//判断是否进行瞄准
		void getImg(const cv::Mat);//读入图片
		bool enemyornot;//判断有无目标  
		int target[20];//有目标时，将目标（装甲板）通过某种方式进行编号 
		int decide;//在有多个目标时根据策略选择一个目标 
		int targetkind;//识别目标种类，如果为基地，默认敌人速度为0（高度确定？） 
		float distance;//读入与预定目标的水平距离 
		float height;//读入目标与炮管相差的高度 
		float myvelocity;//读入自己移动速度 
		float enemyvelocity;//读入敌人移动速度
		float mydegree;//认为自己与目标的连线为0°，此刻自己的速度方向 ，逆时针为正 
		float enemydegree;//认为自己与目标的连线为0°，此刻对方的速度方向，逆时针为正
		const int gunlong;//炮管长度 (好像没必要） 
		float gundegree;//读入自己炮管的仰角 
		const float MAXgundegree;//读入自己炮管的最大仰角
		const float MAXgundegreechange;//读入自己炮管的最大仰角变化率 
		const float MAXgunvelocity;//读入最大发射速度 
		bool rotateornot;//判断对方是否旋转 
		float gundegreechange;//计算为击中目标炮管仰角变化角度 
		void moveinxy;//横向移动炮管使得目标与炮管在同一个竖直平面内 
		void moveinz;//纵向移动炮管使得炮管仰角到达既定计算值 
		float gunvelocity;//存储计算出来的炮弹发射速度（不一定需要） 
		bool OKornot;//判断是否已瞄准 
		bool attack;//发出开火指令 
	protected:
		
};

#endif