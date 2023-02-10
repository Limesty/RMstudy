/*
  i want to give a brief introduction of what i want to do
  so, that's it
  please don't laugh at me 
*/
#include<iostream>
using namespace std;
bool startornot()
{
	//设置为函数主要是防止while(1)无限
	//有更好的方法请告诉我 
} 
bool aimornot()
{
	/*不瞄准的几种可能情况：
	     1、没子弹了 
	     2、某些可能的策略
		 3、犯规被禁
		可能还有更多 
    */
}
bool findtarget()
{
	//从图像中找到目标
	//有则返回1，无则返回0 
} 
void gettargetdo()
{
	//在有目标的前提下，得到目标的各项状态 
}
struct targetdo
{
	//这个结构体用来存储目标的状态 
	bool rotate;//旋转与否 
	float distance;//距离
	int type;//类型（基地or步兵or哨兵or其他）
	float velocity;//目标运行速度 （需定义） 
	float direction;//目标运行方向 （需定义）
	bool canbeattack;//目标能否被击打（避免击打敌方位于装填区的目标） 
	//......待补充 
}tardo[5]//暂定保存五个目标的数据，;
int whotoaim()
{
	//判定瞄准哪一个目标得分最高来决定
	//返回得分最高目标的下标 
	//评分标准会随策略而改变
	//存在一个阈值，如果最高的得分没有 超过阈值，返回0
	//为了避免出现一旦有疑似目标就乱打情况出现 
}
void fire()
{
	//开火程序，包括以下部分
	//1、计算目标下一刻位置
	//2、计算击中目标所需的云台位置与发射速度
	//3、移动云台至所需位置
	//4.确认瞄准（可以不要此步）
	//5.发出开火指令 
	//6、后续：目标跟踪打击（待定） 
}
int main()
{
	bool start=startornot() ;// start用来判断自瞄系统是否开机 
	bool aiming=aimornot();//这个函数用来判断是否进入瞄准程序 
	while(start) 
	{
	  while(aiming)
	  {
	  	if(findtarget()); //寻找目标 从图像中识别 可能同时有多个目标 
		    {
			gettargetdo();//得到多个可能目标的各项状态 
			 int targetnum=whotoaim(); //通过评分选出得分最高的目标来击打 
			 fire(); //开火指令集 
			}//提问：丢失目标之后应该回到寻找目标 还是击打得分第二高的目标？ 
	  }
    }
	return 0;
}
