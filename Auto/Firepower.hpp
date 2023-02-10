#pragma once
#include<iostream>

using namespace std;

class Firepower{
	bool attack; // 是否发起攻击 
    int total_bullets; //总的子弹数
	public:
        int bullet; // 剩余子弹数
        int calculate_left_bullet(int shooted_bullet); // 计算剩余子弹数

		bool doFire(int bullet); // 根据剩余弹量判断能发起进攻还是应该去往补给区 
		
		void doAttack(); // 发射子弹 
		
		void cancelAttack(); // 取消发射 

        void needSupply(int bullet); // 子弹不足时前往补给区 
}; 

