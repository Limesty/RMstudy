#pragma once
#include<iostream>

using namespace std;

class Firepower{
	bool attack; // �Ƿ��𹥻� 
    int total_bullets; //�ܵ��ӵ���
	public:
        int bullet; // ʣ���ӵ���
        int calculate_left_bullet(int shooted_bullet); // ����ʣ���ӵ���

		bool doFire(int bullet); // ����ʣ�൯���ж��ܷ����������Ӧ��ȥ�������� 
		
		void doAttack(); // �����ӵ� 
		
		void cancelAttack(); // ȡ������ 

        void needSupply(int bullet); // �ӵ�����ʱǰ�������� 
}; 

