/*
  i want to give a brief introduction of what i want to do
  so, that's it
  please don't laugh at me 
*/
#include<iostream>
using namespace std;
bool startornot()
{
	//����Ϊ������Ҫ�Ƿ�ֹwhile(1)����
	//�и��õķ���������� 
} 
bool aimornot()
{
	/*����׼�ļ��ֿ��������
	     1��û�ӵ��� 
	     2��ĳЩ���ܵĲ���
		 3�����汻��
		���ܻ��и��� 
    */
}
bool findtarget()
{
	//��ͼ�����ҵ�Ŀ��
	//���򷵻�1�����򷵻�0 
} 
void gettargetdo()
{
	//����Ŀ���ǰ���£��õ�Ŀ��ĸ���״̬ 
}
struct targetdo
{
	//����ṹ�������洢Ŀ���״̬ 
	bool rotate;//��ת��� 
	float distance;//����
	int type;//���ͣ�����or����or�ڱ�or������
	float velocity;//Ŀ�������ٶ� ���趨�壩 
	float direction;//Ŀ�����з��� ���趨�壩
	bool canbeattack;//Ŀ���ܷ񱻻��򣨱������з�λ��װ������Ŀ�꣩ 
	//......������ 
}tardo[5]//�ݶ��������Ŀ������ݣ�;
int whotoaim()
{
	//�ж���׼��һ��Ŀ��÷����������
	//���ص÷����Ŀ����±� 
	//���ֱ�׼������Զ��ı�
	//����һ����ֵ�������ߵĵ÷�û�� ������ֵ������0
	//Ϊ�˱������һ��������Ŀ����Ҵ�������� 
}
void fire()
{
	//������򣬰������²���
	//1������Ŀ����һ��λ��
	//2���������Ŀ���������̨λ���뷢���ٶ�
	//3���ƶ���̨������λ��
	//4.ȷ����׼�����Բ�Ҫ�˲���
	//5.��������ָ�� 
	//6��������Ŀ����ٴ���������� 
}
int main()
{
	bool start=startornot() ;// start�����ж�����ϵͳ�Ƿ񿪻� 
	bool aiming=aimornot();//������������ж��Ƿ������׼���� 
	while(start) 
	{
	  while(aiming)
	  {
	  	if(findtarget()); //Ѱ��Ŀ�� ��ͼ����ʶ�� ����ͬʱ�ж��Ŀ�� 
		    {
			gettargetdo();//�õ��������Ŀ��ĸ���״̬ 
			 int targetnum=whotoaim(); //ͨ������ѡ���÷���ߵ�Ŀ�������� 
			 fire(); //����ָ� 
			}//���ʣ���ʧĿ��֮��Ӧ�ûص�Ѱ��Ŀ�� ���ǻ���÷ֵڶ��ߵ�Ŀ�ꣿ 
	  }
    }
	return 0;
}
