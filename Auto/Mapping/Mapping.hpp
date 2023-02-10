/**
 * @file Mapping
 * @brief Contains the interface to control robot path planning
 * @details This file contains a namespace and two classes: Map and Mapping.
 * Namespace is used to define special variables and prevent naming conflicts
 * Map is used to describe the map, and Mapping is responsible for describing the motion state and integrating the functions that may be used.
 * @author Zhang Yifei
 * @date 2022-12-10
 */
#pragma once
#include <vector>
#include <unordered_set>
using namespace std;
/**
 * @brief Define special variables and prevent naming conflicts, contains information about the map and some units
 */
namespace MAPPING {
	///<地图点类型
	enum PosType {
		_emtpy,///<普通点
		_outside,///<地图外的点
		_supply, ///< 补给区
		_sentry_start_area,///<哨兵启动区
		_start_area,///<启动区
		_outbase,///<我方基地
		_enemy_base,///<敌方基地
		_central_benefits,///<中心增益点
		_obstackle,///<障碍物
		_shelter///<掩体
	};
	typedef vector<vector<PosType>> maptype;///<地图类型
	typedef int unit;///<坐标单位
	typedef pair<unit, unit> pos;///<坐标
	typedef pair<unit, unit> velocity;///<速度矢量
	typedef unordered_set<pos> area;///<区域
	const velocity leftunit = { -1,0 };
	const velocity rightunit = { 1,0 };
	const velocity upunit = { 0,1 };
	const velocity downunit = { 0,-1 };
	const velocity upleftunit = { -1,1 };
	const velocity uprightunit = { 1,1 };
	const velocity downleftunit = { -1,-1 };
	const velocity downrightunit = { 1,-1 };
}
/**
 * @brief map class
 * @details protected section: This section contains information about the different sets of locations in the map, the boundaries of the map and so on. Defined as protected to facilitate direct access to inherited classes, and the development of accessibility requirements are not high
 * public section: This section contains the judgment function, given a point coordinates, to determine what area the point belongs to.
 */
class Map {
protected:
	MAPPING::maptype grid;  ///<矩阵储存地图
	MAPPING::unit leftBound;
	MAPPING::unit rightBound;
	MAPPING::unit upBound;
	MAPPING::unit downBound;///<地图边界
	MAPPING::area supplyArea;///<补给区
	MAPPING::area sentryStartArea;///<哨兵启动区
	MAPPING::area StartArea;///<启动区
	MAPPING::area ourBase;///<我方基地
	MAPPING::area enemyBase;///<敌方基地
	MAPPING::area centralBenefits;///<中心增益点
	MAPPING::area obstackle;///<障碍物
	MAPPING::area shelter;///<掩体
public:
	Map(MAPPING::maptype initialMap,MAPPING::unit l, MAPPING::unit r, MAPPING::unit u, MAPPING::unit d);
	~Map();
	bool isOutside(MAPPING::pos)const;
	bool isOurBase(MAPPING::pos)const;
	bool isEnemyBase(MAPPING::pos)const;
	bool isSupplyArea(MAPPING::pos)const;
	bool isSentryStartArea(MAPPING::pos)const;
	bool isStartArea(MAPPING::pos)const;
	bool isCentralBenefits(MAPPING::pos)const;
	bool isObstackle(MAPPING::pos)const;
	bool isShelter(MAPPING::pos)const; 
};

//void reactForOutside();
//void reactForOurBase();
//void reactForEnemyBase();
//void reactForSupplyArea();
//void reactForSentryStartArea();
//void reactForStartArea();
//void reactForCentralBenefits();
//void reactForObstackle();
//void reactForShelter();

/**
 * @brief path planning class
 * @details private inheritance Map: Mapping class should 'has a' map information, so it is defined as a private inheritance class of Map, which can access the information of Map.
 * private section: variables used to describe the robot's motion, such as speed magnitude, direction of motion, current position, etc.
 * public part: function that defines the robot's motion strategy and describes how the robot should move. It is part of the path planning section.
 * friend function section: used to describe how the robot reacts when it hits different areas. It is defined as a friend function because it may need to call private information from other classes.
 */
class Mapping : private Map{
private:
	MAPPING::velocity direction; ///<移动方向
	double speed;///<移动速度
	MAPPING::pos curPos;///<当前位置
	vector<MAPPING::pos> moveHistory;///<历史位置
	vector<MAPPING::pos> path;///<路径规划
	double sampl_interval;//采样间隔
public:
	Mapping();
	~Mapping();
	uint32_t computeVelocityCommands();//Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
	double feasibility_check_lookahead_distance();//Specify up to which distance(and with an index below feasibility_check_no_poses) from the robot the feasibility should be checked each sampling interval; if - 1, all poses up to feasibility_check_no_poses are checked.
	void detectObstacles();//每次采样检测是否出现障碍物
	void refreshMap();//每次采样更新地图
	vector<MAPPING::pos> moveLine();//根据当前位置和下一位置生成移动路线序列
	void refresh(MAPPING::velocity cur_direction, double cur_speed, MAPPING::pos cur_pos);//更新当前状态
	
	//friend void reaction(MAPPING::pos curPos);//遇到特殊地带的反应，调用reactFor...函数，定义为友元函数，便于共享数据
};

class Robot {
	double 	acc_lim_theta;///<Maximum angular acceleration of the robot
	double 	acc_lim_x;///<Maximum translational acceleration of the robot.
	double 	acc_lim_y;///<Maximum strafing acceleration of the robot.
	double 	max_vel_theta;///<Maximum angular velocity of the robot.
	double 	max_vel_x;///Maximum translational velocity of the robot.
	double 	max_vel_y;///Maximum strafing velocity of the robot

};
