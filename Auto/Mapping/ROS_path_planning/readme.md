# ROS path planning development documentation
## Development Process
**Thanks to autolabor for the tutorial http://www.autolabor.com.cn/book/ROSTutorials/**
1. Installation of ubuntu and ROS
   + Dependency Packages:
   ```
   find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    urdf
    xacro
    gazebo_ros
    gazebo_ros_control
    gazebo_plugins
    gmapping
    map_server 
    amcl 
    move_base
    move_base_msgs
    actionlib
    roscpp
    )
   ```
2. Robot Simulation
   + Construction of static robot model 
     + urdf file directly write static models, however xacro can provide a modular wrapper. To construct the robot more clearly, we use xacro file to build the static model
     + The xacro folder provides the source code for the simulated robot I built. car.xacro constructs the chassis, camera.xacro constructs the camera, and rada.xacro and radasensor.xacro construct the radar. combin.xacro is used to aggregate these files in the gazebo environment to build the static robot. The corresponding launch files are in the launch folder at testgb.launch
     + ` <param name="robot_description" command="$(find xacro)/xacro $(find simirob)/xacro/combin.xacro" />`
   + Get the robot moving
     + Add inertial_matrix.xacro and drive set move.xacro
   + Gazebo simulation environment
     + Model the world in Gazebo, construct the map, and save it in the worlds folder
     + Corresponding to the testgb.launch file
     + ```
        <!-- 启动 gazebo -->
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="world_name" value="$(find simirob)/worlds/real.world" />
        </include>
       ```
   + Collection of sensor information: Rviz
     + The configuration of Rviz is located in the config folder in real.rivz
     + Corresponds to move_base.launch
     + ```
        <!-- 运行rviz -->
        <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
        <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />
        <node pkg="rviz" type="rviz" name="rviz" args="-d $(find simirob)/config/real.rviz"/>
        ```
3. Navigation
   + Loading of the map
     + Load the maps previously placed in the map folder, nav.pgm and nav.yaml respectively, the latter file provides information about the map. resolution: resolution (how many meters a pixel corresponds to); origin: the initial position of the map image in the lower left corner relative to the robot; occupied_thresh, free_thresh are thresholds for treating as obstacles
     + ```
        <!-- 设置地图的配置文件 -->
        <arg name="map" default="nav.yaml" />
        <!-- 运行地图服务器，并且加载设置的地图-->
        <node name="map_server" pkg="map_server" type="map_server" args="$(find simirob)/map/$(arg map)"/>
        ```
   + Set AMCL node for its own positioning
      + ```
        <!-- 启动AMCL节点 -->
        <include file="$(find simirob)/launch/locate.launch" />
        ```
   + Path Planning
     + ```
        <!-- 运行move_base节点 -->
        <include file="$(find simirob)/launch/movebase.launch" />
        ```
     + There are four parameter files, costmap_common_params.yaml, local_costmap_params.yaml, global_costmap_params.yaml, base_local_planner_params.yaml
## Parameter Setting
+ **The parameters can be adjusted directly via the order below**
+ `rosrun rqt_reconfigure rqt_reconfigure`
+ See official documentation http://wiki.ros.org/teb_local_planner#Parameters
+ or see https://blog.csdn.net/weixin_44917390/article/details/107568507?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166977853616782390571244%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=166977853616782390571244&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-6-107568507-null-null.142%5Ev67%5Ewechat_v2,201%5Ev3%5Econtrol_2,213%5Ev2%5Et3_control1&utm_term=teb%20local&spm=1018.2226.3001.4187
+ or
+ ```
  TebLocalPlannerROS:
 
  odom_topic: odom
      
  # Trajectory
    
  teb_autosize: True
  #期望的轨迹时间分辨率
  dt_ref: 0.3
  #根据当前时间分辨率自动调整大小的滞后现象，通常约为。建议使用dt ref的10%
  dt_hysteresis: 0.1
  #最大样本数
  max_samples: 500
  #覆盖由全局规划器提供的局部子目标的方向
  global_plan_overwrite_orientation: True
  allow_init_with_backwards_motion: True
  #指定考虑优化的全局计划子集的最大长度
  max_global_plan_lookahead_dist: 10
  #如果为正值，则通过点（via-points ）从全局计划（路径跟踪模式）展开，该值确定参考路径的分辨率（沿着全局计划的每两个连续通过点之间的最小间隔，可以参考参数weight_viapoint来调整大小
  global_plan_viapoint_sep: -1
  global_plan_prune_distance: 1
  exact_arc_length: False
  #每个采样间隔的姿态可行性分析数，default：4
  feasibility_check_no_poses: 2
  #发布包含完整轨迹和动态障碍的列表的规划器反馈 
  publish_feedback: False
      
  # Robot
          
  max_vel_x: 0.4
  max_vel_x_backwards: 0.2
  max_vel_y: 0.0
  max_vel_theta: 0.3 # the angular velocity is also bounded by min_turning_radius in case of a carlike robot (r = v / omega)
  acc_lim_x: 0.5
  acc_lim_theta: 0.5
 
  # ********************** Carlike robot parameters ********************
  #最小转弯半径
  min_turning_radius: 0.5
  wheelbase: 0.4 
  #是否允许原地转 
  cmd_angle_instead_rotvel: True 
  # ********************************************************************
 
  footprint_model: # types: "point", "circular", "two_circles", "line", "polygon"
    type: "line"
    radius: 0.2 # for type "circular"
    line_start: [0.0, 0.0] # for type "line"
    line_end: [0.4, 0.0] # for type "line"
    front_offset: 0.2 # for type "two_circles"
    front_radius: 0.2 # for type "two_circles"
    rear_offset: 0.2 # for type "two_circles"
    rear_radius: 0.2 # for type "two_circles"
    vertices: [ [0.25, -0.05], [0.18, -0.05], [0.18, -0.18], [-0.19, -0.18], [-0.25, 0], [-0.19, 0.18], [0.18, 0.18], [0.18, 0.05], [0.25, 0.05] ] # for type "polygon"
 
  #目标位置的允许距离误差   
  xy_goal_tolerance: 0.2
  #目标位置的允许角度误差
  yaw_goal_tolerance: 0.1
  #去除目标速度的约束
  free_goal_vel: False
  complete_global_plan: True
      
  #与障碍的最小期望距离,注意，teb_local_planner本身不考虑膨胀半径 
  min_obstacle_dist: 0.5
  #障碍物周围缓冲区(应大于min_obstacle_dist才能生效) 
  inflation_dist: 0.8
  #应否考虑到局部costmap的障碍
  include_costmap_obstacles: True
  #考虑后面n米内的障碍物
  costmap_obstacles_behind_robot_dist: 1.0
  #为了保持距离，每个障碍物位置都与轨道上最近的位置相连
  obstacle_poses_affected: 20
 
  dynamic_obstacle_inflation_dist: 0.6
  include_dynamic_obstacles: True 
 
  costmap_converter_plugin: ""
  costmap_converter_spin_thread: True
  costmap_converter_rate: 8
 
  #在每个外循环迭代中调用的实际求解器迭代次数      
  no_inner_iterations: 5
  #在每个外循环迭代中调用的实际求解器迭代次数
  no_outer_iterations: 4
  optimization_activate: True
  optimization_verbose: False
  #为硬约束近似的惩罚函数添加一个小的安全范围
  penalty_epsilon: 0.1
  obstacle_cost_exponent: 4
  weight_max_vel_x: 2
  weight_max_vel_theta: 1
  weight_acc_lim_x: 1
  weight_acc_lim_theta: 1
  weight_kinematics_nh: 1000
  weight_kinematics_forward_drive: 1
  weight_kinematics_turning_radius: 1
  weight_optimaltime: 1 # must be > 0
  weight_shortest_path: 0
  weight_obstacle: 100
  weight_inflation: 0.2
  weight_dynamic_obstacle: 10 # not in use yet
  weight_dynamic_obstacle_inflation: 0.2
  weight_viapoint: 1
  weight_adapt_factor: 2
  ```
  ```
  global_costmap:
    footprint: [[-0.305, -0.18], [-0.305, 0.18], [0.305, 0.18], [0.305, -0.18]]
    footprint_padding: 0.01
    transform_tolerance: 0.5
    #用来决定全局地图信息更新的频率，单位是Hz
    update_frequency: 10.0
    publish_frequency: 10.0
    
    #用来表示全局代价地图需要在哪个参考系下运行
    global_frame: map
    #用来表示代价地图参考的机器本体参考系
    robot_base_frame: base_footprint
    #用来设置分辨率
    resolution: 0.10
    
    #用来设置在机器人移动过程中是否需要滚动窗口，以保持机器人处于中心位置
    rolling_window: true
    #用来设置全局代价地图宽高，单位是米
    width: 28.0
    height: 13.0
    track_unknown_space: false
 
    plugins: 
        - {name: static,    type: "costmap_2d::StaticLayer"}            
        - {name: sensor,    type: "costmap_2d::ObstacleLayer"}
        - {name: inflation, type: "costmap_2d::InflationLayer"}
 
    static:        
        map_topic: /map 
        subscribe_to_updates: true
 
    sensor:
        observation_sources: laser_scan_sensor
        laser_scan_sensor: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true}
 
    inflation:
        inflation_radius: 0.6
        cost_scaling_factor: 2.0

   ```
        local_costmap:
        footprint: [[-0.305, -0.18], [-0.305, 0.18], [0.305, 0.18], [0.305, -0.18]]
        footprint_padding: 0.01
        #发布tf变换的超时时间
        transform_tolerance: 0.5
        #地图更新频率
        update_frequency: 10.0
        #用来设置代价地图发布可视化信息的频率，单位是hz
        publish_frequency: 10.0
    
        global_frame: map
        robot_base_frame: base_footprint
        #用来设置分辨率
        resolution: 0.10
        #用来设置在机器人移动过程中是否需要滚动窗口，以保持机器人处于中心位置
        rolling_window: true
        #用来设置局部代价地图宽高，单位是米
        width: 5.0
        height: 5.0
        #分辨率可以设置的与静态地图不同，但是一般情况下两者是相同的
        resolution: 0.05
    
        track_unknown_space: false
    
        plugins:            
            - {name: sensor,    type: "costmap_2d::ObstacleLayer"}
            - {name: inflation, type: "costmap_2d::InflationLayer"}
    
        sensor:
            observation_sources: laser_scan_sensor
            laser_scan_sensor: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true}
    
        inflation:
            inflation_radius: 0.7
            cost_scaling_factor: 8.0
    ```
    #当move_base在不活动状态时,是否关掉costmap.
    shutdown_costmaps: false
    #向底盘控制移动话题cmd_vel发送命令的频率.
    controller_frequency: 5 #1.0
    #在空间清理操作执行前,控制器花多长时间等有效控制下发
    controller_patience: 3.0
    #全局规划操作的执行频率.如果设置为0.0,则全局规划器仅在接收到新的目标点或者局部规划器报告路径堵塞时才会重新执行规划操作
    planner_frequency: 10.0
    #在空间清理操作执行前,留给规划器多长时间来找出一条有效规划.
    planner_patience: 5.0
    #执行修复机制前,允许振荡的时长.
    oscillation_timeout: 10.0
    #来回运动在多大距离以上不会被认为是振荡
    oscillation_distance: 0.2
    #机器人是否在尝试清理空间时进行原地旋转
    clearing_rotation_allowed: false
    ```

## Target Interface
   + The send_goals_node.cpp file has been added to the src folder to send out the robot's navigation target points through the program
   + **Note that the coordinates in the program are relative to the initial position of the robot, the coordinates of the map need to be transformed**