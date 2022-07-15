# 机器人运动学动力学以及规划算法库
 
本项目为机械臂(串联六轴)算法开发与验证框架，包含
1. 机械臂轨迹规划库(***robot trajectory planning library***)：`rtpl project`
2. 机械臂动力学库(***robot dynamics control library***)：`rdcl project`
3. 测试和验证：`test project`

## 算法库功能
本项目中包含有机械臂轨迹规划与动力学两个算法库，其具有如下功能：  
- 机械臂正逆运动学
- 机械臂微分运动学
- 机械臂关节空间轨迹规划
- 机械臂笛卡尔空间直线轨迹规划
- 机械臂笛卡尔空间圆弧轨迹规划
- 机械臂笛卡尔空间直线间圆弧过渡轨迹规划
- 机械臂笛卡尔空间椭圆弧轨迹规划
- 机械臂笛卡尔空间B样条曲线轨迹规划
- 机械臂动力学力矩生成
- 机械臂外力检测
 ## 依赖项
 Eigen库：Windows环境下将源码放置于解决方案同级文件夹，并命名为**eigne3**；
 Linux环境下可直接安装`sudo apt-get install libeigen3-dev`

## 环境
VS2015

 ## 简要说明
 test.cpp：主函数，里面包含了机械臂运动控制相关功能测试，以及任务执行级运动仿真  
 GlobalParams.h：全局变量申明，包含机械臂构型DH参数，运动学和动力学相关配置参数等
 RobotParameter/RobotParameter.cpp 全局变量定义以及测试用读取json文件里的参数接口


 ## 轨迹规划库使用
编译生成的静态库文件`rtpl.lib`存放于`lib`文件夹内，在项目中通过链接该静态库，并包含`inc`文件夹内的头文件即可使用。
```
// 初始化，仅需初始化一次
TaskTrajPlanner planner(&urrbt,q_fdb,g_cycle_time,g_jvmax,g_jamax,g_cvmax,g_camax);
// 添加并生成一段路径的完整流程
planner.Reset(q0);
planner.AddTraj(...);
planner.GenerateJPath(...);
```
**注意**：对于机械臂末端工具发生变化的情况，在轨迹规划时必须保证先更新末端工具，再重置轨迹规划器的顺序
```
rbt.UpdateTool(tool_pos,tool_rpy);
planner.Reset(q0);
```

## 动力学库使用
编译生成的静态库文件`rdcl.lib`存放于`lib`文件夹内，在项目中通过链接该静态库，并包含`inc`文件夹内的头文件即可使用。
```
// 初始化，仅需初始化一次
ForceControl force_controller;
force_controller.CreateForceControl(&rbt, all_params);
// 实时计算机械臂动力学
force_controller.RobotDynamicsCalculation(...);
// 生成各关节力矩
force_controller.GetGravityTorque();
force_controller.GetFrictionTorque();
force_controller.GetDynamicsTorque();
force_controller.GetFeedforwardTorque();
// 碰撞检测
force_controller.CheckCollision();
```

## 仿真说明
models文件夹内包含有机械臂simulink运动学仿真模型，以及相应的脚本文件，
* `SimuCleanRobot.slx`
运行环境：Matlab2018b  
该模型可用于机械臂离线仿真，将机械臂关节数据保存为文本文件，并在matlab工作区中定义时间序列(该模型采用定步长5ms进行仿真)，点击运行，机械臂可视化模型将按照所保存的关节数据进行运动。
* `RobotRealTime.slx`
运行环境：Matlab2018b  
依赖：MQTT in MATLAB--v1.4, Real-Time Pacer for Simulink--v1.0.0.1
该模型可用于机械臂在线仿真，通过设置mqtt服务端ip地址，并订阅相应topic，该模型能够以给定时间间隔获取机械臂控制器中所反馈的关节数据。该模型采用定步长进行仿真，并保证该步长与机械臂控制器发送的关节数据频率一致。