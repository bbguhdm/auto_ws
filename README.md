## 项目介绍
该项目使用gazebo仿真实现，用ros实现机器人在迷宫环境中自主导航避障并到达目标点，所有算法均手动实现，不调用现成开源算法。轨迹生成算法使用Polynomial Time Scaling（多项式时间缩放），路径搜索算法使用A*，控制算法使用PID。

## 项目环境
Ubuntu18.04 ROS melodic

## 安装运行

### 克隆仓库到本地
```
git clone https://github.com/bbguhdm/auto_ws.git
```

### 编译
```
cd auto_Ws
catkin_make
```

### 添加环境变量
```
echo "source ~/auto_ws/devel/setup.bash" >> ~/.bashrc
source ~/auto_ws/devel/setup.bash
```

### 运行
```
roslaunch auto_car gazebo.launch
```

#### 另开一个终端
```
cd auto_ws/src/auto_car/scripts/
./auto_go.py
```

