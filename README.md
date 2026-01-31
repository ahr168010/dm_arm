# 使用conda环境运行ros2+mujoco
ros2 jazzy使用python版本为python3.12
参考教程：
https://zhuanlan.zhihu.com/p/14281813754

https://blog.csdn.net/nenchoumi3119/article/details/148159117
## 1. 准备 Conda 虚拟环境
```
conda create -n ros2 python=3.12.3
conda activate ros2
pip install rospkg
pip install -U colcon-common-extensions
```
期间有两个error 目前没有影响
## 2. 利用colcon 编译
确保虚拟环境已经激活
```
python -m colcon build
```

在通常的编译命令前加上 python -m 以使用虚拟环境中的colcon进行编译
## 3. 运行程序
与常规运行ros包一样，运行编译好的包

在虚拟环境中
'''
pip install mujoco
'''
# dm_robotarm
使用 ubuntu24 ros2 jazzy

## 1. dm_robotarm_description
该包为机械臂显示包

下面为搭建流程

### 1.sw模型导入ros2
1. 在sw中标定旋转轴

2. 下载sw2urdf插件

```https://github.com/ros/solidworks_urdf_exporter/releases```


3. 通过插件导出包，并且导入到ubuntu中

4. 创建一个新的工作空间，创建src文件

5. 创建一个description包

```
ros2 pkg create --build-type ament_python --license MIT dm_robotarm_description
```
6. 在包下面创建meshes与urdf文件


将sw导出的urdf包中的meshes与urdf文件复制到description包下的对应文件夹下

7. 打开urdf文件，修改包的路径

filename="package://dm_robotarm_description/meshes/base_link.STL" />

修改到正常的路径

8. 在工作空间ws下创建一个.vscode文件夹，创建一个setting.json文件

```
// settings.json
{
  // other settings
  "urdf-visualizer.packages": {
      "dm_robotarm_description": "src/dm_robotarm_description"
  },
  // other settings
}
```

粘贴上方内容，修改包名，在vscode中下载urdf visualizer插件

打开urdf包中urdf文件，点击显示，即可看到机械臂模型

settings.json 配置是为了帮助 VSCode 定位到正确的包路径。当 URDF 文件引用 package://dm_robotarm_description/meshes/base_link.STL 时，VSCode 会根据配置的路径查找包和资源文件。如果没有正确配置，VSCode 就无法找到这些资源文件，导致模型无法显示。

## 2. dm_robotarm_config
编译加载环境
启动配置助手
```
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
根据配置助手完成配置，生成config文件
打开joint_limits.yaml文件夹
修改文件内容
运行
```
ros2 launch dm_robotarm_config demo.launch.py
```
即可运行rviz

## 3. gazebo_grasping_sim

### dm_arm.gazebo.friction.urdf.xacro
这段代码将名为 dm_arm 的机器人模型与 ROS2 和 Gazebo 仿真环境进行集成，配置了机器人控制和物理属性，以便在仿真中进行控制和交互。

### dm_arm.gazebo.ros2_control.xacro

复制src/dm_robotarm_config/config/dm_arm.ros2_control.xacro文件
修改hardware部分
```
<hardware>
  <!-- By default, set up controllers for simulation. This won't work on real hardware -->
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</hardware>
```
### moveit_cpp.yaml
为moveit配置多个运动规划管道
运行gazebo的launch后gazebo中没有机械臂，无法找到stl文件解决方法，修改urdf文件的引用方式
```
可能是解析路径的写法上的问题。
<mesh filename="file://$(find XBot)/meshes/base_link.STL" />这种是ros1的写法，
<mesh filename="package://XBot/meshes/base_link.STL"/>是ros2推荐的写法，但是gazebo11可能解析不了后一种写法，换成前一种应该就可以了。
我在使用moveit2配置助手的时候也遇到过这种问题。
```
## 4. dm_arm_mujoco

### mujoco仿真配置代码

#### meshes文件
该文件夹下的文件与`src/dm_robotarm_description/meshes`文件夹中的内容相同，
为避免重复存储，采用**软链接**的方式进行复制。

#### mjcf文件
- **dm_arm**  
  存放机械臂相关的配置文件。
  需要修改`actuator`中的kp kd参数，使其符合真实电机参数
  
- **scene_pick**  
  存放整体场景配置的文件。

### mujoco_show.py
这段代码通过 MuJoCo 模拟机械臂，并通过 ROS2 实现了机械臂的控制。关键功能包括：

- 加载和显示 MuJoCo 模型。

- 通过 线性插值（lerp） 逐步过渡到目标关节位置，避免机械臂的瞬移。

- 创建 ROS2 订阅器，接收外部控制命令，更新目标关节位置。

- 每隔一定时间发布机械臂的关节状态。
