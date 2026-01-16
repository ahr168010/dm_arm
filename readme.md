# dm_robotarm
ubuntu24 ros2jazzy

## dm_robotarm_description
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

## moveit配置
编译加载环境
启动配置助手
```
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

