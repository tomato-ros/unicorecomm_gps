# UM982 ROS Driver

## UM982 模组资料

- [UM982 产品简介](https://www.unicorecomm.com/products/detail/26)
- [UPRECISE 上位机](https://www.unicorecomm.com/products/detail/57)

## Clion ROS 远程开发

### CMake 编译选项配置

```text

1. 在 CMake 配置界面的 "CMake options" 中添加

-DCMAKE_TOOLCHAIN_FILE=$CMakeProjectDir$/ros2_toolchain.cmake

2. 在 Clion 中重新加载 CMake 项目
```

### Clion 本地调试配置

如果使用 idea 直接进行调试遇到错误提示：error while loading shared libraries: liblibstatistics_collector.so: cannot open shared object file: No such file or directory

#### LD_LIBRARY_PATH

```text
运行/调试配置 --> 环境变量 增加如下变量:

LD_LIBRARY_PATH

变量取值方式如下:

> 打开终端执行如下命令:

source /opt/ros/humble/setup.bash
source $HOME/autoware_tomato/setup.bash
echo $LD_LIBRARY_PATH

> 复制变量输出值, 将取出的变量值配置到 IDEA 中即可。
```

#### PYTHONPATH

```text
运行/调试配置 --> 环境变量 增加如下变量:

PYTHONPATH

变量取值方式如下:

> 打开终端执行如下命令:

source /opt/ros/humble/setup.bash
source $HOME/autoware_tomato/setup.bash
echo $PYTHONPATH

> 复制变量输出值, 将取出的变量值配置到 IDEA 中即可。
```

本地调试命令行参数语法示例

启动示例：unicorecomm_gps_node --ros-args -p x:=0 -p y:=0 --params-file /home/ros/work/autoware.vehicle_tomato/src/drive/unicorecomm_gps/config/um982.yaml

```text
--ros-args
-p
x:=0
-p
y:=0
--params-file
$ProjectFileDir$/config/um982.yaml
```