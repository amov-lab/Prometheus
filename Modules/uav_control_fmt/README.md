## FMT 仿真

**编译Prometheus**

```
cd Prometheus
./compile_fmt.sh
```

**运行**

```shell
## 先打开QGC地面站（他有个bug，必须开qgc才能正常运行）
## 启动Mavros
roslaunch prometheus_gazebo sitl_fmt_mavros.launch
## 启动FMT仿真
cd qemu-vexpress-a9
./qemu.sh
## 启动测试脚本
rosrun prometheus_uav_control_fmt fmt_test
```

**TODO**

- 在fmt_test.h中完善我们要订阅的所有MAVLINK消息的ID的声明（参考：https://mavlink.io/en/messages/common.html）
- 在fmt_test.cpp中继续添加MAVLINK消息配置信息
- 测试Prometheus需要收到所有mavros消息都正常


**其他TODO**

- GPS和全局位置相关的信息有些混乱，包括距离传感器信息，因为这些信息是后来添加的，没有很好的融入代码框架中，连基本的打印都没有体现
- 在P450和P600进阶款的产品中，一定要完善uav_control的种种残留问题
