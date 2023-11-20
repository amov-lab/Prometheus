<!--
 * @Author: Eason Yi eason473867143@gmail.com
 * @Date: 2023-10-17 20:54:51
 * @LastEditors: Eason Yi eason473867143@gmail.com
 * @LastEditTime: 2023-10-17 21:36:00
 * @FilePath: /Prometheus/Modules/uav_control_fmt/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
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
## 启动测试脚本1 - 读取fmt数据
rosrun prometheus_uav_control_fmt fmt_test
## 启动测试脚本2 - 控制fmt
rosrun prometheus_uav_control_fmt fmt_test
## 启动测试脚本3 - 发送任务层指令
rosrun prometheus_uav_control_fmt fmt_test
```
- 需要在QGC地面站手动解锁
- mcn echo auto_cmd打印外部控制指令


**TODO**

- 等待UE4的封装环境
- 测试FMT在uav_control代码中的biaoxian
- 等待支持vision_poses


**其他TODO**

- GPS和全局位置相关的信息有些混乱，包括距离传感器信息，因为这些信息是后来添加的，没有很好的融入代码框架中，连基本的打印都没有体现
- 在P450和P600进阶款的产品中，一定要完善uav_control的种种残留问题
