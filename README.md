
# Prometheus项目

Open source software for autonomous drones.

项目网址：[https://github.com/amov-lab/Prometheus](https://github.com/amov-lab/Prometheus)

# 安装

 1. 安装ROS
	 
	 参考：[https://www.ros.org/](https://www.ros.org/)
 2. 通过二进制的方法安装Mavros功能包
	 
	 Mavros安装请参考: [https://github.com/mavlink/mavros](https://github.com/mavlink/mavros)
	 
	 如果你已经使用源码的方式安装过Mavros功能包，请先将其删除
 3. 打开一个新终端（一般在home目录下）
	 
	 `crtl+alt+T`
 5. 下载
	 
	 `git clone https://github.com/amov-lab/Prometheus`
 6.source
 	打开一个新终端
	`gedit .bashrc`
	
	在打开的`bashrc.txt`文件中添加 `source /home/$(your computer name)/Prometheus/devel/setup.bash`

# 编译说明

>  cd Prometheus

> sudo chmod 777 ./complie_all.sh (第一次运行才需要执行)
	
> ./complie_all.sh
 
 - 目前每个模块都是一个独立的ros包，编译脚本会负责全部或部分编译模块代码，每个包的命名规则为`prometheus_xxx`
 - complie_all默认编译项目中所有代码
 - complie_control则只编译控制部分代码，若只需要使用控制部分代码，运行`./complie_control.sh`即可

# 运行说明

 - control模块
	 
	 `roslaunch prometheus_control xxx.launch`

# Git Push代码说明

 - 目前属于分模块开发，切记不要动他人模块的代码
 - 不确定的测试代码，可以使用分支进行开发
 - 一般默认的commit的风格为 ：模块名(control,detection,simulator等等)+具体的更新内容
  
	`control: add px4_pos_control.cpp`
   
	`simulator: update readme.md`
