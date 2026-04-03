# 运行命令
string cmd

# 模式：
# 1、不做检测，直接运行 2、通过节点名进行检测 3、通过自定义shell脚本进行检测（需要按照规定shell脚本）
uint8 NOT_DETECTION=1
uint8 DETECTION_NODE_NAME=2
uint8 DETECTION_SHELL_CMD=3
uint8 mode
string node_name # 节点名
string detection_cmd # 检测命令

# 启动位置标识位，考虑到ROS多机情况下将程序启动在其他机载
uint8 flag

# 命令等级：
# 1、定位驱动程序 2、其他功能demo
# 在1情况下，应该多做安全保护机制
uint8 LOCATION_SOURCE_CMD=1
uint8 OTHER_CMD=2
uint8 cmd_level

# 关闭命令
string close_cmd