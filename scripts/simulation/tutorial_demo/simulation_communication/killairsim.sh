#!/bin/bash

# 匹配包含关键字的进程（支持roslaunch参数和可执行文件名）
target_pids=$(pgrep -f 'airsim_ros_pkgs|prometheus_airsim|airsim_node|airsim.launch')

# 显示找到的进程树
echo "[Process Tree]"
ps -f --forest $(echo $target_pids | sed 's/ /,/g') 2>/dev/null

# 终止主进程及其子进程
for pid in $target_pids; do
    # 获取进程组ID（处理子进程）
    pgid=$(ps -o pgid= $pid | tr -d ' ')
    [ -z "$pgid" ] && continue

    # 获取进程详细信息
    cmdline=$(ps -p $pid -o cmd=)
    exe_name=$(basename $(readlink -f /proc/$pid/exe 2>/dev/null) 2>/dev/null)

    echo -e "\nTerminating process tree:"
    echo "PID: $pid | PGID: $pgid | EXE: $exe_name"
    echo "CMD: ${cmdline:0:120}..."

    # 终止（SIGTERM进程组）
    kill -TERM -$pgid 2>/dev/null
    sleep 0.3

    # 强制终止残留
    if ps -p $pid >/dev/null; then
        kill -KILL -$pgid 2>/dev/null
        echo "[Force killed] Process group $pgid"
    else
        echo "[Gracefully exited] Process group $pgid"
    fi
done

# 最终确认
remaining=$(pgrep -f 'airsim_ros_pkgs|prometheus_airsim')
if [ -n "$remaining" ]; then
    echo -e "\n[Warning] Remaining processes:"
    ps -fp $remaining
else
    echo -e "\n[Success] All airsim processes cleaned"
fi
