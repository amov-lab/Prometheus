#!/bin/bash

# 查找目标进程PID（排除grep自身）
pids=$(ps aux | grep '/bin/px4' | grep -v grep | awk '{print $2}')

# 循环终止进程
for pid in $pids; do
    # 二次验证进程可执行路径
    exe_path=$(readlink -f /proc/$pid/exe 2>/dev/null)
    if [[ "$exe_path" == *"/bin/px4" ]]; then
        echo "Killing PID $pid ..."
        kill -9 $pid
    else
        echo "Skip invalid PID $pid (Executable: $exe_path)"
    fi
done

echo "Cleanup completed."
