#!/bin/bash

# 匹配所有ego相关进程（包含节点、launch文件、包名）
target_pattern='ego_planner|traj_server_for_prometheus|sitl_ego'

# 查找主进程PID（包含进程树）
mapfile -t target_pids < <(pgrep -f "$target_pattern")

# 显示进程树状结构
echo "[Process Tree Visualization]"
ps -f --forest -p "$(echo "${target_pids[@]}" | tr ' ' ',')" 2>/dev/null

# 按进程组终止
for pid in "${target_pids[@]}"; do
    # 获取进程详细信息
    pgid=$(ps -o pgid= "$pid" | tr -d ' ')
    [ -z "$pgid" ] && continue
    
    # 获取完整命令行
    cmd=$(ps -p "$pid" -o cmd=)
    exe_name=$(basename "$(readlink -f /proc/"$pid"/exe 2>/dev/null)" 2>/dev/null)

    echo -e "\n■ Terminating Process Group:"
    printf "%-10s %-6s %s\n" "PID" "$pid" "| PGID: $pgid"
    printf "%-10s %-6s %s\n" "Executable" "" "$exe_name"
    printf "%-10s %-6s %s\n" "Command" "" "${cmd:0:100}..."

    # 分级终止策略
    if kill -TERM -"$pgid" 2>/dev/null; then
        echo -n "Waiting for graceful exit..."
        for _ in {1..5}; do
            sleep 0.5
            if ! ps -p "$pid" >/dev/null; then
                echo " [OK]"
                break
            fi
        done
    fi

    # 强制终止检查
    if ps -p "$pid" >/dev/null; then
        kill -KILL -"$pgid" 2>/dev/null
        echo "[Force Killed] Process group $pgid"
    fi
done

# 最终清理检查
remaining=$(pgrep -f "$target_pattern")
if [ -n "$remaining" ]; then
    echo -e "\n⚠️  Remaining processes:"
    ps -fp "$remaining"
else
    echo -e "\n✅ All ego_planner related processes have been terminated"
fi