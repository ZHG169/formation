#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source /home/ncrl/ncrl_mqtt/catkin_ws/install/setup.bash

# Agent 監控程式、PX4 launch、Offboard 節點必須一致
export ROS_DOMAIN_ID=0
# Avoid stale ros2 daemon graph cache causing false DDS NG results.
export ROS2CLI_DISABLE_DAEMON=1

SCRIPT_DIR="$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &&
    pwd
)"

LOG_FILE="${SCRIPT_DIR}/microxrce_agent.log"

AGENT_PID=""
AGENT_STARTED_BY_SCRIPT=0
STOPPING=0


find_agent_pids() {
    pgrep -x MicroXRCEAgent 2>/dev/null || true
}


port_8888_in_use() {
    ss -H -lun 'sport = :8888' 2>/dev/null |
        grep -q .
}


cleanup() {
    if [ "$STOPPING" -eq 1 ]; then
        return
    fi

    STOPPING=1
    trap - INT TERM

    echo

    if [ "$AGENT_STARTED_BY_SCRIPT" -eq 1 ] &&
       [ -n "$AGENT_PID" ] &&
       kill -0 "$AGENT_PID" 2>/dev/null; then

        echo "正在停止本腳本啟動的 MicroXRCEAgent：$AGENT_PID"

        kill -TERM "$AGENT_PID" 2>/dev/null || true

        for _ in 1 2 3 4 5 6 7 8 9 10; do
            if ! kill -0 "$AGENT_PID" 2>/dev/null; then
                break
            fi

            sleep 0.2
        done

        if kill -0 "$AGENT_PID" 2>/dev/null; then
            echo "Agent 未回應 SIGTERM，改用 SIGKILL"
            kill -KILL "$AGENT_PID" 2>/dev/null || true
        fi

        wait "$AGENT_PID" 2>/dev/null || true
        echo "MicroXRCEAgent 已停止"
    else
        echo "監控已停止；既有 MicroXRCEAgent 保持執行"
    fi
}


resolve_topic() {
    local namespace="$1"
    local direction="$2"
    local base_name="$3"
    local topic_list="$4"

    printf '%s\n' "$topic_list" |
        grep -E "^/${namespace}/fmu/${direction}/${base_name}(_v[0-9]+)?$" |
        sort -V |
        head -n 1
}


subscription_count() {
    local topic="$1"

    ros2 topic info "$topic" 2>/dev/null |
        awk -F': *' '
            /Subscription count/ {
                print $2
                exit
            }
        '
}


publisher_count() {
    local topic="$1"

    ros2 topic info "$topic" 2>/dev/null |
        awk -F': *' '
            /Publisher count/ {
                print $2
                exit
            }
        '
}


trap cleanup INT TERM


# ------------------------------------------------------------
# 1. 確保系統內只有一個 MicroXRCEAgent
# ------------------------------------------------------------

mapfile -t EXISTING_AGENT_PIDS < <(find_agent_pids)

if [ "${#EXISTING_AGENT_PIDS[@]}" -gt 1 ]; then
    echo "錯誤：偵測到多個 MicroXRCEAgent："
    printf '  PID %s\n' "${EXISTING_AGENT_PIDS[@]}"
    echo
    echo "請先關閉舊的 Agent，再重新執行此腳本。"
    exit 1
fi


if port_8888_in_use; then
    if [ "${#EXISTING_AGENT_PIDS[@]}" -eq 1 ]; then
        AGENT_PID="${EXISTING_AGENT_PIDS[0]}"

        echo "偵測到既有 MicroXRCEAgent"
        echo "沿用 PID：$AGENT_PID"
        echo "不重複啟動 UDP 8888"
        echo
    else
        echo "錯誤：UDP 8888 已被其他程序占用，但找不到 MicroXRCEAgent。"
        echo
        ss -H -lunp 'sport = :8888' 2>/dev/null || true
        exit 1
    fi
else
    if [ "${#EXISTING_AGENT_PIDS[@]}" -eq 1 ]; then
        echo "錯誤：MicroXRCEAgent 程序存在，但沒有監聽 UDP 8888。"
        echo "PID：${EXISTING_AGENT_PIDS[0]}"
        echo
        echo "請先停止該程序，再重新執行此腳本。"
        exit 1
    fi

    echo "啟動 MicroXRCEAgent..."

    MicroXRCEAgent udp4 -p 8888 -v 4 \
        >"$LOG_FILE" 2>&1 &

    AGENT_PID=$!
    AGENT_STARTED_BY_SCRIPT=1

    # 最多等待 5 秒確認 Agent 真的開始監聽
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        if ! kill -0 "$AGENT_PID" 2>/dev/null; then
            break
        fi

        if port_8888_in_use; then
            break
        fi

        sleep 0.5
    done

    if ! kill -0 "$AGENT_PID" 2>/dev/null; then
        echo "MicroXRCEAgent 啟動失敗。"
        echo "Log：$LOG_FILE"
        tail -n 30 "$LOG_FILE" 2>/dev/null || true
        exit 1
    fi

    if ! port_8888_in_use; then
        echo "Agent 程序存在，但 UDP 8888 尚未監聽。"
        echo "Log：$LOG_FILE"
        tail -n 30 "$LOG_FILE" 2>/dev/null || true
        cleanup
        exit 1
    fi

    echo "MicroXRCEAgent 啟動成功"
    echo "PID：$AGENT_PID"
    echo
fi


# ------------------------------------------------------------
# 2. 持續檢查三台 PX4 的 DDS 狀態
# ------------------------------------------------------------

while true; do
    if ! kill -0 "$AGENT_PID" 2>/dev/null; then
        echo
        echo "MicroXRCEAgent 已停止"
        echo "PID：$AGENT_PID"

        if [ "$AGENT_STARTED_BY_SCRIPT" -eq 1 ]; then
            echo "Log：$LOG_FILE"
            tail -n 30 "$LOG_FILE" 2>/dev/null || true
        fi

        exit 1
    fi

    clear

    echo "MicroXRCEAgent PID：$AGENT_PID"
    echo "ROS_DOMAIN_ID：$ROS_DOMAIN_ID"

    if [ "$AGENT_STARTED_BY_SCRIPT" -eq 1 ]; then
        echo "Agent 來源：本腳本啟動"
        echo "Agent log：$LOG_FILE"
    else
        echo "Agent 來源：既有程序"
    fi

    echo

    TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
    if [ -z "$TOPIC_LIST" ]; then
        echo "[NG] ros2 topic list 沒有回傳任何 topic"
        echo "     可能是 PX4 尚未連上 Agent，或 ROS 2 graph 尚未發現 topic"
        echo
    fi
    ALL_READY=1

    for MAV in MAV1 MAV2 MAV3; do
        echo "========== $MAV =========="

        MAV_READY=1

        # ROS 2 -> PX4：Offboard 實際需要使用的輸入
        for BASE_NAME in \
            offboard_control_mode \
            trajectory_setpoint \
            vehicle_command
        do
            TOPIC="$(
                resolve_topic \
                    "$MAV" \
                    "in" \
                    "$BASE_NAME" \
                    "$TOPIC_LIST"
            )"

            if [ -z "$TOPIC" ]; then
                echo "[NG] 找不到：/$MAV/fmu/in/$BASE_NAME"
                MAV_READY=0
                continue
            fi

            COUNT="$(subscription_count "$TOPIC")"
            COUNT="${COUNT:-0}"

            if [ "$COUNT" -gt 0 ] 2>/dev/null; then
                echo "[OK] $TOPIC"
                echo "     PX4 subscription：$COUNT"
            else
                echo "[NG] $TOPIC 存在，但 PX4 沒有訂閱"
                MAV_READY=0
            fi
        done

        # PX4 -> ROS 2：確認不只是 Topic 存在，而是真的有資料
        STATUS_TOPIC="$(
            resolve_topic \
                "$MAV" \
                "out" \
                "vehicle_status" \
                "$TOPIC_LIST"
        )"

        if [ -z "$STATUS_TOPIC" ]; then
            echo "[NG] 找不到 VehicleStatus Topic"
            MAV_READY=0
        else
            PUB_COUNT="$(publisher_count "$STATUS_TOPIC")"
            PUB_COUNT="${PUB_COUNT:-0}"

            if [ "$PUB_COUNT" -le 0 ] 2>/dev/null; then
                echo "[NG] $STATUS_TOPIC 存在，但沒有 PX4 publisher"
                MAV_READY=0
            elif timeout 3 \
                ros2 topic echo \
                "$STATUS_TOPIC" \
                --once \
                --qos-reliability best_effort \
                --qos-durability volatile \
                >/dev/null 2>&1; then

                echo "[OK] 收到狀態資料：$STATUS_TOPIC"
                echo "     PX4 publisher：$PUB_COUNT"
            else
                echo "[NG] Topic 存在且有 publisher，但收不到資料：$STATUS_TOPIC"
                MAV_READY=0
            fi
        fi

        if [ "$MAV_READY" -eq 1 ]; then
            echo "結果：$MAV DDS READY"
        else
            echo "結果：$MAV DDS NOT READY"
            ALL_READY=0
        fi

        echo
    done

    if [ "$ALL_READY" -eq 1 ]; then
        echo "=============================================="
        echo "三台 PX4 的 DDS 通訊全部 READY"
        echo "現在可以啟動三機 Offboard 控制程式"
        echo "=============================================="
    else
        echo "=============================================="
        echo "尚未全部 READY，先不要起飛"
        echo "請查看上方是哪一台、哪一個 Topic 失敗"
        echo "=============================================="
    fi

    echo
    echo "持續更新中；按 Ctrl+C 停止監控"

    sleep 2
done