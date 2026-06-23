#!/system/bin/sh
# service.sh - service script for Magisk module bat_capacity_correct
#
# Check if the device has the capacity_raw node.
# If not, the module is not applicable to the device.

MODDIR=${0%/*}

capacity_raw=/sys/class/power_supply/bms/capacity_raw
capacity=/sys/class/power_supply/bms/capacity
status=/sys/class/power_supply/battery/status

LOGFILE=/data/local/tmp/bat_correct.log

run_binary(){
  if [[ -f $capacity ]] && [[ -f $capacity_raw ]]; then
    chmod 777 $capacity
    # stderr → 日志文件, stdout 丢弃
    nohup $MODDIR/bat_capacity_correct 2>> $LOGFILE 1>/dev/null &
    echo "bat_capacity_correct: service launched, log → $LOGFILE"
  fi
}

run_binary