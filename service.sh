#!/system/bin/sh
# service.sh - service script for Magisk module bat_capacity_correct
#
# Check if the device has the capacity_raw node.
# If not, the module is not applicable to the device.

MODDIR=${0%/*}

capacity_raw=/sys/class/power_supply/bms/capacity_raw
capacity=/sys/class/power_supply/bms/capacity
status=/sys/class/power_supply/battery/status

run_binary(){
  if [[ -f $capacity ]] && [[ -f $capacity_raw ]]; then
    chmod 777 $capacity
    nohup $MODDIR/bat_capacity_correct > /dev/null 2>&1 &
  fi
}

run_binary
