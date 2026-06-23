#!/system/bin/sh
# customize.sh - install-time detection script for Magisk module
#
# Check if the device has the capacity_raw node.
# If not, the module is not applicable to the device.

DEVICE_CHECK=/sys/class/power_supply/bms/capacity_raw

if [ ! -f "$DEVICE_CHECK" ]; then
  ui_print  ''
  ui_print  'This module is not applicable to your device'
  ui_print  'Your device is missing the BMS battery sysfs interface'
  ui_print  ''
  abort "Error: $1"
fi

# Set module directory permissions
set_perm_recursive $MODPATH 0 0 0755 0644
set_perm_recursive $MODPATH/bat_capacity_fix 0 0 0755 0755
