#!/bin/bash

# 查找 OpenMV 设备路径
find_openmv_path() {
  for device in /sys/bus/usb/devices/*; do
    if [ -f "$device/idVendor" ] && [ -f "$device/idProduct" ]; then
      vendor=$(cat "$device/idVendor")
      product=$(cat "$device/idProduct")
      if [ "$vendor" = "37c5" ] && [ "$product" = "1204" ]; then
        echo "OpenMV device found at: $device"
        return 0
      fi
    fi
  done
  echo "OpenMV device not found."
  return 1
}

find_openmv_path