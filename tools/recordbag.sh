#!/usr/bin/env sh
# The script is used for recording data for QNX trucks.
# Please define the VEHICLE_NAME accordingly
# Sample usage: sh recordbag.sh VEHICLE_NAME
if [ $# -eq 0 ]; then
  vehicle_name="j7-l4e-b0005"
else
  vehicle_name=$1
fi

rosbag record -O ./`TZ=GMT date "+%Y%m%dT%H%M%S"`_${vehicle_name}_0.bag \
/system_metrics \
/notes \
 \
/app_watchdog/latency_report \
/bumper_radar/radar_can_node/latency_report \
/controller_ros_node/latency_report \
/faw_can_node/latency_report \
/front_cameras/latency_report \
/fusion_object_tracker/latency_report \
/lane_detector/latency_report \
/left_radar/radar_can_node/latency_report \
/localization/latency_report \
/prediction/latency_report \
/right_radar/radar_can_node/latency_report \
/scenario_planner/latency_report \
/side_rear_cameras/latency_report \
/ublox/latency_report \
 \
/watchdog/current_state \
 \
/bumper_radar/status_report \
/control/status_report \
/lane/status_report \
/left_radars/status_report \
/localization/status_report \
/obstacle/status_report \
/planning/status_report \
/prediction/status_report \
/right_radars/status_report \
/ublox/status_report \
/vehicle/status_report \
/watchdog/status_report \
\
/imu/data \
/localization/gnss \
/localization/odometry \
/localization/state \
/navsat/odom \
/novatel_data/inspva \
/novatel_data/inspvax \
/ublox/esfalg_debug \
/ublox/esfstatus_debug \
/ublox/navpvt \
/ublox/nav_imu \
 \
/perception/lane_path \
/perception/obstacles \
/planning/lead_info \
/planning/trajectory \
/prediction/obstacles \
/vehicle/button_status \
/vehicle/control_cmd \
/vehicle/dbw_reports \
/vehicle/misc_1_report \
/vehicle/status \
/vehicle/engage_cmd \
/MiranSensorInfo \
/SteeringWheelSensorInfo \
 \
-e "/(.*)/runtime"
