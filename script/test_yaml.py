import os
import yaml

RAD_TO_SPEED_SETTING = 10/0.561  #ratio of the setting tm in whill to the actual angular vel
METER_TO_SPEED_SETTING = 36 #ratio of setting of linear speed in whill to actual linear vel

config_path =  os.path.join("config", "whill_config.yaml") #path of the config file for share control
with open(config_path, 'r') as f:
    config_param = yaml.safe_load(f)

whill_max_linear_speed = min(max(int(config_param["velocity_by_joystick_control_node"]["ros__parameters"]["max_linear_vel"] * METER_TO_SPEED_SETTING), 8), 60)  # max forward speed
whill_min_linear_speed = min(max(int(abs(int(config_param["velocity_by_joystick_control_node"]["ros__parameters"]["min_linear_vel"] * METER_TO_SPEED_SETTING))), 8), 60)  # max backward speed
whill_max_yaw_rate = min(max(int(config_param["velocity_by_joystick_control_node"]["ros__parameters"]["max_yaw_rate"] * RAD_TO_SPEED_SETTING), 8), 60)  # max angular speed
whill_max_acceleration = min(max(int(config_param["velocity_by_joystick_control_node"]["ros__parameters"]["max_acceleration"] * METER_TO_SPEED_SETTING), 10), 90)  # Max forward acceleration (backward deceleration)
whill_max_deceleration = min(max(int(config_param["velocity_by_joystick_control_node"]["ros__parameters"]["max_decceleration"] * METER_TO_SPEED_SETTING), 10), 160)  # Max forward deceleration (backward acceleration)
whill_max_yaw_acceleration = min(max(int(config_param["velocity_by_joystick_control_node"]["ros__parameters"]["max_yaw_acceleration"] * RAD_TO_SPEED_SETTING), 10), 90)  # Max yaw rate acceleration

service_command = [
        'ros2 ', 'service ', 'call ', '/whill/set_speed_profile_srv ', 'ros2_whill_interfaces/SetSpeedProfile ',
        "{s1: 4, fm1: " + str(whill_max_linear_speed) + ", fa1: " + str(whill_max_acceleration) + ", fd1: " + str(whill_max_deceleration) +
        ", rm1: " + str(whill_min_linear_speed) + ", ra1: " + str(whill_max_deceleration) + ", rd1: " + str(whill_max_acceleration) +
        ", tm1: " + str(whill_max_yaw_rate) + ", ta1: " + str(whill_max_yaw_acceleration) + ", td1: " + str(whill_max_yaw_acceleration) + "}"
    ]
print(service_command)