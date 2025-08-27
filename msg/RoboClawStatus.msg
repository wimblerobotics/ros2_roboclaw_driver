# MIT License
# Author: Michael Wimble <mike@wimblerobotics.com>
# Extended RoboClaw status message (redesign rev1)
# NOTE: This supersedes the earlier, smaller status message.

# Motor 1 PID config
float32 m1_p
float32 m1_i
float32 m1_d
uint32  m1_qpps
# Motor 1 dynamic data
int32   m1_speed_command           # last commanded speed (qpps)
int32   m1_speed_measured          # measured encoder speed (qpps)
float32 m1_current_instant         # instantaneous current (A)
float32 m1_current_avg             # exponentially averaged current (A)
uint32  m1_encoder_value           # absolute encoder count
uint8   m1_encoder_status          # raw status bits from encoder read command

# Motor 2 PID config
float32 m2_p
float32 m2_i
float32 m2_d
uint32  m2_qpps
# Motor 2 dynamic data
int32   m2_speed_command
int32   m2_speed_measured
float32 m2_current_instant
float32 m2_current_avg
uint32  m2_encoder_value
uint8   m2_encoder_status

# Board wide electrical & thermal
float32 main_battery_voltage
float32 logic_battery_voltage
float32 temperature1
float32 temperature2

# Error & warning bitfield direct from RoboClaw (32 bits)
uint32  error_bits
# JSON map of bits set for human readability
string  error_json

# Safety & estop
bool    safety_enabled
uint8   safety_state        # 0=OK,1=DECEL,2=ESTOP
string[] active_estop_sources
string[] estop_reasons

# Driver health & statistics
uint32  crc_error_count
uint32  io_error_count
uint32  retry_count
float32 last_command_age     # seconds since last cmd_vel received
float32 avg_loop_period      # average sensor poll loop period (s)

# Timestamp for when snapshot taken (redundant with header stamp in some usage)
builtin_interfaces/Time stamp