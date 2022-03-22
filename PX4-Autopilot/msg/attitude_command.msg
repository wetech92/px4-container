uint64 timestamp		# time since system start (microseconds)

float32 roll_body_cmd		# body angle in NED frame (can be NaN for FW)
float32 pitch_body_cmd		# body angle in NED frame (can be NaN for FW)
float32 yaw_body_cmd		# body angle in NED frame (can be NaN for FW)

float32 yaw_sp_move_rate_cmd	# rad/s (commanded by user)

# For quaternion-based attitude control
float32[4] q_d_cmd		# Desired quaternion for quaternion control

# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32[3] thrust_body_cmd		# Normalized thrust command in body NED frame [-1,1]

bool roll_reset_integral_cmd	# Reset roll integral part (navigation logic change)
bool pitch_reset_integral_cmd	# Reset pitch integral part (navigation logic change)
bool yaw_reset_integral_cmd	# Reset yaw integral part (navigation logic change)

bool fw_control_yaw_cmd	# control heading with rudder (used for auto takeoff on runway)

