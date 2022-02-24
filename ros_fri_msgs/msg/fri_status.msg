# fri_status.msg - the full status message from KuKA FRI library;
# includes robot control mode, robot data, robot errors,
# and some ROS specific diagnostics.
#
# Note that the robot measures position and torques/efforts,
# but we have to calculate velocity ourselves.
#
Header header
string tcp_frame
#
float64   friCycleTime
uint32    friMode
uint32    friCurrentControlScheme
bool      friMachineOK
bool      anyDriveError
bool      anyDriveWarning
float32[] driveTemperatures
#
string[]  jointNames
float32[] measuredJointPositions
float32[] measuredJointTorques
float32[] estimatedExternalJointTorques
#
float32[] estimatedJointVelocities
float32[] estimatedJointPositionErrors
float32[] estimatedJointVelocityErrors
#
geometry_msgs/PoseStamped measuredCartPose
geometry_msgs/PoseStamped measuredCartPoseOffsets
#
float32[] commandedJointPositions
float32[] commandedJointPositionOffsets
float32[] commandedJointTorques
float32[] commandedJointStiffness
float32[] commandedJointDamping
#
geometry_msgs/Vector3  gravityVector
geometry_msgs/PoseStamped commandedCartPose
geometry_msgs/PoseStamped commandedCartPoseOffsets
#
geometry_msgs/WrenchStamped estimatedExternalWrench
geometry_msgs/WrenchStamped commandedWrench
#
uint32 FIR_STATE_OFF = 0
uint32 FRI_STATE_MON = 1
uint32 FRI_STATe_CMD = 2
uint32 JOINT_POSITION_CONTROL = 10
uint32 CART_IMPEDANCE_CONTROL = 20
uint32 JOINT_IMPEDANCE_CONTROL = 30
uint32 JOINT_TORQUE_CONTROL = 90
