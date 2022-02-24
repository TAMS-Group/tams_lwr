# message to trace/debug the RML velocity motion algorithm.
#
Header header
float64[] TargetVelocityVector
float64[] MaxAccelerationVector
#
# RML VIP/VOP planning data
#
float64[] estimatedJointVelocities
float64[] estimatedJointVelocityErrors

#
# actual robot state
#
float64[] commandedJointPositions
float64[] measuredJointPositions
