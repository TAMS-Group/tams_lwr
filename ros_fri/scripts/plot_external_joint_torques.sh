# plot current estimated external joint-torques
# (efforts) on the LWR arm
#
rosrun rqt_plot rqt_plot \
/lwr/estimatedExternalJointTorques/effort[0] \
/lwr/estimatedExternalJointTorques/effort[1] \
/lwr/estimatedExternalJointTorques/effort[2] \
/lwr/estimatedExternalJointTorques/effort[3] \
/lwr/estimatedExternalJointTorques/effort[4] \
/lwr/estimatedExternalJointTorques/effort[5] \
/lwr/estimatedExternalJointTorques/effort[6]
