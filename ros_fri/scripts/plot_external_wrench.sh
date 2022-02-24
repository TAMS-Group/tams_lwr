# plot current measured joint_state efforts on the LWR arm
#
rosrun rqt_plot rqt_plot \
/lwr/estimatedExternalWrench/wrench/force/x \
/lwr/estimatedExternalWrench/wrench/force/y \
/lwr/estimatedExternalWrench/wrench/force/z \
/lwr/estimatedExternalWrench/wrench/torque/x \
/lwr/estimatedExternalWrench/wrench/torque/y \
/lwr/estimatedExternalWrench/wrench/torque/z
