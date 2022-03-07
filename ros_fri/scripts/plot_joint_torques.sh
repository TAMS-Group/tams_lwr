# plot current measured joint_state efforts on the LWR arm
rosrun rqt_plot rqt_plot \
/lwr/joint_states/effort[0] \
/lwr/joint_states/effort[1] \
/lwr/joint_states/effort[2] \
/lwr/joint_states/effort[3] \
/lwr/joint_states/effort[4] \
/lwr/joint_states/effort[5] \
/lwr/joint_states/effort[6]
