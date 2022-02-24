/** find_kuka_base_transform.cpp
 *
 * simple utility to find the transform from the weird coordinate
 * system used by FRI to the coordinates we use in the LWR URDF.
 *
 * 2017.01.18 - created
 *
 * (C) 2017 fnh, hendrich@informatik.uni-hamburg.de
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

void printTransform(tf::Transform t)
{
    // print matrix, order is r00 r01 r02 tx  r10 r11 r12 ty  r20 r21 r22 tz
    // crazy Bullet idiots don't provide element accessor
    char c = ' ';
    tf::Matrix3x3 rot = t.getBasis();
    tf::Vector3 origin = t.getOrigin();

    printf("\n");
    printf("%10.5lf%c %10.5lf%c %10.5lf%c %10.5lf%c%c", rot.getRow(0)[0], c, rot.getRow(0)[1], c, rot.getRow(0)[2], c,
           origin[0], c, '\n');
    printf("%10.5lf%c %10.5lf%c %10.5lf%c %10.5lf%c%c", rot.getRow(1)[0], c, rot.getRow(1)[1], c, rot.getRow(1)[2], c,
           origin[1], c, '\n');
    printf("%10.5lf%c %10.5lf%c %10.5lf%c %10.5lf%c", rot.getRow(2)[0], c, rot.getRow(2)[1], c, rot.getRow(2)[2], c,
           origin[2], '\n');
}

int main(int argc, char** argv)
{
    // a couple of corresponding matrices...
    // Matrix3x3 constructor expects xx xy xz yz yy yz zx zy zz (row major)

    tf::Transform rotz180(tf::Matrix3x3(-1, 0, 0, 0, -1, 0, 0, 0, 1), tf::Vector3(0, 0, 0));

    tf::Transform rotx180(tf::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1), tf::Vector3(0, 0, 0));

    tf::Transform roty180(tf::Matrix3x3(-1, 0, 0, 0, 1, 0, 0, 0, -1), tf::Vector3(0, 0, 0));

    tf::Transform kuka1(tf::Matrix3x3(0.5626, 0.1143, -0.8188, 0.2283, 0.9304, 0.2867, 0.7946, -0.3482, 0.4974),
                        tf::Vector3(-0.478, -0.0758, 1.1451));
    tf::Transform ros1(tf::Matrix3x3(-0.11405, 0.56244, 0.8189, -0.930, 0.2289, -0.2868, -0.3488, -0.7945, 0.4971),
                       tf::Vector3(0.478, -0.076, 1.145));

    printf("Kuka1:");
    printTransform(kuka1);
    printf("ROS1:");
    printTransform(ros1);

    printf("z180* kuka1\n");
    printTransform(rotz180 * kuka1);
    printf("kuka1 * z180\n");
    printTransform(kuka1 * rotz180);
    printf("x180* kuka1");
    printTransform(rotx180 * kuka1);
    printf("kuka1 * x180");
    printTransform(kuka1 * rotx180);
    printf("kuka1 * x180 * z180");
    printTransform(kuka1 * rotx180 * rotz180);

    printf("kuka1 * x180 * y180");
    printTransform(kuka1 * rotx180 * roty180);

    tf::Transform kuka_ros1 = rotz180 * kuka1 * ros1.inverse();
    printTransform(kuka_ros1);

    tf::Transform kuka2(tf::Matrix3x3(0.5582, 0.6001, 0.5729, -0.1343, 0.7468, -0.6514, -0.8188, 0.2867, 0.4974),
                        tf::Vector3(-0.480, 0.0743, 1.1426));
    tf::Transform ros2(tf::Matrix3x3(0.1352, -0.74631, -0.65174, 0.55726, 0.60113, -0.57280, 0.81926, -0.28578, 0.49713),
                       tf::Vector3(0.480, -0.074, 1.143));

    tf::Transform kuka_ros2 = rotz180 * kuka2 * ros2.inverse();
    printTransform(kuka_ros2);

    tf::Transform kuka3(tf::Matrix3x3(0.4878, 0.8601, -0.1493, 0.4642, -0.4004, -0.7901, -0.7393, 0.3161, -0.5946),
                        tf::Vector3(-0.8576, -0.1116, 0.4699));
    tf::Transform ros3(tf::Matrix3x3(-0.4634, 0.40017, -0.79065, 0.48797, 0.86, 0.14926, 0.73969, -0.31665, -0.5938),
                       tf::Vector3(0.86, 0.112, 0.47));

    tf::Transform kuka_ros3 = rotz180 * kuka3 * ros3.inverse();
    printTransform(kuka_ros3);
}
