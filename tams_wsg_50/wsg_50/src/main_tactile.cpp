/*
 * WSG 50 ROS NODE
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Tactile Sensor support
 * Copyright (c) 2015, Hannes Bistry, University of Hamburg
 *
 * Catkin'ized version and fixes
 * Copyright (c) 2016, Norman Hendrich, University of Hamburg
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Marc Benetó (mbeneto@robotnik.es)
 * \brief WSG-50 ROS driver.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wsg_50/common.h"
#include "wsg_50/cmd.h"

#include <ros/ros.h>
#include "wsg_50_common/Status.h"
#include "wsg_50_common/Move.h"
#include "std_srvs/Empty.h"
#include "wsg_50_common/Conf.h"
#include "wsg_50_common/Incr.h"

#include "sensor_msgs/JointState.h"
#include "wsg_50_common/Sensor.h"

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

bool OBJECT_GRASPPED;
status_t LAST_IMMEDIATE_RESPONSE;
status_t LAST_ASYNC_RESPONSE;

int SOFTSTOP = 0;  // whether issue a stop command on touch sensor contact

int ack_fault();

float convert(unsigned char* b)
{
    float tmp;
    unsigned int src = 0;

    /*
    dbgPrint("b[3]=%x\n", b[3]);
    dbgPrint("b[2]=%x\n", b[2]);
    dbgPrint("b[1]=%x\n", b[1]);
    dbgPrint("b[0]=%x\n", b[0]);
    */

    src = b[3] * 16777216 + b[2] * 65536 + b[1] * 256 + b[0];

    memcpy(&tmp, &src, sizeof tmp);
    // printf("Converted value: %f \n", tmp);

    return tmp;
}

//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Note: Argument values that are outside the gripper’s physical limits are clamped to the highest/lowest available value. //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////
// ACTUATION FUNCTIONS //
/////////////////////////

int homing()
{
    status_t status;
    int res;
    unsigned char payload[1];
    unsigned char* resp;
    unsigned int resp_len;

    // Set flags: Homing direction (0: default, 1: widthitive movement, 2: negative movement).
    payload[0] = 0x00;

    // Submit command and wait for response. Push result to stack.
    res = cmd_submit(0x20, payload, 1, true, &resp, &resp_len, &LAST_ASYNC_RESPONSE);

    if (res != 2)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 2)\n", res);
        if (res > 0)
            free(resp);
        return 0;
    }

    // dbgPrint("&resp: %s\n", resp);
    // dbgPrint("&resp_len: %d\n", resp_len);

    // Check response status
    status = cmd_get_response_status(resp);
    LAST_IMMEDIATE_RESPONSE = status;
    free(resp);
    if (status != E_CMD_PENDING)
    {
        dbgPrint("Command HOMING not successful: %s\n", status_to_str(status));
        ack_fault();
        return -1;
    }

    return 0;
}

int move(float width, float speed, bool hold)
{
    status_t status;
    int res;
    unsigned char payload[9];
    unsigned char* resp;
    unsigned int resp_len;

    // Set flags: Absolute movement (bit 0 is 0), stop on block (bit 1 is 1).
    if (!hold)
        payload[0] = 0x02;
    else
        payload[0] = 0x00;

    // Copy target width and speed
    memcpy(&payload[1], &width, sizeof(float));
    memcpy(&payload[5], &speed, sizeof(float));

    // Submit command and wait for response. Push result to stack.
    dbgPrint("WSG Move issuing cmd\n");
    res = cmd_submit(0x21, payload, 9, true, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    dbgPrint("WSG Move issuing cmd returned\n");
    if (res != 2)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 2)\n", res);
        if (res > 0)
            free(resp);
        return 0;
    }

    // Check response status
    dbgPrint("WSG Getting Response Status\n");
    status = cmd_get_response_status(resp);
    dbgPrint("WSG Gotten Response Status\n");
    free(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command MOVE not successful: %s\n", status_to_str(status));
        ack_fault();
        return -1;
    }

    return status;
}

int stop()
{
    status_t status;
    int res;
    unsigned char payload[1];
    unsigned char* resp;
    unsigned int resp_len;

    // payload[0] = 0x00;

    // Submit command and wait for response. Push result to stack.
    res = cmd_submit(0x22, payload, 0, true, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 2)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 2)\n", res);
        if (res > 0)
            free(resp);
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    free(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command STOP not successful: %s\n", status_to_str(status));
        return -1;
    }

    return status;
}

int ack_fault()
{
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char* resp;
    unsigned int resp_len;

    payload[0] =
        0x61;  // MBJ: Està ben enviat, si es posa alrevés no torna error en terminal però si que es posa roig el LED
    payload[1] = 0x63;
    payload[2] = 0x6B;

    // Submit command and wait for response. Push result to stack.
    res = cmd_submit(0x24, payload, 3, true, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 2)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 2)\n", res);
        if (res > 0)
            free(resp);
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    free(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command ACK not successful: %s\n", status_to_str(status));
        return -1;
    }
    dbgPrint("Command ACK successful: %s\n", status_to_str(status));

    return 0;
}

int grasp(float objWidth, float speed)
{
    status_t status;
    int res;
    unsigned char payload[8];
    unsigned char* resp;
    unsigned int resp_len;

    // Copy part width and speed
    memcpy(&payload[0], &objWidth, sizeof(float));
    memcpy(&payload[4], &speed, sizeof(float));

    // Submit command and wait for response. Push result to stack.
    res = cmd_submit(0x25, payload, 8, true, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 2)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 2)\n", res);
        if (res > 0)
            free(resp);
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    free(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command GRASP not successful: %s\n", status_to_str(status));
        return -1;
    }

    return (0);
}

int release(float width, float speed)
{
    status_t status;
    int res;
    unsigned char payload[8];
    unsigned char* resp;
    unsigned int resp_len;

    // Copy part width and speed
    memcpy(&payload[0], &width, sizeof(float));
    memcpy(&payload[4], &speed, sizeof(float));

    // Submit command and wait for response. Push result to stack.
    res = cmd_submit(0x26, payload, 8, true, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 2)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 2)\n", res);
        if (res > 0)
            free(resp);
        return -1;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    // In case the command result is not available immediately, e.g. on
    // movement or referencing commands, the gripper module returns
    // a notification that it did understand the received command and
    // started its execution (command pending). However, the result will
    // be sent in an additional packet after the command execution has
    // completed. The immediate response to such an asynchronous
    // command will be a packet with E_CMD_PENDING as status code
    free(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command RELEASE not successful: %s\n", status_to_str(status));
        return -1;
    }

    return 0;
}

///////////////////
// SET FUNCTIONS //
///////////////////

int setAcceleration(float acc)
{
    status_t status;
    int res;
    unsigned char payload[4];
    unsigned char* resp;
    unsigned int resp_len;

    // Copy target width and speed
    memcpy(&payload[0], &acc, sizeof(float));

    // Submit command and wait for response. Push result to stack.
    res = cmd_submit(0x30, payload, 4, true, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 2)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 2)\n", res);
        if (res > 0)
            free(resp);
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    free(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command SET ACCELERATION not successful: %s\n", status_to_str(status));
        return -1;
    }

    return 0;
}

int setGraspingForceLimit(float force)
{
    status_t status;
    int res;
    unsigned char payload[4];
    unsigned char* resp;
    unsigned int resp_len;

    // Copy target width and speed
    memcpy(&payload[0], &force, sizeof(float));

    // Submit command and wait for response. Push result to stack.
    res = cmd_submit(0x32, payload, 4, true, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 2)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 2)\n", res);
        if (res > 0)
            free(resp);
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    free(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command SET GRASPING FORCE LIMIT not successful: %s\n", status_to_str(status));
        return -1;
    }

    return 0;
}

///////////////////
// GET FUNCTIONS //
///////////////////

const char* systemState()
{
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char* resp;
    unsigned int resp_len;

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 3);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x40, payload, 3, false, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 6)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 6)\n", res);
        if (res > 0)
            free(resp);
        return nullptr;
    }

    // Check response status
    status = cmd_get_response_status(resp);

    /*
    dbgPrint("LSB -> resp[0]: %x\n", resp[2]);
    dbgPrint("       resp[1]: %x\n", resp[3]);
    dbgPrint("       resp[2]: %x\n", resp[4]);
    dbgPrint("MSB -> resp[3]: %x\n", resp[5]);
    */

    return getStateValues(resp);

    if (status != E_SUCCESS)
    {
        dbgPrint("Command GET SYSTEM STATE not successful: %s\n", status_to_str(status));
        free(resp);
        return nullptr;
    }

    free(resp);

    // return (int) resp[2]; MBJ
}

int graspingState()
{
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char* resp;
    unsigned int resp_len;

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 3);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x41, payload, 3, false, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 3)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 3)\n", res);
        if (res == 2)
        {
            status = cmd_get_response_status(resp);
            getStateValues(resp);
            dbgPrint("Command ACK not successful: %s\n", status_to_str(status));
        }
        if (res > 0)
            free(resp);
        // return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command GET GRASPING STATE not successful: %s\n", status_to_str(status));
        free(resp);
        return 0;
    }

    free(resp);

    dbgPrint("GRASPING STATUS: %s\n", status_to_str(status));

    return (int)resp[2];
}

int getOpening()
{
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char* resp;
    unsigned int resp_len;
    unsigned char v_result[4];

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 3);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x43, payload, 3, false, &resp, &resp_len, &LAST_ASYNC_RESPONSE);  // 0x43
    if (res != 6)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 3)\n", res);
        if (res == 2)
        {
            status = cmd_get_response_status(resp);
            // getStateValues(resp);
            dbgPrint("Command getOpening not successful: %s\n", status_to_str(status));
        }

        if (res > 0)
            free(resp);
        // ack_fault();
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command GET OPENNING not successful: %s\n", status_to_str(status));
        free(resp);
        return 0;
    }

    v_result[0] = resp[2];
    v_result[1] = resp[3];
    v_result[2] = resp[4];
    v_result[3] = resp[5];

    // dbgPrint("OPENING WIDTH: %f mm\n", convert(vResult));

    free(resp);

    return convert(v_result);

    // return (int) resp[2];
}

int getForce()
{
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char* resp;
    unsigned int resp_len;
    unsigned char v_result[4];

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 3);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x45, payload, 3, false, &resp, &resp_len, &LAST_ASYNC_RESPONSE);  // 0x43
    if (res != 6)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 3)\n", res);
        if (res > 0)
            free(resp);
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command GET SPEED not successful: %s\n", status_to_str(status));
        free(resp);
        return 0;
    }

    v_result[0] = resp[2];
    v_result[1] = resp[3];
    v_result[2] = resp[4];
    v_result[3] = resp[5];

    free(resp);

    return convert(v_result);

    // return (int) resp[2];
}

int getAcceleration()
{
    status_t status;
    int res;
    unsigned char payload[6];
    unsigned char* resp;
    unsigned int resp_len;
    unsigned char v_result[4];

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 1);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x31, payload, 0, false, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 6)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 3)\n", res);
        if (res > 0)
            free(resp);
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command GET ACCELERATION not successful: %s\n", status_to_str(status));
        free(resp);
        return 0;
    }

    v_result[0] = resp[2];
    v_result[1] = resp[3];
    v_result[2] = resp[4];
    v_result[3] = resp[5];

    free(resp);

    return convert(v_result);

    // return (int) resp[2];
}

int getGraspingForceLimit()
{
    status_t status;
    int res;
    unsigned char payload[6];
    unsigned char* resp;
    unsigned int resp_len;
    unsigned char v_result[4];

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 1);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x33, payload, 0, false, &resp, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 6)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 3)\n", res);
        if (res > 0)
            free(resp);
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command GET GRASPING FORCE not successful: %s\n", status_to_str(status));
        free(resp);
        return 0;
    }

    v_result[0] = resp[2];
    v_result[1] = resp[3];
    v_result[2] = resp[4];
    v_result[3] = resp[5];

    free(resp);

    return convert(v_result);

    // return (int) resp[2];
}

int getFingerSensorData(wsg_50_common::Sensor* psensor_msg)
{
    // sensordata0
    // pthread_mutex_lock( &mutex );
    status_t status;
    int res;
    unsigned char payload[1];
    unsigned char* resp0;
    unsigned int resp_len;

    // read sensor data from finger 0
    memset(payload, 0, 1);
    // Submit command and wait for response. Expecting exactly 174 bytes response payload.
    res = cmd_submit(0x63, payload, 1, false, &resp0, &resp_len, &LAST_ASYNC_RESPONSE);
    if (res != 174)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 174)\n", res);
        if (res > 0)
            free(resp0);
        // pthread_mutex_unlock( &mutex );
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp0);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command GET FINGER SENSOR DATA not successful: %s\n", status_to_str(status));
        free(resp0);
        // pthread_mutex_unlock( &mutex );
        return 0;
    }

    // sensordata1
    status_t status1;
    int res1;
    unsigned char payload1[1];
    unsigned char* resp1;
    unsigned int resp_len1;
    // read sensor data from finger 1
    memset(payload1, 1, 1);
    // Submit command and wait for response. Expecting exactly 174 bytes response payload.
    res1 = cmd_submit(0x63, payload1, 1, false, &resp1, &resp_len1, &LAST_ASYNC_RESPONSE);
    if (res1 != 174)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 174)\n", res1);
        if (res1 > 0)
            free(resp1);
        // pthread_mutex_unlock( &mutex );
        return 0;
    }

    // Check response status
    status = cmd_get_response_status(resp1);
    if (status != E_SUCCESS)
    {
        dbgPrint("Command GET FINGER SENSOR DATA not successful: %s\n", status_to_str(status1));
        free(resp1);
        // pthread_mutex_unlock( &mutex );
        return 0;
    }

    // update wsg_50_common::Sensor message
    (*psensor_msg).readoutTime = ros::Time::now();

    int i, j = 0;
    for (i = 2; i < 170; i = i + 2)
    {
        (*psensor_msg).tactileSensor0[j] = resp0[i + 1] * 256 + resp0[i];
        (*psensor_msg).tactileSensor1[j] = resp1[i + 1] * 256 + resp1[i];
        j++;
    }
    free(resp0);
    free(resp1);
    // pthread_mutex_unlock( &mutex );
    return 1;
}

bool moveSrv(wsg_50_common::Move::Request& req, wsg_50_common::Move::Response& res)
{
    // pthread_mutex_lock( &mutex );
    if ((req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0))
    {
        ROS_INFO("Moving to %f position at %f mm/s.", req.width, req.speed);
        res.error = move(req.width, req.speed, false);
    }
    else if (req.width < 0.0 || req.width > 110.0)
    {
        ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
        res.error = 255;
        // pthread_mutex_unlock( &mutex );
        return false;
    }
    else
    {
        ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
        res.error = move(req.width, req.speed, false);
    }

    ROS_INFO("Target position reached.");
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool moveholdSrv(wsg_50_common::Move::Request& req, wsg_50_common::Move::Response& res)
{
    // pthread_mutex_lock( &mutex );
    ros::Time t1 = ros::Time::now();
    if ((req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0))
    {
        ROS_INFO("Moving to %f position at %f mm/s.", req.width, req.speed);
        res.error = move(req.width, req.speed, true);
    }
    else if (req.width < 0.0 || req.width > 110.0)
    {
        ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
        res.error = 255;
        // pthread_mutex_unlock( &mutex );
        return false;
    }
    else
    {
        ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
        res.error = move(req.width, req.speed, true);
    }

    ros::Time t2 = ros::Time::now();
    ros::Duration dt = t2 - t1;
    ROS_INFO("Move Hold Service Issue lasted %6.3f\n", dt.toSec());
    // ROS_INFO("Target position reached.");
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool graspSrv(wsg_50_common::Move::Request& req, wsg_50_common::Move::Response& res)
{
    // pthread_mutex_lock( &mutex );
    if ((req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0))
    {
        ROS_INFO("Grasping object at %f mm %f mm/s.", req.width, req.speed);
        res.error = grasp(req.width, req.speed);
    }
    else if (req.width < 0.0 || req.width > 110.0)
    {
        ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
        res.error = 255;
        // pthread_mutex_unlock( &mutex );
        return false;
    }
    else
    {
        ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: "
                 "[0.1 - 420.0])  Using clamped values.");
        res.error = grasp(req.width, req.speed);
    }

    ROS_INFO("Object grasped correctly.");
    OBJECT_GRASPPED = true;
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool incrementSrv(wsg_50_common::Incr::Request& req, wsg_50_common::Incr::Response& res)
{
    // pthread_mutex_lock( &mutex );
    if (req.direction == "open")
    {
        if (!OBJECT_GRASPPED)
        {
            float current_width = getOpening();
            float next_width = current_width + req.increment;
            if ((current_width < GRIPPER_MAX_OPEN) && next_width < GRIPPER_MAX_OPEN)
            {
                // grasp(nextWidth, 1);
                move(next_width, 20, false);
                current_width = next_width;
            }
            else if (next_width >= GRIPPER_MAX_OPEN)
            {
                // grasp(GRIPPER_MAX_OPEN, 1);
                move(GRIPPER_MAX_OPEN, 1, false);
                current_width = GRIPPER_MAX_OPEN;
            }
        }
        else
        {
            ROS_INFO("Releasing object...");
            release(GRIPPER_MAX_OPEN, 20);
            OBJECT_GRASPPED = false;
        }
    }
    else if (req.direction == "close")
    {
        if (!OBJECT_GRASPPED)
        {
            float current_width = getOpening();
            float next_width = current_width - req.increment;

            if ((current_width > GRIPPER_MIN_OPEN) && next_width > GRIPPER_MIN_OPEN)
            {
                // grasp(nextWidth, 1);
                move(next_width, 20, false);
                current_width = next_width;
            }
            else if (next_width <= GRIPPER_MIN_OPEN)
            {
                // grasp(GRIPPER_MIN_OPEN, 1);
                move(GRIPPER_MIN_OPEN, 1, false);
                current_width = GRIPPER_MIN_OPEN;
            }
        }
    }
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool releaseSrv(wsg_50_common::Move::Request& req, wsg_50_common::Move::Response& res)
{
    // pthread_mutex_lock( &mutex );
    if ((req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0))
    {
        ROS_INFO("Releasing to %f position at %f mm/s.", req.width, req.speed);
        res.error = release(req.width, req.speed);
    }
    else if (req.width < 0.0 || req.width > 110.0)
    {
        ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
        res.error = 255;
        // pthread_mutex_unlock( &mutex );
        return false;
    }
    else
    {
        ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: "
                 "[0.1 - 420.0])  Using clamped values.");
        res.error = release(req.width, req.speed);
    }
    ROS_INFO("Object released correctly.");
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool homingSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
    ROS_INFO("Homing...");
    // pthread_mutex_lock( &mutex );
    homing();
    // pthread_mutex_unlock( &mutex );
    ROS_INFO("Home position reached.");
    return true;
}

bool stopSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
    ROS_WARN("Stop!");
    // pthread_mutex_lock( &mutex );
    stop();
    // pthread_mutex_unlock( &mutex );
    ROS_WARN("Stopped.");
    return true;
}

bool softstopSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
    ROS_WARN("Initialize SoftStop!");
    // pthread_mutex_lock( &mutex );
    SOFTSTOP = 1;
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool cancelsoftstopSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
    ROS_WARN("cancel SoftStop!\n");
    // pthread_mutex_lock( &mutex );
    SOFTSTOP = 0;
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool setAccSrv(wsg_50_common::Conf::Request& req, wsg_50_common::Conf::Response& res)
{
    // pthread_mutex_lock( &mutex );
    setAcceleration(req.val);
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool setForceSrv(wsg_50_common::Conf::Request& req, wsg_50_common::Conf::Response& res)
{
    // pthread_mutex_lock( &mutex );
    setGraspingForceLimit(req.val);
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool ackSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
    // pthread_mutex_lock( &mutex );
    ack_fault();
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool checkforsoftstop(wsg_50_common::Sensor sensor_msg)
{
    bool result = false;
    int thresh = 1;
    int i = 0;
    for (i = 0; i < 84; i++)
    {
        if (sensor_msg.tactileSensor0[i] >= thresh)
            result = true;
        if (sensor_msg.tactileSensor1[i] >= thresh)
            result = true;
    }
    if (result)
        ROS_WARN("Initiating SoftStop as threshold hit!\n");
    if (result)
        stop();
    return result;
}

/**
 * The main function
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "wsg_50", 1);  // 1 = disable ROS sigint handler

    ros::Timer timer;
    ros::NodeHandle nh("wsg_50");  // so that services are properly prefixed
    ros::NodeHandle nnh("~");
    wsg_50_common::Status status_msg;
    sensor_msgs::JointState joint_states;
    wsg_50_common::Sensor sensor_msg;

    std::string ip;
    int port;
    double rate;

    status_t laststate = (status_t)0;
    LAST_ASYNC_RESPONSE = (status_t)0;

    ROS_INFO("WSG-50 - ROS NODE");
    nnh.param("ip", ip, std::string("192.168.0.20"));
    nnh.param("port", port, 1000);
    nnh.param("loop_rate", rate, 10.0);  // max 18 on our WSG50 (firmware version dependent)

    ROS_INFO("WSG-50: ip address %s port %d loop rate %6.2lf", ip.c_str(), port, rate);

    // connect to device using TCP
    //
    if (cmd_connect_tcp(ip.c_str(), port) == 0)
    {
        ROS_INFO("WSG-50: TCP connection established");
        ack_fault();  // keep Lua happy
        usleep(20000);

        homing();
        usleep(20000);

        // setup ROS services
        //
        ros::ServiceServer move_ss = nh.advertiseService("move", moveSrv);
        ros::ServiceServer movehold_ss = nh.advertiseService("movehold", moveholdSrv);
        ros::ServiceServer grasp_ss = nh.advertiseService("grasp", graspSrv);
        ros::ServiceServer release_ss = nh.advertiseService("release", releaseSrv);
        ros::ServiceServer homing_ss = nh.advertiseService("homing", homingSrv);
        ros::ServiceServer stop_ss = nh.advertiseService("stop", stopSrv);
        ros::ServiceServer softstop_ss = nh.advertiseService("softstop", softstopSrv);
        ros::ServiceServer cancelsoftstop_ss = nh.advertiseService("cancelsoftstop", cancelsoftstopSrv);
        ros::ServiceServer ack_ss = nh.advertiseService("ack", ackSrv);

        ros::ServiceServer increment_ss = nh.advertiseService("move_incrementally", incrementSrv);
        ros::ServiceServer set_acc_ss = nh.advertiseService("set_acceleration", setAccSrv);
        ros::ServiceServer set_force_ss = nh.advertiseService("set_force", setForceSrv);

        // publishers
        //
        ros::Publisher state_pub = nh.advertise<wsg_50_common::Status>("status", 1);
        ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        ros::Publisher sensor_pub = nh.advertise<wsg_50_common::Sensor>("tactile", 1);

        // Timer
        // timer = nh.createTimer( ros::Duration( 1.0/100 ), &timerCallback );
        ROS_INFO("WSG-50: Ready to use.");

        int op = 0, acc = 0, force = 0;  // opening in mm, acceleration, force
        int last_op = 0;                 // previous opening in mm, to calculate velocity
        double vel = 0;
        unsigned int iteration = 0;
        int tactile_sensor_result = 0;
        const char * aux;
        ros::Rate loop_rate(rate);  // loop at user-selected rate
        ros::Time last_update_time = ros::Time::now();
        ros::Time last_op_update_time = ros::Time::now();

        while (ros::ok())
        {
            iteration++;
            // ROS_INFO( "WSG iteration %d last async state %s \n", iteration, status_to_str(last_async_response) );

            // Loop waiting for orders and updating the state
            // Create the msg to send
            // Get state values

            ros::Duration dt = ros::Time::now() - last_update_time;
            // if (dt.toSec() > 0.5) {
            while (true)
            {
                last_update_time = ros::Time::now();
                // pthread_mutex_lock( &mutex );

                aux = systemState();
                // dt = ros::Time::now() - lastUpdateTime;
                // ROS_INFO( "Time after get system state  %6.3f \n", dt.toSec() );
                last_op = op;
                op = getOpening();
                vel = (op - last_op) / 2000.0 / (ros::Time::now() - last_op_update_time).toSec();
                last_op_update_time = ros::Time::now();
                dt = ros::Time::now() - last_update_time;
                // ROS_INFO( "Time after get opening  %6.3f \n", dt.toSec() );
                // usleep(20000);
                ros::spinOnce();
                acc = getAcceleration();
                ros::spinOnce();
                force = getForce();

                dt = ros::Time::now() - last_update_time;
                // ROS_INFO( "Time after get force  %6.3f \n", dt.toSec() );
                ros::spinOnce();

                // publish tactile sensor information
                tactile_sensor_result = getFingerSensorData(&sensor_msg);
                if (tactile_sensor_result != 0)
                {
                    sensor_pub.publish(sensor_msg);
                    if (SOFTSTOP > 0)
                        checkforsoftstop(sensor_msg);
                }
                dt = ros::Time::now() - last_update_time;
                // ROS_INFO( "Time after get tactile  %6.3f \n", dt.toSec() );
                ros::spinOnce();
            }  // if (42)...

            if (laststate != LAST_ASYNC_RESPONSE)
            {
                ROS_INFO("Changing Last Async Response to %s.\n", status_to_str(LAST_ASYNC_RESPONSE));
                printf("Changing Last Async Response to %s.\n", status_to_str(LAST_ASYNC_RESPONSE));
                laststate = LAST_ASYNC_RESPONSE;
                // publish message
            }

            joint_states.header.stamp = ros::Time::now();
            joint_states.header.frame_id = "wsg_50_gripper_base_link";
            joint_states.name.resize(2);
            joint_states.name[0] = "wsg_50_gripper_base_joint_gripper_left";
            joint_states.name[1] = "wsg_50_gripper_base_joint_gripper_right";
            joint_states.position.resize(2);
            joint_states.position[0] = -op / 2000.0;  // op is total width in millimeters,
            joint_states.position[1] = op / 2000.0;   // ROS wants meters, and one-half per finger
            joint_states.velocity.resize(2);
            joint_states.velocity[0] = -vel;
            joint_states.velocity[1] = vel;
            joint_states.effort.resize(2);
            joint_states.effort[0] = force;
            joint_states.effort[1] = force;

            joint_states_pub.publish(joint_states);
            ros::spinOnce();
            loop_rate.sleep();

            std::stringstream ss;
            ss << aux;
            status_msg.status = ss.str();
            status_msg.width = op;
            status_msg.acc = acc;
            status_msg.force = force;
            state_pub.publish(status_msg);//only useless crap, better to be implemented as a service instead

            // ROS_INFO( "entering spin\n" );
            ros::spinOnce();
            dt = ros::Time::now() - last_update_time;
            // ROS_INFO( "Time at loop end %6.3f \n", dt.toSec() );
            // ROS_INFO( "spin finished\n" );
        }
    }
    else
    {
        ROS_ERROR("Unable to connect via TCP, please check the port and address used.");
    }

    // Disconnect - won't be executed atm. as the endless loop in test()
    // will never return.
    cmd_disconnect();

    return 0;
}
