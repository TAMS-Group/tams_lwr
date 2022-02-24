/*
 * WSG 50 ROS NODE
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
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

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "wsg_50/common.h"
#include "wsg_50/cmd.h"
//#include "wsg_50/functions.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "wsg_50_common/Status.h"
#include "wsg_50_common/Move.h"
#include "std_srvs/Empty.h"
#include "wsg_50_common/Conf.h"
#include "wsg_50_common/Incr.h"

#include "sensor_msgs/JointState.h"
#include "wsg_50_common/Sensor.h"

//------------------------------------------------------------------------
// Local macros
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------

float increment;
bool objectGraspped;
status_t last_immediate_response;
status_t last_async_response;

int error_count;
int softstop = 0;  // whether issue a stop command on touch sensor contact

// pthread_mutex_t             mutex;
// unsigned int n_timer_callbacks = 0;
// sensor_msgs::JointState joint_states;
// ros::Publisher joint_states_pub;

//------------------------------------------------------------------------
// Unit testing
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

int ack_fault(void);

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

int homing(void)
{
    status_t status;
    int res;
    unsigned char payload[1];
    unsigned char* resp;
    unsigned int resp_len;

    // Set flags: Homing direction (0: default, 1: widthitive movement, 2: negative movement).
    payload[0] = 0x00;

    // Submit command and wait for response. Push result to stack.
    res = cmd_submit(0x20, payload, 1, true, &resp, &resp_len, &last_async_response);

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
    last_immediate_response = status;
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
    res = cmd_submit(0x21, payload, 9, true, &resp, &resp_len, &last_async_response);
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

int stop(void)
{
    status_t status;
    int res;
    unsigned char payload[1];
    unsigned char* resp;
    unsigned int resp_len;

    // payload[0] = 0x00;

    // Submit command and wait for response. Push result to stack.
    res = cmd_submit(0x22, payload, 0, true, &resp, &resp_len, &last_async_response);
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

int ack_fault(void)
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
    res = cmd_submit(0x24, payload, 3, true, &resp, &resp_len, &last_async_response);
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
    res = cmd_submit(0x25, payload, 8, true, &resp, &resp_len, &last_async_response);
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
    res = cmd_submit(0x26, payload, 8, true, &resp, &resp_len, &last_async_response);
    if (res != 2)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 2)\n", res);
        if (res > 0)
            free(resp);
        return -1;
    }

    // Check response status
    status = cmd_get_response_status(resp);
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
    res = cmd_submit(0x30, payload, 4, true, &resp, &resp_len, &last_async_response);
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
    res = cmd_submit(0x32, payload, 4, true, &resp, &resp_len, &last_async_response);
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

const char* systemState(void)
{
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char* resp;
    unsigned int resp_len;

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 3);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x40, payload, 3, false, &resp, &resp_len, &last_async_response);
    if (res != 6)
    {
        dbgPrint("Response payload length doesn't match (is %d, expected 6)\n", res);
        if (res > 0)
            free(resp);
        return 0;
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
        return 0;
    }

    free(resp);

    // return (int) resp[2]; MBJ
}

int graspingState(void)
{
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char* resp;
    unsigned int resp_len;

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 3);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x41, payload, 3, false, &resp, &resp_len, &last_async_response);
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

int getOpening(void)
{
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char* resp;
    unsigned int resp_len;
    unsigned char vResult[4];

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 3);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x43, payload, 3, false, &resp, &resp_len, &last_async_response);  // 0x43
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

    vResult[0] = resp[2];
    vResult[1] = resp[3];
    vResult[2] = resp[4];
    vResult[3] = resp[5];

    // dbgPrint("OPENING WIDTH: %f mm\n", convert(vResult));

    free(resp);

    return convert(vResult);

    // return (int) resp[2];
}

int getForce(void)
{
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char* resp;
    unsigned int resp_len;
    unsigned char vResult[4];

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 3);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x45, payload, 3, false, &resp, &resp_len, &last_async_response);  // 0x43
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

    vResult[0] = resp[2];
    vResult[1] = resp[3];
    vResult[2] = resp[4];
    vResult[3] = resp[5];

    free(resp);

    return convert(vResult);

    // return (int) resp[2];
}

int getAcceleration(void)
{
    status_t status;
    int res;
    unsigned char payload[6];
    unsigned char* resp;
    unsigned int resp_len;
    unsigned char vResult[4];

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 1);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x31, payload, 0, false, &resp, &resp_len, &last_async_response);
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

    vResult[0] = resp[2];
    vResult[1] = resp[3];
    vResult[2] = resp[4];
    vResult[3] = resp[5];

    free(resp);

    return convert(vResult);

    // return (int) resp[2];
}

int getGraspingForceLimit(void)
{
    status_t status;
    int res;
    unsigned char payload[6];
    unsigned char* resp;
    unsigned int resp_len;
    unsigned char vResult[4];

    // Don't use automatic update, so the payload bytes are 0.
    memset(payload, 0, 1);

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = cmd_submit(0x33, payload, 0, false, &resp, &resp_len, &last_async_response);
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

    vResult[0] = resp[2];
    vResult[1] = resp[3];
    vResult[2] = resp[4];
    vResult[3] = resp[5];

    free(resp);

    return convert(vResult);

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
    res = cmd_submit(0x63, payload, 1, false, &resp0, &resp_len, &last_async_response);
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
    res1 = cmd_submit(0x63, payload1, 1, false, &resp1, &resp_len1, &last_async_response);
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

    ROS_INFO("Target position reached.");
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
    objectGraspped = true;
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool incrementSrv(wsg_50_common::Incr::Request& req, wsg_50_common::Incr::Response& res)
{
    // pthread_mutex_lock( &mutex );
    if (req.direction == "open")
    {
        if (!objectGraspped)
        {
            float currentWidth = getOpening();
            float nextWidth = currentWidth + req.increment;
            if ((currentWidth < GRIPPER_MAX_OPEN) && nextWidth < GRIPPER_MAX_OPEN)
            {
                // grasp(nextWidth, 1);
                move(nextWidth, 20, false);
                currentWidth = nextWidth;
            }
            else if (nextWidth >= GRIPPER_MAX_OPEN)
            {
                // grasp(GRIPPER_MAX_OPEN, 1);
                move(GRIPPER_MAX_OPEN, 1, false);
                currentWidth = GRIPPER_MAX_OPEN;
            }
        }
        else
        {
            ROS_INFO("Releasing object...");
            release(GRIPPER_MAX_OPEN, 20);
            objectGraspped = false;
        }
    }
    else if (req.direction == "close")
    {
        if (!objectGraspped)
        {
            float currentWidth = getOpening();
            float nextWidth = currentWidth - req.increment;

            if ((currentWidth > GRIPPER_MIN_OPEN) && nextWidth > GRIPPER_MIN_OPEN)
            {
                // grasp(nextWidth, 1);
                move(nextWidth, 20, false);
                currentWidth = nextWidth;
            }
            else if (nextWidth <= GRIPPER_MIN_OPEN)
            {
                // grasp(GRIPPER_MIN_OPEN, 1);
                move(GRIPPER_MIN_OPEN, 1, false);
                currentWidth = GRIPPER_MIN_OPEN;
            }
        }
    }
    // pthread_mutex_unlock( &mutex );
    return true;  // FNH: FIXME
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
    softstop = 1;
    // pthread_mutex_unlock( &mutex );
    return true;
}

bool cancelsoftstopSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
    ROS_WARN("cancel SoftStop!\n");
    // pthread_mutex_lock( &mutex );
    softstop = 0;
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

/*
void timerCallback( const ros::TimerEvent& timerEvent ) {
 // TimerEvent :->  Time ->sec,nscec
  n_timer_callbacks++;
  if ((n_timer_callbacks < 5) || ((n_timer_callbacks % (60*100)) == 0)) { // once per minute at 100Hz
    ROS_INFO( "WSG50 timerCallback at %d, %d", timerEvent.current_real.sec, n_timer_callbacks );
  }

  joint_states.header.stamp = ros::Time::now();
  joint_states_pub.publish( joint_states );


  unsigned int N = jointNames.size();
  sensor_msgs::JointState jsmsg;
  jsmsg.header.seq = n_timer_callbacks;
  jsmsg.header.stamp = ros::Time::now();
  jsmsg.header.frame_id = std::string( "doro_joint_states" );

  jsmsg.name.resize( N );
  jsmsg.position.resize( N );
  jsmsg.velocity.resize( N );
  jsmsg.effort.resize( N );
  for( unsigned int i=0; i < N; i++ ) {
    jsmsg.name[i]     = jointNames[i];
    jsmsg.position[i] = jointAngles[i];
    jsmsg.velocity[i] = jointVelocities[i];
    jsmsg.effort[i]   = jointTorques[i];
  }
  jointStatePublisher.publish( jsmsg );

}
*/

/**
 * The main function
 */

int main(int argc, char** argv)
{
    // pthread_mutex_init(&mutex,NULL);
    ros::init(argc, argv, "wsg_50", 1);

    ros::Timer timer;
    ros::NodeHandle nh;
    wsg_50_common::Status status_msg;
    sensor_msgs::JointState joint_states;
    wsg_50_common::Sensor sensor_msg;

    std::string ip;
    int port;

    status_t laststate = (status_t)0;
    last_async_response = (status_t)0;

    ROS_INFO("WSG 50 - ROS NODE");

    nh.param("wsg_50_tcp/ip", ip, std::string("192.168.0.20"));
    nh.param("wsg_50_tcp/port", port, 1000);

    // Connect to device using TCP
    if (cmd_connect_tcp(ip.c_str(), port) == 0)
    {
        ROS_INFO("TCP connection stablished");

        ack_fault();

        // homing();

        // Services
        ros::ServiceServer moveSS = nh.advertiseService("wsg_50/move", moveSrv);
        ros::ServiceServer moveholdSS = nh.advertiseService("wsg_50/movehold", moveholdSrv);
        ros::ServiceServer graspSS = nh.advertiseService("wsg_50/grasp", graspSrv);
        ros::ServiceServer releaseSS = nh.advertiseService("wsg_50/release", releaseSrv);
        ros::ServiceServer homingSS = nh.advertiseService("wsg_50/homing", homingSrv);
        ros::ServiceServer stopSS = nh.advertiseService("wsg_50/stop", stopSrv);
        ros::ServiceServer softstopSS = nh.advertiseService("wsg_50/softstop", softstopSrv);
        ros::ServiceServer cancelsoftstopSS = nh.advertiseService("wsg_50/cancelsoftstop", cancelsoftstopSrv);
        ros::ServiceServer ackSS = nh.advertiseService("wsg_50/ack", ackSrv);

        ros::ServiceServer incrementSS = nh.advertiseService("wsg_50/move_incrementally", incrementSrv);

        ros::ServiceServer setAccSS = nh.advertiseService("wsg_50/set_acceleration", setAccSrv);
        ros::ServiceServer setForceSS = nh.advertiseService("wsg_50/set_force", setForceSrv);

        // Publisher
        ros::Publisher state_pub = nh.advertise<wsg_50_common::Status>("wsg_50/status", 1);
        ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
        ros::Publisher sensor_pub = nh.advertise<wsg_50_common::Sensor>("wsg_50/sensor", 1);

        // Timer
        // timer = nh.createTimer( ros::Duration( 1.0/100 ), &timerCallback );

        ROS_INFO("Ready to use.");

        ros::Rate loop_rate(5);        // loop at 100 Hz
        ros::Duration wsgInterval(1);  // update WSG values at 1 Hz
        ros::Time lastUpdateTime = ros::Time::now();

        // const char * aux;
        int op = 0, acc = 0, force = 0;  // opening in mm, acceleration, force

        unsigned int iteration = 0;
        int tactileSensorResult = 0;
        while (ros::ok())
        {
            // Loop waiting for orders and updating the state
            // Create the msg to send
            // Get state values
            iteration++;
            // ROS_INFO( "WSG iteration %d last async state %s \n", iteration, status_to_str(last_async_response) );

            ros::Duration dt = ros::Time::now() - lastUpdateTime;
            // if (dt.toSec() > 0.5) {
            if (42)
            {
                lastUpdateTime = ros::Time::now();
                // pthread_mutex_lock( &mutex );
                // aux = systemState();
                op = getOpening();
                acc = getAcceleration();
                // force = getGraspingForceLimit();
                force = getForce();
                // pthread_mutex_unlock( &mutex );
                // std::stringstream ss;

                // ss << aux;

                // status_msg.status = ss.str();
                // status_msg.width = op;
                // status_msg.acc = acc;
                // status_msg.force = force;

                // state_pub.publish(status_msg);//only useless crap, better to be implemented as a service instead

                // publish tactile sensor information
                /*
                  tactileSensorResult = getFingerSensorData(&sensor_msg);
                  if (tactileSensorResult != 0) {
                    sensor_pub.publish(sensor_msg);
                    if (softstop>0)checkforsoftstop(sensor_msg);
                  }
                 */
            }

            if (laststate != last_async_response)
            {
                ROS_INFO("Changing Last Async Response to %s.\n", status_to_str(last_async_response));
                printf("Changing Last Async Response to %s.\n", status_to_str(last_async_response));
                laststate = last_async_response;
                // publish message
            }

            joint_states.header.stamp = ros::Time::now();
            joint_states.header.frame_id = "wsg_50_gripper_base_link";

            joint_states.name.resize(2);
            joint_states.name[0] = "wsg_50_gripper_base_joint_gripper_left";
            joint_states.name[1] = "wsg_50_gripper_base_joint_gripper_right";

            joint_states.position.resize(2);
            joint_states.position[0] = -op / 2000.0;
            joint_states.position[1] = op / 2000.0;

            joint_states.velocity.resize(2);
            joint_states.velocity[0] = NAN;
            joint_states.velocity[1] = NAN;

            joint_states.effort.resize(2);
            joint_states.effort[0] = force;
            joint_states.effort[1] = force;

            joint_states_pub.publish(joint_states);

            loop_rate.sleep();
            // ROS_INFO( "entering spin\n" );
            ros::spinOnce();
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

//------------------------------------------------------------------------
// Testing functions
//------------------------------------------------------------------------
