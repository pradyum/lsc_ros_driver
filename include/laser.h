/*
* Copyright (c) 2022, Autonics Co.,Ltd.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*   * Neither the name of the Autonics Co.,Ltd nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/



#ifndef LASER_H
#define LASER_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "socket.hpp"
#include "parser.hpp"

#define CONNECT_ATTEMPT_CNT 999

class AutoLaser
{
public:
    AutoLaser();
    ~AutoLaser();

    int laserInit(void);
    int run(void);


private:
    bool checkConnection();

    void watchingDisconnection(void);
    std::string itostr(int i);
    std::string ftostr(float i);
    unsigned char hexToChar(unsigned int value);

    int laserSendCommand(const std::string str);
    int getLaserData(sensor_msgs::LaserScan::Ptr scan_msg);
    void comSubCallback(const std_msgs::String::ConstPtr& msg);
    void selfTest(diagnostic_updater::DiagnosticStatusWrapper& status);
    int getLscData(void);
    void login(std::string password);

    ros::NodeHandle n;
    ros::NodeHandle priv_nh;
    diagnostic_updater::Updater diagnostic_topic_updater;

    Socket socket;
    Parser p;

    ros::Publisher pub_scan;
    ros::Subscriber sub_cmd;

    sensor_msgs::LaserScan::Ptr scan_msg;
    Lsc_t lsc;

    bool rcv_msg_flag_;
    std::vector<unsigned char> rcv_msg_;

    std::string topic_name;
};

#endif
