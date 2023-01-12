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




#include <iostream>
#include <sstream>

#include <self_test/self_test.h>

#include "laser.h"

AutoLaser::AutoLaser() : n(), priv_nh("~"), scan_msg(new sensor_msgs::LaserScan)
{
    rcv_msg_flag_ = false;
}

AutoLaser::~AutoLaser()
{

}

std::string AutoLaser::itostr(int i)
{
    std::string rt_str;
    std::stringstream ss;

    ss << i;
    ss >> rt_str;

    return rt_str;
}

std::string AutoLaser::ftostr(float i)
{
    std::string rt_str;
    std::stringstream ss;

    ss << i;
    ss >> rt_str;

    return rt_str;
}

unsigned char AutoLaser::hexToChar(unsigned int value)
{
    if(value > 9 && value < 16)
    {
        return value + '0' + 7;
    }
    else if (value >= 16)
    {
        ROS_WARN("the value is out of range");
        return value;
    }
    else
    {
        return value + '0';
    }
}

void AutoLaser::comSubCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string cmd = msg->data.c_str();
    laserSendCommand(cmd);
}


void AutoLaser::login(std::string password)
{
    std::string login_cmd = "SetAccessLevel";
    login_cmd = login_cmd + " " + password;

    if(laserSendCommand(login_cmd) < 0);
}

int AutoLaser::laserSendCommand(const std::string str)
{
    uint16_t s_buffer_size = 0;
    unsigned char s_buffer[64] = {0, };
    int cmd_str_cnt = 0, str_cnt = 0;

    memset(s_buffer, 0x00, sizeof(s_buffer));

    s_buffer[str_cnt++] = 0x02;
    str_cnt++;
    str_cnt++;
    str_cnt++;
    str_cnt++;

    cmd_str_cnt = p.makeCommand(&s_buffer[str_cnt], str);

    if(cmd_str_cnt == -1)
    {
        return -1;
    }
    else
    {
        str_cnt += cmd_str_cnt;
    }
    s_buffer[str_cnt++] = 0x03;

    s_buffer[1] = hexToChar(str_cnt / 16 / 16 / 16);
    s_buffer[2] = hexToChar(str_cnt / 16 / 16 % 16);
    s_buffer[3] = hexToChar(str_cnt / 16 % 16);
    s_buffer[4] = hexToChar(str_cnt % 16);

    s_buffer_size = sizeof(s_buffer);

    if(socket.clientWrite(s_buffer, s_buffer_size) < 0)
    {
        ROS_ERROR("Client write failed");
        if(socket.tryReconnection() < 0)
        {
            return -1;
        }
        else
        {
            ROS_INFO("Reconnect successfully");
        }
    }
    else
    {
        #ifdef PRINT_PARSE_DATA
        std::cout << std::endl << "sent data : ";
        for(int i = 0; i < str_cnt; i++)
        {
            std::cout << std::hex << s_buffer[i];
        }
        std::cout << "" << std::endl;
        #endif

        usleep(1000*100);
    }

    return 0;
}

bool AutoLaser::checkConnection()
{
    bool result =  false;

    if(socket.getRcvError() || socket.getRcvTimeout())
    {

    }
    else
    {
        result = true;
    }

    return result;
}

void AutoLaser::selfTest(diagnostic_updater::DiagnosticStatusWrapper& status)
{
    status.hardware_id = lsc.scan_info.model_name;

    if(!checkConnection())
    {
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "lost connection");
    }
    else
    {
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "connection is ok");
    }
}

int AutoLaser::getLscData(void)
{
    if(!(socket.recvQueue.empty()))
    {
        rcv_msg_ = socket.recvQueue.front();
        socket.recvQueue.pop();

        p.parsingMsg(rcv_msg_, scan_msg, &lsc);
    }
    else
    {
        return -1;
    }

    return 0;
}

int AutoLaser::getLaserData(sensor_msgs::LaserScan::Ptr p_laser_scan)
{
    if(!(socket.recvQueue.empty()))
    {
        rcv_msg_ = socket.recvQueue.front();
        socket.recvQueue.pop();

        p.parsingMsg(rcv_msg_, p_laser_scan, &lsc);
        if(p_laser_scan->ranges.empty() == false)
        {
            p_laser_scan->header.stamp = ros::Time::now();

            rcv_msg_flag_ = true;
        }
    }
    else
    {
        rcv_msg_flag_ = false;
    }

    return 0;
}

void AutoLaser::watchingDisconnection(void)
{
    if(socket.getRcvTimeout())
    {
        if(socket.tryReconnection() >= 0)
        {
            ROS_INFO("Reconnect successfully");
            laserSendCommand("SensorStart");
        }
        else
        {

        }
    }
}

int AutoLaser::laserInit(void)
{
    std::string addr = "192.168.0.1", frame_id = "laser", port = "8000", password = "0000";
    double range_min = 0.05, range_max = 25;
    uint16_t port_num = 0, recnt_cnt = 0;
    topic_name = "scan";

    priv_nh.getParam("addr", addr);
    priv_nh.getParam("frame_id", frame_id);
    priv_nh.getParam("port", port);
    priv_nh.getParam("range_min", range_min);
    priv_nh.getParam("range_max", range_max);
    priv_nh.getParam("password", password);
    priv_nh.getParam("topic_name", topic_name);

    scan_msg->header.frame_id = frame_id;
    scan_msg->range_min = range_min;
    scan_msg->range_max = range_max;

    pub_scan = n.advertise<sensor_msgs::LaserScan>(topic_name, 1000);
    sub_cmd = n.subscribe<std_msgs::String>("command", 10, &AutoLaser::comSubCallback, this);

    if(socket.clientOpen(addr, port) < 0)
    {
        ROS_WARN("Failed to connect to server");
        while(true)
        {
            recnt_cnt++;
            if(socket.tryReconnection() < 0)
            {
                ROS_WARN("Failed to reconnect to server");
                if(recnt_cnt >= CONNECT_ATTEMPT_CNT)
                {
                    ROS_ERROR("Tried to connect server %d times but, Failed to connect to server", recnt_cnt);
                    return -1;
                }
                if(!ros::ok())
                {
                    return -1;
                }
            }
            else
            {
                break;
            }
        }
    }

    laserSendCommand("SensorScanInfo");
    getLscData();

    login(password);

    return 0;
}

int AutoLaser::run(void)
{
    ros::Rate loop_rate(150);
    self_test::TestRunner self_test;

    self_test.setID("self_test");
    self_test.add("Connection", this, &AutoLaser::selfTest);

    double min_freq = 0.00025;  // Hz
    double max_freq = 15.0;     // Hz
    diagnostic_updater::HeaderlessTopicDiagnostic pub_freq(topic_name, diagnostic_topic_updater, diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 1));
    diagnostic_topic_updater.setHardwareID("AutoLaser");

    laserSendCommand("SensorStart");

    while(ros::ok())
    {
        getLaserData(scan_msg);

        if(rcv_msg_flag_)
        {
            pub_scan.publish(scan_msg);
            scan_msg->ranges.clear();
            scan_msg->intensities.clear();

            pub_freq.tick();
            diagnostic_topic_updater.update();
        }

        self_test.checkTest();

        watchingDisconnection();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

