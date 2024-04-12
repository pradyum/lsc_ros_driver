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

#include <sys/ioctl.h>
#include <net/if.h>

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

int AutoLaser::addrChange()
{
    int ret = 0;
    std::vector<unsigned char> udpcmd_vec;
    size_t buff_size = 100;
    unsigned char cmd_buff[buff_size];
    unsigned char* cmd_buff_p = cmd_buff;
    uint8_t cmd_buff_len = 0, len_send = 0, recv_check = 0, try_esteps = 0;
    bool flag_ipChageOnProcess = false;
    IPCHANGE_STAT ic_stat = NONE;
    std::string recv_addr, temp_new_ip = "", udp_client_ip = "";
    uint16_t port_num = 0;

    udp_client_ip = searchClientip(prev_addr);     
    if(udp_client_ip != "") 
    {
        ret = udpsock.clientOpen(udp_client_ip, "55555");

        if(ret < 0)
        {
            std::cout << "failed to connect udp" << std::endl;
            ret = -1;
        }
        else
        {
            flag_ipChageOnProcess = true;
            ic_stat = RQ_INFO;
        }

        while(flag_ipChageOnProcess)
        {
            switch(ic_stat)
            {
                case NONE:
                    break;

                case RQ_INFO:
                    memset(cmd_buff, 0x00, buff_size);
                    cmd_buff_len = udpparser.makeUdpCmd(1, cmd_buff, udpparser.NetworkChange);
                    len_send= udpsock.clientWrite(cmd_buff, cmd_buff_len);
                    try_esteps++;

                    if(len_send > 0)
                    {
                        try_esteps = 0;
                        ic_stat = GET_INFO;
                    }
                    else
                    {    
                        std::cout << "udp send failed, error no : " << strerror(errno) << std::endl;
                        std::cout << "retry : " << try_esteps << std::endl;
                        if(try_esteps > 10)
                        {
                            std::cout << "ip change failed, error no : " << strerror(errno) << std::endl;
                            try_esteps = 0;
                            ic_stat = NONE;
                            flag_ipChageOnProcess = false;
                            ret = -1;
                        }
                    }
                    break;

                case GET_INFO:
                    try_esteps++;
                    if(!udpsock.recvQueue.empty())
                    {
                        recv_addr = udpsock.popAddr();
                        std::cout << "recv_addr : " << recv_addr << std::endl;
                        recv_check = udpparser.parsingMsg(udpsock.recvQueue.front(), recv_addr, &udpparser.NetworkInfo);
                        udpsock.recvQueue.pop();
                        // add ack check
                        if(recv_check > 0 && recv_addr == prev_addr)
                        {
                            try_esteps = 0;
                            ic_stat = INFO_CHANGE;
                        }
                        else if(recv_check > 0 && recv_addr == new_addr)
                        {
                            try_esteps = 0;
                            ic_stat = NONE;
                            flag_ipChageOnProcess = false;
                            ip_addr = new_addr;
                            std::cout << "ip is matched with new_addr" << std::endl;
                            ret = 0;
                        }
                        else
                        {
                            try_esteps = 0;
                            ic_stat = NONE;
                            flag_ipChageOnProcess = false;
                            std::cout << "get network info failed" << std::endl;
                            std::cout << "please check ip address" << std::endl;
                            ret = -1;
                        }
                    }
                    else
                    {
                        std::cout << "messgeQueue is empty, retry : " << try_esteps << std::endl;
                        if(try_esteps > 10)
                        {
                            std::cout << "getting info failed, error no : " << strerror(errno) << std::endl;
                            try_esteps = 0;
                            ic_stat = NONE;
                            flag_ipChageOnProcess = false;
                        }
                    }
                    break;

                case INFO_CHANGE:
                    temp_new_ip = new_addr;
                    // portnum change
                    sscanf(port_str.c_str(), "%u", (unsigned int*)&port_num);
                    std::cout << "port_number : " << port_num << std::endl;
                    udpparser.NetworkInfo.Port[0] = 0;
                    udpparser.NetworkInfo.Port[1] = 0;
                    udpparser.NetworkInfo.Port[2] = port_num/0xFF;
                    udpparser.NetworkInfo.Port[3] = port_num & 0xFF;

                    udpparser.NetworkChange.NewIp[0] = std::stoi(strtok(&temp_new_ip[0], "."));
                    for(int i = 1; i < 4; i++)
                    {
                        char* temp_tok = strtok(NULL, ".");
                        if(temp_tok != NULL)
                        {
                            udpparser.NetworkChange.NewIp[i] = std::stoi(temp_tok);
                        }
                        else
                        {
                            break;
                        }
                    }

                    for(int i = 0; i < 8; i++)
                    {
                        udpparser.NetworkChange.MAC[i] = udpparser.NetworkInfo.MAC[i];
                    }
                    for(int i = 0; i < 4; i++)
                    {
                        udpparser.NetworkChange.OldIp[i] = udpparser.NetworkInfo.IpAddr[i];
                        udpparser.NetworkChange.SubnetMask[i] = udpparser.NetworkInfo.SubnetMask[i];
                        udpparser.NetworkChange.GateWay[i] = udpparser.NetworkInfo.GateWay[i];
                        udpparser.NetworkChange.Port[i] = udpparser.NetworkInfo.Port[i];
                    }

                    memset(cmd_buff, 0x00, buff_size);
                    cmd_buff_len = udpparser.makeUdpCmd(2, cmd_buff, udpparser.NetworkChange);
                    len_send= udpsock.clientWrite(cmd_buff, cmd_buff_len);
                    try_esteps++;

                    if(len_send > 0)
                    {
                        try_esteps = 0;
                        ic_stat = GET_RESP;
                    }
                    else
                    {    
                        std::cout << "info change failed, error no : " << strerror(errno) << std::endl;
                        std::cout << "retry : " << try_esteps << std::endl;
                        if(try_esteps > 10)
                        {
                            std::cout << "ip change failed, error no : " << strerror(errno) << std::endl;
                            try_esteps = 0;
                            ic_stat = NONE;
                            flag_ipChageOnProcess = false;
                            ret = -1;
                        }
                    }
                    break;

                case GET_RESP:
                    try_esteps++;
                    if(!udpsock.recvQueue.empty())
                    {
                        recv_addr = udpsock.popAddr();
                        recv_check = udpparser.parsingMsg(udpsock.recvQueue.front(), recv_addr, &udpparser.NetworkInfo);
                        udpsock.recvQueue.pop();

                        // add ack check
                        if(recv_check > 0)
                        {
                            try_esteps = 0;
                            ic_stat = NONE;
                            flag_ipChageOnProcess = false;
                            std::cout << "ip change compelete" << std::endl;
                            ip_addr = new_addr;
                            ret = 0;
                        }
                        else
                        {
                            try_esteps = 0;
                            ic_stat = NONE;
                            flag_ipChageOnProcess = false;
                            std::cout << "ip change failed" << std::endl;
                            ret = -1;
                        }
                    }
                    else
                    {
                        std::cout << "messgeQueue is empty, retry : " << try_esteps << std::endl;
                        if(try_esteps > 10)
                        {
                            std::cout << "getting response failed, error no : " << strerror(errno) << std::endl;
                            ret = -1;
                            try_esteps = 0;
                            ic_stat = NONE;
                            flag_ipChageOnProcess = false;
                        }
                    }
                    break;

                default:
                    ic_stat = NONE;
                    flag_ipChageOnProcess = false;
                    break;
            }
            usleep(100*1000);
        }
        
        udpsock.clientClose(udpsock.getClientSocket());
        sleep(2);
    }
    else
    {
        std::cout << "failed to get ip address" << std::endl;
    }

    return ret;
}

void AutoLaser::setScanAngle()
{
    char buff[60] = {0,};
    memset(buff, 0x00, sizeof(buff));
    buff[0] = 0x02;
    buff[1] = '0';
    buff[2] = '0';
    buff[3] = '0';
    buff[4] = '0';

    sprintf(&buff[5], ",sWC,LSScanDataConfig,%X,%X,%X,%X,%X\x03",\
    (int) std::round(((scan_msg->angle_min * RAD2DEG) - angle_offset_) * 10000),\
    (int) std::round(((scan_msg->angle_max * RAD2DEG) - angle_offset_) * 10000),\
    lsc.scan_data_config.rssi_activate,\
    lsc.scan_data_config.scan_interval,\
    lsc.scan_data_config.fieldset_output_activate);

    int length = strlen(buff);

    buff[1] = hexToChar(length / 16 / 16 / 16);
    buff[2] = hexToChar(length / 16 / 16 % 16);
    buff[3] = hexToChar(length / 16 % 16);
    buff[4] = hexToChar(length % 16);

    std::cout << "angle send message :" << buff << std::endl;

    if(sock.clientWrite((unsigned char*) buff, sizeof(buff)) < 0)
    {
        ROS_ERROR("Client write failed");
        if(sock.tryReconnection() < 0)
        {
            ;
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
    }
}

void AutoLaser::login(std::string password)
{
    std::string login_cmd = "SetAccessLevel";
    login_cmd = login_cmd + " " + password;

    laserSendCommand(login_cmd);
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

    if(sock.clientWrite(s_buffer, s_buffer_size) < 0)
    {
        ROS_ERROR("Client write failed");
        if(sock.tryReconnection() < 0)
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

    if(sock.getRcvError() || sock.getRcvTimeout())
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
    int ret = 0, tocnt = 0;
    while(1)
    {
        if(!sock.recvQueue.empty())
        {
            p.parsingMsg(sock.recvQueue.front(), scan_msg, &lsc);
            sock.recvQueue.pop();
            ret = 0;
            break;
        }
        else
        {
            ret = -1;
            tocnt++;
            if(tocnt > 200)
            {
                tocnt = 0;
                std::cout << "getLscData timeout" << std::endl;
                break;
            }
        }
        usleep(10*1000);
    }

    return ret;
}

int AutoLaser::getLaserData(sensor_msgs::LaserScan::Ptr p_laser_scan)
{
    if(!sock.recvQueue.empty())
    {
        p.parsingMsg(sock.recvQueue.front(), scan_msg, &lsc);
        sock.recvQueue.pop();

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
    if(sock.getRcvTimeout())
    {
        if(sock.tryReconnection() >= 0)
        {
            ROS_INFO("Reconnect successfully");
            laserSendCommand("SensorStart");
        }
        else
        {

        }
    }
}

std::string AutoLaser::searchMatchedAddr(std::string std_addr, std::vector<std::string> ip_list)
{
        std::vector<std::string> tok_std_addr, tok_ip_list;
        std::string matched_addr, str_iplist;
        char* temp_cmp;
        int tcnt = 0;

        tok_std_addr.clear();
        tok_std_addr.push_back(strtok(&std_addr[0], "."));
        while(1)
        {
            char* temp_tok = strtok(NULL, ".");
            if(temp_tok != NULL)
            {
                tok_std_addr.push_back(temp_tok);
            }
            else
            {
                break;
            }
        }

        if(tok_std_addr.size() != 4)
        {
            std::cout << "invalid ip addr" << std::endl;
            return NULL;
        }

        for(int i = 0; i < ip_list.size(); i++)
        {
            tok_ip_list.clear();
            
            temp_cmp = strtok(&((ip_list[i])[0]), ".");
            if(temp_cmp != NULL)
            {
                tok_ip_list.push_back(temp_cmp);
            }

            while(1)
            {   
                char* temp_cmp = strtok(NULL, ".");                
                if(temp_cmp != NULL)
                {
                    tok_ip_list.push_back(temp_cmp);
                }
                else
                {
                    break;
                }
                
            }

            for(int j = 0; j < 3; j++)
            {
                if(tok_std_addr[j] != tok_ip_list[j])
                {
                    break;
                }
                else
                {
                    if(j >= 2)
                    {
                        while(1)
                        {
                            matched_addr += tok_ip_list[tcnt++];
                            if(tcnt >= tok_ip_list.size())
                            {
                                break;
                            }
                            matched_addr += ".";
                        }
                        
                        return matched_addr;
                    }
                }
            }
        }

        return "none";
}

std::string AutoLaser::searchClientip(std::string baseAddr)
{
    std::string sip_addr;
    int temp_sock = 0;
    struct ifconf ifcfg;
    int numreqs = 10;
    struct sockaddr_in *sin;
    std::string arr_ipaddr[numreqs];
    std::vector<std::string> v_ipaddr;

    std::string clientIp = "";

    memset(&ifcfg, 0, sizeof(ifcfg));

    ifcfg.ifc_buf = NULL;
    ifcfg.ifc_len = sizeof(struct ifreq) * numreqs;
    char addbuff[ifcfg.ifc_len];
    ifcfg.ifc_buf = addbuff;

    temp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if(ioctl(temp_sock, SIOCGIFCONF,(char*)&ifcfg)<0)
    {
        perror("SIOCGIFCONF ");
        std::cout << "siocgifconf error" << std::endl;
        return NULL;
    }

    for(int n = 0; n < ifcfg.ifc_len; n += sizeof(struct ifreq))
    {
        sin = (struct sockaddr_in*)&ifcfg.ifc_req->ifr_addr;
        v_ipaddr.push_back(inet_ntoa(sin->sin_addr));
        ifcfg.ifc_req++;
    }
    
    sip_addr = searchMatchedAddr(baseAddr, v_ipaddr);
    if(sip_addr == "none")
    {
        std::cout << "no matched ip address, please check device is turned on" << std::endl;
        return "";
    }

    std::vector<std::string> tok_sip_addr;
    tok_sip_addr.push_back(strtok(&sip_addr[0], "."));

    while(1)
    {
        char* temp_tok = strtok(NULL, ".");
        if(temp_tok != NULL)
        {
            tok_sip_addr.push_back(temp_tok);
        }
        else
        {
            break;
        }
    }
    
    int vcnt = 0;
    while(1)
    {
        clientIp += tok_sip_addr[vcnt++];
        if(vcnt >= tok_sip_addr.size())
        {
            break;
        }
        clientIp += ".";
    }
    std::cout << "udp_client_ip : " << clientIp << std::endl;

    return clientIp;
}


int AutoLaser::laserInit(void)
{
    std::string frame_id = "laser", port = "8000", password = "0000";
    double range_min = 0.05, range_max = 25, angle_min = -45, angle_max = 225, angle_offset = 0;
    uint16_t port_num = 0, recnt_cnt = 0;
    bool ip_change = false;
    ip_addr = "192.168.0.1";
    topic_name = "scan";
    
    uint8_t udp_try_cnt = 0;

    priv_nh.getParam("addr", ip_addr);
    priv_nh.getParam("frame_id", frame_id);
    priv_nh.getParam("port", port);
    priv_nh.getParam("range_min", range_min);
    priv_nh.getParam("range_max", range_max);
    priv_nh.getParam("password", password);
    priv_nh.getParam("topic_name", topic_name);

    priv_nh.getParam("ip_change", ip_change);
    priv_nh.getParam("prev_addr", prev_addr);
    priv_nh.getParam("new_addr", new_addr);

    priv_nh.getParam("angle_min", angle_min);
    priv_nh.getParam("angle_max", angle_max);
    priv_nh.getParam("angle_offset", angle_offset);
    
    scan_msg->header.frame_id = frame_id;
    scan_msg->range_min = range_min;
    scan_msg->range_max = range_max;
    scan_msg->angle_min = angle_min * DEG2RAD;
    scan_msg->angle_max = angle_max * DEG2RAD;
    angle_offset_ = angle_offset;

    pub_scan = n.advertise<sensor_msgs::LaserScan>(topic_name, 1000);
    sub_cmd = n.subscribe<std_msgs::String>("command", 10, &AutoLaser::comSubCallback, this);

    // check ip change
    if(ip_change == true && (prev_addr != "") && (new_addr != ""))
    {
        port_str = port;
        std::cout << "ip change :" << ip_change << std::endl;
        std::cout << "prev_addr :" << prev_addr << std::endl;
        std::cout << "new_addr :" << new_addr << std::endl;
        addrChange();
    }

    if(sock.clientOpen(ip_addr, port) < 0)
    {
        ROS_WARN("Failed to connect to server");
        while(true)
        {
            recnt_cnt++;
            if(sock.tryReconnection() < 0)
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

    getLscData();

    laserSendCommand("SensorScanInfo");
    getLscData();

    login(password);  
    getLscData();

    laserSendCommand("LSScanDataConfig");
    getLscData();

    setScanAngle();
    getLscData();

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
            scan_msg->angle_min += angle_offset_* DEG2RAD;
            scan_msg->angle_max += angle_offset_* DEG2RAD;

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

