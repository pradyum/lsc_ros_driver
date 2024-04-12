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
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT





 HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
* LIABILITY, OR TORT (IN CLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE.
*/


#include <sys/poll.h>
#include <stdio.h>

#include "socket.hpp"

#include <chrono> // test


#define STX 0x02
#define ETX 0x03

Socket::Socket()
{
    m_server_sock_ = 0;
    m_connected_ = 0;

    rcv_timeout_ = false;            
    rcv_error_ = false;

    pthread_read_running_ = false;
    pthrd_id_ = 0;
}

Socket::~Socket()
{
    pthread_read_running_ = false;

    if(pthrd_id_)   
    {
        pthread_join(pthrd_id_, NULL);

    }

    if(m_connected_ == true)
    {
        close(m_server_sock_);

        m_connected_ = false;

        std::cout << "disconnect server" << std::endl;
    }
}   

int Socket::getServerSocket(void)
{
    return m_server_sock_;
}

bool Socket::getConnected(void)
{
    return m_connected_;
}

bool Socket::getPthreadRunning(void)
{
    return pthread_read_running_;
}


bool Socket::getRcvTimeout(void)
{
    return rcv_timeout_;
}

bool Socket::getRcvError(void)
{
    return rcv_error_;
}

void Socket::setRcvTimeout(bool flag)
{
    rcv_timeout_ = flag;
}

void Socket::setRcvError(bool flag)
{
    rcv_error_ = flag;
}

int Socket::tryReconnection(void)
{
    int ret = 0;
    pthread_read_running_ = false;

    if(pthrd_id_)
    {
        pthread_join(pthrd_id_, NULL);
        pthrd_id_ = 0;
    }
    
    m_connected_ = false;
    close(m_server_sock_);

    usleep(1000*1000);

    ret = clientOpen(addr_st_, port_num_st_);

    if(ret < 0)
    {
        std::cout << "reconnection failed" << std::endl;
        return ret;
    }
    
    return ret;
}

void* thread_readCallback(void* arg)
{
    int32_t read_cnt = 0;
    uint16_t packet_size = 0, buf_size = 8000;
    int ret = 0;
    unsigned char r_buffer[buf_size] = {0, }, temp_len[4] = {0, };
    Socket* sock = (Socket*) arg;    
    struct pollfd fd;
    ssize_t chk_connection = 0;

    std::vector<unsigned char> vec, vec_dataSize; 
    bool flag_stx = false, flag_length = false;
    size_t dataSize = 0;

    fd.fd = sock->getServerSocket();
    fd.events = POLLIN;

    while(sock->getPthreadRunning())
    {
        read_cnt = 0;

        ret = poll(&fd, 1, 1000);

        if(ret < 0)
        {
            sock->setRcvError(true);
            std::cout << "polling error" << std::endl;
        }
        else /// ret >= 0
        {
            sock->setRcvError(false);

            read_cnt = recv(sock->m_server_sock_, r_buffer, sizeof(r_buffer), 0);

            if(ret == 0)
            {
                sock->setRcvTimeout(true);
            #ifdef PRINT_DEBUG
                std::cout << "polling timeout" << std::endl;
            #endif
            }

            if(read_cnt > 0)
            {
                sock->setRcvTimeout(false);

                for(int i = 0; i < read_cnt; i++)
                {
                    vec.push_back(*(r_buffer + i));

                    if(flag_stx == true)
                    {
                        if(flag_length == true)
                        {
                            if(vec.back() == ETX)
                            {
                                if(dataSize == vec.size())
                                {
                                    sock->recvQueue.push(vec);
                                    vec.clear();
                                    flag_stx = false;
                                    flag_length = false; 
                                    dataSize = 0;
                                }
                                else
                                {
                                    std::cout << "data length mismatched, datasize : " << dataSize << " ,msg size : " << vec.size() << std::endl;
                                    vec.clear();
                                    flag_stx = false;
                                    flag_length = false; 
                                    dataSize = 0;
                                    break;
                                }
                            }
                            else
                            {
                                ;
                            }
                        }
                        else
                        {
                            vec_dataSize.push_back(vec.back());
                            if(vec_dataSize.size() >= 4)
                            {
                                for(int i = 0; i < 4; i++)
                                {
                                    temp_len[i] = vec_dataSize[i];
                                }
                                dataSize = strtoul((const char*) temp_len, NULL, 16);

                                if(dataSize > 0)
                                {
                                    flag_length = true;
                                }
                                else
                                {
                                    std::cout << "invaild data size : " << dataSize << std::endl;
                                }
                                vec_dataSize.clear();
                            }
                            else
                            {

                            }
                        }
                    }
                    else
                    {
                        if(vec.back() == STX)
                        {
                            vec.clear();
                            vec.push_back(*(r_buffer + i)); 
                            flag_stx = true;
                        }
                        else
                        {

                        }
                    }
                }
                memset(r_buffer, 0x00, sizeof(r_buffer));
            }
            else if(read_cnt < 0)    /// disconnect
            {
                std::cout << "server is disconnected" << std::endl;
            }
            else /// receive error
            {
                *r_buffer = 'a';
                chk_connection = write(sock->m_server_sock_, r_buffer, sizeof(r_buffer));

                if(chk_connection == -1)
                {
                    sock->setRcvTimeout(true);
                    std::cout << "server disconnected" << std::endl;
                }
            }
        }
        usleep(1000);
    }

    std::cout << "escape tcp read" << std::endl;

    return 0;
}


int Socket::clientOpen(std::string addr, std::string port)
{
    int m_server_addr_size = 0;
    uint16_t port_num = 0;

    timeval tv;
    
    
    addr_st_ = addr;
    port_num_st_ = port;
    
    m_server_sock_ = socket(PF_INET, SOCK_STREAM, 0);    

    if(m_server_sock_ < 0)
    {
        std::cout << "faild to create client socket" << std::endl;
        return -1;
    }

    tv.tv_sec = 0;;
    tv.tv_usec = 500*1000;

    if(setsockopt(m_server_sock_, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv)) < 0)
    {
        std::cout << "failed rcvtimeo setsockopt" << std::endl;
    }
    if(setsockopt(m_server_sock_, SOL_SOCKET, SO_SNDTIMEO, (char*)&tv, sizeof(tv)) < 0)
    {
        std::cout << "failed sndtimeo setsockopt" << std::endl;
    }

    sscanf(port.c_str(), "%u", (unsigned int*)&port_num);

    memset(&m_server_addr_, 0x00, sizeof(m_server_addr_));

    m_server_addr_.sin_family = AF_INET;
    m_server_addr_.sin_addr.s_addr = inet_addr(addr.begin().base());
    m_server_addr_.sin_port = htons(port_num);

    m_server_addr_size = sizeof(m_server_addr_);
    
    if(connect(m_server_sock_, (const sockaddr*)&m_server_addr_, m_server_addr_size) < 0)
    {
        std::cout << "failed client connect to " << addr << ", " << strerror(errno) << std::endl;
        return -1;
    }
    else
    {
        m_connected_ = true;
        std::cout << "connected to " << addr << std::endl;
    }

    pthread_read_running_ = true;

    if(pthread_create(&pthrd_id_, NULL, thread_readCallback, this) < 0)
    {
        pthread_read_running_ = false;
        std::cout << "pthread create failed" << std::endl;
        return -1;
    }
    else
    {
        ;
    }    

    return 0;
}

int32_t Socket::clientRead(unsigned char* buffer, uint16_t buf_size)
{
    int32_t read_size = 0;

    if(m_connected_ == true)
    {
       read_size = recv(m_server_sock_, buffer, buf_size, 0);

       if(read_size < 0)
        {
            std::cout << "client read failed" << std::endl;
        }
        
        return read_size;
    }
    else
    {
        std::cout << "server is disconnected" << std::endl;
        return -1;
    }
}

int32_t Socket::clientWrite(unsigned char* buffer, uint16_t buf_size)
{
    int32_t write_size = 0;

    if(m_connected_ == true)
    {
        write_size = send(m_server_sock_, buffer, buf_size, 0);
        if(write_size < 0)
        {
            std::cout << "client write failed" << std::endl;

            try
            {

            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        return write_size;
    }
    else
    {
        std::cout << "server is disconnected" << std::endl;
        return -1;
    }

    
}




// udp
UdpSocket::UdpSocket()
{
    m_client_sock_ = 0;
    m_connected_ = false;

    rcv_timeout_ = false;            
    rcv_error_ = false;

    pthread_read_running_ = false;
    pthrd_id_ = 0;
}

UdpSocket::~UdpSocket()
{
    clientClose(m_client_sock_);
}

bool VerifyCheckSum(unsigned char* pData, uint16_t nLength)
{
    uint32_t nSum = 0;
    for(int i=0 ; i<nLength ; i++) nSum += pData[i];
    return ((nSum & 0xFF) == 0);
}

void* thread_ReadCallback(void* arg)
{
    int32_t read_cnt = 0;
    uint16_t packet_size = 0, buf_size = 400;
    int ret = 0;
    unsigned char r_buffer[buf_size] = {0, };
    UdpSocket* udpSock = (UdpSocket*) arg;
    struct pollfd fd;

    std::vector<unsigned char> vec, vec_dataSize, vec_cmd; 
    bool flag_stx = false, flag_length = false, flag_cmd = false;
    size_t dataSize = 0;
    uint32_t temp_byte4 = 0;

    vec_cmd.reserve(2);
    vec_dataSize.reserve(4);

    fd.fd = udpSock->getClientSocket();
    fd.events = POLLIN;
    
    char recv_addr[30] = {0};

    while(udpSock->getPthreadRunning())
    {
        read_cnt = 0;

        ret = poll(&fd, 1, 1000);

        if(ret < 0)
        {
            udpSock->setRcvError(true);
            std::cout << "polling error" << std::endl;
        }
        else
        {
            udpSock->setRcvError(false);

            read_cnt = udpSock->clientRead(r_buffer, sizeof(r_buffer), recv_addr);

            if(ret == 0)
            {
                udpSock->setRcvTimeout(true);
            #ifdef PRINT_DEBUG
                std::cout << "polling timeout" << std::endl;
            #endif
            }

            if(read_cnt > 0)
            {
                udpSock->setRcvTimeout(false);

                for(int i = 0; i < read_cnt; i++)
                {
                    vec.push_back(*(r_buffer + i));

                    if(flag_stx == true)
                    {
                        if(flag_cmd == true)
                        {
                            if(flag_length == true)
                            {
                                if(dataSize == vec.size())
                                {
                                    if(VerifyCheckSum(&vec[0], (uint16_t)vec.size()) == 1)
                                    {
                                        udpSock->recvQueue.push(vec);
                                        std::cout << "recvaddr : " << recv_addr << std::endl;
                                        udpSock->pushAddr(recv_addr);
                                        vec.clear();
                                        flag_stx = false;
                                        flag_length = false; 
                                        flag_cmd = false;
                                        dataSize = 0;
                                    }
                                    else
                                    {
                                        std::cout << "checksum mismatched, " << std::endl;
                                        vec.clear();
                                        flag_stx = false;
                                        flag_length = false; 
                                        flag_cmd = false;
                                        dataSize = 0;
                                        break;
                                    }
                                }
                                else
                                {
                                    
                                }
                            }
                            else
                            {
                                vec_dataSize.push_back(vec.back());
                                if(vec_dataSize.size() >= 4)
                                {
                                    memcpy(&temp_byte4, &vec_dataSize[0], sizeof(temp_byte4));
                                    dataSize = htonl(temp_byte4);
                                    if(dataSize > 0)
                                    {
                                        flag_length = true;
                                    }
                                    else
                                    {
                                        std::cout << "invaild data size : " << dataSize << std::endl;
                                    }
                                    vec_dataSize.clear();
                                }
                                else
                                {

                                }
                            }
                        }
                        else
                        {
                            vec_cmd.push_back(vec.back());
                            if(vec_cmd.size()>=2)
                            {
                                flag_cmd = true;
                                vec_cmd.clear();
                            }
                        }
                    }
                    else
                    {
                        if(vec.back() == 0xfc)
                        {
                            if(vec.size() >= 2)
                            {
                                if(vec[vec.size()-2] == 0xf3)
                                {
                                    flag_stx = true;
                                    vec.clear();
                                    vec.push_back(*(r_buffer + i - 1));
                                    vec.push_back(*(r_buffer + i));
                                }
                            }
                        }
                        else
                        {
                        }
                    }
                }
                memset(r_buffer, 0x00, sizeof(r_buffer));
            }
            else if(read_cnt < 0)
            {
                ;
            }
            else 
            {
                ;
            }
        }
        usleep(100);
    }

    std::cout << "escape socket read while loop" << std::endl;

    return 0;
}

int UdpSocket::clientOpen(std::string ipaddr, std::string port_num)
{
    int ret = 0;
    int res = 0;
    unsigned int port = 0;
    std::vector<std::string> iptok;
    char* tokret;
    int vecSize;
    int on = 1;

    timeval tv;
    tv.tv_sec = 0;;
    tv.tv_usec = 500*1000;
    
    m_client_sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    setsockopt(m_client_sock_, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)); // broadcast permission

    if(setsockopt(m_client_sock_, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv)) < 0)
    {
        std::cout << "failed rcvtimeo setsockopt" << std::endl;
    }
    if(setsockopt(m_client_sock_, SOL_SOCKET, SO_SNDTIMEO, (char*)&tv, sizeof(tv)) < 0)
    {
        std::cout << "failed sndtimeo setsockopt" << std::endl;
    }

    if(m_client_sock_ < 0)
    {
        std::cout << "udp open failed" << std::endl;
        ret = -1;
    }
    else
    {
        memset(&m_server_addr_, 0x00, sizeof(struct sockaddr_in));

        m_server_addr_.sin_family = AF_INET;
        m_server_addr_.sin_port = htons(55555);
        m_server_addr_.sin_addr.s_addr = inet_addr(ipaddr.begin().base());

        res = bind(m_client_sock_, (sockaddr *) &m_server_addr_, sizeof(m_server_addr_));

        if(res < 0)
        {
            std::cout << "udp bind failed : " << res << std::endl;
            m_connected_ = false;
            ret = -1;
        }
        else
        {
            m_connected_ = true;
            pthread_read_running_ = true;

            // create udp recv thread
            if(pthread_create(&pthrd_id_, NULL, thread_ReadCallback, this) < 0)
            {
                pthread_read_running_ = false;
                std::cout << "udp read pthread create failed" << std::endl;
                return -1;
            }
            else
            {
                std::cout << "udp read pthread is created" << std::endl;
            }    
        }
    }

    return ret;
}

void UdpSocket::clientClose(int fd_sock)
{
    pthread_read_running_ = false;

    if(pthrd_id_)   
    {
        pthread_join(pthrd_id_, NULL);
    }

    if(m_connected_ == true)
    {
        close(fd_sock);

        m_connected_ = false;
        std::cout << "disconnect udp" << std::endl;
    }
}

int UdpSocket::clientRead(unsigned char* buff, uint16_t buff_size, char* recv_ipaddr)
{
    sockaddr_in any_addr;
    uint32_t udp_any_addr_size = sizeof(any_addr);
    int ret = 0;
    char* temp;

    memset(&any_addr, 0, udp_any_addr_size);

    if(m_connected_ == true)
    {
        int len = recvfrom(m_client_sock_, buff, buff_size, 0, (sockaddr*) &any_addr, &udp_any_addr_size);

        if(len > 0)
        {
            temp = inet_ntoa(any_addr.sin_addr);

            for(int i = 0; i < 30; i++)
            {
                recv_ipaddr[i] = temp[i];
                if(temp[i] == '\n')
                {
                    break;
                }   
            }
        }

        ret = len;
    }
    else
    {
        std::cout << "no udp connection" << std::endl;
        ret = -1;
    }
    
    return ret;
}

int UdpSocket::clientWrite(unsigned char* buff, uint16_t buff_size)
{
    int ret = 0;
    sockaddr_in udp_send_addr;

    udp_send_addr.sin_family = m_server_addr_.sin_family;
    udp_send_addr.sin_port = m_server_addr_.sin_port;
    udp_send_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST); // broadcast

    if(m_connected_ == true)
    {
        int len = sendto(m_client_sock_, buff, buff_size, 0, (sockaddr*) &udp_send_addr, sizeof(udp_send_addr));

        if(len < 0)
        {
            std::cout << "udp send failed, error no : " << strerror(errno) << std::endl;
        }
        ret = len;
    }
    else
    {
        std::cout << "no udp connection" << std::endl;
        ret = -1; 
    }

    return ret;
}


int UdpSocket::getClientSocket(void)
{
    return m_client_sock_;
}

bool UdpSocket::getConnected(void)
{
    return m_connected_;
}

bool UdpSocket::getPthreadRunning(void)
{
    return pthread_read_running_;
}

void UdpSocket::setPthreadRunning(bool flag)
{
    pthread_read_running_ = flag;
}


bool UdpSocket::getRcvTimeout(void)
{
    return rcv_timeout_;
}

bool UdpSocket::getRcvError(void)
{
    return rcv_error_;
}

void UdpSocket::setRcvTimeout(bool flag)
{
    rcv_timeout_ = flag;
}

void UdpSocket::setRcvError(bool flag)
{
    rcv_error_ = flag;
}

void UdpSocket::pushAddr(std::string recvaddr)
{
    queue_addr_.push(recvaddr);
}

std::string UdpSocket::popAddr(void)
{
    std::string addr;
    if(!queue_addr_.empty())
    {
        addr = queue_addr_.front();
        queue_addr_.pop();
        return addr;
    }
    else
    {
        return NULL;
    }
}