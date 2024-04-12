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



#ifndef SOCKET_HPP
#define SOCKET_HPP

#include <pthread.h>
#include <vector>
#include <queue>
#include <mutex>
#include <memory>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>


class Socket
{
    public:
        Socket();
        ~Socket();

        int clientOpen(std::string addr, std::string port);
        int clientOpen(void);
        int32_t clientRead(unsigned char* buffer, uint16_t buf_size);
        int32_t clientWrite(unsigned char* buffer, uint16_t buf_size);

        int tryReconnection(void);

        friend void* thread_readCallback(void* arg);

        int getServerSocket(void);
        bool getConnected(void);

        bool getRcvTimeout(void);
        bool getRcvError(void);

        void setRcvTimeout(bool flag);
        void setRcvError(bool flag);

        bool getPthreadRunning(void);

        std::queue<std::vector<unsigned char>> recvQueue;

    private:
        void putBufToMsg(unsigned char* buf, uint16_t size);

        sockaddr_in m_server_addr_;
        pthread_t pthrd_id_;
        std::string addr_st_        ;
        std::string port_num_st_;

        int m_server_sock_;
        bool pthread_read_running_;
        bool m_connected_;
        bool rcv_timeout_;
        bool rcv_error_;
};


class UdpSocket
{
    public:
        UdpSocket();
        ~UdpSocket();

        int clientOpen(std::string ipaddr, std::string port_num);
        void clientClose(int fd_sock);
        int32_t clientWrite(unsigned char* buff, uint16_t buf_size);
        int32_t clientRead(unsigned char* buff, uint16_t buf_size, char* recv_ipaddr);
        friend void* thread_ReadCallback(void* arg);

        int getClientSocket(void);
        bool getConnected(void);

        bool getRcvTimeout(void);
        bool getRcvError(void);

        void setRcvTimeout(bool flag);
        void setRcvError(bool flag);

        bool getPthreadRunning(void);
        void setPthreadRunning(bool flag);

        void pushAddr(std::string recvaddr);
        std::string popAddr(void);

        std::queue<std::vector<unsigned char>> recvQueue;

    private:
        bool m_connected_;
        sockaddr_in m_server_addr_;
        pthread_t pthrd_id_;

        int m_client_sock_;
        bool pthread_read_running_;
        bool rcv_timeout_;
        bool rcv_error_;

        std::queue<std::string> queue_addr_;
};

#endif