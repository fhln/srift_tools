#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netdb.h> 
#include <vector>

#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

namespace SRI_ft_Driver
{

class TCPClient
{
  private:
    int sock;
    std::string address;
    int port;
    struct sockaddr_in server;

  public:
    TCPClient();
    bool setup(string address, int port);
    bool Send(string data);
    string receive(int size = 4096);
    bool GetADCounts(MatrixXd &pdBuffer);
    bool GetChParameter(MatrixXd &pdBuffer);
  bool GetSamplingRate();
    bool readrecieveBuffer(MatrixXd &pdBuffer);
    string read();
    void exit();
};

}

#endif
