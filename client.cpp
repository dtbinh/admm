#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netdb.h>

using namespace std;

int main ()
{
  int status;
  struct addrinfo host_info;
  struct addrinfo *host_info_list;

  memset(&host_info, 0, sizeof host_info);

  host_info.ai_family = AF_UNSPEC;
  host_info.ai_socktype = SOCK_STREAM;
  status = getaddrinfo("172.16.0.1", "5522", &host_info, &host_info_list);
  cout<<"status: "<<status<<endl;
  if (status != 0)
  {
    cout<<"getaddrinfo error"<<endl<<gai_strerror(status);
  }
  cout<<"Creating a socket \n";
  int socketfd;
  socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
  if (socketfd == -1)
  {
    cout<<"socket error \n";
  }
  status =  connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
  if (status == -1)
  {
    cout<<"connect error \n";
  }
  cout << "send()ing message..."  <<endl;
  while(1)
  {
  char *msg = "Hi! This is Kush";
  int len;
  ssize_t bytes_sent;
  len = strlen(msg);
  bytes_sent = send(socketfd, msg, len, 0);
  cout<<"bytes sent: "<<bytes_sent<<endl;
  }
  return 0;
}
