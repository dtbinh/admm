#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netdb.h>
#include <string>

using namespace std;

int main ()
{
int status;
struct addrinfo host_info;       
struct addrinfo *host_info_list; 
   
memset(&host_info, 0, sizeof host_info);
 
cout<<"Setting up the structs..."<<endl;
 
host_info.ai_family = AF_UNSPEC;
host_info.ai_flags = AI_PASSIVE;     
host_info.ai_socktype = SOCK_STREAM; 
 
status = getaddrinfo("172.16.0.1", "5522", &host_info, &host_info_list);

if (status != 0)  
{
   cout<<"getaddrinfo error: " << gai_strerror(status);
}
cout<<"\nCreating a socket \n";
int socketfd;
socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
if (socketfd == -1)
{
   cout<<"socket error \n";
}
cout<<"Binding socket..."<<endl;

int yes = 1;
status = setsockopt(socketfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
status = bind(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
if (status == -1)  
{
	cout<<"bind error"<<endl;
} 
cout << "Listening for connections..."<<endl;
status =  listen(socketfd, 5);
if (status == -1)  
{
	cout<< "listen error"<<endl;
}
int new_sd;
struct sockaddr_storage their_addr;
socklen_t addr_size = sizeof(their_addr);
new_sd = accept(socketfd, (struct sockaddr *)&their_addr, &addr_size);
if (new_sd == -1)
{
    cout << "listen error" << std::endl ;
}
else
{
    cout << "Connection accepted. Using new socketfd : "<<new_sd<<endl;
}
ssize_t bytes_recieved;
while(1)
{
char incoming_buffer[100];
bytes_recieved = recv(socketfd, incoming_buffer, 100, 0);
cout<<"bytes received: "<<bytes_recieved<<endl;
string s(incoming_buffer);
cout<<s<<endl<<incoming_buffer[0]<<incoming_buffer[1]<<endl;
}
return 0;
}
