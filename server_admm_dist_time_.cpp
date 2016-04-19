// "hi" 
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netdb.h>
#include <string>
#include <vector>
#include <math.h>
#include <time.h>
#include <chrono>

using namespace std;
using namespace std::chrono;

uint64_t pack754(double f, unsigned bits, unsigned expbits)
{
    long double fnorm;
    int shift;
    long long sign, exp, significand;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (f == 0.0) return 0; // get this special case out of the way

    // check sign and begin normalization
    if (f < 0) { sign = 1; fnorm = -f; }
    else { sign = 0; fnorm = f; }

    // get the normalized form of f and track the exponent
    shift = 0;
    while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
    while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
    fnorm = fnorm - 1.0;

    // calculate the binary form (non-float) of the significand data
    significand = fnorm * ((1LL<<significandbits) + 0.5f);

    // get the biased exponent
    exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

    // return the final answer
    return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
}

double unpack754(uint64_t i, unsigned bits, unsigned expbits)
{
    long double result;
    long long shift;
    unsigned bias;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (i == 0) return 0.0;

    // pull the significand
    result = (i&((1LL<<significandbits)-1)); // mask
    result /= (1LL<<significandbits); // convert back to float
    result += 1.0f; // add the one back on

    // deal with the exponent
    bias = (1<<(expbits-1)) - 1;
    shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
    while(shift > 0) { result *= 2.0; shift--; }
    while(shift < 0) { result /= 2.0; shift++; }

    // sign it
    result *= (i>>(bits-1))&1? -1.0: 1.0;

    return result;
}

unsigned char * serialize_uint_64(unsigned char *buffer, uint64_t value)
{
  buffer[0] = value >> 56;
  buffer[1] = value >> 48;
  buffer[2] = value >> 40;
  buffer[3] = value >> 32;
  buffer[4] = value >> 24;
  buffer[5] = value >> 16;
  buffer[6] = value >> 8;
  buffer[7] = value;
  return buffer + 8;
}

unsigned char* deserialize_uint_64(unsigned char *buffer, uint64_t* value)
{
  *value = (uint64_t)buffer[0]<<56;
  *value = *value + ((uint64_t)buffer[1]<<48);
  *value = *value + ((uint64_t)buffer[2]<<40);  
  *value = *value + ((uint64_t)buffer[3]<<32);
  *value = *value + ((uint64_t)buffer[4]<<24);
  *value = *value + ((uint64_t)buffer[5]<<16);
  *value = *value + ((uint64_t)buffer[6]<<8); 
  *value = *value + ((uint64_t)buffer[7]);    
  return buffer + 8;
}

#pragma pack(1)

struct x_and_u
{
  double x;
  double u;
  double z;
};

#pragma pack(0)

#pragma pack(1)

struct admm_z
{
  double z;
};

#pragma pack(0)

void send_msg_struct(int new_sd, unsigned char* buffer, admm_z msg_send, int len)
{
  unsigned char* point;
  ssize_t bytes_sent;
  uint64_t temp_64;
  temp_64 = pack754(msg_send.z,64,11);
  point = serialize_uint_64(buffer,temp_64); 
  bytes_sent = send(new_sd, buffer, len, 0);
  cout<<"\nbytes_sent: "<<bytes_sent<<"\n";
}

void send_msg_struct_cl(int socketfd, unsigned char* buffer, x_and_u msg_send, int len)
{
  unsigned char* point;
  ssize_t bytes_sent;
  uint64_t temp_64;
  temp_64 = pack754(msg_send.x,64,11);
  point = serialize_uint_64(buffer,temp_64);
  temp_64 = pack754(msg_send.u,64,11);
  point = serialize_uint_64(point,temp_64); 
  bytes_sent = send(socketfd, buffer, len, 0);
  cout<<"\nbytes_sent: "<<bytes_sent<<"\n";
}

void recv_msg_struct_cl(int socketfd, unsigned char* buffer, admm_z* msg_recv, int len)
{
  unsigned char* point;
  ssize_t bytes_recv;
  uint64_t temp_64;
  bytes_recv = recv(socketfd, buffer, len, 0);
  point = deserialize_uint_64(buffer, &temp_64);
  cout<<"bytes received: "<<bytes_recv<<endl;
  msg_recv->z = unpack754(temp_64,64,11);
}

void recv_msg_struct(int new_sd, unsigned char* buffer, x_and_u* msg_recv, int len)
{
  unsigned char* point;
  ssize_t bytes_recv;
  uint64_t temp_64;
  bytes_recv = recv(new_sd, buffer, len, 0);
  point = deserialize_uint_64(buffer, &temp_64);
  cout<<"bytes received: "<<bytes_recv<<endl;
  msg_recv->x = unpack754(temp_64,64,11);
  point = deserialize_uint_64(point,&temp_64); 
  msg_recv->u = unpack754(temp_64,64,11);
}

int create_socket(const char* s)
{
 int status;
 struct addrinfo host_info;       
 struct addrinfo *host_info_list; 
   
 memset(&host_info, 0, sizeof host_info);
 
 cout<<"Setting up the structs..."<<endl;
 
 host_info.ai_family = AF_UNSPEC;
 host_info.ai_flags = AI_PASSIVE;     
 host_info.ai_socktype = SOCK_STREAM; 
 
 status = getaddrinfo(s, "5532", &host_info, &host_info_list);

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
 return new_sd;
}

void initialize_addresses(vector<const char*>& adresses)
{
  //adresses[0] = "172.16.0.1";
  adresses[0] = "172.16.0.13";
  adresses[1] = "172.16.0.5";
}

int main ()
{
  int new_sd;
  ssize_t bytes_recieved, bytes_sent;
  unsigned char* point;
  uint64_t temp_64;
  x_and_u msg_recv, avg, msg_send_serv;
  admm_z msg_send, msg_recv_serv;
  double v = 4.0, rho = 0.5;
  int i, j, len_recv = 16, len_send = 8, no_of_sockets = 4, iterations = 30, no_of_neigh = 2, no_of_bridges = 1;
  int len_send_serv = 16, len_recv_serv = 8;
  vector<int> sockets(no_of_neigh,0);
  vector<const char*> address(no_of_neigh,"172.16.0.14");
  //clock_t t_1, t_2;
  initialize_addresses(address);
  for (i = 0; i < no_of_neigh; i++)
  {
    sockets[i] = create_socket(address[i]);
  }
  vector<double> u_b(no_of_bridges, 0.0);
  msg_send_serv.u = 0.0;
  msg_send_serv.x = (2*v + 0.0 - rho*msg_send_serv.u*no_of_bridges)/(2+rho*no_of_bridges); 
  //t_1 = clock();
  system_clock::time_point t_1 = system_clock::now();
  double sum_z_b = 0.0, sum_u_b = 0.0;
  for (j = 0; j < iterations; j++)
  {
    unsigned char buffer_send[8];
    avg.x = msg_send_serv.x;
    avg.u = msg_send_serv.u;
    for (i = 0; i < no_of_neigh; i++)
    {
      unsigned char buffer_recv[16];
      recv_msg_struct(sockets[i], &buffer_recv[0], &msg_recv, len_recv); 
      cout<<"\nFrom client "<<i+1<<":\n";
      cout<<"msg x: "<<msg_recv.x<<"\n";
      cout<<"msg u: "<<msg_recv.u<<"\n"; 
      avg.x = avg.x + msg_recv.x;
      avg.u = avg.u + msg_recv.u;     
    }
    avg.x = avg.x/(no_of_neigh + 1);
    avg.u = avg.u/(no_of_neigh + 1);
    msg_send.z = avg.x + avg.u;
    for (i = 0; i < no_of_neigh; i++)
    {
      send_msg_struct(sockets[i], &buffer_send[0], msg_send, len_send);
    }
     vector<double> z_b(no_of_bridges, 0.0);
    sum_z_b = 0.0;
    for (i = 0; i < no_of_bridges - 1; i++)
    {
      unsigned char buffer_recv_serv[8];
      recv_msg_struct_cl(sockets[i], &buffer_recv_serv[0], &msg_recv_serv, len_recv_serv);
      cout<<"msg z: "<<msg_recv_serv.z<<"\n";
      z_b[i] = msg_recv_serv.z;
      sum_z_b = sum_z_b + z_b[i];
    }
    sum_z_b = sum_z_b + msg_send.z;
    msg_send_serv.x = (2*v + rho*sum_z_b - rho*sum_u_b)/(2+rho*no_of_bridges);
    sum_u_b= 0.0;
    for (i = 0; i < no_of_bridges - 1; i++)
    {
      unsigned char buffer_send_serv[16];
      u_b[i] = u_b[i] + msg_send_serv.x - z_b[i];
      sum_u_b = sum_u_b + u_b[i];
      msg_send_serv.u = u_b[i];
      send_msg_struct_cl(sockets[i], &buffer_send_serv[0], msg_send_serv, len_send_serv);
    }
    u_b[no_of_bridges-1] = u_b[no_of_bridges-1] + msg_send_serv.x - msg_send.z;
    sum_u_b = sum_u_b + u_b[no_of_bridges-1];
    msg_send_serv.u = u_b[no_of_bridges-1];
  }
  //t_2 = clock();
  system_clock::time_point t_2 = system_clock::now();
  //float seconds = (float(t_2) - float(t_1))/CLOCKS_PER_SEC;
  duration<double> elapsed_time = t_2 - t_1;
  cout<<"\nConsensus value: "<<msg_send.z<<"\n";
  cout<<"Value of x: "<<msg_send_serv.x<<"\n";
  cout<<"Value of v: "<<v<<"\n";
 // cout<<"seconds: "<<seconds<<"\n";
  cout<<"duration: "<<elapsed_time.count()<<"\n";
  return 0;
}
