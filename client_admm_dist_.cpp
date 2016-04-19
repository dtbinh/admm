#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netdb.h>
#include <string>
#include <vector>

using namespace std;

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
};

#pragma pack(0)

#pragma pack(1)

struct admm_z
{
  double z;
};

#pragma pack(0)

void send_msg_struct(int socketfd, unsigned char* buffer, x_and_u msg_send, int len)
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

void recv_msg_struct(int socketfd, unsigned char* buffer, admm_z* msg_recv, int len)
{
  unsigned char* point;
  ssize_t bytes_recv;
  uint64_t temp_64;
  bytes_recv = recv(socketfd, buffer, len, 0);
  point = deserialize_uint_64(buffer, &temp_64);
  cout<<"bytes received: "<<bytes_recv<<endl;
  msg_recv->z = unpack754(temp_64,64,11);
}

int create_socket(const char* s)
{
   int status;
  struct addrinfo host_info;
  struct addrinfo *host_info_list;

  memset(&host_info, 0, sizeof host_info);

  host_info.ai_family = AF_UNSPEC;
  host_info.ai_socktype = SOCK_STREAM;
  status = getaddrinfo(s, "5532", &host_info, &host_info_list);
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
  return socketfd;
}

void initialize_addresses(vector<const char*>& adresses)
{
  adresses[0] = "172.16.0.13";
  adresses[1] = "172.16.0.1";
}

int main ()
{
  x_and_u msg_send;
  admm_z msg_recv;
  double rho = 0.5, v = 14.0;
  uint64_t temp_64;
  int i, j, len_recv = 8, len_send = 16, iterations = 30, no_of_bridges = 2;
  ssize_t bytes_sent, bytes_recv;
  vector<int> sockets(no_of_bridges,0);
  vector<const char*> address(no_of_bridges,"172.16.0.14");
  initialize_addresses(address);
  for (i = 0; i < no_of_bridges; i++)
  {
    sockets[i] = create_socket(address[i]);
  }
  msg_send.u = 0.0;
  msg_send.x = (2*v + 0.0 - msg_send.u)/(2+rho); 
  vector<double> u_b(no_of_bridges, 0.0);
  vector<double> z_b(no_of_bridges, 0.0);
  double sum_z_b = 0.0, sum_u_b = 0.0;
  for (j = 0; j < iterations; j++)
  {
    msg_send.x = (2*v + rho*sum_z_b - rho*sum_u_b)/(2+rho*no_of_bridges);
    sum_u_b = 0.0;
    for (i = 0; i < no_of_bridges; i++)
    {
      unsigned char buffer_send[16];
      u_b[i] = u_b[i] + msg_send.x - z_b[i];
      msg_send.u = u_b[i];
      sum_u_b = sum_u_b + u_b[i];
      send_msg_struct(sockets[i], &buffer_send[0], msg_send, len_send);
    }
    sum_z_b = 0.0;
    for (i = 0; i < no_of_bridges; i++)
    {
      unsigned char buffer_recv[8];
      recv_msg_struct(sockets[i], &buffer_recv[0], &msg_recv, len_recv);
      cout<<"msg z: "<<msg_recv.z<<"\n";
      z_b[i] = msg_recv.z;
      sum_z_b = sum_z_b + z_b[i];
    }
    
  }
  cout<<"\nValue of x: "<<msg_send.x<<"\n";
  cout<<"Value of v: "<<v<<"\n";
  return 0;
}


