#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netdb.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

using std::mutex;
using std::thread;
using std::string;
using std::cout;
using std::endl;
using std::vector;

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

void recv_msg_int(int socketfd, int* msg_recv)
{
  unsigned char* point;
  ssize_t bytes_recv;
  uint64_t temp_64;
  unsigned char buffer[8];
  int len = 8;
  bytes_recv = recv(socketfd, &buffer[0], len, 0);
  point = deserialize_uint_64(buffer, &temp_64);
  *msg_recv = temp_64;
  //cout<<"msg_recv: "<<*msg_recv<<"\n";
}

void send_msg_string(int socketfd, char* msg)
{
 int len;
 ssize_t bytes_sent;
 len = strlen(msg);
 bytes_sent = send(socketfd, msg, len, 0);
 cout<<"\nbytes_sent: "<<bytes_sent<<"\n";
}
 

void create_socket_server(const char* s, char& connect_type, int& sockets_server, mutex& mtx)
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
 //cout<<"before listen.\n";
 status =  listen(socketfd, 5);
 //cout<<"after listen.\n";
 if (status == -1)  
 {
   cout<< "listen error"<<endl;
 }
 //cout<<"after if.\n";
 int new_sd;
 struct sockaddr_storage their_addr;
 socklen_t addr_size = sizeof(their_addr);
 //cout<<"before accept.\n";
 new_sd = accept(socketfd, (struct sockaddr *)&their_addr, &addr_size);
 if (new_sd == -1)
 {
    cout << "listen error" << std::endl ;
 }
 else
 {
   cout << "Connection accepted. Using new socketfd : "<<new_sd<<endl;
   mtx.lock();
   connect_type = 's';
   mtx.unlock();
 }
 sockets_server = new_sd;
}

void create_socket_client(const char* s, char& connect_type, int& socket_client, mutex& mtx)
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
  bool error_once = 0;
  while(1)
  {
  status =  connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
  if (status == -1 && error_once == 0)
  {
    cout<<"connect error \n";
    error_once = 1;
  }
  else if (status == 0)
  {
    mtx.lock();
    connect_type = 'c';
    mtx.unlock();
    cout << "send()ing message..."  <<endl;
    cout<<"status: "<<status<<"\n";
    break;
  }
  }
  socket_client = socketfd;
}

struct attributes
{
  char* ip;
  const char* inter_this;
  const char* inter_neigh;
  int neigh_ind; 
  int node_deg;
  bool active;
  char connect_type;
};

void initialize_address_attr(const char* inter_this, const char* inter_neigh, attributes& neighbor_attr, char* ip_,
     int index, int degree, bool act)
{
  //addr = inter_const;
  neighbor_attr.inter_this = inter_this;
  neighbor_attr.inter_neigh = inter_neigh;
  neighbor_attr.ip = ip_;
  neighbor_attr.neigh_ind = index;
  neighbor_attr.node_deg = degree;
  neighbor_attr.active = act;
}

void initialize_all_addresses(vector<const char*>& addresses, vector<attributes>& all_neighbors_attr)
{
  addresses[0] = "172.16.0.5";
  //adresses[1] = "172.16.0.18";
  //all_neighbors_attr[0].inter = "172.16.0.5";
  //all_neighbors_attr[1].inter = "172.16.0.18";
  //all_neighbors_attr[0].neigh_ind = 0;
  //all_neighbors_attr[1].neigh_ind = 1;
  //all_neighbors_attr[0].node_deg = 2;
  //all_neighbors_attr[1].node_deg = 2;
  //all_neighbors_attr[0].active = 1;
  //all_neighbors_attr[1].active = 1;
  initialize_address_attr("172.16.0.6", "172.16.0.5", all_neighbors_attr[0], "147.72.248.19", 0, 1, 1);
  initialize_address_attr("172.16.0.17", "172.16.0.18", all_neighbors_attr[1], "147.72.248.17", 0, 1, 0);
}

//void intialize_neighbors(vector<char*>& all_neighbors, vector<char*>& all_neighbors_ip)
//{
//  all_neighbors[0] = "172.16.0.5";
//  all_neighbors[1] = "172.16.0.18";
//}

void check_neighbors(vector<int>& temp_vec, string& s, vector<attributes>& all_neighbors_attr)
{
  for(int k = 0; k < all_neighbors_attr.size(); k++)
  {
    string s2(all_neighbors_attr[k].ip);
    if (s == s2 && (all_neighbors_attr[k].active == 1 || all_neighbors_attr[k].active == 0))
    { 
      cout<<"\ninside if, ip: "<<s2<<"\n";
      temp_vec.push_back(k);
      break;
    }
  }
}

void receive_bridge_neighbors(vector<attributes>& all_neighbors_attr, vector<int>& sockets, 
     vector< vector<int> >& bridge_neighbors, int no_of_bridges)
{
  int i;
  for (i = 0; i < no_of_bridges; i++)
  {
    vector<int> temp_vec;
    int j, no_of_neigh;
    recv_msg_int(sockets[i], &no_of_neigh);
    cout<<"\nbridge "<<i<<": ";
    cout<<"\nno of neighbors: "<<no_of_neigh<<"\n";
    for (j = 0; j < no_of_neigh; j++)
    {
      ssize_t bytes_recieved;
      char incoming_buffer[13];
      bytes_recieved = recv(sockets[i], incoming_buffer, 13, 0);  // receiving address of bridge node neighbor
      string s(incoming_buffer);
      cout<<"ip: "<<s<<"\n";
      check_neighbors(temp_vec, s, all_neighbors_attr);
    }
    bridge_neighbors.push_back(temp_vec);
  }
}

  
int main ()
{
  x_and_u msg_send;
  admm_z msg_recv;
  double rho = 0.5, v = 14.0, latency_limit = 20.0, latency = 50.0;
  uint64_t temp_64;
  int i, j, len_recv = 8, len_send = 16, iterations = 0, no_of_neigh = 1, no_of_bridges = 1, total_neigh = 2;
  ssize_t bytes_sent, bytes_recv;
  vector<int> sockets_server(total_neigh,0);
  vector<int> sockets_client(total_neigh, 0);
  vector<const char*> address(no_of_bridges,"172.16.0.14");
  mutex mtx_1, mtx_2;
  //vector<char*> all_neighbors(total_neigh, "172.16.0.14");
  attributes temp;
  temp.inter_this = "172.16.0.14";
  temp.connect_type = '0';
  vector<attributes> all_neighbors_attr(total_neigh, temp);
  vector< vector<int> > bridge_neighbors;
  initialize_all_addresses(address, all_neighbors_attr); 
  for (i = 0; i < total_neigh; i++)
  {
    thread first(create_socket_server, all_neighbors_attr[i].inter_this, std::ref(all_neighbors_attr[i].connect_type), std::ref(sockets_server[i]), std::ref(mtx_1));
    thread second(create_socket_client, all_neighbors_attr[i].inter_neigh, std::ref(all_neighbors_attr[i].connect_type), std::ref(sockets_client[i]), std::ref(mtx_2));
    first.join();
    second.join();
  }
  //intialize_neighbors(all_neighbors, all_neighbors_ip);
  //initialize_bridge_neighbors(bridge_neighbors);
  for (i = 0; i < total_neigh; i++)
  {
    cout<<"\nneighbor "<<i+1<<": \n";
    cout<<"connect_type: "<<all_neighbors_attr[i].connect_type<<"\n";
  }
  send_msg_string(sockets_server[0], "This is from server.");
  send_msg_string(sockets_client[0], "This is from client.");
 
  receive_bridge_neighbors(all_neighbors_attr,sockets_client,bridge_neighbors,no_of_bridges);
  msg_send.u = 0.0;
  msg_send.x = (2*v + 0.0 - msg_send.u)/(2+rho); 
  vector<double> u_b(no_of_bridges, 0.0);
  vector<double> z_b(no_of_bridges, 0.0);
  double sum_z_b = 0.0, sum_u_b =0.0;
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
      send_msg_struct(sockets_client[i], &buffer_send[0], msg_send, len_send);
    }
    sum_z_b = 0.0;
    for (i = 0; i < no_of_bridges; i++)
    {
      unsigned char buffer_recv[8];
      recv_msg_struct(sockets_client[i], &buffer_recv[0], &msg_recv, len_recv);
      cout<<"msg z: "<<msg_recv.z<<"\n";
      z_b[i] = msg_recv.z;
      sum_z_b = sum_z_b + z_b[i];
    }
    
  }
  //cout<<"\nValue of x: "<<msg_send.x<<"\n";
  //cout<<"Value of v: "<<v<<"\n";
  return 0;
}


