#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/fcntl.h>
#include <termios.h>
#include <string>
#include <iostream>
#include <cstring>
#include <sstream>
#include <typeinfo>

using namespace std;

void str_to_hex(const char* in, char* out, const char* offset)
{
  int in_idx = 0, out_idx = 0,cnt1 = 0;
  while (cnt1 != 8)
  {
    sprintf((out +out_idx), "%02x%s", in[in_idx], offset);
    in_idx += 1; out_idx += (strlen(offset)+1);
    cnt1++;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ros_tutorial_msg_publisher");
  ros::NodeHandle nh1;
  ros::NodeHandle nh2;
  ros::Publisher pub1 = nh1.advertise<sensor_msgs::NavSatFix>("vc_gps_caddy", 10);
  ros::Publisher pub2 = nh2.advertise<sensor_msgs::NavSatFix>("vc_gps_hu", 10);
  sensor_msgs::NavSatFix msg1;
  sensor_msgs::NavSatFix msg2;
  ros::Rate loop_rate(1);

 printf("UART\n");
 int i = 1; 
 unsigned int j = 0;
 int hex = 0;
 int count = 1;
 char in1[100] = "";
 char out1[100] = {0x00, };
 char in2[100] = "";
 char out2[100] = {0x00, };
 char in3[100] = "";
 char out3[100] = {0x00, };
 char in4[100] = "";
 char out4[100] = {0x00, };
 double Hu_latitude = 0.0;
 double Hu_Longitude = 0.0;
 double caddy_latitude = 0.0;
 double caddy_Longitude = 0.0;
 int uart_sys = -1;
 int flag = 0;
 uart_sys = open("/dev/ttyUSB1", O_RDWR|O_NOCTTY|O_NDELAY);
 if(uart_sys == -1)
  printf("ERR unable to open UART\n");

 struct termios options;
 tcgetattr(uart_sys, &options);
 options.c_cflag = B115200|CS8|CLOCAL|CREAD;
 options.c_iflag = IGNPAR;
 options.c_oflag = 0;
 options.c_lflag = 0;
 tcflush(uart_sys,TCIFLUSH);
 tcsetattr(uart_sys,TCSANOW, &options);

 unsigned char tx_buf[20];
 unsigned char *p_tx_buf;

 p_tx_buf = &tx_buf[0];

 *p_tx_buf++ = '0';
 *p_tx_buf++ = '1';
 
 //p_tx_buf[0] = '0';
 //p_tx_buf[1] = '1';


 while(ros::ok()){
  // if(uart_sys != -1){
  //  int count = write(uart_sys,&tx_buf[i%2],sizeof(char));
  // }
  count++;
  if(uart_sys != -1){
  
   unsigned char rx_buf[255] = "/0";
   int rx_length = read(uart_sys,(void*)rx_buf,64);
   //int temp = read(uart_sys,(void*)rx_buf,64);
   //printf("%d", temp);
   int temp = 1;
   while(temp != -1){temp = read(uart_sys,(void*)rx_buf,64);}
   if(rx_length < 0)
    printf("rx err\n");
   else if(rx_length == 0)
    printf("no data\n");
   else{
    rx_buf[rx_length] = '\0';
    printf("%i byte : %s\n",rx_length,rx_buf);
    for (int  i = 6; i < 14; i++)
    {
      in1[13-i] = rx_buf[i];
      //printf("%d\n", in1[13-i]);
    }
    
    for (int  i = 14; i < 22; i++)
    {
      in2[21-i] = rx_buf[i];
      //printf("%d\n", in2[21-i]);
    }
    
    for (int  i = 38; i < 46; i++)
    {
      in3[45-i] = rx_buf[i];
      //printf("%d\n", in3[45-i]);
    }
    
    for (int  i = 46; i < 54; i++)
    {
      in4[53-i] = rx_buf[i];
      //printf("%d\n", in4[53-i]);
    }

    str_to_hex(in1, out1, " ");
    str_to_hex(in2, out2, " ");
    str_to_hex(in3, out3, " ");
    str_to_hex(in4, out4, " ");
    //cout<<"change before : " << in1 << endl;
    //cout<<"change after : " << out1 << endl;
    
    long unsigned int num1;    
    long unsigned int num2; 
    long unsigned int num3; 
    long unsigned int num4; 
    
    if(rx_length > 32){
      sscanf(out1, "%lx\n", &num1);
      caddy_latitude = *((double*)&num1);
      sscanf(out2, "%lx\n", &num2);
      caddy_Longitude = *((double*)&num2);
      sscanf(out3, "%lx\n", &num3);
      Hu_latitude = *((double*)&num3);
      sscanf(out4, "%lx\n", &num4);
      Hu_Longitude = *((double*)&num4);
      printf("%08lx %.8f\n",num1,caddy_latitude);
      printf("%08lx %.8f\n",num2,caddy_Longitude);
      printf("%08lx %.8f\n",num3,Hu_latitude);
      printf("%08lx %.8f\n",num4,Hu_Longitude);
      msg1.latitude = caddy_latitude;
      msg1.longitude = caddy_Longitude;
      msg1.status.status = 2;
      msg1.header.stamp = ros::Time::now();
      msg1.header.frame_id = "vc_gps_cart_link";
      msg2.latitude = Hu_latitude;
      msg2.longitude = Hu_Longitude;
      msg2.status.status = 2;
      msg2.header.stamp = ros::Time::now();
    } else {
      sscanf(out1, "%lx\n", &num1);
      caddy_latitude = *((double*)&num1);
      sscanf(out2, "%lx\n", &num2);
      caddy_Longitude = *((double*)&num2);
      msg1.latitude = caddy_latitude;
      msg1.longitude = caddy_Longitude;
    }
   }
  }
  
  pub1.publish(msg1);
  pub2.publish(msg2);

  loop_rate.sleep();
 }
  return 0;
}