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
#include <cmath>

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
void str_to_hex1(const char* in, char* out, const char* offset)
{
  int in_idx = 0, out_idx = 0,cnt1 = 0;
  while (cnt1 != 4)
  {
    sprintf((out +out_idx), "%02x%s", in[in_idx], offset);
    in_idx += 1; out_idx += (strlen(offset)+1);
    cnt1++;
  }
}
void str_to_hex2(const char* in, char* out, const char* offset)
{
  int in_idx = 0, out_idx = 0,cnt1 = 0;
  while (cnt1 != 3)
  {
    sprintf((out +out_idx), "%02x%s", in[in_idx], offset);
    in_idx += 1; out_idx += (strlen(offset)+1);
    cnt1++;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "vc_gps_pub");
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
 unsigned int r_time;


 char in1[100] = "";
 char out1[100] = {0x00, };
 char in2[100] = "";
 char out2[100] = {0x00, };
 char in3[100] = "";
 char out3[100] = {0x00, };
 char in4[100] = "";
 char out4[100] = {0x00, };
 char in5[100] = "";
 char out5[100] = {0x00, };
 char in6[100] = "";
 char out6[100] = {0x00, };
 char in7[2] = "";
 char out7[2]; 

 double Hu_latitude = 0.0;
 double Hu_Longitude = 0.0;
 float Hu_Altitude = 0.0;
 char Hu_Satelliotes[10] = "";
 char Hu_quality[10] = "";

 double caddy_latitude = 0.0;
 double caddy_Longitude = 0.0;
 float caddy_Altitude = 0.0;
 char caddy_quality[10] = "";
 char caddy_Satelliotes[10] = "";
 unsigned int caddy_real_time = 0;



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
   if(rx_length < 0){
    r_time = ros::Time::now().sec;
    //printf("%d\n", r_time);
    printf("error\n");
   }
   else if(rx_length == 0)
    printf("no data\n");
   else{
    rx_buf[rx_length] = '\0';
    printf("%i byte : %s\n",rx_length,rx_buf);
    r_time = ros::Time::now().sec;
    for (int  i = 2; i < 4; i++)
    {
      in7[4-i] = rx_buf[i];
      //printf("%d\n", in7[4-i]);
    }
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

    for (int  i = 22; i < 26; i++)
    {
      in5[25-i] = rx_buf[i];
      //printf("%d\n", in5[25-i]);
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

    for (int  i = 54; i < 58; i++)
    {
      in6[57-i] = rx_buf[i];
      //printf("%d\n", in6[57-i]);
    }
   
    std::stringstream time_data;
    std::string str_time_data;
    unsigned int UTC_Time_data = 0;

    time_data << std::hex << r_time;
    str_time_data = time_data.str();
    stringstream convert(str_time_data);
    convert >> std::hex >> UTC_Time_data;
    UTC_Time_data = UTC_Time_data & 0xffff0000;
    
    cout << UTC_Time_data << endl;

    caddy_quality[0] =rx_buf[58];
    //printf("%d\n",caddy_quality[0]);
    Hu_quality[0] =rx_buf[58];
    //printf("%d\n",Hu_quality[0]);

    
    Hu_Satelliotes[0] =rx_buf[28];
    //printf("%d\n",Hu_Satelliotes[0]);
    caddy_Satelliotes[0] =rx_buf[59];
    //printf("%d\n",caddy_Satelliotes[0]);
    
    if(Hu_quality[0] == 1 || Hu_quality[0] == 2){Hu_quality[1] = -1;}
    if(Hu_quality[0] == 3){Hu_quality[1] = 0;}
    if(Hu_quality[0] == 4){Hu_quality[1] = 1;}
    if(Hu_quality[0] == 5){Hu_quality[1] = 2;}
    if(caddy_quality[0] == 1 || caddy_quality[0] == 2){caddy_quality[1] = -1;}
    if(caddy_quality[0] == 3){caddy_quality[1] = 0;}
    if(caddy_quality[0] == 4){caddy_quality[1] = 1;}
    if(caddy_quality[0] == 5){caddy_quality[1] = 2;}

    const double varH = pow(rx_buf[61] / 1000.0, 2);
    const double varV = pow(rx_buf[60] / 1000.0, 2);
    // printf("%d\n", rx_buf[60]);
    // printf("%d\n", rx_buf[61]);
    // printf("%.32f\n",varH);
    // printf("%.32f\n",varV);
    str_to_hex(in1, out1, " ");
    str_to_hex(in2, out2, " ");
    str_to_hex(in3, out3, " ");
    str_to_hex(in4, out4, " ");
    str_to_hex1(in5, out5, " ");
    str_to_hex1(in6, out6, " ");
    str_to_hex2(in7, out7, " ");
    //cout<<"change before : " << in1 << endl;
    //cout<<"change after : " << out1 << endl;
    
    long unsigned int num1;    
    long unsigned int num2; 
    long unsigned int num3; 
    long unsigned int num4;
    long unsigned int num5; 
    long unsigned int num6;  
    long unsigned int num7;
    
    if(rx_length > 32){
      sscanf(out1, "%lx\n", &num1);
      caddy_latitude = *((double*)&num1);
      sscanf(out2, "%lx\n", &num2);
      caddy_Longitude = *((double*)&num2);
      sscanf(out3, "%lx\n", &num3);
      Hu_latitude = *((double*)&num3);
      sscanf(out4, "%lx\n", &num4);
      Hu_Longitude = *((double*)&num4);
      sscanf(out5, "%lx\n", &num5);
      Hu_Altitude = *((float*)&num5);
      sscanf(out6, "%lx\n", &num6);
      caddy_Altitude = *((float*)&num6);
      sscanf(out7, "%lx\n", &num7);
      UTC_Time_data = UTC_Time_data | num7;
      //cout << num7 << endl;
      //cout << UTC_Time_data << endl;

      printf("%08lx %.8f\n",num1,caddy_latitude);
      printf("%08lx %.8f\n",num2,caddy_Longitude);
      printf("%08lx %.8f\n",num3,Hu_latitude);
      printf("%08lx %.8f\n",num4,Hu_Longitude);
      printf("%04lx %.2f\n",num5,Hu_Altitude);
      printf("%04lx %.2f\n",num6,caddy_Altitude);
      printf("%d\n",UTC_Time_data);

      msg1.latitude = caddy_latitude;
      msg1.longitude = caddy_Longitude;
      msg1.altitude = caddy_Altitude;
      msg1.status.status = caddy_quality[1];
      msg1.status.service = caddy_Satelliotes[0];
      //msg1.header.stamp = ros::Time::now();
      msg1.header.stamp.sec = UTC_Time_data;
      msg1.header.frame_id = "vc_gps_cart_link";
      msg1.position_covariance[0] = varH;
      msg1.position_covariance[4] = varH;
      msg1.position_covariance[8] = varV;
      msg1.position_covariance_type = 
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      msg2.latitude = Hu_latitude;
      msg2.longitude = Hu_Longitude;
      msg2.altitude = Hu_Altitude;
      msg2.status.status = Hu_quality[1];
      msg2.status.service = Hu_Satelliotes[0];
      //msg2.header.stamp = ros::Time::now();
      msg2.header.stamp.sec = UTC_Time_data;
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