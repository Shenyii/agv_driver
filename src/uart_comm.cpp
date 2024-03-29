#include "uart_comm.h"
#include "odometry.h"
#include <termios.h>

#define FALSE -1  
#define TRUE 0

int speed_arr[9] = {B500000, B38400, B19200, B115200, B9600, B4800, B2400, B1200, B300 };
int name_arr[9]  = {500000, 38400,  19200, 115200, 9600,  4800,  2400,  1200,  300 };
double velTime = 0.0;  //receive cmd_vel time
int UartOpenFlag = 0;
BaseData basedata;


UARTComm::UARTComm()
{
    pub_ = n_.advertise<std_msgs::Int8>("/battery_voltage", 1);
    pub_camera_pose_ = n_.advertise<geometry_msgs::Pose2D>("/camera_pose",1);
}

void UARTComm::set_speed(int fd,int speed)
{
  int   i;
  int   status;
  struct termios   Opt;
  tcgetattr(fd, &Opt);
  for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) 
  {
    if (speed == name_arr[i]) 
    {
      tcflush(fd, TCIOFLUSH);
      cfsetispeed(&Opt, speed_arr[i]);
      cfsetospeed(&Opt, speed_arr[i]);
      status = tcsetattr(fd, TCSANOW, &Opt);
      if (status != 0) 
      {
        perror("tcsetattr fd1");
        return;
      }
      tcflush(fd,TCIOFLUSH);
    }
  }
}

int UARTComm::set_Parity(int fd,int databits,int stopbits,int parity)
{
  struct termios options;   
  if ( tcgetattr(fd,&options) != 0) 
  {   
    perror("SetupSerial 1");       
    return(FALSE);    
  }  
  options.c_cflag &= ~CSIZE;   
  switch (databits)   
  {     
    case 7:       
        options.c_cflag |= CS7;   
        break;  
    case 8:       
        options.c_cflag |= CS8;  
        break;     
    default:      
        fprintf(stderr,"Unsupported data size\n"); return (FALSE);    
  }  
  switch (parity)   
  {     
    case 'n':  
    case 'N':      
        options.c_cflag &= ~PARENB;   /* Clear parity enable */  
        options.c_iflag &= ~INPCK;    /* Enable parity checking */   
        break;    
    case 'o':     
    case 'O':       
        options.c_cflag |= (PARODD | PARENB);   
        options.c_iflag |= INPCK;     /* Disnable parity checking */   
        break;    
    case 'e':    
    case 'E':     
        options.c_cflag |= PARENB;    /* Enable parity */      
        options.c_cflag &= ~PARODD;      
        options.c_iflag |= INPCK;     /* Disnable parity checking */  
        break;  
    case 'S':   
    case 's':                         /*as no parity*/     
        options.c_cflag &= ~PARENB;  
        options.c_cflag &= ~CSTOPB;break;    
    default:     
        fprintf(stderr,"Unsupported parity\n");      
        return (FALSE);    
  }    
      
  switch (stopbits)  
  {     
    case 1:      
        options.c_cflag &= ~CSTOPB;    
        break;    
    case 2:      
        options.c_cflag |= CSTOPB;    
        break;  
    default:      
        fprintf(stderr,"Unsupported stop bits\n");    
        return (FALSE);   
  }   
  /* Set input parity option */   
  if (parity != 'n')     
      options.c_iflag |= INPCK;

  options.c_cflag |= CLOCAL;
  options.c_cflag |= CREAD;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag  &= ~OPOST;   /*Output*/
  options.c_iflag &= ~(ICRNL | IXON);
   
  tcflush(fd,TCIFLUSH);  
  options.c_cc[VTIME] = 10;   
  options.c_cc[VMIN]  = 14;     /* Update the options and do it NOW */  
  if (tcsetattr(fd,TCSANOW,&options) != 0)     
  {   
    perror("SetupSerial 3");     
    return (FALSE);    
  }   
  return (TRUE); 
}

int left_dis=0;
int right_dis=0;
int left_stop=0;
int right_stop=0;
int count_false=0;
long Lcode_temp0,Rcode_temp0,Lcode_temp1,Rcode_temp1;
short imu_init=0;
int UARTComm::RecvBaseBuf(unsigned char *rbuf)
{

  int i=0;
  unsigned char checksum = 0;
  //debug output 
  //printf("rbuf=");
  //for(i=0;i<22;i++)printf("%d ",rbuf[i]);
  //printf("\n");

  if(rbuf == NULL)
  {
    count_false++;
    printf("receive buf is null!\n");
    return 0;
  }
  else
  {
    if(((rbuf[0]&0xff) != 0x23) || ((rbuf[21]&0xff) != 0x32))
    {
      count_false++;
      printf("buf head or tail fault! buf[0]=%x,buf[21]=%x\n",rbuf[0],rbuf[22]);
      return 0;
    }
    
    for(int i=1; i<20; i++)
    {
      checksum += rbuf[i];
    }
    
    if((rbuf[20]&0xff) != (checksum & 0xff))
    {
      count_false ++;
      printf("checksum fault!=%x\n",(checksum &0xff));
      return 0;
    }

    geometry_msgs::Pose2D camera_pose_pub;
    short swap_value;
    if(rbuf[1] == 0)
    {
        camera_pose_pub.x = rbuf[2] / 1000.0;
    }
    else
    {
        swap_value = (rbuf[1] * 0x80 + rbuf[2] + 0xc000) & 0xffff;
        camera_pose_pub.x = swap_value / 1000.0;
    }
    if(rbuf[3] == 0)
    {
        camera_pose_pub.y = rbuf[4] / 1000.0;
    }
    else
    {
        swap_value = (rbuf[3] * 0x80 + rbuf[4] + 0xc000) & 0xffff;
        camera_pose_pub.y = swap_value / 1000.0;
    }
    camera_pose_pub.theta = (rbuf[5] * 0x80 + rbuf[6]) * 3.14159265 / 180.0;
    pub_camera_pose_.publish(camera_pose_pub);

	
    //ultrasonic [1-6]
    basedata.ultrasonic1 = rbuf[1]&0xff;
    basedata.ultrasonic2 = rbuf[2]&0xff;
    basedata.ultrasonic3 = rbuf[3]&0xff;
    basedata.ultrasonic4 = rbuf[4]&0xff;
    basedata.ultrasonic5 = rbuf[5]&0xff;
    basedata.ultrasonic6 = rbuf[6]&0xff;

    //bumper [7]
    basedata.bump1   = (rbuf[7]>>7) & 0x01;
    basedata.bump2   = (rbuf[7]>>6) & 0x01;
    basedata.bump3   = (rbuf[7]>>5) & 0x01;
    basedata.bump4   = (rbuf[7]>>4) & 0x01;
    basedata.bump5   = (rbuf[7]>>3) & 0x01;
    basedata.bump6   = (rbuf[7]>>2) & 0x01;
    basedata.Lcode_  = (rbuf[7]>>1) & 0x01;
    basedata.Rcode_  = (rbuf[7]>>0) & 0x01;

    //battery [8]
    basedata.battery = rbuf[8]&0xff;
    std_msgs::Int8 voltage;
    voltage.data = rbuf[8]&0xff;
    if(voltage.data != 0)
    {
        pub_.publish(voltage);
    }
	
    //encode [9-16]
    Lcode_temp0 = Lcode_temp1;
    Lcode_temp1 = ((rbuf[9]<<24)&0xff000000) | ((rbuf[10]<<16) & 0xff0000) | ((rbuf[11]<<8)&0xff00) | ((rbuf[12]) & 0xff);
    if(firstrecvflag==2) basedata.Lcode = Lcode_temp1-Lcode_temp0;
    Rcode_temp0 = Rcode_temp1;
    Rcode_temp1 = ((rbuf[13]<<24)&0xff000000) | ((rbuf[14]<<16) & 0xff0000) | ((rbuf[15]<<8)&0xff00) | ((rbuf[16]) & 0xff);
    if(firstrecvflag==2) basedata.Rcode = Rcode_temp0-Rcode_temp1;
     
    //IMU [17-18]
    if(firstrecvflag==0)
    {
       imu_init = (((rbuf[17]<<8) & 0xff00) | ((rbuf[18]) & 0xff));
       basedata.IMU = 0;
    }
    if(firstrecvflag!=0)
       basedata.IMU = (((rbuf[17]<<8) & 0xff00) | ((rbuf[18]) & 0xff)) - imu_init;
    
    //base_state [19]
    basedata.base_state  = rbuf[19]&0xff;
 
    //printf("left_dis=%f,right_dis=%f,Lcode=%d,Rcode=%d\n",(left_dis/100.0),(right_dis/100.0),basedata.Lcode,basedata.Rcode);
    //printf("ult1=%d,ult2=%d,ult3=%d,IMU=%d,Lcode=%d,Rcode=%d,state1=%d,%d,%d,%d,%d,%d,%d,%d,state2=%d,%d,%d,%d,%d,%d\n",basedata.ultrasonic1,basedata.ultrasonic2,basedata.ultrasonic3,basedata.IMU,basedata.Lcode,basedata.Rcode,basedata.wheelstate,basedata.occupystate,basedata.chargestate,basedata.lowbatterystate,basedata.ultstate3,basedata.ultstate2,basedata.ultstate1,basedata.infrared,basedata.fallstate5,basedata.fallstate4,basedata.fallstate3,basedata.fallstate2,basedata.fallstate1,basedata.all3state);
    
  }
  return 1;
}

void UARTComm::SendBuf(unsigned char *sbuf,bool flag,short Lvel,short Rvel)
{
  unsigned char checksum = 0;
  //Lvel = Lvel*1000;
  //Rvel = Rvel*1000;
  sbuf[0] = 0x23;
  sbuf[1] = (Lvel>> 8)&0xFF;
  sbuf[2] = Lvel&0xFF;
  sbuf[3] = (Rvel>> 8)&0xFF;
  sbuf[4] = Rvel&0xFF;
 
  for(int i=1; i<5; i++)
  {
    checksum += sbuf[i];
  }
  sbuf[5] = checksum & 0xff;
  sbuf[6] = 0x32;
  //printf("\n lvel=%d,rvel=%d \r\n",Lvel,Rvel);
}

void UARTComm::UARTComm_CallBack(geometry_msgs::Twist msg)
{
    //printf("UARTComm_CallBack!\n");
    vel_x  = msg.linear.x;  //last_cmd_vel.linear.x / 1.0; //vel ratio
    vel_th = msg.angular.z;  //last_cmd_vel.angular.z;
  
    vel_x = 1000 * vel_x;
    if(vel_x == 0)
    {
      right_vel = vel_th * width_robot / 2.0;
      left_vel = (-1) * right_vel;
    }
    else if(vel_th == 0)
    {
      left_vel = right_vel = vel_x;
    }
    else
    {
      left_vel = vel_x - vel_th * width_robot / 2.0;
      right_vel = vel_x + vel_th * width_robot / 2.0;
    }

}

extern double dtime;
double left_vel=0,right_vel=0;
geometry_msgs::Twist cmd_vel;
unsigned char firstrecvflag;  //first recieve init odometry
void UARTComm::Uart_Main()
{
  int fd;
  int len,fs_sel;
  fd_set fs_read;
  struct timeval time;
  int recvflag = 0;
  unsigned int IMU_Init = 0;
  int IMU_send = 0;
  int timeout = 0;
  unsigned char rbuff[22];   
  int  nread;    
  unsigned char sbuf[5];
  int  retv;
  int  length;  
  double sumtime = 0;
  firstrecvflag = 0;
  Odometry odom;

  printf("open /dev/agv_driver...\n");
  while(timeout<30) //almost 3s break
  {  
    //fd = open("/dev/ttyUSB0",O_RDWR|O_NOCTTY|O_NDELAY); 
    fd = open("/dev/agv_driver",O_RDWR|O_NOCTTY|O_NDELAY); 
    usleep(1000*100); 
    if(fd == -1)  
    {        
      printf("\033[31mopen serialport error \033[0m !!!\n");
      sleep(1);
      timeout+=10;
      continue;
    }  
    else  
    {  
      printf("%s",ttyname(fd));  
      printf(" succesfully\n");   
      timeout =100;
      break;
    } 
    timeout++; 
    if(timeout==30)printf("\033[31mError, operation time out. \033[0m !!!\n");
  }
  timeout=0;

  set_speed(fd,115200);  
  if (set_Parity(fd,8,1,'N') == FALSE)  
  {  
    printf("Set Parity Error\n");  
    exit (0);  
  }  

  
  if(fcntl(fd,F_SETFL,0) < 0)
  {     
    printf("fcntl failed\n");     
  } 
  pthread_mutex_t rMutex = PTHREAD_MUTEX_INITIALIZER;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  UartOpenFlag = 1;


  ros::NodeHandle n;
  n.param("width_robot", width_robot ,351.4); 
  ros::Publisher  recv_pub = n.advertise<geometry_msgs::Twist>("recv_vel",100);
  ros::Subscriber  vel_sub = n.subscribe("cmd_vel", 1000, &UARTComm::UARTComm_CallBack, this); 
  geometry_msgs::Twist msg;

  //init odom
  odom.PoseMove(0,0,0);  
  odom.encode_PoseMove(0,0,0);

  ros::Rate loop_rate(20);
  while (ros::ok()) 
  {  
    //usleep(1000*9);
    tcflush(fd,TCIFLUSH);
    tcflush(fd,TCOFLUSH);

    //printf("uart_comm running: left_vel=%f,right_vel=%f\n",left_vel,right_vel);
    //SendBuf(sbuf,01,10,10);
    //printf("\nuartvel=%f,rvel=%f\n",left_vel,right_vel);
    //if(velTime < 2)
    SendBuf(sbuf,01,left_vel,right_vel);
    //else
    //SendBuf(sbuf,01,0.0,0.0);

    msg.linear.x  = left_vel/100.0;
    msg.angular.z = right_vel/100.0;
    recv_pub.publish(msg);
    //printf("\n");
    //for(int i=0;i<6;i++)
    //printf("%x ", sbuf[i]);
    //printf("\n");
    bzero(rbuff, sizeof(rbuff));
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
    time.tv_sec  = 5;
    time.tv_usec = 0;

    //data send
    length = sizeof(sbuf);
    //retv = write(fd, sbuf, length);
    retv = write(fd, sbuf, 7);
    if(retv == -1)
    {
      perror("Write data error!\n");
    } 
   
    if((nread = read(fd, rbuff, 22))>0)
    { 
      last_time = ros::Time::now();
      dtime = (last_time - current_time).toSec();
      sumtime += dtime;
      velTime += dtime;
      //printf("recv sumtime = %f.\n",sumtime); 
      current_time = ros::Time::now();
      //for(int i=0;i<nread;i++)printf(" %x",rbuff[i]&0xff);printf("\n"); 
 
      recvflag = RecvBaseBuf(rbuff);
      if(recvflag == 1)
      { 
        firstrecvflag++;
        if(firstrecvflag>2) firstrecvflag=2;
        IMU_send=basedata.IMU/10;
        odom.PoseMove(basedata.Lcode,basedata.Rcode,IMU_send);
        odom.encode_PoseMove(basedata.Lcode,basedata.Rcode,IMU_send);
        printf("IMU=%d,left_vel=%f,right_vel=%fmm/s\n",IMU_send,left_vel,right_vel);
      }
      else
      {
	 odom.PoseMove(basedata.Lcode,basedata.Rcode,IMU_send);
	 printf("read uart data fault count_false=%d!\n",count_false);
      }

    }
    else
    {
       printf("read uart data timeout 1s!\n");
    }

    ros::spinOnce();
    loop_rate.sleep();
    
  }//end while  
  close(fd); 
}
