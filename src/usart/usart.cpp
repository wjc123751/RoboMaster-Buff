#include "usart.h"

#define err_Max 3
int teamColor=0;
int mode=0;
int direction=0;
int fd;
char recive_buff[1];


unsigned char buff_angle[7] = {0x66,0x00,0x00,0x00,0x00,0x00,0x67};
//x1红点坐标   x2边框左上   x3边框右上  x4边框右下  x5边框左下

void sendbuff(int fd,int16_t x_err,int16_t y_err,int8_t fire)
{
    std::cout<<"yaw_angle"<<x_err<<"ptich_angle:"<<y_err<<std::endl;
    buff_angle[0] = 0x66;
    buff_angle[1] = x_err >> 8;
    buff_angle[2] = x_err;
    buff_angle[3] = y_err >> 8;
    buff_angle[4] = y_err;
    buff_angle[5] = 1;
    // buff_angle[1] = 0x01;
    // buff_angle[2] = 0x02;
    // buff_angle[3] = 0x0a;
    // buff_angle[4] = 0x0b;
    // buff_angle[5] = 0x05;
    buff_angle[6] = 0x67;
    // for(int i=0;i<6;i++)
    // {
    //     if(buff_angle[i]==10)
    //     {
    //         buff_angle[i]=9;
    //     }
    // }
    write(fd,buff_angle,sizeof(buff_angle));
}

int getBit(unsigned char b,int i)
{
    int bit = (int)((b>>i)&0x1);
    return bit;
}
void* reciveThread(void *arg)
{
    while(1)
    {
        if(fd>0)
        {
            read(fd,recive_buff,sizeof(recive_buff));
            teamColor = getBit(recive_buff[0],1);
            //printf("team=%d",teamColor);
            mode = getBit(recive_buff[0],1);
            direction = getBit(recive_buff[0],2)+getBit(recive_buff[0],3);

        }
    }
}
/*
* 打开串口
*/
int open_port(int com_port)
{
    
    /* 使用普通串口 */
    // TODO::在此处添加串口列表
    char* dev[] = { "/dev/ttyACM0", "/dev/ttyUSB0" ,"/dev/pts/1"};
 
    //O_NDELAY 同 O_NONBLOCK。
    fd = open(dev[com_port], O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror("open serial port");
        return(-1);
    }
 
    //恢复串口为阻塞状态 
    //非阻塞：fcntl(fd,F_SETFL,FNDELAY)  
    //阻塞：fcntl(fd,F_SETFL,0) 
    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        perror("fcntl F_SETFL\n");
    }
    /*测试是否为终端设备*/
    if (isatty(STDIN_FILENO) == 0)
    {
        perror("standard input is not a terminal device");
    }
    return fd;
}
 
/*
* 串口设置
*/
int set_uart_config(int fd, int baud_rate, int data_bits, char parity, int stop_bits)
{
    struct termios opt;
    int speed;
    opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON );
    if (tcgetattr(fd, &opt) != 0)
    {
        perror("tcgetattr");
        return -1;
    }
 
    /*设置波特率*/
    switch (baud_rate)
    {
    case 2400:  speed = B2400;  break;
    case 4800:  speed = B4800;  break;
    case 9600:  speed = B9600;  break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    default:    speed = B115200; break;
    }
    cfsetispeed(&opt, speed);
    cfsetospeed(&opt, speed);
    tcsetattr(fd, TCSANOW, &opt);
 
    opt.c_cflag &= ~CSIZE;
 
    /*设置数据位*/
    switch (data_bits)
    {
    case 7: {opt.c_cflag |= CS7; }break;//7个数据位  
    default: {opt.c_cflag |= CS8; }break;//8个数据位 
    }
 
    /*设置奇偶校验位*/
    switch (parity) //N
    {
    case 'n':case 'N':
    {
        opt.c_cflag &= ~PARENB;//校验位使能     
        opt.c_iflag &= ~INPCK; //奇偶校验使能  
    }break;
    case 'o':case 'O':
    {
        opt.c_cflag |= (PARODD | PARENB);//PARODD使用奇校验而不使用偶校验 
        opt.c_iflag |= INPCK;
    }break;
    case 'e':case 'E':
    {
        opt.c_cflag |= PARENB;
        opt.c_cflag &= ~PARODD;
        opt.c_iflag |= INPCK;
    }break;
    case 's':case 'S': /*as no parity*/
    {
        opt.c_cflag &= ~PARENB;
        opt.c_cflag &= ~CSTOPB;
    }break;
    default:
    {
        opt.c_cflag &= ~PARENB;//校验位使能     
        opt.c_iflag &= ~INPCK; //奇偶校验使能          	
    }break;
    }
 
    /*设置停止位*/
    switch (stop_bits)
    {
    case 1: {opt.c_cflag &= ~CSTOPB; } break;
    case 2: {opt.c_cflag |= CSTOPB; }   break;
    default: {opt.c_cflag &= ~CSTOPB; } break;
    }
 
    /*处理未接收字符*/
    tcflush(fd, TCIFLUSH);
 
    /*设置等待时间和最小接收字符*/
    opt.c_cc[VTIME] = 1000;
    opt.c_cc[VMIN] = 0;
 
    /*关闭串口回显*/
     opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | NOFLSH);
 
    /*禁止将输入中的回车翻译为新行 (除非设置了 IGNCR)*/
    opt.c_iflag |= IGNCR;
    opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON );
    /*禁止将所有接收的字符裁减为7比特*/
     opt.c_iflag &= ~ISTRIP;
 
    /*激活新配置*/
    if ((tcsetattr(fd, TCSANOW, &opt)) != 0)
    {
        perror("tcsetattr");
        return -1;
    }
 
    return 0;
}
