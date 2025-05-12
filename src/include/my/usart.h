#ifndef __USART__H
#define __USART__H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iconv.h>
#include <string>
#include <iostream>

extern int errxAdd;
extern int teamColor;
extern int mode;
extern int direction;
extern int fd;
void sendMessage(int fd,int x,int y);
void sendangle(int fd,int16_t x1,int16_t y1,int16_t x2,int16_t y2,int16_t x3,int16_t y3,int16_t x4,int16_t y4,int16_t x5,int16_t y5);
int open_port(int com_port);
void* reciveThread(void *arg);
int set_uart_config(int fd, int baud_rate, int data_bits, char parity, int stop_bits);
void sendbuff(int fd,int16_t x_err,int16_t y_err,int8_t fire);


#endif // !__USART__H