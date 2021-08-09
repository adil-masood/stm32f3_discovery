#ifndef UART_PRINT_H
#define UART_PRINT_H
#define WINDOWS 1
#define LINUX 2
void print_int(UART_HandleTypeDef *huart,void *any,int size);
void print_uint(UART_HandleTypeDef *huart,void *any,int size);
void lineend(UART_HandleTypeDef *huart,int OS_TYPE);
void print_str(UART_HandleTypeDef *huart,char *str);
#endif