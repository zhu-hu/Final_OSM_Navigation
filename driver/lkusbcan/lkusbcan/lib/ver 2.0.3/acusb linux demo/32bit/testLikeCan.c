//build command line: gcc -lCanCmd -lpthread -lrt -g -o testLikeCan testLikeCan.c

// cp libCanCmd.so /usr/lib/
// make
// sudo ./testLikeCan

#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>

#include "ICANCmd.h"
using namespace std;
#define __countof(a)  (sizeof(a)/sizeof(a[0]))

#define  SEND_FRAMES             64    // 每次发送帧数
#define  SEND_TIMES              1000  // 发送次数

#define  USE_CAN_NUM             1      // 1-CAN0自发自收；2-CAN0、CAN1对发

DWORD dwDeviceHandle;
typedef struct {
   int   Run;
   DWORD ch;
}thread_arg_t;
int CanSendcount[2] = { 0, 0 };

void* receive_func(void *param)  //接收线程的处理函数
{
   int reclen = 0;
   thread_arg_t *thread_arg = (thread_arg_t *)param;
   int ind = thread_arg->ch;
   int count = 0;
   int errcount = 0;
   CAN_DataFrame rec[100];
   int i;
   CAN_DataFrame snd;
   CAN_ErrorInformation err;
   printf("receive thread running....%d\n", ind);
   while ( thread_arg->Run ) {
      if ( (reclen = CAN_ChannelReceive(dwDeviceHandle, ind, rec, __countof(rec), 100)) > 0 ) {
         printf("CAN%d Receive: %08X", ind, rec[reclen - 1].uID);
         for ( i = 0; i < rec[reclen - 1].nDataLen; i++ ) {
            printf(" %02X", rec[reclen - 1].arryData[i]);
         }
         printf("\n");
         count += reclen;
         printf("CAN%d rcv count=%d\n", ind, count);
      } else {
         if ( CAN_GetErrorInfo(dwDeviceHandle, ind, &err) == CAN_RESULT_OK ) {
            errcount++;
         }
      }
   }
   printf("CAN%d rcv count=%d err count:%d\n", ind, count, errcount);
}

void* send_func(void *param)
{
   int ch = (long)param;
   int i;
   CAN_DataFrame  send[SEND_FRAMES];
   int times  = SEND_TIMES;  //  26000-12h

   for ( int j = 0; j < __countof(send); j++ ) {
      send[j].uID = ch;         // ID
#if USE_CAN_NUM > 1		   
      send[j].nSendType   = 0;  // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
#else
	  send[j].nSendType   = 2;  // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
#endif	   
      send[j].bRemoteFlag = 0;  // 0-数据帧；1-远程帧
      send[j].bExternFlag = 0;  // 0-标准帧；1-扩展帧
      send[j].nDataLen = 8;     // DLC
      for ( i = 0; i < send[j].nDataLen; i++ ) {
         send[j].arryData[i] = i;
      }
   }
   while ( times ) {
      printf("CAN%d Send %d\r\n", ch, times);
      unsigned long sndCnt = CAN_ChannelSend(dwDeviceHandle, ch, send, __countof(send));
      CanSendcount[ch] += sndCnt;
      if(sndCnt) times--;
   }
   printf("CAN%d Send Count:%d end \r\n", ch, CanSendcount[ch]);
}

int main(void)
{
   CAN_InitConfig config;
   int i = 0;
   thread_arg_t rcv_thread_arg0;
   thread_arg_t rcv_thread_arg1;
   pthread_t rcv_threadid0;
   pthread_t rcv_threadid1;
   pthread_t snd_threadid0;
   pthread_t snd_threadid1;
   int ret;
   char c;
   int times   = 100;
   int sendind = 3;
   time_t tm1, tm2;
   int minute,second;
   if ( (dwDeviceHandle = CAN_DeviceOpen(ACUSB_132B, 0, 0)) == 0 ) {
      printf("open deivce error\n");
      goto ext;
   }
   config.dwAccCode = 0;
   config.dwAccMask = 0xffffffff;
   config.nFilter  = 0;      // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
   config.bMode    = 0;      // 工作模式(0表示正常模式,1表示只听模式)
   config.nBtrType = 1;      // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
   config.dwBtr[0] = 0x00;   // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
   config.dwBtr[1] = 0x14;   // BTR1
   config.dwBtr[2] = 0;
   config.dwBtr[3] = 0;
   if ( CAN_ChannelStart(dwDeviceHandle, 0, &config) != CAN_RESULT_OK ) {
      printf("Start CAN 0 error\n");
      goto ext;
   }
#if USE_CAN_NUM > 1	
   if ( CAN_ChannelStart(dwDeviceHandle, 1, &config) != CAN_RESULT_OK ) {
      printf("Start CAN 1 error\n");
      goto ext;
   }
#endif	
   rcv_thread_arg0.Run = TRUE;
   rcv_thread_arg0.ch  = 0;
   rcv_thread_arg1.Run = TRUE;
   rcv_thread_arg1.ch  = 1;
   ret = pthread_create(&rcv_threadid0, NULL, receive_func, &rcv_thread_arg0);
#if USE_CAN_NUM > 1		
   ret = pthread_create(&rcv_threadid1, NULL, receive_func, &rcv_thread_arg1);
#endif	
   ret = pthread_create(&snd_threadid0, NULL, send_func, (void *)0);
#if USE_CAN_NUM > 1		
   ret = pthread_create(&snd_threadid1, NULL, send_func, (void *)1);
#endif	
   time(&tm1);
   printf("create thread\n");
   pthread_join(snd_threadid0, NULL);
#if USE_CAN_NUM > 1			
   pthread_join(snd_threadid1, NULL);
#endif	
   time(&tm2);
   minute = (tm2 - tm1) / 60;
   second = (tm2 - tm1) % 60;
   printf("CAN0 Sendcount:%d CAN1 Sendcount:%d With time: %d minute,%d second\r\n", CanSendcount[0], CanSendcount[1], minute, second);
   usleep(200000);                  // 等待接收线程读取完所有帧
   // 退出接收线程
   rcv_thread_arg0.Run = FALSE;
   pthread_join(rcv_threadid0, NULL);
   rcv_thread_arg1.Run = FALSE;
#if USE_CAN_NUM > 1		
   pthread_join(rcv_threadid1, NULL);
#endif	

ext:
   CAN_ChannelStop(dwDeviceHandle,0);
#if USE_CAN_NUM > 1		
   CAN_ChannelStop(dwDeviceHandle,1);
#endif	
   CAN_DeviceClose(dwDeviceHandle);
   printf("CAN_DeviceClose\r\n");
   printf("Press Enter key to exit the program\r\n");
   scanf("%c", &c);
}
