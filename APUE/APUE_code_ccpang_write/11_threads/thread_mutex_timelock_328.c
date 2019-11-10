#include "pthread.h"
#include <time.h>

//pthread_mutex_timelock(pthread_mutex_t *restrict mutex,
//                      const struct timespec *restrict tsptr); 
//                      //Retures: 若成功返回0，若失败返回错误编号
//
//tsptr是超时等待的绝对时间，并不是说要等待tsptr秒

//以下给出pthread_mutex_timelock避免某个线程陷入永久阻塞的方法：
int main(void)
{
    int err;
    struct timespec tout;
    struct tm*tmp;
    char buff[64];
    pthread_mutex_t lock =  PTHRAD_MUTEX_INITIALIZER;

    pthread_mutex_lock(&lock);
    printf("mutex is lock\n");
    clock_gettime(CLOCK_REALTIME,&tout);
    tmp = locktime(&tout.tv_sec);
    strftime(buf,sizeof(buf), "&r", tmp);
    printf("current time is %s\n" ,buf);
    tout.tv_sec +=10; //从当前时刻往后10秒钟

    //this could lead to deadlock
    
    err = pthread_mutex_timelock(&lock, &tout);  //使用超时就取消阻塞等待函数，
    //若在到达设定时间时，就返回ETIMEDOUT


    clock_gettime(CLOCK_REALTIME, &tout);
    tmp = localtime(&tout.tv_sec);
    strftime(buf, sizeof(buf), "&r", tmp);
    printf("the time is now %s\n", buf);
    if (err == 0)
        printf("mutex locked again !\n");
    else 
        printf("can't lock mutex again%\n", strerror(err));
    exit(0);
    }
