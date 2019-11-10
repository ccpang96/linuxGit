//条件变量的类型：pthread_cond_t
//初始化静态分配的条件变量： 赋值PTHREAD_COND_INITIALIZER
//初始化动态分配的条件变量：pthread_cond_init函数对其进行初始化
//

//int pthread_cond_init(pthread_cond_t *restrict cond,
//                      const pthread_condattr_r *restrict attr); //动态初始化
//
//int pthread_cond_destroy(pthread_cond_t *cond); 
//                      Both return 0 : if OK,error number on failure


//int pthread_cond_wait(pthread_cond_t *restrict cond,
//                      pthread_mutex_t *restrict mutex);

//int pthread_cond_timedwait(pthread_cond_t *restrict cond,
//                            pthread_mutex_t *restrict mutex,
                            const struct timespec *restrict tsptr);
                //Both return 0: if OK, error number on failure


//	-参数：
//		- `cond`：要等待的条件变量的地址
//		- `mutex`：与条件变量配套的互斥量的地址
//		- `tsptr`：指向一个`timespec`的指针，该`timepsec`指定了一个绝对时间（并不是相对时间，比如10秒）
//	- 返回值：
//		- 成功：返回0
//		

/*
 * void maketimeout(struct timespec *tsp, long minutes) {
 *
 *  struct timeval now;
 *
 *gettimeofday(&now, NULL);
 tsp->tv_sec = now.tv_sec;  //获取当前的秒数
 tsp->tv_nsec = now.tv_usec *1000;  //use to nsec
 tsp->tv_sec += minutes * 60; //add the offset to get timeout value
 * }


 //int pthread_cond_signal(pthread_cond_t *cond);
 //int pthread_cond_broadcast(pthread_cond_t *cond);
 //在调用pthread_cond_signal或者pthread_cond_broadcast时，我们说这是在给线程或者条件发送信号
 //一定要在改变条件状态以后再给线程发送信号
 */ 

//当前线程执行pthread_cond_wait时，处于临界区访问共享资源，存在一个mutex与该临界区相关联
//这是理解pthread_cond_wait带有mutex参数的关键。
#include <pthread.h>
struct msg {
    struct msg *m_next;
};

struct msg*workq;
pthread_cond_t qready = PTHREAD_COND_INITIALIZER;   //条件变量
pthread_mutex_t qlock = PTHREAD_MUTEX_INITIALIZER;  //互斥量

void process_msg(void) {
    struct msg *mp;
    for (; ; ) {
        pthread_mutex_lock(&qlock);  //对互斥量加锁
        while (workq == NULL)
            pthread_cond_wait(&qready, &qlock); //调用者线程阻塞，等待被别的线程唤醒。
        mp = workq;
        workq = mp->m_next;
        pthrad_mutex_unlock(&qlock);
        /*now process the message mp*/
    }
}

void enqueue_msg(struct msg*mp) {
    pthread_mutex_lock(&qlock);
    mp->m_next = workq;
    workq = mp;
    pthread_mutex_unlock(&qlock);
    pthread_cond_signal(&qready);
}



