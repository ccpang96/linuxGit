#include "apue.h"
#include "pthread.h"
#include "myerr.h"
pthread_t ntid;

void printids(const char *s) {
    pid_t     pid;
    pthread_t tid;

    pid = getpid();
    tid = pthread_self();
    printf("%s pid %lu tid %lu (0x%lu)\n", s, (unsigned long)pid,  
            (unsigned long)tid, (unsigned long)tid );
    //%lu : unsigned long int 
}

void * thr_fn(void *arg) {
    printids("new thread:");
    return((void*)0);
}

int main(void) {
    int err;
    //创建线程的方法：
    //int pthread_create(pthread_t *restrict tidp,
    //                  const pthread_attr_t *restrict attr,
    //                  void *(*start_rtn)(void *), void *restrict arg);
    //                  Returns: 0 if OK,error number on failuer
    //  tidp指向的memory location 被设置成新的thread ID
    //  attr被用于定制不同的thread attributes(线程属性) set to NULL with default attributes
    //  新线程newly thread starts running at the start_rht函数的地址.
    //  如果你想向start_rth，传入多个参数，那么你应该将它们封装在structure里，
    //  并将structure的地址传入arg参数。
    //
    //  新创建的进程可以访问进程的地址空间，并且继承了调用继承的floating-point 环境以及
    //  signal mask.但是pending signals 被cleared了。
    //
    //
    //
    //
    err = pthread_create(&ntid, NULL, thr_fn, NULL);
    
    if (err != 0 )
        err_exit(err, "can't create thread");
    printids("main thread:");
    sleep(1);
    exit(0);
}

//in theis example:
//if new threads runs before the main thread （calling pthread_create之前就运行了）
//then the new thread will be uninitialized content 
//
//我们运行以上代码：
//main thread: pid 11022 tid 140160711866176 (0x140160711866176)
//new thread: pid 11022 tid 140160703543040 (0x140160703543040)
//as we see:ubuntu使用指向其thread data structure作为它的thread ID
//
