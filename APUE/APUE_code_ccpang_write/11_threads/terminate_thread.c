// a signal sent to a thread wil terminate the entire process
// 我们可以在不终止进程的情况下，stopping its flow of control


// 1.thread 可以简单的从start routine中return,return value is the thread's exit code .
// 2.thread can be 取消,by another in the same process
// 3.call pthread_exit

//void pthread_exit(void *rval_ptr)

//rval_ptr 能够被进程中的其它线程访问，通过调用pthread_join()：
//void pthread_join(pthread_t thread, void **rval_ptr);
                    //Returns 0 if OK, error number on failure

//通过调用pthread_join()，自动将我们join的thread置于detached state.
//so that, its resources can be recovered.
//if 我们join 的thread已经是detached state状态，那么pthread_join
//会失败，返回EINVAL



#include "apue.h"
#include <pthread.h>
#include "myerr.h"

void * thr_fn1(void * arg) {
    printf("thread 1 returning\n");
    return((void * )1); //线程1是通过第一种方式终止：即从启动例程中return，
    //return value is the thread's exit code 
}

void * thr_fn2(void *arg) {
    printf("thread 2 exiting\n");  //线程2是通过pthread_exit()方式返回
    pthread_exit((void * )2);
    //通过pthread_join(pthread_t thread, void **rval_ptr); 
    //可以获得pthread_exit()的rval_ptr
}

//if the thread is canceled,rval_ptr指向的内存位置被设置为PTHREAD_CANCELED

int main(void) {

    int err;
    pthread_t tid1, tid2;
    void *tret;

    err = pthread_create(&tid1, NULL, thr_fn1, NULL);
    if ( err != 0 )
        err_exit(err, "can't create thread 1");
    err = pthread_create(&tid2, NULL, thr_fn2, NULL);
    if (err != 0)
        err_exit(err, "can't create thread2");
    err = pthread_join(tid1, &tret);
    if (err != 0)
        err_exit(err, "can't join with thread 1");
    printf("thread 1 exit code %ld\n", (long)tret);
    err = pthread_join(tid2, &tret); //tret中存储的是线程的退出状态
    if (err != 0)
        err_exit(err, "can't join with thread 2");
    printf("thread 2 exit code %ld\n", (long)tret);
}


