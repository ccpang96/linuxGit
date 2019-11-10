#include "apue.h"
#include "myerr.h"
#include <stdlib.h>
#include <pthread.h>
//当一个以上的线程访问动态分配的对象时，我们可以在对象中嵌入引用技术，确保
//在所有使用对象的线程完成数据访问之前，该对象的内存空间不会被释放。

struct foo {
    int     f_count;
    pthread_mutex_t f_lock;
    int f_id;
};

struct foo * foo_alloc(int id) /*allocate the object*/
{
    struct foo* fp;
    if((fp = malloc(sizeof(struct foo))) != NULL) {
        fp->f_count = 1;
        fp->f_id = id;
        if (pthread_mutex_init(&fp->f_lock, NULL) != 0) { //默认属性初始化，
            //互斥量，只需要将arr设置为NULL
            free(fp);
            return(NULL);
        }    
    }
    return fp;
}
//在使用该对象之前，需要调用foo_hold对这个
void foo_hold(struct foo*fp) /*add a reference to the object*/
{
    pthread_mutex_lock(&fp->f_lock);  //先对互斥量进行加锁，如果有其他线程想要对f_count进行
    //操作，那么它必然都要进行这一步操作，所以任何试图对互斥量加锁的线程都被阻塞。
    fp->f_count++;
    phread_mutex_unlock(&fp->f_lock);
}

void foo_rele(struct foo*fp) /* release a reference to the object*/
{
    pthread_mutex_lock(&fp->f_lock); //当对象使用完毕后，我们要释放这个引用计数
    if (--fp->f_count == 0) {  //先检查引用计数的状态
        if(--fp->f_count == 0) {/*last reference*/
            pthread_mutex_unlock(&fp->f_lock);  //释放互斥锁
            pthread_mutex_destroy(&fp->f_lock); //在释放内存之前，调用pthread_mutex_destory
            free(fp);
        } else {
            pthread_mutex_unlock(&fp->f_lock); //如果引用计数仍不为0，说明仍有其他线程
            //在使用该内存，释放互斥锁就行了。
        }
    
    }

}


int main() {

}
