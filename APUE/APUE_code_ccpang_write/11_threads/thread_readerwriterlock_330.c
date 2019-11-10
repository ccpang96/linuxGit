//Reader-Writer Locks
//读写锁有三种状态：
//1.读模式下加锁（locked in read mode）
//2.写模式下加锁 (locked in write mode)
//3.不加锁状态（unlocked）

//在同一时间只有一个thread，可以拥有write mode下的读写锁，但
//是可以有多个线程同时拥有read mode下的读写锁。
//
//Reader-writer locks 非常适合用于对数据结构读的次数大于写的次数的情况。
//Reader-writer locks也叫共享互斥锁（shared-exclusive lock）:
//当读写锁是以读模式锁住的，就可以说成是以共享模式锁住的；当它是写模式锁住的时候，
//就可以说成是互斥模式锁住的




//与互斥量相比，读写锁在使用之前必须初始化，在释放它们的底层内存之前必须销毁。

//int pthread_rwlock_init(pthread_rwlock_t * restrict rwlock,
//                        const pthread_rwlockattr_t *restrict attr);
//
//int pthread_rwlock_destroy(pthread_rwlock_t *rwlock);
//                          Both return 0: if OK,error number on failure
//


//默认属性：null传递给attr


//int pthread_rwlock_rdlock(pthread_rwlock_t *rwlock);  // 读模式下锁定读写锁
//int pthread_rwlock_wrlock(pthread_rwlock_t *rwlock);  // 写模式下锁定读写锁
//int pthread_rwlock_unlock(pthread_rwlock_t *rwlock);  //解锁
//                  所有函数的返回值，if OK ，return 0; else return error number





//读写锁原语的条件版本：（Reader-writer locking primitives）
//int pthread_rwlock_tryrdlock(pthread_rwlock_t *rwlock);
//int pthread_rwlock_trywrlock(pthread_rwlock_t *rwlock);
//          可以获取读写锁时，返回0；否则，返回错误BUSY
//

//A queue of job request is protected by a single reader-writer lock;
//一个工作队列由于单个读写锁进行保护的例子如下所示：
//multiple worker threads obtain jobs assigned to them by a single master thread;
//(多个工作线程由一个主线程处获得工作)

#include <stdlib.h>
#include <pthread.h>
// 主线程把新的作业放到一个工作队列qp中，由多个工作线程组成的线程池从队列中移出作业；
// 主线程不允许每个线程任意处理从队列顶端取出的作业，而是由主线程控制作业的分配；
// 主线程会在待处理作业结构（job）中放置处理该作业的线程ID,每个工作线程只能取出标有自己
// 线程ID的作业。
//无论是插入新的job，还是取走job，都需要对队列进行写模式下锁定读写锁；
//若某个线程想在队列中查找具有和自己id相同的job，则只需要对队列进行读模式写锁定就可以了。
struct job {
    struct job *j_next; //指向后一个作业
    struct job *j_prev; //指向前一个作业
    //处理该作业的线程ID
    pthread_t j_id; /* tells which threads handles this job*/ 
};

//主线程将新的作业放入这个队列中
struct queue {
    struct job *q_head;
    struct job *q_tail;
    pthread_rwlock_t q_lock; //定义了一个读写锁

};
/*Initialize a queue */
//初始化工作队列
int queue_init(struct queue *qp) {

    int err;

    qp->q_head = NULL;
    qp->q_tail = NULL;
    err = pthread_rwlock_init(&qp->lock, NULL); //初始化读写锁，以默认属性
    if (err != 0)
        return(err);
    /*continue initialization....*/
    return 0;
}


/*insert a job at the head of queue*/
//在工作队列的首部插入一个job
void job_insert(struct queue *qp, struct job *jp) {
    pthread_rwlock_wrlock(*qp->q_lock); //此时先将工作队列锁住 write mode 阻塞其他妄图写的线程
    jp->j_next = qp->q_head;  //将工作队列的头指针，指向当前job的下一个job
    jp->j_prev = NULL;
    if (qp->q_head != NULL) 
        qp->q_head->j_prev = jp; //如果插入的作业有效，将先插入的作业的j_prev指向前一个job
    else
        qp->q_tail = jp;  /*list was empty插入的这个工作队列是NULL的，还讲当前的q_tail指向原来的jp*/
    qp->q_head = jp;    
    pthread_rwlock_unlock(&qp->q_lock);
}

/*Append a job on the tail of queue 看来这还是一个双向队列呢*/
//在队列的尾端，添加一个job
void join_append(struct queue* qp, struct job *jp)
{
    pthread_rwlock_wrlock(&qp->q_lock); //在向队列中增加作业的时候，采用write mode 对读写锁进行加锁
    jp->j_next  = NULL;
    jp->j_prev = qp->q_tail; 
    if (qp->q_tail != NULL)
        qp->q_tail->j_next = jp;
    else 
        qp->q_head = jp; 
    pthread_rwlock_unlock(&qp->q_lock);
}

/*remove the given job from a queue*/
//对应线程池从工作队列中取走job
void job_remove(struct queue*qp, struct job *jp)
{
    pthread_rwlock_wrlock(&qp->q_lock);
    if (jp == qp->q_head) {  //jp就是头部指向的这个
        qp->q_head = jp->j_next; //队列的头指针指向队列顶端job的下一个job。
        if (qp->q_tail == jp) //如果队列的尾指针，也指向jp,那么说明队列中只有这一个job
            qp->q_tail = NULL; //就让q_tail指向NULL，因为后面没有job可指向了啊，不怪我
        else //queue中还有其他job 
            jp->j_next->j_prev = jp->j_prev;
    } else if (jp = qp->q_tail) { //这说明要取出的job是位于队列的最尾部的
        qp->q_tail = jp->prev;
        jp->j_prev->j_next = jp->j_next; //jp前一个工作的下一个指针指向jp的next指针。
        
    } else {  //要删除的jp是位于queue中间的
        jp->j_prev->j_next = jp->next; //jp前一个job的下一个指针，指向jp的下一个指针
        jp->j_next->prev = jp_perv;
    }
    pthread_rwlock_unlock(&qp->q_lock); //释放锁
}

/*find a job for the given thread ID*/
struct job *job_find(struct queue *qp, pthread_t id) {
    struct job*jp;
    if (pthread_rwlock_rdlock(&qp->q_lcok) != 0) //对queue加读模式锁
        return (NULL);
    for (jp = qp->q_head; jp != NULL; jp = jp->next)
        if (pthread_equal(jp->j_id, id)) //判断两个线程id相等
           break;
    pthread_rwlock_unlock(&qp->q_lock);
    return(jp);  
}


























