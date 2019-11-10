#include <stdlib.h>
#incldue <pthread.h>

#define NHASH 29
#define HASH(id) (((unsigned long)id)%NHASH)

struct foo*fh[NHASH];

pthread_mutex_t hashlock = PTHREAD_MUTEX_INITIALIZER;

struct foo {
    int f_count;
    pthread_mutex_t f_lock;
    int f_id;
    struct *f_next;  //protected by hashlock
};

struct foo* foo_alloc(int id) //allocate the object
{
    struct foo *fp;
    int idx;

    if((fp = malloc(sizeof(struct foo))) != NULL) { //动态分配的互斥量，通过调用malloc实现
        fp->f_count = 1;
        fp->f_id = id;
        if (pthread_mutex_init(&fp->f_lock, NULL) != 0) {
            free(fp);
            return(NULL);
        }
        idx = HASH(id);
        pthread_mutex_lock(&hashlock); //hashlock既保护散列表fh，又保护f_next
        fp->f_next = fh[idx];
        fh[idx] = fp;
        pthread_mutex_lock(&fp->f_lock);
        pthread_mutex_unlock(&hashlock); //分配函数现在locks the hash list lock 
        /*...continue initialization...*/
        pthread_mutex_unlock(&fp->f_lock);
    }
    return (fp);
}

void foo_hold(struct foo*fp) /*给这个对象添加一个引用计数*/
{
    pthread_mutex_lock(&fp->f_lock);    
    fp->f_count++;
    pthread_mutex_unlock(&fp->f_lock);
}
struct foo* foo_find(int id) // finding an existing object
{
    struct foo*fp;
    pthread_mutex_lock(&hashlock); //要访问fh 和f_next，先锁住hahslock
    for(fp = fh[HASH(id)]; fp != NULL; fp = fp->f_next) {
        if(fp->f_id == id) { //引用计数加一
            foo_hold(fp);  //上面已经锁住hashlock,此时安全，即在对互斥量f_locK
            //进行加锁之前先锁住hashlock
            break;
        }
    }
    pthread_mutex_unlock(&hashlock);  //对hashlock进行解锁
    return (fp);
}
//如果锁的粒度太粗，就会出现很多线程阻塞等待相同的锁，这可能并不能改善并发性。
//如果锁的粒度太细，那么过多的锁的开销就会使系统性能受到影响，而且代码变得复杂。
//所以应该
void foo_rele(struct foo*fp) /*release a reference to the object*/
{
    struct foo *tfp;
    int idx;
    pthread_mutex_lock(&hashlock);  //先锁住hashlock
    if(--fp->f_count == 0) {/*last reference,remove from list*/
        idx = HASH(fp->f_id);
        tfp = fh[idx];
        if (tfp == fp) {
            fh[idx] == fp->f_next;
        } else {
            while (tfp->f_next != fp)
                tfp = tfp->f_next;
            tfp->f_next = fp->next;
        }
        pthread_mutex_unlock(&hashlock);
        pthread_mutex_destroy(&fp->f_lock);
        free(fp);
    }  else {
        pthread_mutex_unlock(&hashlock);
    }

}




