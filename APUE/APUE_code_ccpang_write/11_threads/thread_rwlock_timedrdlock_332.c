//带有超时的读写锁加锁函数，使得程序在获取读写锁时避免陷入永久阻塞状态
//int pthread_rwlock_timedrdlock(pthread_rwlock_t *restrict rwlock,
//                              const struct timespec *restrict tsptr);  //读模式下的超时加锁程序
//
//int pthrad_rwlock_timedwrlock(pthread_rwlokc_t *restrict rwlock,
//                              const sturct timespec *restrict tsptr);  //写模式下的超时加锁程序

//超时时间是绝对时间，而不是相对时间； 如果超时到期，仍任不能获取读写锁，则应该返回ETIMEDOUT;
//
