/*                                                                  
 * POSIX Real Time Example
 * using a single pthread as RT thread
 */

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

void *thread_func(void *data)
{

    //we can set one or more bits here, each one representing a single CPU
    cpu_set_t cpuset;

    //the CPU we whant to use
    const int core_id = 1;
    const pid_t pid = getpid();

    CPU_ZERO(&cpuset);         //clears the cpuset
    CPU_SET(core_id, &cpuset); //set CPU 2 on cpuset

    /*
        * cpu affinity for the calling thread 
        * first parameter is the pid, 0 = calling thread
        * second parameter is the size of your cpuset
        * third param is the cpuset in which your thread will be
        * placed. Each bit represents a CPU
        */
    //const int set_result = sched_setaffinity(0, sizeof(cpuset), &cpuset);
    // or
    const int set_result = pthread_setaffinity_np(pid, sizeof(cpu_set_t), &cpuset);

    if (set_result != 0)
    {
        printf("Failed to set thread affinity: %d\n", set_result);
    }

    // Verify it was set:
    if (CPU_ISSET(core_id, &cpuset))
    {
        printf("Successfully set thread %d to affinity to CPU %d\n", pid, core_id);
    }
    else
    {
        printf("Failed to set thread %d to affinity to CPU %d\n", pid, core_id);
    }

    /* Do RT specific stuff here */
    while (1)
        ;
    ; //burns the CPU 2

    return NULL;
}

int main(int argc, char *argv[])
{
    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread;
    int ret;

    /* Lock memory */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        printf("mlockall failed: %m\n");
        exit(-2);
    }

    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret)
    {
        printf("init pthread attributes failed\n");
        goto out;
    }

    /* Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (ret)
    {
        printf("pthread setstacksize failed\n");
        goto out;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret)
    {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret)
    {
        printf("pthread setschedparam failed\n");
        goto out;
    }
    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret)
    {
        printf("pthread setinheritsched failed\n");
        goto out;
    }

    /* Create a pthread with specified attributes */
    ret = pthread_create(&thread, &attr, thread_func, NULL);
    if (ret)
    {
        printf("create pthread failed\n");
        goto out;
    }

    /* Join the thread and wait until it is done */
    ret = pthread_join(thread, NULL);
    if (ret)
        printf("join pthread failed: %m\n");

out:
    return ret;
}
