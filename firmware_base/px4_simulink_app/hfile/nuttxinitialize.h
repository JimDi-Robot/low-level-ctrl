/* Copyright 2014 The MathWorks, Inc. */

#define _GNU_SOURCE
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <include/visibility.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <errno.h>
#include <signal.h>
#include <time.h>


/*Other common includes*/
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <string.h>

#define BASERATE_TIMER_SIGNAL 17
// #define BASERATE_TIMER_SIGNAL SIGRTMIN
#define SIGVALUE_INT 42
#define TERMTASK_STACK_SIZE 2048
#define SCHEDTASK_STACK_SIZE 2048


#define CHECK_STATUS(status, expStatus, fcn) PX4_INFO("Call to %s returned status (%d)", fcn, status);if (status != expStatus) {PX4_INFO("Call to %s returned error status (%d).\n", fcn, status); perror(fcn);}
#define CHECK_STATUS_NOT(status, errStatus, fcn) PX4_INFO("Call to %s returned status (%d)", fcn, status);if (status == errStatus) {PX4_INFO("Call to %s returned error status (%d).", fcn, status); perror(fcn);}


// Main PX4 Simulink App entry
int px4_simulink_app_main(int argc, char *argv[]);
void PX4_Simulink_Task_Callback(void *arg);
void print_sem_value(void); //for debugging



extern bool g_baseRateLife;

typedef struct {
    int sigNo;
    sigset_t sigMask;
    double period;
} baseRateInfo_t; //this struct is used only to create a background task for external mode


void px4_app_usage(const char *reason);
void px4_WaitForExtModeStartPkt(void);
extern void backgroundTask(void* arg);



/* Declare function prototypes here */
void nuttxRTOSInit(double baseRatePeriod, int baseRatePriority, int numSubrates);
void myWaitForThisEvent(int sigNo);
void myAddBlockForThisEvent(int sigNo);


void setTaskPeriod(double periodInSeconds, int sigNo);
void myAddBlockForThisEvent(int sigNo);
void setTaskPeriod(double periodInSeconds, int sigNo);
void schedulerTask(void* arg);



