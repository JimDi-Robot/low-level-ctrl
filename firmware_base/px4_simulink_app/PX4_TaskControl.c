
/* ---------------------------- */
/* RTOS-specific headers        */
/* Note: must be included first */
/* ---------------------------- */
#include "nuttxinitialize.h"

#include "arch/board/board.h"
#include "sys/mount.h"
#include "sys/ioctl.h" 
#include "sys/stat.h"
#include "perf/perf_counter.h"
#include "systemlib/err.h"
#include "parameters/param.h"
#include "errno.h"


/* ---------------------------- */
/* Required Coder Target header */
/* ---------------------------- */
#include "MW_custom_RTOS_header.h"
#include "MW_target_hardware_resources.h"


/* ---------------------------- */
/* Required for Ext-Mode */
/* ---------------------------- */
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "ext_work.h"

/* Base-rate HRT-callback is used to post base-rate semaphore */
extern struct hrt_call BaseRate_HRT;
extern int terminatingmodel;
extern int baserate_sem_copy; //used for checking HRT semaphore water-mark

sem_t PX4_Simulink_Task_Sem; //semaphore for main PX4 Simulink App

/* used to fetch model init and terminate calls */
void (*MW_model_init)(void) = NULL;
void (*MW_model_term)(void) = NULL;

void PX4_Simulink_Task_Callback(void *arg)
{
	
	terminatingmodel = 0; // reset global variable. For more info on the rational behind this, please see:
						  // http://nuttx.org/doku.php?id=wiki:nxinternal:tasks-vs-processes#nuttx_flat-build_behavior
	
    #if defined(EXT_MODE)    
        char *ExtModeInputArray[6];
          ExtModeInputArray[0]= "\0"; 
          ExtModeInputArray[1]= "-port\0";     
          ExtModeInputArray[2]= EXT_MODE_DESCRIPTOR;
          ExtModeInputArray[4]= BAUD_RATE;    
          ExtModeInputArray[3]= "-baud\0";
          ExtModeInputArray[5] = "-w\0";  //This will force the program to wait 
                                          //until it receives a packet from host machine        
          rtExtModeParseArgs(6,ExtModeInputArray,NULL);           
    #endif
        
    /* call model init */
    (*MW_model_init)(); 
	
	#if defined(EXT_MODE)  
		px4_WaitForExtModeStartPkt();
	#endif
	
    sleep(1);
    
    /* Spawn multiple threads based on sample-rates in model */
    nuttxRTOSInit(MW_BASERATE_PERIOD,
                  MW_BASERATE_PRIORITY,
                  MW_NUMBER_SUBRATES);
    
    /* Wait for user to run stop command on app */
    sem_wait(&PX4_Simulink_Task_Sem);
    hrt_cancel(&BaseRate_HRT);
    
    #ifdef PIXHAWK_PSP_HARD_REAL_TIME 
    if (baserate_sem_copy > SEM_WATER_MARK)
    {
        PX4_INFO("Hard real-time constraint violated, shutting down. Updating log file. \n");
        FILE* fp_taskover_run = NULL;
        char msg_watermark[100] = { 0 };
        char msg_timestamp[50] = { 0 };
        struct tm *sTm;
        time_t now = time(NULL);
        sTm = gmtime (&now);
        sprintf(msg_watermark,"Base-rate semaphore exceeded water-mark value %d. Model base-rate = %.3f sec.",SEM_WATER_MARK,(float)MW_BASERATE_PERIOD);
        strftime(msg_timestamp, sizeof(msg_timestamp),"%Y-%m-%d %H:%M:%S", sTm);
        fp_taskover_run = fopen("/fs/microsd/log/task_overrun_log.txt","a+");        
        fprintf(fp_taskover_run,"%s Timestamp=%s \n",msg_watermark,msg_timestamp);
        fclose(fp_taskover_run);  
        
        g_baseRateLife = false; // set status of PX4 Simulink App
    }  
    #endif
    
    PX4_INFO("Received semaphore to end this task \n");
    
    /* Kill model */
    (*MW_model_term)();   
}

/* Print the correct usage. */
void px4_app_usage(const char *reason)
{
  if (reason)
  PX4_INFO("%s\n", reason);
  errx(1,"usage: px4_simulink_app {start|stop|status} [-p <additional params>]\n\n");
}

