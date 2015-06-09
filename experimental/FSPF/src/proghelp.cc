#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "timer.h"

#include "proghelp.h"


static bool *enable_run;
static int grace_time;


size_t CopyFile(FILE *source,FILE *dest)
{
  char buf[256];
  size_t len, total=0;

  while((len = fread(buf,sizeof(char),256,source)) > 0){
    fwrite(buf,sizeof(char),len,dest);
    total += len;
  }

  return(total);
}

void HandleStop(int i)
// Signal handler for breaks (Ctrl-C)
{
  // clear running flag, and set up alarm in case we've hung
  alarm(grace_time);
  if(enable_run) *enable_run = false;
}

void HandleAlarm(int i)
// Signal handler that forces an exit when the code hangs on Ctrl-C
{
  exit(0);
}

void InitHandleStop(bool *_enable_run,int _grace_time)
{
  enable_run = _enable_run;
  grace_time = (_grace_time > 0)? _grace_time : 1;

  // Connect the stop signals
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleAlarm);
}

bool Renice(int nice,bool verbose)
{
  int old_prio = getpriority(PRIO_PROCESS,0);
  setpriority(PRIO_PROCESS,0,nice);
  int new_prio = getpriority(PRIO_PROCESS,0);

  if(verbose && new_prio!=old_prio){
    printf("renice: %d -> %d\n",old_prio,new_prio);
  }
  return(new_prio == nice);
}

void GetDateStr(CharString &date)
{
  struct tm cur;
  GetDate(cur);
  date.printf("%04d%02d%02d-%02d%02d",
              1900+cur.tm_year, cur.tm_mon+1, cur.tm_mday,
              cur.tm_hour, cur.tm_min);
}

bool SetTimerInterrupt(unsigned int interval, void (*callback)(int))
{
  struct itimerval value, ovalue, pvalue;
  struct sigaction sact;
  
  value.it_interval.tv_sec = 0;
  value.it_interval.tv_usec = interval;
  value.it_value.tv_sec = 0;
  value.it_value.tv_usec = interval;
  
  sigemptyset( &sact.sa_mask );
  sact.sa_flags = 0;
  sact.sa_handler = callback;
  sigaction( SIGALRM, &sact, NULL );
  
  getitimer( ITIMER_REAL, &pvalue );
  setitimer( ITIMER_REAL, &value, &ovalue );
  
  if( ovalue.it_interval.tv_sec != pvalue.it_interval.tv_sec  ||
    ovalue.it_interval.tv_usec != pvalue.it_interval.tv_usec ||
    ovalue.it_value.tv_sec != pvalue.it_value.tv_sec ||
    ovalue.it_value.tv_usec != pvalue.it_value.tv_usec ){
    return false;
  }
  return true;
}

void CancelTimerInterrupts()
{
  struct itimerval value;
  int whichTimer = ITIMER_REAL;
  getitimer( whichTimer, &value );
  value.it_value.tv_sec = 0;
  value.it_value.tv_usec = 0;
  setitimer( whichTimer, &value, NULL );
}
