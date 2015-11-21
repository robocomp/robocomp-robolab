// helpers for writing the main programs for small size

#ifndef __PROG_HELP_H__
#define __PROG_HELP_H__

#include "sstring.h"

void InitHandleStop(bool *_enable_run=NULL,int _grace_time=0);
bool Renice(int nice,bool verbose=true);

const char *GetProjectRoot();
void Usage(const char *program_path);

// get a string for the current date
void GetDateStr(CharString &date);

bool StrToInt(int &val,const char *str,int base=10);
bool UpdateSideAndTeam(int &side,int &team,const char *opt);

/**
SetTimerInterrupt: Used to set timer interrupt.
Arguments: 
  interval : interval of interrupt in micro-seconds
  callback : pointer to function used as callback
**/
bool SetTimerInterrupt(unsigned int interval, void (*callback)(int));

/**
CancelTimerInterrupts: Used to Cancel all existing timer interrupts
**/
void CancelTimerInterrupts();

#endif
