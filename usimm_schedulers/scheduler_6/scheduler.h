#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

void init_scheduler_vars(); //called from main
void scheduler_stats(); //called from main
void schedule(int); // scheduler function called every cycle

// reset all bank status to 0
void init_bank_status(int);

void get_bank_status(int);

int get_readable_bank(int);

int get_writeable_bank(int);

#endif //__SCHEDULER_H__
