#include <stdio.h>
#include "utlist.h"
#include "utils.h"
#include "params.h"

#include "memory_controller.h"
#include "scheduler.h"
//extern long long int current_core_cycle[0];

#define MAX_THREADS 64

//Time threshold to update top prioritiy
int     UPDATE_THRESHOLE;

//Priority of each thread
int     priority[MAX_NUM_CHANNELS][MAX_THREADS];
//Top priority and its max/min value
int     TOP_PRIORITY, MAX_PRIORITY, MIN_PRIORITY;
//Number of read requests from different threads in last time interval
int     read_in_last_interval[MAX_NUM_CHANNELS][MAX_THREADS];
//Number of read requests to a channel in last time interval
int     total_read_in_last_interval[MAX_NUM_CHANNELS];
/* A data structure to see if a bank is a candidate for precharge. */
int     recent_colacc[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];




void MemoryController::init_scheduler_vars()
{
  HI_WM = DRAM_WQ_SIZE - param->NUM_BANKS;
  LO_WM =  (HI_WM - 16);
	// initialize all scheduler variables here

	// As in scheduler-close.c
	int i, j, k;
	for (i=0; i<MAX_NUM_CHANNELS; i++) {
	  for (j=0; j<MAX_NUM_RANKS; j++) {
	    for (k=0; k<MAX_NUM_BANKS; k++) {
	      recent_colacc[i][j][k] = 0;
	    }
	  }
	}

	// Each thread has top priority at initialization, which is set to NUM_CPUS
	TOP_PRIORITY = NUM_CPUS;
	for(int channel = 0; channel < param->NUM_CHANNELS; channel ++) {
		for(int core =0; core < NUM_CPUS; core ++) {
			priority[channel][core] = TOP_PRIORITY;
			//Initialize read count of each thread
			read_in_last_interval[channel][core] = 0;
		}
		//Initialize read count of each channel
		total_read_in_last_interval[channel]   = 0;	
	}

	// The initial value of update threshold and min/max top-priority
	UPDATE_THRESHOLE = NUM_CPUS << 4;
	MIN_PRIORITY     = (NUM_CPUS >> 1) ? (NUM_CPUS >> 1) : 1;
	MAX_PRIORITY     = NUM_CPUS << 2;
	return;
}

// write queue high water mark; begin draining writes if write queue exceeds this value
//#define HI_WM  (DRAM_WQ_SIZE - param->NUM_BANKS)

// end write queue drain once write queue has this many writes in it
//#define LO_WM  (HI_WM - 16)

// 1 means we are in write-drain mode for that channel
//int drain_writes[MAX_NUM_CHANNELS];

/* Each cycle it is possible to issue a valid command from the read or write queues
   OR
   a valid precharge command to any bank (issue_precharge_command())
   OR 
   a valid precharge_all bank command to a rank (issue_all_bank_precharge_command())
   OR
   a power_down command (issue_powerdown_command()), programmed either for fast or slow exit mode
   OR
   a refresh command (issue_refresh_command())
   OR
   a power_up command (issue_powerup_command())
   OR
   an activate to a specific row (issue_activate_command()).

   If a COL-RD or COL-WR is picked for issue, the scheduler also has the
   option to issue an auto-precharge in this cycle (issue_autoprecharge()).

   Before issuing a command it is important to check if it is issuable. For the RD/WR queue resident commands, checking the "command_issuable" flag is necessary. To check if the other commands (mentioned above) can be issued, it is important to check one of the following functions: is_precharge_allowed, is_all_bank_precharge_allowed, is_powerdown_fast_allowed, is_powerdown_slow_allowed, is_powerup_allowed, is_refresh_allowed, is_autoprecharge_allowed, is_activate_allowed.
   */

void MemoryController::schedule(int channel)
{
	request_t * rd_ptr = NULL;
	request_t * wr_ptr = NULL;


	// if in write drain mode, keep draining writes until the
	// write queue occupancy drops to LO_WM
	if (drain_writes[channel] && (write_queue_length[channel] > LO_WM)) {
	  drain_writes[channel] = 1; // Keep draining.
	}
	else {
	  drain_writes[channel] = 0; // No need to drain.
	}

	// initiate write drain if either the write queue occupancy
	// has reached the HI_WM , OR, if there are no pending read
	// requests
	if(write_queue_length[channel] > HI_WM)
	{
		drain_writes[channel] = 1;
	}
	else {
	  if (!read_queue_length[channel])
	    drain_writes[channel] = 1;
	}


	// If in write drain mode, look through all the write queue
	// elements (already arranged in the order of arrival), and
	// issue the command for the first request that is ready
	if(drain_writes[channel])
	{

		LL_FOREACH(write_queue_head[channel], wr_ptr)
		{
			if(wr_ptr->command_issuable)
			{
				/* Before issuing the command, see if this bank is now a candidate for closure (if it just did a column-rd/wr).
				   If the bank just did an activate or precharge, it is not a candidate for closure. */
				if (wr_ptr->next_command == COL_WRITE_CMD) {
				  recent_colacc[channel][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank] = 1;
				}
				if (wr_ptr->next_command == ACT_CMD) {
				  recent_colacc[channel][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank] = 0;
				}
				if (wr_ptr->next_command == PRE_CMD) {
				  recent_colacc[channel][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank] = 0;
				}
				issue_request_command(wr_ptr);
				break;
			}
		}
		return;
	}

	// Draining Reads
	// look through the queue and find the first request whose
	// command can be issued in this cycle and issue it 
	// Simple FCFS 
	if(!drain_writes[channel])
	{
	
		int max_priority = 0;
		// Find the max priority of all threads
		for(int core = 0; core < NUM_CPUS; core++) {
			max_priority = priority[channel][core] >  max_priority ?  priority[channel][core] : max_priority;
		}
		        	

		//Adjust each thread's priority if no thread has top priority
		if(max_priority < TOP_PRIORITY) {
	        	for(int core = 0; core < NUM_CPUS; core++) {
				if(priority[channel][core] >= 0) priority[channel][core]++; // Filter idle thread out  
			}
			max_priority++; // Must be TOP_PRIORITY as each memory cycle this check is made
		}	

		
		// Actually, this should be reset when a read request in inssert in  queue
		LL_FOREACH(read_queue_head[channel],rd_ptr)
		{
			if(priority[channel][rd_ptr->thread_id] == -1) { // Not a idle thread
				priority[channel][rd_ptr->thread_id] = TOP_PRIORITY;
			}
		}


		//Try to issue a request with top priority
		LL_FOREACH(read_queue_head[channel],rd_ptr)
		{
			if(rd_ptr->command_issuable && (priority[channel][rd_ptr->thread_id] == max_priority))
			{
				if(rd_ptr->next_command == COL_READ_CMD) {
					//Update thread priority and other counters when COL_READ_CMD issued
					priority[channel][rd_ptr->thread_id] --;
					read_in_last_interval[channel][rd_ptr->thread_id] ++;
					total_read_in_last_interval[channel] ++;
				}
				/* Before issuing the command, see if this bank is now a candidate for closure (if it just did a column-rd/wr).
				   If the bank just did an activate or precharge, it is not a candidate for closure. */
				if (rd_ptr->next_command == COL_READ_CMD) {
				  recent_colacc[channel][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank] = 1;
				}
				if (rd_ptr->next_command == ACT_CMD) {
				  recent_colacc[channel][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank] = 0;
				}
				if (rd_ptr->next_command == PRE_CMD) {
				  recent_colacc[channel][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank] = 0;
				}
				issue_request_command(rd_ptr);	
				break;
			}
		}

		//Try to issue a request with other priority
		if(! command_issued_current_cycle[channel]) {
		        LL_FOREACH(read_queue_head[channel],rd_ptr)
			{
				if(rd_ptr->command_issuable) {
					if(rd_ptr->next_command == COL_READ_CMD) {
						// Adjust thread's priority and update read counters when COL_READ_CMD issued
						priority[channel][rd_ptr->thread_id] --;
						read_in_last_interval[channel][rd_ptr->thread_id] ++;
						total_read_in_last_interval[channel] ++;
						// If priority before issue is 0, keep it unchanged
						if( priority[channel][rd_ptr->thread_id] < 0) priority[channel][rd_ptr->thread_id] = 0;
					}
					/* Before issuing the command, see if this bank is now a candidate for closure (if it just did a column-rd/wr).
					   If the bank just did an activate or precharge, it is not a candidate for closure. */
					if (rd_ptr->next_command == COL_READ_CMD) {
					  recent_colacc[channel][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank] = 1;
					}
					if (rd_ptr->next_command == ACT_CMD) {
					  recent_colacc[channel][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank] = 0;
					}
					if (rd_ptr->next_command == PRE_CMD) {
					  recent_colacc[channel][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank] = 0;
					}
					issue_request_command(rd_ptr);
					break;
				}
			}
			
		}
		// Copied from scheduler-close.c and add an extra condition to avoid unusable prefetch after refreshing or power up 
		/* If a command hasn't yet been issued to this channel in this cycle, issue a precharge. */
		if (!command_issued_current_cycle[channel]) {
		  for (int i=0; i<param->NUM_RANKS; i++) {
		    for (int j=0; j<param->NUM_BANKS; j++) {  /* For all banks on the channel.. */
		      if (recent_colacc[channel][i][j]) {  /* See if this bank is a candidate. */
			if (is_precharge_allowed(channel,i,j) && dram_state[channel][i][j].state == ROW_ACTIVE) {  /* See if precharge is doable. */
			  if (issue_precharge_command(channel,i,j)) {
			    recent_colacc[channel][i][j] = 0;
			  }
			}
		      }
		    }
		  }
		}
		// Update TOP_PRIORITY value
		if(total_read_in_last_interval[channel] == UPDATE_THRESHOLE) {
			int max_read = 0;
			int min_read = UPDATE_THRESHOLE; // a very large value
			int core_active = 0;
			for(int core = 0; core < NUM_CPUS; core++) {
				//Find 
				max_read = max_read > read_in_last_interval[channel][core] ? max_read : read_in_last_interval[channel][core];

				if(read_in_last_interval[channel][core]){
				       	core_active++;
					min_read = min_read < read_in_last_interval[channel][core] ? min_read : read_in_last_interval[channel][core];
				} else {
					priority[channel][core] = -1; //not alived in last interval, maybe idle 
				}
				
				read_in_last_interval[channel][core] = 0;
			}
			
			//Reset following paras according active thread counts
			UPDATE_THRESHOLE = core_active << 4;
			MIN_PRIORITY = (core_active >> 1) ? (core_active >> 1) : 1;
			MAX_PRIORITY = (core_active << 2);

			//Adjust the TOP_PRIORITY 
			if(max_read > 2 * min_read) {
				TOP_PRIORITY = TOP_PRIORITY < MAX_PRIORITY  ? TOP_PRIORITY + 1 : MAX_PRIORITY;
			} else if ( max_read < min_read + (min_read >> 2)) { // max_read < 1.25 min_read
				TOP_PRIORITY = TOP_PRIORITY > MIN_PRIORITY  ? TOP_PRIORITY - 1 : MIN_PRIORITY;
				for(int core = 0; core < NUM_CPUS; core++) {
					if( priority[channel][core] > TOP_PRIORITY )
						priority[channel][core] = TOP_PRIORITY;
				}
			} // Otherwhise TOP_PRIORITY unchanged




			//Reset all counters of read requests
			for(int core = 0; core < NUM_CPUS; core++) {
				read_in_last_interval[channel][core] = 0;
			}
			total_read_in_last_interval[channel] = 0;
			
		}	

		//Try to find a COL_WRITE_CMD to issue which needs less time
		if(! command_issued_current_cycle[channel] ) {
			LL_FOREACH(write_queue_head[channel], wr_ptr)
			{
				if(wr_ptr->command_issuable && wr_ptr->next_command == COL_WRITE_CMD)
				{
					recent_colacc[channel][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank] = 1;
					issue_request_command(wr_ptr);
					break;
				}
			}
		}

		//Check the write queue lastly to find a issualbe request
		if(! command_issued_current_cycle[channel]) { // && read_queue_length[channel] <= 10) {
			LL_FOREACH(write_queue_head[channel], wr_ptr)
			{
				if(wr_ptr->command_issuable )
				{
					/* Before issuing the command, see if this bank is now a candidate for closure (if it just did a column-rd/wr).
					   If the bank just did an activate or precharge, it is not a candidate for closure. */
					if (wr_ptr->next_command == COL_WRITE_CMD) {
					  recent_colacc[channel][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank] = 1;
					}
					if (wr_ptr->next_command == ACT_CMD) {
					  recent_colacc[channel][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank] = 0;
					}
					if (wr_ptr->next_command == PRE_CMD) {
					  recent_colacc[channel][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank] = 0;
					}
					issue_request_command(wr_ptr);
					break;
				}
			}
		}
		return;
	}
}

void MemoryController::scheduler_stats()
{
  /* Nothing to print for now. */
}

