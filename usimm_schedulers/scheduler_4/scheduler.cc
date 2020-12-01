#include <stdio.h>
#include "utlist.h"
#include "utils.h"

#include "memory_controller.h"
#include "params.h"

// CPU CYCLE
//extern long long int current_core_cycle[0];

// A data structure to see if a bank is a candidate for precharge
// It is set to 1 when the read and write command is issued,
// and is reset to 0 when the active and precharge command is issued.
int recent_colacc[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];

// A data structure to store the number of refresh commands issued
// in the "8*tREFI" refresh window.
int refreshes[MAX_NUM_CHANNELS][MAX_NUM_RANKS];

// Constant register that stores the delayed write drain cycle.
// When the read queue is empty, the write drain is delayed by the amount of DD_MAX
int DD_MAX = 2;
// A counter that is used for delayed write drain. It is increased at every memory cycle. 
// When dd_counter reaches DD_MAX, write is issued.
int dd_counter[MAX_NUM_CHANNELS];

// A data structure to store the difference of the number of the read command per banks 
// and the active for read command per banks
long long int read_history_counter[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];

// A data structure to store the difference of the number of the write command per banks 
// and the active for write command per banks
long long int write_history_counter[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];

// this counter monitors the number of the request density
long long int request_density_counter[MAX_NUM_CHANNELS];

// if the request_density_counter > REQUEST_DENSITY_THRESHOLD,
// delayed drain valid is set to 0, in order to disable delayed drain
int delayed_drain_valid[MAX_NUM_CHANNELS];

// ysmoon
long long int time_over_high_watermark[MAX_NUM_CHANNELS];
long long int time_under_low_watermark[MAX_NUM_CHANNELS];
MemoryController::optype_t request_type_last[MAX_NUM_CHANNELS]; 
long long int write_to_write[MAX_NUM_CHANNELS];
long long int write_to_read[MAX_NUM_CHANNELS];
long long int read_to_write[MAX_NUM_CHANNELS];
long long int read_to_read[MAX_NUM_CHANNELS];

// Constant register to store the attenuation period of the read_count and active_count_r
// read_count and active_count_r is divided by half for every COUNTER_ATTENUATION_PERIOD
int COUNTER_ATTENUATION_PERIOD = 1000000;

// Constant register to store the hit rate update period per bank
// delay_close_valid and delay_close_counter is updated for every HIT_RATE_UPDATE_PERIOD
// If the read_history_counter exceeds 0, delay_close_valid is set to 1 and 
// delay_close_counter is set to CLOSE_DELAYING_TIME
int HIT_RATE_UPDATE_PERIOD = 10000;

// Constant register to store the value which is used for delayed precharge
int CLOSE_DELAYING_TIME;

// Delayed_drain_valid update period
// delayed_drain_valid counter is updated for every DENSITY_UPDATE_PERIOD
// by referencing request_density_counter
int DENSITY_UPDATE_PERIOD = 1000;

// pre-defined value which is used for determining whether to apply a delayed drain or not
int REQUEST_DENSITY_THRESHOLD = 20;

// A data structure to store validity of the delayed close policy
// It is periodically updated to 1 or 0. 
// When it is set to 1, a corresponding bank becomes a candidate of delayed close
// When it is set to 0, a corresponding bank is closed immediately when it is possible.
// _r is referenced when read, _w is referenced when write
int delayed_close_valid_r[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
int delayed_close_valid_w[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
// A counter to count close timing at the delayed close 
// It is reduced when the precharge command is issuable to the corresponding bank.
// When the delayed_close_valid equals 1, delayed_close_counter should be 0 to issue a precharge command.
// _r is referenced when read, _w is referenced when write
int delayed_close_counter_r[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
int delayed_close_counter_w[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];

// high watermark and low watermark for write queue
int HI_WM, LO_WM;

void MemoryController::init_scheduler_vars()
{
	//HI_WM = WQ_CAPACITY - 2;
	HI_WM = DRAM_WQ_SIZE - 2;
	//LO_WM = WQ_CAPACITY - 28;
	LO_WM = DRAM_WQ_SIZE - 28;
	CLOSE_DELAYING_TIME = param->T_RTP/4 + 1;

	// initialize all scheduler variables here
	int i, j, k;
	for (i=0; i<MAX_NUM_CHANNELS; i++) {
	  for (j=0; j<MAX_NUM_RANKS; j++) {
	    for (k=0; k<MAX_NUM_BANKS; k++) {
	      recent_colacc[i][j][k] = 0;
	    }
	  }
	}

	for (i=0;i<MAX_NUM_CHANNELS;i++) {
	  for (j=0;j<MAX_NUM_RANKS;j++) {
	    refreshes[i][j] = 0;
	  }
	  dd_counter[i] = 0;
	}

	for (i=0; i<MAX_NUM_CHANNELS; i++) {
	  delayed_drain_valid[i] = 0 ;
	  request_density_counter[i] = 0;
	  for (j=0; j<MAX_NUM_RANKS; j++) {
	    for (k=0; k<MAX_NUM_BANKS; k++) {
			read_history_counter[i][j][k] = 0;
			write_history_counter[i][j][k] = 0;
			delayed_close_valid_r[i][j][k] = 0;
			delayed_close_counter_r[i][j][k] = CLOSE_DELAYING_TIME;
			delayed_close_valid_w[i][j][k] = 0;
			delayed_close_counter_w[i][j][k] = CLOSE_DELAYING_TIME;
	    }
	  }
	}

	// ysmoon
	for(i=0; i<MAX_NUM_CHANNELS; i++) {
		time_over_high_watermark[i] = 0;
		time_under_low_watermark[i] = 0;
		request_type_last[i] = READ;
		write_to_write[i] = 0;
		write_to_read[i] = 0;
		read_to_write[i] = 0;
		read_to_read[i] = 0;
	}

	return;
}

// 1 means we are in write-drain mode for that channel
// 0 means we are in read_drain mode for that channel
//int drain_writes[MAX_NUM_CHANNELS];

// the function to check the row hit & issuable 
// write request in current cycle in the write queue
int MemoryController::is_row_hit_in_wq(int channel)
{
	request_t * wr_ptr = NULL;

	LL_FOREACH(write_queue_head[channel], wr_ptr)
	{
		if( (wr_ptr->command_issuable) && (wr_ptr->next_command==COL_WRITE_CMD) )
		{
			return 1;
		}
	}

	return 0;
}

// the function to check the row hit & issuable 
// read request in current cycle in the read queue
int MemoryController::is_row_hit_in_rq(int channel)
{
	request_t * rd_ptr = NULL;

	LL_FOREACH(read_queue_head[channel], rd_ptr)
	{
		if( (rd_ptr->command_issuable) && (rd_ptr->next_command==COL_READ_CMD) )
		{
			return 1;
		}
	}

	return 0;
}

// the function to check if there is a row hit write requests in the write queue to a specific bank
int MemoryController::check_row_hit_in_wq(int channel, int rank, int bank) {
	request_t *wr_ptr = NULL;

	LL_FOREACH(write_queue_head[channel], wr_ptr)
	{
		if( (wr_ptr->next_command==COL_WRITE_CMD) && (rank==wr_ptr->dram_addr.rank) && (bank==wr_ptr->dram_addr.bank) && (dram_state[channel][rank][bank].active_row==wr_ptr->dram_addr.row) )
		{
			return 1;
		}
	}

	return 0;

}

// the function to check if there is a row hit read requests in the read queue to a specific bank
int MemoryController::check_row_hit_in_rq(int channel, int rank, int bank) {
	request_t *rd_ptr = NULL;

	LL_FOREACH(read_queue_head[channel], rd_ptr)
	{
		if( (rd_ptr->next_command==COL_READ_CMD) && (rank==rd_ptr->dram_addr.rank) && (bank==rd_ptr->dram_addr.bank) && (dram_state[channel][rank][bank].active_row==rd_ptr->dram_addr.row) )
		{
			return 1;
		}
	}

	return 0;

}


// main function of the scheduler.c
void MemoryController::schedule(int channel)
{
	request_t * rd_ptr = NULL;
	request_t * wr_ptr = NULL;
	int i, j, k;

	// Reset the number of refresh commands for every 8*tREFI
    if ((current_core_cycle[0] % (8*param->T_REFI)) == 0) {
      for (i=0;i<param->NUM_RANKS;i++) {
        refreshes[channel][i] = 0;
      }
    }

	// Update the delayed_close_valid and delayed_close_counter 
	// for every HIT_RATE_UPDATE_PERIOD
	// if the number of the read_history_counter/write_history_counter exceeds zero,
	// the corresponding delayed_close_valid is set to 1,
	// and the corresponding delayed_close_counter is set to CLOSE_DELAYING_TIME.
	// Otherwise, both delayed_close_valid and delayed_close_counter is set to 0.
	if( (current_core_cycle[0] % HIT_RATE_UPDATE_PERIOD) == 0 ) {
	    for (j=0; j<param->NUM_RANKS; j++) {
	      for (k=0; k<param->NUM_BANKS; k++) {
			  if( read_history_counter[channel][j][k] > 0 ) {
			  	delayed_close_valid_r[channel][j][k] = 1;
				delayed_close_counter_r[channel][j][k] = CLOSE_DELAYING_TIME;
			  }
			  else {
				  delayed_close_valid_r[channel][j][k] = 0;
				  delayed_close_counter_r[channel][j][k] = 0;
			  }
			  if( write_history_counter[channel][j][k] > 0 ) {
				delayed_close_valid_w[channel][j][k] = 1;
				delayed_close_counter_w[channel][j][k] = CLOSE_DELAYING_TIME;
			  }
			  else {
			    delayed_close_valid_w[channel][j][k] = 0;
				delayed_close_counter_w[channel][j][k] = 0;
			  }
		  }
		}

	}

	// Periodic request density update
	// For every DENSITY_UPDATE_PERIOD, if the request_density_counter
	// exceeds REQUEST_DENSITY_THRESHOLD, delayed_drain_valid is set to 0
	// otherwise, delayed_drain_valid is set to 1
	// request_density_counter is reset to 0 for every DENSITY_UPDATE_PERIOD
	if( (current_core_cycle[0] % DENSITY_UPDATE_PERIOD == 0) ) {
		if(request_density_counter[channel] > REQUEST_DENSITY_THRESHOLD) {
			delayed_drain_valid[channel] = 0;
		}
		else {
			delayed_drain_valid[channel] = 1;
		}
		request_density_counter[channel] = 0;
	}

	// Attenuate the read_history_counter
	// for every COUNTER_ATTENUATION_PERIOD by the ratio of 50%
	// In here, COUNTER_ATTENUATION_PERIOD is 250,000 for memory cycle
	// so that the maximum number of the read_count and active_count_r is 500,000, mathmatically.
	// We need 19 bit for read_history_counter, 19 bit for write_history_counter
	if( (current_core_cycle[0] % COUNTER_ATTENUATION_PERIOD) == 0 ) {
	    for (j=0; j<param->NUM_RANKS; j++) {
	      for (k=0; k<param->NUM_BANKS; k++) {
			read_history_counter[channel][j][k] = read_history_counter[channel][j][k]/2;
			write_history_counter[channel][j][k] = write_history_counter[channel][j][k]/2;
		  }
		}
	}

	// ysmoon
	if(write_queue_length[channel] > HI_WM) {
		time_over_high_watermark[channel]++;
	}
	if(write_queue_length[channel] < LO_WM) {
		time_under_low_watermark[channel]++;
	}



	// Row Locality based Drain Policy
	
	// Check if the length of the write queue is lower than LO_WM. 
	// In here, write is drained when the read queue is empty.
	// Otherwise, read is drained.
	if(write_queue_length[channel] < LO_WM) {
		drain_writes[channel] = 0;
	}

	// Check if the length of the write queue exceeds HI_WM
	// If so, write is drained
	if(write_queue_length[channel] > HI_WM)
	{
		drain_writes[channel] = 1;
	}

	// Check if the length of the write queue is lower than HI_WM
	else {
	  // If the length of the write queue equals zero, drain writes. 
	  if( write_queue_length[channel]==0 ) {
		  drain_writes[channel] = 0;
	  }

	  // if the length of the write queue is not zero
	  else {
		  // if read queue == 0 
		  if (!read_queue_length[channel]) {
			// if the delayed drain is turned on
			if( delayed_drain_valid[channel] ) {
			  // delayed write drain.
			  // write is delayed for the amount of the DD_MAX cycle.
			  // If the read request is arrived during waiting time, read is issued.
			  // Otherwise, the write request is issued.
			  if(dd_counter[channel] < DD_MAX) {
				  drain_writes[channel] = 0;
				  dd_counter[channel]++;
			  }
			  else {
				  drain_writes[channel] = 1;

				  // When the length of the write queue is lower than LO_WM, 
				  // delayed write drain is performed for every DD_MAX. 
				  // When the length of the write queue is higher than LO_WM, 
				  // delayed write drain is performed consecutively.
				  if( write_queue_length[channel] < LO_WM ) {
				  	dd_counter[channel] = 0;
				  }
			  }
			}
			// if the delayed drain is turned off, drain read immediately
			else {
				drain_writes[channel] = 1;
			}
		  }
		  // if length of the read queue > 0
		  else {
			  // reset the dd_counter
			  dd_counter[channel] = 0;
			  // when write drain mode, 
			  // keep write drain if there is a row hit write request in the write queue. 
			  // If there is no row hit write request in the write queue while there is the row hit in the read queue, switch to read drain.
			  // If there is no row hit request both in the write queue and the read queue, keep write draining in order to avoid write-to-read turn-around time.
			  if(drain_writes[channel]) {
				  if( is_row_hit_in_wq(channel) ) {
				  	drain_writes[channel] = 1;
				  }
				  else if( is_row_hit_in_rq(channel) ) {
					  drain_writes[channel] = 0;
				  }
				  else {
					  drain_writes[channel] = 1;
				  }
			  }
			  // at read drain mode,
			  // If there is a row hit read request in the read queue, keep draining read.
			  // If there is no row hit in the read queue and there is a row hit in the write queue, switch to write drain.
			  // If there is no row hit both in the read queue and the write queue, keep draining read in order to avoid read-to-write turn-around time.
			  else {
				  if( is_row_hit_in_rq(channel) ) {
					drain_writes[channel] = 0;
				  }
				  else if( is_row_hit_in_wq(channel) ) {
					  drain_writes[channel] = 1;
				  }
				  else {
					  drain_writes[channel] = 0;
				  }
			  }
		  }
	  }
	}

	// If in write drain mode, look through all the write queue
	// elements (already arranged in the order of arrival), and
	// issue the command for the first ready(row-hit) request, 
	// and then first come request.
	if(drain_writes[channel])
	{
		// FRFS
		// search from the head of the write queue
		LL_FOREACH(write_queue_head[channel], wr_ptr)
		{

			// if the write command is row hit and issuable at current cycle, issue that request
			if( (wr_ptr->command_issuable) && (wr_ptr->next_command==COL_WRITE_CMD) ) {
				int rank = wr_ptr->dram_addr.rank;
				int bank = wr_ptr->dram_addr.bank;

				// set recent_colacc to 1, which is referenced at precharge time
				recent_colacc[channel][rank][bank] = 1;

				// increase the write_history_counter when the write is issued
				write_history_counter[channel][rank][bank]++;

				// increase the request density counter
				request_density_counter[channel]++;

				// set the delayed_close_counter_w to CLOSE_DELAYING_TIME
				// for close a certain row, this value should be 0
				// the value of the delayed_close_counter_w is decreased by 1
				delayed_close_counter_w[channel][rank][bank] = CLOSE_DELAYING_TIME;

				// ysmoon
				// optional print out registers
				if(request_type_last[channel] == READ) {
					read_to_write[channel]++;
				}
				else if(request_type_last[channel] == WRITE) {
					write_to_write[channel]++;
				}
				request_type_last[channel] = WRITE;

				// issue write
				issue_request_command(wr_ptr);		
				break;
			}
		}

		// FCFS
		// if FR is not exist in queue
		if(!command_issued_current_cycle[channel])
		{
			// from the head of the write queue
			LL_FOREACH(write_queue_head[channel], wr_ptr)
			{
				if(wr_ptr->command_issuable)
				{
					int rank = wr_ptr->dram_addr.rank;
					int bank = wr_ptr->dram_addr.bank;

					// if the active command is needed
					if (wr_ptr->next_command == ACT_CMD) {

					  // reset the recent_colacc to 0
					  recent_colacc[channel][rank][bank] = 0;

					  // decrease the write_history_counter by 1 
					  // write_history_counter represents "# write - # active for write"
					  write_history_counter[channel][rank][bank]--;

					  // issue active command
					  issue_request_command(wr_ptr);
					  break;
					}

					// if the precharge is needed
					if (wr_ptr->next_command == PRE_CMD) {
					  // if the row hit write request is not exist in write queue, issue precharge
					  // otherwise, does not issue precharge
					  if( check_row_hit_in_wq(channel,rank,bank)==0 ) {
						// check timing parameter
						if(is_precharge_allowed(channel,rank,bank)) {
							// if the delayed_close is turned off for a certain bank
							if(delayed_close_valid_w[channel][rank][bank]==0) {
								// issue precharge command
								if( issue_precharge_command(channel,rank,bank) ) {
									// reset the recent_colacc to 0
									recent_colacc[channel][rank][bank] = 0;
									break;
								}
							}
							// if the delayed_close is turned on for a certain bank, and the delayed_close_counter equals 0,
							// issue the precharge command
							else if( (delayed_close_valid_w[channel][rank][bank]==1) && (delayed_close_counter_w[channel][rank][bank]==0) ) {
								if( issue_precharge_command(channel,rank,bank) ) {
									// reset the recent_colacc
									recent_colacc[channel][rank][bank] = 0;

									// reset the delayed_close_counter
									delayed_close_counter_w[channel][rank][bank] = CLOSE_DELAYING_TIME;
									break;
								}
							}
						}
					  }
					}
				}
			}
		}
	}

    // do a refresh if there aren't many reads waiting
    if( (read_queue_length[channel]==0) && (command_issued_current_cycle[channel]==0) )
    {
        for(i=0; i<param->NUM_RANKS; i++)
  		{
			// if the issued refresh command does not exceed 8
  	  		if((is_refresh_allowed(channel,i)) && (refreshes[channel][i] < 8))
	  	  	{
				// restriction to prevent bug in the memory_controller.c
  		  		if( current_core_cycle[0] < (next_refresh_completion_deadline[channel][i]-param->T_RP-(9-num_issued_refreshes[channel][i])*param->T_RFC) ) 
	  			{
					// issue refresh command
	  	      		issue_refresh_command(channel,i);

					// increase the number of the refreshes
	  	      		refreshes[channel][i]++;

	  		  		// update the refresh_issue deadline
	  		  		refresh_issue_deadline[channel][i] = next_refresh_completion_deadline[channel][i] - param->T_RP - (8-num_issued_refreshes[channel][i]) * param->T_RFC;
					break;
	  			}
  		  	}
  		}
    }

	// Draining Reads
	// look through the queue and find the first request whose
	// command can be issued in this cycle and issue it 
	// Drain first ready(row-hit) request, and then first-come request.
	if(!drain_writes[channel])
	{
		// FRFS
		// search from the head of the read queue
		LL_FOREACH(read_queue_head[channel],rd_ptr)
		{
			// If the address of the read request is row hit while the read command is issuable at current cycle.
			if( (rd_ptr->command_issuable) && (rd_ptr->next_command==COL_READ_CMD) )
			{
				int rank = rd_ptr->dram_addr.rank;
				int bank = rd_ptr->dram_addr.bank;
				// set recent_colacc to indicate aggressive precharge candidate
			    recent_colacc[channel][rank][bank] = 1;

				// increase the read_history_counter, because the read command is issued
				read_history_counter[channel][rank][bank]++;

				// increase the request density counter
				request_density_counter[channel]++;

				// set the delayed_close_counter
				// this value is decreased by 1 for every cycle
				delayed_close_counter_r[channel][rank][bank] = CLOSE_DELAYING_TIME;

				// register update for print out
				if(request_type_last[channel] == READ) {
					read_to_read[channel]++;
				}
				else if(request_type_last[channel] == WRITE) {
					write_to_read[channel]++;
				}
				request_type_last[channel] = READ;

				// issue read 
				issue_request_command(rd_ptr);
				break;
			}
		}

		// FCFS
		// if FR is not exist in queue
		if(!command_issued_current_cycle[channel])
		{
			// search from the head of the read queue
			LL_FOREACH(read_queue_head[channel],rd_ptr)
			{
				// if the read request is issuable at current cycle
				if(rd_ptr->command_issuable)
				{
					int rank = rd_ptr->dram_addr.rank;
					int bank = rd_ptr->dram_addr.bank;

					// active
					if (rd_ptr->next_command == ACT_CMD) {
					  // reset the recent_colacc
					  recent_colacc[channel][rank][bank] = 0;

					  // decrease the read history counter
					  // it is increased when the read is issued,
					  // and is decreased when the active is issued
					  read_history_counter[channel][rank][bank]--;

					  // issue active command
					  issue_request_command(rd_ptr);
					  break;
					}

					// precharge
					if (rd_ptr->next_command == PRE_CMD) {
					  // if there exists a read request in the read queue is row hit, 
					  // do not close a row
					  if( check_row_hit_in_rq(channel,rank,bank)==0 ) {
						// timing check
						if( is_precharge_allowed(channel,rank,bank) ) {

							// if the delayed_close is turned off
							if( delayed_close_valid_r[channel][rank][bank]==0 ) {
								// issue precharge
								if( issue_precharge_command(channel,rank,bank) ) {
									// reset the recent_colacc
									recent_colacc[channel][rank][bank] = 0;
									break;
								}
							}
							// if the delayed close is turned on, and the delayed close counter is 0
							else if ( (delayed_close_valid_r[channel][rank][bank]==1) && (delayed_close_counter_r[channel][rank][bank]==0) ) {
								// issue precharge command
								if( issue_precharge_command(channel,rank,bank) ) {
									// reset the recent_colacc
									recent_colacc[channel][rank][bank] = 0;

									// reset the delayed_close_counter
									delayed_close_counter_r[channel][rank][bank] = CLOSE_DELAYING_TIME;
									break;
								}
							}
						}
					  }
					}
				}
			}
		}
	}

	// If a command hasn't yet been issued to this channel in this cycle, issue a precharge.
	// recent_colacc is referenced to judge if a certain bank is a candidate for the precharge or not
	if (!command_issued_current_cycle[channel]) {
	  for (i=0; i<param->NUM_RANKS; i++) {
	    for (j=0; j<param->NUM_BANKS; j++) {  
		  // if the recent_colacc is set
	      if (recent_colacc[channel][i][j]) {  
			 // while read drain, check the read queue if there exists row hit read request
			 if( (drain_writes[channel]==0) && (!check_row_hit_in_rq(channel,i,j)) ) {
				// check timing
	    	    if ( is_precharge_allowed(channel,i,j) ) {  
					// if the delayed close is turned off
					if( delayed_close_valid_r[channel][i][j] == 0 ) {
		  				if (issue_precharge_command(channel,i,j)) {
			    			recent_colacc[channel][i][j] = 0;
							break;
						}
					}
					// if the delayed close is turned on, and the delayed close counter is zero
					else if ( (delayed_close_valid_r[channel][i][j]==1) && (delayed_close_counter_r[channel][i][j]==0) ) {
						// issue precharge
		  				if (issue_precharge_command(channel,i,j)) {
							// reset the recent_colacc and delayed_close_counter
			    			recent_colacc[channel][i][j] = 0;
							delayed_close_counter_r[channel][i][j] = CLOSE_DELAYING_TIME;
							break;
						}
					}
				}
			 }

			 // while write drain, check the write queue if there exists row hit write request
			 else if( (drain_writes[channel]==1) && (!check_row_hit_in_wq(channel,i,j)) ) {
				 // check timing
				 if( is_precharge_allowed(channel,i,j) ) {
					 // if the delayed close is turned off
				     if( delayed_close_valid_w[channel][i][j]==0 ) {
						// issue precharge
						if( issue_precharge_command(channel,i,j) ) {
							// reset the recent_colacc
							recent_colacc[channel][i][j] = 0;
							break;
						}
					 }
					 // if the delayed close valid is turned on and the delayed close counter is zero
					 else if ( (delayed_close_valid_w[channel][i][j]==1) && (delayed_close_counter_w[channel][i][j]==0) ) {
						// issue precharge
						if( issue_precharge_command(channel,i,j) ) {
							// reset the recent_colacc and delayed close counter
							recent_colacc[channel][i][j] = 0;
							delayed_close_counter_w[channel][i][j] = CLOSE_DELAYING_TIME;
							break;
						}
					 }
				 }
			 }
	      }
	    }
	  }
	}

	// decrease the delayed close counter for every cycle
	for (i=0; i<param->NUM_RANKS; i++) {
    	for (j=0; j<param->NUM_BANKS; j++) {
			if(delayed_close_valid_r[channel][i][j]==1) {
				if(delayed_close_counter_r[channel][i][j] > 0) {
					delayed_close_counter_r[channel][i][j]--;
				}
			}
			if(delayed_close_valid_w[channel][i][j]==1) {
				if(delayed_close_counter_w[channel][i][j] > 0) {
					delayed_close_counter_w[channel][i][j]--;
				}
			}
		}
	}
}


void MemoryController::scheduler_stats()
{
  // user defined print 
  printf("=========== Special Information ==========\n");
  printf("time_over_high_watermark\n");	
  if(param->NUM_CHANNELS==1) {
	printf("%lld\n", time_over_high_watermark[0]);
  }
  else if(param->NUM_CHANNELS==4) {
	printf("%lld\n", time_over_high_watermark[0]+time_over_high_watermark[1]+time_over_high_watermark[2]+time_over_high_watermark[3]);
  }

  printf("time_under_low_watermark\n");
  if(param->NUM_CHANNELS==1) {
	printf("%lld\n", time_under_low_watermark[0]);
  }
  else if(param->NUM_CHANNELS==4) {
	printf("%lld\n", time_under_low_watermark[0]+time_under_low_watermark[1]+time_under_low_watermark[2]+time_under_low_watermark[3]);
  }

  printf("write to write\n");
  if(param->NUM_CHANNELS==1) {
	printf("%lld\n", write_to_write[0]);
  }
  else if(param->NUM_CHANNELS==4) {
	printf("%lld\n", write_to_write[0]+write_to_write[1]+write_to_write[2]+write_to_write[3]);
  }

  printf("write to read\n");
  if(param->NUM_CHANNELS==1) {
	printf("%lld\n", write_to_read[0]);
  }
  else if(param->NUM_CHANNELS==4) {
	printf("%lld\n", write_to_read[0]+write_to_read[1]+write_to_read[2]+write_to_read[3]);
  }

  printf("read to write\n");
  if(param->NUM_CHANNELS==1) {
	printf("%lld\n", read_to_write[0]);
  }
  else if(param->NUM_CHANNELS==4) {
	printf("%lld\n", read_to_write[0]+read_to_write[1]+read_to_write[2]+read_to_write[3]);
  }

  printf("read to read\n");
  if(param->NUM_CHANNELS==1) {
	printf("%lld\n", read_to_read[0]);
  }
  else if(param->NUM_CHANNELS==4) {
	printf("%lld\n", read_to_read[0]+read_to_read[1]+read_to_read[2]+read_to_read[3]);
  }
}
