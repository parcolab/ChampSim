/*
 * Write Leak memory scheduling Algorithm
 *
 * Authors: 
 *		Long Chen, longc@iastate.edu
 *		Yanan Cao, yanan@iastate.edu
 *		Sarah Kabala, skabala@iastate.edu
 *		Parijat Shukla parijats@iastate.edu
 *
 * ID: 9
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "utlist.h"
#include "utils.h"
#include "params.h"
#include "memory_controller.h"

int HI_WM;
int LO_WM;
int MD_WM;

//extern long long int current_core_cycle[0];

//long long int write_queue_full_stats[MAX_NUM_CHANNELS]; //how many cycles is the write queue full, which stalls the ROB commiting
//long long int drain_mode_cycles_stats[MAX_NUM_CHANNELS];//how many cycles do the memory controller stay in write drain mode

int numBanksPerChannel; //number of banks in each channel

int *rdBkStatus;  // 1 means this bank has read request(s)
int *wrBkStatus;  // 1 means this bank has read request(s)
int *readableBkID; // 1 means there is some bank with read requests but no conflicting write requests
int *writeableBkID; // 1 means there is some bank with write requests but no conflicting read requests

// 1 means we are in write-drain mode for that channel
//int drain_writes[MAX_NUM_CHANNELS];

void MemoryController::init_scheduler_vars()
{
	//Number of banks per channel
	numBanksPerChannel = param->NUM_RANKS * param->NUM_BANKS;
	
	//initialize drain mode flags and stats variables
	int channel;
	for(channel= 0; channel< param->NUM_CHANNELS; channel++)
	{
		write_queue_full_stats[channel] = 0;
		drain_mode_cycles_stats[channel] = 0;
		drain_writes[channel] = 0;
	}
	
	//initialize Watermark values for write drain mode
	HI_WM=60;
	MD_WM=54;
	LO_WM=48;
	
	//allocate space for 
	rdBkStatus = (int*) malloc(sizeof(int) * numBanksPerChannel * param->NUM_CHANNELS);
	wrBkStatus = (int*) malloc(sizeof(int) * numBanksPerChannel * param->NUM_CHANNELS);
	readableBkID = (int*) malloc(sizeof(int) * numBanksPerChannel * param->NUM_CHANNELS);
	writeableBkID = (int*) malloc(sizeof(int) * numBanksPerChannel * param->NUM_CHANNELS);
	
	return;
}

// reset all bank status to 0
void MemoryController::init_bank_status(int channel)
{
	int bank = 0; 
	int base = channel * numBanksPerChannel;
	for(bank = 0; bank < numBanksPerChannel; bank++)
	{
		rdBkStatus[base + bank] = 0;
		wrBkStatus[base + bank] = 0;
	}
}

// fill banks status variables with correct value
// 1 means there is request to that bank, 0 otherwise
void MemoryController::get_bank_status(int channel)
{
	request_t * rd_ptr = NULL;
	request_t * wr_ptr = NULL;

	// indexing bank from 0 to NUM_RANKS*NUM_BANKS
	int globalBkIdx = 0;

	// offset to distinguish channels
	int base = channel * numBanksPerChannel;

	init_bank_status(channel);

	//traverse read and write queues to fill the rdBkStatus and wrBkStatus
	LL_FOREACH(read_queue_head[channel], rd_ptr)
	{
		globalBkIdx = (rd_ptr->dram_addr).rank * param->NUM_BANKS + (rd_ptr->dram_addr).bank + base;
		rdBkStatus[globalBkIdx] = 1;
	}
	
	LL_FOREACH(write_queue_head[channel], wr_ptr)
	{
		globalBkIdx = (wr_ptr->dram_addr).rank * param->NUM_BANKS + (wr_ptr->dram_addr).bank + base;
		wrBkStatus[globalBkIdx] = 1;
	}
}

// go through the bank status variable to see if there is any read requests that doesn't have
// no bank conflict or row conflict with write requests
int MemoryController::get_readable_bank(int channel)
{
	int i = 0;
	int base = channel * numBanksPerChannel;

	// only when a bank has read request but no write request readableBkID is 1
	// e.g. rd: 0100 1100, wr: 1011 0110; readable: 
	for(i = 0; i < numBanksPerChannel; i++)
		readableBkID[base + i] = (rdBkStatus[base + i] ^ wrBkStatus[base + i]) & rdBkStatus[base + i];

	//as long as there is one readable bank, return 1, otherwise return 0
	int readable = 0;
	for(i = 0; i < numBanksPerChannel; i++)
		readable |= readableBkID[base + i];
	
	return readable;
}

// go through the bank status variable to see if there is any write requests that doesn't have
// no bank conflict or row conflict with read requests
int MemoryController::get_writeable_bank(int channel)
{
	int i = 0;
	int base = channel * numBanksPerChannel;

	// only when a bank has write request but no read request readableBkID is 1
	for(i = 0; i < numBanksPerChannel; i++)
		writeableBkID[base + i] = (rdBkStatus[base + i] ^ wrBkStatus[base + i]) & wrBkStatus[base + i];

	//as long as there is one writeable bank, return 1, otherwise return 0
	int writeable = 0;
	for(i = 0; i < numBanksPerChannel; i++)
		 writeable |= writeableBkID[base + i];
	
	return writeable;
}

void MemoryController::schedule(int channel)
{
	request_t * rd_ptr = NULL;
	request_t * wr_ptr = NULL;
	request_t * auto_ptr = NULL;
	
	// flag variable that tells us whether to pre issue some commands
	// including PRE/ACT to prepare for read requests
	// so that COL_ACCESS can be issued after drain mode
	int pre_issue_read_cmd = 0;

	//count how many cycles the write queue stalls ROB committing
    if(write_queue_length[channel] == DRAM_WQ_SIZE)
        write_queue_full_stats[channel]+=param->PROCESSOR_CLK_MULTIPLIER;

   	//if last cycle was in drain mode and write queue is still long, keep draining
	if(drain_writes[channel] && (write_queue_length[channel] > MD_WM))
		drain_writes[channel] = 1;
	else if(drain_writes[channel] && (write_queue_length[channel] > LO_WM))
	{ //if write queue is relatively short, which means we are close to
	  // the end of write drain mode, we can issue some command to prepare for reads
		if(!pre_issue_read_cmd)
		{	
			get_bank_status(channel);

			//see if there is a bank that has read that does not conflict with any write
			pre_issue_read_cmd = get_readable_bank(channel);
		}

		drain_writes[channel] = 1;
	}
	else
		drain_writes[channel] = 0;
         
	// if write queue length is over danger line, always drain
    if(write_queue_length[channel] > HI_WM)
		drain_writes[channel] = 1;
	else if(!read_queue_length[channel])
		drain_writes[channel] = 1;

	if(drain_writes[channel]) // drain writs
	{
		
		//stats update for drain mode cycles
		drain_mode_cycles_stats[channel]+=param->PROCESSOR_CLK_MULTIPLIER;

		//traverse write queue to find WRITE ROW BUFFER HIT request and execute it
		LL_FOREACH(write_queue_head[channel], wr_ptr)
		{
			if(wr_ptr->command_issuable && (wr_ptr->next_command == COL_WRITE_CMD))
			{	
				issue_request_command(wr_ptr);
				
				//try to issue auto precharge
				if (is_autoprecharge_allowed(channel,wr_ptr->dram_addr.rank,wr_ptr->dram_addr.bank))
				{
					LL_FOREACH(write_queue_head[channel], auto_ptr)
					{
						if (!auto_ptr->request_served
							&& auto_ptr->dram_addr.rank == wr_ptr->dram_addr.rank
							&& auto_ptr->dram_addr.bank == wr_ptr->dram_addr.bank
							&& auto_ptr->dram_addr.row == wr_ptr->dram_addr.row) 
							return; // has hit pending to this row, no auto precharge
		
					}
					// no hit pending to this row, auto precharge
					issue_autoprecharge(channel, wr_ptr->dram_addr.rank, wr_ptr->dram_addr.bank);
				}

				return;
			}
		}

		//traverse write queue to find ANY ISSUABLE WRITE request and execute it
		LL_FOREACH(write_queue_head[channel], wr_ptr)
		{
			if(wr_ptr->command_issuable )
			{
				issue_request_command(wr_ptr);
				
				//try to issue auto precharge
				if (is_autoprecharge_allowed(channel,wr_ptr->dram_addr.rank,wr_ptr->dram_addr.bank))
				{
					LL_FOREACH(write_queue_head[channel], auto_ptr)
					{
						if (!auto_ptr->request_served
							&& auto_ptr->dram_addr.rank == wr_ptr->dram_addr.rank
							&& auto_ptr->dram_addr.bank == wr_ptr->dram_addr.bank
							&& auto_ptr->dram_addr.row == wr_ptr->dram_addr.row) 
							return; // has hit, no auto precharge
					}
					// no hit pending, auto precharge
					issue_autoprecharge(channel, wr_ptr->dram_addr.rank, wr_ptr->dram_addr.bank);
				}
				return;
			}
		}
			
		//if we are allowed to execute PRE/ACT to prepare for reads
		//find a read request that is not COL_READ_CMD, which may take over the bus
		if(pre_issue_read_cmd)
		{
			LL_FOREACH(read_queue_head[channel], rd_ptr)
			{
				int rd_global_bk_idx = (rd_ptr->dram_addr).rank * param->NUM_BANKS + (rd_ptr->dram_addr).bank + channel * numBanksPerChannel;
				if(rd_ptr->command_issuable && readableBkID[rd_global_bk_idx] && rd_ptr->next_command != COL_READ_CMD)
				{
					// the bank has no write and then do the issue read command
					issue_request_command(rd_ptr);
					return;
				}
			}
		}

		return;
	}

	// From here we are outside drain mode 
	
	//traverse the read queue to find READ ROW BUFFER HIT request 
	//and issue autoprecharge if there is no further hit to this row
	LL_FOREACH(read_queue_head[channel],rd_ptr)
	{
		if(rd_ptr->command_issuable && (rd_ptr->next_command == COL_READ_CMD))
		{
			issue_request_command(rd_ptr);	
			
			// try to issue auto precharge
			if (is_autoprecharge_allowed(channel,rd_ptr->dram_addr.rank,rd_ptr->dram_addr.bank))
			{
				LL_FOREACH(read_queue_head[channel], auto_ptr)
				{
					if (!auto_ptr->request_served
						&& auto_ptr->dram_addr.rank == rd_ptr->dram_addr.rank
						&& auto_ptr->dram_addr.bank == rd_ptr->dram_addr.bank
						&& auto_ptr->dram_addr.row == rd_ptr->dram_addr.row) 
						return; // has hit, no auto precharge
				}
				// no hit pending, auto precharge
				issue_autoprecharge(channel, rd_ptr->dram_addr.rank, rd_ptr->dram_addr.bank);
			}		
			return;
		}
	}

	//traverse the read queue to find WRITE ROW BUFFER HIT request 
	//and issue autoprecharge if there is no further hit to this row
	LL_FOREACH(write_queue_head[channel], wr_ptr)
	{
		if(wr_ptr->command_issuable && (wr_ptr->next_command == COL_WRITE_CMD))
		{
			issue_request_command(wr_ptr);
			
			//try to issue auto precharge
			if (is_autoprecharge_allowed(channel,wr_ptr->dram_addr.rank,wr_ptr->dram_addr.bank))
			{
				LL_FOREACH(write_queue_head[channel], auto_ptr)
				{
					if (!auto_ptr->request_served
						&& auto_ptr->dram_addr.rank == wr_ptr->dram_addr.rank
						&& auto_ptr->dram_addr.bank == wr_ptr->dram_addr.bank
						&& auto_ptr->dram_addr.row == wr_ptr->dram_addr.row) 
						return; // has hit, no auto precharge
				}
				// no hit pending, auto precharge
				issue_autoprecharge(channel, wr_ptr->dram_addr.rank, wr_ptr->dram_addr.bank);
			}
			return;
		}
	}
		
	//traverse the read queue to find ANY ISSUABLE READ request 
	//and issue autoprecharge if there is no further hit to this row
	LL_FOREACH(read_queue_head[channel],rd_ptr)
	{
		if(rd_ptr->command_issuable)
		{
			issue_request_command(rd_ptr);	
			
			// try to issue auto precharge
			if (is_autoprecharge_allowed(channel,rd_ptr->dram_addr.rank,rd_ptr->dram_addr.bank))
			{
				LL_FOREACH(read_queue_head[channel], auto_ptr)
				{
					if (!auto_ptr->request_served
						&& auto_ptr->dram_addr.rank == rd_ptr->dram_addr.rank
						&& auto_ptr->dram_addr.bank == rd_ptr->dram_addr.bank
						&& auto_ptr->dram_addr.row == rd_ptr->dram_addr.row) 
						return; // has hit, no auto precharge
				}
				// no hit pending, auto precharge
				issue_autoprecharge(channel, rd_ptr->dram_addr.rank, rd_ptr->dram_addr.bank);
			}	
			return;
		}
	}
		
	// taking last five bits of current_core_cycle[0]
	// it that is 0, current_core_cycle[0] is divisible by 32
	// Since current_core_cycle[0] advances by PROCESSOR_CLK_MULTIPLIER=4
	// the chance of divisible by 32 is 4/32, which is 12.5%
	unsigned int mask=0x1F; //in binary this is 00011111
	if( (current_core_cycle[0] & mask) == 0)
	{
		get_bank_status(channel);

		//find write request that doesn't conflict with read requests
		int writeable = get_writeable_bank(channel);

		if(writeable)
		{
			//if we find a write request that is non-conflict with read, issueit
			LL_FOREACH(write_queue_head[channel], wr_ptr)
			{
				int global_bk_idx = wr_ptr->dram_addr.rank * param->NUM_BANKS + wr_ptr->dram_addr.bank + channel * numBanksPerChannel;
				if(wr_ptr->command_issuable && writeableBkID[global_bk_idx])
				{
					issue_request_command(wr_ptr);
					return;
				}
			}
		}
	}	
	return;
}

void MemoryController::scheduler_stats()
{
	int channel = 0;
	for(channel = 0; channel < param->NUM_CHANNELS; channel++)
	{
		printf("write_queue_full_stats[%d]:                   %8lld\n",channel, write_queue_full_stats[channel]);
		printf("write_queue_full_percent[%d]:                 %f\n", channel, (double)write_queue_full_stats[channel]/(double)current_core_cycle[0]);
		printf("drain_write_mode_percent[%d]:                 %f\n", channel, (double)drain_mode_cycles_stats[channel]/(double)current_core_cycle[0]);
	}
}


