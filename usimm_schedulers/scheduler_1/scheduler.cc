#ifndef SCHEDULER_H
#define SCHEDULER_H
#include <stdio.h>
#include "utlist.h"
#include "utils.h"

#include "memory_controller.h"
#include "params.h"

typedef unsigned int uint;


#define MAX_NUM_CPUS (16)
#define LASTREADLEN (1)
#define GHTLEN (512)



int MAX_REF_CNT = 127;
int TH_BOOST = 7;
long long int MAX_INTERVAL = 1000 * 1000;
long long int MAX_QUANTUM = 1000;
int ght_refcnt_inc = 1;
int ght_refcnt_dec = 0;
int ght_refcnt_init = 1;

int coe_readcnt = 8;
int coe_gain = 1;
int coe_tcm = 0;
int coe_tcm_latencycluster = 16;
int TH_GAIN;
int NUM_TCM_LATENCY_CLUSTER = 1;
int WAIT_CYCLE_DEF;

MemoryController::GhtEntry_t ght [MAX_NUM_CPUS][GHTLEN];
MemoryController::LastReadStat_t lastread [MAX_NUM_CPUS][LASTREADLEN];
int lastread_head [MAX_NUM_CPUS], lastread_tail [MAX_NUM_CPUS];
long long int latestread_time [MAX_NUM_CPUS];

long long int interval_cnt;
long long int quantum_cnt;
long long int tcm_read_cnt [MAX_NUM_CPUS];
long long int all_read_cnt[MAX_NUM_CPUS];
int tcm_priority [MAX_NUM_CPUS];
int tcm_priority_pos;
int tcm_up;
int gainful_thread [MAX_NUM_CPUS];
int gainful_thread_ptr [MAX_NUM_CHANNELS];

int gainful_inst_thread [MAX_NUM_CPUS];
int gainful_inst_thread_ptr [MAX_NUM_CHANNELS];

/******************************************************************************/
//extern int NUM_CPUS;

/* A data structure to see if a bank is a candidate for precharge. */
int recent_colacc[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
long long int lastrow[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];

/* Keeping track of how many preemptive precharges are performed. */
int HI_WM;
int LO_WM;

// 1 means we are in write-drain mode for that bank
int channel_drain_writes[MAX_NUM_CHANNELS];
int drain_writes[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
int ignore_bank[MAX_NUM_RANKS][MAX_NUM_BANKS];
int refreshes[MAX_NUM_CHANNELS][MAX_NUM_RANKS];
/******************************************************************************/
int activating_by_drain_writes[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
int wait_cycle_for_drain_writes[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];

/******************************************************************************/
void MemoryController::init_ght()
{
  for(int c=0; c<NUM_CPUS; c++){
    for(int i=0; i<GHTLEN; i++){
      ght[c][i].valid = 0;
      ght[c][i].tag = 0;
      ght[c][i].refcnt = 0;
    }
  }
}

void MemoryController::init_lastread()
{
  for(int c=0; c<NUM_CPUS; c++){
    for(int i=0; i<LASTREADLEN; i++){
      lastread[c][i].valid = 0;
      lastread[c][i].pc = 0;
      lastread[c][i].gain = 0;
    }
    lastread_head[c] = 0;
    lastread_tail[c] = 0;
    latestread_time[c] = 0;
  }
}

void MemoryController::init_gainful_thread()
{
    //TH_GAIN = 2000 * NUM_CPUS / NUM_CHANNELS;
    //TH_GAIN = 1000 * NUM_CPUS / NUM_CHANNELS;
  TH_GAIN = 1000 * NUM_CPUS;    
  if(NUM_CPUS == 1){
    NUM_TCM_LATENCY_CLUSTER = 2;
    coe_readcnt = 8;
    coe_gain = 1;
    coe_tcm = 0;
    coe_tcm_latencycluster = 16;
  }
  else if(NUM_CPUS == 2){
    NUM_TCM_LATENCY_CLUSTER = 2;
    coe_readcnt = 8;
    coe_gain = 1;
    coe_tcm = 0;
    coe_tcm_latencycluster = 16;
  }
  else if(NUM_CPUS <= 4){
    NUM_TCM_LATENCY_CLUSTER = 2;
    coe_readcnt = 8;
    coe_gain = 1;
    coe_tcm = 0;
    coe_tcm_latencycluster = 16;
  }
  else if(NUM_CPUS <= 8){
    NUM_TCM_LATENCY_CLUSTER = 3;
    coe_readcnt = 8;
    coe_gain = 1;
    coe_tcm = 1;
    coe_tcm_latencycluster = 16;
  }
  else if(NUM_CPUS <= 16){
    NUM_TCM_LATENCY_CLUSTER = 3;
    coe_readcnt = 8;
    coe_gain = 1;
    coe_tcm = 0;//or 1?
    coe_tcm_latencycluster = 16;
  }else{
    NUM_TCM_LATENCY_CLUSTER = 3;
    coe_readcnt = 8;
    coe_gain = 1;
    coe_tcm = 0;
    coe_tcm_latencycluster = 16;
  }

  for(int i=0; i<NUM_CPUS; i++){
    tcm_priority[i] = -1;
    tcm_read_cnt[i] = 0;
    all_read_cnt[i] = 0;
  }
  tcm_priority_pos = 0;
  tcm_up = 1;
  interval_cnt = 0;
  quantum_cnt = 0;

  for(int i=0; i<NUM_CPUS; i++){
    gainful_thread[i] = -1;
  }
  for(int i=0; i<param->NUM_CHANNELS; i++){
    gainful_thread_ptr[i] = 0;
  }

  init_ght();
  init_lastread();
}

/******************************************************************************/
MemoryController::GhtEntry_t* MemoryController::get_ght(int core, uint pc)
{
  //full-way set associative
  GhtEntry_t* ret = NULL;
  for(int i=0; i<GHTLEN; i++){
    GhtEntry_t* g = &ght[core][i];
    if(g->valid != 0 && g->tag == pc) {
      ret = g;
      break;
    }
  }
  return ret;
}

MemoryController::GhtEntry_t* MemoryController::spil_ght(int core)
{
  //full-way set associative
  int min_refcnt = MAX_REF_CNT + 1;
  GhtEntry_t* ret = NULL;
  for(int i=0; i<GHTLEN; i++){
    GhtEntry_t* g = &ght[core][i];
    if(g->valid == 0){
      ret = g;
      break;
    }
    else if(g->valid != 0 && g->refcnt < min_refcnt) {
      min_refcnt = g->refcnt;
      ret = g;
    }
  }
  return ret;
}

void MemoryController::commit_ght(int core, uint pc)
{
  GhtEntry_t* g = get_ght(core, pc);
  if(g){
    g->valid = 1;
    g->refcnt += ght_refcnt_inc;
    if(g->refcnt >= MAX_REF_CNT) g->refcnt = MAX_REF_CNT;
  }else{
    g = spil_ght(core);
    g->valid = 1;
    g->tag = pc;
    g->refcnt = ght_refcnt_init;
  }
}

void MemoryController::decrement_ght(int core, uint pc)
{
  GhtEntry_t* g = get_ght(core, pc);
  if(g && g->refcnt > 0){
    g->refcnt -= ght_refcnt_dec;
  }
}

void MemoryController::commit_lastread(int core, uint pc)
{
  int head_valid = lastread[core][lastread_head[core]].valid;
  int head_gain = lastread[core][lastread_head[core]].gain;
  uint head_pc = lastread[core][lastread_head[core]].pc;
  if(head_valid && head_gain >= TH_GAIN){
    commit_ght(core, head_pc);
  }
  else if(head_valid && head_gain < TH_GAIN){
    // decrement_ght(core, head_pc);
  }
  if(lastread_head[core] < LASTREADLEN-1) lastread_head[core]++;
  else lastread_head[core] = 0;
    
  //enque to last read queue
  lastread[core][lastread_tail[core]].valid = 1;
  lastread[core][lastread_tail[core]].pc = pc;
  lastread[core][lastread_tail[core]].gain = 0;
  if(lastread_tail[core] < LASTREADLEN-1) lastread_tail[core]++;
  else lastread_tail[core] = 0;
}

void MemoryController::update_lastread()
{
  request_t * rd_ptr = NULL;
    
  for(int c=0; c<NUM_CPUS; c++){
    for(int i=0; i<LASTREADLEN; i++){
      if(lastread[c][i].valid){
        lastread[c][i].gain++;
      }
    }
  }
    
  for(int ch=0; ch<param->NUM_CHANNELS; ch++){
    LL_FOREACH(read_queue_head[ch],rd_ptr){
      for(int c=0; c<NUM_CPUS; c++){
        if(rd_ptr->thread_id == c &&
           rd_ptr->arrival_time > latestread_time[c]){
          commit_lastread(c, rd_ptr->instruction_pc);
        }
      }
    }
  }
    
  //TCM
  quantum_cnt++;
  interval_cnt++;
  if(interval_cnt > MAX_INTERVAL){
    for(int c=0; c<NUM_CPUS; c++){
      tcm_priority[c] = -1;
    }
        
    int total_read = 0;
    int count_read = 0;
    long long int point[MAX_NUM_CPUS];
    for (int i = 0; i < NUM_CPUS; i++){
            
      total_read += tcm_read_cnt[i];
      point[i] = 0;
    }
    for(int i=0; i<NUM_CPUS; i++){
      for(int j=0; j<NUM_CPUS; j++){
        if(tcm_read_cnt[i] > tcm_read_cnt[j]){
          point[i] ++;
        }
        if(tcm_read_cnt[i] == tcm_read_cnt[j] ) { 
          if (all_read_cnt[i] >  all_read_cnt[j]){
            point[i] ++;
          }
          else if (all_read_cnt[i] ==  all_read_cnt[j]){
            if(i < j){
              point[i] ++;
            }
          }

        }
      }
    }
    for(int i=0; i<NUM_CPUS; i++){
      tcm_priority[point[i]] = i;
    }
        
    NUM_TCM_LATENCY_CLUSTER = 0;
    double thresh_rate = 0;
        
    while(thresh_rate < 0.10){
      for (int j = 0; j < NUM_CPUS; j++){
        if(tcm_priority[j] == NUM_TCM_LATENCY_CLUSTER){
          count_read += tcm_read_cnt[j];
          thresh_rate = (double)count_read / (double)total_read;
          NUM_TCM_LATENCY_CLUSTER ++;
          break;
        }
      }
    }
        
    interval_cnt = 0;
    //reset
    for(int c=0; c<NUM_CPUS; c++){
      tcm_read_cnt[c] = 0;
    }
  }else if(quantum_cnt > MAX_QUANTUM){
    //shuffle
    quantum_cnt = 0;
    if(NUM_CPUS == 1){
      //nothing
    }
    else if(tcm_up){
      int tmp_pri[MAX_NUM_CPUS];
      for(int c=NUM_TCM_LATENCY_CLUSTER; c<NUM_CPUS; c++){
        if(c == NUM_TCM_LATENCY_CLUSTER){
          tmp_pri[c] = tcm_priority[tcm_priority_pos];
        }else if(c <= tcm_priority_pos){
          tmp_pri[c] = tcm_priority[c-1];
        }else{
          tmp_pri[c] = tcm_priority[c];
        }
      }
      for(int c=NUM_TCM_LATENCY_CLUSTER; c<NUM_CPUS; c++){
        tcm_priority[c] = tmp_pri[c];
      }
      tcm_priority_pos++;
      if(tcm_priority_pos == NUM_CPUS){
        tcm_up = 0;
      }
    }else{
      int tmp_pri[MAX_NUM_CPUS];
      if(tcm_priority_pos != NUM_CPUS){
        for(int c=NUM_TCM_LATENCY_CLUSTER; c<NUM_CPUS; c++){
          if(c < tcm_priority_pos){
            tmp_pri[c] = tcm_priority[c+1];
          }else if(c == tcm_priority_pos){
            tmp_pri[c] = tcm_priority[NUM_TCM_LATENCY_CLUSTER];
          }else{
            tmp_pri[c] = tcm_priority[c];
          }
        }
        for(int c=NUM_TCM_LATENCY_CLUSTER; c<NUM_CPUS; c++){
          tcm_priority[c] = tmp_pri[c];
        }
      }
      tcm_priority_pos--;
      if(tcm_priority_pos == NUM_TCM_LATENCY_CLUSTER){
        tcm_up = 1;
      }
    }
  }
  //counting
  for(int ch=0; ch<param->NUM_CHANNELS; ch++){
    LL_FOREACH(read_queue_head[ch],rd_ptr){
      for(int c=0; c<NUM_CPUS; c++){
        if(rd_ptr->thread_id == c &&
           rd_ptr->arrival_time > latestread_time[c]){
          tcm_read_cnt[c]++;
          all_read_cnt[c]++;
        }
      }
    }
  }
    
  //Update Last Read Time
  for(int ch=0; ch<param->NUM_CHANNELS; ch++){
    LL_FOREACH(read_queue_head[ch],rd_ptr){
      for(int c=0; c<NUM_CPUS; c++){
        if(rd_ptr->thread_id == c &&
           rd_ptr->arrival_time > latestread_time[c]){
          latestread_time[c] = rd_ptr->arrival_time;
        }
      }
    }
  }
}

/******************************************************************************/
void MemoryController::sort_gainful_thread()
{
  request_t * rd_ptr = NULL;
  int gain_count[MAX_NUM_CPUS];
  int read_count[MAX_NUM_CPUS];
  int total_read_count;
    
  for(int i=0; i<NUM_CPUS; i++){
    gainful_thread[i] = -1;
    gainful_inst_thread[i] = -1;
    gain_count[i] = 0;
    read_count[i] = 0;
  }
  total_read_count = 0;
    
  int channel;
  for(channel=0; channel<param->NUM_CHANNELS; channel++){
    int pos_readque = 0;
    LL_FOREACH(read_queue_head[channel], rd_ptr) {
      uint th = rd_ptr->thread_id;
      read_count[th]++;
      total_read_count++;
      uint pc = rd_ptr->instruction_pc;
      GhtEntry_t* e = get_ght(th, pc);
      if(e){
        if(pos_readque < TH_BOOST){
            //gain_count[th] += (e->refcnt) * coe_gain;
        }
      }
      pos_readque++;
    }
  }
  
  int inst_point [MAX_NUM_CPUS];
  int inst_pos [MAX_NUM_CPUS];
  for (int i = 0; i < NUM_CPUS; i++){
      inst_point[i] = 0;
      inst_pos[i] = 0;
  }
  for(channel=0; channel<param->NUM_CHANNELS; channel++){
      LL_FOREACH(read_queue_head[channel], rd_ptr) {
          uint th = rd_ptr->thread_id;
          uint pc = rd_ptr->instruction_pc;
          GhtEntry_t* e = get_ght(th, pc);
          if(e){
              if(inst_pos[th] == 0){
                  inst_point[th] += (e->refcnt) * coe_gain;
              }
          }
          inst_pos[th]++;
      }
  }
  for(int i=0; i<NUM_CPUS; i++){
      gainful_inst_thread[i] = i;
      if(inst_point[i] == 0) gainful_inst_thread[i] = -1;
  }
  for(int i=0; i<NUM_CPUS; i++){
      for(int j=i+1; j<NUM_CPUS; j++){
          if(inst_point[j] > inst_point[i]){
              int tmp = inst_point[i];
              inst_point[i] = inst_point[j];
              inst_point[j] = tmp;
              tmp = gainful_inst_thread[i];
              gainful_inst_thread[i] = gainful_inst_thread[j];
              gainful_inst_thread[j] = tmp;
          }
      }
  }
  
  for(int i=0; i<NUM_CPUS; i++){
      if(i > 1) gainful_inst_thread[i] = -1;
  }
  


    
  for(int i=0; i<NUM_CPUS; i++){
    gain_count[i] += coe_readcnt * (total_read_count - read_count[i]);
  }
  for(int i=0; i<NUM_CPUS; i++){
    int t = tcm_priority[i];
    if(t < NUM_TCM_LATENCY_CLUSTER){
      if(i < NUM_CPUS){
        gain_count[i] += coe_tcm * (NUM_CPUS * coe_tcm_latencycluster);
      }
    }
    else{
      if(i < NUM_CPUS){
        gain_count[i] += coe_tcm * (NUM_CPUS - t -1);
      }
    }
  }
  int tmp_cnt = 0;
  for(int i=0; i<NUM_CPUS; i++){
    if(gain_count[i] <= 0){
      tmp_cnt ++;
    }
  }
    
  long long int point[MAX_NUM_CPUS];
  for (int i = 0; i < NUM_CPUS; i++){
    point[i] = 0;
  }
    
  for(int i=0; i<NUM_CPUS; i++){
    for(int j=0; j<NUM_CPUS; j++){
      if(gain_count[i] < gain_count[j] ){
        point[i] ++;
      }
      if(gain_count[i] == gain_count[j]){
        if (tcm_priority[i] >  tcm_priority[j]){
          point[i] ++;
        }
        else if (tcm_priority[i] ==  tcm_priority[j]){
          if(i < j){
            point[i] ++;
          }
        }
      }
    }
  }
    
  for(int i=0; i<NUM_CPUS; i++){
    gainful_thread[point[i]] = i;
  }
}

void MemoryController::init_gainful_thread_ptr(int channel)
{
  gainful_thread_ptr[channel] = 0;
}

void MemoryController::init_gainful_inst_thread_ptr(int channel)
{
  gainful_inst_thread_ptr[channel] = 0;
}

void MemoryController::set_gainful_thread()
{
  update_lastread();
  for(int i=0; i<MAX_NUM_CHANNELS; i++){
    init_gainful_thread_ptr(i);
    init_gainful_inst_thread_ptr(i);
  }
  sort_gainful_thread();
}

int MemoryController::get_gainful_thread(int channel)
{
  if(gainful_thread_ptr[channel] < NUM_CPUS){
    int ret = gainful_thread[gainful_thread_ptr[channel]];
    gainful_thread_ptr[channel]++;
    return ret;
  }
  return -1;
}

int MemoryController::get_gainful_inst_thread(int channel)
{
  if(gainful_inst_thread_ptr[channel] < NUM_CPUS){
    int ret = gainful_inst_thread[gainful_inst_thread_ptr[channel]];
    gainful_inst_thread_ptr[channel]++;
    return ret;
  }
  return -1;
}

/******************************************************************************/
void MemoryController::set_drain_writes_mode(int channel)
{
  // end of channel write drain mode
  if (channel_drain_writes[channel] && write_queue_length[channel] < LO_WM){
    channel_drain_writes[channel] = 0;
  }
  // beginning of channel write drain mode
  if (write_queue_length[channel] > HI_WM){
    channel_drain_writes[channel] = 1;
  }
    

  for (int i=0; i<param->NUM_RANKS; i++){
    for (int j=0; j<param->NUM_BANKS; j++) {
      drain_writes[channel][i][j] = 1;
      ignore_bank[i][j] = 0;
    }
  }


  // if not in channel write drain mode and there is/are pending read request(s) in a bank,
  // read request(s) will precede in that bank.
  if (! channel_drain_writes[channel]) {
    request_t * rd_ptr = NULL;
    LL_FOREACH(read_queue_head[channel], rd_ptr) {
      drain_writes[channel][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank] = 0;
    }
  }
}

/******************************************************************************/
int MemoryController::read_rowhit_thread(int channel, int thread_id)
{
  request_t * rd_ptr = NULL;
  int rank, bank, row;
  LL_FOREACH(read_queue_head[channel],rd_ptr) {
    rank = rd_ptr->dram_addr.rank;
    bank = rd_ptr->dram_addr.bank;
    row = rd_ptr->dram_addr.row;
    // issue read requests only when the bank is not in write drain mode
    if (! drain_writes[channel][rank][bank]) {
      if (rd_ptr->command_issuable &&
          rd_ptr->thread_id == thread_id &&
          dram_state[channel][rank][bank].active_row == row){ //Row Hit
                
        if (rd_ptr->next_command == COL_READ_CMD) {
          recent_colacc[channel][rank][bank] = 1;
          lastrow[channel][rank][bank] = rd_ptr->dram_addr.row;
          wait_cycle_for_drain_writes[channel][rank][bank] = WAIT_CYCLE_DEF;
        }
        if (rd_ptr->next_command == ACT_CMD) {
          if (activating_by_drain_writes[channel][rank][bank] == -1 ||
              activating_by_drain_writes[channel][rank][bank] == rd_ptr->dram_addr.row) {
            activating_by_drain_writes[channel][rank][bank] = 0;
          }
          recent_colacc[channel][rank][bank] = 0;
        }
        if (rd_ptr->next_command == PRE_CMD) {
          recent_colacc[channel][rank][bank] = 0;
        }
        issue_request_command(rd_ptr);
        return 1;
      }
    }
  }
  return 0;
}

int MemoryController::read_thread(int channel, int thread_id)
{
  request_t * rd_ptr = NULL;
  int rank, bank;
  LL_FOREACH(read_queue_head[channel],rd_ptr) {
    rank = rd_ptr->dram_addr.rank;
    bank = rd_ptr->dram_addr.bank;
    // issue read requests only when the bank is not in write drain mode
    if (! drain_writes[channel][rank][bank]) {
      //Filtering the GAINFUL THREAD
      if (rd_ptr->command_issuable &&
          rd_ptr->thread_id == thread_id){
        if (rd_ptr->next_command == COL_READ_CMD) {
          recent_colacc[channel][rank][bank] = 1;
          lastrow[channel][rank][bank] = rd_ptr->dram_addr.row;
          wait_cycle_for_drain_writes[channel][rank][bank] = WAIT_CYCLE_DEF;
        }
        if (rd_ptr->next_command == ACT_CMD) {
          if (activating_by_drain_writes[channel][rank][bank] == -1 ||
              activating_by_drain_writes[channel][rank][bank] == rd_ptr->dram_addr.row) {
            activating_by_drain_writes[channel][rank][bank] = 0;
          }
          recent_colacc[channel][rank][bank] = 0;
        }
        if (rd_ptr->next_command == PRE_CMD) {
          recent_colacc[channel][rank][bank] = 0;
        }
        issue_request_command(rd_ptr);
        return 1;
      }
    }
  }
  return 0;
}

int MemoryController::read_rowhit(int channel)
{
  request_t * rd_ptr = NULL;
  int rank, bank, row;
  LL_FOREACH(read_queue_head[channel],rd_ptr) {
    rank = rd_ptr->dram_addr.rank;
    bank = rd_ptr->dram_addr.bank;
    row = rd_ptr->dram_addr.row;
    // issue read requests only when the bank is not in write drain mode
    if (! drain_writes[channel][rank][bank]) {
      if (rd_ptr->command_issuable &&
          dram_state[channel][rank][bank].active_row == row){ //Row Hit
        if (rd_ptr->next_command == COL_READ_CMD) {
          recent_colacc[channel][rank][bank] = 1;
          lastrow[channel][rank][bank] = rd_ptr->dram_addr.row;
          wait_cycle_for_drain_writes[channel][rank][bank] = WAIT_CYCLE_DEF;
        }
        if (rd_ptr->next_command == ACT_CMD) {
          if (activating_by_drain_writes[channel][rank][bank] == -1 ||
              activating_by_drain_writes[channel][rank][bank] == rd_ptr->dram_addr.row) {
            activating_by_drain_writes[channel][rank][bank] = 0;
          }
          recent_colacc[channel][rank][bank] = 0;
        }
        if (rd_ptr->next_command == PRE_CMD) {
          recent_colacc[channel][rank][bank] = 0;
        }
        issue_request_command(rd_ptr);
        return 1;
      }
    }
  }
  return 0;
}

int MemoryController::read(int channel)
{
  request_t * rd_ptr = NULL;
  int rank, bank;
  LL_FOREACH(read_queue_head[channel],rd_ptr) {
    rank = rd_ptr->dram_addr.rank;
    bank = rd_ptr->dram_addr.bank;
    // issue read requests only when the bank is not in write drain mode
    if (! drain_writes[channel][rank][bank]) {
      if (rd_ptr->command_issuable){
        if (rd_ptr->next_command == COL_READ_CMD) {
          recent_colacc[channel][rank][bank] = 1;
          lastrow[channel][rank][bank] = rd_ptr->dram_addr.row;
          wait_cycle_for_drain_writes[channel][rank][bank] = WAIT_CYCLE_DEF;
        }
        if (rd_ptr->next_command == ACT_CMD) {
          if (activating_by_drain_writes[channel][rank][bank] == -1 ||
              activating_by_drain_writes[channel][rank][bank] == rd_ptr->dram_addr.row) {
            activating_by_drain_writes[channel][rank][bank] = 0;
          }
          recent_colacc[channel][rank][bank] = 0;
        }
        if (rd_ptr->next_command == PRE_CMD) {
          recent_colacc[channel][rank][bank] = 0;
        }
        issue_request_command(rd_ptr);
        return 1;
      }
    }
  }
  return 0;
}

int MemoryController::activate_thread(int channel, int thread_id)
{
  request_t * rd_ptr = NULL;
  int rank, bank, row;
  LL_FOREACH(read_queue_head[channel],rd_ptr){
    rank = rd_ptr->dram_addr.rank;
    bank = rd_ptr->dram_addr.bank;
    row = rd_ptr->dram_addr.row;
    if (! drain_writes[channel][rank][bank] &&
        rd_ptr->thread_id == thread_id &&
        is_activate_allowed(channel,rank,bank)) {
      issue_activate_command(channel,rank,bank,row);
      recent_colacc[channel][rank][bank] = 0;
      return 1;
    }
  }
  return 0;
}

int MemoryController::activate(int channel)
{
  request_t * rd_ptr = NULL;
  int rank, bank, row;
  LL_FOREACH(read_queue_head[channel],rd_ptr){
    rank = rd_ptr->dram_addr.rank;
    bank = rd_ptr->dram_addr.bank;
    row = rd_ptr->dram_addr.row;
    if (! drain_writes[channel][rank][bank] &&
        is_activate_allowed(channel,rank,bank)) {
      issue_activate_command(channel,rank,bank,row);
      recent_colacc[channel][rank][bank] = 0;
      return 1;
    }
  }

  return 0;
}

int MemoryController::write_rowhit(int channel)
{
  request_t * wr_ptr = NULL;
  int rank, bank, row;
  LL_FOREACH(write_queue_head[channel], wr_ptr) {
    rank = wr_ptr->dram_addr.rank;
    bank = wr_ptr->dram_addr.bank;
    row = wr_ptr->dram_addr.row;
    // issue write requests only when the bank is in write drain mode
    if (drain_writes[channel][rank][bank]) {
      if (wr_ptr->command_issuable &&
          dram_state[channel][rank][bank].active_row == row &&
          ! ignore_bank[rank][bank]) {
        if (wr_ptr->next_command == COL_WRITE_CMD) {
          recent_colacc[channel][rank][bank] = 1;
          lastrow[channel][rank][bank] = wr_ptr->dram_addr.row;
          activating_by_drain_writes[channel][rank][bank] = 0;
        }
        if (wr_ptr->next_command == ACT_CMD) { 
          if (channel_drain_writes[channel])
            wait_cycle_for_drain_writes[channel][rank][bank] = 0;
          if (wait_cycle_for_drain_writes[channel][rank][bank]) {
            wait_cycle_for_drain_writes[channel][rank][bank]--;
            ignore_bank[rank][bank] = 1;
            continue;
          }
          recent_colacc[channel][rank][bank] = 0;
          activating_by_drain_writes[channel][rank][bank] = -1;
        }
        if (wr_ptr->next_command == PRE_CMD) { 
          if (channel_drain_writes[channel])
            wait_cycle_for_drain_writes[channel][rank][bank] = 0;
          if (wait_cycle_for_drain_writes[channel][rank][bank]) {
            wait_cycle_for_drain_writes[channel][rank][bank]--;
            ignore_bank[rank][bank] = 1;
            continue;
          }
          recent_colacc[channel][rank][bank] = 0;
          activating_by_drain_writes[channel][rank][bank] = lastrow[channel][rank][bank];
        }
        issue_request_command(wr_ptr);
        return 1;
      }
    }
  }
  return 0;
}

int MemoryController::write(int channel)
{
  request_t * wr_ptr = NULL;
  int rank, bank;
  LL_FOREACH(write_queue_head[channel], wr_ptr) {
    rank = wr_ptr->dram_addr.rank;
    bank = wr_ptr->dram_addr.bank;
    // issue write requests only when the bank is in write drain mode
    if (drain_writes[channel][rank][bank]) {
      if (wr_ptr->command_issuable && ! ignore_bank[rank][bank]) {
        if (wr_ptr->next_command == COL_WRITE_CMD) {
          recent_colacc[channel][rank][bank] = 1;
          lastrow[channel][rank][bank] = wr_ptr->dram_addr.row;
          activating_by_drain_writes[channel][rank][bank] = 0;
        }
        if (wr_ptr->next_command == ACT_CMD) { 
          if (channel_drain_writes[channel])
            wait_cycle_for_drain_writes[channel][rank][bank] = 0;
          if (wait_cycle_for_drain_writes[channel][rank][bank]) {
            wait_cycle_for_drain_writes[channel][rank][bank]--;
            ignore_bank[rank][bank] = 1;
            continue;
          }
          recent_colacc[channel][rank][bank] = 0;
          activating_by_drain_writes[channel][rank][bank] = -1;
        }
        if (wr_ptr->next_command == PRE_CMD) { 
          if (channel_drain_writes[channel])
            wait_cycle_for_drain_writes[channel][rank][bank] = 0;
          if (wait_cycle_for_drain_writes[channel][rank][bank]) {
            wait_cycle_for_drain_writes[channel][rank][bank]--;
            ignore_bank[rank][bank] = 1;
            continue;
          }
          recent_colacc[channel][rank][bank] = 0;
          activating_by_drain_writes[channel][rank][bank] = lastrow[channel][rank][bank];
        }
        issue_request_command(wr_ptr);
        return 1;
      }
    }
  }
  return 0;
}

int MemoryController::precharge(int channel)
{
  int i, j;
  if(NUM_CPUS == 1 || NUM_CPUS == 2){
    int close_flag[param->NUM_RANKS][param->NUM_BANKS];
    int rank;
    int bank;
    request_t * rd_ptr = NULL;
    for(i = 0; i < param->NUM_RANKS; i++){
      for(j = 0; j < param->NUM_BANKS; j++){
        close_flag[i][j] = 1;
      }
    }
    LL_FOREACH(read_queue_head[channel],rd_ptr){
      rank = rd_ptr->dram_addr.rank;
      bank = rd_ptr->dram_addr.bank;
      for (i=0; i<param->NUM_RANKS; i++) {
        for (j=0; j<param->NUM_BANKS; j++) {  /* For all banks on the channel.. */
          if(dram_state[channel][rank][bank].active_row == rd_ptr->dram_addr.row && close_flag[rank][bank]){
            close_flag[rank][bank] = 0;
          }
        }
      }
    }
    for (i=0; i<param->NUM_RANKS; i++) {
      for (j=0; j<param->NUM_BANKS; j++) {  /* For all banks on the channel.. */
        if (recent_colacc[channel][i][j]) {  /* See if this bank is a candidate. */
          if (is_precharge_allowed(channel,i,j) && close_flag[i][j]) {  /* See if precharge is doable. */
            if (issue_precharge_command(channel,i,j)) {
              recent_colacc[channel][i][j] = 0;
            }
          }
        }
      }
    }
  }
  else {
    for (i=0; i<param->NUM_RANKS; i++) {
      for (j=0; j<param->NUM_BANKS; j++) {  /* For all banks on the channel.. */
        if (recent_colacc[channel][i][j]) {  /* See if this bank is a candidate. */
          if (is_precharge_allowed(channel,i,j)) {  /* See if precharge is doable. */
            if (issue_precharge_command(channel,i,j)) {
              recent_colacc[channel][i][j] = 0;
            }
          }
        }
      }
    }
  }
  return 0;
}	

/******************************************************************************/
void MemoryController::init_scheduler_vars()
{
  if(NUM_CPUS == 1){
    HI_WM = DRAM_WQ_SIZE - 1;
    LO_WM = HI_WM - 15;
  }
  else if(NUM_CPUS == 2 && param->NUM_CHANNELS == 1){
    HI_WM = DRAM_WQ_SIZE - 1;
    LO_WM = HI_WM - 20;
  }
  else if(NUM_CPUS == 2 && param->NUM_CHANNELS == 4){
    HI_WM = DRAM_WQ_SIZE - 1;
    LO_WM = HI_WM - 15;
  }
  else if(NUM_CPUS == 4 && param->NUM_CHANNELS == 1){
    HI_WM = DRAM_WQ_SIZE - 1;
    LO_WM = HI_WM - 20;
  }
  else if(NUM_CPUS == 4 && param->NUM_CHANNELS == 4){
    HI_WM = DRAM_WQ_SIZE - 1;
    LO_WM = HI_WM - 15;
  }
  else if(NUM_CPUS == 8 && param->NUM_CHANNELS == 4){
    HI_WM = DRAM_WQ_SIZE - 1;
    LO_WM = HI_WM - 15;
  }
  else if(NUM_CPUS == 16 && param->NUM_CHANNELS == 4){
    HI_WM = DRAM_WQ_SIZE - 5;
    LO_WM = HI_WM - 20;
  }
  else {
    HI_WM = DRAM_WQ_SIZE - 5;
    LO_WM = HI_WM - 20;
  }

  if(param->NUM_CHANNELS < NUM_CPUS){

    WAIT_CYCLE_DEF = 40;
  }
  else {
    WAIT_CYCLE_DEF = 100;
  }
  // initialize all scheduler variables here
  int i, j, k;
  for (i=0; i<param->NUM_CHANNELS; i++) {
    for (j=0; j<param->NUM_RANKS; j++) {
      refreshes[i][j]=0;
      for (k=0; k<param->NUM_BANKS; k++) {
        recent_colacc[i][j][k] = 0;
        lastrow[i][j][k] = 0xdeaddead;
        drain_writes[i][j][k] = 0;
        activating_by_drain_writes[i][j][k] = 0;
      }
    }
  }
    
  //Gainful Thread
  init_gainful_thread();
  return;
}

int MemoryController::refresh(int channel)
{

  //  if((read_queue_length[channel] == 0))// && (current_core_cycle[0]-cycle_last_aggressive_refresh > T_RFC))
  //  {
    // it's safe to try to do a refresh if there's no more pending work in this channel
    int i,j;
    int result = 0;
    int wait = 0;
    int ref = 0;
    for(i=0; i<param->NUM_RANKS; i++)
    {
      result = 1;
      wait = 100;
      for(j = 0; j < param->NUM_BANKS; j++){
        result &= drain_writes[channel][i][j];
        if(wait > wait_cycle_for_drain_writes[channel][i][j]){
          wait = wait_cycle_for_drain_writes[channel][i][j];
        }
      }
      if((is_refresh_allowed(channel,i)) && (refreshes[channel][i] < 8)  && result && (wait == 0))
      {
        issue_refresh_command(channel,i);
        refreshes[channel][i]++;
        //        printf("Issuing a refresh to c%d r%d at %lld\n",channel,i,current_core_cycle[0]);
        ref = 1;
      }
    }
    //  }
  return ref;  

}

int MemoryController::refresh2(int channel){

  int result = 0;
  int i;
  if(read_queue_length[channel] == 0){
    for(i=0; i<param->NUM_RANKS; i++) {
      if((is_refresh_allowed(channel,i)) && (refreshes[channel][i] < 8))
      {
        issue_refresh_command(channel,i);
        refreshes[channel][i]++;
        //        printf("Issuing a refresh to c%d r%d at %lld\n",channel,i,current_core_cycle[0]);
        result = 1;
      }
    }
  }
    return result;

}


/******************************************************************************/
void MemoryController::schedule(int channel)
{
  int gt;
  int i;
  if(current_core_cycle[0] % (8 * param->T_REFI) == 0){
    for (i=0;i<param->NUM_RANKS;i++) {
      refreshes[channel][i] = 0;
    }
  }

  
  if(channel == 0){
    set_gainful_thread();
  }
  set_drain_writes_mode(channel);

  if(NUM_CPUS == 1 && param->NUM_CHANNELS == 1){
    if(read_rowhit(channel)) return;
    if(read(channel)) return;
    if(write_rowhit(channel)) return;
    if(activate(channel)) return;
    if(write(channel)) return;
    if(precharge(channel)) return;
    if(refresh(channel)) return;
  }
  else if(NUM_CPUS == 2 && param->NUM_CHANNELS == 1){
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    init_gainful_thread_ptr(channel);
    for(gt=get_gainful_thread(channel); gt > -1; gt=get_gainful_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    if(read_rowhit(channel)) return;
    if(read(channel)) return;
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    init_gainful_thread_ptr(channel);
    for(gt=get_gainful_thread(channel); gt > -1; gt=get_gainful_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    if(activate(channel)) return;
    if(write_rowhit(channel)) return;
    if(write(channel)) return;
    if(precharge(channel)) return;
    if(refresh(channel)) return;
  }
  else if(NUM_CPUS > 2 && param->NUM_CHANNELS == 1){
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    init_gainful_thread_ptr(channel);
    for(gt=get_gainful_thread(channel); gt > -1; gt=get_gainful_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    if(read_rowhit(channel)) return;
    if(read(channel)) return;
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    init_gainful_thread_ptr(channel);
    for(gt=get_gainful_thread(channel); gt > -1; gt=get_gainful_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    if(activate(channel)) return;
    if(write_rowhit(channel)) return;
    if(write(channel)) return;
    if(precharge(channel)) return;
    if(refresh(channel)) return;
  }
  else if(NUM_CPUS <= 2 && param->NUM_CHANNELS == 4){
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    if(read_rowhit(channel)) return;
    if(read(channel)) return;
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    if(activate(channel)) return;
    if(write_rowhit(channel)) return;
    if(write(channel)) return;
    if(precharge(channel)) return;
    if(refresh(channel)) return;
  }
  else if(NUM_CPUS == 4 && param->NUM_CHANNELS == 4){
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    init_gainful_thread_ptr(channel);
    for(gt=get_gainful_thread(channel); gt > -1; gt=get_gainful_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    if(read_rowhit(channel)) return;
    if(read(channel)) return;
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    init_gainful_thread_ptr(channel);
    for(gt=get_gainful_thread(channel); gt > -1; gt=get_gainful_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    if(activate(channel)) return;
    if(write_rowhit(channel)) return;
    if(write(channel)) return;
    if(precharge(channel)) return;
    if(refresh2(channel)) return;
  }
  else if(NUM_CPUS > 4 && param->NUM_CHANNELS == 4){
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    init_gainful_thread_ptr(channel);
    for(gt=get_gainful_thread(channel); gt > -1; gt=get_gainful_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    if(read_rowhit(channel)) return;
    if(read(channel)) return;
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    init_gainful_thread_ptr(channel);
    for(gt=get_gainful_thread(channel); gt > -1; gt=get_gainful_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    if(activate(channel)) return;
    if(write_rowhit(channel)) return;
    if(write(channel)) return;
    if(precharge(channel)) return;
    //if(refresh2(channel)) return;
  } 
  else {
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    init_gainful_thread_ptr(channel);
    for(gt=get_gainful_thread(channel); gt > -1; gt=get_gainful_thread(channel)){
      if(read_rowhit_thread(channel, gt)) return;
      if(read_thread(channel, gt)) return;
    }
    if(read_rowhit(channel)) return;
    if(read(channel)) return;
    init_gainful_inst_thread_ptr(channel); //Inst
    for(gt=get_gainful_inst_thread(channel); gt > -1; gt=get_gainful_inst_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    init_gainful_thread_ptr(channel);
    for(gt=get_gainful_thread(channel); gt > -1; gt=get_gainful_thread(channel)){
      if(activate_thread(channel, gt)) return;
    }
    if(activate(channel)) return;
    if(write_rowhit(channel)) return;
    if(write(channel)) return;
    if(precharge(channel)) return;
    //    if(refresh2(channel)) return;
  }
 
}
void MemoryController::scheduler_stats()
{
}

#endif
