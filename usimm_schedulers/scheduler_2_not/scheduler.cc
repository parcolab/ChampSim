// -*- mode:c; indent-tabs-mode:nil; -*-

#include <stdio.h>
#include <stdlib.h>
#include "utlist.h"
#include "utils.h"

#include "memory_controller.h"
#include "params.h"

//extern long long int current_core_cycle[0];

/////////////////////////////////
// Design Parameter
/////////////////////////////////

#define STATE_READ           0
#define STATE_WRITE          1
#define STATE_BEFORE_REFRESH 2
#define STATE_REFRESH        3

#define MAX_NUM_CORES   16

#define MLP_THRESHOLD     2 
#define MLP_AGE_THRESHOLD 22

#define C2C_THRESHOLD     220
#define M2C_THRESHOLD     970
#define MAX_DISTANCE      13

#define WQ_FULL_THRES ((WQ_CAPACITY    ) - 2)
#define R2W_THRESHOLD ((WQ_CAPACITY*3/4)    )
#define W2R_THRESHOLD ((WQ_CAPACITY*2/4) - 6)
#define REF_THRESHOLD ((WQ_CAPACITY*1/4) + 2)

#define PRIORITY_THRESHOLD (100*1000)
#define TIMEOUT_THRESHOLD (1000*1000)

////////////////////////////////////////////////////////////////
// Storage of Memory Controller (Total Hardware Cost: 2469B)
////////////////////////////////////////////////////////////////

// This value is used by the incoming request only.
// The commit ratio is attached to the memory request
extern long long int *committed;

// This flag is attached to each read / write requests.
//
// Total hardware cost is
// READ :  3-bit x 160 (ROB size) x 16 (Core)    = 960B
// WRITE:  3-bit x  96 (WQ  size) x  4 (Channel) = 144B
//
// Each flag consumes 3-bit
typedef struct {
  int count;   // 1-bit row-hit status
  int level;   // 1-bit priority level
  int timeout; // 1-bit timeout flag
} request_attr_t ;


// This data structure is employed by each channel.
// Total hardware cost is 341.25B x 4 (Channel) = 1365B
// Each scoreboard consumes: 341.25B
struct scoreboard_t {
  // Main Status (6-bit)
  int state;      // 2-bit controller state
  int last_rank;  // 1-bit last accessed rank
  int last_bank;  // 3-bit last accessed bank

  // tFAW tracking (64.5B)
  struct FAW_track_t {
    long long int t[4]; // 64-bit x 4 values
    int ptr;            //  2-bit pointer
  } faw[MAX_NUM_RANKS]; // 2-entries => 64B + 4-bit

  // Core tracking (258B), this data is used for Compute-Phase Prediction
  struct CORE_track_t {
    int comp_phase;         // 1-bit, represents the corresponding thread affiliated with the compute-phase or not.
    long long int distance; // 64-bit
    long long int interval; // 64-bit
  } core[MAX_NUM_CORES];    // 16-cores => 258B
  
  // Refresh Queue (18B), this data is used for Writeback-Refresh Overlap
  struct refresh_state_t {
    long long int intv;      // 64-bit value, the cycles left before the next refresh deadline.
    int rank;                // 2-bit value, the next rank to be refreshed.
    int last;                // 1-bit value, the last refreshed rank.
    int count;               // 5-bit value, the refresh issued count in the current interval.
    int cont;                // 5-bit value, the delay time of the Elastic Refresh.
    int rest[MAX_NUM_RANKS]; // 32-bit x 2entry
  } refresh;
} scoreboard[MAX_NUM_CHANNELS];

///////////////////////////////////////////////////////////
// Attribute API
///////////////////////////////////////////////////////////

void set_req_attr(request_t *req, request_attr_t* attr) {
  assert(req->user_ptr == NULL);
  req->user_ptr = (void*)attr;
}

request_attr_t* get_req_attr(request_t *req) {
  request_attr_t *p = (request_attr_t*)(req->user_ptr);
  return p;
}

///////////////////////////////////////////////////////////
// Basic functions
///////////////////////////////////////////////////////////

int is_same_row(request_t *a, request_t *b) {
  return 
    (a->dram_addr.channel == b->dram_addr.channel) &&
    (a->dram_addr.rank    == b->dram_addr.rank   ) &&
    (a->dram_addr.bank    == b->dram_addr.bank   ) &&
    (a->dram_addr.row     == b->dram_addr.row    ) ;
}

/////////////////////////////////
// FAW tracking
/////////////////////////////////

// check how many requests are served in current tFAW constraint
int get_activate_hist(int channel, int rank, long long int cycle) {
  int i, r=0;
  for(i=0; i<4; i++)
    r += (cycle <= (scoreboard[channel].faw[rank].t[i]+T_FAW)) ? 1 : 0;
  return r;
}

// update tFAW constraint
void set_activate_hist(int channel, int rank) {
  int p = scoreboard[channel].faw[rank].ptr;
  scoreboard[channel].faw[rank].ptr = (p + 1) % 4;
  scoreboard[channel].faw[rank].t[p] = current_core_cycle[0];
}

//////////////////////////////////
// Compute-Phase Prediction
//////////////////////////////////

// check if target thread is computing phase or not.
int get_priority(int channel, int thread_id) {
  return scoreboard[channel].core[thread_id].comp_phase;
}

// Updates the running state based on the committed instruction counts.
// The condition of the state transition is as follows.
// Compute Phase -> Memory  Phase : When the distance counter exceeds the MAX_DISTANCE.
// Memory  Phase -> Compute Phase : When the interval counter exceeds the M2C_THRESHOLD.
void update_thread_priority(int channel) {
  request_t *req_ptr;
  struct CORE_track_t *c = &(scoreboard[channel].core[0]);

  LL_FOREACH(read_queue_head[channel],req_ptr) {
    // commit count is used by the incoming request only.
    if(req_ptr->user_ptr == NULL) {
      int thread_id = req_ptr->thread_id;

      if(c[thread_id].comp_phase == 1) {
        if(committed[thread_id] > c[thread_id].interval) {
          c[thread_id].distance = 0;
          c[thread_id].interval = committed[thread_id] + C2C_THRESHOLD;
          c[thread_id].comp_phase = 1;
        } else {
          c[thread_id].distance += 1;
          if(c[thread_id].distance > MAX_DISTANCE){
            c[thread_id].comp_phase = 0;
          }
        }
      } else {
        if(committed[thread_id] > c[thread_id].interval) {
          c[thread_id].distance = 0;
          c[thread_id].interval = committed[thread_id] + M2C_THRESHOLD;
          c[thread_id].comp_phase = 1;
        } else {
          c[thread_id].distance += 1;
          if(c[thread_id].distance > MAX_DISTANCE){
            c[thread_id].comp_phase = 0;
          }
        }
      }
    }
  }
}

//////////////////////////////////
// Refresh Control
//////////////////////////////////

// Updates the refresh deadline and the finish time of the refreshing rank.
void update_refresh(int channel) {
  int i;
  for(i=0; i<MAX_NUM_RANKS; i++) {
    if(scoreboard[channel].refresh.rest[i] >= PROCESSOR_CLK_MULTIPLIER) {
      scoreboard[channel].refresh.rest[i] -= PROCESSOR_CLK_MULTIPLIER;
    }
  }
  scoreboard[channel].refresh.intv -= PROCESSOR_CLK_MULTIPLIER;
  if(scoreboard[channel].refresh.intv == 0) {
    scoreboard[channel].refresh.count = 8 * NUM_RANKS;
    scoreboard[channel].refresh.intv = 8 * T_REFI;
    for(i=0; i<MAX_NUM_RANKS; i++) {
      scoreboard[channel].refresh.rest[i] = T_REFI;
    }
    scoreboard[channel].refresh.rank = 0;
    scoreboard[channel].state        = STATE_READ;
  }
}

void issue_refresh(int channel, int rank) {
  scoreboard[channel].refresh.last = rank;
  scoreboard[channel].refresh.rank = (rank + 1) % NUM_RANKS;
  scoreboard[channel].refresh.rest[rank] = 2*T_RFC;
  scoreboard[channel].refresh.count--;
  if(scoreboard[channel].refresh.count == 0) {
    scoreboard[channel].refresh.rank = -1;
  }
}

//////////////////////////////////
// Issue Requests
//////////////////////////////////

void issue_activate(int channel, int rank, int bank) {
  set_activate_hist(channel, rank);
}

void issue_precharge(int channel, int rank, int bank) {
  // Do nothing
}

void issue_write(request_t *req) {
  int channel = req->dram_addr.channel;
  int rank = req->dram_addr.rank;
  int bank = req->dram_addr.bank;
  scoreboard[channel].last_rank = rank;
  scoreboard[channel].last_bank = bank;

  // If no row hit access can be issued, then issue autoprecharge command.
  if(get_req_attr(req)->count == 0) {
    issue_autoprecharge(channel, rank, bank);
  }
}

void issue_read(request_t *req) {
  int channel = req->dram_addr.channel;
  int rank = req->dram_addr.rank;
  int bank = req->dram_addr.bank;
  scoreboard[channel].last_rank = rank;
  scoreboard[channel].last_bank = bank;
}

void init_request(request_t *req_ptr, request_t *queue) {
  request_t *tmp_ptr = NULL;
  request_attr_t *attr = NULL;
  attr = (request_attr_t*)malloc(sizeof(request_attr_t));
  attr->count = 0;
  attr->level = 0;
  attr->timeout = 0;

  LL_FOREACH(queue, tmp_ptr) {
    request_attr_t *tmp_attr = get_req_attr(tmp_ptr);
    if(tmp_attr != NULL) {
      if(is_same_row(req_ptr, tmp_ptr)) {
        tmp_attr->count = 1;
      }
    }
  }
  set_req_attr(req_ptr, attr);
}

void issue_request(request_t *req) {
  int channel = req->dram_addr.channel;
  int rank    = req->dram_addr.rank;
  int bank    = req->dram_addr.bank;

  assert(req->command_issuable);
  issue_request_command(req);

  switch(req->next_command) {
  case COL_WRITE_CMD:
    issue_write(req);
    break;
  case COL_READ_CMD:
    issue_read(req);
    break;
  case ACT_CMD:
    issue_activate(channel, rank, bank);
    break;
  case PRE_CMD:
    issue_precharge(channel, rank, bank);
    break;
  default:
    assert(0);
  }
}

//////////////////////////////////
// Scheduler Main Routine
//////////////////////////////////

void schedule(int channel)
{
  int i,j,rq=0,wq=0,drain_write;
  request_t * req_ptr = NULL;
  request_t * rdat_ptr = NULL;
  request_t * rdat_ptr_pri = NULL;
  request_t * ract_ptr_pri = NULL;
  request_t * rpre_ptr_pri = NULL;
  request_t * wact_ptr_pri[MAX_NUM_RANKS][MAX_NUM_BANKS];
  request_t * wpre_ptr_pri[MAX_NUM_RANKS][MAX_NUM_BANKS];
  request_t * wdat_ptr[MAX_NUM_RANKS][MAX_NUM_BANKS];
  request_t * wact_ptr[MAX_NUM_RANKS][MAX_NUM_BANKS];
  request_t * wpre_ptr[MAX_NUM_RANKS][MAX_NUM_BANKS];
  request_t * ract_ptr[MAX_NUM_RANKS][MAX_NUM_BANKS];
  request_t * rpre_ptr[MAX_NUM_RANKS][MAX_NUM_BANKS];
  int p[MAX_NUM_RANKS][MAX_NUM_BANKS];
  int r[MAX_NUM_RANKS][MAX_NUM_BANKS];
  int w[MAX_NUM_RANKS][MAX_NUM_BANKS];
  int activate_count;
  int priority_re[MAX_NUM_RANKS];
  int priority_we[MAX_NUM_RANKS];
  int priority_act[MAX_NUM_RANKS];
  int write_count[MAX_NUM_RANKS];
  int read_per_core[MAX_NUM_CORES];
  int drain_read       = 1;
  int refresh_stop     = -1;
  int refresh_issuable = 0;

  activate_count = 0;
  for(i=0; i<MAX_NUM_RANKS; i++) {
    priority_re[i] = 1;
    priority_we[i] = 1;
    priority_act[i] = 1;
    write_count[i] = 0;
    for(j=0; j<MAX_NUM_BANKS; j++) {
      r[i][j] = w[i][j] = p[i][j] = 0;
      wdat_ptr[i][j] = NULL;
      wact_ptr[i][j] = NULL;
      wpre_ptr[i][j] = NULL;
      wact_ptr_pri[i][j] = NULL;
      wpre_ptr_pri[i][j] = NULL;
      ract_ptr[i][j] = NULL;
      rpre_ptr[i][j] = NULL;
      activate_count += (dram_state[channel][i][j].state == ROW_ACTIVE);
    }
  }
  for(i=0; i<MAX_NUM_CORES; i++) {
    read_per_core[i] = 0;
  }

  ////////////////////////////////////////////////////////////////////
  // Updating Controller States
  ////////////////////////////////////////////////////////////////////

  // Update Thread Priority based on Compute-Phase Prediction
  update_thread_priority(channel);

  // Update Refresh Status
  update_refresh(channel);

  // Update Priority of Read Requests
  LL_FOREACH(read_queue_head[channel],req_ptr) {
    request_attr_t *attr = get_req_attr(req_ptr);

    // Setting In-coming requests.
    if(attr == NULL) {
      init_request(req_ptr, read_queue_head[channel]);
      attr = get_req_attr(req_ptr);
    }

    // Priority check
    if(!attr->level) {
      if(get_priority(channel, req_ptr->thread_id)) {
        attr->level = 1;
      }
    }

    // Promotion check
    if((current_core_cycle[0] - req_ptr->arrival_time) > PRIORITY_THRESHOLD) {
      attr->level = 1;
    }

    // Timeout check
    if((current_core_cycle[0] - req_ptr->arrival_time) > TIMEOUT_THRESHOLD) {
      attr->timeout = 1;
    }

    // Counts pending requests
    if(scoreboard[channel].refresh.rest[req_ptr->dram_addr.rank] < T_RFC) { rq++; }
    read_per_core[req_ptr->thread_id]++;
  }
  
  // Update Priority of Write Requests
  LL_FOREACH(write_queue_head[channel], req_ptr) {    
    request_attr_t *attr = get_req_attr(req_ptr);

    // Setting In-coming requests.
    if(attr == NULL) {
      init_request(req_ptr, write_queue_head[channel]);
    }
    
    // Counts pending requests
    write_count[req_ptr->dram_addr.rank]++;
  }

  // Checks the write queue depth.
  if(scoreboard[channel].refresh.rank >= 0)
    wq = write_count[scoreboard[channel].refresh.rank];
  else
    wq = write_queue_length[channel];
  
  
  ////////////////////////////////////////////////////////////////////
  // Controller State Transition.
  ////////////////////////////////////////////////////////////////////

  switch(scoreboard[channel].state) {
  case STATE_READ:
    if(write_queue_length[channel] > R2W_THRESHOLD) {
      int go_refresh = 0;
      if(scoreboard[channel].refresh.count > 0) {
        if (scoreboard[channel].refresh.rest[scoreboard[channel].refresh.last] == 0) {
          if (wq < (write_queue_length[channel] / 2)) {
            go_refresh = 1;
          }
        }
      }
      // if sufficient write requests are found in the write queue, then issue refresh command.
      if(go_refresh) {
        scoreboard[channel].state = STATE_BEFORE_REFRESH;
      } else {
        scoreboard[channel].state = STATE_WRITE;
      }
    }
    break;
  case STATE_WRITE:
    if(write_queue_length[channel] < W2R_THRESHOLD) {
      scoreboard[channel].state = STATE_READ;
    }
    break;
  case STATE_BEFORE_REFRESH:
    if(scoreboard[channel].refresh.rest[scoreboard[channel].refresh.last] > T_RFC) {
      scoreboard[channel].state = STATE_REFRESH;
    }
    break;
  case STATE_REFRESH:
    if(scoreboard[channel].refresh.rest[scoreboard[channel].refresh.last] < T_RFC ||
       write_count[scoreboard[channel].refresh.last] == 0 ||
       write_queue_length[channel] < REF_THRESHOLD) {
      if(write_queue_length[channel] > W2R_THRESHOLD) {
        scoreboard[channel].state = STATE_WRITE;
      } else {
        scoreboard[channel].state = STATE_READ;
      }
    }
    break;
  default:
    assert(0);
  }
  
  ////////////////////////////////////////////////////////////////////
  // Decode Controller State
  ////////////////////////////////////////////////////////////////////

  // If write_queue is full, write requests are prioritized by inhibiting to issue read request.
  switch(scoreboard[channel].state) {
  case STATE_READ:
    drain_read       = 1;
    drain_write      = 0;
    refresh_issuable =
      (scoreboard[channel].refresh.count > 0) &&
      (read_queue_length[channel] == 0);
    refresh_stop     = -1;
    break;
  case STATE_WRITE:
    drain_read       = (write_queue_length[channel] < WQ_FULL_THRES);
    drain_write      = 1;
    refresh_issuable =
      (scoreboard[channel].refresh.count > 0) &&
      (read_queue_length[channel] == 0) &&
      (wq < REF_THRESHOLD);
    refresh_stop     = -1;
    break;
  case STATE_BEFORE_REFRESH:
    drain_read       = (write_queue_length[channel] < WQ_FULL_THRES);
    drain_write      = 1;
    refresh_issuable = 1;
    refresh_stop     = scoreboard[channel].refresh.rank;
    break;
  case STATE_REFRESH:
    drain_read       = (write_queue_length[channel] < WQ_FULL_THRES);
    drain_write      = 1;
    refresh_issuable = 0;
    refresh_stop     = -1;
    break;
  default:
    assert(0);
  }
  
  if(refresh_stop < 0) {
    rq = read_queue_length[channel];
    wq = write_queue_length[channel];
  }

  ////////////////////////////////////////////////////////////////////
  // Issue Refresh Command
  ////////////////////////////////////////////////////////////////////

  if(refresh_issuable) {
    int rank = scoreboard[channel].refresh.rank;
    if(is_refresh_allowed(channel, rank)) {
      int limit = 1 +
        (NUM_RANKS * scoreboard[channel].refresh.intv) / T_REFI -
        scoreboard[channel].refresh.count;
      if( (scoreboard[channel].state == STATE_BEFORE_REFRESH) ||
          (scoreboard[channel].refresh.cont >= limit) ) {
        issue_refresh_command(channel, rank);
        issue_refresh(channel, rank);
        scoreboard[channel].refresh.cont = 0;
        return;
      }
      scoreboard[channel].refresh.cont++;
    } else {
      scoreboard[channel].refresh.cont=0;
    }
  } else {
    scoreboard[channel].refresh.cont=0;
  }

  ////////////////////////////////////////////////////////////////////
  // The special priority requests (TIMEOUT request or Very Low-MLP)
  ////////////////////////////////////////////////////////////////////

  if(drain_read) {
    int timeout_found =   0;
    int cand_num      = 100;
    int cand_priority =  -1;
    LL_FOREACH(read_queue_head[channel],req_ptr) {
      request_attr_t *attr = get_req_attr(req_ptr);

      // Prioritizes the timeout requests to avoid starvation.
      if(timeout_found == 0) {
        if(req_ptr->command_issuable && attr->timeout) {
          timeout_found = 1;
          rdat_ptr_pri = req_ptr;
        }
        if(req_ptr->command_issuable && (read_per_core[req_ptr->thread_id] < MLP_THRESHOLD) && (current_core_cycle[0]-req_ptr->arrival_time > MLP_AGE_THRESHOLD)) {
          if(attr->level > cand_priority) {
            if(read_per_core[req_ptr->thread_id] < cand_num) {
              rdat_ptr_pri  = req_ptr;
              cand_num      = read_per_core[req_ptr->thread_id];
              cand_priority = get_req_attr(req_ptr)->level;
            }
          }
        }
      }
    }
  }


  ////////////////////////////////////////////////////////////////////
  // Priority checking of read commands
  ////////////////////////////////////////////////////////////////////

  if(drain_read) {
    int priority_found = 0;
    LL_FOREACH(read_queue_head[channel],req_ptr) {
      int rank = req_ptr->dram_addr.rank;
      int bank = req_ptr->dram_addr.bank;
      if(rank != refresh_stop) {
        int priority = get_req_attr(req_ptr)->level;
        
        if(scoreboard[channel].state != STATE_REFRESH) {
          if(priority) {
            if(!req_ptr->request_served) {
              if(priority_found == 0) {
                // The value "next_read" and "next_act" in dram_state is referred for simplification.
                // It needs negligible hardware costs to emurate the dram state in memory controller,
                // and there is plenty of the room of the hardware badget in our scheduler.
                long long int t = dram_state[channel][rank][bank].next_read;
                long long int a = dram_state[channel][rank][bank].next_act ;
                priority_found = 1;
                switch(req_ptr->next_command) {
                case COL_READ_CMD:
                  for(i=0; i<MAX_NUM_RANKS; i++) {
                    if(i != rank) {
                      priority_re[i] = t >= (current_core_cycle[0] + T_DATA_TRANS + T_RTRS);
                      if(!drain_write) {
                        priority_we[i] = t >= (current_core_cycle[0] + T_DATA_TRANS + T_CWD + T_RTRS - T_CAS);
                      }
                    } else {
                      priority_re[i] = 1;
                      if(!drain_write) {
                        priority_we[i] = t >= (current_core_cycle[0] + T_DATA_TRANS + T_CWD + T_WTR);
                      }
                    }
                  }
                  break;
                case ACT_CMD:
                  priority_act[rank] = a > (current_core_cycle[0] + T_RRD);
                  if(get_activate_hist(channel,rank,a) == 3) {
                    priority_act[rank] = 0;
                  }
                case PRE_CMD:
                  break;
                default:
                  break;
                }
              }
            }
          }
        }
        
        switch(req_ptr->next_command) {
        case COL_READ_CMD:
          if(req_ptr->command_issuable) {
            if(priority) {
              if(rdat_ptr_pri == NULL) rdat_ptr_pri = req_ptr;
            }
            if(priority_re[req_ptr->dram_addr.rank]) {
              if(rdat_ptr == NULL) rdat_ptr = req_ptr;
            }
          }
          if(priority) p[rank][bank]++;
          r[rank][bank]++;
          break;
        case ACT_CMD:
          if(req_ptr->command_issuable) {
            if(priority) {
              if(ract_ptr_pri == NULL) ract_ptr_pri = req_ptr;
            }
            if(priority_act[rank]) {
              if(ract_ptr[rank][bank] == NULL) ract_ptr[rank][bank] = req_ptr;
            }
          }
          break;
        case PRE_CMD:
          if(req_ptr->command_issuable && !p[rank][bank]) {
            if(priority) {
              if(rpre_ptr_pri == NULL) rpre_ptr_pri = req_ptr;
            }
            if(rpre_ptr[rank][bank] == NULL) rpre_ptr[rank][bank] = req_ptr;
          }
          break;
        default:
          assert(0);
          break;
        }
      }
    }
  }
  
  ////////////////////////////////////////////////////////////////////
  // Priority checking of write commands
  ////////////////////////////////////////////////////////////////////

  LL_FOREACH(write_queue_head[channel], req_ptr) {    
    int priority = 0;
    int rank = req_ptr->dram_addr.rank;
    int bank = req_ptr->dram_addr.bank;
    int act_hist = get_activate_hist(channel,rank,current_core_cycle[0]);
    
    if(rank != refresh_stop) {
      int count = get_req_attr(req_ptr)->count;

      // Set priority based on the tFAW and the row hit rate.
      // If the request number in current tFAW window is more than 1,
      // the requests that will cause row-hit accesses are prioritized.
      if(drain_write) {
        priority = (count >= ((act_hist + 3) / 4));
      } else {
        priority = (count == 0);
      }
      
      switch(req_ptr->next_command) {
      case COL_WRITE_CMD:
        if(req_ptr->command_issuable) {
          if(wdat_ptr[rank][bank] == NULL)
            wdat_ptr[rank][bank] = req_ptr;
        }
        w[rank][bank]++;
        break;
      case ACT_CMD:
        if(req_ptr->command_issuable && priority_act[rank]) {
          if(wact_ptr[rank][bank] == NULL)
            wact_ptr[rank][bank] = req_ptr;
          if(priority && (wact_ptr_pri[rank][bank] == NULL))
            wact_ptr_pri[rank][bank] = req_ptr;
        }
        break;
      case PRE_CMD:
        if(req_ptr->command_issuable) {
          if(wpre_ptr[rank][bank] == NULL)
            wpre_ptr[rank][bank] = req_ptr;
          if(priority && (wpre_ptr_pri[rank][bank] == NULL))
            wpre_ptr_pri[rank][bank] = req_ptr;
        }
        break;
      default:
        assert(0);
        break;
      }
    }
  }
  
  //////////////////////////////////////////////////////////
  // Issues Read / Write Command
  //////////////////////////////////////////////////////////

  if(!drain_write) {
    if(rdat_ptr_pri) { issue_request(rdat_ptr_pri); return; }
    if(ract_ptr_pri) { issue_request(ract_ptr_pri); return; }
    if(rpre_ptr_pri) { issue_request(rpre_ptr_pri); return; }
  }

  for(i=0; i<MAX_NUM_RANKS; i++) {
    int rank = (scoreboard[channel].last_rank + i) % MAX_NUM_RANKS;
    if(scoreboard[channel].refresh.rank >= 0) {
      rank = (scoreboard[channel].refresh.rank + i) % MAX_NUM_RANKS;
    }
    if(priority_we[rank]) {
      for(j=0; j<MAX_NUM_BANKS; j++) {
        int bank = (scoreboard[channel].last_bank + j) % MAX_NUM_BANKS;
        if(wdat_ptr[rank][bank]) {
          issue_request(wdat_ptr[rank][bank]);
          return;
        }
      }
    }
  }

  if(rdat_ptr_pri) { issue_request(rdat_ptr_pri); return; }

  if(rdat_ptr) {
    int rank = rdat_ptr->dram_addr.rank;
    int bank = rdat_ptr->dram_addr.bank;
    issue_request(rdat_ptr);
    if(rpre_ptr_pri && !drain_write) {
      if ( ( (get_req_attr(rdat_ptr)->count == 0) &&
             rpre_ptr_pri &&
             (bank == rpre_ptr_pri->dram_addr.bank) &&
             (rank == rpre_ptr_pri->dram_addr.rank) ) ) {
        issue_autoprecharge(channel, rank, bank);
      }
    }
    return;
  }
  
  if(ract_ptr_pri) { issue_request(ract_ptr_pri); return; }
  if(rpre_ptr_pri) { issue_request(rpre_ptr_pri); return; }
  
  if(!drain_write) {
    for(i=0; i<MAX_NUM_RANKS; i++) {
      int rank = (scoreboard[channel].last_rank + i) % MAX_NUM_RANKS;
      if(scoreboard[channel].refresh.rank >= 0) {
        rank = (scoreboard[channel].refresh.rank + i) % MAX_NUM_RANKS;
      }
      for(j=0; j<MAX_NUM_BANKS; j++) {
        int bank = (scoreboard[channel].last_bank + j) % MAX_NUM_BANKS;
        if(ract_ptr[rank][bank]) {
          issue_request(ract_ptr[rank][bank]);
          return;
        }
        if(rpre_ptr[rank][bank]) {
          if(!(r[rank][bank] || w[rank][bank])) {
            issue_request(rpre_ptr[rank][bank]);
            return;
          }
        } 
      }
    }
  }

  int issue_wact = (rq == 0) && (activate_count <= log_base2(write_queue_length[channel]+1));
  if(drain_write || issue_wact) {
    for(i=0; i<MAX_NUM_RANKS; i++) {
      int rank = (scoreboard[channel].last_rank + i) % MAX_NUM_RANKS;
      if(scoreboard[channel].refresh.rank >= 0) {
        rank = (scoreboard[channel].refresh.rank + i) % MAX_NUM_RANKS;
      }
      for(j=0; j<MAX_NUM_BANKS; j++) {
        int bank = (scoreboard[channel].last_bank + j) % MAX_NUM_BANKS;
        if(wact_ptr_pri[rank][bank]) {
          issue_request(wact_ptr_pri[rank][bank]);
          return;
        }
        if(wpre_ptr_pri[rank][bank]) {
          if(!(r[rank][bank] || w[rank][bank])) {
            issue_request(wpre_ptr_pri[rank][bank]);
            return;
          }
        } 
      }
    }

    for(i=0; i<MAX_NUM_RANKS; i++) {
      int rank = (scoreboard[channel].last_rank + i) % MAX_NUM_RANKS;
      if(scoreboard[channel].refresh.rank >= 0) {
        rank = (scoreboard[channel].refresh.rank + i) % MAX_NUM_RANKS;
      }
      for(j=0; j<MAX_NUM_BANKS; j++) {
        int bank = (scoreboard[channel].last_bank + j) % MAX_NUM_BANKS;
        if(wact_ptr[rank][bank]) {
          issue_request(wact_ptr[rank][bank]);
          return;
        }
        if(wpre_ptr[rank][bank]) {
          if(!(r[rank][bank] || w[rank][bank])) {
            issue_request(wpre_ptr[rank][bank]);
            return;
          }
        } 
      }
    }
  }


  //////////////////////////////////////////////////////////
  // Aggressive Precharge
  //////////////////////////////////////////////////////////

  for(i=0; i<MAX_NUM_RANKS; i++) {
    int rank = (scoreboard[channel].last_rank + i) % MAX_NUM_RANKS;
    if(scoreboard[channel].refresh.rank >= 0) {
      rank = (scoreboard[channel].refresh.rank + i) % MAX_NUM_RANKS;
    }
    if(rank != refresh_stop) {
      for(j=0; j<MAX_NUM_BANKS; j++) {
        // In the case the activation cannot be issued by the tFAW constraint,
        // it is not so useful to issue aggressive precharge.
        // For this reason, aggressive precharge is avoided when
        // the number of the requests in the current tFAW window exceeds 2.
        int bank = (scoreboard[channel].last_bank + j) % MAX_NUM_BANKS;
        int chit = dram_state[channel][rank][bank].state == ROW_ACTIVE;
        int act = get_activate_hist(channel,rank,current_core_cycle[0]+T_RP) < 3; // change
        if(chit && act && !(r[rank][bank] || w[rank][bank])) {
          if(is_precharge_allowed(channel,rank,bank) ) {
            issue_precharge_command(channel,rank,bank);
            issue_precharge(channel,rank,bank);
            return;
          }
        }
      }
    }
  }
}

void init_scheduler_vars() {
  int i,j,k;

  for(i=0; i<MAX_NUM_CHANNELS; i++) {
    scoreboard[i].state = STATE_READ;
    scoreboard[i].last_rank = 0;
    scoreboard[i].last_bank = 0;
  }

  for(i=0; i<MAX_NUM_CHANNELS; i++) {
    scoreboard[i].refresh.count = 8 * NUM_RANKS;
    scoreboard[i].refresh.intv = 8 * T_REFI;
    scoreboard[i].refresh.rank = 0;
    scoreboard[i].refresh.last = 1;
    scoreboard[i].refresh.cont = 0;
    for(j=0; j<MAX_NUM_RANKS; j++) {
      scoreboard[i].refresh.rest[j] = T_REFI;
    }
  }

  for(i=0; i<MAX_NUM_CHANNELS; i++) {
    for(j=0; j<MAX_NUM_RANKS; j++) {
      scoreboard[i].faw[j].ptr  = 0;
      for(k=0; k<4; k++) {
        scoreboard[i].faw[j].t[k] = i;
      }
    }
  }

  for (i=0; i<MAX_NUM_CHANNELS; i++) {
    for (j=0; j<MAX_NUM_CORES; j++) {
      scoreboard[i].core[j].comp_phase = 0;
      scoreboard[i].core[j].distance = 0;
      scoreboard[i].core[j].interval = 0;
    }
  }
}

void scheduler_stats() { }
