/*******************************************************************************
* Copyright (c) 2019-2022, Chair for Compiler Construction
* Department of Computer Science, TU Dresden
* All rights reserved.
* 
* This source code is part of NVMain - A cycle accurate timing, bit accurate
* energy simulator for both volatile (e.g., DRAM) and non-volatile memory
* (e.g., PCRAM). 
* 
* The original NVMain doesn't support simulating RaceTrack memory.
* This current version, which we call RTSim, enables RTM simulation. 
* 
* The source code is free and you can redistribute and/or
* modify it by providing that the following conditions are met:
* 
*  1) Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
* 
*  2) Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
* Author list: 
*
*   Asif Ali Khan   ( Email: asif_ali.khan@tu-dresden.de )
* 
*******************************************************************************/

#ifndef __RTM_H__
#define __RTM_H__

#include "src/MemoryController.h"
#include <deque>

namespace NVM {

class RTM : public MemoryController
{
  public:
    RTM();
    ~RTM();

    void Cycle(ncycle_t steps);

    bool IsIssuable(NVMainRequest* request, FailReason* fail = NULL);
    bool IssueCommand(NVMainRequest* req);
    bool RequestComplete(NVMainRequest* request);

    void CalculateStats();
    void RegisterStats();
    void SetConfig(Config* conf, bool createChildren = true);
    
  protected:
    //Shorthand for the only TransactionQ we use
    NVMTransactionQueue* memQueue;

    //Cached Configuration Variables
    bool lazyPortUpdate;
    bool serialLayout;
    bool staticPortAccess;
    ncounter_t DOMAINS;
    ncounter_t nPorts;
    ncounter_t queueSize;
    ncounter_t wordSize;

    //Stats
    double averageLatency;
    double averageQueueLatency;
    double averageTotalLatency;
    ncounter_t measuredLatencies;
    ncounter_t measuredQueueLatencies;
    ncounter_t measuredTotalLatencies;    
    ncounter_t mem_reads;
    ncounter_t mem_writes;
    ncounter_t rb_hits;
    ncounter_t rb_miss;
    ncounter_t starvation_precharges;
    ncounter_t write_pauses;

    NVMainRequest* MakeShiftRequest(NVMainRequest* triggerRequest);

    bool IssueMemoryCommands(NVMainRequest* req);

    //All Find...() have yet to be implemented
    bool FindCloseAPHit(std::list<NVMainRequest*>& transactionQueue, NVMainRequest** hitRequest);
    bool FindCloseAPHit(std::list<NVMainRequest*>& transactionQueue, NVMainRequest** hitRequest, NVM::SchedulingPredicate& p);
    bool FindRTMRowBufferHit(std::list<NVMainRequest*>& transactionQueue, NVMainRequest** hitRequest);
    bool FindRTMRowBufferHit(std::list<NVMainRequest*>& transactionQueue, NVMainRequest** hitRequest, NVM::SchedulingPredicate& p);
};

};

#endif