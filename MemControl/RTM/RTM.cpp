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

#include "MemControl/RTM/RTM.h"
#include "src/EventQueue.h"
#include "src/Params.h"
#include "include/NVMainRequest.h"
#include "src/SubArray.h"
#ifndef TRACE
#ifdef GEM5
  #include "SimInterface/Gem5Interface/Gem5Interface.h"
  #include "base/statistics.hh"
  #include "base/types.hh"
  #include "sim/core.hh"
  #include "sim/stat_control.hh"
#endif
#endif
#include <assert.h>
#include <iostream>
#include <set>

using namespace NVM;


RTM::RTM()
{
    std::cout << "Created a RTM memory controller!" << std::endl;

    //Cached Config Variables
    lazyPortUpdate = true;
    serialLayout = true;
    staticPortAccess = true;
    DOMAINS = 128;
    nPorts = 2;
    queueSize = 32;
    starvationThreshold = 4;
    wordSize = 4;
    
    //Stats
    averageLatency = 0.0f;
    averageQueueLatency = 0.0f;
    averageTotalLatency = 0.0f;
    measuredLatencies = 0;
    measuredQueueLatencies = 0;
    measuredTotalLatencies = 0;
    mem_reads = 0;
    mem_writes = 0;
    rb_hits = 0;
    rb_miss = 0;
    starvation_precharges = 0;
    write_pauses = 0;

    //No Idea what category
    psInterval = 0;

    InitQueues(1);
    memQueue = &(transactionQueues[0]);
}

RTM::~RTM()
{
    std::cout << "RTM memory controller destroyed. " << memQueue->size() 
              << " commands still in memory queue." << std::endl;
}

void RTM::SetConfig(Config* conf, bool createChildren)
{
    Params *params = new Params();
    params->SetParams(conf);
    SetParams(params);
  
    p->PortUpdate == "lazy" ? lazyPortUpdate = true : lazyPortUpdate = false;
    p->RTMLayout == "serial" ? serialLayout = true : serialLayout = false;
    p->PortAccess == "static" ? staticPortAccess = true : staticPortAccess = false;

    DOMAINS = p->DOMAINS;
    nPorts = p->nPorts;
    starvationThreshold = p->starvationThreshold;
    queueSize = p->queueSize;
    wordSize = p->wordSize;

    MemoryController::SetConfig(conf, createChildren);

    SetDebugName("RTM", conf);
}

void RTM::RegisterStats()
{
    AddStat(averageLatency);
    AddStat(averageQueueLatency);
    AddStat(averageTotalLatency);
    AddStat(measuredLatencies);
    AddStat(measuredQueueLatencies);
    AddStat(measuredTotalLatencies);
    AddStat(mem_reads);
    AddStat(mem_writes);
    AddStat(rb_hits);
    AddStat(rb_miss);
    AddStat(starvation_precharges);
    AddStat(write_pauses);

    MemoryController::RegisterStats();
}

bool RTM::IsIssuable(NVMainRequest* /*request*/, FailReason* /*fail*/)
{
    //Limit the number of commands in the queue. This will stall the caches/CPU.
    if (memQueue->size() >= queueSize)
    {
        return false;
    }

    return true;
}

/*
 * This method is called whenever a new transaction from the processor is issued to
 * this memory controller/channel. All scheduling decisions should be made here.
 */
bool RTM::IssueCommand(NVMainRequest* req)
{
    if (!IsIssuable(req))
    {
        return false;
    }

    req->arrivalCycle = GetEventQueue()->GetCurrentCycle();

    Enqueue(0, req);

    if (req->type == READ)
        mem_reads++;
    else
        mem_writes++;

    return true;
}

bool RTM::RequestComplete(NVMainRequest* request)
{
    if (request->type == WRITE || request->type == WRITE_PRECHARGE)
    {
        //Put cancelled requests at the head of the write queue like nothing ever happened
        if (request->flags & NVMainRequest::FLAG_CANCELLED || request->flags & NVMainRequest::FLAG_PAUSED)
        {
            Prequeue(0, request);

            return true;
        }
    }

    //Only reads and writes are sent back to NVMain and checked for in the transaction queue
    if (request->type == READ || request->type == READ_PRECHARGE 
        || request->type == WRITE || request->type == WRITE_PRECHARGE)
    {
        request->status = MEM_REQUEST_COMPLETE;
        request->completionCycle = GetEventQueue()->GetCurrentCycle();

        //Update the average latencies based on this request for READ/WRITE only
        averageLatency = ((averageLatency * static_cast<double>(measuredLatencies))
                           + static_cast<double>(request->completionCycle)
                           - static_cast<double>(request->issueCycle))
                       / static_cast<double>(measuredLatencies + 1);
        measuredLatencies += 1;

        averageQueueLatency = ((averageQueueLatency * static_cast<double>(measuredQueueLatencies))
                                + static_cast<double>(request->issueCycle)
                                - static_cast<double>(request->arrivalCycle))
                            / static_cast<double>(measuredQueueLatencies + 1);
        measuredQueueLatencies += 1;

        averageTotalLatency = ((averageTotalLatency * static_cast<double>(measuredTotalLatencies))
                                + static_cast<double>(request->completionCycle)
                                - static_cast<double>(request->arrivalCycle))
                            / static_cast<double>(measuredTotalLatencies + 1);
        measuredTotalLatencies += 1;
    }

    return MemoryController::RequestComplete(request);
}

void RTM::Cycle(ncycle_t steps)
{
    NVMainRequest *nextRequest = NULL;

    //Check for starved requests first
    if (FindStarvedRequest(*memQueue, &nextRequest))
    {
        rb_miss++;
        starvation_precharges++;
    }
    //Check for Rowbuffer hits
    else if (FindRTMRowBufferHit(*memQueue, &nextRequest))
    {
        rb_hits++;
    }
    //Check for Requests close to APs
    else if (FindCloseAPHit(*memQueue, &nextRequest))
    {
        rb_miss++;
    }
    //Check if the address is accessible through any other means
    else if (FindCachedAddress(*memQueue, &nextRequest)) {}
    else if (FindWriteStalledRead(*memQueue, &nextRequest))
    {
        if (nextRequest != NULL)
            write_pauses++;
    }
    //Find the oldest request that can be issued
    else if (FindOldestReadyRequest(*memQueue, &nextRequest))
    {
        rb_miss++;
    }
    //Find requests to a closed bank
    else if (FindClosedBankRequest(*memQueue, &nextRequest))
    {
        rb_miss++;
    }
    else
    {
        nextRequest = NULL;
    }

    //Issue the commands for this transaction
    if(nextRequest != NULL)
    {
        IssueMemoryCommands(nextRequest);
    }

    //Issue any commands in the command queues
    CycleCommandQueues();

    MemoryController::Cycle(steps);
}

void RTM::CalculateStats()
{
    MemoryController::CalculateStats();
}

NVMainRequest* RTM::MakeShiftRequest(NVMainRequest* triggerRequest)
{
    NVMainRequest* shiftRequest = new NVMainRequest();

    shiftRequest->type = SHIFT;
    shiftRequest->issueCycle = GetEventQueue()->GetCurrentCycle();
    shiftRequest->address = triggerRequest->address;
    shiftRequest->owner = this;

    if (triggerRequest->type == WRITE || triggerRequest->type == WRITE_PRECHARGE) 
        shiftRequest->flags = shiftRequest->flags | NVMainRequest::FLAG_WRITE_SHIFT;

    return shiftRequest;
}

//Copied to here from MemoryController.cpp
bool RTM::IssueMemoryCommands(NVMainRequest* req)
{
    bool rv = false;
    ncounter_t rank, bank, row, subarray, col;

    req->address.GetTranslatedAddress(&row, &col, &bank, &rank, NULL, &subarray);

    SubArray* writingArray = FindChild(req, SubArray);

    ncounter_t muxLevel = static_cast<ncounter_t>(col / p->RBSize);
    ncounter_t queueId = GetCommandQueueId(req->address);

    /*
     *  If the request is somehow accessible (e.g., via caching, etc), but the
     *  bank state does not match the memory controller, just issue the request
     *  without updating any internal states.
     */
    FailReason reason;
    NVMainRequest* cachedRequest = MakeCachedRequest(req);

    if (GetChild()->IsIssuable(cachedRequest, &reason))
    {
        /* Differentiate from row-buffer hits. */
        if (!activateQueued[rank][bank] 
             || !activeSubArray[rank][bank][subarray]
             || effectiveRow[rank][bank][subarray] != row 
             || effectiveMuxedRow[rank][bank][subarray] != muxLevel) 
        {
            req->issueCycle = GetEventQueue()->GetCurrentCycle();

            // Update starvation ??
            commandQueues[queueId].push_back(req);

            delete cachedRequest;

            return true;
        }
    }

    delete cachedRequest;

    if (!activateQueued[rank][bank] && commandQueues[queueId].empty())
    {
        /* Any activate will request the starvation counter */
        activateQueued[rank][bank] = true;
        activeSubArray[rank][bank][subarray] = true;
        effectiveRow[rank][bank][subarray] = row;
        effectiveMuxedRow[rank][bank][subarray] = muxLevel;
        starvationCounter[rank][bank][subarray] = 0;

        req->issueCycle = GetEventQueue()->GetCurrentCycle();

        NVMainRequest* actRequest = MakeActivateRequest(req);
        actRequest->flags |= (writingArray != NULL && writingArray->IsWriting()) ? NVMainRequest::FLAG_PRIORITY : 0;
        commandQueues[queueId].push_back(actRequest);

        /* Different row buffer management policy has different behavior */ 
        /*
         * There are two possibilities that the request is the last request:
         * 1) ClosePage == 1 and there is no other request having row
         * buffer hit
         * or 2) ClosePage == 2, the request is always the last request
         */
        if (req->flags & NVMainRequest::FLAG_LAST_REQUEST && p->UsePrecharge)
        {
            commandQueues[queueId].push_back(MakeImplicitPrechargeRequest(req));
            activeSubArray[rank][bank][subarray] = false;
            effectiveRow[rank][bank][subarray] = p->ROWS;
            effectiveMuxedRow[rank][bank][subarray] = p->ROWS;
            activateQueued[rank][bank] = false;
        }
        else
        {
            NVMainRequest *shiftRequest = MakeShiftRequest(req); //Place a shift request before the actual read/write on the command queue
            shiftRequest->flags |= (writingArray != NULL && writingArray->IsWriting()) ? NVMainRequest::FLAG_PRIORITY : 0;
            commandQueues[queueId].push_back(shiftRequest);

            commandQueues[queueId].push_back(req);
        }

        rv = true;
    }
    else if (activateQueued[rank][bank] 
            && (!activeSubArray[rank][bank][subarray] 
                || effectiveRow[rank][bank][subarray] != row 
                || effectiveMuxedRow[rank][bank][subarray] != muxLevel)
            && commandQueues[queueId].empty())
    {
        /* Any activate will request the starvation counter */
        starvationCounter[rank][bank][subarray] = 0;
        activateQueued[rank][bank] = true;

        req->issueCycle = GetEventQueue()->GetCurrentCycle();

        if (activeSubArray[rank][bank][subarray] && p->UsePrecharge)
        {
            commandQueues[queueId].push_back( 
                    MakePrechargeRequest(effectiveRow[rank][bank][subarray], 0, bank, rank, subarray));
        }

        NVMainRequest* actRequest = MakeActivateRequest(req);
        actRequest->flags |= (writingArray != NULL && writingArray->IsWriting()) ? NVMainRequest::FLAG_PRIORITY : 0;
        commandQueues[queueId].push_back(actRequest);

        NVMainRequest* shiftRequest = MakeShiftRequest(req); //Place a shift request before the actual read/write on the command queue
        shiftRequest->flags |= (writingArray != NULL && writingArray->IsWriting()) ? NVMainRequest::FLAG_PRIORITY : 0;
        commandQueues[queueId].push_back(shiftRequest);

        commandQueues[queueId].push_back(req);
        activeSubArray[rank][bank][subarray] = true;
        effectiveRow[rank][bank][subarray] = row;
        effectiveMuxedRow[rank][bank][subarray] = muxLevel;

        rv = true;
    }
    else if (activateQueued[rank][bank] 
            && activeSubArray[rank][bank][subarray]
            && effectiveRow[rank][bank][subarray] == row 
            && effectiveMuxedRow[rank][bank][subarray] == muxLevel)
    {
        starvationCounter[rank][bank][subarray]++;

        req->issueCycle = GetEventQueue()->GetCurrentCycle();

        /* Different row buffer management policy has different behavior */ 
        /*
         * There are two possibilities that the request is the last request:
         * 1) ClosePage == 1 and there is no other request having row
         * buffer hit
         * or 2) ClosePage == 2, the request is always the last request
         */
        if (req->flags & NVMainRequest::FLAG_LAST_REQUEST && p->UsePrecharge)
        {
            /* if Restricted Close-Page is applied, we should never be here */
            assert(p->ClosePage != 2);

            NVMainRequest* shiftRequest = MakeShiftRequest(req); //Place a shift request before the actual read/write on the command queue
            shiftRequest->flags |= (writingArray != NULL && writingArray->IsWriting()) ? NVMainRequest::FLAG_PRIORITY : 0;
            commandQueues[queueId].push_back(shiftRequest);

            commandQueues[queueId].push_back(MakeImplicitPrechargeRequest(req));
            activeSubArray[rank][bank][subarray] = false;
            effectiveRow[rank][bank][subarray] = p->ROWS;
            effectiveMuxedRow[rank][bank][subarray] = p->ROWS;

            bool idle = true;
            for (ncounter_t i = 0; i < subArrayNum; i++)
            {
                if (activeSubArray[rank][bank][i] == true)
                {
                    idle = false;
                    break;
                }
            }

            if (idle)
                activateQueued[rank][bank] = false;
        }
        else
        {
            NVMainRequest* shiftRequest = MakeShiftRequest(req); //Place a shift request before the actual read/write on the command queue
            shiftRequest->flags |= (writingArray != NULL && writingArray->IsWriting()) ? NVMainRequest::FLAG_PRIORITY : 0;
            commandQueues[queueId].push_back(shiftRequest);

            commandQueues[queueId].push_back(req);
        }

        rv = true;
    }
    else
    {
        rv = false;
    }

    /* Schedule wake event for memory commands if not scheduled. */
    if (rv == true)
    {
        ScheduleCommandWake();
    }

    return rv;
}

bool RTM::FindRTMRowBufferHit(std::list<NVMainRequest*>& transactionQueue, NVMainRequest** hitRequest)
{
    DummyPredicate pred;

    return FindRTMRowBufferHit(transactionQueue, hitRequest, pred);
}

bool RTM::FindRTMRowBufferHit(std::list<NVMainRequest*>& transactionQueue, NVMainRequest** hitRequest, SchedulingPredicate& pred)
{
    bool rv = false;
    std::list<NVMainRequest*>::iterator it;

    *hitRequest = NULL;

    for (it = transactionQueue.begin(); it != transactionQueue.end(); it++)
    {
        ncounter_t rank, bank, subarray, dbc, dom;
        ncounter_t queueId = GetCommandQueueId((*it)->address);

        if (!commandQueues[queueId].empty()) continue;

        (*it)->address.GetTranslatedAddress(&dbc, &dom, &bank, &rank, NULL, &subarray);

        //By design, mux level can only be a subset of the selected columns
        ncounter_t muxLevel = static_cast<ncounter_t>(dom / p->RBSize);

        if (activateQueued[rank][bank]                              // The bank is active
            && activeSubArray[rank][bank][subarray]                 // The subarray is open
            && effectiveRow[rank][bank][subarray] == dbc            // The effective row is the row of this request
            && effectiveMuxedRow[rank][bank][subarray] == muxLevel  // Subset of row buffer is currently at the sense amps
            && !bankNeedRefresh[rank][bank]                         // The bank is not waiting for a refresh
            && !refreshQueued[rank][bank]                           // Don't interrupt refreshes queued on bank group head
            && (*it)->arrivalCycle != GetEventQueue()->GetCurrentCycle()
            && commandQueues[queueId].empty()                       // The request queue is empty
            && pred((*it)))                                         // User-defined predicate is true
        {
            *hitRequest = (*it);
            transactionQueue.erase(it);

            if (IsLastRequest(transactionQueue, (*hitRequest)))
            {
                (*hitRequest)->flags |= NVMainRequest::FLAG_LAST_REQUEST;
            }

            rv = true;

            break;
        }
    }

    return rv;
}

bool RTM::FindCloseAPHit(std::list<NVMainRequest*>& transactionQueue, NVMainRequest** hitRequest)
{
    DummyPredicate pred;

    return FindCloseAPHit(transactionQueue, hitRequest, pred);
}

bool RTM::FindCloseAPHit(std::list<NVMainRequest*>& transactionQueue, NVMainRequest** hitRequest, SchedulingPredicate& pred)
{
    bool rv = false;
    std::list<NVMainRequest*>::iterator it;

    *hitRequest = NULL;

    for (it = transactionQueue.begin(); it != transactionQueue.end(); it++)
    {
        ncounter_t rank, bank, subarray, dbc, dom;
        ncounter_t queueId = GetCommandQueueId((*it)->address);

        if (!commandQueues[queueId].empty()) continue;

        (*it)->address.GetTranslatedAddress(&dbc, &dom, &bank, &rank, NULL, &subarray);

        if (activateQueued[rank][bank]                      // The bank is active
            && activeSubArray[rank][bank][subarray]         // The subarray is open
            && !bankNeedRefresh[rank][bank]                 // The bank is not waiting for a refresh
            && !refreshQueued[rank][bank]                   // Don't interrupt refreshes queued on bank group head
            && (*it)->arrivalCycle != GetEventQueue()->GetCurrentCycle()
            && commandQueues[queueId].empty()               // The request queue is empty
            && pred((*it)))                                 // User-defined predicate is true
        {
            *hitRequest = (*it);
            transactionQueue.erase(it);

            if (IsLastRequest(transactionQueue, (*hitRequest)))
            {
                (*hitRequest)->flags |= NVMainRequest::FLAG_LAST_REQUEST;
            }

            rv = true;

            break;
        }
    }

    return rv;
}