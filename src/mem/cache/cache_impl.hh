/*
 * Copyright (c) 2010-2013 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Erik Hallnor
 *          Dave Greene
 *          Nathan Binkert
 *          Steve Reinhardt
 *          Ron Dreslinski
 *          Andreas Sandberg
 */

/**
 * @file
 * Cache definitions.
 */
#include <climits>
#include <fstream>
#include <math.h>

#include "base/misc.hh"
#include "base/types.hh"
#include "debug/CheckAddr.hh"
#include "debug/Cache.hh"
#include "debug/LargeBlock.hh"
#include "debug/ExpiredBlock.hh"
#include "debug/CachePort.hh"
#include "debug/CacheTags.hh"
#include "debug/DeadStat.hh"
#include "debug/SttCache.hh"
#include "debug/TestData.hh"
#include "debug/TestPacket.hh"
#include "debug/ALT0.hh"
#include "debug/ALT1.hh"
#include "debug/ALT2.hh"
#include "debug/ALT3.hh"
#include "debug/TestTrans.hh"
#include "mem/cache/prefetch/base.hh"
#include "mem/cache/blk.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/mshr.hh"
#include "mem/cache/sampler.cc"
#include "mem/cache/monitor.cc"
#include "sim/sim_exit.hh"
#include "mem/cache/tags/lrustt.hh"

template<class TagStore>
Cache<TagStore>::Cache(const Params *p)
    : BaseCache(p),
      tags(dynamic_cast<TagStore*>(p->tags)),
      //stt_tags(dynamic_cast<TagStore*>(p->stt_tags)),
      prefetcher(p->prefetcher),
      doFastWrites(true),
      prefetchOnAccess(p->prefetch_on_access),
      handleExpiredEvent(this),
      handleRefreshEvent(this),
      predBlkSizeEvent(this)
{
    tempBlock = new BlkType();
    tempBlock->data = new uint8_t[blkSize];
    stt_tags = dynamic_cast<LRUSTT*>(p->stt_tags);

    cpuSidePort = new CpuSidePort(p->name + ".cpu_side", this,
                                  "CpuSidePort");
    memSidePort = new MemSidePort(p->name + ".mem_side", this,
                                  "MemSidePort");
    assert(tags);
    //assert(tags2);
    assert(stt_tags != NULL);
    //set the sampler
    if(alt_mech == 3) {
        online_sampler = new sampler(p->assoc, p->size / (64 * p->assoc * 64), p->pred_num);
    }
    else {
        online_sampler = NULL;
    }

    //set the monitor(ADT + Lscore), 4 in total(64, 128, 256, 512)
    if(isBottomLevel && alt_mech == 0) {
        online_monitor = new monitor*[4];
        for(int k = 0; k < 4; k++) {
            online_monitor[k] = new monitor(p->assoc, p->size / (64 * p->assoc), 64 * ((int)(pow(2,k))), 64);
        }
    }
    else {
        online_monitor = NULL;
    }
    tags->setCache(this);
    printf("bottom %d,tags set cache successfully, blk size %d\n",isBottomLevel,tags->getBlockSize());
    stt_tags->setCache(this);
    printf("bottom %d,stt_tags set cache successfully, blk size %d\n",isBottomLevel,stt_tags->getBlockSize());
    if (prefetcher)
        prefetcher->setCache(this);
}

template<class TagStore>
Cache<TagStore>::~Cache()
{
    delete [] tempBlock->data;
    delete tempBlock;

    delete cpuSidePort;
    delete memSidePort;
}

template<class TagStore>
void
Cache<TagStore>::regStats()
{
    BaseCache::regStats();
}

template<class TagStore>
void
Cache<TagStore>::cmpAndSwap(BlkType *blk, PacketPtr pkt)
{
    uint64_t overwrite_val;
    bool overwrite_mem;
    uint64_t condition_val64;
    uint32_t condition_val32;

    int offset = tags->extractBlkOffset(pkt->getAddr());
    uint8_t *blk_data = blk->data + offset;

    assert(sizeof(uint64_t) >= pkt->getSize());

    overwrite_mem = true;
    // keep a copy of our possible write value, and copy what is at the
    // memory address into the packet
    pkt->writeData((uint8_t *)&overwrite_val);
    pkt->setData(blk_data);

    if (pkt->req->isCondSwap()) {
        if (pkt->getSize() == sizeof(uint64_t)) {
            condition_val64 = pkt->req->getExtraData();
            overwrite_mem = !std::memcmp(&condition_val64, blk_data,
                                         sizeof(uint64_t));
        } else if (pkt->getSize() == sizeof(uint32_t)) {
            condition_val32 = (uint32_t)pkt->req->getExtraData();
            overwrite_mem = !std::memcmp(&condition_val32, blk_data,
                                         sizeof(uint32_t));
        } else
            panic("Invalid size for conditional read/write\n");
    }

    if (overwrite_mem) {
        std::memcpy(blk_data, &overwrite_val, pkt->getSize());
        blk->status |= BlkDirty;
    }
}


template<class TagStore>
void
Cache<TagStore>::satisfyCpuSideRequest(PacketPtr pkt, BlkType *blk,
                                       bool deferred_response,
                                       bool pending_downgrade)
{
    assert(blk && blk->isValid());
    // Occasionally this is not true... if we are a lower-level cache
    // satisfying a string of Read and ReadEx requests from
    // upper-level caches, a Read will mark the block as shared but we
    // can satisfy a following ReadEx anyway since we can rely on the
    // Read requester(s) to have buffered the ReadEx snoop and to
    // invalidate their blocks after receiving them.
    // assert(!pkt->needsExclusive() || blk->isWritable());
    assert(pkt->getOffset(blkSize) + pkt->getSize() <= blkSize);

    // Check RMW operations first since both isRead() and
    // isWrite() will be true for them
    if (pkt->cmd == MemCmd::SwapReq) {
        cmpAndSwap(blk, pkt);
    } else if (pkt->isWrite()) {
        if (blk->checkWrite(pkt)) {
			//DPRINTF(DeadStat,"dead value, blk source %d, ref count %d\n", blk->blkSource, blk->refCount);
			//if(blk->blkSource == BlockFill && blk->refCount <= 1)
				//num_dead_value++;
            pkt->writeDataToBlock(blk->data, blkSize);
            blk->status |= BlkDirty;
        }
    } else if (pkt->isRead()) {
        if (pkt->isLLSC()) {
            blk->trackLoadLocked(pkt);
        }
        pkt->setDataFromBlock(blk->data, blkSize);
        if (pkt->getSize() == blkSize) {
            // special handling for coherent block requests from
            // upper-level caches
            if (pkt->needsExclusive()) {
                // if we have a dirty copy, make sure the recipient
                // keeps it marked dirty
                if (blk->isDirty()) {
                    pkt->assertMemInhibit();
                }
                // on ReadExReq we give up our copy unconditionally
                assert(blk != tempBlock);
		if(!isBottomLevel || blk->sourceTag == 0)
                    tags->invalidate(blk);
                else    stt_tags->invalidate(blk);
                blk->invalidate();
                if(alt_mech == 1) {
                    tags->printSet(tags->regenerateBlkAddr(blk->tag,blk->set));
                }
            } else if (blk->isWritable() && !pending_downgrade
                      && !pkt->sharedAsserted() && !pkt->req->isInstFetch()) {
                // we can give the requester an exclusive copy (by not
                // asserting shared line) on a read request if:
                // - we have an exclusive copy at this level (& below)
                // - we don't have a pending snoop from below
                //   signaling another read request
                // - no other cache above has a copy (otherwise it
                //   would have asseretd shared line on request)
                // - we are not satisfying an instruction fetch (this
                //   prevents dirty data in the i-cache)

                if (blk->isDirty()) {
                    // special considerations if we're owner:
                    if (!deferred_response && !isTopLevel) {
                        // if we are responding immediately and can
                        // signal that we're transferring ownership
                        // along with exclusivity, do so
                        pkt->assertMemInhibit();
                        blk->status &= ~BlkDirty;
                    } else {
                        // if we're responding after our own miss,
                        // there's a window where the recipient didn't
                        // know it was getting ownership and may not
                        // have responded to snoops correctly, so we
                        // can't pass off ownership *or* exclusivity
                        pkt->assertShared();
                    }
                }
            } else {
                // otherwise only respond with a shared copy
                pkt->assertShared();
            }
        }
    } else {
        // Not a read or write... must be an upgrade.  it's OK
        // to just ack those as long as we have an exclusive
        // copy at this level.
        assert(pkt->isUpgrade());
        assert(blk != tempBlock);
        if(!isBottomLevel || blk->sourceTag == 0)    tags->invalidate(blk);
        else    stt_tags->invalidate(blk);
        blk->invalidate();
    }
}


/////////////////////////////////////////////////////
//
// MSHR helper functions
//
/////////////////////////////////////////////////////


template<class TagStore>
void
Cache<TagStore>::markInService(MSHR *mshr, PacketPtr pkt)
{
    markInServiceInternal(mshr, pkt);
#if 0
        if (mshr->originalCmd == MemCmd::HardPFReq) {
            DPRINTF(HWPrefetch, "%s:Marking a HW_PF in service\n",
                    name());
            //Also clear pending if need be
            if (!prefetcher->havePending())
            {
                deassertMemSideBusRequest(Request_PF);
            }
        }
#endif
}


template<class TagStore>
void
Cache<TagStore>::squash(int threadNum)
{
    bool unblock = false;
    BlockedCause cause = NUM_BLOCKED_CAUSES;

    if (noTargetMSHR && noTargetMSHR->threadNum == threadNum) {
        noTargetMSHR = NULL;
        unblock = true;
        cause = Blocked_NoTargets;
    }
    if (mshrQueue.isFull()) {
        unblock = true;
        cause = Blocked_NoMSHRs;
    }
    mshrQueue.squash(threadNum);
    if (unblock && !mshrQueue.isFull()) {
        clearBlocked(cause);
    }
}

/////////////////////////////////////////////////////
//
// Access path: requests coming in from the CPU side
//
/////////////////////////////////////////////////////

template<class TagStore>
bool
Cache<TagStore>::access(PacketPtr pkt, BlkType *&blk,
                        Cycles &lat, PacketList &writebacks)
{
    //DPRINTF(DeadStat,"enter into access function\n");
    //result = false;
    DPRINTF(Cache, "%s for %s address %x original address %x size %d\n", 
           __func__, pkt->cmdString(), pkt->getAddr(), 
           pkt->getOriAddr(), pkt->getSize());
    if(isBottomLevel && blockAlign(pkt->getAddr())==0x8f9c0) {
        DPRINTF(SttCache, "%s for %s address 0x8f9c0 pkt addr %x, size %d\n", __func__,  pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    }
    //Qi: collect locality information
    if(isLLC && localityMode) {
        FILE* fptr = fopen("locality.txt","a");
        //printf("%lx\n",pkt->getOriAddr());
        fprintf(fptr,"%lx\n",pkt->getOriAddr());
        fclose(fptr);
    }
    //cout << pkt->cmdString() << endl;
    if (pkt->cmd == MemCmd::Writeback)
        num_actual_write_backs++;
    if (pkt->req->isUncacheable()) {
        uncacheableFlush(pkt);
        blk = NULL;
        lat = hitLatency;
	if(pkt->isWrite())  lat = writeLatency;
        return false;
    }

    if(pkt->isWrite())  lat = writeLatency;

    int id = pkt->req->hasContextId() ? pkt->req->contextId() : -1;

    //Qi: only used for ALT0, update monitor
    if(alt_mech == 0 && isBottomLevel) {
        for(int k = 0; k < 4; k++) {
            online_monitor[k]->updateMonitor(pkt);
        }
        if(init_Lscore == 0) {
            DPRINTF(ALT0,"Try schedule print Lscore\n");
            schedule(predBlkSizeEvent, clockEdge() + 100000000);
            init_Lscore = 1;
        }
    }

    //Qi: only used for ALT3, update pred table and get the pred result
    bool bypass_decision = false;
    if(alt_mech == 3 && isLLC) {
        online_sampler->updateSampler(pkt,tags->extractSet(pkt->getAddr()),tags->extractTag(pkt->getAddr()));
        bypass_decision = online_sampler->getPredDecision(pkt);
        DPRINTF(ALT3,"addr %x, update sampler and get bypass decision %d\n",pkt->getAddr(),bypass_decision);
    }


    Addr align_addr = eDRAM_blkAlign(pkt->getAddr());
    int num_sub_block = int(eDRAMblkSize / blkSize);
    DPRINTF(Cache,"align addr %x, # sub blocks %d\n",align_addr,num_sub_block);
    if(!isBottomLevel) {
        blk = tags->accessBlock(pkt->getAddr(), lat, id);
    }
    else {
	blk = tags->eDRAM_accessBlock(align_addr, pkt->getAddr(), lat, id,
					 num_sub_block);
    }

    if(blk != NULL) {
        blk->setBlkSourceTag(0);
        assert(!stt_tags->findBlock(pkt->getAddr()));
    }

    if(isBottomLevel && blk == NULL) {
        if(pkt->isWrite()) {
            lat = stt_writeLatency;
        }
        else {
            lat = stt_readLatency;
        }
        blk = stt_tags->accessBlock(pkt->getAddr(), lat, id);
	if(blk != NULL) {
            blk->setBlkSourceTag(1);
            DPRINTF(TestData,"%s for addr %x hit in stt cache\n",pkt->cmdString(),pkt->getAddr());
            for(int i = 0; i < blkSize; i++) {
                DPRINTF(TestData,"%x",(unsigned int)*(blk->data+i));
                if(i == blkSize - 1)
                    DPRINTF(TestData,"\n");
            }
        }
        else {
            if(pkt->isWrite())    lat = hitLatency;
            else    lat = writeLatency;
        }
    }
    
    if(isBottomLevel && blk != NULL) {
        DPRINTF(SttCache,"Find Block addr %x in %s\n",pkt->getAddr(),(blk->sourceTag)?"STT RAM":"EDRAM");
    }

    DPRINTF(Cache, "%s%s %x %s %s\n", pkt->cmdString(),
            pkt->req->isInstFetch() ? " (ifetch)" : "",
            pkt->getAddr(), blk ? "hit" : "miss", blk ? blk->print() : "");

    if(blockAlign(pkt->getAddr()) == 0x8f9c0 && isBottomLevel) {
        DPRINTF(SttCache, "%s%s %x %s %s\n", pkt->cmdString(),
            pkt->req->isInstFetch() ? " (ifetch)" : "",
            pkt->getAddr(), blk ? "hit" : "miss", blk ? blk->print() : "");

    }

	/*Qi:Hit*/
    if (blk != NULL) {
        if(blk->blkSource == BlockFill && blk->refCount <= 1 && pkt->cmd == MemCmd::Writeback)
	    num_dead_value++;
    }

    if (blk != NULL) {

        if (pkt->needsExclusive() ? blk->isWritable() : blk->isReadable()) {
            // OK to satisfy access
            //Qi: reassign the expire time, collect some stats
            if(isLLC && !isBottomLevel) {
                sttRamHit++;
                sttRamAccesses++;
            }
            if(isBottomLevel) {
                DPRINTF(SttCache,"clockEdge %d, expiredPeriod %d, total %d\n",clockEdge(),expiredPeriod, clockEdge()+expiredPeriod);
                if(blk->sourceTag == 0)    sttRamHit++;
                else    edRamHit++;
                sttRamAccesses++;
                if(blk->expired_count <= clockEdge() + expiredPeriod) {
                    assert(clockEdge() + expiredPeriod == blk->expired_count);
                }
                assert(clockEdge()+expiredPeriod <= blk->expired_count);
            }
            if(isBottomLevel && blk->sourceTag == 0 && (clockEdge()+expiredPeriod)>=blk->expired_count) {
                setExpired(clockEdge()+expiredPeriod,blk,pkt->getAddr());
            }
            //Qi: update blk firstAccessTick here
            if(isBottomLevel && blk->firstAccessTick == 0) {
                blk->firstAccessTick = curTick();
            }

            incHitCount(pkt);
            satisfyCpuSideRequest(pkt, blk);
            //Qi: Hit here, for ALT1 we need to upate some information
            if(isLLC && alt_mech == 1 && !blk->isSram) {
                //Qi: is write, update SC and lastWrite for ALT1
                if(pkt->isWrite()) {
                    blk->SC++;
                    DPRINTF(ALT1,"write hit addr %x, current SC %d\n",tags->regenerateBlkAddr(blk->tag,blk->set), blk->SC);
                    // Qi: check if meet transfer condition, if yes, transfer it from stt ram entry to sram entry
                    if(blk->SC == 3 || (blk->SC == 2 && blk->lastWrite == true))
                    {
                        pushToSram(blk, writebacks);
                    }
                    else
                        blk->lastWrite = true;
                }
                else if(pkt->isRead()) {
                    blk->lastWrite = false;
                }
            }
            return true;
        }
    }

    // Can't satisfy access normally... either no block (blk == NULL)
    // or have block but need exclusive & only have shared.

    // Writeback handling is special case.  We can write the block
    // into the cache without having a writeable copy (or any copy at
    // all).
    if (pkt->cmd == MemCmd::Writeback) {
	//num_actual_write_backs++;
	//DPRINTF(DeadStat,"processing writeback instruction");
	/*if (blk != NULL) {
	    if(blk->blkSource == BlockFill && blk->refCount <= 1)
		num_dead_value++;
	}*/
        assert(blkSize == pkt->getSize());
        //Qi: to prevent scenario writeback happen between HWprefetch
        // set and HWprefetch launch, have order problem
        if(isBottomLevel) {
            MSHR *temp_mshr = mshrQueue.findMatch(pkt->getAddr());
            if(temp_mshr != NULL && temp_mshr->getTarget()->pkt->cmd == MemCmd::HardPFReq) { 
                printf("let us see how many times this scenario happens\n");
                assert(blk == NULL);
                incMissCount(pkt);
                return false;
            }
        }
 
        if (blk == NULL) {
            // need to do a replacement
            blk = allocateBlock(pkt->getAddr(), writebacks,1);
            if (blk == NULL) {
                // no replaceable block available, give up.
                // writeback will be forwarded to next level.
                incMissCount(pkt);
                if(isLLC && !isBottomLevel) {
                    LLCMiss++;
                    sttRamAccesses++;
                }
                if(isBottomLevel) {
                    LLCMiss++;
                    sttRamAccesses++;
                }
                return false;
            }
	    int dead_on_arrival=0;
	    int closing_writes=0;
            tags->insertBlock(pkt, blk, dead_on_arrival, closing_writes);
	    if(isBottomLevel) {
                setExpired(clockEdge() + responseLatency * clockPeriod() + writeLatency * clockPeriod()  + expiredPeriod,blk,pkt->getAddr());
	        /*blk->setExpiredTime(clockEdge() + responseLatency * clockPeriod() + writeLatency * clockPeriod()  + expiredPeriod);
		DPRINTF(ExpiredBlock,"try schedule expired check at writeback operation, addr %x, time %d\n",pkt->getAddr(),blk->expired_count);
		if(!handleExpiredEvent.scheduled())
		    schedule(handleExpiredEvent,blk->expired_count);
		else {
		    DPRINTF(ExpiredBlock,"wait for reschedule, push tick %ld\n",blk->expired_count);
		    PendingExpiredQueue.push_back(blk->expired_count);
		}*/
	    }
	    if(dead_on_arrival != 0)
	    	num_dead_on_arrival++;
	    if(closing_writes != 0)
		num_closing_writes++;
	    DPRINTF(DeadStat, "Update dead stat here,dead_on_arrival %d, closing_writes %d\n",dead_on_arrival,closing_writes);
            DPRINTF(SttCache,"update readable status for addr %x\n",tags->regenerateBlkAddr(blk->tag,blk->OID));
            blk->status = BlkValid | BlkReadable;
        }
	//if(blk->blkSource == BlockFill && blk->refCount == 0)
		//num_dead_value++;
        // writeback hit
        else {
            if(isBottomLevel) {
                DPRINTF(SttCache,"clockEdge %d, expiredPeriod %d, total %d\n",clockEdge(),expiredPeriod, clockEdge()+expiredPeriod);
                if(blk->expired_count <= clockEdge() + expiredPeriod) {
                    assert(clockEdge() + expiredPeriod == blk->expired_count);
                }
                assert(clockEdge()+expiredPeriod <= blk->expired_count);
            }
            if(isBottomLevel && blk->sourceTag == 0 && (clockEdge()+expiredPeriod)>=blk->expired_count) {
                setExpired(clockEdge()+expiredPeriod,blk,pkt->getAddr());
                /*blk->setExpiredTime(clockEdge()+expiredPeriod);
                DPRINTF(ExpiredBlock,"Try schedule for handlefill addr %x expire at ticks %d\n",tags->regenerateBlkAddr(blk->tag,blk->set),blk->expired_count);
                if(!handleExpiredEvent.scheduled())
	            schedule(handleExpiredEvent,blk->expired_count);
	        else {
		    DPRINTF(ExpiredBlock,"wait for reschedule\n");
		    PendingExpiredQueue.push_back(blk->expired_count);
                }*/
            }
        }
        std::memcpy(blk->data, pkt->getPtr<uint8_t>(), blkSize);
        blk->status |= BlkDirty;
        if (pkt->isSupplyExclusive()) {
            DPRINTF(SttCache,"update writable staus for addr %x\n",pkt->getAddr());
            blk->status |= BlkWritable;
        }
        // nothing else to do; writeback doesn't expect response
        assert(!pkt->needsResponse());
        DPRINTF(Cache, "%s new state is %s\n", __func__, blk->print());
        // Qi: collect some stats
        if(isLLC && !isBottomLevel) {
            sttRamHit++;
            sttRamAccesses++;
        }
        if(isBottomLevel) {
            if(blk->sourceTag == 0)    sttRamHit++;
            else    edRamHit++;
            sttRamAccesses++;
        }

        incHitCount(pkt);
        if(isLLC && alt_mech == 1 && !blk->isSram) {
        //Qi: is write, update SC and lastWrite for ALT1
            if(pkt->isWrite()) {
                blk->SC++;
                DPRINTF(ALT1,"writeback addr %x, current SC %d\n",tags->regenerateBlkAddr(blk->tag,blk->set), blk->SC);
                // Qi: check if meet transfer condition, if yes, transfer it from stt ram entry to sram entry
                if(blk->SC == 3 || (blk->SC == 2 && blk->lastWrite == true))
                {
                    pushToSram(blk, writebacks);
                }
                else
                    blk->lastWrite = true;
            }
            else if(pkt->isRead()) {
                blk->lastWrite = false;
            }
        }

        //Qi: for alt3, please pay attention here, if bypass
        //decision is true, do not write back here, write back
        //to main memory directly, also if the blk is in llc
        //now, we need to invalidate it first
        if(isLLC && alt_mech == 3 && bypass_decision && pkt->cmd == MemCmd::Writeback && blk != NULL) {
            //Qi:writeback the blk
            writebacks.push_back(writebackBlk(blk));
            //Qi: invalidate the blk here
            tags->invalidate(blk);
            blk->invalidate();
            
            return true;
        }

        return true;
    }

    incMissCount(pkt);
    // Qi: collect some stats
    if(isLLC && !isBottomLevel) {
        LLCMiss++;
        sttRamAccesses++;
    }
    if(isBottomLevel) {
        LLCMiss++;
        sttRamAccesses++;
    }

    if (blk == NULL && pkt->isLLSC() && pkt->isWrite()) {
        // complete miss on store conditional... just give up now
        pkt->req->setExtraData(0);
        return true;
    }

    return false;
}


class ForwardResponseRecord : public Packet::SenderState
{
  public:

    PortID prevSrc;

    ForwardResponseRecord(PortID prev_src) : prevSrc(prev_src)
    {}
};


template<class TagStore>
bool
Cache<TagStore>::recvCheckAddr(Addr a) {
    DPRINTF(CheckAddr,"%s for HWPrefetch addr %x\n",__func__,a);
    if(tags->findBlock(a) == NULL) {
        DPRINTF(CheckAddr,"Addr %x not in this cache\n",a);
        return false;
    }
    else {
        DPRINTF(CheckAddr,"Addr %x is in this cache\n",a);
        return true;
    }
}


template<class TagStore>
void
Cache<TagStore>::recvTimingSnoopResp(PacketPtr pkt)
{
    DPRINTF(Cache, "%s for %s address %x size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    Tick time = clockEdge(hitLatency);

    assert(pkt->isResponse());

    // must be cache-to-cache response from upper to lower level
    ForwardResponseRecord *rec =
        dynamic_cast<ForwardResponseRecord *>(pkt->senderState);
    assert(!system->bypassCaches());

    if (rec == NULL) {
        assert(pkt->cmd == MemCmd::HardPFResp);
        // Check if it's a prefetch response and handle it. We shouldn't
        // get any other kinds of responses without FRRs.
        DPRINTF(Cache, "Got prefetch response from above for addr %#x\n",
                pkt->getAddr());
        recvTimingResp(pkt);
        return;
    }

    pkt->popSenderState();
    pkt->setDest(rec->prevSrc);
    delete rec;
    // @todo someone should pay for this
    pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;
    memSidePort->schedTimingSnoopResp(pkt, time);
}

template<class TagStore>
bool
Cache<TagStore>::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(CacheTags, "%s tags: %s\n", __func__, tags->print());
//@todo Add back in MemDebug Calls
//    MemDebug::cacheAccess(pkt);
    if(isBottomLevel && blockAlign(pkt->getAddr()) == 0x8f9c0) {

        DPRINTF(SttCache,"recvTiming Req 0x8f9c0\n");
    }


    /// @todo temporary hack to deal with memory corruption issue until
    /// 4-phase transactions are complete
    for (int x = 0; x < pendingDelete.size(); x++)
        delete pendingDelete[x];
    pendingDelete.clear();

    // we charge hitLatency for doing just about anything here
    Tick time = clockEdge(hitLatency);

    assert(pkt->isRequest());

    // Just forward the packet if caches are disabled.
    if (system->bypassCaches()) {
	//if(isBottomLevel)
	   // pkt->setSize(eDRAMblkSize);
        memSidePort->sendTimingReq(pkt);
        return true;
    }

    if (pkt->memInhibitAsserted()) {
        DPRINTF(Cache, "mem inhibited on 0x%x: not responding\n",
                pkt->getAddr());
        assert(!pkt->req->isUncacheable());
        // Special tweak for multilevel coherence: snoop downward here
        // on invalidates since there may be other caches below here
        // that have shared copies.  Not necessary if we know that
        // supplier had exclusive copy to begin with.
        if (pkt->needsExclusive() && !pkt->isSupplyExclusive()) {
            Packet *snoopPkt = new Packet(pkt, true);  // clear flags
            // also reset the bus time that the original packet has
            // not yet paid for
            snoopPkt->busFirstWordDelay = snoopPkt->busLastWordDelay = 0;
            snoopPkt->setExpressSnoop();
            snoopPkt->assertMemInhibit();
	    //if(isBottomLevel)
	        //snoopPkt->setSize(eDRAMblkSize);
            memSidePort->sendTimingReq(snoopPkt);
            // main memory will delete snoopPkt
        }
        // since we're the official target but we aren't responding,
        // delete the packet now.

        /// @todo nominally we should just delete the packet here,
        /// however, until 4-phase stuff we can't because sending
        /// cache is still relying on it
        pendingDelete.push_back(pkt);
        return true;
    }

    if (pkt->req->isUncacheable()) {
        uncacheableFlush(pkt);
	//if(isBottomLevel)
	    //pkt->setSize(eDRAMblkSize);

        // @todo: someone should pay for this
        pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;

        // writes go in write buffer, reads use MSHR
        if (pkt->isWrite() && !pkt->isRead()) {
            DPRINTF(ExpiredBlock,"Uncache, into write buffer, addr %x\n",pkt->getAddr());
            allocateWriteBuffer(pkt, time, true);
        } else {
            allocateUncachedReadBuffer(pkt, time, true);
        }
        assert(pkt->needsResponse()); // else we should delete it here??
        return true;
    }

    Cycles lat = hitLatency;
    BlkType *blk = NULL;
    PacketList writebacks;
    if(isBottomLevel && pkt->getAddr() == 0x8f9c0) {
        DPRINTF(SttCache,"recv timing req, addr 0x8f9c0\n");
    }
    bool satisfied = access(pkt, blk, lat, writebacks);

#if 0
    /** @todo make the fast write alloc (wh64) work with coherence. */

    // If this is a block size write/hint (WH64) allocate the block here
    // if the coherence protocol allows it.
    if (!blk && pkt->getSize() >= blkSize && coherence->allowFastWrites() &&
        (pkt->cmd == MemCmd::WriteReq
         || pkt->cmd == MemCmd::WriteInvalidateReq) ) {
        // not outstanding misses, can do this
        MSHR *outstanding_miss = mshrQueue.findMatch(pkt->getAddr());
        if (pkt->cmd == MemCmd::WriteInvalidateReq || !outstanding_miss) {
            if (outstanding_miss) {
                warn("WriteInv doing a fastallocate"
                     "with an outstanding miss to the same address\n");
            }
            blk = handleFill(NULL, pkt, BlkValid | BlkWritable,
                                   writebacks);
            ++fastWrites;
        }
    }
#endif

    // track time of availability of next prefetch, if any
    Tick next_pf_time = 0;

    bool needsResponse = pkt->needsResponse();
    //Qi: Just used to test, it is not useful
    timingInstructions++;
    if (satisfied) {
        //Qi: if the blk is found from stt_ram, prefetch other sub blocks if
        //it is the head sub block
        if(isBottomLevel && blk->sourceTag == 1) {
            prefetchReusedBlk(blk,pkt,time);
        }

        if (prefetcher && (prefetchOnAccess || (blk && blk->wasPrefetched()))) {
            if (blk)
                blk->status &= ~BlkHWPrefetched;
            next_pf_time = prefetcher->notify(pkt, time);
        }

        if (needsResponse) {
            DPRINTF(TestPacket,"make timing response 1\n");
            pkt->makeTimingResponse();
            // @todo: Make someone pay for this
            pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;
			//Qi: curious about whether here we send the timing response
            cpuSidePort->schedTimingResp(pkt, clockEdge(lat));
        } else {
            /// @todo nominally we should just delete the packet here,
            /// however, until 4-phase stuff we can't because sending
            /// cache is still relying on it
            pendingDelete.push_back(pkt);
        }
    } else {
        // miss

        // @todo: Make someone pay for this
        pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;

        Addr blk_addr = blockAlign(pkt->getAddr());
	/*if(isBottomLevel){    
	    Addr blk_addr2 = (blk_addr == (pkt->getAddr()&~(Addr(eDRAMblkSize-1)))) ? blk_addr + blkSize : blk_addr - blkSize;
            MSHR *mshr2 = mshrQueue.findMatch(blk_addr2);
	}*/
		//Qi: let mshr queue to handle the miss here
        MSHR *mshr = mshrQueue.findMatch(blk_addr);
	

        if (mshr && (pkt->cmd != MemCmd::Writeback)) {
            /// MSHR hit
            /// @note writebacks will be checked in getNextMSHR()
            /// for any conflicting requests to the same block

            //@todo remove hw_pf here
            assert(pkt->req->masterId() < system->maxMasters());
            mshr_hits[pkt->cmdToIndex()][pkt->req->masterId()]++;
            if (mshr->threadNum != 0/*pkt->req->threadId()*/) {
                mshr->threadNum = -1;
            }
            mshr->allocateTarget(pkt, time, order++);
            if (mshr->getNumTargets() == numTarget) {
                noTargetMSHR = mshr;
                setBlocked(Blocked_NoTargets);
                // need to be careful with this... if this mshr isn't
                // ready yet (i.e. time > curTick()_, we don't want to
                // move it ahead of mshrs that are ready
                // mshrQueue.moveToFront(mshr);
            }
        } else {
            assert(pkt->req->masterId() < system->maxMasters());
            mshr_misses[pkt->cmdToIndex()][pkt->req->masterId()]++;
            // always mark as cache fill for now... if we implement
            // no-write-allocate or bypass accesses this will have to
            // be changed.
            if (pkt->cmd == MemCmd::Writeback) {
                DPRINTF(ExpiredBlock,"have to write to next level, addr %x\n",pkt->getAddr());
                allocateWriteBuffer(pkt, time, true);
            } else {
                if (blk && blk->isValid()) {
                    // If we have a write miss to a valid block, we
                    // need to mark the block non-readable.  Otherwise
                    // if we allow reads while there's an outstanding
                    // write miss, the read could return stale data
                    // out of the cache block... a more aggressive
                    // system could detect the overlap (if any) and
                    // forward data out of the MSHRs, but we don't do
                    // that yet.  Note that we do need to leave the
                    // block valid so that it stays in the cache, in
                    // case we get an upgrade response (and hence no
                    // new data) when the write miss completes.
                    // As long as CPUs do proper store/load forwarding
                    // internally, and have a sufficiently weak memory
                    // model, this is probably unnecessary, but at some
                    // point it must have seemed like we needed it...
                    if(pkt->getAddr() == 0x8f9c0) {
                        DPRINTF(SttCache,"%s, 0x8f9c0, pkt need exclusive %d, blk is writeable %d, blk is readable %d, blk status %x\n ",pkt->cmdString(),pkt->needsExclusive(),blk->isWritable(),blk->isReadable(),blk->status);
                    }
                    if(!(pkt->needsExclusive() && !blk->isWritable())) {
                        Addr temp_addr;
                        temp_addr = (blk->sourceTag==0)?tags->regenerateBlkAddr(blk->tag,blk->set):stt_tags->regenerateBlkAddr(blk->tag,blk->set);
		        DPRINTF(SttCache,"%s,error here! pkt addr %x, blk addr %x, pkt need exclusive %d, blk is writeable %d, blk is readable %d\n",pkt->cmdString(),pkt->getAddr(),temp_addr,pkt->needsExclusive(),blk->isWritable(),blk->isReadable());
                    }
                    assert(pkt->needsExclusive() && !blk->isWritable());
                    blk->status &= ~BlkReadable;
                }

                allocateMissBuffer(pkt, time, true);
            }

            if (prefetcher) {
                next_pf_time = prefetcher->notify(pkt, time);
            }

	    if(largeBlockEnabled && (isLLC || isBottomLevel)) {
                if(testTimeStampMode) {
                    FILE* fptr = fopen("timestamp.txt","a");
                    //printf("%lx\n",pkt->getOriAddr());
                    fprintf(fptr,"%lx %ld\n",blk_addr,curTick());
                    fclose(fptr);
                }
                prefetchLargeBlk(blk_addr,pkt,time);

	        /*Addr blk_addr2 = (blk_addr == (pkt->getAddr()&~(Addr(eDRAMblkSize-1)))) ? blk_addr + blkSize : blk_addr - blkSize;
                DPRINTF(CheckAddr,"Check hit in higher level cache for addr %x\n",blk_addr2);
	        if(!tags->findBlock(blk_addr2) && !stt_tags->findBlock(blk_addr2) && !mshrQueue.findMatch(blk_addr2) && !writeBuffer.findMatch(blk_addr2)) {
	            DPRINTF(LargeBlock, "try fetch sub block addr %x, orignial sub block addr %x\n",blk_addr2,blk_addr);
                    if(blk_addr2 == 0x8f9c0) {
                        DPRINTF(SttCache,"prefetch 0x8f9c0\n");
                    }
		    Request *subBlockReq = new Request(blk_addr2, blkSize, 0, pkt->req->masterId());
     	            PacketPtr subBlockPkt = new Packet(subBlockReq, MemCmd::HardPFReq);
	            subBlockPkt->allocate();
	            subBlockPkt->req->setThreadContext(pkt->req->contextId(), pkt->req->threadId());
		    subBlockPkt->setBaseAddr(blk_addr);
		    subBlockPkt->setOriCmd(MemCmd::HardPFReq);
	            assert(subBlockPkt->needsResponse());
	            allocateMissBuffer(subBlockPkt, time, true);
	        }*/
            }   
        }
    }

    if (next_pf_time != 0)
        requestMemSideBus(Request_PF, std::max(time, next_pf_time));

    // copy writebacks to write buffer
    while (!writebacks.empty()) {
        PacketPtr wbPkt = writebacks.front();
	//if(isBottomLevel)    wbPkt->setSize(eDRAMblkSize);
        DPRINTF(ExpiredBlock,"Within timingreq, from writeback, addr %x\n",wbPkt->getAddr());
        allocateWriteBuffer(wbPkt, time, true);
        writebacks.pop_front();
    }

    return true;
}

template<class TagStore>
void 
Cache<TagStore>::updateTransStat(Addr repl_addr, Addr align_addr, int num_sub_block) {
    DPRINTF(TestTrans,"addr %x, align addr %x, update transfer stat\n",repl_addr,align_addr);
    int i = num_sub_block;
    BlkType *required_blk = tags->findBlock(repl_addr);
    assert(required_blk->isValid());
    //Qi: if the sub-block has been checked, return
    if(required_blk->transferCheck) {
        DPRINTF(TestTrans,"addr %x has been checked, nothing to do here\n", repl_addr);
        return;
    }
    //Qi: find the earliest accessed sub-block
    Tick minimum_tick = 0;
    int target_blk_num = -1;
    while(i != 0) {
        Addr temp_addr = align_addr + (i-1)*blkSize;
        BlkType *blk = tags->findBlock(temp_addr);
        if(blk == NULL) {
            blk = stt_tags->findBlock(temp_addr);
        }
        if(blk != NULL && blk->reUsed && !blk->transferCheck) {
            if(minimum_tick == 0) {
                minimum_tick = blk->firstAccessTick;
                target_blk_num = i;
            }
            else if(blk->firstAccessTick < minimum_tick) {
                minimum_tick = blk->firstAccessTick;
                target_blk_num = i;
            }
        }
        i--;
    }
    if(target_blk_num == -1) {
        DPRINTF(TestTrans, "no sub block reused, no one should be stored\n");
        //Qi: in this condition, reset all sub blocks' bit vector
        i = num_sub_block;
        while(i != 0) {
            Addr temp_addr = align_addr + (i-1)*blkSize;
            BlkType *blk = tags->findBlock(temp_addr);
            if(blk == NULL) {
                blk = stt_tags->findBlock(temp_addr);
            }
            if(blk != NULL) {
                for(int k = 0; k < 8; k++) {
                    blk->bit_vector[i] = 0;
                }
            }
            i--;
        }
        return;
    }
    DPRINTF(TestTrans,"earliest accessed addr %x, #%d sub block, accessed tick %ld\n",align_addr+(target_blk_num-1)*blkSize,target_blk_num,minimum_tick);
    //Qi: find if the earliest one is much earlier than the others
    //threshold is set 150 cycles (75000000 ticks)

    bool earliest_blk = true;//indicate if we only need to store one sub-block
    i = num_sub_block;
    while(i != 0) {
        Addr temp_addr = align_addr + (i-1)*blkSize;
        BlkType *blk = tags->findBlock(temp_addr);
        if(blk == NULL) {
            blk = stt_tags->findBlock(temp_addr);
        }
        if(blk != NULL && blk->reUsed && !blk->transferCheck && i != target_blk_num) {
            if(blk->firstAccessTick - minimum_tick < 75000000) {
                earliest_blk = false;
                break;
            }
        }
        i--;
    }

    //Qi: if we only need to store one, iterate to set transferCheck and
    //transferrable of all sub blocks and bit_vector of the head sub block
    //set to make all sub blocks except the head as non transferrable and
    //check transfer as true.
    if(earliest_blk) {
        DPRINTF(TestTrans,"head sub block exist, only store the head\n");
        i = num_sub_block;
        //get the target blk firstly
        Addr target_addr = align_addr + (target_blk_num-1)*blkSize;
        BlkType *target_blk = tags->findBlock(target_addr);
        if(target_blk == NULL) {
            target_blk = stt_tags->findBlock(target_addr);
        }
        assert(target_blk);
        target_blk->transferrable = true;
        target_blk->transferCheck = true;
        target_blk->bit_vector[target_blk_num] = 2;
        while(i != 0) {
            Addr temp_addr = align_addr + (i-1)*blkSize;
            BlkType *blk = tags->findBlock(temp_addr);
            if(blk == NULL) {
                blk = stt_tags->findBlock(temp_addr);
            }
            if(blk != NULL) {
                DPRINTF(TestTrans,"check #%d sub block, addr %x,transfer check bit %d, reUsed bit %d, transferrable bit %d, firstAccessTick %x\n",i,temp_addr,blk->transferCheck,blk->reUsed,blk->transferrable,blk->firstAccessTick);
                //Qi: reset the bit vector of non-head sub block
                for(int k = 0; k < 8; k++) {
                    blk->bit_vector[k] = 0;
                }
            }
            if(blk != NULL && i != target_blk_num && !blk->transferCheck) {
                target_blk->transferrable = false;
                target_blk->transferCheck = true;
                if(blk->reUsed) {
                    DPRINTF(TestTrans,"#%d sub block addr %x reused, set corresponding bit vector\n",i,temp_addr);
                    target_blk->bit_vector[i] = 1;
                }
            }
            i--;
        }
    }
    //Qi: else disable bit_vector of all sub blocks, set transferCheck
    // and transferrable bit of all sub blocks
    else {
        DPRINTF(TestTrans,"head sub block does not exist\n");
        i = num_sub_block;
        while(i != 0) {
            Addr temp_addr = align_addr + (i-1)*blkSize;
            BlkType *blk = tags->findBlock(temp_addr);
            if(blk == NULL) {
                blk = stt_tags->findBlock(temp_addr);
            }
            if(blk != NULL) {
                DPRINTF(TestTrans,"check #%d sub block, addr %x, transfer check bit %d, reUsed bit %d, transferrable bit %d, firstAccessTick %x\n",i,temp_addr,blk->transferCheck,blk->reUsed,blk->transferrable,blk->firstAccessTick);
                //Qi: reset the bit vector of non-head sub block
                for(int k = 0; k < 8; k++) {
                    blk->bit_vector[k] = 0;
                }
            }
            if(blk != NULL && !blk->transferCheck) {
                blk->transferCheck = true;
                if(blk->reUsed) {
                    DPRINTF(TestTrans,"current sub block addr %x reused, mark transferrable\n",temp_addr);
                    blk->transferrable = true;
                }
                else {
                    blk->transferrable = false;
                }
            }
            i--;
        }
    }
 
}


template<class TagStore>
void
Cache<TagStore>::prefetchLargeBlk(Addr ori_addr, PacketPtr pkt, Tick time) {
    int num_blk = int(eDRAMblkSize/blkSize);
    Addr base_addr = eDRAM_blkAlign(pkt->getAddr());
    while(num_blk != 0) {
        Addr blk_addr = base_addr + (num_blk - 1) * blkSize;
        if(blk_addr != ori_addr) {
            if(!tags->findBlock(blk_addr) && !stt_tags->findBlock(blk_addr)
               && !mshrQueue.findMatch(blk_addr) 
               && !writeBuffer.findMatch(blk_addr)) {
                
                DPRINTF(LargeBlock, "try fetch sub block addr %x, orignial sub block addr %x\n",blk_addr,ori_addr);
       	        Request *subBlockReq = new Request(blk_addr, blkSize, 0, pkt->req->masterId());
                PacketPtr subBlockPkt = new Packet(subBlockReq, MemCmd::HardPFReq);
   	        subBlockPkt->allocate();
                subBlockPkt->req->setThreadContext(pkt->req->contextId(), pkt->req->threadId());
                subBlockPkt->setBaseAddr(ori_addr);
                subBlockPkt->setOriCmd(MemCmd::HardPFReq);
                assert(subBlockPkt->needsResponse());
                allocateMissBuffer(subBlockPkt, time, true);
            }
        }
        num_blk--;
    }
}


template<class TagStore>
void
Cache<TagStore>::prefetchReusedBlk(BlkType *blk, PacketPtr pkt, Tick time) {

    //Qi: the current sub block is a head sub block, refresh the timestamp
    //since the head sub block is re-accessed
    blk->firstAccessTick = curTick();
    //Qi: reset the transferCheck bit
    blk->transferCheck = false;
    //Qi: find the head sub block #
    int head_sub_block_num = -1;
    for(int i = 0; i < 8; i++) {
        if(blk->bit_vector[i] == 2) {
            head_sub_block_num = i;
            break;
        }
    }

    DPRINTF(TestTrans,"prefetch reused blk, head sub blk num %d\n",head_sub_block_num);
    //Qi: if no "2" is found, means current blk is not head sub block, return
    if(head_sub_block_num == -1) {
        return;
    }
    
    Addr head_addr = tags->regenerateBlkAddr(blk->tag, blk->set);
    //Qi: if current blk is head sub block, prefetch all the marked sub block
    for(int i = 0; i < 8; i++) {
        Addr blk_addr;
        if(i >= head_sub_block_num) {
            blk_addr = head_addr + (i - head_sub_block_num) * blkSize;
        }
        else {
            blk_addr = head_addr - (head_sub_block_num - i) * blkSize;
        }
        DPRINTF(TestTrans, "#%d sub block, addr %x, reused %d\n",i,blk_addr,blk->bit_vector[i]);
        if(blk->bit_vector[i] == 1) {
            if(!tags->findBlock(blk_addr) && !stt_tags->findBlock(blk_addr)
               && !mshrQueue.findMatch(blk_addr) 
               && !writeBuffer.findMatch(blk_addr)) {
                
                DPRINTF(TestTrans, "try prefetch reused sub block addr %x, head sub block addr %x\n",blk_addr,head_addr);
       	        Request *subBlockReq = new Request(blk_addr, blkSize, 0, pkt->req->masterId());
                PacketPtr subBlockPkt = new Packet(subBlockReq, MemCmd::HardPFReq);
   	        subBlockPkt->allocate();
                subBlockPkt->req->setThreadContext(pkt->req->contextId(), pkt->req->threadId());
                subBlockPkt->setBaseAddr(head_addr);
                subBlockPkt->setOriCmd(MemCmd::HardPFReq);
                assert(subBlockPkt->needsResponse());
                allocateMissBuffer(subBlockPkt, time, true);
            }
        }
    }
}



// See comment in cache.hh.
template<class TagStore>
PacketPtr
Cache<TagStore>::getBusPacket(PacketPtr cpu_pkt, BlkType *blk,
                              bool needsExclusive) const
{
    bool blkValid = blk && blk->isValid();

    if (cpu_pkt->req->isUncacheable()) {
        //assert(blk == NULL);
        return NULL;
    }

    if (!blkValid &&
        (cpu_pkt->cmd == MemCmd::Writeback || cpu_pkt->isUpgrade())) {
        // Writebacks that weren't allocated in access() and upgrades
        // from upper-level caches that missed completely just go
        // through.
        return NULL;
    }

    assert(cpu_pkt->needsResponse());

    MemCmd cmd;
    // @TODO make useUpgrades a parameter.
    // Note that ownership protocols require upgrade, otherwise a
    // write miss on a shared owned block will generate a ReadExcl,
    // which will clobber the owned copy.
    const bool useUpgrades = true;
    if (blkValid && useUpgrades) {
        // only reason to be here is that blk is shared
        // (read-only) and we need exclusive
        assert(needsExclusive && !blk->isWritable());
        cmd = cpu_pkt->isLLSC() ? MemCmd::SCUpgradeReq : MemCmd::UpgradeReq;
    } else {
        // block is invalid
        cmd = needsExclusive ? MemCmd::ReadExReq : MemCmd::ReadReq;
    }
    PacketPtr pkt = new Packet(cpu_pkt->req, cmd, blkSize);
    if(cpu_pkt->cmd == MemCmd::HardPFReq) {
        pkt->setBaseAddr(cpu_pkt->getBaseAddr());
        pkt->setOriCmd(cpu_pkt->cmd);
    }

    pkt->allocate();
    DPRINTF(Cache, "%s created %s address %x size %d\n",
            __func__, pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    return pkt;
}


template<class TagStore>
Tick
Cache<TagStore>::recvAtomic(PacketPtr pkt)
{
    Cycles lat = hitLatency;
    int temp_tags_alt_mech;

    // @TODO: make this a parameter
    bool last_level_cache = false;

    // Forward the request if the system is in cache bypass mode.
    if (system->bypassCaches())
        return ticksToCycles(memSidePort->sendAtomic(pkt));

    if (pkt->memInhibitAsserted()) {
        assert(!pkt->req->isUncacheable());
        // have to invalidate ourselves and any lower caches even if
        // upper cache will be responding
        if (pkt->isInvalidate()) {
            BlkType *blk = tags->findBlock(pkt->getAddr());
            if (blk && blk->isValid()) {
                if(!isBottomLevel || blk->sourceTag == 0) { 
                    temp_tags_alt_mech = tags->alt_mech;
                    tags->alt_mech = 0;
                    tags->invalidate(blk);
                    tags->alt_mech = temp_tags_alt_mech;
                }
		else    stt_tags->invalidate(blk);
                blk->invalidate();
                DPRINTF(Cache, "rcvd mem-inhibited %s on 0x%x: invalidating\n",
                        pkt->cmdString(), pkt->getAddr());
            }
            if (!last_level_cache) {
                DPRINTF(Cache, "forwarding mem-inhibited %s on 0x%x\n",
                        pkt->cmdString(), pkt->getAddr());
                lat += ticksToCycles(memSidePort->sendAtomic(pkt));
            }
        } else {
            DPRINTF(Cache, "rcvd mem-inhibited %s on 0x%x: not responding\n",
                    pkt->cmdString(), pkt->getAddr());
        }

        return lat * clockPeriod();
    }

    // should assert here that there are no outstanding MSHRs or
    // writebacks... that would mean that someone used an atomic
    // access in timing mode

    BlkType *blk = NULL;
    PacketList writebacks;
    bool temp_bottom_level = isBottomLevel;
    bool temp_LLC = isLLC;
    unsigned temp_alt_mech = alt_mech;
    temp_tags_alt_mech = tags->alt_mech;
    isBottomLevel = false;
    isLLC = false;
    alt_mech = 0;
    tags->alt_mech = 0;
 
    if (!access(pkt, blk, lat, writebacks)) {
        // MISS
        if(temp_bottom_level)    isBottomLevel = true;
        if(temp_LLC)    isLLC = true;
        alt_mech = temp_alt_mech;
        tags->alt_mech = temp_tags_alt_mech;
        PacketPtr bus_pkt = getBusPacket(pkt, blk, pkt->needsExclusive());

        bool is_forward = (bus_pkt == NULL);

        if (is_forward) {
            // just forwarding the same request to the next level
            // no local cache operation involved
            bus_pkt = pkt;
        }

        DPRINTF(Cache, "Sending an atomic %s for %x\n",
                bus_pkt->cmdString(), bus_pkt->getAddr());

#if TRACING_ON
        CacheBlk::State old_state = blk ? blk->status : 0;
#endif

        lat += ticksToCycles(memSidePort->sendAtomic(bus_pkt));

        DPRINTF(Cache, "Receive response: %s for addr %x in state %i\n",
                bus_pkt->cmdString(), bus_pkt->getAddr(), old_state);

        // If packet was a forward, the response (if any) is already
        // in place in the bus_pkt == pkt structure, so we don't need
        // to do anything.  Otherwise, use the separate bus_pkt to
        // generate response to pkt and then delete it.
        if (!is_forward) {
            if (pkt->needsResponse()) {
                assert(bus_pkt->isResponse());
                if (bus_pkt->isError()) {
                    pkt->makeAtomicResponse();
                    pkt->copyError(bus_pkt);
                } else if (bus_pkt->isRead() ||
                           bus_pkt->cmd == MemCmd::UpgradeResp) {
                    // we're updating cache state to allow us to
                    // satisfy the upstream request from the cache
                    temp_bottom_level = isBottomLevel;
                    temp_LLC = isLLC;
                    temp_alt_mech = alt_mech;
                    temp_tags_alt_mech = tags->alt_mech;
                    isBottomLevel = false;
                    isLLC = false;
                    alt_mech = 0;
                    tags->alt_mech = 0;
                    blk = handleFill(bus_pkt, blk, writebacks);
                    satisfyCpuSideRequest(pkt, blk);
                    if(temp_bottom_level)    isBottomLevel = true;
                    if(temp_LLC)    isLLC = true;
                    alt_mech = temp_alt_mech;
                    tags->alt_mech = temp_tags_alt_mech;
                } else {
                    // we're satisfying the upstream request without
                    // modifying cache state, e.g., a write-through
                    pkt->makeAtomicResponse();
                }
            }
            delete bus_pkt;
        }
    }

    if(temp_bottom_level)    isBottomLevel = true;
    if(temp_LLC)    isLLC = true;
    alt_mech = temp_alt_mech;
    tags->alt_mech = temp_tags_alt_mech;

    // Note that we don't invoke the prefetcher at all in atomic mode.
    // It's not clear how to do it properly, particularly for
    // prefetchers that aggressively generate prefetch candidates and
    // rely on bandwidth contention to throttle them; these will tend
    // to pollute the cache in atomic mode since there is no bandwidth
    // contention.  If we ever do want to enable prefetching in atomic
    // mode, though, this is the place to do it... see timingAccess()
    // for an example (though we'd want to issue the prefetch(es)
    // immediately rather than calling requestMemSideBus() as we do
    // there).

    // Handle writebacks if needed
    while (!writebacks.empty()){
        PacketPtr wbPkt = writebacks.front();
        memSidePort->sendAtomic(wbPkt);
        writebacks.pop_front();
        delete wbPkt;
    }

    // We now have the block one way or another (hit or completed miss)

    if (pkt->needsResponse()) {
        pkt->makeAtomicResponse();
    }

    return lat * clockPeriod();
}


template<class TagStore>
void
Cache<TagStore>::functionalAccess(PacketPtr pkt, bool fromCpuSide)
{
    if (system->bypassCaches()) {
        // Packets from the memory side are snoop request and
        // shouldn't happen in bypass mode.
        assert(fromCpuSide);

        // The cache should be flushed if we are in cache bypass mode,
        // so we don't need to check if we need to update anything.
        memSidePort->sendFunctional(pkt);
        return;
    }

    Addr blk_addr = blockAlign(pkt->getAddr());
    BlkType *blk = tags->findBlock(pkt->getAddr());
    MSHR *mshr = mshrQueue.findMatch(blk_addr);

    pkt->pushLabel(name());

    CacheBlkPrintWrapper cbpw(blk);

    // Note that just because an L2/L3 has valid data doesn't mean an
    // L1 doesn't have a more up-to-date modified copy that still
    // needs to be found.  As a result we always update the request if
    // we have it, but only declare it satisfied if we are the owner.

    // see if we have data at all (owned or otherwise)
    bool have_data = blk && blk->isValid()
        && pkt->checkFunctional(&cbpw, blk_addr, blkSize, blk->data);

    // data we have is dirty if marked as such or if valid & ownership
    // pending due to outstanding UpgradeReq
    bool have_dirty =
        have_data && (blk->isDirty() ||
                      (mshr && mshr->inService && mshr->isPendingDirty()));

    bool done = have_dirty
        || cpuSidePort->checkFunctional(pkt)
        || mshrQueue.checkFunctional(pkt, blk_addr)
        || writeBuffer.checkFunctional(pkt, blk_addr)
        || memSidePort->checkFunctional(pkt);

    DPRINTF(Cache, "functional %s %x %s%s%s\n",
            pkt->cmdString(), pkt->getAddr(),
            (blk && blk->isValid()) ? "valid " : "",
            have_data ? "data " : "", done ? "done " : "");

    // We're leaving the cache, so pop cache->name() label
    pkt->popLabel();

    if (done) {
        pkt->makeResponse();
    } else {
        // if it came as a request from the CPU side then make sure it
        // continues towards the memory side
        if (fromCpuSide) {
            memSidePort->sendFunctional(pkt);
        } else if (forwardSnoops && cpuSidePort->isSnooping()) {
            // if it came from the memory side, it must be a snoop request
            // and we should only forward it if we are forwarding snoops
            cpuSidePort->sendFunctionalSnoop(pkt);
        }
    }
}


/////////////////////////////////////////////////////
//
// Response handling: responses from the memory side
//
/////////////////////////////////////////////////////


template<class TagStore>
void
Cache<TagStore>::recvTimingResp(PacketPtr pkt)
{
    assert(pkt->isResponse());

    Tick time = clockEdge(hitLatency);
    MSHR *mshr = dynamic_cast<MSHR*>(pkt->senderState);
    bool is_error = pkt->isError();

    assert(mshr);

    if (is_error) {
        DPRINTF(Cache, "Cache received packet with error for address %x, "
                "cmd: %s\n", pkt->getAddr(), pkt->cmdString());
    }

    DPRINTF(Cache, "Handling response to %s for address %x\n",
            pkt->cmdString(), pkt->getAddr());

    MSHRQueue *mq = mshr->queue;
    bool wasFull = mq->isFull();

    if (mshr == noTargetMSHR) {
        // we always clear at least one target
        clearBlocked(Blocked_NoTargets);
        noTargetMSHR = NULL;
    }

    // Initial target is used just for stats
    MSHR::Target *initial_tgt = mshr->getTarget();
    //Qi:please pay attention here, if it is a upgrade, the block maybe in stt ram
    BlkType *blk = tags->findBlock(pkt->getAddr());
    if(isBottomLevel && blk == NULL && stt_tags->findBlock(pkt->getAddr()) != NULL) {
        blk = stt_tags->findBlock(pkt->getAddr());
    }
    int stats_cmd_idx = initial_tgt->pkt->cmdToIndex();
    Tick miss_latency = curTick() - initial_tgt->recvTime;
    PacketList writebacks;

    if (pkt->req->isUncacheable()) {
        assert(pkt->req->masterId() < system->maxMasters());
        mshr_uncacheable_lat[stats_cmd_idx][pkt->req->masterId()] +=
            miss_latency;
    } else {
        assert(pkt->req->masterId() < system->maxMasters());
        mshr_miss_latency[stats_cmd_idx][pkt->req->masterId()] +=
            miss_latency;
    }

    bool is_fill = !mshr->isForward &&
        (pkt->isRead() || pkt->cmd == MemCmd::UpgradeResp);
    if(isBottomLevel && pkt->getAddr() == 0x8f9c0) {
        DPRINTF(SttCache,"is fill %d, is error %d\n",is_fill,is_error);
    }
    if (is_fill && !is_error) {
        DPRINTF(Cache, "Block for addr %x being updated in Cache\n",
                pkt->getAddr());

        // give mshr a chance to do some dirty work
        mshr->handleFill(pkt, blk);

        blk = handleFill(pkt, blk, writebacks);
        assert(blk != NULL);
        //Qi: set firstAccessTick of the blk here, do we need to add the
        //condition that blk->firstAccessTick == 0?
        if(isBottomLevel && blk->firstAccessTick == 0 && pkt->cmd != MemCmd::HardPFResp && pkt->cmd != MemCmd::HardPFReq) {
            blk->firstAccessTick = curTick();
        }

        //Qi: if the pkt is not hard prefetch, means the blk is acutually
        //accessed, please set the reused bit
        if(isBottomLevel && pkt->cmd != MemCmd::HardPFResp && pkt->cmd != MemCmd::HardPFReq) {
            blk->reUsed = true;
        }
    }

    //Qi: only used for alt3
    bool bypass_decision = false;
    if(isLLC && alt_mech == 3) {
        bypass_decision = online_sampler->getPredDecision(pkt);
        DPRINTF(ALT3,"recvTimingRecv, addr %x, bypass decision %d\n",pkt->getAddr(),bypass_decision);
    }

    // First offset for critical word first calculations
    int initial_offset = 0;

    if (mshr->hasTargets()) {
        initial_offset = mshr->getTarget()->pkt->getOffset(blkSize);
    }

    while (mshr->hasTargets()) {
        MSHR::Target *target = mshr->getTarget();

        switch (target->source) {
          case MSHR::Target::FromCPU:
            Tick completion_time;
            if (is_fill) {
                satisfyCpuSideRequest(target->pkt, blk,
                                      true, mshr->hasPostDowngrade());
                // How many bytes past the first request is this one
                int transfer_offset =
                    target->pkt->getOffset(blkSize) - initial_offset;
                if (transfer_offset < 0) {
                    transfer_offset += blkSize;
                }

                // If critical word (no offset) return first word time.
                // responseLatency is the latency of the return path
                // from lower level caches/memory to an upper level cache or
                // the core.
                if(alt_mech == 1) {
                    if(blk->isSram) {
                        completion_time = clockEdge(sram_readLatency) +
		            sram_writeLatency * clockPeriod() +
                            (transfer_offset ? pkt->busLastWordDelay :
                            pkt->busFirstWordDelay);
                    }
                    else {
                        completion_time = clockEdge(responseLatency) +
		            writeLatency * clockPeriod() +
                            (transfer_offset ? pkt->busLastWordDelay :
                            pkt->busFirstWordDelay);
                    }
                }
                else {
                    completion_time = clockEdge(responseLatency) +
		        writeLatency * clockPeriod() +
                        (transfer_offset ? pkt->busLastWordDelay :
                        pkt->busFirstWordDelay);
                }

                assert(!target->pkt->req->isUncacheable());

                assert(target->pkt->req->masterId() < system->maxMasters());
                missLatency[target->pkt->cmdToIndex()][target->pkt->req->masterId()] +=
                    completion_time - target->recvTime;
            } else if (pkt->cmd == MemCmd::UpgradeFailResp) {
                // failed StoreCond upgrade
                assert(target->pkt->cmd == MemCmd::StoreCondReq ||
                       target->pkt->cmd == MemCmd::StoreCondFailReq ||
                       target->pkt->cmd == MemCmd::SCUpgradeFailReq);
                // responseLatency is the latency of the return path
                // from lower level caches/memory to an upper level cache or
                // the core.
                completion_time = clockEdge(responseLatency) +
                    pkt->busLastWordDelay;
                target->pkt->req->setExtraData(0);
            } else {
                // not a cache fill, just forwarding response
                // responseLatency is the latency of the return path
                // from lower level cahces/memory to the core.
                completion_time = clockEdge(responseLatency) +
                    pkt->busLastWordDelay;
                if (pkt->isRead() && !is_error) {
                    target->pkt->setData(pkt->getPtr<uint8_t>());
                }
            }
            DPRINTF(TestPacket,"make timing response 2\n");
            target->pkt->makeTimingResponse();
            // if this packet is an error copy that to the new packet
            if (is_error)
                target->pkt->copyError(pkt);
            if (target->pkt->cmd == MemCmd::ReadResp &&
                (pkt->isInvalidate() || mshr->hasPostInvalidate())) {
                // If intermediate cache got ReadRespWithInvalidate,
                // propagate that.  Response should not have
                // isInvalidate() set otherwise.
                target->pkt->cmd = MemCmd::ReadRespWithInvalidate;
                DPRINTF(Cache, "%s updated cmd to %s for address %x\n",
                        __func__, target->pkt->cmdString(),
                        target->pkt->getAddr());
            }
            // reset the bus additional time as it is now accounted for
            target->pkt->busFirstWordDelay = target->pkt->busLastWordDelay = 0;
            //Qi: if bypass in alt3, we do not need to fill in current level cache
            //So response to higher level cache immediatly
            if(isLLC && alt_mech == 3 && bypass_decision) {
                cpuSidePort->schedTimingResp(target->pkt, clockEdge());
            }
            else {
                cpuSidePort->schedTimingResp(target->pkt, completion_time);
            }
            break;

          case MSHR::Target::FromPrefetcher:
            assert(target->pkt->cmd == MemCmd::HardPFReq);
            if (blk)
                blk->status |= BlkHWPrefetched;
            delete target->pkt->req;
            delete target->pkt;
            break;

          case MSHR::Target::FromSnoop:
            // I don't believe that a snoop can be in an error state
            assert(!is_error);
            // response to snoop request
            DPRINTF(Cache, "processing deferred snoop...\n");
            assert(!(pkt->isInvalidate() && !mshr->hasPostInvalidate()));
            handleSnoop(target->pkt, blk, true, true,
                        mshr->hasPostInvalidate());
            break;

          default:
            panic("Illegal target->source enum %d\n", target->source);
        }

        mshr->popTarget();
    }

    //Qi: if bypass in alt3, we do not fill in current level cache, invalidate the blk
    if(isLLC && alt_mech == 3 && bypass_decision) {
        if(blk && blk->isValid()) {
            tags->invalidate(blk);
            blk->invalidate();
        }
    }

    if (blk && blk->isValid()) {
        if (pkt->isInvalidate() || mshr->hasPostInvalidate()) {
            assert(blk != tempBlock);
            if(!isBottomLevel || blk->sourceTag == 0)    tags->invalidate(blk);
            else    stt_tags->invalidate(blk);
            blk->invalidate();
        } else if (mshr->hasPostDowngrade()) {
            blk->status &= ~BlkWritable;
        }
    }

    if (mshr->promoteDeferredTargets()) {
        // avoid later read getting stale data while write miss is
        // outstanding.. see comment in timingAccess()
        if (blk) {
            if(pkt->getAddr() == 0x8f9c0) {
                DPRINTF(SttCache,"mark unreadable for addr 0x8f9c0, blk source %d\n",blk->sourceTag);
            }
            blk->status &= ~BlkReadable;
        }
        mq = mshr->queue;
        mq->markPending(mshr);
        requestMemSideBus((RequestCause)mq->index, clockEdge() +
                          pkt->busLastWordDelay);
    } else {
        mq->deallocate(mshr);
        if (wasFull && !mq->isFull()) {
            clearBlocked((BlockedCause)mq->index);
        }
    }

    // copy writebacks to write buffer
    while (!writebacks.empty()) {
        PacketPtr wbPkt = writebacks.front();
	DPRINTF(ExpiredBlock,"Within timing resp,writeback,addr %x\n",wbPkt->getAddr());
        allocateWriteBuffer(wbPkt, time, true);
        writebacks.pop_front();
    }
    // if we used temp block, clear it out
    if (blk == tempBlock) {
        if (blk->isDirty()) {
            DPRINTF(ExpiredBlock,"here problem? addr %x\n",tags->regenerateBlkAddr(blk->tag,blk->set));
            allocateWriteBuffer(writebackBlk(blk), time, true);
        }
        blk->invalidate();
    }

    DPRINTF(Cache, "Leaving %s with %s for address %x\n", __func__,
            pkt->cmdString(), pkt->getAddr());
    if(alt_mech == 1) {
        tags->printSet(pkt->getAddr());
    }
    delete pkt;
}




template<class TagStore>
PacketPtr
Cache<TagStore>::writebackBlk(BlkType *blk)
{
    assert(blk && blk->isValid() && blk->isDirty());
    int wbMasterId = Request::wbMasterId;
    DPRINTF(DeadStat, "Update WriteBacks stats here,wbMasterId %d\n",wbMasterId);
    num_write_backs++;
    writebacks[Request::wbMasterId]++;
    Addr repl_addr = 0;
    if(blk->sourceTag == 0) {
        if(blk->isSram) {
            repl_addr = tags->regenerateBlkAddr(blk->tag,blk->OID);
        }
        else {
            repl_addr = tags->regenerateBlkAddr(blk->tag,blk->set);
        }
    }
    else if(blk->sourceTag == 1) {
        if(blk->isSram) {
            repl_addr = tags->regenerateBlkAddr(blk->tag,blk->OID);
        }
        else {
            repl_addr = tags->regenerateBlkAddr(blk->tag,blk->set);
        }

    }
    assert(repl_addr != 0);

    Request *writebackReq =
        new Request(repl_addr, blkSize, 0,
                Request::wbMasterId);
    PacketPtr writeback = new Packet(writebackReq, MemCmd::Writeback);
    if (blk->isWritable()) {
        writeback->setSupplyExclusive();
    }
    writeback->allocate();
    std::memcpy(writeback->getPtr<uint8_t>(), blk->data, blkSize);

    blk->status &= ~BlkDirty;
    return writeback;
}

template<class TagStore>
void
Cache<TagStore>::memWriteback()
{
    WrappedBlkVisitor visitor(*this, &Cache<TagStore>::writebackVisitor);
    tags->forEachBlk(visitor);
    if(isBottomLevel)    stt_tags->forEachBlk(visitor);
}

template<class TagStore>
void
Cache<TagStore>::memInvalidate()
{
    WrappedBlkVisitor visitor(*this, &Cache<TagStore>::invalidateVisitor);
    tags->forEachBlk(visitor);
    if(isBottomLevel)    stt_tags->forEachBlk(visitor);
}

template<class TagStore>
bool
Cache<TagStore>::isDirty() const
{
    CacheBlkIsDirtyVisitor<BlkType> visitor;
    tags->forEachBlk(visitor);
    stt_tags->forEachBlk(visitor);

    return visitor.isDirty();
}

template<class TagStore>
bool
Cache<TagStore>::writebackVisitor(BlkType &blk)
{
    if (blk.isDirty()) {
        assert(blk.isValid());

        Addr repl_addr = 0;
        if(blk.sourceTag == 0) {
            if(blk.isSram) {
                repl_addr = tags->regenerateBlkAddr(blk.tag,blk.OID);
            }
            else {
                repl_addr = tags->regenerateBlkAddr(blk.tag,blk.set);
            }

        }
        else if(blk.sourceTag == 1) {
            if(blk.isSram) {
                repl_addr = tags->regenerateBlkAddr(blk.tag,blk.OID);
            }
            else {
                repl_addr = tags->regenerateBlkAddr(blk.tag,blk.set);
            }
        }
        assert(repl_addr != 0);

        Request request(repl_addr,
                        blkSize, 0, Request::funcMasterId);

        Packet packet(&request, MemCmd::WriteReq);
        packet.dataStatic(blk.data);

        memSidePort->sendFunctional(&packet);

        blk.status &= ~BlkDirty;
    }

    return true;
}

template<class TagStore>
bool
Cache<TagStore>::invalidateVisitor(BlkType &blk)
{

    if (blk.isDirty())
        warn_once("Invalidating dirty cache lines. Expect things to break.\n");

    if (blk.isValid()) {
        assert(!blk.isDirty());
        if(!isBottomLevel || blk.sourceTag == 0)    tags->invalidate(dynamic_cast< BlkType *>(&blk));
        else    stt_tags->invalidate(dynamic_cast< BlkType *>(&blk));
        blk.invalidate();
    }

    return true;
}

template<class TagStore>
void
Cache<TagStore>::uncacheableFlush(PacketPtr pkt)
{
    DPRINTF(Cache, "%s%s %x uncacheable\n", pkt->cmdString(),
            pkt->req->isInstFetch() ? " (ifetch)" : "",
            pkt->getAddr());

    if (pkt->req->isClearLL())
        tags->clearLocks();

    BlkType *blk(tags->findBlock(pkt->getAddr()));
    if (blk) {
        writebackVisitor(*blk);
        invalidateVisitor(*blk);
    }
    BlkType *blk2(stt_tags->findBlock(pkt->getAddr()));
    if (blk2) {
        writebackVisitor(*blk2);
        invalidateVisitor(*blk2);
    }
    
}


template<class TagStore>
typename Cache<TagStore>::BlkType*
Cache<TagStore>::allocateBlock(Addr addr, PacketList &writebacks, int _op)
{
   
    DPRINTF(Cache,"LLC allocate new blk entry for addr %x\n",addr);
    BlkType *blk = NULL;
    //Qi: for alt mech 1
    if(isLLC && alt_mech == 1) {
        //Qi: for read miss, allocate stt ram entry
        if(_op == 0) {
            blk = tags->findSttRamVictim(addr);
        }
        //Qi: for write miss, allocate sram entry
        else {
            blk = tags->findSramVictim(addr);
        }
    }
    else {
        blk = tags->findVictim(addr, writebacks);
    }

    if (blk->isValid()) {
        Addr repl_addr = 0;
        if(blk->isSram) {
            repl_addr = tags->regenerateBlkAddr(blk->tag, blk->OID);
        }
        else {
            repl_addr = tags->regenerateBlkAddr(blk->tag, blk->set);
        }
        MSHR *repl_mshr = mshrQueue.findMatch(repl_addr);
        if (repl_mshr) {
            // must be an outstanding upgrade request on block
            // we're about to replace...
            assert(!blk->isWritable());
            assert(repl_mshr->needsExclusive());
            // too hard to replace block with transient state
            // allocation failed, block not inserted
            return NULL;
        } else {
            DPRINTF(Cache, "replacement: replacing %x with %x: %s\n",
                    repl_addr, addr,
                    blk->isDirty() ? "writeback" : "clean");
            if(isBottomLevel)
                DPRINTF(SttCache,"blk resued ? %s\n",(blk->reUsed)?"true":"false");
            //Qi: indicate whether there are multiple dirty sub blocks in the logical large block, if yes, do not push the current block to stt ram and also mark the other sub blocks so that they will not be pushed into stt ram
            //bool multi_reuse_blocks = tags->checkMultipleReuseBlk(repl_addr,eDRAM_blkAlign(repl_addr),eDRAMblkSize/blkSize);

            //bool multi_reuse_blocks = false;
            if(isBottomLevel) {
            //Qi: collect some stats
                if(blk->reUsed)    reusedBlkNum++;
                else    unreusedBlkNum++;
            }
            if(isBottomLevel) {
                DPRINTF(SttCache,"begin check transfer condition, addr %x\n",repl_addr);
                //Qi: update trans bit of the current blk and all other sub-blocks
                //tags->updateTransStat(repl_addr, eDRAM_blkAlign(repl_addr), int(eDRAMblkSize/blkSize));
                updateTransStat(repl_addr, eDRAM_blkAlign(repl_addr), int(eDRAMblkSize/blkSize));

                //Qi: transfer the blk based on transferrable bit
                if(blk->transferrable && blk->reUsed) {
	            DPRINTF(SttCache,"transfer between stt and edram here, addr %x\n",repl_addr);
                    handleTransferBetweenTags(repl_addr, writebacks, blk);
                }
                else if(blk->isDirty()) {
                    writebacks.push_back(writebackBlk(blk));
                }
            }
            else if(isLLC && alt_mech == 1 && blk->isSram && blk->isValid() && tags->getBlkPos(blk) < K_value) {
                DPRINTF(ALT1,"push addr %x to stt ram\n",addr);
                pushToSttRam(blk,writebacks);
            }

            else if (blk->isDirty()) {
                if(isBottomLevel && tags->regenerateBlkAddr(blk->tag,blk->set)==0x8f9c0) {
                    DPRINTF(SttCache,"addr 0x8f9c0 is never reused so evict from edram\n");
                }
                // Save writeback packet for handling by caller
                writebacks.push_back(writebackBlk(blk));
            }

            else {
                if(isBottomLevel && tags->regenerateBlkAddr(blk->tag,blk->set)==0x8f9c0) {
                    DPRINTF(SttCache,"addr 0x8f9c0 is never reused and not dirty, wait to be replaced\n");
                }
            }
        }
    }

    return blk;
}

template<class TagStore>
void
Cache<TagStore>::pushToSttRam(BlkType *_blk, PacketList &writebacks) {
    assert(_blk->isSram);
    Addr blk_addr = tags->regenerateBlkAddr(_blk->tag, _blk->OID);
    DPRINTF(ALT1,"push addr %x from sram to sttram\n",blk_addr);
    BlkType *blk = allocateBlock(blk_addr, writebacks, 0);
    if(blk == NULL) {
        return;
    }
    tags->insertBlockNoPkt(blk_addr,blk);
    blk->whenReady = clockEdge() + hitLatency * clockPeriod() + writeLatency * clockPeriod();
    blk->status = _blk->status;
    blk->srcMasterId = _blk->srcMasterId;
    assert(blk->isReadable());
    std::memcpy(blk->data, _blk->data, blkSize);

    //Qi: invalidate the original stt ram entry
    tags->invalidate(_blk);
    _blk->invalidate();
    //Qi: move the corresponding stt ram entry to specified position
    tags->moveToPos(blk, K_value);
}

template<class TagStore>
void
Cache<TagStore>::pushToSram(BlkType *_blk, PacketList &writebacks) {
    assert(!(_blk->isSram));
    Addr blk_addr = tags->regenerateBlkAddr(_blk->tag, _blk->set);
    DPRINTF(ALT1,"push addr %x from stt ram to sram\n",blk_addr);
    BlkType *blk = allocateBlock(blk_addr, writebacks, 1);
    if(blk == NULL) {
        return;
    }
    tags->insertBlockNoPkt(blk_addr,blk);
    blk->whenReady = clockEdge() + sram_readLatency * clockPeriod() + sram_writeLatency * clockPeriod();
    blk->status = _blk->status;
    blk->srcMasterId = _blk->srcMasterId;
    assert(blk->OID == _blk->set);
    assert(blk->isReadable());
    std::memcpy(blk->data, _blk->data, blkSize);

    //Qi: invalidate the original stt ram entry
    tags->invalidate(_blk);
    _blk->invalidate();
}

template<class TagStore>
void
Cache<TagStore>::handleTransferBetweenTags(Addr addr, PacketList &writebacks,BlkType *_blk) {
    DPRINTF(SttCache,"push addr %x to stt ram\n",addr);
    // Qi: collect some stats
    transferBlkNum++;
    //DPRINTF(SttCache,"handleTransfer, 0x8f9c0 %s\n",stt_tags->findBlock(0x8f9c0)==NULL ? "Miss" : "Hit");
    if(blockAlign(addr) == 0x8f9c0) {
        DPRINTF(SttCache,"0x8f9c0 from edram to sttram\n");
    }    
    assert(!stt_tags->findBlock(addr));
    BlkType* blk = allocateSttBlock(addr, writebacks);
    if(blk == NULL) {
        return;
    }
    //DPRINTF(SttCache,"evict from SttCache %x\n",stt_tags->regenerateBlkAddr(blk->tag,blk->set));
    if(blk->isValid() && stt_tags->regenerateBlkAddr(blk->tag,blk->set) == 0x8f9c0) {
        DPRINTF(SttCache,"evict from SttCache 0x8f9c0\n");
    }
    stt_tags->insertBlockNoPkt(addr, blk);
    blk->whenReady = clockEdge() + stt_readLatency * clockPeriod() + stt_writeLatency * clockPeriod();
    blk->status = _blk->status;
    assert(blk->isReadable());
    blk->srcMasterId = _blk->srcMasterId;
    //Qi: copy bit_vector here
    for(int i = 0; i < 8; i++) {
        blk->bit_vector[i] = _blk->bit_vector[i];
    }
    //Qi: transfer data here...
    std::memcpy(blk->data, _blk->data, blkSize);
}

template<class TagStore>
void
Cache<TagStore>::handleExpired() {
    DPRINTF(ExpiredBlock,"begin dealing with expiration block\n");
    WrappedBlkVisitor visitor(*this, &Cache<TagStore>::checkExpiredVisitor);
    tags->forEachBlk(visitor);
    if(!PendingExpiredQueue.empty()) {
        DPRINTF(ExpiredBlock, "reschedule the expired event, next current time %ld, assigned sechdule time %ld\n",clockEdge(),PendingExpiredQueue[0]);
        reschedule(handleExpiredEvent, std::max(PendingExpiredQueue[0], clockEdge()), true);
	PendingExpiredQueue.pop_front();
    }
}

template<class TagStore>
void
Cache<TagStore>::handleRefresh() {
    DPRINTF(ALT2,"begin dealing with refresh block\n");
    WrappedBlkVisitor visitor(*this, &Cache<TagStore>::checkRefreshVisitor);
    tags->forEachBlk(visitor);
    // schedule for next refresh operation
    DPRINTF(ALT2,"reschedule the refresh event, next current time %ld, assigned schedule time %ld\n",clockEdge(),clockEdge()+refreshPeriod);
    reschedule(handleRefreshEvent, std::max((clockEdge()+refreshPeriod),clockEdge()),true);
}

template<class TagStore>
void
Cache<TagStore>::predBlkSize() {
    DPRINTF(ALT0, "current Tick %ld\n",curTick());
    unsigned temp_blk_size = eDRAMblkSize;
    int temp_index = (int)(log(double(temp_blk_size/64)) / log(2));
    int temp_max_hit = online_monitor[temp_index]->getLscore();
    for(int i = 0; i < 4; i++) {
        online_monitor[i]->printLscore();
        //pred the blk Size we should use
        if(online_monitor[i]->getLscore() > temp_max_hit) {
            temp_max_hit = online_monitor[i]->getLscore();
            temp_blk_size = 64 * ((int)(pow(2,i)));
        }
        //After get the information, reset the Lscore
        eDRAMblkSize = temp_blk_size;
        DPRINTF(ALT0,"curret eDRAM size %d\n",eDRAMblkSize);
        online_monitor[i]->resetLscore();
    }
    
    //reset the eDRAMblkSize
    eDRAMblkSize = temp_blk_size;
    reschedule(predBlkSizeEvent, clockEdge() + 100000000, true);
}

template<class TagStore>
void
Cache<TagStore>::setExpired(Tick expired_, BlkType *blk_, Addr addr_) {
    blk_->setExpiredTime(expired_);
    DPRINTF(ExpiredBlock,"Try schedule for handlefill addr %x expire at ticks %d\n",addr_,blk_->expired_count);
    if(!handleExpiredEvent.scheduled())
        schedule(handleExpiredEvent, blk_->expired_count);
    else {
        DPRINTF(ExpiredBlock,"wait for reschedule,push tick %ld\n",blk_->expired_count);
	PendingExpiredQueue.push_back(blk_->expired_count);
    }
}

template<class TagStore>
void
Cache<TagStore>::setRefresh() {
    if(init_refresh == 0) {
        DPRINTF(ALT2,"Try schedule refresh operation");
        schedule(handleRefreshEvent, clockEdge() + refreshPeriod);
        init_refresh = 1;
    }
    else {}
}

template<class TagStore>
bool
Cache<TagStore>::checkRefreshVisitor(BlkType &blk) {
    PacketList writebacks;
    Tick time = clockEdge(hitLatency);
    Addr repl_addr = tags->regenerateBlkAddr(blk.tag, blk.set);
    //Qi: if the blk is predicted as dead we do not refresh it, we ignore the expire count in alt2
    //Qi: since the refresh operations are periodical autimatically
    //Qi: So what we do here is updating pred_stat and accumulate to reach TIME
    if(blk.isValid()) {
        assert(!blk.disabled);
        int indicator_stat = tags->getIndicatorStat(repl_addr);
        //Qi: if the indicator turn off the dead line predictor, return directly
        if(indicator_stat == 6) {
            DPRINTF(ALT2,"turn off prediction for block addr %x\n",repl_addr);
            blk.TIME = 0;
            return true;
        }
        blk.TIME++;
        assert(blk.pred_stat != 2);
        //Qi: first check if it is in S1 state, if yes, it means now the blk expired, we need to do something
        DPRINTF(ALT2,"current pred stat for blk addr %x is %d\n",repl_addr, blk.pred_stat);
        if(blk.pred_stat == 1) {
            if(mshrQueue.findMatch(repl_addr)) {
                return true;
            }
            else {
                if(blk.isDirty()) {
                    DPRINTF(ALT2,"addr %x is dirty ,write back then expire\n",repl_addr);
	            allocateWriteBuffer(writebackBlk(&blk),time,true);
                }
                else {
                    DPRINTF(SttCache,"addr %x is clean, expire normally\n",repl_addr);
                }
                tags->invalidate(&blk); 
                blk.invalidate(); 
                blk.pred_stat = 2; 
                blk.disabled = true;
                return true;
            }
        }
        //Qi: Time elapsed, we could do something to update the pred stat.
        else if(blk.TIME == 256) {
            if(blk.pred_stat == 0) {
                switch(indicator_stat) {
                    case 0:    blk.pred_stat = 1;  break;
                    case 1:    blk.pred_stat = 3;  break;
                    case 2:    blk.pred_stat = 4;  break;
                    case 3:    blk.pred_stat = 5;  break;
                    case 4:    blk.pred_stat = 6;  break;
                    case 5:    blk.pred_stat = 7;  break;
                    default:   assert(0);  break;
                }
            }
            else {
                switch(blk.pred_stat) {
                    case 3:    blk.pred_stat = 1;  break;
                    case 4:    blk.pred_stat = 3;  break;
                    case 5:    blk.pred_stat = 4;  break;
                    case 6:    blk.pred_stat = 5;  break;
                    case 7:    blk.pred_stat = 6;  break;
                    default:   assert(0);  break;
                }
            }
            DPRINTF(ALT2,"after update the pred stat for blk addr %x is %d\n",repl_addr, blk.pred_stat);
            return true;
        }
    }

    return true;
}


template<class TagStore>
bool
Cache<TagStore>::checkExpiredVisitor(BlkType &blk) {
    PacketList writebacks;
    Tick time = clockEdge(hitLatency);
    Addr repl_addr = tags->regenerateBlkAddr(blk.tag, blk.set);
    //if(blk.expired_count <= curTick() && blk.isValid() && !mshrQueue.findMatch(repl_addr) && !writeBuffer.findMatch(repl_addr)) {
    if(blk.expired_count <= curTick() && blk.isValid() && !mshrQueue.findMatch(repl_addr)) {
        //Qi: collect some stats
        expiredBlkNum++;
        if(blk.reUsed)    reusedBlkNum++;
        else    unreusedBlkNum++;
        blk.setExpiredTime(LLONG_MAX);
        //bool multi_reuse_blocks = tags->checkMultipleReuseBlk(repl_addr,eDRAM_blkAlign(repl_addr),eDRAMblkSize/blkSize);
        DPRINTF(ExpiredBlock,"blk at addr %x expired, handle it\n", repl_addr);
        
        DPRINTF(ExpiredBlock,"begin check transfer condition, expired, addr %x\n",repl_addr);
        //tags->updateTransStat(repl_addr, eDRAM_blkAlign(repl_addr), int(eDRAMblkSize/blkSize));
        updateTransStat(repl_addr, eDRAM_blkAlign(repl_addr), int(eDRAMblkSize/blkSize));

        if(blk.transferrable && blk.reUsed) {
            DPRINTF(ExpiredBlock,"addr %x transferrable, push to stt tag\n",repl_addr);
	    handleTransferBetweenTags(repl_addr,writebacks,&blk);
            
            while(!writebacks.empty()) {
                PacketPtr wbPkt = writebacks.front();
                allocateWriteBuffer(wbPkt,time,true);
                writebacks.pop_front();
            }
        }
        
	else if(blk.isDirty()) {
            DPRINTF(SttCache,"addr %x has never been used but dirty ,write back\n",repl_addr);
	    allocateWriteBuffer(writebackBlk(&blk),time,true);
        }
        else {
            DPRINTF(SttCache,"addr %x has never been used and clean, expire normally\n",repl_addr);
        }
        tags->invalidate(&blk); 
        blk.invalidate();  
    }
    return true;
}

template<class TagStore>
typename Cache<TagStore>::BlkType*
Cache<TagStore>::allocateSttBlock(Addr addr, PacketList &writebacks)
{
    BlkType *blk = stt_tags->findVictim(addr, writebacks);

    if (blk->isValid()) {
        Addr repl_addr = stt_tags->regenerateBlkAddr(blk->tag, blk->set);
        MSHR *repl_mshr = mshrQueue.findMatch(repl_addr);
        if(repl_mshr) {
            assert(!blk->isWritable());
            assert(repl_mshr->needsExclusive());
            DPRINTF(SttCache,"Situation happens when allocating stt blk\n");
            return NULL;
        }
        DPRINTF(SttCache, "replacement: replacing %x with %x in stt tags: %s\n",
                repl_addr, addr,
                blk->isDirty() ? "writeback" : "clean");

        if (blk->isDirty()) {
            // Save writeback packet for handling by caller
            writebacks.push_back(writebackBlk(blk));
        }   
    }
    return blk;
}


// Note that the reason we return a list of writebacks rather than
// inserting them directly in the write buffer is that this function
// is called by both atomic and timing-mode accesses, and in atomic
// mode we don't mess with the write buffer (we just perform the
// writebacks atomically once the original request is complete).
template<class TagStore>
typename Cache<TagStore>::BlkType*
Cache<TagStore>::handleFill(PacketPtr pkt, BlkType *blk,
                            PacketList &writebacks)
{
    Addr addr = pkt->getAddr();
#if TRACING_ON
    CacheBlk::State old_state = blk ? blk->status : 0;
#endif

    if (blk == NULL) {
        // better have read new data...
        assert(pkt->hasData());
        // need to do a replacement
        int pkt_op = pkt->isWrite() ? 1 : 0;
        blk = allocateBlock(addr, writebacks, pkt_op);
        if (blk == NULL) {
            // No replaceable block... just use temporary storage to
            // complete the current request and then get rid of it
            assert(!tempBlock->isValid());
            blk = tempBlock;
            tempBlock->set = tags->extractSet(addr);
            tempBlock->tag = tags->extractTag(addr);
            DPRINTF(Cache, "using temp block for %x\n", addr);
        } else {
        	int dead_on_arrival=0;
		int closing_writes=0;
           	tags->insertBlock(pkt, blk, dead_on_arrival, closing_writes);
		if(isBottomLevel) {
                    setExpired(clockEdge() + responseLatency * clockPeriod() + writeLatency * clockPeriod() + pkt->busLastWordDelay + expiredPeriod,blk,pkt->getAddr());
                }
                if(isLLC && alt_mech == 2) {
                    setRefresh();
                }
		if(dead_on_arrival != 0) {
	            num_dead_on_arrival++;
                }
		if(closing_writes != 0) {
		    num_closing_writes++;
                }
            //tags->insertBlock(pkt, blk);
        }

        // we should never be overwriting a valid block
        assert(!blk->isValid());
    } else {
        // existing block... probably an upgrade
        assert(blk->tag == tags->extractTag(addr) || blk->tag == stt_tags->extractTag(addr));
        // either we're getting new data or the block should already be valid
        assert(pkt->hasData() || blk->isValid());
        // don't clear block status... if block is already dirty we
        // don't want to lose that
    }
    if(isBottomLevel && pkt->getAddr() == 0x8f9c0) {
        DPRINTF(SttCache,"update readable status for addr 0x8f9c0, blk source %d, %s\n",blk->sourceTag,pkt->cmdString());
    }
    blk->status |= BlkValid | BlkReadable;

    if (!pkt->sharedAsserted()) {
        if(isBottomLevel && pkt->getAddr() == 0x8f9c0) {
            DPRINTF(SttCache,"update writable status for addr 0x8f9c0, blk source %d, %s\n",blk->sourceTag,pkt->cmdString());
        }
        blk->status |= BlkWritable;
        // If we got this via cache-to-cache transfer (i.e., from a
        // cache that was an owner) and took away that owner's copy,
        // then we need to write it back.  Normally this happens
        // anyway as a side effect of getting a copy to write it, but
        // there are cases (such as failed store conditionals or
        // compare-and-swaps) where we'll demand an exclusive copy but
        // end up not writing it.
        if (pkt->memInhibitAsserted())
            blk->status |= BlkDirty;
    }

    DPRINTF(Cache, "Block addr %x moving from state %x to %s\n",
            addr, old_state, blk->print());

    // if we got new data, copy it in
    if (pkt->isRead()) {
        std::memcpy(blk->data, pkt->getPtr<uint8_t>(), blkSize);
    }
    if(alt_mech == 1) {
        if(blk->isSram) {
            blk->whenReady = clockEdge() + sram_readLatency * clockPeriod() 
                + sram_writeLatency * clockPeriod() + pkt->busLastWordDelay;
        }
        else {
            blk->whenReady = clockEdge() + responseLatency * clockPeriod() +
                writeLatency * clockPeriod() + pkt->busLastWordDelay;

        }
    }
    else {
        blk->whenReady = clockEdge() + responseLatency * clockPeriod() +
            writeLatency * clockPeriod() + pkt->busLastWordDelay;
    }

    return blk;
}


/////////////////////////////////////////////////////
//
// Snoop path: requests coming in from the memory side
//
/////////////////////////////////////////////////////

template<class TagStore>
void
Cache<TagStore>::
doTimingSupplyResponse(PacketPtr req_pkt, uint8_t *blk_data,
                       bool already_copied, bool pending_inval)
{
    DPRINTF(Cache, "%s for %s address %x size %d\n", __func__,
            req_pkt->cmdString(), req_pkt->getAddr(), req_pkt->getSize());
    // timing-mode snoop responses require a new packet, unless we
    // already made a copy...
    PacketPtr pkt = already_copied ? req_pkt : new Packet(req_pkt);
    assert(req_pkt->isInvalidate() || pkt->sharedAsserted());
    pkt->allocate();
    DPRINTF(TestPacket,"make timing response 3\n");
    pkt->makeTimingResponse();
    // @todo Make someone pay for this
    pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;
    if (pkt->isRead()) {
        pkt->setDataFromBlock(blk_data, blkSize);
    }
    if (pkt->cmd == MemCmd::ReadResp && pending_inval) {
        // Assume we defer a response to a read from a far-away cache
        // A, then later defer a ReadExcl from a cache B on the same
        // bus as us.  We'll assert MemInhibit in both cases, but in
        // the latter case MemInhibit will keep the invalidation from
        // reaching cache A.  This special response tells cache A that
        // it gets the block to satisfy its read, but must immediately
        // invalidate it.
        pkt->cmd = MemCmd::ReadRespWithInvalidate;
    }
    DPRINTF(Cache, "%s created response: %s address %x size %d\n",
            __func__, pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    memSidePort->schedTimingSnoopResp(pkt, clockEdge(hitLatency));
}

template<class TagStore>
void
Cache<TagStore>::handleSnoop(PacketPtr pkt, BlkType *blk,
                             bool is_timing, bool is_deferred,
                             bool pending_inval)
{
    DPRINTF(Cache, "%s for %s address %x size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    // deferred snoops can only happen in timing mode
    assert(!(is_deferred && !is_timing));
    // pending_inval only makes sense on deferred snoops
    assert(!(pending_inval && !is_deferred));
    assert(pkt->isRequest());

    // the packet may get modified if we or a forwarded snooper
    // responds in atomic mode, so remember a few things about the
    // original packet up front
    bool invalidate = pkt->isInvalidate();
    bool M5_VAR_USED needs_exclusive = pkt->needsExclusive();

    if (forwardSnoops) {
        // first propagate snoop upward to see if anyone above us wants to
        // handle it.  save & restore packet src since it will get
        // rewritten to be relative to cpu-side bus (if any)
        bool alreadyResponded = pkt->memInhibitAsserted();
        if (is_timing) {
            Packet snoopPkt(pkt, true);  // clear flags
            snoopPkt.setExpressSnoop();
            snoopPkt.pushSenderState(new ForwardResponseRecord(pkt->getSrc()));
            // the snoop packet does not need to wait any additional
            // time
            snoopPkt.busFirstWordDelay = snoopPkt.busLastWordDelay = 0;
            cpuSidePort->sendTimingSnoopReq(&snoopPkt);
            if (snoopPkt.memInhibitAsserted()) {
                // cache-to-cache response from some upper cache
                assert(!alreadyResponded);
                pkt->assertMemInhibit();
            } else {
                delete snoopPkt.popSenderState();
            }
            if (snoopPkt.sharedAsserted()) {
                if(isBottomLevel && pkt->getAddr() == 0x8f9c0) {
                    DPRINTF(SttCache,"shared assert for 0x8f9c0 in handlesnoop\n");
                }
                pkt->assertShared();
            }
        } else {
            cpuSidePort->sendAtomicSnoop(pkt);
            if (!alreadyResponded && pkt->memInhibitAsserted()) {
                // cache-to-cache response from some upper cache:
                // forward response to original requester
                assert(pkt->isResponse());
            }
        }
    }

     if (!blk || !blk->isValid()) {
         DPRINTF(Cache, "%s snoop miss for %s address %x size %d\n",
                 __func__, pkt->cmdString(), pkt->getAddr(), pkt->getSize());
         return;
     } else {
        DPRINTF(Cache, "%s snoop hit for %s for address %x size %d, "
                "old state is %s\n", __func__, pkt->cmdString(),
                pkt->getAddr(), pkt->getSize(), blk->print());
     }

    // we may end up modifying both the block state and the packet (if
    // we respond in atomic mode), so just figure out what to do now
    // and then do it later
    bool respond = blk->isDirty() && pkt->needsResponse();
    bool have_exclusive = blk->isWritable();

    if (pkt->isRead() && !invalidate) {
        assert(!needs_exclusive);

        if(isBottomLevel && pkt->getAddr() == 0x8f9c0) {
            DPRINTF(SttCache,"shared assert for 0x8f9c0 in handlesnoop 2\n");
        }
        pkt->assertShared();
        int bits_to_clear = BlkWritable;
        const bool haveOwnershipState = true; // for now
        if (!haveOwnershipState) {
            // if we don't support pure ownership (dirty && !writable),
            // have to clear dirty bit here, assume memory snarfs data
            // on cache-to-cache xfer
            bits_to_clear |= BlkDirty;
        }
        blk->status &= ~bits_to_clear;
    }

    if (respond) {
        assert(!pkt->memInhibitAsserted());
        pkt->assertMemInhibit();
        if (have_exclusive) {
            pkt->setSupplyExclusive();
        }
        if (is_timing) {
            doTimingSupplyResponse(pkt, blk->data, is_deferred, pending_inval);
        } else {
            pkt->makeAtomicResponse();
            pkt->setDataFromBlock(blk->data, blkSize);
        }
    } else if (is_timing && is_deferred) {
        // if it's a deferred timing snoop then we've made a copy of
        // the packet, and so if we're not using that copy to respond
        // then we need to delete it here.
        delete pkt;
    }

    // Do this last in case it deallocates block data or something
    // like that
    if (invalidate) {
        assert(blk != tempBlock);
        if(!isBottomLevel || blk->sourceTag == 0)     tags->invalidate(blk);
        else    stt_tags->invalidate(blk);
        blk->invalidate();
    }

    DPRINTF(Cache, "new state is %s\n", blk->print());
}


template<class TagStore>
void
Cache<TagStore>::recvTimingSnoopReq(PacketPtr pkt)
{
    DPRINTF(Cache, "%s for %s address %x size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    // Snoops shouldn't happen when bypassing caches
    assert(!system->bypassCaches());

    // Note that some deferred snoops don't have requests, since the
    // original access may have already completed
    if ((pkt->req && pkt->req->isUncacheable()) ||
        pkt->cmd == MemCmd::Writeback) {
        //Can't get a hit on an uncacheable address
        //Revisit this for multi level coherence
        return;
    }

    BlkType *blk = tags->findBlock(pkt->getAddr());
    if(isBottomLevel && blk == NULL && stt_tags->findBlock(pkt->getAddr()) != NULL) {
        blk = stt_tags->findBlock(pkt->getAddr());
    }

    Addr blk_addr = blockAlign(pkt->getAddr());
    MSHR *mshr = mshrQueue.findMatch(blk_addr);

    // Let the MSHR itself track the snoop and decide whether we want
    // to go ahead and do the regular cache snoop
    if (mshr && mshr->handleSnoop(pkt, order++)) {
        DPRINTF(Cache, "Deferring snoop on in-service MSHR to blk %x."
                "mshrs: %s\n", blk_addr, mshr->print());

        if (mshr->getNumTargets() > numTarget)
            warn("allocating bonus target for snoop"); //handle later
        return;
    }

    //We also need to check the writeback buffers and handle those
    std::vector<MSHR *> writebacks;
    if (writeBuffer.findMatches(blk_addr, writebacks)) {
        DPRINTF(Cache, "Snoop hit in writeback to addr: %x\n",
                pkt->getAddr());

        //Look through writebacks for any non-uncachable writes, use that
        if (writebacks.size()) {
            // We should only ever find a single match
            assert(writebacks.size() == 1);
            mshr = writebacks[0];
            assert(!mshr->isUncacheable());
            assert(mshr->getNumTargets() == 1);
            PacketPtr wb_pkt = mshr->getTarget()->pkt;
            assert(wb_pkt->cmd == MemCmd::Writeback);

            assert(!pkt->memInhibitAsserted());
            pkt->assertMemInhibit();
            if (!pkt->needsExclusive()) {
                
                if(isBottomLevel && pkt->getAddr() == 0x8f9c0) {
                    DPRINTF(SttCache,"shared assert for 0x8f9c0 in recvtimingsnoop\n");
                }
                pkt->assertShared();
                // the writeback is no longer the exclusive copy in the system
                wb_pkt->clearSupplyExclusive();
            } else {
                // if we're not asserting the shared line, we need to
                // invalidate our copy.  we'll do that below as long as
                // the packet's invalidate flag is set...
                assert(pkt->isInvalidate());
            }
            doTimingSupplyResponse(pkt, wb_pkt->getPtr<uint8_t>(),
                                   false, false);

            if (pkt->isInvalidate()) {
                // Invalidation trumps our writeback... discard here
                markInService(mshr);
                delete wb_pkt;
            }
        } // writebacks.size()
    }

    // If this was a shared writeback, there may still be
    // other shared copies above that require invalidation.
    // We could be more selective and return here if the
    // request is non-exclusive or if the writeback is
    // exclusive.
    handleSnoop(pkt, blk, true, false, false);
}

template<class TagStore>
bool
Cache<TagStore>::CpuSidePort::recvTimingSnoopResp(PacketPtr pkt)
{
    // Express snoop responses from master to slave, e.g., from L1 to L2
    cache->recvTimingSnoopResp(pkt);
    return true;
}

template<class TagStore>
Tick
Cache<TagStore>::recvAtomicSnoop(PacketPtr pkt)
{
    // Snoops shouldn't happen when bypassing caches
    assert(!system->bypassCaches());

    if (pkt->req->isUncacheable() || pkt->cmd == MemCmd::Writeback) {
        // Can't get a hit on an uncacheable address
        // Revisit this for multi level coherence
        return 0;
    }

    BlkType *blk = tags->findBlock(pkt->getAddr());
    bool temp_bottom_level = isBottomLevel;
    isBottomLevel = false;
    handleSnoop(pkt, blk, false, false, false);
    if(temp_bottom_level)    isBottomLevel = true;
    return hitLatency * clockPeriod();
}


template<class TagStore>
MSHR *
Cache<TagStore>::getNextMSHR()
{
    // Check both MSHR queue and write buffer for potential requests
    MSHR *miss_mshr  = mshrQueue.getNextMSHR();
    MSHR *write_mshr = writeBuffer.getNextMSHR();

    // Now figure out which one to send... some cases are easy
    if (miss_mshr && !write_mshr) {
	DPRINTF(LargeBlock,"mhsr is not empty\n");
        return miss_mshr;
    }
    if (write_mshr && !miss_mshr) {
	DPRINTF(LargeBlock,"write buffer is not empty\n");
        return write_mshr;
    }

    if (miss_mshr && write_mshr) {
	DPRINTF(LargeBlock,"both are not empty\n");
        // We have one of each... normally we favor the miss request
        // unless the write buffer is full
        if (writeBuffer.isFull() && writeBuffer.inServiceEntries == 0) {
            // Write buffer is full, so we'd like to issue a write;
            // need to search MSHR queue for conflicting earlier miss.
            MSHR *conflict_mshr =
                mshrQueue.findPending(write_mshr->addr, write_mshr->size);

            if (conflict_mshr && conflict_mshr->order < write_mshr->order) {
                // Service misses in order until conflict is cleared.
                return conflict_mshr;
            }

            // No conflicts; issue write
            return write_mshr;
        }

        // Write buffer isn't full, but need to check it for
        // conflicting earlier writeback
        MSHR *conflict_mshr =
            writeBuffer.findPending(miss_mshr->addr, miss_mshr->size);
        if (conflict_mshr) {
            // not sure why we don't check order here... it was in the
            // original code but commented out.

            // The only way this happens is if we are
            // doing a write and we didn't have permissions
            // then subsequently saw a writeback (owned got evicted)
            // We need to make sure to perform the writeback first
            // To preserve the dirty data, then we can issue the write

            // should we return write_mshr here instead?  I.e. do we
            // have to flush writes in order?  I don't think so... not
            // for Alpha anyway.  Maybe for x86?
            return conflict_mshr;
        }

        // No conflicts; issue read
        return miss_mshr;
    }

    // fall through... no pending requests.  Try a prefetch.
    assert(!miss_mshr && !write_mshr);
    DPRINTF(LargeBlock,"both are empty\n");
    if (prefetcher && !mshrQueue.isFull()) {
        // If we have a miss queue slot, we can try a prefetch
        PacketPtr pkt = prefetcher->getPacket();
        if (pkt) {
            Addr pf_addr = blockAlign(pkt->getAddr());
            if (!tags->findBlock(pf_addr) && !mshrQueue.findMatch(pf_addr) &&
                                             !writeBuffer.findMatch(pf_addr)) {
                // Update statistic on number of prefetches issued
                // (hwpf_mshr_misses)
                assert(pkt->req->masterId() < system->maxMasters());
                mshr_misses[pkt->cmdToIndex()][pkt->req->masterId()]++;
                // Don't request bus, since we already have it
                return allocateMissBuffer(pkt, curTick(), false);
            } else {
                // free the request and packet
                delete pkt->req;
                delete pkt;
            }
        }
    }

    return NULL;
}


template<class TagStore>
PacketPtr
Cache<TagStore>::getTimingPacket()
{
    MSHR *mshr = getNextMSHR();

    if (mshr == NULL) {
        return NULL;
    }

    // use request from 1st target
    PacketPtr tgt_pkt = mshr->getTarget()->pkt;
    PacketPtr pkt = NULL;

    DPRINTF(CachePort, "%s %s for address %x size %d\n", __func__,
            tgt_pkt->cmdString(), tgt_pkt->getAddr(), tgt_pkt->getSize());

    if (tgt_pkt->cmd == MemCmd::SCUpgradeFailReq ||
        tgt_pkt->cmd == MemCmd::StoreCondFailReq) {
        // SCUpgradeReq or StoreCondReq saw invalidation while queued
        // in MSHR, so now that we are getting around to processing
        // it, just treat it as if we got a failure response
        pkt = new Packet(tgt_pkt);
        pkt->cmd = MemCmd::UpgradeFailResp;
        pkt->senderState = mshr;
        pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;
        recvTimingResp(pkt);
        return NULL;
    } else if (mshr->isForwardNoResponse()) {
        // no response expected, just forward packet as it is
        // Qi: let us see which addr has the problem
        if(tags->findBlock(mshr->addr) != NULL) {
            DPRINTF(ExpiredBlock,"error with timing packet, addr %x, cmd %s\n",mshr->addr, tgt_pkt->cmdString());
        }
        assert(tags->findBlock(mshr->addr) == NULL);
        pkt = tgt_pkt;
    } else {
        BlkType *blk = tags->findBlock(mshr->addr);
        if(isBottomLevel && blk == NULL && stt_tags->findBlock(mshr->addr) != NULL) {
            blk = stt_tags->findBlock(mshr->addr);
        }

        if (tgt_pkt->cmd == MemCmd::HardPFReq) {
            // It might be possible for a writeback to arrive between
            // the time the prefetch is placed in the MSHRs and when
            // it's selected to send... if so, this assert will catch
            // that, and then we'll have to figure out what to do.
	    DPRINTF(LargeBlock,"HardPrefetch in cache level\n");
	    //if(!isBottomLevel)
            assert(blk == NULL);

            // We need to check the caches above us to verify that
            // they don't have a copy of this block in the dirty state
            // at the moment. Without this check we could get a stale
            // copy from memory that might get used in place of the
            // dirty one.
            Packet snoop_pkt(tgt_pkt, true);
            snoop_pkt.setExpressSnoop();
            snoop_pkt.senderState = mshr;
            cpuSidePort->sendTimingSnoopReq(&snoop_pkt);

            if (snoop_pkt.memInhibitAsserted()) {
                markInService(mshr, &snoop_pkt);
                DPRINTF(Cache, "Upward snoop of prefetch for addr %#x hit\n",
                        tgt_pkt->getAddr());
                if(isBottomLevel) {
                    DPRINTF(SttCache,"Cannot send to lower level\n");
                }
                return NULL;
            }
        }

        pkt = getBusPacket(tgt_pkt, blk, mshr->needsExclusive());

        mshr->isForward = (pkt == NULL);

        if (mshr->isForward) {
            // not a cache block request, but a response is expected
            // make copy of current packet to forward, keep current
            // copy for response handling
            pkt = new Packet(tgt_pkt);
            pkt->allocate();
            if (pkt->isWrite()) {
                pkt->setData(tgt_pkt->getPtr<uint8_t>());
            }
        }
    }

    assert(pkt != NULL);
    pkt->senderState = mshr;
    return pkt;
}


template<class TagStore>
Tick
Cache<TagStore>::nextMSHRReadyTime() const
{
    Tick nextReady = std::min(mshrQueue.nextMSHRReadyTime(),
                              writeBuffer.nextMSHRReadyTime());

    if (prefetcher) {
        nextReady = std::min(nextReady,
                             prefetcher->nextPrefetchReadyTime());
    }

    return nextReady;
}

template<class TagStore>
void
Cache<TagStore>::serialize(std::ostream &os)
{
    bool dirty(isDirty());

    if (dirty) {
        warn("*** The cache still contains dirty data. ***\n");
        warn("    Make sure to drain the system using the correct flags.\n");
        warn("    This checkpoint will not restore correctly and dirty data in "
             "the cache will be lost!\n");
    }

    // Since we don't checkpoint the data in the cache, any dirty data
    // will be lost when restoring from a checkpoint of a system that
    // wasn't drained properly. Flag the checkpoint as invalid if the
    // cache contains dirty data.
    bool bad_checkpoint(dirty);
    SERIALIZE_SCALAR(bad_checkpoint);
}

template<class TagStore>
void
Cache<TagStore>::unserialize(Checkpoint *cp, const std::string &section)
{
    bool bad_checkpoint;
    UNSERIALIZE_SCALAR(bad_checkpoint);
    if (bad_checkpoint) {
        fatal("Restoring from checkpoints with dirty caches is not supported "
              "in the classic memory system. Please remove any caches or "
              " drain them properly before taking checkpoints.\n");
    }
}

///////////////
//
// CpuSidePort
//
///////////////

template<class TagStore>
AddrRangeList
Cache<TagStore>::CpuSidePort::getAddrRanges() const
{
    return cache->getAddrRanges();
}

template<class TagStore>
bool
Cache<TagStore>::CpuSidePort::recvTimingReq(PacketPtr pkt)
{
    // always let inhibited requests through even if blocked
    if (!pkt->memInhibitAsserted() && blocked) {
        assert(!cache->system->bypassCaches());
        DPRINTF(Cache,"Scheduling a retry while blocked\n");
        mustSendRetry = true;
        return false;
    }

    cache->recvTimingReq(pkt);
    return true;
}

template<class TagStore>
Tick
Cache<TagStore>::CpuSidePort::recvAtomic(PacketPtr pkt)
{
    return cache->recvAtomic(pkt);
}

template<class TagStore>
void
Cache<TagStore>::CpuSidePort::recvFunctional(PacketPtr pkt)
{
    // functional request
    cache->functionalAccess(pkt, true);
}

template<class TagStore>
Cache<TagStore>::
CpuSidePort::CpuSidePort(const std::string &_name, Cache<TagStore> *_cache,
                         const std::string &_label)
    : BaseCache::CacheSlavePort(_name, _cache, _label), cache(_cache)
{
}

///////////////
//
// MemSidePort
//
///////////////

template<class TagStore>
bool
Cache<TagStore>::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    cache->recvTimingResp(pkt);
    return true;
}

// Express snooping requests to memside port
template<class TagStore>
void
Cache<TagStore>::MemSidePort::recvTimingSnoopReq(PacketPtr pkt)
{
    // handle snooping requests
    cache->recvTimingSnoopReq(pkt);
}

template<class TagStore>
Tick
Cache<TagStore>::MemSidePort::recvAtomicSnoop(PacketPtr pkt)
{
    return cache->recvAtomicSnoop(pkt);
}

template<class TagStore>
void
Cache<TagStore>::MemSidePort::recvFunctionalSnoop(PacketPtr pkt)
{
    // functional snoop (note that in contrast to atomic we don't have
    // a specific functionalSnoop method, as they have the same
    // behaviour regardless)
    cache->functionalAccess(pkt, false);
}

template<class TagStore>
void
Cache<TagStore>::MemSidePacketQueue::sendDeferredPacket()
{
    // if we have a response packet waiting we have to start with that
    if (deferredPacketReady()) {
        // use the normal approach from the timing port
        trySendTiming();
    } else {
        // check for request packets (requests & writebacks)
        PacketPtr pkt = cache.getTimingPacket();
        if (pkt == NULL) {
            // can happen if e.g. we attempt a writeback and fail, but
            // before the retry, the writeback is eliminated because
            // we snoop another cache's ReadEx.
            waitingOnRetry = false;
        } else {
            MSHR *mshr = dynamic_cast<MSHR*>(pkt->senderState);

            waitingOnRetry = !masterPort.sendTimingReq(pkt);

            if (waitingOnRetry) {
                DPRINTF(CachePort, "now waiting on a retry\n");
                if (!mshr->isForwardNoResponse()) {
                    // we are awaiting a retry, but we
                    // delete the packet and will be creating a new packet
                    // when we get the opportunity
                    delete pkt;
                }
                // note that we have now masked any requestBus and
                // schedSendEvent (we will wait for a retry before
                // doing anything), and this is so even if we do not
                // care about this packet and might override it before
                // it gets retried
            } else {
                cache.markInService(mshr, pkt);
            }
        }
    }

    // if we succeeded and are not waiting for a retry, schedule the
    // next send, not only looking at the response transmit list, but
    // also considering when the next MSHR is ready
    if (!waitingOnRetry) {
        scheduleSend(cache.nextMSHRReadyTime());
    }
}

template<class TagStore>
Cache<TagStore>::
MemSidePort::MemSidePort(const std::string &_name, Cache<TagStore> *_cache,
                         const std::string &_label)
    : BaseCache::CacheMasterPort(_name, _cache, _queue),
      _queue(*_cache, *this, _label), cache(_cache)
{
}
