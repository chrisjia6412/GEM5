/*
 * Copyright (c) 2012-2013 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 */

/**
 * @file
 * Definitions of LRU tag store.
 */

#include <string>

#include "base/intmath.hh"
#include "debug/Cache.hh"
#include "debug/SttCache.hh"
#include "debug/CacheRepl.hh"
#include "debug/SttCacheRepl.hh"
#include "debug/DeadStat.hh"
#include "debug/TestTags.hh"
#include "mem/cache/tags/lrustt.hh"
#include "mem/cache/base.hh"
#include "sim/core.hh"

using namespace std;

LRUSTT::LRUSTT(const Params *p)
    :BaseTags(p),
     assoc(p->assoc),
     numSets((p->assoc != 0)?(p->size / (p->block_size * p->assoc)):0)
{

    DPRINTF(TestTags,"LRUSTT,create tags here, assoc %d, size %ld, set %d\n",assoc,p->size,numSets);
    alt_mech = 0;
    //printf("LRUSTT,create tags here, assoc %d, size %ld\n",assoc,p->size);
    // Check parameters
    if (blkSize < 4 || !isPowerOf2(blkSize)) {
        fatal("Block size must be at least 4 and a power of 2");
    }
    if (numSets <= 0 || !isPowerOf2(numSets)) {
        fatal("# of sets must be non-zero and a power of 2");
    }
    if (assoc <= 0) {
        fatal("associativity must be greater than zero");
    }
    if (hitLatency <= 0) {
        fatal("access latency must be greater than zero");
    }

    blkMask = blkSize - 1;
    setShift = floorLog2(blkSize);
    setMask = numSets - 1;
    tagShift = setShift + floorLog2(numSets);
    cout<<name()<<" ";
    printf("stt ram, setshift %x, tagshift %x, blksize %d, assoc %d, # set %d\n",setShift,tagShift,blkSize,assoc,numSets);
    warmedUp = false;
    /** @todo Make warmup percentage a parameter. */
    warmupBound = numSets * assoc;

    sets = new SetType[numSets];
    blks = new BlkType[numSets * assoc];
    // allocate data storage in one big chunk
    numBlocks = numSets * assoc;
    dataBlks = new uint8_t[numBlocks * blkSize];

    unsigned blkIndex = 0;       // index into blks array
    for (unsigned i = 0; i < numSets; ++i) {
        sets[i].assoc = assoc;

        sets[i].blks = new BlkType*[assoc];

        // link in the data blocks
        for (unsigned j = 0; j < assoc; ++j) {
            // locate next cache block
            BlkType *blk = &blks[blkIndex];
            blk->data = &dataBlks[blkSize*blkIndex];
            blk->setBlkSourceTag(1);
            ++blkIndex;

            // invalidate new cache block
            blk->invalidate();

            //EGH Fix Me : do we need to initialize blk?

            // Setting the tag to j is just to prevent long chains in the hash
            // table; won't matter because the block is invalid
            blk->tag = j;
            blk->whenReady = 0;
            blk->isTouched = false;
            blk->size = blkSize;
            sets[i].blks[j]=blk;
            blk->set = i;
            //assert(!sets[i].blks[j]);
        }
    }
}

LRUSTT::~LRUSTT()
{
    delete [] dataBlks;
    delete [] blks;
    delete [] sets;
}

LRUSTT::BlkType*
LRUSTT::accessBlock(Addr addr, Cycles &lat, int master_id)
{
    DPRINTF(TestTags,"addr %x edram miss, so access stt ram here\n",addr);
    Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    BlkType *blk = sets[set].findBlk(tag);
    //lat = hitLatency;
    if (blk != NULL) {
        // move this block to head of the MRU list
        sets[set].moveToHead(blk);
        DPRINTF(CacheRepl, "set %x: moving blk %x to MRU\n",
                set, regenerateBlkAddr(tag, set));
        if (blk->whenReady > curTick()
            && cache->ticksToCycles(blk->whenReady - curTick()) > lat) {
            lat = cache->ticksToCycles(blk->whenReady - curTick());
        }
        blk->refCount += 1;
    }

    return blk;
}

LRUSTT::BlkType*
LRUSTT::eDRAM_accessBlock(Addr align_addr, Addr addr, Cycles &lat, int master_id,int num_sub_block)
{
    Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    int i = num_sub_block;
    BlkType *required_blk = sets[set].findBlk(tag);
    while(i != 0) {
      Addr temp_addr = align_addr + (i-1)*blkSize;
      unsigned temp_set = extractSet(temp_addr);
      Addr temp_tag = extractTag(temp_addr);
      BlkType *blk = sets[temp_set].findBlk(temp_tag);
      //lat = hitLatency;
      if (blk != NULL) {
          // move this block to head of the MRU list
          sets[temp_set].moveToHead(blk);
          DPRINTF(CacheRepl, "Move sub block %d set %x: moving blk %x to MRU\n",
                  i, temp_set, regenerateBlkAddr(temp_tag, temp_set));
          if (temp_addr == addr && blk->whenReady > curTick()
              && cache->ticksToCycles(blk->whenReady - curTick()) > lat) {
              lat = cache->ticksToCycles(blk->whenReady - curTick());
          }
          blk->refCount += 1;
      }
      i--;
    }

    return required_blk;
}


LRUSTT::BlkType*
LRUSTT::findBlock(Addr addr) const
{
    Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    BlkType *blk = sets[set].findBlk(tag);
    return blk;
}

LRUSTT::BlkType*
LRUSTT::findVictim(Addr addr, PacketList &writebacks)
{
    unsigned set = extractSet(addr);
    DPRINTF(TestTags,"stt tags set %d, assoc %d\n",set,assoc);
    // grab a replacement candidate
    BlkType *blk = sets[set].blks[assoc-1];

    if (blk->isValid()) {
        DPRINTF(SttCacheRepl, "set %x: selecting blk %x for replacement\n",
                set, regenerateBlkAddr(blk->tag, set));
    }
    return blk;
}

void
LRUSTT::insertBlock(PacketPtr pkt, BlkType *blk, int &num_dead_on_arrival_, int &num_closing_writes_)
{
    //printf("using lru\n");
    Addr addr = pkt->getAddr();
    MasterID master_id = pkt->req->masterId();
    if (!blk->isTouched) {
        tagsInUse++;
        blk->isTouched = true;
        if (!warmedUp && tagsInUse.value() >= warmupBound) {
            warmedUp = true;
            warmupCycle = curTick();
        }
    }

    // If we're replacing a block that was previously valid update
    // stats for it. This can't be done in findBlock() because a
    // found block might not actually be replaced there if the
    // coherence protocol says it can't be.
    if (blk->isValid()) {
		/*Qi: collect stats about dead_on_arrival and closing writes here*/
		DPRINTF(DeadStat,"block evict, ref count %d, blk source %d\n",blk->refCount, blk->blkSource);
		if(blk->refCount == 0 && blk->blkSource == BlockFill) {
			num_dead_on_arrival_++;
			DPRINTF(DeadStat,"dead on arrival happens, dead addr %x replaced by new addr %x\n",
			blk->tag, addr);
		}
		if(blk->refCount == 0 && blk->blkSource == WriteBack) {
			num_closing_writes_++;
			DPRINTF(DeadStat,"closing writes happens, dead addr %x replaced by new addr %x\n",
			blk->tag, addr);
		}
        replacements[0]++;
        totalRefs += blk->refCount;
        ++sampledRefs;
        blk->refCount = 0;

        // deal with evicted block
        assert(blk->srcMasterId < cache->system->maxMasters());
        occupancies[blk->srcMasterId]--;

        blk->invalidate();
    }

    blk->isTouched = true;
    // Set tag for new block.  Caller is responsible for setting status.
    blk->tag = extractTag(addr);

	//Qi:set the source of the block
	DPRINTF(DeadStat,"pkt mem cmd %s\n",pkt->cmd.toString());
	if(pkt->cmd == MemCmd::ReadReq || pkt->cmd == MemCmd::WriteReq
	  || pkt->cmd == MemCmd::ReadResp || pkt->cmd == MemCmd::WriteResp
	  || pkt->cmd == MemCmd::ReadExReq || pkt->cmd == MemCmd::ReadExResp)
		blk->blkSource = BlockFill;
	else if(pkt->cmd == MemCmd::Writeback)
		blk->blkSource = WriteBack;
	else
		blk->blkSource = 2;

    // deal with what we are bringing in
    assert(master_id < cache->system->maxMasters());
    occupancies[master_id]++;
    blk->srcMasterId = master_id;

    unsigned set = extractSet(addr);
    sets[set].moveToHead(blk);
}


void
LRUSTT::insertBlockNoPkt(Addr addr, BlkType *blk)
{
    //printf("using lru\n");
    //MasterID master_id = pkt->req->masterId();
    DPRINTF(SttCache,"insert block at addr %x\n",addr);
    if (!blk->isTouched) {
        tagsInUse++;
        blk->isTouched = true;
        if (!warmedUp && tagsInUse.value() >= warmupBound) {
            warmedUp = true;
            warmupCycle = curTick();
        }
    }

    // If we're replacing a block that was previously valid update
    // stats for it. This can't be done in findBlock() because a
    // found block might not actually be replaced there if the
    // coherence protocol says it can't be.
    if (blk->isValid()) {
        replacements[0]++;
        totalRefs += blk->refCount;
        ++sampledRefs;
        blk->refCount = 0;

        // deal with evicted block
        assert(blk->srcMasterId < cache->system->maxMasters());
        occupancies[blk->srcMasterId]--;

        blk->invalidate();
    }

    blk->isTouched = true;
    // Set tag for new block.  Caller is responsible for setting status.
    blk->tag = extractTag(addr);
    blk->setBlkSourceTag(1);

    // deal with what we are bringing in
    //assert(master_id < cache->system->maxMasters());
    //occupancies[master_id]++;
    //blk->srcMasterId = master_id;

    unsigned set = extractSet(addr);
    sets[set].moveToHead(blk);
}


void
LRUSTT::invalidate(BlkType *blk)
{
    assert(blk);
    assert(blk->isValid());
    tagsInUse--;
    assert(blk->srcMasterId < cache->system->maxMasters());
    occupancies[blk->srcMasterId]--;
    blk->srcMasterId = Request::invldMasterId;
    if(blk->isValid())
        DPRINTF(SttCache,"Stt Ram invalidate at addr %x\n",regenerateBlkAddr(blk->tag,blk->set));

    // should be evicted before valid blocks
    unsigned set = blk->set;
    sets[set].moveToTail(blk);
}

void
LRUSTT::clearLocks()
{
    for (int i = 0; i < numBlocks; i++){
        blks[i].clearLoadLocks();
    }
}

LRUSTT *
LRUSTTParams::create()
{
    return new LRUSTT(this);
}

std::string
LRUSTT::print() const {
    std::string cache_state;
    for (unsigned i = 0; i < numSets; ++i) {
        // link in the data blocks
        for (unsigned j = 0; j < assoc; ++j) {
            BlkType *blk = sets[i].blks[j];
            if (blk->isValid())
                cache_state += csprintf("\tset: %d block: %d %s\n", i, j,
                        blk->print());
        }
    }
    if (cache_state.empty())
        cache_state = "no valid tags\n";
    return cache_state;
}

void
LRUSTT::cleanupRefs()
{
    for (unsigned i = 0; i < numSets*assoc; ++i) {
        if (blks[i].isValid()) {
            totalRefs += blks[i].refCount;
            ++sampledRefs;
        }
    }
}
