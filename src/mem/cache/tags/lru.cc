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
#include "debug/DeadStat.hh"
#include "debug/TestTags.hh"
#include "debug/ALT1.hh"
#include "debug/ALT2.hh"
#include "mem/cache/tags/lru.hh"
#include "mem/cache/base.hh"
#include "sim/core.hh"

using namespace std;

LRU::LRU(const Params *p)
    :BaseTags(p), assoc(p->assoc),
     numSets(p->size / (p->block_size * p->assoc)),
     sram_assoc(p->sram_assoc),
     sram_per_set(p->sram_per_set),
     numSramSets(numSets * sram_per_set / sram_assoc),
     sram_readLatency(p->sram_read_latency),
     sram_writeLatency(p->sram_write_latency),
     alt_mech(p->alternative_mech)
{

    DPRINTF(TestTags,"LRU,create tags here, assoc %d, size %ld\n",assoc,p->size);
    if(alt_mech != 0) {
        DPRINTF(TestTags,"ALT %d, sram assoc %d, sram entry per set %d, # of sram sets %d\n", alt_mech, sram_assoc, sram_per_set, numSramSets);
    }
    //printf("LRU, create tags here, assoc %d, size %ld\n",assoc,p->size);
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
    printf("edram setshift %x, tagshift %x, blksize %d, assoc %d, # sets %d\n",setShift,tagShift,blkSize,assoc,numSets);
    warmedUp = false;
    /** @todo Make warmup percentage a parameter. */
    warmupBound = numSets * assoc;

    sets = new SetType[numSets];
    // Qi: in alt mech 1, assign a new semi sram sets
    if(alt_mech == 1) {
        semiSramSets = new SetType[numSramSets];
    }
    blks = new BlkType[numSets * assoc];
    // allocate data storage in one big chunk
    numBlocks = numSets * assoc;
    dataBlks = new uint8_t[numBlocks * blkSize];

    unsigned blkIndex = 0;       // index into blks array
    for (unsigned i = 0; i < numSets; ++i) {
        sets[i].assoc = assoc;
        sets[i].indicator_stat = 0;
        sets[i].blks = new BlkType*[assoc];

        // link in the data blocks
        for (unsigned j = 0; j < assoc; ++j) {
            // locate next cache block
            BlkType *blk = &blks[blkIndex];
            blk->data = &dataBlks[blkSize*blkIndex];
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
            //Qi: set SC and OID information
            blk->SC = 0;
            blk->OID = -1;
            if(j >= (assoc-sram_per_set) && alt_mech == 1) {
                blk->isSram = true;
            }
            sets[i].blks[j]=blk;
            blk->set = i;
        }
    }
    // Qi: only used for alt1 mech, set up a new semi sram cache set
    if(alt_mech == 1) {
        for (unsigned i = 0; i < numSramSets; ++i) {
            semiSramSets[i].assoc = sram_assoc;
            semiSramSets[i].blks = new BlkType*[sram_assoc];
        }
        unsigned current_way = 0;
        for (unsigned i = 0; i < numSets; ++i) {
            if(i != 0 && i % numSramSets == 0) {
                current_way += sram_per_set;
            }
            unsigned sram_set_id = i % numSramSets; 
            unsigned temp_way = current_way;
            for (unsigned j = 0; j < assoc; ++j) {
                if(sets[i].blks[j]->isSram == true) {
                    semiSramSets[sram_set_id].blks[temp_way] = sets[i].blks[j];
                    semiSramSets[sram_set_id].blks[temp_way]->LRUs = temp_way;
                    temp_way++;    
                }
            }
        } 
    } 
}

LRU::~LRU()
{
    delete [] dataBlks;
    delete [] blks;
    delete [] sets;
}

LRU::BlkType*
LRU::accessBlock(Addr addr, Cycles &lat, int master_id, int op_type)
{
    Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    unsigned semiSramSet = set % numSramSets;
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
        // Qi: for ALT2, if the blk is accessed, return to S0 stat
        if(alt_mech == 2) {
            assert(blk->pred_stat != 2);
            blk->pred_stat = 0;
        }
    }
    // Qi: for ALT1, miss in stt ram sets, try finding in sram sets
    else if(alt_mech == 1) {
        blk = semiSramSets[semiSramSet].findSramBlk(tag,set);
        if(blk != NULL) {
            semiSramSets[semiSramSet].moveToHead(blk);
            // Qi: if hit in sram set, als
            BlkType *stt_blk = sets[blk->set].findTag(blk->tag, blk->OID, true);
            assert(stt_blk && stt_blk->isValid());
            sets[blk->set].moveToHead(stt_blk);
            DPRINTF(CacheRepl,"hit in semi Sram set %x, actual stt set %x, OID %x: moving blk %x to MRU\n",semiSramSet,blk->set,blk->OID,addr);
            if(op_type == 0) {
                lat = sram_readLatency;
            }
            else {
                lat = sram_writeLatency;
            }
            if (blk->whenReady > curTick()
                && cache->ticksToCycles(blk->whenReady - curTick()) > lat) {
                lat = cache->ticksToCycles(blk->whenReady - curTick());
            } 
            blk->refCount += 1;
        }
    }
    //Qi: for ALT2, if miss let see if it is due to wrong prediction
    else if(alt_mech == 2) {
        bool prediction_result = false;
        int current_stat = 0;
        current_stat = sets[set].updateIndicatorStat(tag, prediction_result);
        DPRINTF(ALT2,"current indicator stat %d, prediction result %d\n",current_stat, prediction_result);
    }

    return blk;
}


LRU::BlkType*
LRU::eDRAM_accessBlock(Addr align_addr, Addr addr, Cycles &lat, int master_id,int num_sub_block)
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
      if (required_blk !=NULL && blk != NULL) {
          // move this block to head of the MRU list
          sets[temp_set].moveToHead(blk);
          blk->reUsed = true;
          DPRINTF(SttCache,"Previous expired %d, new expired %d, clockEdge %d, expiredPeriod %d\n",blk->expired_count,clockEdge()+expiredPeriod,clockEdge(),expiredPeriod);
          if(required_blk != NULL && (clockEdge()+expiredPeriod) > blk->expired_count) {
              blk->setExpiredTime(clockEdge()+expiredPeriod);
          }
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

LRU::BlkType*
LRU::findBlock(Addr addr) const
{
    Addr tag = extractTag(addr);
    unsigned set = extractSet(addr);
    BlkType *blk = sets[set].findBlk(tag);
    if(blk == NULL && alt_mech == 1) {
        blk = semiSramSets[set % numSramSets].findSramBlk(tag,set);
    } 
    return blk;
}

LRU::BlkType*
LRU::findVictim(Addr addr, PacketList &writebacks)
{
    unsigned set = extractSet(addr);
    int i = 0;
    // grab a replacement candidate
    BlkType *blk = sets[set].blks[assoc-1];
    if(addr == 0x92900) {
        DPRINTF(CacheRepl, "show me corresponding set entry\n");
        while (i < assoc) {
            BlkType *temp_blk = sets[set].blks[i];
            assert(temp_blk);
            DPRINTF(ALT1,"blk %d, OID %d, set %x, blk tag %x, Valid %d\n",i,temp_blk->OID,set,temp_blk->tag,temp_blk->isValid());    
            i++; 
        }
    }
    if (blk->isValid()) {
        DPRINTF(CacheRepl, "set %x: selecting blk %x for replacement\n",
                set, regenerateBlkAddr(blk->tag, set));
    }
    return blk;
}

void 
LRU::printSet(Addr addr) {
    int i = 0;
    unsigned set = extractSet(addr);
    while (i < assoc) {
        BlkType *temp_blk = sets[set].blks[i];
        assert(temp_blk);
        DPRINTF(ALT2,"after handle fill blk %d, OID %d, set %x, blk tag %x, Valid %d, isSram %d, blk value %d\n",i,temp_blk->OID,set,temp_blk->tag,temp_blk->isValid(),temp_blk->isSram,temp_blk);    
            i++; 
    }
}

int
LRU::getIndicatorStat(Addr addr) {
    unsigned set = extractSet(addr);
    int indicator_stat = sets[set].getIndicatorStat();
    return indicator_stat;
}

LRU::BlkType*
LRU::findSttRamVictim(Addr addr)
{
    unsigned set = extractSet(addr);
    // grab a replacement candidate
    unsigned temp_assoc = assoc;
    BlkType *blk = sets[set].blks[temp_assoc-1];
    while(blk->isSram) {
        assert(temp_assoc >= 0);
        temp_assoc--;
        blk = sets[set].blks[temp_assoc-1];
    }
    assert(blk);
    DPRINTF(ALT1,"STT RAM set %x, blk set %x, blk tag %x , isSram %d as victim\n",set,blk->set,blk->tag,blk->isSram);
    if (blk->isValid()) {
        DPRINTF(CacheRepl, "Stt Ram set %x: selecting blk %x for replacement\n", set, regenerateBlkAddr(blk->tag, set));
    }
    return blk;
}

LRU::BlkType*
LRU::findSramVictim(Addr addr)
{
    unsigned set = SramExtractSet(addr);
    unsigned i = 0;
    // grab a replacement candidate
    BlkType *blk = semiSramSets[set].blks[sram_assoc-1];
    assert(blk);
    DPRINTF(ALT1,"SRAM set %x, blk set %x, blk tag %x, isSram %d as victim\n",set,blk->set,blk->tag,blk->isSram);
    DPRINTF(ALT1,"show corresponding STT RAM entry\n");
    while (i < assoc) {
        BlkType *stt_blk = sets[blk->set].blks[i];
        assert(stt_blk);
        DPRINTF(ALT1,"blk %d, OID %x, set %x, blk set %x, blk tag %x, isSram %d as victim\n",i,blk->OID,set,stt_blk->set,stt_blk->tag,stt_blk->isSram);    
        i++; 
    }
    if (blk->isValid()) {
        DPRINTF(CacheRepl, "Sramset %x: selecting blk %x for replacement\n", set, regenerateBlkAddr(blk->tag,blk->OID));
    }
    return blk;
}

int
LRU::getBlkPos(BlkType *blk) {
    assert(blk->isSram);
    assert(blk->isValid());
    int way_id = 0;
    way_id = sets[blk->set].findBlkPos(blk->tag,blk->OID);
    assert(way_id >= 0);
    return way_id;
}

void
LRU::moveToPos(BlkType *blk, int pos) {
    assert(!blk->isSram);
    DPRINTF(ALT2,"set %d, tag %x, valid %d, blk value %d\n",blk->set,blk->tag,blk->isValid(),blk);
    printSet(regenerateBlkAddr(blk->tag, blk->set));    
    //DPRINTF(ALT2,"The current blk pos %d\n",getBlkPos(blk));
    sets[blk->set].moveToPos(blk,pos);
}

void
LRU::insertBlock(PacketPtr pkt, BlkType *blk, int &num_dead_on_arrival_, int &num_closing_writes_)
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
    // set the blk source
    blk->setBlkSourceTag(0);
    // Reset the reused bit attached with the blk
    blk->reUsed = false;
    //Reset the transferrable bit
    blk->transferrable = true;
    //Reset the SC bit
    blk->SC = 0;
    //Reset last write
    blk->lastWrite = false;
    //if Sram entry set OID
    if(blk->isSram) {
        blk->OID = extractSet(addr);
    }
    //Reset pred stat and disable bit
    blk->pred_stat = 0;
    blk->disabled = false;

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
    unsigned semiSramSet = set % numSramSets;
    //sets[set].moveToHead(blk); 
    if(alt_mech == 1) {
        if(blk->isSram == false) {
            sets[set].moveToHead(blk);
        }
        else {
            assert(semiSramSet == blk->set % numSramSets);
            BlkType *stt_blk = sets[blk->set].findTag(blk->tag,blk->OID,true);
            assert(stt_blk);
            sets[blk->set].moveToHead(stt_blk);
            semiSramSets[blk->set % numSramSets].moveToHead(blk);
        }
    }
    else {
        sets[set].moveToHead(blk);
    }
}

void
LRU::insertBlockNoPkt(Addr addr, BlkType *blk)
{
 
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
    // Qi: if sram entry, set OID
    if(blk->isSram) {
        blk->OID = extractSet(addr);
    }
    blk->pred_stat = 0;
    blk->disabled = false;

    // deal with what we are bringing in
    //assert(master_id < cache->system->maxMasters());
    //occupancies[master_id]++;
    //blk->srcMasterId = master_id;

    unsigned set = extractSet(addr);
    unsigned sram_set = SramExtractSet(addr);
    //sets[set].moveToHead(blk);
    if(alt_mech == 1) {
        if(blk->isSram == false) {
            sets[set].moveToHead(blk);
        }
        else {
            assert(sram_set == blk->set % numSramSets);
            BlkType *stt_blk = sets[blk->set].findTag(blk->tag,blk->OID,true);
            assert(stt_blk);
            sets[blk->set].moveToHead(stt_blk);
            semiSramSets[sram_set].moveToHead(blk);
        }
    }
    else {
        sets[set].moveToHead(blk);
    }
}


void
LRU::invalidate(BlkType *blk)
{
    if(blk->isValid() && regenerateBlkAddr(blk->tag,blk->set) == 0x8f9c0) {
        DPRINTF(SttCache,"EDRAM invalidates at 0x8f9c0\n");
    }
    if(blk->isValid()) {
        DPRINTF(SttCache,"EDRAM invalidates at %x\n",regenerateBlkAddr(blk->tag,blk->set));
    }
    assert(blk);
    assert(blk->isValid());
    tagsInUse--;
    assert(blk->srcMasterId < cache->system->maxMasters());
    occupancies[blk->srcMasterId]--;
    blk->srcMasterId = Request::invldMasterId;

    // should be evicted before valid blocks
    unsigned set = blk->set;
    unsigned sram_set = set % numSramSets;
    if(alt_mech == 1) {
        if(blk->isSram == false) {
            sets[set].moveToTail(blk);
        }
        else {
            assert(sram_set == blk->set % numSramSets);
            BlkType *stt_blk = sets[blk->set].findSramBlk(blk->tag,blk->OID);
            BlkType *sram_blk = semiSramSets[sram_set].findSramBlk(blk->tag,blk->OID);
            assert(stt_blk && stt_blk->isSram);
            assert(sram_blk && sram_blk->isSram);
            sets[blk->set].moveToTail(stt_blk);
            semiSramSets[sram_set].moveToHead(sram_blk);
        }
    }
    else {
        sets[set].moveToTail(blk);
    }
}

void
LRU::clearLocks()
{
    for (int i = 0; i < numBlocks; i++){
        blks[i].clearLoadLocks();
    }
}

LRU *
LRUParams::create()
{
    return new LRU(this);
}

std::string
LRU::print() const {
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
LRU::cleanupRefs()
{
    for (unsigned i = 0; i < numSets*assoc; ++i) {
        if (blks[i].isValid()) {
            totalRefs += blks[i].refCount;
            ++sampledRefs;
        }
    }
}

bool LRU::checkMultipleReuseBlk(Addr repl_addr, Addr align_addr, int num_sub_block) {
    int i = num_sub_block;
    int num_reused_blk = 0;
    BlkType *required_blk = sets[extractSet(repl_addr)].findBlk(extractTag(align_addr));
    assert(required_blk->isValid());
    //Qi: this block must belong to a large block which has been detected 
    //multiple reuse sub block
    if(!required_blk->transferrable) {
        return true;
    }
    while(i != 0) {
      Addr temp_addr = align_addr + (i-1)*blkSize;
      unsigned temp_set = extractSet(temp_addr);
      Addr temp_tag = extractTag(temp_addr);
      BlkType *blk = sets[temp_set].findBlk(temp_tag);
      if(blk != NULL && blk->reUsed) {
          num_reused_blk++;
      }
      i--;
    }
    
    if(num_reused_blk <= 1) {
    //Qi: only one sub block or no block is reused in large block, return false
        return false;
    }

    else {
    //Qi: multiple reused block exist, please mark all sub blocks as untransferrable
        i = num_sub_block;
        while(i != 0) {
            Addr temp_addr = align_addr + (i-1)*blkSize;
            unsigned temp_set = extractSet(temp_addr);
            Addr temp_tag = extractTag(temp_addr);
            BlkType *blk = sets[temp_set].findBlk(temp_tag);
            if(blk != NULL) {
                blk->transferrable = false;
            }
            i--;
        }
 
        return true;
    }
 
}
