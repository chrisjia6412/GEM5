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
 * Declaration of a LRU tag store.
 */

#ifndef __MEM_CACHE_TAGS_LRU_HH__
#define __MEM_CACHE_TAGS_LRU_HH__

#include <cassert>
#include <cstring>
#include <list>

#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/cacheset.hh"
#include "mem/cache/blk.hh"
#include "mem/packet.hh"
#include "params/LRU.hh"

class BaseCache;


/**
 * A LRU cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 */
class LRU : public BaseTags
{
  public:
    /** Typedef the block type used in this tag store. */
    typedef CacheBlk BlkType;
    /** Typedef for a list of pointers to the local block class. */
    typedef std::list<BlkType*> BlkList;
    /** Typedef the set type used in this tag store. */
    typedef CacheSet<CacheBlk> SetType;


  protected:
    /** The associativity of the cache. */
    const unsigned assoc;
    /** The number of sets in the cache. */
    const unsigned numSets;
    /** Assoc of semi-independent SRAM cache in the hybrid cache. */
    const unsigned sram_assoc;
    /** The nunmber of SRAM entries each set in the hybrid cache. */
    const unsigned sram_per_set;
    /** The number of sram sets in the hybrid cache. */
    const unsigned numSramSets;
    /** The sram entry read latency*/
    const Cycles sram_readLatency;
    /** The sram entry write latency*/
    const Cycles sram_writeLatency;
  public:
    /** Param indicate whether and what alt mech*/
    unsigned alt_mech;
  protected:
    /** The cache sets. */
    SetType *sets;
    /** The semi sram cache sets. */
    SetType *semiSramSets;
    /** The cache blocks. */
    BlkType *blks;
    /** The data blocks, 1 per cache block. */
    uint8_t *dataBlks;

    /** The amount to shift the address to get the set. */
    int setShift;
    /** The amount to shift the address to get the tag. */
    int tagShift;
    /** Mask out all bits that aren't part of the set index. */
    unsigned setMask;
    /** Mask out all bits that aren't part of the block offset. */
    unsigned blkMask;

public:

    /** Convenience typedef. */
     typedef LRUParams Params;

    /**
     * Construct and initialize this tag store.
     */
    LRU(const Params *p);

    /**
     * Destructor
     */
    virtual ~LRU();

    /**
     * Return the block size.
     * @return the block size.
     */
    unsigned
    getBlockSize() const
    {
        return blkSize;
    }

    /**
     * Return the subblock size. In the case of LRU it is always the block
     * size.
     * @return The block size.
     */
    unsigned
    getSubBlockSize() const
    {
        return blkSize;
    }

    /**
     * Invalidate the given block.
     * @param blk The block to invalidate.
     */
    void invalidate(BlkType *blk);

    /**
     * Access block and update replacement data.  May not succeed, in which case
     * NULL pointer is returned.  This has all the implications of a cache
     * access and should only be used as such. Returns the access latency as a side effect.
     * @param addr The address to find.
     * @param asid The address space ID.
     * @param lat The access latency.
     * @param op_type indicate the inst is read(0) or write(1)
     * @return Pointer to the cache block if found.
     */
    BlkType* accessBlock(Addr addr, Cycles &lat, int context_src, int op_type = 0);

    /**
     * For edram, in which block size is different from others. Access block and update replacement data.  May not succeed, in which case
     * NULL pointer is returned.  This has all the implications of a cache
     * access and should only be used as such. Returns the access latency as a side effect.
     * @param align_addr The aligned address based on addr
     * @param addr The address to find.
     * @param asid The address space ID.
     * @param lat The access latency.
     * @param num_sub_block The # of blocks in the larger block in LLC
     * @return Pointer to the cache block if found.
     */
    BlkType* eDRAM_accessBlock(Addr align_addr, Addr addr, Cycles &lat, int context_src, int num_sub_block);


    /** Return the determination whether this block should be pushed into 
     *  STT RAM. If the sub block is marked as "cannot be transferred", return f     *  alse directly. If only the current sub block is reused in the large
     *  block then the current sub block should be pushed, return true. 
     *  If multiple sub blocks are reused, do not push and mark 
     *  all sub blocks as 'cannot be transferred'
     */ 
     bool checkMultipleReuseBlk(Addr repl_addr, Addr align_addr, int num_sub_block);


    /**
     * Finds the given address in the cache, do not update replacement data.
     * i.e. This is a no-side-effect find of a block.
     * @param addr The address to find.
     * @param asid The address space ID.
     * @return Pointer to the cache block if found.
     */
    BlkType* findBlock(Addr addr) const;

    /**
     * Find a block to evict for the address provided.
     * @param addr The addr to a find a replacement candidate for.
     * @param writebacks List for any writebacks to be performed.
     * @return The candidate block.
     */
    BlkType* findVictim(Addr addr, PacketList &writebacks);

    /**
     * Qi: only used for ALT1, find victim of STT-RAM
     */
    BlkType* findSttRamVictim(Addr addr);

    /**
     * Qi: only used for ALT1, find victim of SRAM  
     */
    BlkType* findSramVictim(Addr addr);

    /**
     * Qi: only used for ALT1, find the way_id of the block
     */
    int getBlkPos(BlkType *blk);

    /**
     * Qi: only used for ALT1, move the blk to specified pos
     */
    void moveToPos(BlkType *blk, int pos);

    /**
     * Qi: only used for ALT2, get indicator stat of the set
     */
    int getIndicatorStat(Addr addr);

    /**
     * Insert the new block into the cache.  For LRU this means inserting into
     * the MRU position of the set.
     * @param pkt Packet holding the address to update
     * @param blk The block to update.
     */
     void insertBlock(PacketPtr pkt, BlkType *blk, int&, int&);
    
    /** Qi: only used for alt mech1 
      */
    void insertBlockNoPkt(Addr addr, BlkType *blk);   
    void printSet(Addr addr); 
    /**
     * Generate the tag from the given address.
     * @param addr The address to get the tag from.
     * @return The tag of the address.
     */
    Addr extractTag(Addr addr) const
    {
        return (addr >> tagShift);
    }

    /**
     * Calculate the set index from the address.
     * @param addr The address to get the set from.
     * @return The set index of the address.
     */
    int extractSet(Addr addr) const
    {
        return ((addr >> setShift) & setMask);
    }

    /**
     * Qi: only used for ALT1, Calculate the SRAM set index from addr 
     */
    int SramExtractSet(Addr addr) const {
        return (((addr >> setShift) & setMask) % numSramSets);
    }

    /**
     * Get the block offset from an address.
     * @param addr The address to get the offset of.
     * @return The block offset.
     */
    int extractBlkOffset(Addr addr) const
    {
        return (addr & blkMask);
    }

    /**
     * Align an address to the block size.
     * @param addr the address to align.
     * @return The block address.
     */
    Addr blkAlign(Addr addr) const
    {
        return (addr & ~(Addr)blkMask);
    }

    /**
     * Regenerate the block address from the tag.
     * @param tag The tag of the block.
     * @param set The set of the block.
     * @return The block address.
     */
    Addr regenerateBlkAddr(Addr tag, unsigned set) const
    {
        return ((tag << tagShift) | ((Addr)set << setShift));
    }

    /**
     * Return the hit latency.
     * @return the hit latency.
     */
    Cycles getHitLatency() const
    {
        return hitLatency;
    }
    /**
     *iterated through all blocks and clear all locks
     *Needed to clear all lock tracking at once
     */
    virtual void clearLocks();

    /**
     * Called at end of simulation to complete average block reference stats.
     */
    virtual void cleanupRefs();

    /**
     * Print all tags used
     */
    virtual std::string print() const;

    /**
     * Visit each block in the tag store and apply a visitor to the
     * block.
     *
     * The visitor should be a function (or object that behaves like a
     * function) that takes a cache block reference as its parameter
     * and returns a bool. A visitor can request the traversal to be
     * stopped by returning false, returning true causes it to be
     * called for the next block in the tag store.
     *
     * \param visitor Visitor to call on each block.
     */
    template <typename V>
    void forEachBlk(V &visitor) {
        for (unsigned i = 0; i < numSets * assoc; ++i) {
            if (!visitor(blks[i]))
                return;
        }
    }
};

#endif // __MEM_CACHE_TAGS_LRU_HH__
