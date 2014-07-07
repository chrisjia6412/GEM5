/*
 * Copyright (c) 2013 ARM Limited
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
 * Copyright (c) 2009 The Regents of The University of Michigan
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
 * Authors: Lisa Hsu
 */

/**
 * @file
 * Declaration of an associative set
 */

#ifndef __CACHESET_HH__
#define __CACHESET_HH__

#include <cassert>

#include "mem/cache/blk.hh" // base class

/**
 * An associative set of cache blocks.
 */
template <class Blktype>
class CacheSet
{
  public:
    /** The associativity of this set. */
    int assoc;

    /** The prediction indicator state. */
    int indicator_stat;

    /** Cache blocks in this set, maintained in LRU order 0 = MRU. */
    Blktype **blks;

    /**
     * Find a block matching the tag in this set.
     * @param way_id The id of the way that matches the tag.
     * @param tag The Tag to find.
     * @return Pointer to the block if found. Set way_id to assoc if none found
     */
    Blktype* findBlk(Addr tag, int& way_id) const ;
    Blktype* findBlk(Addr tag) const ;
    /**
     * Qi: for ALT1, find a block matching the tag in semi sram set
     * pay attention to check the OID to see if they match
     */
    Blktype* findSramBlk(Addr _tag, int _set) const;
    int findBlkPos(Addr tag, int set);

    /**
     * Find if there are tag hit in the set, regardless if that is valid or not
     */
    Blktype* findTag(Addr tag, int set = -1, bool sram = false) const;

    /**
     * Move the given block to the head of the list.
     * @param blk The block to move.
     */
    void moveToHead(Blktype *blk);

    /**
     * Move the given block to the tail of the list.
     * @param blk The block to move
     */
    void moveToTail(Blktype *blk);

    /**
     * Qi: only used for alt mech1, move the given block to specified position.
     * @param blk The block to move. pos_id The position to move.
     */
    void moveToPos(Blktype *blk, int pos_id);

    /**
     * Qi: return the indicator stat, only used in ALT2
     */
    int getIndicatorStat() {return indicator_stat;}
  
    /**
     * Qi: update the indicator stat, only used in ALT2
     */
    int updateIndicatorStat(Addr tag, bool &result);
};

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, int& way_id) const
{
    /**
     * Way_id returns the id of the way that matches the block
     * If no block is found way_id is set to assoc.
     */
    way_id = assoc;
    for (int i = 0; i < assoc; ++i) {
        if (blks[i]->tag == tag && blks[i]->isValid() && !(blks[i]->isSram)) {
            way_id = i;
            return blks[i];
        }
    }
    return NULL;
}

template <class Blktype>
Blktype*
CacheSet<Blktype>::findSramBlk(Addr _tag, int _set) const
{
    for (int i = 0; i < assoc; ++i) {
        if(blks[i]->isSram && blks[i]->tag == _tag && blks[i]->isValid() && blks[i]->OID == _set) {
            return blks[i];
        }
    }
    return NULL;
}

template <class Blktype>
int
CacheSet<Blktype>::findBlkPos(Addr _tag, int _set) {
    for (int i = 0; i < assoc; ++i) {
        if(blks[i]->tag == _tag && blks[i]->isValid() && blks[i]->isSram && blks[i]->OID == _set) {
            return i;
        }
    }
    return -1;
}

template <class Blktype>
Blktype*
CacheSet<Blktype>::findTag(Addr tag, int set, bool sram) const {
    for (int i = 0; i < assoc; ++i) {
        if(blks[i]->tag == tag) {
            if(sram) {
                if(blks[i]->OID == set) {
                    assert(blks[i]->isSram);
                    return blks[i];
                }
            }
            else {
                return blks[i];
            }
        }
    }
    return NULL;
}

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag) const
{
    int ignored_way_id;
    return findBlk(tag, ignored_way_id);
}

template <class Blktype>
int
CacheSet<Blktype>::updateIndicatorStat(Addr tag, bool &result) {
    if(indicator_stat != 6) {
        bool prediction_result = true;
        for(int i = 0; i < assoc; i++) {
            if(blks[i]->tag == tag && blks[i]->disabled) {
                assert(!blks[i]->isValid());
                prediction_result = false;
                break;
            }
        }
        result = prediction_result;
        //Qi: predict correct, update the stat
        if(prediction_result) {
            if(indicator_stat != 0) {
                indicator_stat--;
            }
        }
        //Qi: predict wrong, update the stat
        else {
            indicator_stat++;
        }
        return indicator_stat;
    }
    else {
        return 6;
    }
}

template <class Blktype>
void
CacheSet<Blktype>::moveToHead(Blktype *blk)
{
    // nothing to do if blk is already head
    if (blks[0] == blk)
        return;

    // write 'next' block into blks[i], moving up from MRU toward LRU
    // until we overwrite the block we moved to head.

    // start by setting up to write 'blk' into blks[0]
    int i = 0;
    Blktype *next = blk;

    do {
        assert(i < assoc);
        // swap blks[i] and next
        Blktype *tmp = blks[i];
        blks[i] = next;
        next = tmp;
        ++i;
    } while (next != blk);
}

template <class Blktype>
void
CacheSet<Blktype>::moveToTail(Blktype *blk)
{
    // nothing to do if blk is already tail
    if (blks[assoc - 1] == blk)
        return;

    // write 'next' block into blks[i], moving from LRU to MRU
    // until we overwrite the block we moved to tail.

    // start by setting up to write 'blk' into tail
    int i = assoc - 1;
    Blktype *next = blk;

    do {
        assert(i >= 0);
        // swap blks[i] and next
        Blktype *tmp = blks[i];
        blks[i] = next;
        next = tmp;
        --i;
    } while (next != blk);
}

template <class Blktype>
void
CacheSet<Blktype>::moveToPos(Blktype *blk, int pos_id)
{
    // nothing to do if blk is already in the postion
    if (blks[pos_id] == blk)
        return;

    // write 'next' block into blks[i], moving up from MRU toward LRU
    // until we overwrite the block we moved to head.

    // start by setting up to write 'blk' into blks[0]
    int i = pos_id;
    Blktype *next = blk;

    do {
        assert(i >= 0);
        // swap blks[i] and next
        Blktype *tmp = blks[i];
        blks[i] = next;
        next = tmp;
        --i;
    } while (next != blk);
}



#endif
