#ifndef __MONITOR_CC__
#define __MONITOR_CC__

#include <string>
#include <assert.h>

#include "base/intmath.hh"
#include "sim/core.hh"
#include "base/callback.hh"
#include "base/statistics.hh"
#include "sim/clocked_object.hh"
#include "mem/packet.hh"
#include "debug/ALT0.hh"


class BaseCache;

class monitor_entry {
  public:
    bool valid;
    Addr tag;

    monitor_entry() : valid(false), tag(-1)
    {} 

    void invalidate() {
        valid = false;
        tag = -1;
    }
};

class monitor_set {
  public:
    int assoc;
    int set;//indicate the real set #
    monitor_entry **entries;

    monitor_entry* findEntry(Addr tag, int& _hit_assoc) {
        for (int i = 0; i < assoc; ++i) {
            if(entries[i]->tag == tag && entries[i]->valid) {
                _hit_assoc = i;
                return entries[i];
            }
        }
        return NULL;
    }

    void moveToHead(monitor_entry* entry) {
        if (entries[0] == entry)
            return;

        int i = 0;
        monitor_entry *next = entry;

        do {
            assert(i < assoc);
            // swap entries[i] and next
            monitor_entry *tmp = entries[i];
            entries[i] = next;
            next = tmp;
            ++i;
        } while (next != entry);
    }

    void moveToTail(monitor_entry* entry) {
        if (entries[assoc - 1] == entry)
            return;

        int i = assoc - 1;
        monitor_entry *next = entry;

        do {
            assert(i >= 0);
            // swap entries[i] and next
            monitor_entry *tmp = entries[i];
            entries[i] = next;
            next = tmp;
            --i;
        } while (next != entry);
    }

};

class monitor {
  public:
    monitor_set *sets;
    monitor_entry *entries;
    int *Lscore;
    unsigned assoc;
    unsigned numSets;
    int blkSize;
    int tagShift;
    int setShift;
    unsigned int setMask;

    monitor(unsigned _assoc, unsigned _total_numSets, int _blk_size, int sample_ratio) {
        assoc = _assoc;
        numSets = (_total_numSets * 64) / (sample_ratio * _blk_size);
        blkSize = _blk_size;
        setShift = floorLog2(_blk_size);
        tagShift = floorLog2((_total_numSets * 64) / _blk_size) + setShift;
        setMask = ((_total_numSets * 64) / _blk_size) - 1;
        
        sets = new monitor_set[numSets];
        entries = new monitor_entry[numSets * assoc];
        Lscore = new int[assoc];

        unsigned entryIndex = 0;
        for (unsigned i = 0; i < numSets; ++i) {
            sets[i].assoc = assoc;
            sets[i].entries = new monitor_entry*[assoc];
            sets[i].set = numSets * 8 * ((i * blkSize) / 512) + (i % (512 / blkSize));
            DPRINTF(ALT0,"blk size %d, assign set # %d\n",blkSize,sets[i].set);
            for (unsigned j = 0; j < assoc; ++j) {
                monitor_entry *entry = &entries[entryIndex];
                ++entryIndex;

                entry->invalidate();

                entry->tag = j;
                sets[i].entries[j] = entry;
            }
        }
        
        for (unsigned k = 0; k < assoc; ++k) {
            Lscore[k] = 0;
        }
    }

    Addr extractTag(Addr addr) const {
        return (addr >> tagShift);
    }

    int extractSet(Addr addr) const {
        return ((addr >> setShift) & setMask);
    }

    Addr regenerateBlkAddr(Addr tag, int set) {
        return ((tag << tagShift) | ((Addr)set << setShift));
    }

    void updateMonitor(PacketPtr pkt) {
        //only update if set < numSets
        Addr align_addr = pkt->getAddr() & (~(blkSize - 1));
        DPRINTF(ALT0,"blk size %d, addr %x, align addr %x, update monitor\n",blkSize,pkt->getAddr(),align_addr);
        //Qi: check whether the addr is in sampled set
        int temp_set = extractSet(pkt->getAddr());
        Addr temp_tag = extractTag(pkt->getAddr());
        int index = -1;
        for(int i = 0; i < numSets; i++) {
            if(sets[i].set == temp_set) {
                index = i;
                break;
            }
        }
        
        if(index != -1) {
            int hit_assoc = -1;
            monitor_entry *entry = sets[index].findEntry(temp_tag, hit_assoc);
            //should give us information at which assoc position it hits
            //hit
            if(entry != NULL) {
                DPRINTF(ALT0,"Hit in #%d entry, #%d set\n", hit_assoc,index);
                //update Lscore here
                assert(hit_assoc != -1);
                assert(hit_assoc < assoc);
                for(int i = hit_assoc; i < assoc; i++) {
                    Lscore[i]++;
                }
                sets[index].moveToHead(entry);
            }
            //miss
            else {
                DPRINTF(ALT0,"miss, evict invalid entry in #%d set\n",index);
                monitor_entry *victim = sets[index].entries[assoc - 1];
                if(victim->valid == true) {
                    DPRINTF(ALT0,"evict align addr %x in #%d set\n",regenerateBlkAddr(victim->tag,sets[index].set), index);
                }

                victim->invalidate();
                victim->tag = temp_tag;
                victim->valid = true;
                sets[index].moveToHead(victim);
            }
        }
        else {
            DPRINTF(ALT0,"not in the sampled set\n");
        }    
    }

    void printLscore() {
        DPRINTF(ALT0, "show the Lscore for blk size %d\n",blkSize);
        for(int i = 0; i < assoc; i++) {
            DPRINTF(ALT0,"#%d value %d\n",i,Lscore[i]);
        }
        //DPRINTF(ALT0, "\n");
    }

    void resetLscore() {
        for(int i = 0; i < assoc; i++) {
            Lscore[i] = 0;
        }
    }

    int getLscore() {
        return Lscore[assoc-1];
    }
};

#endif
