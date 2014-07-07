#ifndef __SAMPLER_CC__
#define __SAMPLER_CC__

#include <string>

//#include "mem/cache/base.hh"
//#include "mem/cache/cache.hh"
#include "sim/core.hh"
#include "base/callback.hh"
#include "base/statistics.hh"
#include "sim/clocked_object.hh"
#include "mem/packet.hh"
#include "debug/ALT3.hh"


class BaseCache;

class sampler_entry {
  public:
    enum LA {
        Invalid = 0,
        ReadMiss,
        ReadHit,
        WriteMiss,
        WriteHit
    };

    bool valid;
    Addr tag;
    Addr signature;
    LA last_access;

    sampler_entry() : valid(false), tag(-1), signature(-1), last_access(Invalid)
    {} 

    void invalidate() {
        valid = false;
        tag = -1;
        signature = -1;
        last_access = Invalid;
    }
};

class sampler_set {
  public:
    int assoc;
    sampler_entry **entries;

    sampler_entry* findEntry(Addr tag) {
        for (int i = 0; i < assoc; ++i) {
            if(entries[i]->tag == tag && entries[i]->valid) {
                return entries[i];
            }
        }
        return NULL;
    }

    void moveToHead(sampler_entry* entry) {
        if (entries[0] == entry)
            return;

        int i = 0;
        sampler_entry *next = entry;

        do {
            assert(i < assoc);
            // swap entries[i] and next
            sampler_entry *tmp = entries[i];
            entries[i] = next;
            next = tmp;
            ++i;
        } while (next != entry);
    }

    void moveToTail(sampler_entry* entry) {
        if (entries[assoc - 1] == entry)
            return;

        int i = assoc - 1;
        sampler_entry *next = entry;

        do {
            assert(i >= 0);
            // swap entries[i] and next
            sampler_entry *tmp = entries[i];
            entries[i] = next;
            next = tmp;
            --i;
        } while (next != entry);
    }

};

class sampler {
  public:
    sampler_set *sets;
    sampler_entry *entries;
    int *predTable;
    unsigned assoc;
    unsigned numSets;
    unsigned numPredEntry;
    long signature_mask;

    sampler(unsigned _assoc, unsigned _numSets, unsigned _numPredEntry) {
        assoc = _assoc;
        numSets = _numSets;
        numPredEntry = _numPredEntry;
        signature_mask = 65535;
        
        sets = new sampler_set[numSets];
        entries = new sampler_entry[numSets * assoc];
        predTable = new int[numPredEntry];

        unsigned entryIndex = 0;
        for (unsigned i = 0; i < numSets; ++i) {
            sets[i].assoc = assoc;
            sets[i].entries = new sampler_entry*[assoc];

            for (unsigned j = 0; j < assoc; ++j) {
                sampler_entry *entry = &entries[entryIndex];
                ++entryIndex;

                entry->invalidate();

                entry->tag = j;
                sets[i].entries[j] = entry;
            }
        }
        
        for (unsigned k = 0; k < numPredEntry; ++k) {
            predTable[k] = 0;
        }
    }

    void updateSampler(PacketPtr pkt, int set, Addr tag) {
        //only update if set < numSets
        DPRINTF(ALT3,"addr %x update sampler\n",pkt->getAddr());
        if(set < numSets) {
            assert(pkt->getLA() != -1);
            //update sampler entry
            //check whether hit or miss
            sampler_entry *entry = sets[set].findEntry(tag);

            //hit
            if(entry != NULL) {
                sets[set].moveToHead(entry);
                sampler_entry::LA event = (pkt->isWrite()) ? (sampler_entry::WriteHit) : (sampler_entry::ReadHit);
                //update predict table based on the event
                if(event == (sampler_entry::WriteHit) && entry->last_access == (sampler_entry::ReadMiss)) {
                    if(predTable[entry->signature] != 7) {
                        predTable[entry->signature]++;
                    }
                    DPRINTF(ALT3,"addr %x update dead, current pred value %d, current event write hit, last event read miss\n",pkt->getLA(), predTable[entry->signature]);
                }
                else if(entry->last_access == (sampler_entry::ReadMiss) && event == (sampler_entry::ReadHit)) {
                    if(predTable[entry->signature] != 0) {
                        predTable[entry->signature]--;
                    }
                    DPRINTF(ALT3,"addr %x update live, current pred value %d, current event read hit, last event read miss\n",pkt->getLA(), predTable[entry->signature]);
                }
                else if(entry->last_access == (sampler_entry::WriteHit) || entry->last_access == (sampler_entry::WriteMiss)) {
                    if(predTable[entry->signature] != 0) {
                        predTable[entry->signature]--;
                    }
                    DPRINTF(ALT3,"addr %x update live, current pred value %d, current event hit, last event write\n",pkt->getLA(), predTable[entry->signature]);
                }
                else {
                    DPRINTF(ALT3, "addr %x update ignore, current pred value %d\n",pkt->getLA(), predTable[entry->signature]);
                }
                //assign current event to last access of the entry
                entry->last_access = event;
                //assign new signature
                entry->signature = (pkt->getLA()) & signature_mask;
            }
            //miss
            else {
                sampler_entry *victim = sets[set].entries[assoc - 1];
                
                if(victim->valid) {
                    //event: eviction, update predict table
                    if(victim->last_access == (sampler_entry::ReadMiss) || victim->last_access == (sampler_entry::WriteHit) || victim->last_access == (sampler_entry::WriteMiss)) {
                        if(predTable[victim->signature] != 7) {
                            predTable[victim->signature]++;
                        }
                        DPRINTF(ALT3,"addr %x update dead, current pred value %d, current event eviction, last event Read miss or write\n",pkt->getLA(),predTable[victim->signature]);
                    }
                    else {
                        DPRINTF(ALT3,"addr %x update ignore, current pred value %d, current event eviction, last event read hit\n",pkt->getLA(), predTable[victim->signature]);

                    }
                    victim->invalidate();
                }

                victim->tag = tag;
                victim->valid = true;
                victim->signature = (pkt->getLA()) & signature_mask;
                sampler_entry::LA event = (pkt->isWrite()) ? (sampler_entry::WriteMiss) : (sampler_entry::ReadMiss);
                victim->last_access = event;
                sets[set].moveToHead(victim);
            }
            
        }
    }

    bool getPredDecision(PacketPtr pkt) {
        int index = pkt->getLA() & signature_mask;
        DPRINTF(ALT3,"get pred decision for addr %x, value %d\n",pkt->getLA(), predTable[index]);
        if(predTable[index] >= 6) {
            return true;
        }
        else {
            return false;
        }
    }
};

#endif
