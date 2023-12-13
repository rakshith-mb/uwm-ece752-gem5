/**
 * Copyright (c) 2018-2020 Inria
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
 */

#include "mem/cache/replacement_policies/new_rp.hh"

#include <cassert>
#include <memory>
#include <math.h>

#include "params/BRRIPRP.hh"
#include "sim/cur_tick.hh"


namespace gem5
{

namespace replacement_policy
{

BRRIP::BRRIP(const Params &p)
  : Base(p), insertionThreshold(insertion_threshold / 100.0),
    demand_SHCT(shct_size, uint64_t(numRRPVBits)),
    perset_optgen(LLC_SETS),
    perset_timer(LLC_SETS,0) // creating NUM_SETS entries with initial value as 0
{
    // We are going to assume that Params has a field called 
    //          1. NUM_SETS which says the total number of sets
    //          2. NUM_WAYS which says the total number of ways
    for(int i=0;i<LLC_SETS;i++){
        perset_optgen[i].init(LLC_WAYS-2);
    }

    addr_history.resize(SAMPLER_SETS);
    for (int i=0; i<SAMPLER_SETS; i++) 
        addr_history[i].clear();

    // printf("\n\n\nNew RP Constructor is called here \n");
}

BRRIP::SignatureType
BRRIP::BRRIPReplData::getSignature() const
{
    return signature;
}

void
BRRIP::BRRIPReplData::setSignature(SignatureType new_signature)
{
    signature = new_signature;
}
void
BRRIP::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    // Reset last touch timestamp
    std::static_pointer_cast<BRRIPReplData>(
        replacement_data)->lastTouchTick = Tick(0);
    std::static_pointer_cast<BRRIPReplData>(
        replacement_data)->valid = false;
}

void
BRRIP::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Update last touch timestamp
    std::static_pointer_cast<BRRIPReplData>(
        replacement_data)->lastTouchTick = curTick();
}

void
BRRIP::touch(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    // printf("\n\n\nNew RP touch is called here \n");

    std::shared_ptr<BRRIPReplData> casted_replacement_data =
        std::static_pointer_cast<BRRIPReplData>(replacement_data);

    // Get signature
    const SignatureType signature = getSignature(pkt);

    // When a hit happens the SHCT entry indexed by the signature is
    // incremented
    // SHCT[signature]++;

    UpdateReplacementState(replacement_data->_set, replacement_data->_way, pkt->getAddr(), signature, DEMAND, CACHE_HIT);

    // // This was a hit; update replacement data accordingly
    // BRRIP::touch(replacement_data);
}

void
BRRIP::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set last touch timestamp
    std::static_pointer_cast<BRRIPReplData>(
        replacement_data)->lastTouchTick = curTick();
}
void
BRRIP::reset(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    std::shared_ptr<BRRIPReplData> casted_replacement_data =
        std::static_pointer_cast<BRRIPReplData>(replacement_data);

    // Get signature
    const SignatureType signature = getSignature(pkt);


    // // Store signature
    casted_replacement_data->setSignature(signature);

    UpdateReplacementState(casted_replacement_data->_set, casted_replacement_data->_way, pkt->getAddr(), signature, DEMAND, CACHE_MISS);

    // If SHCT for signature is set, predict intermediate re-reference.
    // Predict distant re-reference otherwise
    // BRRIP::reset(replacement_data);
    // if (SHCT[signature].calcSaturation() >=  ) {
    //     casted_replacement_data->rrpv--;
    // }
}
ReplaceableEntry*
BRRIP::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Use first candidate as dummy victim
    ReplaceableEntry* victim = candidates[0];

    // Store victim->rrpv in a variable to improve code readability
    int victim_RRPV = std::static_pointer_cast<BRRIPReplData>(
                        victim->replacementData)->rrpv;

    // Visit all candidates to find victim
    for (const auto& candidate : candidates) {
        std::shared_ptr<BRRIPReplData> candidate_repl_data =
            std::static_pointer_cast<BRRIPReplData>(
                candidate->replacementData);

        // Stop searching for victims if an invalid entry is found
        if (!candidate_repl_data->valid) {
            return candidate;
        }
        if (candidate_repl_data->rrpv == maxRRPV){
            victim = candidate;
            return victim;          
        }

        // Update victim entry if necessary
        int candidate_RRPV = candidate_repl_data->rrpv;
        if (candidate_RRPV > victim_RRPV) {
            victim = candidate;
            victim_RRPV = candidate_RRPV;
        }
    }

    return victim;
}
// Hashing function to index into demand_SHCT structure- 
//      see if it is required. 
//      SHiP does not have any hashing to index into SCHT structure
uint64_t BRRIP::CRC( uint64_t _blockAddress )
{
    static const unsigned long long crcPolynomial = 3988292384ULL;
    unsigned long long _returnVal = _blockAddress;
    for( unsigned int i = 0; i < 32; i++ )
        _returnVal = ( ( _returnVal & 1 ) == 1 ) ? ( ( _returnVal >> 1 ) ^ crcPolynomial ) : ( _returnVal >> 1 );
    return _returnVal;
}
void BRRIP::demand_SHCT_increment (uint64_t pc)
{
    uint64_t signature = CRC(pc) % SHCT_SIZE;
    if (std::find(demand_SHCT.begin(), demand_SHCT.end(), signature) == demand_SHCT.end())
        demand_SHCT[signature] = (1+MAX_SHCT)/2;

    demand_SHCT[signature] = (demand_SHCT[signature] < MAX_SHCT) ? (demand_SHCT[signature]+1) : MAX_SHCT;
}

void BRRIP::demand_SHCT_decrement (uint64_t pc)
{
    uint64_t signature = CRC(pc) % SHCT_SIZE;
    if(std::find(demand_SHCT.begin(), demand_SHCT.end(), signature) == demand_SHCT.end())
        demand_SHCT[signature] = (1+MAX_SHCT)/2;
    if(demand_SHCT[signature] != 0)
        demand_SHCT[signature] = demand_SHCT[signature]-1;
}

bool BRRIP::demand_SHCT_get_prediction (uint64_t pc)
{
    uint64_t signature = CRC(pc) % SHCT_SIZE;
    if(std::find(demand_SHCT.begin(), demand_SHCT.end(), signature) != demand_SHCT.end() && demand_SHCT[signature] < ((MAX_SHCT+1)/2))
        return false;
    return true;
}
std::shared_ptr<ReplacementData>
BRRIP::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new BRRIPReplData());
}

BRRIP::SignatureType
BRRIP::getSignature(const PacketPtr pkt) const
{
    SignatureType signature;

    if (pkt->req->hasPC()) {
        signature = static_cast<SignatureType>(pkt->req->getPC());
    } else {
        signature = (SignatureType)0;
    }

    return signature % demand_SHCT.size();
}

void BRRIP::replace_addr_history_element(unsigned int sampler_set)
{
    uint64_t lru_addr = 0;
    
    for(std::map<uint64_t, ADDR_INFO>::iterator it=addr_history[sampler_set].begin(); it != addr_history[sampler_set].end(); it++)
    {
   //     uint64_t timer = (it->second).last_quanta;

        if((it->second).lru == (SAMPLER_WAYS-1))
        {
            //lru_time =  (it->second).last_quanta;
            lru_addr = it->first;
            break;
        }
    }

    addr_history[sampler_set].erase(lru_addr);
}

void BRRIP::update_addr_history_lru(unsigned int sampler_set, unsigned int curr_lru)
{
    for(std::map<uint64_t, ADDR_INFO>::iterator it=addr_history[sampler_set].begin(); it != addr_history[sampler_set].end(); it++)
    {
        if((it->second).lru < curr_lru)
        {
            (it->second).lru++;
            assert((it->second).lru < SAMPLER_WAYS); 
        }
    }
}

void BRRIP::set_current_cache_block_data(uint32_t set, uint32_t way)
{
    curr_set = set;
    curr_way = way;
}


// called on every cache hit and cache fill
void BRRIP::UpdateReplacementState (uint32_t set, uint32_t way, uint64_t paddr, uint64_t PC, uint32_t type, uint8_t hit)
{
    paddr = (paddr >> 6) << 6;

    // if(type == PREFETCH)
    // {
    //     if (!hit)
    //         prefetched[set][way] = true;
    // }
    // else
    //     prefetched[set][way] = false;

    // //Ignore writebacks
    // if (type == WRITEBACK)
    //     return;


    // //If we are sampling, OPTgen will only see accesses from sampled sets
    if(SAMPLED_SET(set))
     {
        //The current timestep 
        uint64_t curr_quanta = perset_timer[set] % OPTGEN_VECTOR_SIZE;  //rsuresh6 the current timestamp - the current index 

        uint32_t sampler_set = (paddr >> 6) % SAMPLER_SETS;  // rsuresh6  will give which set we need to look at 
        uint64_t sampler_tag = CRC(paddr >> 12) % 256;       // rsuresh6  tag of the address
        assert(sampler_set < SAMPLER_SETS); 

        // This line has been used before. Since the right end of a usage interval is always 
        // a demand, ignore prefetches
        if((addr_history[sampler_set].find(sampler_tag) != addr_history[sampler_set].end()) && (type != PREFETCH))// rsuresh6  will check if sampler_tag is present in the address history - if not present then we cannot 
        {
            unsigned int curr_timer = perset_timer[set];
            if(curr_timer < addr_history[sampler_set][sampler_tag].last_quanta)
               curr_timer = curr_timer + TIMER_SIZE;
            bool wrap =  ((curr_timer - addr_history[sampler_set][sampler_tag].last_quanta) > OPTGEN_VECTOR_SIZE);
            uint64_t last_quanta = addr_history[sampler_set][sampler_tag].last_quanta % OPTGEN_VECTOR_SIZE;
            //and for prefetch hits, we train the last prefetch trigger PC
            if( !wrap && perset_optgen[set].should_cache(curr_quanta, last_quanta))
            {
                if(addr_history[sampler_set][sampler_tag].prefetched)
                    // prefetch_SHCT.increment(addr_history[sampler_set][sampler_tag].PC);
                    ;
                else
                    demand_SHCT_increment(addr_history[sampler_set][sampler_tag].PC);
            }
            else
            {
                //Train the predictor negatively because OPT would not have cached this line
                if(addr_history[sampler_set][sampler_tag].prefetched)
                    // prefetch_SHCT.decrement(addr_history[sampler_set][sampler_tag].PC);
                    ;
                else
                    demand_SHCT_decrement(addr_history[sampler_set][sampler_tag].PC);
            }
            //Some maintenance operations for OPTgen
            assert (set < 2048);
            perset_optgen[set].add_access(curr_quanta);
            update_addr_history_lru(sampler_set, addr_history[sampler_set][sampler_tag].lru);

            //Since this was a demand access, mark the prefetched bit as false
            addr_history[sampler_set][sampler_tag].prefetched = false;
        }
        // This is the first time we are seeing this line (could be demand or prefetch)
        else if(addr_history[sampler_set].find(sampler_tag) == addr_history[sampler_set].end())
        {
        //     // Find a victim from the sampled cache if we are sampling
            if(addr_history[sampler_set].size() == SAMPLER_WAYS) 
                replace_addr_history_element(sampler_set);

            assert(addr_history[sampler_set].size() < SAMPLER_WAYS);
            //Initialize a new entry in the sampler
            addr_history[sampler_set][sampler_tag].init(curr_quanta);
        //     //If it's a prefetch, mark the prefetched bit;
        //     if(type == PREFETCH)
        //     {
        //         addr_history[sampler_set][sampler_tag].mark_prefetch();
        //         perset_optgen[set].add_prefetch(curr_quanta);
        //     }
        //     else
                perset_optgen[set].add_access(curr_quanta);
            update_addr_history_lru(sampler_set, SAMPLER_WAYS-1);
        }
        else //This line is a prefetch
        {
            assert(addr_history[sampler_set].find(sampler_tag) != addr_history[sampler_set].end());
            //if(hit && prefetched[set][way])
            uint64_t last_quanta = addr_history[sampler_set][sampler_tag].last_quanta % OPTGEN_VECTOR_SIZE;
            if (perset_timer[set] - addr_history[sampler_set][sampler_tag].last_quanta < 5*NUM_CORE) 
            {
                if(perset_optgen[set].should_cache(curr_quanta, last_quanta))
                {
                    if(addr_history[sampler_set][sampler_tag].prefetched)
                        // prefetch_SHCT.increment(addr_history[sampler_set][sampler_tag].PC);
                        ;
                    else
                       demand_SHCT_increment(addr_history[sampler_set][sampler_tag].PC);
                }
            }

            //Mark the prefetched bit
            addr_history[sampler_set][sampler_tag].mark_prefetch(); 
            //Some maintenance operations for OPTgen
            perset_optgen[set].add_prefetch(curr_quanta);
            update_addr_history_lru(sampler_set, addr_history[sampler_set][sampler_tag].lru);
        }

        // Get Hawkeye's prediction for this line
        bool new_prediction = demand_SHCT_get_prediction (PC);
        //if (type == PREFETCH)
            // new_prediction = prefetch_SHCT.get_prediction (PC);
        // Update the sampler with the timestamp, PC and our prediction
        // For prefetches, the PC will represent the trigger PC
        addr_history[sampler_set][sampler_tag].update(perset_timer[set], PC, new_prediction);
        addr_history[sampler_set][sampler_tag].lru = 0;
        //Increment the set timer
        perset_timer[set] = (perset_timer[set]+1) % TIMER_SIZE;
    }

    bool new_prediction = demand_SHCT_get_prediction (PC);
    // //if (type == PREFETCH)
    //     // new_prediction = prefetch_SHCT.get_prediction (PC);

    signatures[set][way] = PC;

    // // //Set RRIP values and age cache-friendly line
    // if(!new_prediction)
    //      rrpv[set][way] = maxRRPV;
    // else
    // {
    //     rrpv[set][way] = 0;
    //     if(!hit)
    //     {
    //         bool saturated = false;
    //         for(uint32_t i=0; i<LLC_WAYS; i++)
    //             if (rrpv[set][i] == maxRRPV-1)
    //                 saturated = true;

    //         //Age all the cache-friendly  lines
    //         for(uint32_t i=0; i<LLC_WAYS; i++)
    //         {
    //             if (!saturated && rrpv[set][i] < maxRRPV-1)
    //                 rrpv[set][i]++;
    //         }
    //     }
    //     rrpv[set][way] = 0;
    // }
}
} // namespace replacement_policy
} // namespace gem5
