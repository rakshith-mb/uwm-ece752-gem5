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

/**
 * @file
 * Declaration of a Least Recently Used replacement policy.
 * The victim is chosen using the last touch timestamp.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_BRRIP_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_BRRIP_RP_HH__

#include "mem/cache/replacement_policies/base.hh"
#include "optgen.h"
// defines Hawkeye
#define MAX_SHCT 31
#define SHCT_SIZE_BITS 11
#define SHCT_SIZE (1<<SHCT_SIZE_BITS)
#define TIMER_SIZE 1024
#define maxRRPV 7
#define NUM_CORE 1
// the following two values has to come from the params file
// CHANGE NEEDED
#define LLC_SETS NUM_CORE*2048
#define LLC_WAYS 16

// Sampler to track 8x cache history for sampled sets
// 2800 entris * 4 bytes per entry = 11.2KB
#define SAMPLED_CACHE_SIZE 2800
#define SAMPLER_WAYS 8
#define SAMPLER_SETS SAMPLED_CACHE_SIZE/SAMPLER_WAYS


#define bitmask(l) (((l) == 64) ? (unsigned long long)(-1LL) : ((1LL << (l))-1LL))
#define bits(x, i, l) (((x) >> (i)) & bitmask(l))
//Sample 64 sets per core
#define SAMPLED_SET(set) (bits(set, 0 , 6) == bits(set, ((unsigned long long)log2(LLC_SETS) - 6), 6) )

// Added by Us
#define CACHE_HIT   1
#define CACHE_MISS  0
#define DEMAND      8
#define PREFETCH    9
#define WRITEBACK   10
// #define shct_size 16384
// #define numRRPVBits 64
#define  insertion_threshold 1

namespace gem5
{

struct BRRIPRPParams;

namespace replacement_policy
{

class BRRIP : public Base
{
  typedef std::size_t SignatureType;

  protected:
    /** BRRIP-specific implementation of replacement data. */
    struct BRRIPReplData : ReplacementData
    {
        /** Tick on which the entry was last touched. */

        // bool valid;
        /**
         * Default constructor. Invalidate data.
         */
        BRRIPReplData() {
          
        }

    };

  public:
    typedef BRRIPRPParams Params;
    BRRIP(const Params &p);
    ~BRRIP() = default;
    const double insertionThreshold;
    std::map<uint64_t, uint32_t> demand_SHCT;
    std::map<uint64_t, uint32_t> prefetch_SHCT;
  
    // Hawkeye implementation requirements : OPTGen, addr_history, perset_timer 
    std::vector<OPTgen>                     perset_optgen; //OPTGen Structure
    std::vector<uint64_t>                   perset_timer;  //holds the timestamp of access in a per set basis
    std::vector<std::map<uint64_t, ADDR_INFO> >  addr_history;  //addr_history is an array tag and ADDR_INFO. It is a part of the sampled cache design

    uint64_t signatures[LLC_SETS][LLC_WAYS];
    uint32_t rrpv[LLC_SETS][LLC_WAYS];

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Sets its last touch tick as the starting tick.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;
    uint64_t CRC( uint64_t _blockAddress );
    /**
     * Touch an entry to update its replacement data.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;
    void touch(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
    /**
     * Reset replacement data. Used when an entry is inserted.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;
    void reset(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
    /**
     * Find replacement victim using BRRIP timestamps.
     *
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const
                                                                     override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
    // increments the trainer Signature indexed counter value
    void demand_SHCT_increment (uint64_t pc) ;

    // decrement the trainer Signature indexed counter value
    void demand_SHCT_decrement (uint64_t pc);

    // gets the prediction from the counter value indexed through the signature
    bool demand_SHCT_get_prediction (uint64_t pc);

    SignatureType getSignature(const PacketPtr pkt) const;

    void UpdateReplacementState (uint32_t set, uint32_t way, uint64_t paddr, uint64_t PC, uint32_t type, uint8_t hit);

    void replace_addr_history_element(unsigned int sampler_set);
    void update_addr_history_lru(unsigned int sampler_set, unsigned int curr_lru);

    void set_current_cache_block_data(uint32_t set, uint32_t way);
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_BRRIP_RP_HH__
