/**
 * Copyright (c) 2019, 2020 Inria
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
 * Declaration of a the Hawkeye Replacement Policy, as described in "Hawkeye:
 * Signature-based Hit Predictor for High Performance Caching", by
 * Wu et al.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_SHIP_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_SHIP_RP_HH__

#include <cstddef>
#include <vector>
#include <map>

#include "base/compiler.hh"
#include "base/sat_counter.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/packet.hh"
#include "params/HawkeyeRP.hh"  

// Hawkeye OPTGen
#include "optgen.h"

// defines Hawkeye
#define MAX_SHCT 31
#define SHCT_SIZE_BITS 13 
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

namespace gem5
{

struct HawkeyeRPParams;

namespace replacement_policy
{

class Hawkeye : public Base
{
  protected:
    typedef std::size_t SignatureType;
    /** Signature to be used when no PC is provided in an access. */
    const SignatureType NO_PC_SIGNATURE = 0;

    /** Hawkeye-specific implementation of replacement data. */
    struct HawkeyeReplData : ReplacementData
    {
      // private:
      //   /** Signature that caused the insertion of this entry. */

      // public:
        SignatureType signature; // this is not assigned any value but used ! 

        bool valid;
        uint32_t rrpv;
        HawkeyeReplData(const int num_bits) : rrpv(num_bits), signature(0), valid(false)
        {
        }

        /** Get entry's signature. */
        SignatureType getSignature() const;

        /**
         * Set this entry's signature and reset outcome.
         *
         * @param signature New signature value/
         */
        void setSignature(SignatureType signature);
    };

    const double insertionThreshold;

    /**
     * Signature History Counter Table; learns the re-reference behavior
     * of a signature. A zero entry provides a strong indication that
     * future lines brought by that signature will not receive any hits.

     demand_SHCT - demand based accesses only. 
     Prefetcher accesses to be covered later in this project (Hopefully :P)
     */
    std::vector<uint64_t> demand_SHCT;
    std::vector<uint64_t> prefetch_SHCT;
  
    // Hawkeye implementation requirements : OPTGen, addr_history, perset_timer 
    std::vector<OPTgen>                     perset_optgen; //OPTGen Structure
    std::vector<uint64_t>                   perset_timer;  //holds the timestamp of access in a per set basis
    std::vector<std::map<uint64_t, ADDR_INFO> >  addr_history;  //addr_history is an array tag and ADDR_INFO. It is a part of the sampled cache design

    uint64_t signatures[LLC_SETS][LLC_WAYS];
    uint32_t rrpv[LLC_SETS][LLC_WAYS];

    uint32_t curr_set, curr_way;

    /**
     * Extract signature from packet.
     *
     * @param pkt The packet to extract a signature from.
     * @return The signature extracted.
     */
    virtual SignatureType getSignature(const PacketPtr pkt) const = 0;

    uint64_t CRC( uint64_t _blockAddress );

  public:
    typedef HawkeyeRPParams Params;
    Hawkeye(const Params &p);
    ~Hawkeye() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Updates predictor and invalidate data.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;

    /**
     * Touch an entry to update its replacement data.
     * Updates predictor and assigns RRPV values of Table 3.
     *
     * @param replacement_data  Replacement data to be touched.
     * @param pkt Packet that generated this hit.
     */

    void touch(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
        
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
        override;

    /**
     * Reset replacement data. Used when an entry is inserted.
     * Updates predictor and assigns RRPV values of Table 3.
     *
     * @param replacement_data Replacement data to be reset.
     * @param pkt Packet that generated this miss.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
        override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;

    // increments the trainer Signature indexed counter value
    void demand_SHCT_increment (uint64_t pc);

    // decrement the trainer Signature indexed counter value
    void demand_SHCT_decrement (uint64_t pc);

    // gets the prediction from the counter value indexed through the signature
    bool demand_SHCT_get_prediction (uint64_t pc);

    // SignatureType getSignature(const PacketPtr pkt) const;

    void UpdateReplacementState (uint32_t set, uint32_t way, uint64_t paddr, uint64_t PC, uint32_t type, uint8_t hit);

    void replace_addr_history_element(unsigned int sampler_set);
    void update_addr_history_lru(unsigned int sampler_set, unsigned int curr_lru);

    void set_current_cache_block_data(uint32_t set, uint32_t way);

    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const override;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_SHIP_RP_HH__


