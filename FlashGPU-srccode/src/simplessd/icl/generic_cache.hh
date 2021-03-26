/*
 * Copyright (C) 2017 CAMELab
 *
 * This file is part of SimpleSSD.
 *
 * SimpleSSD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SimpleSSD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleSSD.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ICL_GENERIC_CACHE__
#define __ICL_GENERIC_CACHE__

#include <functional>
#include <random>
#include <vector>
#include <map>
#include <assert.h>

#include "icl/abstract_cache.hh"

namespace SimpleSSD {

namespace ICL {

class GenericCache : public AbstractCache {
 private:
  const uint32_t lineCountInSuperPage;
  const uint32_t superPageSize;
  const uint32_t lineSize;
  const uint32_t parallelIO;
  const uint32_t lineCountInMaxIO;
  uint32_t setSize;
  uint32_t waySize;
  uint32_t writeSetSize;
  uint32_t writeWaySize;

  const uint32_t prefetchIOCount;
  const float prefetchIORatio;

  const bool useReadCaching;
  const bool useWriteCaching;
  const bool useReadPrefetch;

  Request lastRequest;
  bool prefetchEnabled;
  uint32_t hitCounter;
  uint32_t accessCounter;

  EVICT_POLICY policy;
  std::function<uint32_t(uint32_t, uint64_t &)> evictFunction;
  std::function<Line *(Line *, Line *)> compareFunction;
  EVICT_POLICY writePolicy;
  std::function<uint32_t(uint32_t, uint64_t &)> writeEvictFunction;
  std::function<Line *(Line *, Line *)> writeCompareFunction;  
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_int_distribution<> dist;

  std::vector<Line *> cacheData;
  std::vector<Line *> writeCacheData;
  std::vector<Line **> evictData;

  typedef struct _TableEntry{
    uint64_t lastAddr;
    uint32_t length;
    uint32_t counter;
  }TableEntry;
  typedef struct _PredictTable{
    std::map<int, TableEntry> warpEntry;
    TableEntry globalEntry;
  }PredictTable;
  PredictTable predictTable[1024];

  uint32_t calcSetIndex(bool, uint64_t);
  void calcIOPosition(uint64_t, uint32_t &, uint32_t &);

  uint32_t getEmptyWay(bool, uint32_t, uint64_t &);
  uint32_t getValidWay(bool, uint64_t, uint64_t &);
  void checkPrefetch(Request &);
  uint32_t setEvictionPolicy(EVICT_POLICY, 
        std::function<uint32_t(uint32_t, uint64_t &)> &, 
        std::function<Line *(Line *, Line *)> &);

  void evictCache(uint64_t);

  // Stats
  struct {
    uint64_t request[2];
    uint64_t cache[2];
  } stat;

 public:
  GenericCache(ConfigReader *, FTL::FTL *, DRAM::AbstractDRAM *);
  ~GenericCache();

  bool read(Request &, uint64_t &) override;
  bool write(Request &, uint64_t &) override;
  bool flush(Request &, uint64_t &) override;
  bool trim(Request &, uint64_t &) override;

  void format(LPNRange &, uint64_t &) override;

  void checkAvailableTime(Request &, uint64_t &) override;

  void getStats(std::vector<Stats> &) override;
  void getStatValues(std::vector<uint64_t> &) override;
  void resetStats() override;
};

}  // namespace ICL

}  // namespace SimpleSSD

#endif
