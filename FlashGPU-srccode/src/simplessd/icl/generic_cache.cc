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

#include "icl/generic_cache.hh"

#include <algorithm>
#include <cstddef>
#include <limits>

#include "log/trace.hh"
#include "util/algorithm.hh"

namespace SimpleSSD {

namespace ICL {

#define CACHE_DELAY 20

GenericCache::GenericCache(ConfigReader *c, FTL::FTL *f, DRAM::AbstractDRAM *d)
    : AbstractCache(c, f, d),
      lineCountInSuperPage(f->getInfo()->ioUnitInPage),
      superPageSize(f->getInfo()->pageSize),
      lineSize(superPageSize / lineCountInSuperPage),
      parallelIO(f->getInfo()->pageCountToMaxPerf),
      lineCountInMaxIO(parallelIO * lineCountInSuperPage),
      waySize(c->iclConfig.readUint(ICL_WAY_SIZE)),
      writeWaySize(c->iclConfig.readUint(ICL_WRITE_WAY_SIZE)),
      prefetchIOCount(c->iclConfig.readUint(ICL_PREFETCH_COUNT)),
      prefetchIORatio(c->iclConfig.readFloat(ICL_PREFETCH_RATIO)),
      useReadCaching(c->iclConfig.readBoolean(ICL_USE_READ_CACHE)),
      useWriteCaching(c->iclConfig.readBoolean(ICL_USE_WRITE_CACHE)),
      useReadPrefetch(c->iclConfig.readBoolean(ICL_USE_READ_PREFETCH)),
      gen(rd()),
      dist(std::uniform_int_distribution<>(0, waySize - 1)) {
  uint64_t cacheSize = c->iclConfig.readUint(ICL_CACHE_SIZE);
  uint64_t writeCacheSize = c->iclConfig.readUint(ICL_WRITE_CACHE_SIZE);

  printf("parallelIO %d, lineCountInSuperPage %d, lineCountInMaxIO %d\n",
          parallelIO, lineCountInSuperPage, lineCountInMaxIO);

  if (!useReadCaching && !useWriteCaching) {
    return;
  }

  // Fully-associated?
  if (waySize == 0) {
    setSize = 1;
    waySize = MAX(cacheSize / lineSize, 1);
  }
  else {
    setSize = MAX(cacheSize / lineSize / waySize, 1);
  }
  if (writeWaySize == 0) {
    writeSetSize = 1;
    writeWaySize = MAX(cacheSize / lineSize, 1);
  }
  else {
    writeSetSize = MAX(cacheSize / lineSize / writeWaySize, 1);
  }  

  Logger::debugprint(
      Logger::LOG_ICL_GENERIC_CACHE,
      "CREATE  | READ CACHE | Set size %u | Way size %u | Line size %u | Capacity %" PRIu64,
      setSize, waySize, lineSize, (uint64_t)setSize * waySize * lineSize);
  Logger::debugprint(
      Logger::LOG_ICL_GENERIC_CACHE,
      "CREATE  | WRITE CACHE | Set size %u | Way size %u | Line size %u | Capacity %" PRIu64,
      writeSetSize, writeWaySize, lineSize, (uint64_t)writeSetSize * writeWaySize * lineSize);      
  Logger::debugprint(
      Logger::LOG_ICL_GENERIC_CACHE,
      "CREATE  | line count in super page %u | line count in max I/O %u",
      lineCountInSuperPage, lineCountInMaxIO);

  cacheData.resize(setSize);

  for (uint32_t i = 0; i < setSize; i++) {
    cacheData[i] = new Line[waySize]();
  }

  writeCacheData.resize(writeSetSize);

  for (uint32_t i = 0; i < writeSetSize; i++) {
    writeCacheData[i] = new Line[writeWaySize]();
  }  

  evictData.resize(lineCountInSuperPage);

  for (uint32_t i = 0; i < lineCountInSuperPage; i++) {
    evictData[i] = (Line **)calloc(parallelIO, sizeof(Line *));
  }

  lastRequest.reqID = 1;
  prefetchEnabled = false;
  hitCounter = 0;
  accessCounter = 0;

  // Set evict policy functional
  policy = (EVICT_POLICY)c->iclConfig.readInt(ICL_EVICT_POLICY);
  setEvictionPolicy(policy, evictFunction, compareFunction);
  writePolicy = (EVICT_POLICY)c->iclConfig.readInt(ICL_WRITE_EVICT_POLICY);
  setEvictionPolicy(writePolicy, writeEvictFunction, writeCompareFunction);

  memset(&stat, 0, sizeof(stat));

  for (uint32_t i = 0; i < 1024; i++){
    predictTable[i].globalEntry.lastAddr = (uint64_t)-1;
    predictTable[i].globalEntry.counter = 0;
  }
}

GenericCache::~GenericCache() {
  for (uint32_t i = 0; i < setSize; i++) {
    delete[] cacheData[i];
  }
  for (uint32_t i = 0; i < writeSetSize; i++) {
    delete[] writeCacheData[i];
  }  

  for (uint32_t i = 0; i < lineCountInSuperPage; i++) {
    free(evictData[i]);
  }
}

uint32_t GenericCache::setEvictionPolicy(EVICT_POLICY policy, 
        std::function<uint32_t(uint32_t, uint64_t &)> &evictFunction, 
        std::function<Line *(Line *, Line *)> &compareFunction)
{
  switch (policy) {
    case POLICY_RANDOM:
      evictFunction = [this](uint32_t setIdx, uint64_t &tick) -> uint32_t {
        return dist(gen);
      };
      compareFunction = [this](Line *a, Line *b) -> Line * {
        if (a && b) {
          return dist(gen) > waySize / 2 ? a : b;
        }
        else if (a || b) {
          return a ? a : b;
        }
        else {
          return nullptr;
        }
      };

      break;
    case POLICY_FIFO:
      evictFunction = [this](uint32_t setIdx, uint64_t &tick) -> uint32_t {
        uint32_t wayIdx = 0;
        uint64_t min = std::numeric_limits<uint64_t>::max();

        for (uint32_t i = 0; i < waySize; i++) {
          tick += CACHE_DELAY * 8;
          // pDRAM->read(MAKE_META_ADDR(setIdx, i, offsetof(Line, insertedAt)),
          // 8, tick);

          if (cacheData[setIdx][i].insertedAt < min) {
            min = cacheData[setIdx][i].insertedAt;
            wayIdx = i;
          }
        }

        return wayIdx;
      };
      compareFunction = [](Line *a, Line *b) -> Line * {
        if (a && b) {
          if (a->insertedAt < b->insertedAt) {
            return a;
          }
          else {
            return b;
          }
        }
        else if (a || b) {
          return a ? a : b;
        }
        else {
          return nullptr;
        }
      };

      break;
    case POLICY_LEAST_RECENTLY_USED:
      evictFunction = [this](uint32_t setIdx, uint64_t &tick) -> uint32_t {
        uint32_t wayIdx = 0;
        uint64_t min = std::numeric_limits<uint64_t>::max();

        for (uint32_t i = 0; i < waySize; i++) {
          tick += CACHE_DELAY * 8;
          // pDRAM->read(MAKE_META_ADDR(setIdx, i, offsetof(Line,
          // lastAccessed)), 8, tick);

          if (cacheData[setIdx][i].lastAccessed < min) {
            min = cacheData[setIdx][i].lastAccessed;
            wayIdx = i;
          }
        }

        return wayIdx;
      };
      compareFunction = [](Line *a, Line *b) -> Line * {
        if (a && b) {
          if (a->lastAccessed < b->lastAccessed) {
            return a;
          }
          else {
            return b;
          }
        }
        else if (a || b) {
          return a ? a : b;
        }
        else {
          return nullptr;
        }
      };

      break;
    default:
      Logger::panic("Undefined cache evict policy");

      break;
  }
}


uint32_t GenericCache::calcSetIndex(bool isWrite, uint64_t lca) {
  if (!isWrite) return lca % setSize;
  else return lca % writeSetSize;
}

void GenericCache::calcIOPosition(uint64_t lca, uint32_t &row, uint32_t &col) {
  uint32_t tmp = lca % lineCountInMaxIO;

  row = tmp % lineCountInSuperPage;
  col = tmp / lineCountInSuperPage;
}

uint32_t GenericCache::getEmptyWay(bool isWrite, uint32_t setIdx, uint64_t &tick) {
  uint32_t retIdx = ((!isWrite)?waySize:writeWaySize);
  uint64_t minInsertedAt = std::numeric_limits<uint64_t>::max();

  for (uint32_t wayIdx = 0; wayIdx < ((!isWrite)?waySize:writeWaySize); wayIdx++) {
    Line &line = ((!isWrite)?cacheData[setIdx][wayIdx]:writeCacheData[setIdx][wayIdx]);

    if (!line.valid) {
      tick += CACHE_DELAY * 8;
      // pDRAM->read(MAKE_META_ADDR(setIdx, wayIdx, offsetof(Line, insertedAt)),
      // 8, tick);

      if (minInsertedAt > line.insertedAt) {
        minInsertedAt = line.insertedAt;
        retIdx = wayIdx;
      }
    }
  }

  return retIdx;
}

uint32_t GenericCache::getValidWay(bool isWrite, uint64_t lca, uint64_t &tick) {
  uint32_t setIdx = calcSetIndex(isWrite, lca);
  uint32_t wayIdx;

  for (wayIdx = 0; wayIdx < ((!isWrite)?waySize:writeWaySize); wayIdx++) {
    Line &line = ((!isWrite)?cacheData[setIdx][wayIdx]:writeCacheData[setIdx][wayIdx]);

    tick += CACHE_DELAY * 8;
    // pDRAM->read(MAKE_META_ADDR(setIdx, wayIdx, offsetof(Line, tag)), 8,
    // tick);

    if (line.valid && line.tag == lca) {
      break;
    }
  }

  return wayIdx;
}

void GenericCache::checkPrefetch(Request &req) {
  prefetchEnabled = false;
  if (lastRequest.reqID == req.reqID) {
    lastRequest.range = req.range;
    lastRequest.offset = req.offset;
    lastRequest.length = req.length;
    return;
  }
  if (predictTable[req.pc % 1024].warpEntry.find(req.thread_id) == 
                                    predictTable[req.pc % 1024].warpEntry.end()){
      TableEntry entry; 
      entry.lastAddr = req.range.slpn;
      entry.length = req.length;
      entry.counter = 0;
      predictTable[req.pc % 1024].warpEntry.insert(std::pair<int, TableEntry>(req.thread_id, entry));
  }
  else {
    if ((predictTable[req.pc % 1024].warpEntry)[req.thread_id].lastAddr / lineCountInSuperPage ==
      req.range.slpn / lineCountInSuperPage){
      if ((predictTable[req.pc % 1024].warpEntry)[req.thread_id].counter < 7)
        (predictTable[req.pc % 1024].warpEntry)[req.thread_id].counter++;
    }
    else {
      if ((predictTable[req.pc % 1024].warpEntry)[req.thread_id].counter > 0)
        (predictTable[req.pc % 1024].warpEntry)[req.thread_id].counter--;
    }  
    (predictTable[req.pc % 1024].warpEntry)[req.thread_id].lastAddr = req.range.slpn;
    if ((predictTable[req.pc % 1024].warpEntry)[req.thread_id].counter > 3)
      prefetchEnabled = true;        
  }

  if (predictTable[req.pc % 1024].globalEntry.lastAddr == (uint64_t)-1){
    predictTable[req.pc % 1024].globalEntry.lastAddr = req.range.slpn;
    predictTable[req.pc % 1024].globalEntry.length = req.length;
    predictTable[req.pc % 1024].globalEntry.counter = 0;        
  }
  else {
    if (predictTable[req.pc % 1024].globalEntry.lastAddr / lineCountInSuperPage ==
                req.range.slpn / lineCountInSuperPage){
      if (predictTable[req.pc % 1024].globalEntry.counter < 7)
        predictTable[req.pc % 1024].globalEntry.counter++;
    }
    else {
      if (predictTable[req.pc % 1024].globalEntry.counter > 0)
        predictTable[req.pc % 1024].globalEntry.counter--;      
    }
    predictTable[req.pc % 1024].globalEntry.lastAddr = req.range.slpn;
    if (predictTable[req.pc % 1024].globalEntry.counter > 3)
      prefetchEnabled = true;
  }

  lastRequest = req;
}

void GenericCache::evictCache(uint64_t tick) {
  FTL::Request reqInternal(lineCountInSuperPage);
  uint64_t beginAt;
  uint64_t finishedAt = tick;

  Logger::debugprint(Logger::LOG_ICL_GENERIC_CACHE, "----- | Begin eviction");

  for (uint32_t row = 0; row < lineCountInSuperPage; row++) {
    for (uint32_t col = 0; col < parallelIO; col++) {
      beginAt = tick;

      if (evictData[row][col] == nullptr) {
        continue;
      }

      if (evictData[row][col]->valid && evictData[row][col]->dirty) {
        reqInternal.lpn = evictData[row][col]->tag / lineCountInSuperPage;
        reqInternal.ioFlag.reset();
        reqInternal.ioFlag.set(row);

        pFTL->write(reqInternal, beginAt);
      }

      evictData[row][col]->insertedAt = beginAt;
      evictData[row][col]->lastAccessed = beginAt;
      evictData[row][col]->valid = false;
      evictData[row][col]->dirty = false;
      evictData[row][col]->tag = 0;
      evictData[row][col] = nullptr;

      finishedAt = MAX(finishedAt, beginAt);
    }
  }

  Logger::debugprint(Logger::LOG_ICL_GENERIC_CACHE,
                     "----- | End eviction | %" PRIu64 " - %" PRIu64
                     " (%" PRIu64 ")",
                     tick, finishedAt, finishedAt - tick);
}

void GenericCache::checkAvailableTime(Request &req, uint64_t &tick) {
  /* leave for future implementation */
  // uint64_t arrived = tick;
  // if (useReadCaching || useWriteCaching){
  //   uint32_t setIdx = calcSetIndex(req.range.slpn);
  //   uint32_t wayIdx;
  //   wayIdx = getValidWay(req.range.slpn, arrived);
  //   if (wayIdx != waySize) {
  //     tick = cacheData[setIdx][wayIdx].lastAccessed;
  //   } 
  //   else{
  //     wayIdx = getEmptyWay(setIdx, arrived);
  //     if (wayIdx == waySize) {
  //       wayIdx = evictFunction(setIdx, arrived);
  //       tick = cacheData[setIdx][wayIdx].lastAccessed;
  //     }     
  //   }   
  // }
}

// True when hit
bool GenericCache::read(Request &req, uint64_t &tick) {
  bool ret = false;
  bool isWrite = false;
  Logger::debugprint(Logger::LOG_ICL_GENERIC_CACHE,
                     "READ  | REQ %7u-%-4u | LCA %" PRIu64 " | SIZE %" PRIu64,
                     req.reqID, req.reqSubID, req.range.slpn, req.length);
  if (useWriteCaching) {
    uint32_t setIdx = calcSetIndex(!isWrite, req.range.slpn);
    uint32_t wayIdx;
    wayIdx = getValidWay(!isWrite, req.range.slpn, tick);
    if (wayIdx != waySize){
      uint64_t arrived = tick;

      // Wait cache to be valid
      if (tick < writeCacheData[setIdx][wayIdx].insertedAt) {
        tick = writeCacheData[setIdx][wayIdx].insertedAt;
      }
      // Update last accessed time
      writeCacheData[setIdx][wayIdx].lastAccessed = tick;

      // DRAM access
      pDRAM->read(&writeCacheData[setIdx][wayIdx], req.length, tick);
      Logger::debugprint(Logger::LOG_ICL_GENERIC_CACHE,
                         "READ  | Cache hit at (%u, %u) | %" PRIu64
                         " - %" PRIu64 " (%" PRIu64 ")",
                         setIdx, wayIdx, arrived, tick, tick - arrived);

      ret = true; 
      return ret;     
    }
  }

  if (useReadCaching) {
    uint32_t setIdx = calcSetIndex(isWrite, req.range.slpn);
    uint32_t wayIdx;

    if (useReadPrefetch) {
      checkPrefetch(req);
    }

    wayIdx = getValidWay(isWrite, req.range.slpn, tick);
    // Do we have valid data?
    if (wayIdx != waySize) {
      uint64_t arrived = tick;

      // Wait cache to be valid
      if (tick < cacheData[setIdx][wayIdx].insertedAt) {
        tick = cacheData[setIdx][wayIdx].insertedAt;
      }
      // Update last accessed time
      cacheData[setIdx][wayIdx].lastAccessed = tick;

      // DRAM access
      pDRAM->read(&cacheData[setIdx][wayIdx], req.length, tick);
      Logger::debugprint(Logger::LOG_ICL_GENERIC_CACHE,
                         "READ  | Cache hit at (%u, %u) | %" PRIu64
                         " - %" PRIu64 " (%" PRIu64 ")",
                         setIdx, wayIdx, arrived, tick, tick - arrived);

      ret = true;
    }
    // We should read data from NVM
    else {
      FTL::Request reqInternal(lineCountInSuperPage, req);
      std::vector<std::pair<uint64_t, uint64_t>> readList;
      uint32_t row, col;  // Variable for I/O position (IOFlag)
      uint64_t dramAt;
      uint64_t beginLCA, endLCA;
      uint64_t beginAt, finishedAt = tick;

      if (prefetchEnabled) {
        beginLCA = req.range.slpn;
        // endLCA = beginLCA + lineCountInMaxIO;
        endLCA = beginLCA + lineCountInSuperPage;
        printf("Jie: prefetch @ addr %lu\n", beginLCA);
      }
      else {
        beginLCA = req.range.slpn;
        endLCA = beginLCA + 1;
      }

      for (uint64_t lca = beginLCA; lca < endLCA; lca++) {
        // Check cache
        if (getValidWay(isWrite, lca, beginAt) != waySize) {
          continue;
        }
        if (getValidWay(!isWrite, lca, beginAt) != waySize) {
          continue;
        }
        // Find way to write data read from NVM
        setIdx = calcSetIndex(isWrite, lca);
        wayIdx = getEmptyWay(isWrite, setIdx, tick);

        if (wayIdx == waySize) {
          wayIdx = evictFunction(setIdx, tick);

          if (cacheData[setIdx][wayIdx].dirty) {
            // We need to evict data before write
            calcIOPosition(cacheData[setIdx][wayIdx].tag, row, col);
            evictData[row][col] = cacheData[setIdx] + wayIdx;
          }
        }

        cacheData[setIdx][wayIdx].insertedAt = tick;
        cacheData[setIdx][wayIdx].lastAccessed = tick;
        cacheData[setIdx][wayIdx].valid = true;
        cacheData[setIdx][wayIdx].dirty = false;

        readList.push_back({lca, ((uint64_t)setIdx << 32) | wayIdx});
      }

      evictCache(tick);
      for (auto &iter : readList) {
        Line *pLine = &cacheData[iter.second >> 32][iter.second & 0xFFFFFFFF];

        // Read data
        reqInternal.lpn = iter.first / lineCountInSuperPage;
        reqInternal.ioFlag.reset();
        reqInternal.ioFlag.set(iter.first % lineCountInSuperPage);

        beginAt = tick;  // Ignore cache metadata access
        pFTL->read(reqInternal, beginAt);

        // DRAM delay
        dramAt = pLine->insertedAt;
        pDRAM->read(pLine, lineSize, dramAt);

        // Set cache data
        beginAt = MAX(beginAt, dramAt);

        pLine->insertedAt = beginAt;
        pLine->lastAccessed = beginAt;
        pLine->tag = iter.first;

        if (pLine->tag == req.range.slpn) {
          finishedAt = beginAt;
        }

        Logger::debugprint(Logger::LOG_ICL_GENERIC_CACHE,
                           "READ  | Cache miss at (%u, %u) | %" PRIu64
                           " - %" PRIu64 " (%" PRIu64 ")",
                           iter.second >> 32, iter.second & 0xFFFFFFFF, tick,
                           beginAt, beginAt - tick);
      }

      tick = finishedAt;
    }
  }
  else {
    FTL::Request reqInternal(lineCountInSuperPage, req);

    pFTL->read(reqInternal, tick);
  }

  stat.request[0]++;

  if (ret) {
    stat.cache[0]++;
  }

  return ret;
}

// True when cold-miss/hit
bool GenericCache::write(Request &req, uint64_t &tick) {
  bool ret = false;
  bool isWrite = true;
  Logger::debugprint(Logger::LOG_ICL_GENERIC_CACHE,
                     "WRITE | REQ %7u-%-4u | LCA %" PRIu64 " | SIZE %" PRIu64,
                     req.reqID, req.reqSubID, req.range.slpn, req.length);
  if (useReadCaching) {
    // invalidate the line from read cache if hit
    uint32_t setIdx = calcSetIndex(!isWrite, req.range.slpn);
    uint32_t wayIdx;

    wayIdx = getValidWay(!isWrite, req.range.slpn, tick); 

    if (wayIdx != waySize) {
      cacheData[setIdx][wayIdx].valid = false;
      cacheData[setIdx][wayIdx].dirty = false;      
    }   
  }
  if (useWriteCaching) {
    uint32_t setIdx = calcSetIndex(isWrite, req.range.slpn);
    uint32_t wayIdx;

    wayIdx = getValidWay(isWrite, req.range.slpn, tick);

    // Can we update old data?
    if (wayIdx != waySize) {
      uint64_t arrived = tick;

      // Wait cache to be valid
      if (tick < writeCacheData[setIdx][wayIdx].insertedAt) {
        tick = writeCacheData[setIdx][wayIdx].insertedAt;
      }

      // Update last accessed time
      writeCacheData[setIdx][wayIdx].insertedAt = tick;
      writeCacheData[setIdx][wayIdx].lastAccessed = tick;
      writeCacheData[setIdx][wayIdx].dirty = true;

      // DRAM access
      pDRAM->read(&writeCacheData[setIdx][wayIdx], req.length, tick);

      Logger::debugprint(Logger::LOG_ICL_GENERIC_CACHE,
                         "WRITE | Cache hit at (%u, %u) | %" PRIu64
                         " - %" PRIu64 " (%" PRIu64 ")",
                         setIdx, wayIdx, arrived, tick, tick - arrived);

      ret = true;
    }
    else {
      uint64_t arrived = tick;

      wayIdx = getEmptyWay(isWrite, setIdx, tick);

      // Do we have place to write data?
      if (wayIdx != waySize) {
        // Wait cache to be valid
        if (tick < writeCacheData[setIdx][wayIdx].insertedAt) {
          tick = writeCacheData[setIdx][wayIdx].insertedAt;
        }

        // Update last accessed time
        writeCacheData[setIdx][wayIdx].insertedAt = tick;
        writeCacheData[setIdx][wayIdx].lastAccessed = tick;
        writeCacheData[setIdx][wayIdx].valid = true;
        writeCacheData[setIdx][wayIdx].dirty = true;
        writeCacheData[setIdx][wayIdx].tag = req.range.slpn;

        // DRAM access
        pDRAM->read(&writeCacheData[setIdx][wayIdx], req.length, tick);

        ret = true;
      }
      // We have to flush
      else {
        uint32_t row, col;  // Variable for I/O position (IOFlag)
        wayIdx = writeEvictFunction(setIdx, tick);
        assert(writeCacheData[setIdx][wayIdx].dirty);
        calcIOPosition(writeCacheData[setIdx][wayIdx].tag, row, col);
        evictData[row][col] = writeCacheData[setIdx] + wayIdx;
        evictCache(tick);
        // DRAM latency
        pDRAM->read(&writeCacheData[setIdx][wayIdx], req.length, tick);        
        // Update cache data
        writeCacheData[setIdx][wayIdx].insertedAt = tick;
        writeCacheData[setIdx][wayIdx].lastAccessed = tick;
        writeCacheData[setIdx][wayIdx].valid = true;
        writeCacheData[setIdx][wayIdx].dirty = true;
        writeCacheData[setIdx][wayIdx].tag = req.range.slpn;        
      }

      Logger::debugprint(Logger::LOG_ICL_GENERIC_CACHE,
                         "WRITE | Cache miss at (%u, %u) | %" PRIu64
                         " - %" PRIu64 " (%" PRIu64 ")",
                         setIdx, wayIdx, arrived, tick, tick - arrived);
    }
  }
  else {
    FTL::Request reqInternal(lineCountInSuperPage, req);

    pFTL->write(reqInternal, tick);
  }

  stat.request[1]++;

  if (ret) {
    stat.cache[1]++;
  }

  return ret;
}

// True when flushed
bool GenericCache::flush(Request &req, uint64_t &tick) {
  bool ret = false;
  bool isWrite = true;
  if (useReadCaching || useWriteCaching) {
    uint32_t setIdx = calcSetIndex(isWrite, req.range.slpn);
    uint32_t wayIdx;

    // Check cache that we have data for corresponding LBA
    wayIdx = getValidWay(isWrite, req.range.slpn, tick);

    // We have data to flush
    if (wayIdx != waySize) {
      FTL::Request reqInternal(lineCountInSuperPage);

      reqInternal.reqID = req.reqID;
      reqInternal.reqSubID = req.reqSubID;
      reqInternal.lpn = req.range.slpn / lineCountInSuperPage;
      reqInternal.ioFlag.set(req.range.slpn % lineCountInSuperPage);

      // We have data which is dirty
      if (writeCacheData[setIdx][wayIdx].dirty) {
        // we have to flush this
        pFTL->write(reqInternal, tick);
      }

      // Invalidate
      writeCacheData[setIdx][wayIdx].valid = false;

      ret = true;
    }
  }

  return ret;
}

// True when hit
bool GenericCache::trim(Request &req, uint64_t &tick) {
  bool ret = false;
  bool isWrite = true;
  FTL::Request reqInternal(lineCountInSuperPage);

  Logger::debugprint(Logger::LOG_ICL_GENERIC_CACHE,
                     "TRIM  | REQ %7u-%-4u | LCA %" PRIu64 " | SIZE %" PRIu64,
                     req.reqID, req.reqSubID, req.range.slpn, req.length);

  if (useReadCaching || useWriteCaching) {
    uint32_t setIdx = calcSetIndex(isWrite, req.range.slpn);
    uint32_t wayIdx;

    // Check cache that we have data for corresponding LBA
    wayIdx = getValidWay(isWrite, req.range.slpn, tick);

    if (wayIdx != waySize) {
      // Invalidate
      writeCacheData[setIdx][wayIdx].valid = false;
    }
  }

  reqInternal.reqID = req.reqID;
  reqInternal.reqSubID = req.reqSubID;
  reqInternal.lpn = req.range.slpn / lineCountInSuperPage;
  reqInternal.ioFlag.set(req.range.slpn % lineCountInSuperPage);

  // we have to flush this
  pFTL->trim(reqInternal, tick);

  return ret;
}

void GenericCache::format(LPNRange &range, uint64_t &tick) {
  if (useReadCaching || useWriteCaching) {
    uint64_t lpn;
    uint32_t setIdx;
    uint32_t wayIdx;
    bool isWrite = true;

    for (uint64_t i = 0; i < range.nlp; i++) {
      lpn = range.slpn + i;
      setIdx = calcSetIndex(isWrite, lpn);
      wayIdx = getValidWay(isWrite, lpn, tick);

      if (wayIdx != waySize) {
        // Invalidate
        writeCacheData[setIdx][wayIdx].valid = false;
      }
    }
  }

  // Convert unit
  range.slpn /= lineCountInSuperPage;
  range.nlp = (range.nlp - 1) / lineCountInSuperPage + 1;

  pFTL->format(range, tick);
}

void GenericCache::getStats(std::vector<Stats> &list) {
  Stats temp;

  temp.name = "icl.generic_cache.read.request_count";
  temp.desc = "Read request count";
  list.push_back(temp);

  temp.name = "icl.generic_cache.read.from_cache";
  temp.desc = "Read requests that served from cache";
  list.push_back(temp);

  temp.name = "icl.generic_cache.write.request_count";
  temp.desc = "Write request count";
  list.push_back(temp);

  temp.name = "icl.generic_cache.write.to_cache";
  temp.desc = "Write requests that served to cache";
  list.push_back(temp);
}

void GenericCache::getStatValues(std::vector<uint64_t> &values) {
  values.push_back(stat.request[0]);
  values.push_back(stat.cache[0]);
  values.push_back(stat.request[1]);
  values.push_back(stat.cache[1]);
}

void GenericCache::resetStats() {
  memset(&stat, 0, sizeof(stat));
}

}  // namespace ICL

}  // namespace SimpleSSD
