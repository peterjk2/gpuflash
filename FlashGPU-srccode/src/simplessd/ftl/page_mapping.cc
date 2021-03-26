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

#include "ftl/page_mapping.hh"

#include <algorithm>
#include <limits>

#include "log/trace.hh"
#include "util/algorithm.hh"
#include <iostream>
namespace SimpleSSD {

namespace FTL {

PageMapping::PageMapping(Parameter *p, PAL::PAL *l, ConfigReader *c)
    : AbstractFTL(p, l),
      pPAL(l),
      conf(c->ftlConfig),
      pFTLParam(p),
      latency(conf.readUint(FTL_LATENCY), conf.readUint(FTL_REQUEST_QUEUE)),
      lastFreeBlock(pFTLParam->pageCountToMaxPerf),
      bReclaimMore(false) {
  for (uint32_t i = 0; i < pFTLParam->totalPhysicalBlocks; i++) {
    freeBlocks.insert(
        {i, Block(pFTLParam->pagesInBlock, pFTLParam->ioUnitInPage)});
  }

  status.totalLogicalPages =
      pFTLParam->totalLogicalBlocks * pFTLParam->pagesInBlock;

  // Allocate free blocks
  for (uint32_t i = 0; i < pFTLParam->pageCountToMaxPerf; i++) {
    lastFreeBlock.at(i) = getFreeBlock(i);
  }

  lastFreeBlockIndex = 0;

  memset(&stat, 0, sizeof(stat));
}

PageMapping::~PageMapping() {}

bool PageMapping::initialize() {
  uint64_t nPagesToWarmup;
  uint64_t nTotalPages;
  uint64_t tick;
  Request req(pFTLParam->ioUnitInPage);

  nTotalPages = pFTLParam->totalLogicalBlocks * pFTLParam->pagesInBlock;
  nPagesToWarmup = nTotalPages * conf.readFloat(FTL_WARM_UP_RATIO);
  nPagesToWarmup = MIN(nPagesToWarmup, nTotalPages);
  req.ioFlag.set();

  for (req.lpn = 0; req.lpn < nPagesToWarmup; req.lpn++) {
    tick = 0;

    writeInternal(req, tick, false);
  }

  return true;
}

void PageMapping::read(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  readInternal(req, tick);

  Logger::debugprint(Logger::LOG_FTL_PAGE_MAPPING,
                     "READ  | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64
                     " (%" PRIu64 ")",
                     req.lpn, begin, tick, tick - begin);
}

void PageMapping::write(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  writeInternal(req, tick);

  Logger::debugprint(Logger::LOG_FTL_PAGE_MAPPING,
                     "WRITE | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64
                     " (%" PRIu64 ")",
                     req.lpn, begin, tick, tick - begin);
}

void PageMapping::trim(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  trimInternal(req, tick);

  Logger::debugprint(Logger::LOG_FTL_PAGE_MAPPING,
                     "TRIM  | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64
                     " (%" PRIu64 ")",
                     req.lpn, begin, tick, tick - begin);
}

void PageMapping::format(LPNRange &range, uint64_t &tick) {
  PAL::Request req(pFTLParam->ioUnitInPage);
  std::vector<uint32_t> list;

  req.ioFlag.set();

  for (auto iter = table.begin(); iter != table.end();) {
    if (iter->first >= range.slpn && iter->first < range.slpn + range.nlp) {
      auto &mappingList = iter->second;

      // Do trim
      for (uint32_t idx = 0; idx < pFTLParam->ioUnitInPage; idx++) {
        auto &mapping = mappingList.at(idx);
        auto block = blocks.find(mapping.first);

        if (block == blocks.end()) {
          Logger::panic("Block is not in use");
        }

        block->second.invalidate(mapping.second, idx);

        // Collect block indices
        list.push_back(mapping.first);
      }

      iter = table.erase(iter);
    }
    else {
      iter++;
    }
  }

  // Get blocks to erase
  std::sort(list.begin(), list.end());
  auto last = std::unique(list.begin(), list.end());
  list.erase(last, list.end());

  // Do GC only in specified blocks
  doGarbageCollection(list, tick);
}

Status *PageMapping::getStatus() {
  status.freePhysicalBlocks = freeBlocks.size();
  status.mappedLogicalPages = table.size();

  return &status;
}

float PageMapping::freeBlockRatio() {
  return (float)freeBlocks.size() / pFTLParam->totalPhysicalBlocks;
}

uint32_t PageMapping::convertBlockIdx(uint32_t blockIdx) {
  return blockIdx % pFTLParam->pageCountToMaxPerf;
}

uint32_t PageMapping::getFreeBlock(uint32_t idx) {
  uint32_t blockIndex = 0;

  if (idx >= pFTLParam->pageCountToMaxPerf) {
    Logger::panic("Index out of range");
  }

  if (freeBlocks.size() > 0) {
    uint32_t eraseCount = std::numeric_limits<uint32_t>::max();
    auto found = freeBlocks.end();

    // Found least erased block
    for (auto iter = freeBlocks.begin(); iter != freeBlocks.end(); iter++) {
      if (idx == convertBlockIdx(iter->first)) {
        uint32_t current = iter->second.getEraseCount();

        if (current < eraseCount) {
          eraseCount = current;
          blockIndex = iter->first;
          found = iter;
        }
      }
    }

    // No free block found on specified index
    if (found == freeBlocks.end()) {
      Logger::panic("No free block at index %d found", idx);
    }

    // Insert found block to block list
    if (blocks.find(blockIndex) != blocks.end()) {
      Logger::panic("Corrupted");
    }

    blocks.insert({blockIndex, found->second});

    // Remove found block from free block list
    freeBlocks.erase(found);
  }
  else {
    Logger::panic("No free block left");
  }

  return blockIndex;
}

uint32_t PageMapping::getLastFreeBlock() {
  auto freeBlock = blocks.find(lastFreeBlock.at(lastFreeBlockIndex));
  uint32_t blockIndex = 0;

  // Sanity check
  if (freeBlock == blocks.end()) {
    Logger::panic("Corrupted");
  }

  // If current free block is full, get next block
  if (freeBlock->second.getNextWritePageIndex() == pFTLParam->pagesInBlock) {
    lastFreeBlock.at(lastFreeBlockIndex) = getFreeBlock(lastFreeBlockIndex);

    bReclaimMore = true;
  }

  blockIndex = lastFreeBlock.at(lastFreeBlockIndex);

  // Update lastFreeBlockIndex
  lastFreeBlockIndex++;

  if (lastFreeBlockIndex == pFTLParam->pageCountToMaxPerf) {
    lastFreeBlockIndex = 0;
  }

  return blockIndex;
}

void PageMapping::selectVictimBlock(std::vector<uint32_t> &list,
                                    uint64_t &tick) {
  static const GC_MODE mode = (GC_MODE)conf.readInt(FTL_GC_MODE);
  static const EVICT_POLICY policy =
      (EVICT_POLICY)conf.readInt(FTL_GC_EVICT_POLICY);
  uint64_t nBlocks = conf.readInt(FTL_GC_RECLAIM_BLOCK);
  std::vector<std::pair<uint32_t, float>> weight;
  uint64_t i = 0;

  list.clear();

  // Calculate number of blocks to reclaim
  if (mode == GC_MODE_0) {
    // DO NOTHING
  }
  else if (mode == GC_MODE_1) {
    static const float t = conf.readFloat(FTL_GC_RECLAIM_THRESHOLD);

    nBlocks = pFTLParam->totalPhysicalBlocks * t - freeBlocks.size();
  }
  else {
    Logger::panic("Invalid GC mode");
  }

  // reclaim one more if last free block fully used
  if (bReclaimMore) {
    nBlocks += pFTLParam->pageCountToMaxPerf;

    bReclaimMore = false;
  }

  // Calculate weights of all blocks
  weight.resize(blocks.size());

  if (policy == POLICY_GREEDY) {
    for (auto &iter : blocks) {
      weight.at(i).first = iter.first;
      weight.at(i).second =
          pFTLParam->pagesInBlock - iter.second.getDirtyPageCount();

      i++;
    }
  }
  else if (policy == POLICY_COST_BENEFIT) {
    float temp;

    for (auto &iter : blocks) {
      temp =
          (float)(pFTLParam->pagesInBlock - iter.second.getDirtyPageCount()) /
          pFTLParam->pagesInBlock;

      weight.at(i).first = iter.first;
      weight.at(i).second =
          temp / ((1 - temp) * (tick - iter.second.getLastAccessedTime()));

      i++;
    }
  }
  else {
    Logger::panic("Invalid evict policy");
  }

  // Sort weights
  std::sort(
      weight.begin(), weight.end(),
      [](std::pair<uint32_t, float> a, std::pair<uint32_t, float> b) -> bool {
        return a.second < b.second;
      });

  // Select victims
  i = 0;
  list.resize(nBlocks);

  for (auto &iter : list) {
    iter = weight.at(i++).first;
  }
}

void PageMapping::doGarbageCollection(std::vector<uint32_t> &blocksToReclaim,
                                      uint64_t &tick) {
  PAL::Request req(pFTLParam->ioUnitInPage);
  std::vector<uint64_t> lpns;
  DynamicBitset bit(pFTLParam->ioUnitInPage);
  uint64_t beginAt;
  uint64_t beginAt2;
  uint64_t finishedAt = tick;
  uint64_t finishedAt2 = tick;

  if (blocksToReclaim.size() == 0) {
    return;
  }

  // For all blocks to reclaim
  for (auto &iter : blocksToReclaim) {
    auto block = blocks.find(iter);

    if (block == blocks.end()) {
      Logger::panic("Invalid block");
    }

    // Copy valid pages to free block
    for (uint32_t pageIndex = 0; pageIndex < pFTLParam->pagesInBlock;
         pageIndex++) {
      // Valid?
      if (block->second.getPageInfo(pageIndex, lpns, bit)) {
        // Retrive free block
        auto freeBlock = blocks.find(getLastFreeBlock());

        // Issue Read
        req.blockIndex = block->first;
        req.pageIndex = pageIndex;
        req.ioFlag = bit;

        beginAt = tick;

        pPAL->read(req, beginAt);

        // Update mapping table
        uint32_t newBlockIdx = freeBlock->first;

        for (uint32_t idx = 0; idx < pFTLParam->ioUnitInPage; idx++) {
          if (bit.test(idx)) {
            // Invalidate
            block->second.invalidate(pageIndex, idx);

            auto mappingList = table.find(lpns.at(idx));

            if (mappingList == table.end()) {
              Logger::panic("Invalid mapping table entry");
            }

            auto &mapping = mappingList->second.at(idx);

            uint32_t newPageIdx = freeBlock->second.getNextWritePageIndex(idx);

            mapping.first = newBlockIdx;
            mapping.second = newPageIdx;

            freeBlock->second.write(newPageIdx, lpns.at(idx), idx, beginAt);

            // Issue Write
            req.blockIndex = newBlockIdx;
            req.pageIndex = newPageIdx;
            req.ioFlag.reset();
            req.ioFlag.set(idx);

            beginAt2 = beginAt;

            pPAL->write(req, beginAt2);

            finishedAt2 = MAX(finishedAt2, beginAt2);
          }
        }
      }
    }

    // Erase block
    req.blockIndex = block->first;
    req.pageIndex = 0;
    req.ioFlag.set();

    eraseInternal(req, finishedAt2);

    // Merge timing
    finishedAt = MAX(finishedAt, finishedAt2);
  }

  tick = finishedAt;
}

void PageMapping::readInternal(Request &req, uint64_t &tick) {
  PAL::Request palRequest(req);
  uint64_t beginAt;
  uint64_t finishedAt = tick;
  auto mappingList = table.find(req.lpn);

  if (mappingList != table.end()) {
    latency.access(req.ioFlag.count(), tick);

    for (uint32_t idx = 0; idx < pFTLParam->ioUnitInPage; idx++) {
      if (req.ioFlag.test(idx)) {
        auto &mapping = mappingList->second.at(idx);

        if (mapping.first < pFTLParam->totalPhysicalBlocks &&
            mapping.second < pFTLParam->pagesInBlock) {
          palRequest.blockIndex = mapping.first;
          palRequest.pageIndex = mapping.second;
          palRequest.ioFlag.reset();
          palRequest.ioFlag.set(idx);

          auto block = blocks.find(palRequest.blockIndex);

          if (block == blocks.end()) {
            Logger::panic("Block is not in use");
          }

          beginAt = tick;

          block->second.read(palRequest.pageIndex, idx, beginAt);
          pPAL->read(palRequest, beginAt);

          finishedAt = MAX(finishedAt, beginAt);
        }
      }
    }

    tick = finishedAt;
  }
}

void PageMapping::writeInternal(Request &req, uint64_t &tick, bool sendToPAL) {
  PAL::Request palRequest(req);
  std::unordered_map<uint32_t, Block>::iterator block;
  auto mappingList = table.find(req.lpn);
  uint64_t beginAt;
  uint64_t finishedAt = tick;

  latency.access(req.ioFlag.count(), tick);

  if (mappingList != table.end()) {
    for (uint32_t idx = 0; idx < pFTLParam->ioUnitInPage; idx++) {
      if (req.ioFlag.test(idx)) {
        auto &mapping = mappingList->second.at(idx);

        if (mapping.first < pFTLParam->totalPhysicalBlocks &&
            mapping.second < pFTLParam->pagesInBlock) {
          block = blocks.find(mapping.first);

          // Invalidate current page
          block->second.invalidate(mapping.second, idx);
        }
      }
    }
  }
  else {
    // Create empty mapping
    auto ret = table.insert(
        {req.lpn, std::vector<std::pair<uint32_t, uint32_t>>(
                      pFTLParam->ioUnitInPage, {pFTLParam->totalPhysicalBlocks,
                                                pFTLParam->pagesInBlock})});

    if (!ret.second) {
      Logger::panic("Failed to insert new mapping");
    }

    mappingList = ret.first;
  }

  // Write data to free block
  block = blocks.find(getLastFreeBlock());

  if (block == blocks.end()) {
    Logger::panic("No such block");
  }

  for (uint32_t idx = 0; idx < pFTLParam->ioUnitInPage; idx++) {
    if (req.ioFlag.test(idx)) {
      uint32_t pageIndex = block->second.getNextWritePageIndex(idx);
      auto &mapping = mappingList->second.at(idx);

      beginAt = tick;

      block->second.write(pageIndex, req.lpn, idx, beginAt);

      // update mapping to table
      mapping.first = block->first;
      mapping.second = pageIndex;

      if (sendToPAL) {
        palRequest.blockIndex = block->first;
        palRequest.pageIndex = pageIndex;
        palRequest.ioFlag.reset();
        palRequest.ioFlag.set(idx);

        pPAL->write(palRequest, beginAt);
      }

      finishedAt = MAX(finishedAt, beginAt);
    }
  }

  tick = finishedAt;

  // GC if needed
  if (freeBlockRatio() < conf.readFloat(FTL_GC_THRESHOLD_RATIO)) {
    std::vector<uint32_t> list;
    uint64_t beginAt = tick;

    selectVictimBlock(list, beginAt);

    Logger::debugprint(Logger::LOG_FTL_PAGE_MAPPING,
                       "GC   | On-demand | %u blocks will be reclaimed",
                       list.size());

    doGarbageCollection(list, beginAt);

    Logger::debugprint(Logger::LOG_FTL_PAGE_MAPPING,
                       "GC   | Done | %" PRIu64 " - %" PRIu64 " (%" PRIu64 ")",
                       tick, beginAt, beginAt - tick);

    stat.gcCount++;
    stat.reclaimedBlocks += list.size();
  }
}

void PageMapping::trimInternal(Request &req, uint64_t &tick) {
  auto mappingList = table.find(req.lpn);

  if (mappingList != table.end()) {
    // Do trim
    for (uint32_t idx = 0; idx < pFTLParam->ioUnitInPage; idx++) {
      auto &mapping = mappingList->second.at(idx);
      auto block = blocks.find(mapping.first);

      if (block == blocks.end()) {
        Logger::panic("Block is not in use");
      }

      block->second.invalidate(mapping.second, idx);
    }

    // Remove mapping
    table.erase(mappingList);
  }
}

void PageMapping::eraseInternal(PAL::Request &req, uint64_t &tick) {
  static uint64_t threshold = conf.readUint(FTL_BAD_BLOCK_THRESHOLD);
  auto block = blocks.find(req.blockIndex);

  // Sanity checks
  if (block == blocks.end()) {
    Logger::panic("No such block");
  }

  if (freeBlocks.find(req.blockIndex) != freeBlocks.end()) {
    Logger::panic("Corrupted");
  }

  if (block->second.getValidPageCount() != 0) {
    Logger::panic("There are valid pages in victim block");
  }

  // Erase block
  block->second.erase();

  pPAL->erase(req, tick);

  // Check erase count
  if (block->second.getEraseCount() < threshold) {
    // Insert block to free block list
    freeBlocks.insert({req.blockIndex, block->second});
  }

  // Remove block from block list
  blocks.erase(block);
}

void PageMapping::getStats(std::vector<Stats> &list) {
  Stats temp;

  temp.name = "ftl.page_mapping.gc_count";
  temp.desc = "Total GC count";
  list.push_back(temp);

  temp.name = "ftl.page_mapping.reclaimed_blocks";
  temp.desc = "Total reclaimed blocks in GC";
  list.push_back(temp);
}

void PageMapping::getStatValues(std::vector<uint64_t> &values) {
  values.push_back(stat.gcCount);
  values.push_back(stat.reclaimedBlocks);
}

void PageMapping::resetStats() {
  memset(&stat, 0, sizeof(stat));
}

}  // namespace FTL

}  // namespace SimpleSSD
