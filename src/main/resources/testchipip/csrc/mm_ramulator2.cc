// See LICENSE for license details.

#include "mm_ramulator2.h"
#include "mm.h"
#include <iostream>
#include <fstream>
#include <list>
#include <queue>
#include <cstring>
#include <cstdlib>
#include <cassert>

//#define DEBUG_RAMULATOR2

//using namespace Ramulator;

static uint64_t dbg_read_sent = 0;
static uint64_t dbg_read_done = 0;
static uint64_t dbg_write_sent = 0;
static uint64_t dbg_write_done = 0;
static uint64_t dbg_read_reject = 0;
static uint64_t dbg_write_reject = 0;

void mm_ramulator2_t::read_complete(uint64_t address)
{
  dbg_read_done++;
  assert(!rreq[address].empty());
  auto req = rreq[address].front();
  uint64_t start_addr = (req.addr / word_size) * word_size;
  for (size_t i = 0; i < req.len; i++) {
    auto dat = read(start_addr + i * word_size);
    rresp.push(mm_rresp_t(req.id, dat, (i == req.len - 1)));
  }
  read_id_busy[req.id] = false;
  rreq[address].pop();
}

void mm_ramulator2_t::write_complete(uint64_t address)
{
  dbg_write_done++;
  assert(!wreq[address].empty());
  auto b_id = wreq[address].front();
  bresp.push(b_id);
  write_id_busy[b_id] = false;
  wreq[address].pop();
}

/*void power_callback(double a, double b, double c, double d)
{
    //fprintf(stderr, "power callback: %0.3f, %0.3f, %0.3f, %0.3f\n",a,b,c,d);
}*/

mm_ramulator2_t::mm_ramulator2_t(
    size_t mem_base,
    size_t mem_sz,
    size_t word_sz,
    size_t line_sz,
    backing_data_t& dat,
    std::string ramulator2_config_path,
    int axi4_ids,
    size_t clock_hz) :
  mm_t(mem_base, mem_sz, word_sz, line_sz, dat),
  ramulator2_frontend(nullptr),
  ramulator2_memorysystem(nullptr),
  read_id_busy(axi4_ids, false),
  write_id_busy(axi4_ids, false),
  config_path(ramulator2_config_path),
  clock_hz(clock_hz),
  cycle(0) {

  assert(line_sz == 64); // assumed by this wrapper
  assert(mem_sz % (1024*1024) == 0);

  YAML::Node config = Ramulator::Config::parse_config_file(config_path, {});
  ramulator2_frontend = Ramulator::Factory::create_frontend(config);
  ramulator2_memorysystem = Ramulator::Factory::create_memory_system(config);

  ramulator2_frontend->connect_memory_system(ramulator2_memorysystem);
  ramulator2_memorysystem->connect_frontend(ramulator2_frontend);
}

mm_ramulator2_t::~mm_ramulator2_t()
{
  if (ramulator2_frontend) {
    ramulator2_frontend->finalize();
  }

  if (ramulator2_memorysystem) {
    ramulator2_memorysystem->finalize();
  }
}

bool mm_ramulator2_t::ar_ready() {
  return true;
}

bool mm_ramulator2_t::aw_ready() {
  return !store_inflight;
}

void mm_ramulator2_t::tick(
  bool reset,

  bool ar_valid,
  uint64_t ar_addr,
  uint64_t ar_id,
  uint64_t ar_size,
  uint64_t ar_len,

  bool aw_valid,
  uint64_t aw_addr,
  uint64_t aw_id,
  uint64_t aw_size,
  uint64_t aw_len,

  bool w_valid,
  uint64_t w_strb,
  void *w_data,
  bool w_last,

  bool r_ready,
  bool b_ready)
{
  bool ar_fire = !reset && ar_valid && ar_ready();
  bool aw_fire = !reset && aw_valid && aw_ready();
  bool w_fire = !reset && w_valid && w_ready();
  bool r_fire = !reset && r_valid() && r_ready;
  bool b_fire = !reset && b_valid() && b_ready;

  for (auto it = rreq_queue.begin(); it != rreq_queue.end(); it++) {
    if (!read_id_busy[it->id]) {
      auto transaction = *it;
      uint64_t addr = transaction.addr;

      bool accepted = ramulator2_frontend->receive_external_requests(
          0,
          addr,
          transaction.id,
          [this, addr](Ramulator::Request& req) {
            read_complete(addr);
          });

      if (accepted) {
        dbg_read_sent++;
        read_id_busy[transaction.id] = true;
        rreq[addr].push(transaction);
        rreq_queue.erase(it);
      } else {
        dbg_read_reject++;
      }
      break;
    }
  }

  if (!pending_wreq_queue.empty()) {
    auto pending = pending_wreq_queue.front();
    uint64_t addr = pending.first;
    uint64_t id = pending.second;

    bool accepted = ramulator2_frontend->receive_external_requests(
        1,
        addr,
        id,
        [](Ramulator::Request& req) {
        });

    if (accepted) {
      dbg_write_sent++;
      dbg_write_done++;
      bresp.push(id);
      pending_wreq_queue.pop();
    } else {
      dbg_write_reject++;
    }
  }

  if (ar_fire) {
    rreq_queue.push_back(mm_ramulator2_req_t(ar_id, 1 << ar_size, ar_len + 1, ar_addr));
  }

  if (aw_fire) {
    store_addr = aw_addr;
    store_base_addr = aw_addr;
    store_id = aw_id;
    store_count = aw_len + 1;
    store_size = 1 << aw_size;
    store_inflight = true;
  }

  if (w_fire) {
    write(store_addr, (uint8_t*)w_data, w_strb, store_size);
    store_addr += store_size;
    store_count--;

    if (store_count == 0) {
      store_inflight = false;
      pending_wreq_queue.push(std::make_pair(store_base_addr, store_id));
      assert(w_last);
    }
  }

  if (b_fire)
    bresp.pop();

  if (r_fire)
    rresp.pop();

  ramulator2_frontend->tick();
  ramulator2_memorysystem->tick();
  cycle++;

  if ((cycle % 1000000) == 0) {
    fprintf(stderr,
      "[ramulator2 dbg] cycle=%lu r_sent=%lu r_done=%lu r_rej=%lu w_sent=%lu w_done=%lu w_rej=%lu rreq_queue=%lu pending_wreq=%lu rresp=%lu bresp=%lu rreq=%lu wreq=%lu store_inflight=%d\n",
      cycle,
      dbg_read_sent,
      dbg_read_done,
      dbg_read_reject,
      dbg_write_sent,
      dbg_write_done,
      dbg_write_reject,
      rreq_queue.size(),
      pending_wreq_queue.size(),
      rresp.size(),
      bresp.size(),
      rreq.size(),
      wreq.size(),
      store_inflight ? 1 : 0);
  }

  if (reset) {
    while (!bresp.empty()) bresp.pop();
    while (!rresp.empty()) rresp.pop();
    while (!pending_wreq_queue.empty()) pending_wreq_queue.pop();

    rreq_queue.clear();
    rreq.clear();
    wreq.clear();

    store_inflight = false;
    cycle = 0;
  }
}