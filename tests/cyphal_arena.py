#!/usr/bin/env python3
"""Check live libcanard RX/TX allocations across external O1 arena reuse."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
LIB = ROOT / 'Drivers/libcxxcanard'
SOURCE = r'''
#include <array>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <cyphal/cyphal.h>
#include <cyphal/allocators/o1/o1_allocator.h>

struct Listener : TransferListener {
 unsigned calls=0;
 void accept(CanardRxTransfer* transfer) override {
  assert(transfer->payload_size==5);
  assert(static_cast<uint8_t*>(transfer->payload)[0]==0x5A);
  ++calls;
 }
};
struct Provider : AbstractCANProvider {
 Provider(O1Allocator& allocator,const UtilityConfig& utilities)
  :AbstractCANProvider(64,72,50,utilities) {
   setup(&allocator,1,[](AbstractAllocator*){});
 }
 void receive(const CanardFrame& frame) {accept_rx_frame(frame);}
 void subscribe(CanardRxSubscription& sub,CanardPortID port,Listener& listener) {
  sub.user_reference=&listener;
  assert(canardRxSubscribe(&canard,CanardTransferKindMessage,port,256,1000000,&sub)>=0);
 }
 uint32_t len_to_dlc(size_t n) override {return n;}
 size_t dlc_to_len(uint32_t n) override {return n;}
 void can_loop(bool) override {}
 void process_canard_rx(bool) override {}
 bool read_frame(CanardFrame*,void*) override {return false;}
 int write_frame(const CanardTxQueueItem* item) override {return item->frame.payload_size;}
};
int main() {
 uint64_t now=100;
 UtilityConfig utility([&](){return now;},[](){assert(false);});
 alignas(O1HEAP_ALIGNMENT) std::array<uint8_t,16384> arena{};
 O1Allocator allocator(arena.size(),arena.data(),utility);
 Provider provider(allocator,utility);
 CyphalInterface interface(1,utility,&provider,[](AbstractCANProvider*){});
 std::array<CanardRxSubscription,5> subscriptions{};
 std::array<Listener,5> listeners{};
 const std::array<CanardPortID,5> ports{300,100,400,200,250};
 for(size_t i=0;i<ports.size();++i)provider.subscribe(subscriptions[i],ports[i],listeners[i]);
 CanardInstance remote=canardInit([](CanardInstance*,size_t n){return std::malloc(n);},
                                  [](CanardInstance*,void* p){std::free(p);});
 remote.node_id=42;
 CanardTxQueue tx=canardTxInit(50,64);
 std::array<uint8_t,160> payload{};payload.fill(0x5A);
 auto send=[&](CanardPortID port,unsigned tid,bool partial) {
  CanardTransferMetadata meta{CanardPriorityNominal,CanardTransferKindMessage,port,CANARD_NODE_ID_UNSET,static_cast<CanardTransferID>(tid)};
  assert(canardTxPush(&tx,&remote,0,&meta,partial?payload.size():5,payload.data())>0);
  bool first=true;
  while(auto* item=canardTxPeek(&tx)) {
   if(first || !partial)provider.receive(item->frame);
   first=false;
   remote.memory_free(&remote,canardTxPop(&tx,item));
  }
  ++now;
 };
 for(unsigned cycle=0;cycle<3;++cycle) {
  for(auto port:ports)send(port,cycle*2,true);
  CanardTransferMetadata meta{CanardPriorityNominal,CanardTransferKindMessage,500,CANARD_NODE_ID_UNSET,0};
  interface.push(0,&meta,5,payload.data());
  assert(interface.queue_size()==1 && o1heapGetDiagnostics(allocator.get_heap()).allocated>0);
  interface.halt();
  assert(interface.queue_size()==0 && o1heapGetDiagnostics(allocator.get_heap()).allocated==0);
  interface.halt(); // Repeated halt must not double-free sessions or queue items.
  assert(o1heapGetDiagnostics(allocator.get_heap()).allocated==0);
  // This models calibration using and then wiping the entire arena.
  arena.fill(0xA5);interface.reset();
  assert(o1heapDoInvariantsHold(allocator.get_heap()));
  for(size_t i=0;i<ports.size();++i) {
   assert(subscriptions[i].user_reference==&listeners[i]);
   send(ports[i],cycle*2+1,false);
   assert(listeners[i].calls==cycle+1);
  }
  interface.push(0,&meta,5,payload.data());interface.process_canard_tx(false);
  assert(interface.queue_size()==0);
  interface.halt();assert(o1heapGetDiagnostics(allocator.get_heap()).allocated==0);
  interface.reset();assert(o1heapDoInvariantsHold(allocator.get_heap()));
 }
}
'''
with tempfile.TemporaryDirectory(prefix='vbdrive-arena-') as directory:
    tmp = Path(directory)
    includes = ['-I'+str(LIB), '-I'+str(LIB/'libs')]
    objects = []
    for source in ('libs/libcanard/canard.c', 'libs/o1heap/o1heap.c'):
        obj = tmp / (Path(source).stem+'.o')
        subprocess.run(['cc', '-std=c11', '-O2', *includes, '-c', str(LIB/source), '-o', str(obj)], check=True)
        objects.append(str(obj))
    (tmp/'test.cpp').write_text(SOURCE)
    subprocess.run(['c++', '-std=c++17', '-O2', *includes, str(tmp/'test.cpp'),
                    str(LIB/'cyphal/cyphal.cpp'), str(LIB/'cyphal/providers/provider.cpp'),
                    str(LIB/'cyphal/allocators/o1/o1_allocator.cpp'), *objects,
                    '-o', str(tmp/'test')], check=True)
    subprocess.run([str(tmp/'test')], check=True)
print('PASS: libcanard RX sessions/TX released, arena reused, all five listeners preserved, RX/TX resumed')
