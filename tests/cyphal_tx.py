#!/usr/bin/env python3
"""Exercise actual bounded TX loop and STM32 busy-FIFO path with transport doubles."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
def body(text, signature):
    start=text.index(signature); pos=text.index('{',start); depth=1; end=pos+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}'); end+=1
    return text[start:end]
provider=(root/'Drivers/libcxxcanard/cyphal/providers/provider.cpp').read_text()
g4=(root/'Drivers/libcxxcanard/cyphal/providers/G4CAN.cpp').read_text()
hal=(root/'Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c').read_text()
fifo_functions=''
for fifo in (0,1):
    begin=hal.index(f'        /* Check that the Rx FIFO {fifo} is full & overwrite mode is on */')
    end=hal.index(f'        /* Calculate Rx FIFO {fifo} element address */',begin)
    block=hal[begin:end]
    for name,value in [(f'FDCAN_RXF{fifo}S_F{fifo}F_Pos','8'),(f'FDCAN_RXF{fifo}S_F{fifo}F','256'),
                       (f'FDCAN_RXGFC_F{fifo}OM_Pos','0'),(f'FDCAN_RXGFC_F{fifo}OM','1'),
                       (f'FDCAN_RXF{fifo}S_F{fifo}GI_Pos','0'),(f'FDCAN_RXF{fifo}S_F{fifo}GI','3'),
                       (f'SRAMCAN_RF{fifo}_NBR','3'),('FDCAN_RX_FIFO_OVERWRITE','1')]:
        block=block.replace(name,value)
    block=block.replace(f'hfdcan->Instance->RXF{fifo}S','status').replace('hfdcan->Instance->RXGFC','overwrite')
    fifo_functions+=f'unsigned fifo{fifo}(unsigned status,unsigned overwrite) {{unsigned GetIndex=0;\n'+block+'return GetIndex;}\n'
source=r'''
#include <cassert>
#include <cstdint>
struct CanardFrame {uint32_t extended_can_id=0; unsigned payload_size=8; uint8_t* payload=nullptr;};
struct CanardTxQueueItem {uint64_t tx_deadline_usec=200; CanardFrame frame;};
struct Queue {unsigned size=0;};
CanardTxQueueItem item;
const CanardTxQueueItem* canardTxPeek(Queue*) {return &item;}
void* canardTxPop(Queue* q,const CanardTxQueueItem*) {--q->size;return nullptr;}
struct Allocator {void memory_free(Allocator*,void*) {}};
struct Utility {uint64_t micros_64() {return 100;}};
struct AbstractCANProvider {
 Queue queue; Allocator canard; Utility utilities; unsigned writes=0; bool busy=false;
 void lock_canard() {} void unlock_canard() {}
 int write_frame(const CanardTxQueueItem*) {++writes;return busy?-1:8;}
 void process_canard_tx(bool all=true);
};
struct FDCAN_TxHeaderTypeDef {
 unsigned Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator,BitRateSwitch,
 FDFormat,TxEventFifoControl,MessageMarker;
};
constexpr unsigned FDCAN_EXTENDED_ID=0,FDCAN_DATA_FRAME=0,FDCAN_ESI_ACTIVE=0,
 FDCAN_BRS_ON=0,FDCAN_FD_CAN=0,FDCAN_STORE_TX_EVENTS=0,HAL_OK=0;
unsigned CanardFDCANLengthToDLC[65]{};
unsigned free_slots=3,added=0;
unsigned HAL_FDCAN_GetTxFifoFreeLevel(void*) {return free_slots;}
int HAL_FDCAN_AddMessageToTxFifoQ(void*,FDCAN_TxHeaderTypeDef*,uint8_t*) {++added;return 0;}
struct G4CAN {void* handler=nullptr; int write_frame(const CanardTxQueueItem*);};
'''+body(provider,'void AbstractCANProvider::process_canard_tx(')+body(g4,'int G4CAN::write_frame(')+fifo_functions+r'''
int main() {
 for(unsigned index=0;index<3;++index) for(unsigned full=0;full<2;++full) for(unsigned overwrite=0;overwrite<2;++overwrite) {
   unsigned expected=(index+(full && overwrite))%3;
   assert(fifo0(index|(full<<8),overwrite)==expected);
   assert(fifo1(index|(full<<8),overwrite)==expected);
 }
 AbstractCANProvider p;p.queue.size=3;p.process_canard_tx(false);
 assert(p.queue.size==2 && p.writes==1);
 p.process_canard_tx();assert(p.queue.size==0 && p.writes==3);
 p.queue.size=3;p.busy=true;p.process_canard_tx(false);assert(p.queue.size==3);
 p.busy=false;item.tx_deadline_usec=1;p.process_canard_tx(false);
 assert(p.queue.size==2); // Expired frames also obey the per-pass budget.
 G4CAN g;free_slots=2;assert(g.write_frame(&item)<0 && added==0);
 free_slots=3;assert(g.write_frame(&item)>=0 && added==1);
}
'''
with tempfile.TemporaryDirectory(prefix='vbdrive-tx-') as d:
    p=Path(d);(p/'test.cpp').write_text(source)
    subprocess.run(['c++','-std=c++20',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
print('PASS: bounded TX, expiry, busy FIFO, default do-all, RX overwrite wrap')
