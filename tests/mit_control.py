#!/usr/bin/env python3
"""Host smoke of actual FOC handler, generated serializers and libcanard framing.

Run after building RelWithDebInfo. No hardware access or motion.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
app = (root / 'App/app.cpp').read_text()
handler = app[app.index('bool apply_mit_command('):app.index('class ServoSub:')]
servo_handler = app[app.index('class ServoSub:'):app.index('// NOTE: underlying CanardRxSubscriptions')]
source = r'''
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <initializer_list>
#include <utility>
#include <libcanard/canard.h>
#include <voltbro/foc/command_1_0.h>
#include <voltbro/foc/MITCommand_1_0.h>
#include <voltbro/foc/State_1_0.h>
#include <voltbro/foc/state_simple_1_0.h>
#include <voltbro/foc/Servo_1_0.h>
#include <voltbro/foc/specific_control_1_0.h>
using InterfacePtr = int;
template<class T> struct AbstractSubscription {
    AbstractSubscription(int, CanardPortID) {}
    virtual void handler(const T&, CanardRxTransfer*) = 0;
};
struct FOCTarget {float torque, angle, velocity, angle_kp, velocity_kp;};
struct Motor {
    FOCTarget target{};
    float kp=17, ki=19;
    int targets=0, gains=0;
    bool valid=true;
    int servo_type=-1;
    float servo_value=0;
    bool set_velocity_point(float v) {servo_type=0; servo_value=v; return valid;}
    bool set_torque_point(float v) {servo_type=1; servo_value=v; return valid;}
    bool set_angle_point(float v) {servo_type=2; servo_value=v; return valid;}
    bool set_voltage_point(float v) {servo_type=3; servo_value=v; return valid;}
    bool set_foc_point(FOCTarget t) {target=t; ++targets; return valid;}
    void set_current_regulator_params(float p,float i) {kp=p; ki=i; ++gains;}
} device;
Motor* motor=&device;
Motor* get_motor() {return motor;}
int errors=0;
void record_invalid_command() {++errors;}
'''+handler+servo_handler+r'''
int main() {
    ServoSub servo_sub(0,3418);
    for (uint8_t type : {0,1,2,3,4,255}) {
        device=Motor{}; errors=0;
        voltbro_foc_Servo_1_0 command{};
        command.set_point_type=type; command.set_point_value=.25f;
        servo_sub.handler(command,nullptr);
        if (type<4) {
            assert(device.servo_type==type && device.servo_value==.25f && errors==0);
        } else assert(device.servo_type==-1 && errors==1);
        voltbro_foc_specific_control_1_0 legacy{};
        legacy.set_point_type=type; legacy.set_point_value=.25f;
        uint8_t a[5]{},b[5]{}; size_t na=5,nb=5;
        assert(voltbro_foc_Servo_1_0_serialize_(&command,a,&na)==0);
        assert(voltbro_foc_specific_control_1_0_serialize_(&legacy,b,&nb)==0);
        assert(na==5 && nb==5 && std::memcmp(a,b,5)==0);
    }
    device=Motor{}; device.valid=false; errors=0;
    servo_sub.handler({},nullptr); assert(errors==1);
    static_assert(voltbro_foc_command_1_0_EXTENT_BYTES_==28);
    static_assert(voltbro_foc_MITCommand_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_==20);
    FOCCommandSub sub(0,2118);
    for (size_t mtu : {8,12,16,20,24,32,48,64}) {
        // New publisher -> old subscriber: retain the four-field prefix.
        voltbro_foc_State_1_0 state{};
        state.timestamp.microsecond=0x123456789ABCDEULL;
        state.pos.radian=1.25f; state.vel.radian_per_second=-2.5f;
        state._torq.newton_meter=3.75f;
        uint8_t state_payload[voltbro_foc_State_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_]{};
        size_t state_size=sizeof(state_payload);
        assert(voltbro_foc_State_1_0_serialize_(&state,state_payload,&state_size)==0);
        assert(state_size==19);
        auto state_alloc=+[](CanardInstance*,size_t n)->void* {return std::malloc(n);};
        auto state_free=+[](CanardInstance*,void* p) {std::free(p);};
        auto state_tx=canardInit(state_alloc,state_free), state_rx=canardInit(state_alloc,state_free);
        state_tx.node_id=11; state_rx.node_id=42;
        auto state_queue=canardTxInit(32,mtu);
        CanardRxSubscription state_subscription{};
        assert(canardRxSubscribe(&state_rx,CanardTransferKindMessage,3811,
            voltbro_foc_state_simple_1_0_EXTENT_BYTES_,CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
            &state_subscription)>=0);
        CanardTransferMetadata state_metadata{};
        state_metadata.priority=CanardPriorityNominal;
        state_metadata.transfer_kind=CanardTransferKindMessage;
        state_metadata.port_id=3811; state_metadata.remote_node_id=CANARD_NODE_ID_UNSET;
        assert(canardTxPush(&state_queue,&state_tx,1000000,&state_metadata,state_size,state_payload)>0);
        int state_received=0;
        while (auto* item=canardTxPeek(&state_queue)) {
            CanardRxTransfer transfer{};
            int result=canardRxAccept(&state_rx,100,&item->frame,0,&transfer,nullptr);
            assert(result>=0);
            if (result==1) {
                ++state_received;
                voltbro_foc_state_simple_1_0 old_state{};
                size_t n=transfer.payload_size;
                assert(voltbro_foc_state_simple_1_0_deserialize_(&old_state,
                    static_cast<const uint8_t*>(transfer.payload),&n)==0);
                assert(old_state.timestamp.microsecond==state.timestamp.microsecond);
                assert(old_state.angle.radian==state.pos.radian);
                assert(old_state.velocity.radian_per_second==state.vel.radian_per_second);
                assert(old_state._torque.newton_meter==state._torq.newton_meter);
                assert(old_state.current.ampere==0 && old_state.bus_voltage.volt==0);
                assert(old_state.mcu_temp.kelvin==0 && old_state.stator_temp.kelvin==0 && !old_state.has_fault.value);
                std::printf("MTU=%zu State -> state_simple PASS\n",mtu);
                state_free(&state_rx,transfer.payload);
            }
            state_free(&state_tx,canardTxPop(&state_queue,item));
        }
        assert(state_received==1);
        canardRxUnsubscribe(&state_rx,CanardTransferKindMessage,3811);
        for (bool old : {false,true}) {
            uint8_t payload[28]{};
            size_t size=sizeof(payload);
            voltbro_foc_command_1_0 legacy{};
            legacy._torque.newton_meter=1; legacy.angle.radian=2;
            legacy.velocity.radian_per_second=3; legacy.angle_kp.value=4;
            legacy.velocity_kp.value=5; legacy.I_kp.value=6; legacy.I_ki.value=7;
            assert(voltbro_foc_command_1_0_serialize_(&legacy,payload,&size)==0 && size==28);
            if (!old) {
                voltbro_foc_MITCommand_1_0 msg{};
                msg._torq.newton_meter=1; msg.pos.radian=2; msg.vel.radian_per_second=3;
                msg.pos_gain.value=4; msg.vel_gain.value=5;
                uint8_t mit[20]{}; size=sizeof(mit);
                assert(voltbro_foc_MITCommand_1_0_serialize_(&msg,mit,&size)==0 && size==20);
                assert(std::memcmp(mit,payload,20)==0);
            }
            auto alloc=+[](CanardInstance*,size_t n)->void* {return std::malloc(n);};
            auto release=+[](CanardInstance*,void* p) {std::free(p);};
            auto tx=canardInit(alloc,release), rx=canardInit(alloc,release);
            tx.node_id=42; rx.node_id=11;
            auto queue=canardTxInit(32,mtu);
            CanardRxSubscription subscription{};
            assert(canardRxSubscribe(&rx,CanardTransferKindMessage,2118,voltbro_foc_MITCommand_1_0_EXTENT_BYTES_,
                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,&subscription)>=0);
            CanardTransferMetadata metadata{};
            metadata.priority=CanardPriorityNominal;
            metadata.transfer_kind=CanardTransferKindMessage;
            metadata.port_id=2118; metadata.remote_node_id=CANARD_NODE_ID_UNSET;
            assert(canardTxPush(&queue,&tx,1000000,&metadata,size,payload)>0);
            int received=0; device=Motor{}; errors=0;
            while (auto* item=canardTxPeek(&queue)) {
                CanardRxTransfer transfer{};
                int result=canardRxAccept(&rx,100,&item->frame,0,&transfer,nullptr);
                assert(result>=0);
                if (result==1) {
                    ++received;
                    voltbro_foc_MITCommand_1_0 decoded{}; size_t n=transfer.payload_size;
                    assert(transfer.payload_size==20);
                    assert(voltbro_foc_MITCommand_1_0_deserialize_(&decoded,
                        static_cast<const uint8_t*>(transfer.payload),&n)==0);
                    sub.handler(decoded,&transfer);
                    std::printf("MTU=%zu %s received=%zu\n",mtu,old?"legacy":"MIT",transfer.payload_size);
                    release(&rx,transfer.payload);
                }
                release(&tx,canardTxPop(&queue,item));
            }
            assert(received==1 && errors==0 && device.targets==1);
            assert(device.target.torque==1 && device.target.angle==2 && device.target.velocity==3);
            assert(device.target.angle_kp==4 && device.target.velocity_kp==5);
            assert(device.gains==0 && device.kp==17 && device.ki==19);
            canardRxUnsubscribe(&rx,CanardTransferKindMessage,2118);
        }
    }
    device=Motor{}; device.valid=false; errors=0;
    sub.handler({},nullptr);
    assert(errors==1 && device.gains==0);
    puts("PASS: MIT/legacy wire prefix, Classic CAN and CAN FD, legacy tail ignored, invalid target counted");
}
'''
with tempfile.TemporaryDirectory(prefix='vbdrive-mit-') as directory:
    tmp = Path(directory)
    (tmp / 'test.cpp').write_text(source)
    lib = root / 'Drivers/libcxxcanard/libs'
    subprocess.run(['cc', '-std=c11', '-I'+str(lib/'libcanard'), '-c',
                    str(lib/'libcanard/canard.c'), '-o', str(tmp/'canard.o')], check=True)
    subprocess.run(['c++', '-std=c++20', '-I'+str(lib),
                    '-I'+str(root/'build/RelWithDebInfo/cyphal_types/c'),
                    str(tmp/'test.cpp'), str(tmp/'canard.o'), '-o', str(tmp/'test')], check=True)
    subprocess.run([str(tmp/'test')], check=True)
