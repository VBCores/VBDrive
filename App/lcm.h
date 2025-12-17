TYPE_ALIAS(FloatArray, uavcan_primitive_array_Real32_1_0)
TYPE_ALIAS(EmptyMsg, uavcan_primitive_Empty_1_0)
static constexpr CanardPortID LCM_RX_PORT = 1100;
static constexpr CanardPortID LCM_TX_PORT = 1101;
static constexpr CanardPortID LCM_REQUEST_PORT = 900;

void in_loop_reporting(millis current_t) {
    static millis report_time = 0;
    EACH_N(current_t, report_time, 1, {
        static CanardTransferID report_msg_transfer_id = 0;
        FloatArray::Type report_msg = {};

        report_msg.value.count = 7;
        report_msg.value.elements[0] = motor->get_angle();
        report_msg.value.elements[1] = motor->get_velocity();
        report_msg.value.elements[2] = motor->get_torque();
        report_msg.value.elements[3] = 0.0f;
        report_msg.value.elements[4] = 0.0;
        report_msg.value.elements[5] = 0.0f;
        report_msg.value.elements[6] = 0.0f;

        get_interface()->send_msg<FloatArray>(&report_msg, LCM_TX_PORT, &report_msg_transfer_id);
    })
}


class LCMCommandSub: public AbstractSubscription<FloatArray> {
public:
    LCMCommandSub(InterfacePtr interface): AbstractSubscription<FloatArray>(interface, LCM_RX_PORT) {};
    void handler(const FloatArray::Type& msg, CanardRxTransfer*) override {
        motor->set_foc_point(FOCTarget{
            .torque = msg.value.elements[4],
            .angle = msg.value.elements[0],
            .velocity = msg.value.elements[2],
            .angle_kp = msg.value.elements[1],
            .velocity_kp = msg.value.elements[3]
        });
        motor->set_current_regulator_params(msg.value.elements[6], msg.value.elements[7]);
    }
};

ReservedObject<LCMCommandSub> lcm_command_sub;
