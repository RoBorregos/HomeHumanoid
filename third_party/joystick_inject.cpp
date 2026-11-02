// Throwaway test tool: publishes synthetic WirelessController_ DDS messages
// to drive unitree_rl_lab's g1_ctrl FSM (Passive -> FixStand -> Velocity)
// without a physical gamepad. Bit layout from unitree_sdk2's
// example/wireless_controller/advanced_gamepad.hpp xKeySwitchUnion.
#include <chrono>
#include <thread>
#include <iostream>

#include "unitree/idl/go2/WirelessController_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_factory.hpp"

#define TOPIC_JOYSTICK "rt/wirelesscontroller"

using namespace std::chrono_literals;

enum Bits : uint16_t {
    R1 = 1 << 0,
    L1 = 1 << 1,
    START = 1 << 2,
    SELECT = 1 << 3,
    R2 = 1 << 4,
    L2 = 1 << 5,
    F1 = 1 << 6,
    F2 = 1 << 7,
    A = 1 << 8,
    B = 1 << 9,
    X = 1 << 10,
    Y = 1 << 11,
    UP = 1 << 12,
    RIGHT = 1 << 13,
    DOWN = 1 << 14,
    LEFT = 1 << 15,
};

int main(int argc, char** argv) {
    std::string iface = argc > 1 ? argv[1] : "lo";
    unitree::robot::ChannelFactory::Instance()->Init(0, iface);

    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::WirelessController_> pub;
    pub.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    pub->InitChannel();

    auto send = [&](uint16_t keys, float lx, float ly, float rx, float ry, int repeats, int delay_ms) {
        unitree_go::msg::dds_::WirelessController_ msg(lx, ly, rx, ry, keys);
        for (int i = 0; i < repeats; ++i) {
            pub->Write(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    };

    std::cout << "1/6 neutral\n";
    send(0, 0, 0, 0, 0, 20, 20);

    std::cout << "2/6 hold LT\n";
    send(L2, 0, 0, 0, 0, 20, 20);

    std::cout << "3/6 LT + up (edge) -> Passive to FixStand\n";
    send(L2 | UP, 0, 0, 0, 0, 10, 20);

    std::cout << "4/6 neutral\n";
    send(0, 0, 0, 0, 0, 30, 20);

    std::cout << "5/6 hold RB\n";
    send(R1, 0, 0, 0, 0, 20, 20);

    std::cout << "6/6 RB + X (edge) -> FixStand to Velocity\n";
    send(R1 | X, 0, 0, 0, 0, 10, 20);

    std::cout << "release\n";
    send(0, 0, 0, 0, 0, 10, 20);

    std::cout << "done\n";
    return 0;
}
