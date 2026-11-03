// Throwaway test tool: relays unitree_mujoco's real LowState (rt/lowstate)
// unmodified except for overriding the embedded wireless_remote bytes with a
// synthetic button sequence, since uinput is unavailable on this Jetson
// kernel (no virtual joystick device possible) and directly publishing a
// standalone fake LowState would corrupt the real physics/motor data g1_ctrl
// relies on. This relay keeps real telemetry flowing while injecting just
// the joystick bits, driving Passive -> FixStand -> Velocity once.
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>

#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"

using namespace std::chrono_literals;

enum Bits : uint16_t {
    R1 = 1 << 0, L1 = 1 << 1, START = 1 << 2, SELECT = 1 << 3,
    R2 = 1 << 4, L2 = 1 << 5, F1 = 1 << 6, F2 = 1 << 7,
    A = 1 << 8, B = 1 << 9, X = 1 << 10, Y = 1 << 11,
    UP = 1 << 12, RIGHT = 1 << 13, DOWN = 1 << 14, LEFT = 1 << 15,
};

std::atomic<uint16_t> g_keys{0};
std::mutex g_pub_mutex;
unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowState_> g_pub;

std::atomic<float> g_lx{0.0f};

void handler(const void* message) {
    auto relayed = *(unitree_hg::msg::dds_::LowState_*)message;
    auto remote = relayed.wireless_remote();
    std::memset(remote.data(), 0, remote.size());
    uint16_t keys = g_keys.load();
    float lx = g_lx.load();
    std::memcpy(remote.data() + 2, &keys, sizeof(keys));
    std::memcpy(remote.data() + 4, &lx, sizeof(lx));
    relayed.wireless_remote(remote);
    std::lock_guard<std::mutex> lock(g_pub_mutex);
    g_pub->Write(relayed);
}

int main(int argc, char** argv) {
    std::string iface = argc > 1 ? argv[1] : "lo";
    unitree::robot::ChannelFactory::Instance()->Init(0, iface);

    g_pub.reset(new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
    g_pub->InitChannel();

    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> sub;
    sub.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
    sub->InitChannel(handler, 1);

    std::cout << "relaying rt/lowstate with synthetic wireless_remote overlay...\n";
    std::this_thread::sleep_for(500ms);

    std::cout << "1/6 neutral\n"; g_keys = 0; std::this_thread::sleep_for(500ms);
    std::cout << "2/6 hold LT\n"; g_keys = L2; std::this_thread::sleep_for(500ms);
    std::cout << "3/6 LT+up edge -> FixStand\n"; g_keys = L2 | UP; std::this_thread::sleep_for(300ms);
    std::cout << "4/6 neutral, waiting (generous, possible rendering-induced sim lag)\n";
    g_keys = 0; std::this_thread::sleep_for(15000ms);
    std::cout << "5/6 hold RB\n"; g_keys = R1; std::this_thread::sleep_for(500ms);
    std::cout << "6/6 RB+X edge -> Velocity\n"; g_keys = R1 | X; std::this_thread::sleep_for(300ms);
    std::cout << "release buttons, hold still for 3s\n"; g_keys = 0;
    std::this_thread::sleep_for(3000ms);
    std::cout << "walking forward (lx=0.3) for 5s\n"; g_lx = 0.3f;
    std::this_thread::sleep_for(5000ms);
    std::cout << "stop, keep relaying\n"; g_lx = 0.0f;

    std::this_thread::sleep_for(20s);
    return 0;
}
