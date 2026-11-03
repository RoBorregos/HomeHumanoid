// Throwaway test tool: same relay approach as lowstate_relay.cc, but drives
// a forward velocity command (lx) instead of FSM transition buttons, to
// confirm the G1 actually walks once in the Velocity state.
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

std::atomic<float> g_lx{0};
std::mutex g_pub_mutex;
unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowState_> g_pub;

void handler(const void* message) {
    auto relayed = *(unitree_hg::msg::dds_::LowState_*)message;
    auto remote = relayed.wireless_remote();
    std::memset(remote.data(), 0, remote.size());
    uint16_t keys = 0;
    std::memcpy(remote.data() + 2, &keys, sizeof(keys));
    float lx = g_lx.load();
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

    std::cout << "relaying with forward lx command...\n";
    std::this_thread::sleep_for(500ms);

    std::cout << "walking forward (lx=0.3) for 4s\n";
    g_lx = 0.3f;
    std::this_thread::sleep_for(4s);

    std::cout << "stop\n";
    g_lx = 0.0f;
    std::this_thread::sleep_for(2s);
    return 0;
}
