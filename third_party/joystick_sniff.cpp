#include <iostream>
#include "unitree/idl/go2/WirelessController_.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"

#define TOPIC_JOYSTICK "rt/wirelesscontroller"

void handler(const void* message) {
    auto* m = (unitree_go::msg::dds_::WirelessController_*)message;
    std::cout << "GOT keys=" << m->keys() << " lx=" << m->lx() << std::endl;
}

int main(int argc, char** argv) {
    std::string iface = argc > 1 ? argv[1] : "lo";
    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> sub;
    sub.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    sub->InitChannel(handler, 1);
    std::cout << "sniffing...\n";
    while (true) { sleep(1); }
}
