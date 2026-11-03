#include <iostream>
#include "unitree/idl/hg/SportModeState_.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"

void handler(const void* message) {
    auto* m = (unitree_hg::msg::dds_::SportModeState_*)message;
    auto pos = m->position();
    std::cout << "x=" << pos[0] << " y=" << pos[1] << " z=" << pos[2] << std::endl;
}

int main(int argc, char** argv) {
    std::string iface = argc > 1 ? argv[1] : "lo";
    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::SportModeState_> sub;
    sub.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::SportModeState_>("rt/sportmodestate"));
    sub->InitChannel(handler, 1);
    sleep(2);
    return 0;
}
