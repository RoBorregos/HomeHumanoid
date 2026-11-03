#include <iostream>
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
void handler(const void* message) { std::cout << "GOT hg lowstate" << std::endl; }
int main(int argc, char** argv) {
    unitree::robot::ChannelFactory::Instance()->Init(0, "lo");
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> sub;
    sub.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
    sub->InitChannel(handler, 1);
    std::cout << "sniffing hg lowstate for 5s..." << std::endl;
    sleep(5);
}
