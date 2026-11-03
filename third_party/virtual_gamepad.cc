// Throwaway test tool: creates a virtual Xbox-layout joystick via /dev/uinput
// so unitree_mujoco's existing joystick-reading code (physics_joystick.h's
// XBoxJoystick, which reads /dev/input/jsN via the classic Linux joystick API)
// can drive the G1 FSM without a physical gamepad.
//
// Button index order (ascending evdev keycode of exactly these BTN_* codes,
// matching joydev's enumeration): A=0 B=1 X=2 Y=3 LB=4 RB=5 SELECT=6 START=7
// Axis index order (ascending evdev ABS_* code): LX=0 LY=1 LT=2 RX=3 RY=4 RT=5 DX=6 DY=7
// Mapping confirmed against third_party/unitree_mujoco/simulate/src/physics_joystick.h

#include <linux/uinput.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

static int fd;

void emit(int type, int code, int val) {
    struct input_event ev{};
    ev.type = type;
    ev.code = code;
    ev.value = val;
    write(fd, &ev, sizeof(ev));
}

void sync() { emit(EV_SYN, SYN_REPORT, 0); }

int main(int argc, char** argv) {
    fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) { std::cerr << "open /dev/uinput failed (need root?)\n"; return 1; }

    ioctl(fd, UI_SET_EVBIT, EV_KEY);
    int btns[] = {BTN_A, BTN_B, BTN_X, BTN_Y, BTN_TL, BTN_TR, BTN_SELECT, BTN_START};
    for (int b : btns) ioctl(fd, UI_SET_KEYBIT, b);

    ioctl(fd, UI_SET_EVBIT, EV_ABS);
    int axes[] = {ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_HAT0X, ABS_HAT0Y};
    for (int a : axes) ioctl(fd, UI_SET_ABSBIT, a);

    struct uinput_user_dev uidev{};
    strncpy(uidev.name, "HomeHumanoid Virtual Xbox Gamepad", UINPUT_MAX_NAME_SIZE);
    uidev.id.bustype = BUS_USB;
    uidev.id.vendor = 0x045e;
    uidev.id.product = 0x028e;
    for (int a : axes) {
        uidev.absmin[a] = -32768;
        uidev.absmax[a] = 32767;
        uidev.absfuzz[a] = 0;
        uidev.absflat[a] = 0;
    }

    write(fd, &uidev, sizeof(uidev));
    if (ioctl(fd, UI_DEV_CREATE) < 0) {
        std::cerr << "UI_DEV_CREATE failed\n";
        return 1;
    }

    std::cout << "virtual gamepad created, waiting for /dev/input/jsN to appear...\n";
    std::this_thread::sleep_for(1500ms);

    if (argc > 1 && std::string(argv[1]) == "sequence") {
        std::cout << "neutral\n";
        sync();
        std::this_thread::sleep_for(500ms);

        std::cout << "hold LT (axis Z = ABS_Z)\n";
        emit(EV_ABS, ABS_Z, 20000);
        sync();
        std::this_thread::sleep_for(500ms);

        std::cout << "up (axis DY = ABS_HAT0Y negative) -> Passive to FixStand\n";
        emit(EV_ABS, ABS_HAT0Y, -20000);
        sync();
        std::this_thread::sleep_for(300ms);
        emit(EV_ABS, ABS_HAT0Y, 0);
        sync();
        std::this_thread::sleep_for(500ms);

        std::cout << "release LT\n";
        emit(EV_ABS, ABS_Z, 0);
        sync();
        std::this_thread::sleep_for(500ms);

        std::cout << "hold RB (BTN_TR)\n";
        emit(EV_KEY, BTN_TR, 1);
        sync();
        std::this_thread::sleep_for(500ms);

        std::cout << "press X (BTN_X) -> FixStand to Velocity\n";
        emit(EV_KEY, BTN_X, 1);
        sync();
        std::this_thread::sleep_for(300ms);
        emit(EV_KEY, BTN_X, 0);
        sync();
        std::this_thread::sleep_for(500ms);

        std::cout << "release RB\n";
        emit(EV_KEY, BTN_TR, 0);
        sync();

        std::cout << "sequence done, keeping device alive (Ctrl+C to remove)\n";
    } else {
        std::cout << "device alive, no sequence requested (pass 'sequence' as arg)\n";
    }

    while (true) std::this_thread::sleep_for(1s);
}
