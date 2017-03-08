#include "firmware.hpp"
#include "dummyboard.hpp"
#include "dummycommlink.hpp"

int main(void)
{
    using namespace rosflight;

    Firmware firmware(new DummyBoard(), new DummyCommLink());
    firmware.setup();

    while (true) {
        firmware.loop();
        firmware.get_board()->delay_micros(2000);
    }
}

