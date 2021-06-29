#include "py/mphal.h"

void board_early_init(void) {
    #if 0
    // Turn off the USB switch
    #define USB_PowerSwitchOn pin_G6
    mp_hal_pin_output(USB_PowerSwitchOn);
    mp_hal_pin_low(USB_PowerSwitchOn);
    #endif
}
