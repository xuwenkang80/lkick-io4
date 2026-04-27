#include "stdinclude.h"


const uint8_t LED_PINS[6] = {
    LED_L1_PIN, LED_L2_PIN, LED_L3_PIN,
    LED_R1_PIN, LED_R2_PIN, LED_R3_PIN
};

int main() {
    board_init();
    uart_init(uart0, BAUD_RATE);
    i2c_init(i2c_default, 100*1000);
    gpio_set_function(21, GPIO_FUNC_I2C);
    gpio_set_function(20, GPIO_FUNC_I2C);
    gpio_pull_up(21);
    gpio_pull_up(20);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // initialise leds gpio
    for (unsigned char i: LED_PINS) {
        gpio_pull_down(LED_PINS[i]);
    }

    gpio_pull_up(23);

    component::config::init();
    component::config::init_from_boot_analog();
    component::ongeki_hardware::init();
    component::io4_usb::usb_init();
    component::serial::init();
    vTaskStartScheduler();
}
