
#include "stdinclude.h"
#include <array>

#include "pico/stdlib.h"

#include "hardware/adc.h"
#include "pico/bootrom.h"
#include "pico/error.h"
#include "../analogRead/analog_read.h"
#include "scancode.h"

namespace component {

    const uint8_t PIN_MAP[12] = {
            BTN_L1_PIN, BTN_L2_PIN, BTN_L3_PIN, BTN_LSIDE_PIN, BTN_LMENU_PIN,
            BTN_R1_PIN, BTN_R2_PIN, BTN_R3_PIN, BTN_RSIDE_PIN, BTN_RMENU_PIN,
            BTN_TEST_PIN, BTN_SERV_PIN
    };
    const uint8_t LED_PINS[6] = {
        LED_L1_PIN, LED_L2_PIN, LED_L3_PIN,
        LED_R1_PIN, LED_R2_PIN, LED_R3_PIN
    };

    const uint8_t PIN_BIT[12] = {  // for keyboard mode
            // L: A B C SIDE MENU
            0, 0, 0, 0, 1,
            0, 0, 0, 0, 1,
            1, 1};

    const uint8_t SWITCH_INDEX[12] = {
            0, 0, 0, 1, 1,
            0, 1, 0, 0, 0,
            0,0
    };

    const uint8_t SWITCH_OFFSET[12] = {
            0, 5, 4, 15, 14,
            1, 0, 15, 14, 13,
            9,6
    };

    const uint8_t KEYBOARD_KEYS[10] = {
        // L: A B C SIDE MENU
        USB_HID_SCANCODE_S, USB_HID_SCANCODE_D, USB_HID_SCANCODE_F, USB_HID_SCANCODE_R, USB_HID_SCANCODE_Q,
        // L: A B C SIDE MENU
        USB_HID_SCANCODE_J, USB_HID_SCANCODE_K, USB_HID_SCANCODE_L, USB_HID_SCANCODE_U, USB_HID_SCANCODE_P
    };



    
    auto LED_L1 = PicoLed::addLeds<PicoLed::WS2812B>(pio0, 0, LED_L1_PIN, 1, PicoLed::FORMAT_GRB);
    auto LED_L2 = PicoLed::addLeds<PicoLed::WS2812B>(pio0, 1, LED_L2_PIN, 1, PicoLed::FORMAT_GRB);
    auto LED_L3 = PicoLed::addLeds<PicoLed::WS2812B>(pio0, 2, LED_L3_PIN, 1, PicoLed::FORMAT_GRB);
    auto LED_R1 = PicoLed::addLeds<PicoLed::WS2812B>(pio0, 3, LED_R1_PIN, 1, PicoLed::FORMAT_GRB);
    auto LED_R2 = PicoLed::addLeds<PicoLed::WS2812B>(pio1, 0, LED_R2_PIN, 1, PicoLed::FORMAT_GRB);
    auto LED_R3 = PicoLed::addLeds<PicoLed::WS2812B>(pio1, 1, LED_R3_PIN, 1, PicoLed::FORMAT_GRB);
    
    std::array<PicoLed::PicoLedController, 6> leds = {LED_R1, LED_R2, LED_R3, LED_L1, LED_L2, LED_L3};

    bool hasI2cLever = false;
    uint8_t addr = 0b0000110;
    uint8_t reg1 = 0x03, reg2 = 0x04;
    ResponsiveAnalogRead analog(LEVER_PIN, true, 0.0005);
    namespace ongeki_hardware {
        void init() {

            for (unsigned char i: PIN_MAP) {
                gpio_init(i);
                gpio_set_dir(i, GPIO_IN);
                gpio_pull_up(i);
            }
            
            gpio_init(PICO_DEFAULT_LED_PIN);
            gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

            // check i2c lever
            auto writeResult = i2c_write_blocking_until(i2c_default, addr,
                                                        &reg1, 1, true, delayed_by_ms(get_absolute_time(), 10));
            if (writeResult == PICO_ERROR_GENERIC || writeResult == PICO_ERROR_TIMEOUT) {
                // no i2c lever;
                gpio_put(PICO_DEFAULT_LED_PIN, false);
            } else {
                hasI2cLever = true;
                gpio_put(PICO_DEFAULT_LED_PIN, true);
            }
        }
 
        const uint bitPosMap[] =
        {
                23, 20, 22, 19, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6
        };

        void set_led(uint ledData) {
            const uint8_t bitOffsets[6][3] = {{9,0,0}, {9,3,0}, {9,6,0}, {0,0,0}, {0,3,0}, {0,6,0}};
            for (int i = 0; i < 6; i++) {
                uint r = (ledData >> bitPosMap[bitOffsets[i][0] + bitOffsets[i][1]]) & 1;
                uint g = (ledData >> bitPosMap[bitOffsets[i][0] + bitOffsets[i][1] + 1]) & 1;
                uint b = (ledData >> bitPosMap[bitOffsets[i][0] + bitOffsets[i][1] + 2]) & 1;
                leds[i].setPixelColor(0, PicoLed::RGB(r*255,g*255, b*255));
            }
            
            for (auto& led : leds) {
                led.show();
            }
        }

        void set_led_brightness(uint8_t brightness) {
            for (auto& led : leds) {
                led.setBrightness(brightness);
            }
        }

        void show_mode_effect(uint8_t mode) {
            using namespace component::config;
            uint key_led = 0;

            switch(mode) {
            case MODE::IO4:
                key_led = 0xA0C440;
                break;
            case MODE::KEYBOARD:
                key_led = 0x8A4900;
                break;
            }
            
            for(int i = 0; i < 2; i++) {
                set_led(key_led);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                set_led(0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }


        uint16_t rawArr[6] = {};
        bool coin = false;
        bool rg = false;

        void update_hardware(component::io4_usb::output_t *data) {
            data->switches[0] = 0;
            data->switches[1] = 0;

            if (!gpio_get(BTN_COIN_PIN)) {
                if (!coin) {
                    data->coin[0].count++;
                    data->coin[1].count++;
                    coin = true;
                }
            } else {
                coin = false;
            }
            
            coin = false;
            rg = false;

            for (auto i = 0; i < 12; i++) {
                auto read = gpio_get(PIN_MAP[i]) ^ PIN_BIT[i];
                if (read) {
                    data->switches[SWITCH_INDEX[i]] += 1 << SWITCH_OFFSET[i];
                }
            }

            if (hasI2cLever) {
                uint8_t result1, result2;
                i2c_write_blocking(i2c_default, addr, &reg1, 1, true);
                i2c_read_blocking(i2c_default, addr, &result1, 1, false);

                i2c_write_blocking(i2c_default, addr, &reg2, 1, true);
                i2c_read_blocking(i2c_default, addr, &result2, 1, false);

                uint16_t finalResult = (result1 << 8) + result2;
                finalResult = finalResult > 16383 ? 65535 : finalResult << 2;
                finalResult = ~finalResult;
                data->analog[0] = *(int16_t *) &finalResult;
                data->rotary[0] = *(int16_t *) &finalResult;
            } else {
                analog.update();
                uint16_t raw = analog.getValue() << 4;
                data->analog[0] = *(int16_t *) &raw;
                data->rotary[0] = *(int16_t *) &raw;
            }
        }

        void add_key(uint8_t *keycodes, uint8_t key, int index) {
            if (index >= 6) return;
            keycodes[index] = key;
        }

        void update_keyboard(component::io4_usb::output_keyboard_t *data) {
            data->modifier = 0;
            memset(data->keycodes, 0, sizeof(data->keycodes));

            // inHello = !gpio_get(5);
            // do {
            //     if (!inHello) break;

            //     if (!gpio_get(PIN_MAP[7])) {
            //         reset_usb_boot(0, 0);
            //     }

            //     if (!gpio_get(PIN_MAP[6])) {
            //         if (!rg) {
            //             rg = true;
            //             uint8_t mode = config::cycle_mode();
            //             show_mode_effect(mode);
            //         }
            //     } else {
            //         rg = false;
            //     }

            // } while (false);

            rg = false;

            int key_index = 0;
            for (auto i = 0; i < 10; i++) {
                auto read = gpio_get(PIN_MAP[i]) ^ PIN_BIT[i];
                if (read) {
                    add_key(data->keycodes, KEYBOARD_KEYS[i], key_index++);
                }
            }
        }
    }
}

