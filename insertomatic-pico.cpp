#include "pico/stdlib.h"

#include "lcd_display.hpp"

#include "hardware/gpio.h"
#include "pico/time.h"

#define NO_OF_CHANNELS 6

#define BUTTON_F 6
#define BUTTON_U 7
#define BUTTON_D 8
#define BUTTON_INTERVAL 50

#define I2C_SCL_PIN 16 // Define SCL pin
#define I2C_SDA_PIN_1 10 // Define SDA pin
#define I2C_SDA_PIN_2 11 // Define SDA pin
#define I2C_SDA_PIN_3 12 // Define SDA pin
#define I2C_SDA_PIN_4 13 // Define SDA pin
#define I2C_SDA_PIN_5 14 // Define SDA pin
#define I2C_SDA_PIN_6 15 // Define SDA pin
int I2C_SDA_PINS[] = {I2C_SDA_PIN_1, I2C_SDA_PIN_2, I2C_SDA_PIN_3, I2C_SDA_PIN_4, I2C_SDA_PIN_5, I2C_SDA_PIN_6};
#define I2C_ADDRESS (0xCA >> 1) // Define I2C address
#define I2C_DELAY 5 // Delay in microseconds for timing

#define FN_IDLE 0
#define FN_CHANNELS 1
#define FN_STANDARD 2
#define FN_TESTPATTERN 3

void gpio_put_all_sda_pins(bool value)
{
    gpio_put(I2C_SDA_PIN_1, value);
    gpio_put(I2C_SDA_PIN_2, value);
    gpio_put(I2C_SDA_PIN_3, value);
    gpio_put(I2C_SDA_PIN_4, value);
    gpio_put(I2C_SDA_PIN_5, value);
    gpio_put(I2C_SDA_PIN_6, value);
}

void gpio_set_dir_all_sda_pins(bool out)
{
    gpio_set_dir(I2C_SDA_PIN_1, out);
    gpio_set_dir(I2C_SDA_PIN_2, out);
    gpio_set_dir(I2C_SDA_PIN_3, out);
    gpio_set_dir(I2C_SDA_PIN_4, out);
    gpio_set_dir(I2C_SDA_PIN_5, out);
    gpio_set_dir(I2C_SDA_PIN_6, out);
}

// ChatGPT generated section
void i2c_delay() {
    sleep_us(I2C_DELAY);
}

void i2c_start() {
    gpio_set_dir_all_sda_pins(false);
    gpio_set_dir(I2C_SCL_PIN, GPIO_IN);
    i2c_delay();
    gpio_set_dir_all_sda_pins(true);
    i2c_delay();
    gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);
}

void i2c_stop() {
    gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);
    gpio_set_dir_all_sda_pins(true);
    i2c_delay();
    gpio_set_dir(I2C_SCL_PIN, GPIO_IN);
    i2c_delay();
    gpio_set_dir_all_sda_pins(false);
    i2c_delay();
}

void i2c_write_bits(bool bits[]) {
    for (int i = 0; i < NO_OF_CHANNELS; i++) {
        if (bits[i] == 0)
        {
            gpio_set_dir(I2C_SDA_PINS[i], true);
        }
        else
        {
            gpio_set_dir(I2C_SDA_PINS[i], false);
        }
    }
    i2c_delay();
    gpio_set_dir(I2C_SCL_PIN, GPIO_IN);
    i2c_delay();
    gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);
    i2c_delay();
}

bool i2c_read_ack() {
    gpio_set_dir_all_sda_pins(GPIO_IN);
    i2c_delay();
    gpio_set_dir(I2C_SCL_PIN, GPIO_IN);
    i2c_delay();
    bool ack = 1; //!gpio_get(I2C_SDA_PIN);
    gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);
    gpio_set_dir_all_sda_pins(GPIO_OUT);
    return ack;
}

bool i2c_write_bytes(uint8_t bytes[][4], uint8_t byteno) {
    bool bits[NO_OF_CHANNELS];
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < NO_OF_CHANNELS; j++) {
            bits[j] = (bytes[j][byteno] >> (7 - i)) & 0x01;
        }
        i2c_write_bits(bits);
    }
    return i2c_read_ack();
}

bool i2c_write_byte(uint8_t byte) {
    bool bits[NO_OF_CHANNELS];
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < NO_OF_CHANNELS; j++) {
            bits[j] = byte & 0x80;
        }
        byte <<= 1;
        i2c_write_bits(bits);
    }
    return i2c_read_ack();
}

void i2c_bitbang_write(uint8_t address, uint8_t data[][4], size_t length) {
    i2c_start();
    if (!i2c_write_byte(address << 1)) { // Send address with write bit
        i2c_stop();
        return;
    }
    for (size_t i = 0; i < length; i++) {
        if (!i2c_write_bytes(data, i)) {
            i2c_stop();
            return;
        }
    }
    i2c_stop();
}

// End of ChatGPT generated section

LCDdisplay myLCD(5,4,3,2,1,0,24,2); // DB4, DB5, DB6, DB7, RS, E, character_width, no_of_lines

void printChannelNumbers(uint8_t * channels)
{
    for (int i = 0; i < NO_OF_CHANNELS; i++)
    {
        myLCD.write(' ');
        myLCD.write((channels[i] & 0x80) ? 'S' : 'C');
        myLCD.write((channels[i] & 0x7f) / 10 % 10 + '0');
        myLCD.write((channels[i] & 0x7f) % 10 + '0');
    }
}

void printSystemTestEnable(uint8_t standard, bool testpattern)
{
    myLCD.print(" System: ");
    switch (standard)
    {
        case 0:
        myLCD.print(" M ");
        break;

        case 1:
        myLCD.print("B/G");
        break;
        
        case 2:
        myLCD.print(" I ");
        break;

        case 3:
        myLCD.print("D/K");
        break;
        
        default:
        myLCD.print(" L ");
        break;        
    }

    myLCD.print("  Test: ");
    myLCD.print(testpattern ? "On  " : "Off ");
}

void programModulators(uint8_t * channels, uint8_t standard, bool testpattern)
{
    // Modulator data array
    uint8_t data[NO_OF_CHANNELS][4];
    /* Example array data
        = {
        {0x80, 0x10, 0x5D, 0x74}, // 21
        {0x80, 0x10, 0x5E, 0x74}, // 23
        {0x80, 0x10, 0x5F, 0x74}, // 25
        {0x80, 0x10, 0x60, 0x74}, // 27
        {0x80, 0x10, 0x61, 0x74}, // 29
        {0x80, 0x10, 0x62, 0x74}  // 31
    };*/

    bool secam = (standard >> 2) & 1;
    if (secam)
    {
        standard = 3;
    }

    for (uint8_t i = 0; i < NO_OF_CHANNELS; i++) {
        data[i][0] = 0x80 | secam;
        data[i][1] = standard << 3;
        uint16_t desired_frequency = (channels[i] - 21) * 4 * 8 + 1885;
        uint16_t desired_n = (desired_frequency << 2) & 0x3ffc;
        data[i][2] = (desired_n >> 8) | (testpattern << 6);
        data[i][3] = desired_n & 0xff;
    }
    
    i2c_bitbang_write(I2C_ADDRESS, data, 4);
}

int main() {
    sleep_ms(100);
	myLCD.init();
    myLCD.clear();
    myLCD.print("Insertomatic 6000 ------");
    sleep_ms(2500);
    //sleep_ms(2500);
    //myLCD.goto_pos(0,0);
    //myLCD.print("P1 [Station name]       ");
    //myLCD.goto_pos(0,1);
    //myLCD.print("[Now playing info]      ");
    //sleep_ms(2500);

    gpio_init_mask((1 << BUTTON_F) | (1 << BUTTON_D) | (1 << BUTTON_U));
    gpio_set_pulls(BUTTON_F, true, false);
    gpio_set_pulls(BUTTON_U, true, false);
    gpio_set_pulls(BUTTON_D, true, false);

    gpio_init(I2C_SCL_PIN);
    gpio_set_dir(I2C_SCL_PIN, GPIO_IN);
    gpio_put(I2C_SCL_PIN, 0);

    gpio_init(I2C_SDA_PIN_1);
    gpio_init(I2C_SDA_PIN_2);
    gpio_init(I2C_SDA_PIN_3);
    gpio_init(I2C_SDA_PIN_4);
    gpio_init(I2C_SDA_PIN_5);
    gpio_init(I2C_SDA_PIN_6);
    gpio_set_dir(I2C_SDA_PIN_1, GPIO_IN);
    gpio_set_dir(I2C_SDA_PIN_2, GPIO_IN);
    gpio_set_dir(I2C_SDA_PIN_3, GPIO_IN);
    gpio_set_dir(I2C_SDA_PIN_4, GPIO_IN);
    gpio_set_dir(I2C_SDA_PIN_5, GPIO_IN);
    gpio_set_dir(I2C_SDA_PIN_6, GPIO_IN);
    gpio_put_all_sda_pins(0);

    

    uint8_t standard = 2;
    bool testpattern = true;
    uint8_t channels[] = {21, 23, 25, 27, 29, 31};
    programModulators(channels, standard, testpattern);

    myLCD.goto_pos(0,0);
    myLCD.print(" P1  P2  P3  P4  P5  P6 ");
    myLCD.goto_pos(0,1);
    printChannelNumbers(channels);

    uint32_t lastms = 0;
    uint32_t function = FN_IDLE;
    uint32_t channelSelect = 0;
    bool buttonFLast = 1;
    bool buttonULast = 1;
    bool buttonDLast = 1;

    // Main loop
    while (true)
    {
        if (to_ms_since_boot(get_absolute_time()) > lastms + BUTTON_INTERVAL)
        {
            lastms = to_ms_since_boot(get_absolute_time());

            bool buttonFState = gpio_get(BUTTON_F);
            bool buttonUState = gpio_get(BUTTON_U);
            bool buttonDState = gpio_get(BUTTON_D);

            if (!buttonFState && buttonFLast)
            {
                switch (function)
                {
                    case FN_IDLE:
                    function = FN_CHANNELS;
                    myLCD.goto_pos(0,1);
                    printChannelNumbers(channels);
                    myLCD.cursor_on();
                    myLCD.goto_pos(3, 1);
                    break;

                    case FN_CHANNELS:
                    if (channelSelect < 5)
                    {
                        channelSelect++;
                        myLCD.goto_pos(3 + channelSelect * 4, 1);
                    }
                    else
                    {
                        channelSelect = 0;
                        function = FN_STANDARD;
                        myLCD.goto_pos(0,1);
                        printSystemTestEnable(standard, testpattern);
                        myLCD.goto_pos(10,1);
                    }
                    break;

                    case FN_STANDARD:
                    function = FN_TESTPATTERN;
                    myLCD.goto_pos(20,1);
                    break;

                    case FN_TESTPATTERN:
                    programModulators(channels, standard, testpattern);
                    function = FN_IDLE;
                    myLCD.cursor_off();
                    myLCD.goto_pos(0,0);
                    myLCD.print("Modulator settings      ");
                    myLCD.goto_pos(0,1);
                    myLCD.print("updated                 ");
                    break;
                }
            }

            if (!buttonUState && buttonULast)
            {
                switch (function)
                {
                    case FN_CHANNELS:
                    channels[channelSelect]++;
                    myLCD.goto_pos(0,1);
                    printChannelNumbers(channels);
                    myLCD.goto_pos(3 + channelSelect * 4, 1);
                    break;

                    case FN_STANDARD:
                    standard = (standard < 4) ? standard + 1 : 0;
                    myLCD.goto_pos(0,1);
                    printSystemTestEnable(standard, testpattern);
                    myLCD.goto_pos(10,1);
                    break;

                    case FN_TESTPATTERN:
                    testpattern ^= 1;
                    myLCD.goto_pos(0,1);
                    printSystemTestEnable(standard, testpattern);
                    myLCD.goto_pos(20,1);
                    break;
                }
            }

            if (!buttonDState && buttonDLast)
            {
                switch (function)
                {
                    case FN_CHANNELS:
                    channels[channelSelect]--;
                    myLCD.goto_pos(0,1);
                    printChannelNumbers(channels);
                    myLCD.goto_pos(3 + channelSelect * 4, 1);
                    break;

                    case FN_STANDARD:
                    standard = (standard == 0) ? 4 : standard - 1;
                    myLCD.goto_pos(0,1);
                    printSystemTestEnable(standard, testpattern);
                    myLCD.goto_pos(10,1);
                    break;

                    case FN_TESTPATTERN:
                    testpattern ^= 1;
                    myLCD.goto_pos(0,1);
                    printSystemTestEnable(standard, testpattern);
                    myLCD.goto_pos(20,1);
                    break;
                }
            }

            buttonFLast = buttonFState;
            buttonULast = buttonUState;
            buttonDLast = buttonDState;
        }
    }
	return 0;
};
