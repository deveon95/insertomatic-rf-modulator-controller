#include "pico/stdlib.h"

#include "lcd_display.hpp"

#include "hardware/gpio.h"
#include "hardware/flash.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "uart_rx.pio.h"

// Button GPIOs
#define BUTTON_F 6
#define BUTTON_U 7
#define BUTTON_D 8
// Button polling interval
#define BUTTON_INTERVAL 50

// Define I2C pins for the modulators
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

// Menu definitions
#define FN_IDLE 0
#define FN_TOP_MENU 1
#define FN_CHANNELS 2
#define FN_STANDARD 3
#define FN_TESTPATTERN 4
#define FN_SWITCH_BANK 5
#define FN_COMPLETE 6

#define TOP_MENU_ITEMS 6

#define NO_OF_CHANNELS 6
#define NO_OF_CHANNEL_BANKS 8

/*
* Flash layout
* 0x000-0x001: fixed to "TX"
* 0x002-0x003: checksum
* 0x004-0x005: number of writes
* 0x006      : bank selection, system selection and test enable
* 0x00A-0x00F: bank 0
* 0x010-0x015: bank 1
* 0x016-0x01B: bank 2
* 0x01C-0x021: bank 3
* 0x022-0x027: bank 4
* 0x028-0x02D: bank 5
* 0x02E-0x033: bank 6
* 0x034-0x039: bank 7
* 0x03A-0x03F: bank 8
*/

// Create flash definitions
// Flash for settings storage will be the last sector (4096 bytes)
#define FLASH_TARGET_OFFSET (2097152 - FLASH_SECTOR_SIZE)
#define FLASH_USED_SIZE 0x040
#define FLASH_FIXED_CHECK_CHAR_0 'T'
#define FLASH_FIXED_CHECK_CHAR_1 'A'
// flash_target_contents can be used as an array, which can be read from to read the contents
// of the area of flash defined above, but this cannot be written to directly
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

// UART configuration
#define UART_BAUD 19200
#define UART_CHANNELS 4
#define FIRST_P_WITH_UART 3
#define UART_PIN_1 18
#define UART_PIN_2 20
#define UART_PIN_3 21
#define UART_PIN_4 19
#define IDLE_SCREEN_INTERVAL 2000
#define LCD_LINES 2
#define LCD_COLS 24

#define REBOOT_PIN 17

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

LCDdisplay myLCD(5,4,3,2,1,0,LCD_COLS,LCD_LINES); // DB4, DB5, DB6, DB7, RS, E, character_width, no_of_lines

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
        uint8_t channelNumber = channels[i] & 0x7f;
        uint8_t channelBand = (channels[i] & 0x80) >> 7;
        uint8_t divider = 0;
        uint16_t desired_frequency = 0;
        
        // Calculate the modulator frequency, desired_frequency will be frequency * 4 * (2 ^ divider).
        // For each section of the band, the base value of desired_frequency has been calculated by hand
        // e.g. C21 (471.25MHz) is 471.25 * 4 = 1885.
        // The channel number relative to that of the base value is then multiplied by the
        // channel spacing * 4 * (2 ^ divider) to get desired_frequency.
        if (channelBand == 0 && channelNumber >= 21)
        {
            divider = 0;
            desired_frequency = (channelNumber - 21) * 8 * 4 + 1885;
        }
        else if (channelBand == 1 && channelNumber >= 36) 
        {
            divider = 0;
            desired_frequency = (channelNumber - 36) * 8 * 4 + 1693;
        }
        else if (channelBand == 1 && channelNumber >= 21)
        {
            divider = 1;
            desired_frequency = (channelNumber - 21) * 8 * 4 * 2 + 2426;
        }
        else if (channelBand == 1 && channelNumber >= 11)
        {
            divider = 1;
            desired_frequency = (channelNumber - 11) * 7 * 4 * 2 + 1850;
        }
        else if (channelBand == 0 && channelNumber >= 10)
        {
            divider = 1;
            desired_frequency = (channelNumber - 10) * 7 * 4 * 2 + 1682;
        }
        else if (channelBand == 0 && channelNumber >= 5)
        {
            divider = 2;
            desired_frequency = (channelNumber - 5) * 7 * 4 * 4 + 2804;
        }
        else
        {
            divider = 2;
            desired_frequency = (channelNumber - 1) * 7 * 4 * 4 + 1684;
        }

        uint16_t desired_n = ((desired_frequency << 2) & 0x3ffc) | divider;

        data[i][0] = 0x80 | secam;
        data[i][1] = standard << 3;
        data[i][2] = (desired_n >> 8) | (testpattern << 6);
        data[i][3] = desired_n & 0xff;
    }
    
    i2c_bitbang_write(I2C_ADDRESS, data, 4);
}

void decrementChannel(uint8_t * channel)
{
    uint8_t channelNumber = *channel & 0x7f;
    uint8_t channelBand = (*channel & 0x80) >> 7;
    if (channelBand == 0 && channelNumber == 21)
    {
        channelBand = 1;
        channelNumber = 41;
    }
    else if (channelBand == 1 && channelNumber == 11)
    {
        channelBand = 0;
        channelNumber = 12;
    }
    else if (channelBand == 0 && channelNumber == 5)
    {
        channelBand = 1;
        channelNumber = 10;
    }
    else if (channelBand == 1 && channelNumber == 1)
    {
        channelBand = 0;
        channelNumber = 90;
    }
    else
    {
        channelNumber--;
    }
    *channel = (channelBand << 7) | channelNumber;
}

void incrementChannel(uint8_t * channel)
{
    uint8_t channelNumber = *channel & 0x7f;
    uint8_t channelBand = (*channel & 0x80) >> 7;
    if (channelBand == 1 && channelNumber == 10)
    {
        channelBand = 0;
        channelNumber = 5;
    }
    else if (channelBand == 0 && channelNumber == 12)
    {
        channelBand = 1;
        channelNumber = 11;
    }
    else if (channelBand == 1 && channelNumber == 41)
    {
        channelBand = 0;
        channelNumber = 21;
    }
    else if (channelBand == 0 && channelNumber == 90)
    {
        channelBand = 1;
        channelNumber = 1;
    }
    else
    {
        channelNumber++;
    }

    *channel = (channelBand << 7) | channelNumber;
}

void topMenuPrint(uint32_t menuItem)
{
    myLCD.goto_pos(0,0);
    myLCD.print(" Insertomatic 6000 Menu ");
    myLCD.goto_pos(0,1);
    switch (menuItem)
    {
        case 0: myLCD.print("<         Exit         >"); break;
        case 1: myLCD.print("<Set Modulator Channels>"); break;
        case 2: myLCD.print("< Switch Channel Bank  >"); break;
        case 3: myLCD.print("<    Save to FLASH     >"); break;
        case 4: myLCD.print("<    Full Shutdown     >"); break;
        case 5: myLCD.print("<     Full Reboot      >"); break;
    }
}

uint16_t saveFlash(uint8_t standard, uint8_t testpattern, uint8_t currentBank, uint8_t channels[NO_OF_CHANNEL_BANKS][NO_OF_CHANNELS])
{
    uint8_t flashData[FLASH_PAGE_SIZE];
    uint16_t numberOfWrites = flash_target_contents[0x004] << 8 | flash_target_contents[0x005];
    numberOfWrites++;

    flashData[0x000] = FLASH_FIXED_CHECK_CHAR_0;
    flashData[0x001] = FLASH_FIXED_CHECK_CHAR_1;
    flashData[0x004] = numberOfWrites >> 8;
    flashData[0x005] = numberOfWrites & 0xFF;

    flashData[0x006] = ((currentBank & 0b111) << 4) | ((standard & 0b111) << 1) | (testpattern & 1);
    for (uint8_t i = 0; i < NO_OF_CHANNEL_BANKS; i++)
    {
        for (uint8_t j = 0; j < NO_OF_CHANNELS; j++)
        {
             flashData[0x00A + i * NO_OF_CHANNELS + j] = channels[i][j];
        }
    }

    // Minimum write is one page, so pad out the rest of the page with 0xFFs to leave it unwritten
    for (uint32_t i = FLASH_USED_SIZE; i < FLASH_PAGE_SIZE - 1; i++)
    {
        flashData[i] = 0xFF;
    }

    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, flashData, FLASH_PAGE_SIZE);
    return numberOfWrites;
}

int main() {
    // Define main function local variables
    uint8_t currentBank = 0;
    uint8_t standard = 2;
    bool testpattern = true;
    uint8_t channels[NO_OF_CHANNEL_BANKS][NO_OF_CHANNELS];
    uint8_t uartLineOnes[UART_CHANNELS][LCD_COLS + 1];
    uint8_t uartLineTwos[UART_CHANNELS][LCD_COLS + 1];
    uint8_t uartChar;
    uint8_t uartsWhichHaveReceived = 0;
    uint8_t uartCol[UART_CHANNELS];
    uint8_t uartRow[UART_CHANNELS];

    uint32_t lastms = 0;
    uint32_t lastmsIdleScreenChange = 0;
    uint32_t idleScreen = 0;
    uint32_t function = FN_IDLE;
    uint32_t menuSelect = 0;
    bool buttonFLast = 1;
    bool buttonULast = 1;
    bool buttonDLast = 1;

    sleep_ms(100);
	myLCD.init();
    myLCD.clear();
    myLCD.print("   Insertomatic 6000    ");
    //sleep_ms(2500);

    // Initialise the GPIO
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

    gpio_init(REBOOT_PIN);
    gpio_set_dir(REBOOT_PIN, GPIO_IN);
    gpio_set_pulls(REBOOT_PIN, true, false);

    // Boot progress indicator used to make it possible to see where it freezes up if it does.
    myLCD.goto_pos(0,1);
    myLCD.print("A");

    // Initialise the data arrays for received UART data
    for (uint8_t i = 0; i < UART_CHANNELS; i++)
    {
        myLCD.goto_pos(1,1);
        myLCD.write('0' + i);
        for (uint8_t j = 0; j < LCD_COLS; j++)
        {
            myLCD.goto_pos(2,1);
            myLCD.write('0' + (j / 10));
            myLCD.write('0' + (j % 10));
            uartLineOnes[i][j] = ' ';
            uartLineTwos[i][j] = ' ';
        }
        uartLineOnes[i][0] = 'P';
        uartLineOnes[i][1] = '0' + FIRST_P_WITH_UART + i;
        uartLineOnes[i][2] = ':';
        uartLineOnes[i][LCD_COLS] = 0;
        uartLineTwos[i][LCD_COLS] = 0;
        uartCol[i] = 4;
        uartRow[i] = 0;
    }

    myLCD.goto_pos(0,1);
    myLCD.print("B   ");

    // Initialise the channels array with the default channels that will be used
    // should the saved channel numbers be unable to be read from flash
    for (uint8_t i = 0; i < NO_OF_CHANNEL_BANKS; i++)
    {
        for (uint8_t j = 0; j < NO_OF_CHANNELS; j++)
        {
            channels[i][j] = 21 + i + j * 2;
        }
    }
    myLCD.goto_pos(0,1);
    myLCD.print("C");
    
    // Read the flash
    if (flash_target_contents[0] == FLASH_FIXED_CHECK_CHAR_0 && flash_target_contents[1] == FLASH_FIXED_CHECK_CHAR_1)
    {
        // Fixed identification characters matched so load all the data
        currentBank = (flash_target_contents[0x06] >> 4) & 0x07;
        standard = (flash_target_contents[0x06] >> 1) & 0x07;
        testpattern = flash_target_contents[0x06] & 1;
        for (uint8_t i = 0; i < NO_OF_CHANNEL_BANKS; i++)
        {
            for (uint8_t j = 0; j < NO_OF_CHANNELS; j++)
            {
                channels[i][j] = flash_target_contents[0x00A + i * NO_OF_CHANNELS + j];
            }
        }
    }
    else
    {
        // Fixed identification characters did not match
        myLCD.goto_pos(0,0);
        myLCD.print("No saved data found in  ");
        myLCD.goto_pos(0,1);
        myLCD.print("flash memory.   Data: ");
        myLCD.write(flash_target_contents[0]);
        myLCD.write(flash_target_contents[1]);
        sleep_ms(2500);
    }

    myLCD.goto_pos(0,1);
    myLCD.print("D");

    programModulators(channels[currentBank], standard, testpattern);

    myLCD.goto_pos(0,1);
    myLCD.print("E");

    // Initialise UARTs
    uint offset = pio_add_program(pio0, &uart_rx_program);
    uart_rx_program_init(pio0, 0, offset, UART_PIN_1, UART_BAUD);
    uart_rx_program_init(pio0, 1, offset, UART_PIN_2, UART_BAUD);
    uart_rx_program_init(pio0, 2, offset, UART_PIN_3, UART_BAUD);
    uart_rx_program_init(pio0, 3, offset, UART_PIN_4, UART_BAUD);

    myLCD.goto_pos(0,1);
    myLCD.print("F");

    // Main loop
    while (true)
    {
        // Check UARTs
        for (uint8_t i = 0; i < UART_CHANNELS; i++)
        {
            while (pio_sm_get_rx_fifo_level(pio0, i))
            {
                // Read from PIO FIFO
                uartChar = uart_rx_program_getc(pio0, i);
                if (uartChar == '\n')
                {
                    // Newline character sets cursor back to the start of line 2
                    uartCol[i] = 0;
                    uartRow[i] = 1;
                }
                else if (uartChar == '\0')
                {
                    // Null terminator sets cursor back to start of line 1 after
                    // the channel number indicator
                    uartCol[i] = 4;
                    uartRow[i] = 0;
                }
                else if (uartChar == '\b')
                {
                    // Backspace character clears the 
                    for (uint8_t j = 0; j < LCD_COLS; j++)
                    {
                        uartLineTwos[i][j] = ' ';
                        if (j > 2)
                        {
                            uartLineOnes[i][j] = ' ';
                        }
                    }
                    uartCol[i] = 4;
                    uartRow[i] = 0;
                }
                else
                {
                    // Write character only if column position is within LCD size
                    // to avoid overwriting the next memory space if a longer string
                    // is received via the UART
                    if (uartCol[i] < LCD_COLS)
                    {
                        if (uartRow[i] == 0)
                        {
                            uartLineOnes[i][uartCol[i]] = uartChar;
                        }
                        else
                        {
                            uartLineTwos[i][uartCol[i]] = uartChar;
                        }
                        // Move to next character position for next byte
                        uartCol[i]++;

                        // Set bit to indicate that the current UART has received
                        // some data at least once since powerup
                        uartsWhichHaveReceived |= (1 << i);
                    }
                }
            }
        }

        if (function == FN_IDLE)
        {
            // Display idle LCD stuff
            if (to_ms_since_boot(get_absolute_time()) > lastmsIdleScreenChange + IDLE_SCREEN_INTERVAL)
            {
                lastmsIdleScreenChange = to_ms_since_boot(get_absolute_time());

                if (idleScreen == 0)
                {
                    // First screen is all the channel numbers
                    myLCD.goto_pos(0,0);
                    myLCD.print(" P1  P2  P3  P4  P5  P6 ");
                    myLCD.goto_pos(0,1);
                    printChannelNumbers(channels[currentBank]);
                }
                else
                {
                    // Subsequent screens are the UART-received now playing info
                    myLCD.goto_pos(0,0);
                    myLCD.print((char *)uartLineOnes[idleScreen - 1]);
                    myLCD.goto_pos(0,1);
                    myLCD.print((char *)uartLineTwos[idleScreen - 1]);
                }
                if (idleScreen < UART_CHANNELS)
                {
                    // Increment index of currently shown screen on idle screen
                    idleScreen++;
                    // Check whether UART data has been received for the newly selected
                    // screen yet, if not, skip this screen.
                    while (((uartsWhichHaveReceived >> (idleScreen - 1)) & 1) == 0)
                    {
                        if (idleScreen < UART_CHANNELS)
                        {
                            idleScreen++;
                        }
                        else
                        {
                            idleScreen = 0;
                            break;
                        }
                    }
                }
                else
                {
                    idleScreen = 0;
                }
            }
        }
        else
        {
            // Ensures that first screen is displayed immediately upon leaving menu
            idleScreen = 0;
            lastmsIdleScreenChange = 0;
        }

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
                    function = FN_TOP_MENU;
                    menuSelect = 0;
                    topMenuPrint(menuSelect);
                    break;

                    case FN_TOP_MENU:
                    if (menuSelect == 0)
                    {
                        function = FN_IDLE;
                    }
                    else if (menuSelect == 1)
                    {
                        function = FN_CHANNELS;
                        menuSelect = 0;
                        myLCD.goto_pos(0,1);
                        printChannelNumbers(channels[currentBank]);
                        myLCD.cursor_on();
                        myLCD.goto_pos(3, 1);
                    }
                    else if (menuSelect == 2)
                    {
                        function = FN_SWITCH_BANK;
                        menuSelect = 0;
                        myLCD.goto_pos(0,0);
                        myLCD.print("Selected bank:         ");
                        myLCD.write('0' + currentBank);
                        myLCD.goto_pos(0,1);
                        printChannelNumbers(channels[currentBank]);
                    }
                    else if (menuSelect == 3)
                    {
                        function = FN_COMPLETE;
                        myLCD.goto_pos(0,1);
                        myLCD.print("Saving...               ");
                        uint16_t numberOfSaves = saveFlash(standard, testpattern, currentBank, channels);
                        myLCD.goto_pos(0,1);
                        myLCD.print("Saved.   ");
                        myLCD.write(numberOfSaves / 10000 + '0');
                        myLCD.write((numberOfSaves / 1000 % 10) + '0');
                        myLCD.write((numberOfSaves / 100 % 10) + '0');
                        myLCD.write((numberOfSaves / 10 % 10) + '0');
                        myLCD.write((numberOfSaves % 10) + '0');
                    }
                    else if (menuSelect == 4 || menuSelect == 5)
                    {
                        function = FN_COMPLETE;
                        myLCD.goto_pos(0,1);
                        myLCD.print("Sending signal...       ");
                        gpio_set_dir(REBOOT_PIN, GPIO_OUT);
                        sleep_ms(menuSelect == 4 ? 3000 : 1000);
                        myLCD.goto_pos(0,1);
                        myLCD.print("Signal sent             ");
                        gpio_set_dir(REBOOT_PIN, GPIO_IN);
                    }
                    break;

                    case FN_COMPLETE :
                    function = FN_TOP_MENU;
                    menuSelect = 0;
                    topMenuPrint(menuSelect);
                    break;

                    case FN_CHANNELS:
                    if (menuSelect < 5)
                    {
                        menuSelect++;
                        myLCD.goto_pos(3 + menuSelect * 4, 1);
                    }
                    else
                    {
                        menuSelect = 0;
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
                    programModulators(channels[currentBank], standard, testpattern);
                    menuSelect = 0;
                    myLCD.cursor_off();
                    function = FN_TOP_MENU;
                    topMenuPrint(menuSelect);
                    break;

                    case FN_SWITCH_BANK:
                    programModulators(channels[currentBank], standard, testpattern);
                    function = FN_TOP_MENU;
                    topMenuPrint(menuSelect);
                    break;
                }
            }

            if (!buttonUState && buttonULast || !buttonDState && buttonDLast)
            {
                switch (function)
                {
                    case FN_TOP_MENU:
                    if (!buttonUState && buttonULast)
                    {
                        // If it's the up button...
                        if (menuSelect < TOP_MENU_ITEMS - 1)
                        {
                            menuSelect++;
                        }
                        else
                        {
                            menuSelect = 0;
                        }
                    }
                    else
                    {
                        // If it's the down button...
                        if (menuSelect > 0)
                        {
                            menuSelect--;
                        }
                        else
                        {
                            menuSelect = TOP_MENU_ITEMS - 1;
                        }
                    }
                    topMenuPrint(menuSelect);
                    break;

                    case FN_CHANNELS:
                    if (!buttonUState && buttonULast)
                    {
                        incrementChannel(&channels[currentBank][menuSelect]);
                    }
                    else
                    {
                        decrementChannel(&channels[currentBank][menuSelect]);
                    }
                    myLCD.goto_pos(0,1);
                    printChannelNumbers(channels[currentBank]);
                    myLCD.goto_pos(3 + menuSelect * 4, 1);
                    break;

                    case FN_STANDARD:
                    if (!buttonUState && buttonULast)
                    {
                        standard = (standard < 4) ? standard + 1 : 0;
                    }
                    else
                    {
                        standard = (standard == 0) ? 4 : standard - 1;
                    }
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

                    case FN_SWITCH_BANK:
                    if (!buttonUState && buttonULast)
                    {
                        // If it's the up button...
                        if (currentBank < NO_OF_CHANNEL_BANKS - 1)
                        {
                            currentBank++;
                        }
                        else
                        {
                            currentBank = 0;
                        }
                    }
                    else
                    {
                        // If it's the down button...
                        if (currentBank > 0)
                        {
                            currentBank--;
                        }
                        else
                        {
                            currentBank = NO_OF_CHANNEL_BANKS - 1;
                        }
                    }
                    myLCD.goto_pos(0,0);
                    myLCD.print("Selected bank:         ");
                    myLCD.write('0' + currentBank);
                    myLCD.goto_pos(0,1);
                    printChannelNumbers(channels[currentBank]);
                }
            }

            buttonFLast = buttonFState;
            buttonULast = buttonUState;
            buttonDLast = buttonDState;
        }
    }
	return 0;
};
