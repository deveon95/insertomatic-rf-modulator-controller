#include "pico/stdlib.h"

#include "lcd_display.hpp"

#include "hardware/gpio.h"

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
    for (int i = 0; i < 6; i++) {
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
    bool bits[6];
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 6; j++) {
            bits[j] = (bytes[j][byteno] >> (7 - i)) & 0x01;
        }
        i2c_write_bits(bits);
    }
    return i2c_read_ack();
}

bool i2c_write_byte(uint8_t byte) {
    bool bits[6];
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 6; j++) {
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

int main() {
	myLCD.init();
	while (true) {
		myLCD.clear();
		myLCD.print("Insertomatic 6000 ------");
		sleep_ms(2500);
		myLCD.goto_pos(0,0);
		myLCD.print(" P1  P2  P3  P4  P5  P6 ");
		myLCD.goto_pos(0,1);
		myLCD.print(" C21 C24 C27 C31 C34 C37");
		sleep_ms(2500);
		myLCD.goto_pos(0,0);
		myLCD.print("P1 [Station name]       ");
		myLCD.goto_pos(0,1);
		myLCD.print("[Now playing info]      ");
		//sleep_ms(2500);

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

		uint8_t data[6][4] = {
            {0x80, 0x10, 0x5D, 0x74}, // 21
            {0x80, 0x10, 0x5E, 0x74}, // 23
            {0x80, 0x10, 0x5F, 0x74}, // 25
            {0x80, 0x10, 0x60, 0x74}, // 27
            {0x80, 0x10, 0x61, 0x74}, // 29
            {0x80, 0x10, 0x62, 0x74}  // 31
        };

		while (true) {
			i2c_bitbang_write(I2C_ADDRESS, data, 4);
			sleep_ms(1000); // Write every second
		}
	};
	return 0;
};
