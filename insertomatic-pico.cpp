#include "pico/stdlib.h"

#include "lcd_display.hpp"

// ChatGPT generated section

#include "hardware/gpio.h"

#define I2C_SCL_PIN 16 // Define SCL pin
#define I2C_SDA_PIN 15 // Define SDA pin
#define I2C_ADDRESS (0xCA >> 1) // Define I2C address
#define I2C_DELAY 5 // Delay in microseconds for timing

void i2c_delay() {
    sleep_us(I2C_DELAY);
}

void i2c_start() {
    gpio_put(I2C_SDA_PIN, 1);
    gpio_put(I2C_SCL_PIN, 1);
    i2c_delay();
    gpio_put(I2C_SDA_PIN, 0);
    i2c_delay();
    gpio_put(I2C_SCL_PIN, 0);
}

void i2c_stop() {
    gpio_put(I2C_SCL_PIN, 0);
    gpio_put(I2C_SDA_PIN, 0);
    i2c_delay();
    gpio_put(I2C_SCL_PIN, 1);
    i2c_delay();
    gpio_put(I2C_SDA_PIN, 1);
    i2c_delay();
}

void i2c_write_bit(bool bit) {
    gpio_put(I2C_SDA_PIN, bit);
    i2c_delay();
    gpio_put(I2C_SCL_PIN, 1);
    i2c_delay();
    gpio_put(I2C_SCL_PIN, 0);
    i2c_delay();
}

bool i2c_read_ack() {
    gpio_set_dir(I2C_SDA_PIN, GPIO_IN);
    i2c_delay();
    gpio_put(I2C_SCL_PIN, 1);
    i2c_delay();
    bool ack = !gpio_get(I2C_SDA_PIN);
    gpio_put(I2C_SCL_PIN, 0);
    gpio_set_dir(I2C_SDA_PIN, GPIO_OUT);
    return ack;
}

bool i2c_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        i2c_write_bit(byte & 0x80);
        byte <<= 1;
    }
    return i2c_read_ack();
}

void i2c_bitbang_write(uint8_t address, uint8_t *data, size_t length) {
    i2c_start();
    if (!i2c_write_byte(address << 1)) { // Send address with write bit
        i2c_stop();
        return;
    }
    for (size_t i = 0; i < length; i++) {
        if (!i2c_write_byte(data[i])) {
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
		sleep_ms(2500);

		gpio_init(I2C_SCL_PIN);
		gpio_set_dir(I2C_SCL_PIN, GPIO_OUT);
		gpio_put(I2C_SCL_PIN, 1);

		gpio_init(I2C_SDA_PIN);
		gpio_set_dir(I2C_SDA_PIN, GPIO_OUT);
		gpio_put(I2C_SDA_PIN, 1);

		uint8_t data[4] = {0x80, 0x10, 0x5E, 0x74}; // Example data

		while (true) {
			i2c_bitbang_write(I2C_ADDRESS, data, 4);
			sleep_ms(1000); // Write every second
		}
	};
	return 0;
};
