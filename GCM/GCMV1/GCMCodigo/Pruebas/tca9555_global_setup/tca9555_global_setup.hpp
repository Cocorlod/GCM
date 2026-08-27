#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9

#define TCA9555_ADDR 0x20

#define INPUT_PORT_0   0x00
#define INPUT_PORT_1   0x01
#define OUTPUT_PORT_0  0x02
#define OUTPUT_PORT_1  0x03
#define POLARITY_0     0x04
#define POLARITY_1     0x05
#define CONFIG_0       0x06
#define CONFIG_1       0x07

void tcaWriteRegister (uint8_t reg, uint8_t value);
uint8_t tcaReadRegister(uint8_t reg);