// Thanks to Antonin RAFFIN https://medium.com/@araffin/simple-and-robust-computer-arduino-serial-communication-f91b95596788
// code copied from https://github.com/araffin/cpp-arduino-serial/tree/2075fd8f203483af9c82caffba4bdbc84024ea63

#ifndef _INCL_GUARD
#define _INCL_GUARD

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include "BluetoothSerial.h"
#include <functional>

void update_motors_orders();

enum Command
{
    HANDSHAKE = 101,
    ALREADY_CONNECTED = 105,
    COMMAND_ERROR = 108,
    RECEIVED = 100,
    BYE = 102,
    LED = 106,
    GPS = 110,
    BMP = 111,
    MAG = 112,

    MESSAGE = 70,
   
   
};

typedef enum Command command;

template <class T>
class Parser
{
private:
    uint8_t serialBufferTX[64];
    uint8_t serialBufferRX[64];
    char PREAMBLE = '$';
    T SerialPort;
    bool connected = false;
    void (*callback)(Command command, uint8_t data[64]);

public:
    Parser(void (*call)(Command command,uint8_t data[64]));
    // Parser()
  
    ~Parser();


    void begin(String name = "Hello");

    void begin(int baudrate = 9600);

    Command read_order();

    float readFloat();

    void write_float(float num);

    void wait_for_bytes(int num_bytes, unsigned long timeout);

    void read_signed_bytes(int8_t *buffer, size_t n);

    int8_t read_i8();

    int16_t read_i16();

    int32_t read_i32();

    void write_order(enum Command command);

    void write_i8(int8_t num);

    void write_i16(int16_t num);

    void write_i32(int32_t num);

    void get_messages_from_serial();

    void sendPacket(int8_t size, Command command, int8_t data);

    void sendHeader(int8_t size, Command command);

    void stop();

    void checkHandshake();

    bool isConnected();

    void checkCommand(Command command);

    void update();
};

#include "Parser.tpp"
#endif
