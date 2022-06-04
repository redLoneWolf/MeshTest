
#include "Parser.h"

template <typename T>
Parser<T>::Parser(void (*call)(Command command,uint8_t data[64])){
        callback = call;
}


// template <class T>
// Parser<T>::Parser(){
//     //    SerialPort = t;
// }
template <class T>
Parser<T>::~Parser(){
       
}



template <class T>
void Parser<T>::begin(String name)
{
    SerialPort.begin(name);
}

template <class T>
void Parser<T>::begin(int baudrate)
{
    SerialPort.begin(baudrate);
}

template <class T>
Command Parser<T>::read_order()
{
    return (Command)read_i32();
}

template <class T>
void Parser<T>::wait_for_bytes(int num_bytes, unsigned long timeout)
{
    unsigned long startTime = millis();
    // Wait for incoming bytes or exit if timeout
    while ((SerialPort.available() < num_bytes) && (millis() - startTime < timeout))
    {
    }
}

// NOTE : SerialPort.readBytes is SLOW
// this one is much faster, but has no timeout
template <class T>
void Parser<T>::read_signed_bytes(int8_t *buffer, size_t n)
{
    size_t i = 0;
    int c;
    while (i < n)
    {
        c = SerialPort.read();
        if (c < 0)
            break;
        *buffer++ = (int8_t)c; // buffer[i] = (int8_t)c;
        i++;
    }
}

template <class T>
int8_t Parser<T>::read_i8()
{
    wait_for_bytes(1, 100); // Wait for 1 byte with a timeout of 100 ms
    return (int8_t)SerialPort.read();
}

template <class T>
int16_t Parser<T>::read_i16()
{
    int8_t buffer[2];
    wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
    read_signed_bytes(buffer, 2);
    return (((int16_t)buffer[0]) & 0xff) | (((int16_t)buffer[1]) << 8 & 0xff00);
}

template <class T>
int32_t Parser<T>::read_i32()
{
    int8_t buffer[4];
    wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
    read_signed_bytes(buffer, 4);
    return (((int32_t)buffer[0]) & 0xff) | (((int32_t)buffer[1]) << 8 & 0xff00) | (((int32_t)buffer[2]) << 16 & 0xff0000) | (((int32_t)buffer[3]) << 24 & 0xff000000);
}

template <class T>
void Parser<T>::write_order(enum Command command)
{
    //   uint8_t *Order = (uint8_t *)&command;
    write_i32(command);
}

template <class T>
void Parser<T>::write_i8(int8_t num)
{
    SerialPort.write(num);
}

template <class T>
float Parser<T>::readFloat()
{
    int8_t buffer[4];
    for (int i = 0; i < 4; i++)
    {
        buffer[i] = read_i8();
    }
    float x = *(float *)&buffer;
    return x;
}

template <class T>
void Parser<T>::write_float(float num)
{
    SerialPort.write((byte *)&num, sizeof(float));
}

template <class T>
void Parser<T>::write_i16(int16_t num)
{
    int8_t buffer[2] = {(int8_t)(num & 0xff), (int8_t)(num >> 8)};
    SerialPort.write((uint8_t *)&buffer, 2 * sizeof(int8_t));
}

template <class T>
void Parser<T>::write_i32(int32_t num)
{
    int8_t buffer[4] = {(int8_t)(num & 0xff), (int8_t)(num >> 8 & 0xff), (int8_t)(num >> 16 & 0xff), (int8_t)(num >> 24 & 0xff)};
    SerialPort.write((uint8_t *)&buffer, 4 * sizeof(int8_t));
}

template <class T>
void Parser<T>::sendHeader(int8_t size, Command command)
{
    write_i8((byte)PREAMBLE); // $

    write_i8(size); // size

    write_i8((byte)command); // command
}

template <class T>
void Parser<T>::sendPacket(int8_t size, Command command, int8_t data)
{
    sendHeader(size, command);

    int t = 0;
    while (t <= size)
    {
        serialBufferRX[t] = t;
        write_i8(t);
        t++;
    }
}

template <class T>
bool Parser<T>::isConnected()
{
    return connected;
}

template <class T>
void Parser<T>::checkHandshake()
{
    uint8_t count = 0;

    if (connected)
    {
        // write_i8(ALREADY_CONNECTED);
        while (count <= 3)
        {
            serialBufferRX[count] = 125;
            count++;
        }
        sendPacket(count, ALREADY_CONNECTED, 1);

        // write_float(12.3);
    }
    else
    {
        connected = true;
        // write_i8(HANDSHAKE);

        while (count <= 3)
        {
            serialBufferRX[count] = 125;
            count++;
        }

        sendPacket(count, HANDSHAKE, 1);
        // write_float(12.2);
    }
}

template <class T>
void Parser<T>::update()
{
    if (SerialPort.available() > 0)
    {
        if ((byte)read_i8() == (byte)PREAMBLE)
        {
            int8_t size = read_i8();
            if (size > 0)
            {
                Command command = (Command)read_i8();
                checkCommand(command);
            }
        }
        else
        {
            /* code */
        }
    }
}

template <class T>
void Parser<T>::checkCommand(Command command)
{

    switch (command)
    {
    case HANDSHAKE:
        checkHandshake();
        break;

    // case LED:
    //     led();
    //     break;

    default:
        uint8_t dat[5];
        dat[0] =15;
        callback(command,dat);
        break;
    }
}