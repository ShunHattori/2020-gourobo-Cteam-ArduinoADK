#pragma once

namespace ADKbreakoutboard
{

typedef enum statsLEDs
{
    No1 = 22,
    No2,
    No3,
    No4,
    No5,
} onBoardLEDs;

struct
{
    const long HardwareSerial = 256000; //256kHz
    const int SoftwareSerial = 38400;
    const long I2C = 400000; //400kHz
} SerialBaud;


void initializeOnBoardLEDs()
{
  pinMode(onBoardLEDs::No1, OUTPUT);
  pinMode(onBoardLEDs::No2, OUTPUT);
  pinMode(onBoardLEDs::No3, OUTPUT);
  pinMode(onBoardLEDs::No4, OUTPUT); // pinmode setup(PS4 Dual Shock Controller connection stats LED)
}

extern void updateOnBoardLEDs();

} // namespace ADKbreakoutboard