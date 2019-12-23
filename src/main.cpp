#include <Arduino.h>
#include "PWMfrequency.h"
#include "ADKbreakoutboard.h"
#include "PS4BT.h"
#include "MPU9250.h"
#include "OmniKinematics3WD.h"
#include <avr/wdt.h>
#include "OmniManualFixedBaseVector.h"

struct parameter
{
  const int MaxPWM = 245;
  const double RCfilterIntensity = 0.5;
  const double pwmMultiplyIncreaseRate = 0.05;
  const double solenoidValueOpenTime = 250; //in ms
  double pwmMultiply = 1.0;
} RobotParam;

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);
MPU9250 IMU(ADKbreakoutboard::SerialBaud.I2C);
OmniKinematics3WD reverseKinematics(RobotParam.MaxPWM);
bool getButtonClickOnce(ButtonEnum);
void watchDogReset(uint8_t);
bool dengerKeybinds();
void manualMotorRateFixes(int *, int *);
void pwmApplyForDriveTesting(int *); //実験用基板用仮関数

void setup()
{
  ADKbreakoutboard::initializeOnBoardLEDs();
  setPwmFrequencyMEGA2560(5, 1); //setup pwm frequency to 30kHz
  setPwmFrequencyMEGA2560(6, 1);
  setPwmFrequencyMEGA2560(7, 1);
  setPwmFrequencyMEGA2560(8, 1);
  setPwmFrequencyMEGA2560(9, 1);
  setPwmFrequencyMEGA2560(10, 1);
  Serial.begin(ADKbreakoutboard::SerialBaud.HardwareSerial);
  Serial1.begin(ADKbreakoutboard::SerialBaud.HardwareSerial);
  Serial2.begin(ADKbreakoutboard::SerialBaud.HardwareSerial);
  Serial3.begin(ADKbreakoutboard::SerialBaud.HardwareSerial);
  while (!Serial)
    ;                   // waiting for opening hardware Serial port
  if (Usb.Init() == -1) // initialize USB device
  {
    Serial.println(F("\nArduino hasn't attached USB_HOST_SHIELD.\n"));
    while (1)
      ;
  }
  Serial.println(F("\nUSB_HOST_SHIELD detected, Success opening Serial port.\n"));
  IMU.addStatsLED(ADKbreakoutboard::onBoardLEDs::No5);
  IMU.calibration(); // initialize 9-DOF IMU sensor and calclating bias
  Serial.println(F("IMU calibrated successfully."));
}

void loop()
{
  for (int i = 0; i < 100; i++)
  {
    Usb.Task(); // running USB tasks
  }
  ADKbreakoutboard::updateOnBoardLEDs();
  if (!PS4.connected()) //未接続の場合以下の処理を弾く
    return;
  if (dengerKeybinds()) //PSボタンが押されたらすべての以下の処理を弾く
    return;

  int stickVector[3], reverseKinematicsOutput[3], fixedMotorAppliedPWM[3];
  if (PS4.getButtonPress(SHARE) && getButtonClickOnce(R1))
    RobotParam.pwmMultiply += RobotParam.pwmMultiplyIncreaseRate;
  else if (PS4.getButtonPress(SHARE) && getButtonClickOnce(L1))
    RobotParam.pwmMultiply -= RobotParam.pwmMultiplyIncreaseRate;
  RobotParam.pwmMultiply = (RobotParam.pwmMultiply > 2.0) ? 2.0 : (RobotParam.pwmMultiply < 0) ? 0 : RobotParam.pwmMultiply;

  stickVector[0] = (PS4.getAnalogHat(LeftHatX) - 127) * RobotParam.pwmMultiply;
  if (-3 < stickVector[0] && stickVector[0] < 3)
    stickVector[0] = 0;
  stickVector[1] = (PS4.getAnalogHat(LeftHatY) - 127) * RobotParam.pwmMultiply;
  if (-3 < stickVector[1] && stickVector[1] < 3)
    stickVector[1] = 0;
  stickVector[2] = (PS4.getAnalogButton(R2) - PS4.getAnalogButton(L2)) * 0.07;

  reverseKinematics.getOutput(stickVector[0], stickVector[1], -stickVector[2], -IMU.getYaw(), reverseKinematicsOutput);
  manualMotorRateFixes(reverseKinematicsOutput, fixedMotorAppliedPWM);
  pwmApplyForDriveTesting(fixedMotorAppliedPWM);
  // Serial.print(reverseKinematicsOutput[0]);
  // Serial.print('\t');
  // Serial.print(reverseKinematicsOutput[1]);
  // Serial.print('\t');
  // Serial.print(reverseKinematicsOutput[2]);
  // Serial.print('\t');
  // Serial.print('\t');
  // Serial.print(fixedMotorAppliedPWM[0]);
  // Serial.print('\t');
  // Serial.print(fixedMotorAppliedPWM[1]);
  // Serial.print('\t');
  // Serial.print(fixedMotorAppliedPWM[2]);
  // Serial.print('\t');
  // Serial.print(IMU.getYaw());
  // Serial.print('\t');
  // static double prevMicros;
  // Serial.print(1.0 / (double(micros() - prevMicros) / 1000.0 / 1000.0));
  // Serial.print("Hz (LOOP RATE)");
  // prevMicros = micros();
  // Serial.print(getButtonClickOnce(CIRCLE));
  // Serial.print("\r\n");
}

void manualMotorRateFixes(int *kinematicsOut, int *calibratedOut)
{
  for (int i = 0; i < 3; i++)
  {
    if (kinematicsOut[i] > 0)
    {
      calibratedOut[i] = int(double(kinematicsOut[i]) * double(vectorFix::omniBaseVector[0][i]));
    }
    else
    {
      calibratedOut[i] = int(double(kinematicsOut[i]) * double(vectorFix::omniBaseVector[1][i]));
    }
  }
}

void pwmApplyForDriveTesting(int *motorPWMs)
{
  if (motorPWMs[0] > 0)
  {
    analogWrite(5, motorPWMs[0]);
    analogWrite(6, 0);
  }
  else
  {
    analogWrite(5, 0);
    analogWrite(6, -motorPWMs[0]);
  }
  if (motorPWMs[1] > 0)
  {
    analogWrite(7, motorPWMs[1]);
    analogWrite(8, 0);
  }
  else
  {
    analogWrite(7, 0);
    analogWrite(8, -motorPWMs[1]);
  }
  if (motorPWMs[2] > 0)
  {
    analogWrite(9, motorPWMs[2]);
    analogWrite(10, 0);
  }
  else
  {
    analogWrite(9, 0);
    analogWrite(10, -motorPWMs[2]);
  }
}

void ADKbreakoutboard::updateOnBoardLEDs()
{
  return;
}

void watchDogReset(uint8_t prescaller)
{
  wdt_enable(prescaller);
  while (1)
  {
  }
}

bool dengerKeybinds()
{
  if (!PS4.getButtonPress(PS))
  {
    return 0;
  }
  if (PS4.getButtonPress(SHARE) && PS4.getButtonPress(OPTIONS))
  {
    PS4.disconnect();
    PS4.setLed(0xF2, 0x46, 0x07);
    PS4.setRumbleOn(RumbleHigh);
    for (int i = 0; i < 200; i++)
    {
      Usb.Task(); // running USB tasks
      _delay_ms(1);
    }
    watchDogReset(WDTO_15MS);
    return 0;
  }
  if (PS4.getButtonPress(OPTIONS))
  {
    PS4.disconnect();
    PS4.setRumbleOn(RumbleHigh);
    return 0;
  }
  return 1;
}

bool getButtonClickOnce(ButtonEnum b)
{
  static bool previousButtonStats[16];
  bool currentStats = PS4.getButtonPress(b);
  if (!currentStats)
  {
    previousButtonStats[b] = currentStats;
    return 0;
  }
  if (currentStats != previousButtonStats[b])
  {
    previousButtonStats[b] = currentStats;
    return 1;
  }
  return 0;
}