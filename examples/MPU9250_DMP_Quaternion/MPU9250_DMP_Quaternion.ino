/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>

#define SerialPort Serial

MPU9250_DMP imu;

int FIFOReady = false;
void MPUInt(void)
{
  FIFOReady = true;
}

void setup() 
{
  attachInterrupt(digitalPinToInterrupt(2), MPUInt, RISING);
  SerialPort.begin(9600);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

  interrupts();
}

#define ADDR (0x68)
void I2CW(char* s, int c)
{
    Wire.beginTransmission(ADDR);
    Wire.write(s, c);
    Wire.endTransmission(true);
}

static char str[6*6];
static int strl = 0;
#define Hex2Dec(c) ((c)>'9'?(c)+10-'A':(c)-'0')
#define SZ (16)
static char buf[2*SZ+3];
void loop() 
{
  // Check for new data in the FIFO
  if ( FIFOReady)
  {
    FIFOReady = false;
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      //printIMUData();
    }
  }
  else
  {
    while (Serial.available())
    {
      str[strl] = Serial.read();
      strl++;
      if (str[strl-1]==0x0A)
      {
        // "?AALLxx..."
        int add = (Hex2Dec(str[1])<<4) | Hex2Dec(str[2]);
        int len = ((Hex2Dec(str[3])<<4) | Hex2Dec(str[4])) & 0x7F;
        int req = 5+((str[0]=='w')?2*len:0);
        if (strl>req)
        {
          sprintf (buf, "%c %d %02X %02X", str[0], strl, add, len);
          Serial.println(buf);
          switch (str[0])
          {
            case 'r':
              Wire.beginTransmission(ADDR);
              Wire.write(add);
              Wire.endTransmission(false);
              (void) Wire.requestFrom(ADDR, len, true);
              for (int i=0; i<len; i++)
              {
                int v = Wire.read();
                sprintf (buf+2*i, "%02X", v);
              }
              buf[2*len] = 0;
              Serial.println(buf);
              break;
            case 'w':
              buf[0] = add;
              for (int i=0; i<len; i++)
              {
                buf[1+i] = (Hex2Dec(str[2*i+5])<<4) | Hex2Dec(str[2*i+6]);
              }
              I2CW(buf, len+1);
              break;
          }
        }
        else
        {
          Serial.println ("msg too short");
        }
        strl=0;
      }
    }
  }
}

void printIMUData(void)
{  
  //SerialPort.print("\r" + String(imu.roll) + "\t"
  SerialPort.println("R:" + String(imu.roll) + "\tP:"
            + String(imu.pitch) + "\tY:" + String(imu.yaw) + 
            "\tt:" + String(imu.time));
}


/*
r 6 75 01
71
00: BFCBE51D A8392680 01E10406 006A6372  E007
10: BEB2C900 00000000 00040218 00480000  FFF8
20: 00000000 408C0288 0C0A8100 00000000  FFF8
30: 00000000 13000080 020107F8 E0FF103F  FDFF
40: 80FFC500 070011FF FE014C01 34006802  FFFF
50: 10000000 00000000 00000000 00000000  FFFF
60: 00000100 11000003 0000E001 000ACEFF  1FF9
70: 04000000 0071000D 54001C02 0025B200  6DBC
     0 1 2 3  4 5 6 7  8 9 A B  C D E F
*/

/*
w6D020000 r6F10
w6D020010 r6F10
w6D020020 r6F10
w6D020030 r6F10
w6D020040 r6F10
w6D020050 r6F10
w6D020060 r6F10
w6D020070 r6F10
w6D020080 r6F10
w6D020090 r6F10
w6D0200A0 r6F10
w6D0200B0 r6F10
w6D0200C0 r6F10
w6D0200D0 r6F10
w6D0200E0 r6F10
w6D0200F0 r6F10

w6D020100 r6F10
w6D020110 r6F10
w6D020120 r6F10
w6D020130 r6F10
w6D020140 r6F10
w6D020150 r6F10
w6D020160 r6F10
w6D020170 r6F10
w6D020180 r6F10
w6D020190 r6F10
w6D0201A0 r6F10
w6D0201B0 r6F10
w6D0201C0 r6F10
w6D0201D0 r6F10
w6D0201E0 r6F10
w6D0201F0 r6F10

w6D020200 r6F10
w6D020210 r6F10
w6D020220 r6F10
w6D020230 r6F10
w6D020240 r6F10
w6D020250 r6F10
w6D020260 r6F10
w6D020270 r6F10
w6D020280 r6F10
w6D020290 r6F10
w6D0202A0 r6F10
w6D0202B0 r6F10
w6D0202C0 r6F10
w6D0202D0 r6F10
w6D0202E0 r6F10
w6D0202F0 r6F10

w6D020300 r6F10
w6D020310 r6F10
w6D020320 r6F10
w6D020330 r6F10
w6D020340 r6F10
w6D020350 r6F10
w6D020360 r6F10
w6D020370 r6F10
w6D020380 r6F10
w6D020390 r6F10
w6D0203A0 r6F10
w6D0203B0 r6F10
w6D0203C0 r6F10
w6D0203D0 r6F10
w6D0203E0 r6F10
w6D0203F0 r6F10

w6D020400 r6F10
w6D020410 r6F10
w6D020420 r6F10
w6D020430 r6F10
w6D020440 r6F10
w6D020450 r6F10
w6D020460 r6F10
w6D020470 r6F10
w6D020480 r6F10
w6D020490 r6F10
w6D0204A0 r6F10
w6D0204B0 r6F10
w6D0204C0 r6F10
w6D0204D0 r6F10
w6D0204E0 r6F10
w6D0204F0 r6F10

w6D020500 r6F10
w6D020510 r6F10
w6D020520 r6F10
w6D020530 r6F10
w6D020540 r6F10
w6D020550 r6F10
w6D020560 r6F10
w6D020570 r6F10
w6D020580 r6F10
w6D020590 r6F10
w6D0205A0 r6F10
w6D0205B0 r6F10
w6D0205C0 r6F10
w6D0205D0 r6F10
w6D0205E0 r6F10
w6D0205F0 r6F10

w6D020600 r6F10
w6D020610 r6F10
w6D020620 r6F10
w6D020630 r6F10
w6D020640 r6F10
w6D020650 r6F10
w6D020660 r6F10
w6D020670 r6F10
w6D020680 r6F10
w6D020690 r6F10
w6D0206A0 r6F10
w6D0206B0 r6F10
w6D0206C0 r6F10
w6D0206D0 r6F10
w6D0206E0 r6F10
w6D0206F0 r6F10

w6D020700 r6F10
w6D020710 r6F10
w6D020720 r6F10
w6D020730 r6F10
w6D020740 r6F10
w6D020750 r6F10
w6D020760 r6F10
w6D020770 r6F10
w6D020780 r6F10
w6D020790 r6F10
w6D0207A0 r6F10
w6D0207B0 r6F10
w6D0207C0 r6F10
w6D0207D0 r6F10
w6D0207E0 r6F10
w6D0207F0 r6F10

w6D020800 r6F10
w6D020810 r6F10
w6D020820 r6F10
w6D020830 r6F10
w6D020840 r6F10
w6D020850 r6F10
w6D020860 r6F10
w6D020870 r6F10
w6D020880 r6F10
w6D020890 r6F10
w6D0208A0 r6F10
w6D0208B0 r6F10
w6D0208C0 r6F10
w6D0208D0 r6F10
w6D0208E0 r6F10
w6D0208F0 r6F10

w6D020900 r6F10
w6D020910 r6F10
w6D020920 r6F10
w6D020930 r6F10
w6D020940 r6F10
w6D020950 r6F10
w6D020960 r6F10
w6D020970 r6F10
w6D020980 r6F10
w6D020990 r6F10
w6D0209A0 r6F10
w6D0209B0 r6F10
w6D0209C0 r6F10
w6D0209D0 r6F10
w6D0209E0 r6F10
w6D0209F0 r6F10

w6D020A00 r6F10
w6D020A10 r6F10
w6D020A20 r6F10
w6D020A30 r6F10
w6D020A40 r6F10
w6D020A50 r6F10
w6D020A60 r6F10
w6D020A70 r6F10
w6D020A80 r6F10
w6D020A90 r6F10
w6D020AA0 r6F10
w6D020AB0 r6F10
w6D020AC0 r6F10
w6D020AD0 r6F10
w6D020AE0 r6F10
w6D020AF0 r6F10

w6D020B00 r6F10
w6D020B10 r6F10
w6D020B20 r6F10
w6D020B30 r6F10
w6D020B40 r6F10
w6D020B50 r6F10
w6D020B60 r6F10
w6D020B70 r6F10
w6D020B80 r6F10
w6D020B90 r6F10
w6D020BA0 r6F10
w6D020BB0 r6F10
w6D020BC0 r6F10
w6D020BD0 r6F10
w6D020BE0 r6F10
w6D020BF0 r6F10
*/