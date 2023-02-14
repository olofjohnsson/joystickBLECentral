/*
  This example scans for Bluetooth® Low Energy peripherals until one with the advertised service
  "76ad7aaa-3782-11ed-a261-0242ac120002" UUID is found. Once discovered and connected it will listen to updates in the 
  x_reading characteristic 76ad7aa1-3782-11ed-a261-0242ac120002 and y_reading characteristic 76ad7aa2-3782-11ed-a261-0242ac120002
*/

#define PRO_BOAT_CONTROL
// #define PRO_SERVO_CONTROL
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Servo.h>
#include "mbed.h"
union ArrayToInteger 
{
  byte array[4];
  uint32_t integer;
};

#define SERVICE_UUID "76ad7aaa-3782-11ed-a261-0242ac120002"
#define PWM_PIN               12
#define PWM_FREQUENCY         10000

mbed::PwmOut pwmPin(digitalPinToPinName(PWM_PIN));

int mainMotorPWM_PIN = 3;
int turnCW_PIN = 4;
int turnCCW_PIN = 5;
int runFwd_PIN = 6;
int runBwd_PIN = 7;
int led_PIN = 2;
#ifdef PRO_SERVO_CONTROL
int servo_PIN = 9;
int servo_x_val = 90;
#endif

Servo myServo;

void read_x_y_values(BLEDevice peripheral);
int byteArrayToInt(const byte data[], int length);
void setDutyCycle(int duty);

void setup() {
  BLE.setConnectionInterval(0x0006, 0x0006);
  Serial.begin(9600);
  //while (!Serial);
  pinMode(mainMotorPWM_PIN, OUTPUT);
  pinMode(runFwd_PIN, OUTPUT);
  pinMode(runBwd_PIN, OUTPUT);
  pinMode(turnCW_PIN, OUTPUT);
  pinMode(turnCCW_PIN, OUTPUT);
  pinMode(led_PIN, OUTPUT);
  digitalWrite(led_PIN, HIGH);
  digitalWrite(runFwd_PIN, LOW);
  digitalWrite(runBwd_PIN, LOW);
  digitalWrite(turnCW_PIN, LOW);
  digitalWrite(turnCCW_PIN, LOW);
  pwmPin.write(0);
  #ifdef PRO_SERVO_CONTROL
  myServo.attach(servo_PIN);
  #endif
  pwmPin.period( 1.0 / PWM_FREQUENCY );
  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central. Joystick central with motor electronics");
  Serial.print("Scanning for ");
  Serial.print(SERVICE_UUID);

  // start scanning for peripherals
  BLE.scanForUuid(SERVICE_UUID);
}

void setDutyCycle(int duty)
{
  pwmPin.write( duty / 100.0 );
  Serial.print("dutyCycle is: ");
  Serial.println(duty);
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    digitalWrite(led_PIN, LOW);
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "JoyStickReading") {
      return;
    }

    // stop scanning
    BLE.stopScan();

    read_x_y_values(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("76ad7aaa-3782-11ed-a261-0242ac120002");
  }
}

void read_x_y_values(BLEDevice peripheral)
{
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve characteristic
  BLECharacteristic x_readingChar = peripheral.characteristic("76ad7aa1-3782-11ed-a261-0242ac120002");
  BLECharacteristic y_readingChar = peripheral.characteristic("76ad7aa2-3782-11ed-a261-0242ac120002");
  BLECharacteristic x_rawReadingChar = peripheral.characteristic("76ad7ab1-3782-11ed-a261-0242ac120002");
  BLECharacteristic y_rawReadingChar = peripheral.characteristic("76ad7ab2-3782-11ed-a261-0242ac120002");
  BLECharacteristic turningDirectionChar = peripheral.characteristic("76ad7aa6-3782-11ed-a261-0242ac120002");
  BLECharacteristic runningDirectionChar = peripheral.characteristic("76ad7aa7-3782-11ed-a261-0242ac120002");

  BLECharacteristic button2Char = peripheral.characteristic("76ad7aa3-3782-11ed-a261-0242ac120002");
  BLECharacteristic button3Char = peripheral.characteristic("76ad7aa4-3782-11ed-a261-0242ac120002");
  BLECharacteristic button4Char = peripheral.characteristic("76ad7aa5-3782-11ed-a261-0242ac120002");

  if (!x_readingChar) {
    Serial.println("Peripheral does not have x_readingChar!");
    peripheral.disconnect();
    return;
  }
  if (!y_readingChar) {
    Serial.println("Peripheral does not have y_readingChar!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) 
  {
    turningDirectionChar.read();
    if(byteArrayToInt(turningDirectionChar.value(), turningDirectionChar.valueLength()) == 1)
    {
      digitalWrite(turnCW_PIN, HIGH);
      digitalWrite(turnCCW_PIN, LOW);
    }
    else
    {
      digitalWrite(turnCW_PIN, LOW);
      digitalWrite(turnCCW_PIN, HIGH);
    }
    runningDirectionChar.read();
    if(byteArrayToInt(runningDirectionChar.value(), runningDirectionChar.valueLength()) == 1)
    {
      digitalWrite(runFwd_PIN, HIGH);
      digitalWrite(runBwd_PIN, LOW);
    }
    else
    {
      digitalWrite(runFwd_PIN, LOW);
      digitalWrite(runBwd_PIN, HIGH);
    }
    
    x_readingChar.read();
    y_readingChar.read();
    turningDirectionChar.read();
    runningDirectionChar.read();

    setDutyCycle(byteArrayToInt(x_readingChar.value(), x_readingChar.valueLength()));
    #ifdef PRO_SERVO_CONTROL
    myServo.write(map(byteArrayToInt(x_readingChar.value(), x_readingChar.valueLength()), 0, 100, 0, 180));
    #endif
  }
  Serial.println("Peripheral disconnected");
  digitalWrite(led_PIN, HIGH);
  setDutyCycle(0);
}

int byteArrayToInt(const byte data[], int length)
{
  byte dataW[length];
  for (int i = 0;i < length; i++)
  {
    byte b = data[i];
    dataW[i] = data[i];
  }
  ArrayToInteger converter;
  converter.array[0] = dataW[0];
  converter.array[1] = dataW[1];
  converter.array[2] = dataW[2];
  converter.array[3] = dataW[3];
  return converter.integer;
}
