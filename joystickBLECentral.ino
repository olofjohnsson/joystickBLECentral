/*
  LED Control

  This example scans for Bluetooth® Low Energy peripherals until one with the advertised service
  "76ad7aaa-3782-11ed-a261-0242ac120002" UUID is found. Once discovered and connected it will listen to updates in the 
  x_reading characteristic 76ad7aa1-3782-11ed-a261-0242ac120002 and y_reading characteristic 76ad7aa2-3782-11ed-a261-0242ac120002
*/

#include <ArduinoBLE.h>
union ArrayToInteger 
{
  byte array[4];
  uint32_t integer;
};

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central");

  // start scanning for peripherals
  BLE.scanForUuid("76ad7aaa-3782-11ed-a261-0242ac120002");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();
  delay(300);
  Serial.print(".");
  delay(300);
  Serial.print(".");
  delay(300);
  Serial.print(".");
  delay(300);
  Serial.print(".");
  if (peripheral) {
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

void read_x_y_values(BLEDevice peripheral) {
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

  // retrieve the LED characteristic
  BLECharacteristic x_readingChar = peripheral.characteristic("76ad7aa1-3782-11ed-a261-0242ac120002");
  BLECharacteristic y_readingChar = peripheral.characteristic("76ad7aa2-3782-11ed-a261-0242ac120002");

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
  Serial.print("x_reading\ty_reading");
  while (peripheral.connected()) 
  {
    x_readingChar.read();
    Serial.print(byteArrayToInt(x_readingChar.value(), x_readingChar.valueLength()));
    y_readingChar.read();
    Serial.print("\t");
    Serial.print(byteArrayToInt(y_readingChar.value(), y_readingChar.valueLength()));
    Serial.println();
  }
  Serial.println("Peripheral disconnected");
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
