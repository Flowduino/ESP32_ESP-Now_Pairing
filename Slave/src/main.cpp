#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define PIN_BUTTON    34 // GPIO34
#define PIN_LED_RED   23 // GPIO23
#define PIN_LED_BLUE  22 // GPIO22

#define UUID_SERVICE          "d91fdc86-46f8-478f-8dec-ebdc0a1188b2"
#define UUID_CHARACTERISTIC   "56100987-749a-4014-bc22-0be2f5af59d0"

BLEScan* pBLEScan;
BLEClient* pBLEClient;
BLERemoteService* pRemoteService;
BLERemoteCharacteristic* pRemoteCharacteristic;
BLEAdvertisedDevice _advertisedDevice;
bool deviceFound = false;
bool clientConnected = false;
unsigned long pairedAt;

// We use an Enum to define the Mode of our Device
enum DeviceMode {
  Waiting, // Not pairing, not timed out
  Pairing, // We're in Pairing mode
  Paired,  // Pairing Succeeded
  Failed,  // Pairing Failed (Timed Out)
};

DeviceMode deviceMode = Waiting; // We are initially Waiting

enum ButtonState {
  ButtonDown, // The button is being pressed/held
  ButtonUp    // The button has been released
};

ButtonState buttonState;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if (clientConnected) { return; };
      
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().toString() == UUID_SERVICE) {
        Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
        _advertisedDevice = advertisedDevice;
        deviceFound = true;
        return;
      }
    }
};

inline ButtonState getButtonState() {
  return digitalRead(PIN_BUTTON) == HIGH ? ButtonDown : ButtonUp;
}

inline void startPairing() {
  // This method will switch on BLE and set it up to broadcast the WiFi Mac Address
}

inline void stopPairing() {
  // This method will switch off BLE.
}

unsigned long nextFlash; // This will hold the millis() value of our next Flash
#define INTERVAL_FLASH  500 // This is our Flash Interval (500ms, 0.5 seconds)

inline void flashBlueLED() {
  if (millis() < nextFlash) { return; } // It isn't time yet, so nothing to do.

  digitalWrite(PIN_LED_BLUE, !digitalRead(PIN_LED_BLUE)); // Toggle the LED State

  nextFlash = millis() + INTERVAL_FLASH; // Sets the time it should toggle again.
}

inline void setRedLED(bool ledOn) {
  digitalWrite(PIN_LED_RED, ledOn); // True = HIGH, False = LOW
}

void setup() {
  // Initialise Serial first
  Serial.begin(115200); // Set Serial Monitor to 115200 baud

  BLEDevice::init("Flowduino Auto-Discovery Demo - Slave");

  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value

  // Set our Pin Modes
  pinMode(PIN_BUTTON, INPUT);     // Button Input
  pinMode(PIN_LED_RED, OUTPUT);   // Red LED Output
  pinMode(PIN_LED_BLUE, OUTPUT);  // Blue LED Output

  // Get the initial state of our Button
  buttonState = getButtonState();
}

unsigned long buttonHoldStart; // The millis() value of the initial Button push down
#define BUTTON_HOLD_TIME  3000 // The number of millis for which we must hold the button
unsigned long pairingStart; // The millis() value at which Pairing started
#define PAIRING_TIMEOUT   30000 // 30 seconds in milliseconds for Timeout

// The Loop routine when our Device is in Waiting Mode
inline void loopWaiting() {
  ButtonState currentState = getButtonState();

  // Down to Up
  if (buttonState == ButtonDown && currentState == ButtonUp) {
    buttonState = currentState; // Update the global variable accordingly
    return; // Need not proceed further
  }

  // Up to Down
  if (buttonState == ButtonUp && currentState == ButtonDown) {
    // The Button was just pressed down...
    buttonHoldStart = millis();
    buttonState = currentState;
    Serial.println("Button Hold Started");
    return; // Need not proceed further
  }

  // Held Down
  if (buttonState == ButtonDown && currentState == ButtonDown && millis() > buttonHoldStart + BUTTON_HOLD_TIME) {
    // We now initiate Pairing!
    Serial.println("Initiating Pairing");
    deviceMode = Pairing;
    setRedLED(false);
    pairingStart = millis();
    buttonHoldStart = pairingStart;
    startPairing();
  }
}

inline bool connectToDevice() {
  pBLEClient = BLEDevice::createClient();
  Serial.println("Connecting To \"Flowduino Auto-Discovery Demo - Master\"");
  clientConnected = pBLEClient->connect(&_advertisedDevice);
  
  if (!clientConnected) {
    Serial.println("Connection Failed!");
    return false;
  }
    
  Serial.println("Connected... let's get the ESPNow Address!");
  pRemoteService = pBLEClient->getService(UUID_SERVICE);
  if (pRemoteService == nullptr) {
    Serial.println("Unable to get our Service from the Controller!");
    return false;
  }

  Serial.println("Service acquired!");

  pRemoteCharacteristic = pRemoteService->getCharacteristic(UUID_CHARACTERISTIC);
  
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Couldn't get the pRemoteCharacteristic");
    return false;
  }

  Serial.println("YAY! We got the pRemoteCharacteristic!");

  uint8_t mac[6];
  char macStr[18] = { 0 };
  const char* rawData = pRemoteCharacteristic->readValue().c_str();//pRemoteCharacteristic->readRawData();
  
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", rawData[0], rawData[1], rawData[2], rawData[3], rawData[4], rawData[5]);
  
  Serial.print("Value is: ");
  Serial.println(String(macStr));

  deviceMode = Paired;
  pairedAt = millis();

  return true;
}

// The Loop routine when our Device is in Pairing Mode
inline void loopPairing() {
  flashBlueLED();

  // Scan for BLE Devices
  deviceFound = false;
  BLEScanResults foundDevices = pBLEScan->start(3, false);
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory

  if (deviceFound) {
    if (connectToDevice()) { return; }
  }

  ButtonState currentState = getButtonState();

  // Down to Up
  if (buttonState == ButtonDown && currentState == ButtonUp) {
    buttonState = currentState; // Update the global variable accordingly
    return; // Need not proceed further
  }

  // Up to Down
  if (buttonState == ButtonUp && currentState == ButtonDown) {
    // The Button was just pressed down...
    buttonHoldStart = millis();
    buttonState = currentState;
    Serial.println("Button Hold Started");
    return; // Need not proceed further
  }

  // Held Down OR Timed Out
  if (
       (buttonState == ButtonDown && currentState == ButtonDown && millis() > buttonHoldStart + BUTTON_HOLD_TIME) ||
       (millis() > pairingStart + PAIRING_TIMEOUT)
     ){
    // We now initiate Pairing!
    Serial.println("Cancelling Pairing");
    deviceMode = Waiting;
    setRedLED(true);
    digitalWrite(PIN_LED_BLUE, LOW); // Ensure Blue LED is OFF
    buttonHoldStart = millis();
    stopPairing();
  }
}

// The Loop routine when our Device is in Paired Mode
inline void loopPaired() {
  if (pairedAt == 0) {
    stopPairing();
    pairedAt = millis();
    return;
  }
  
  if (millis() > pairedAt + BUTTON_HOLD_TIME) {
    digitalWrite(PIN_LED_BLUE, LOW);
    deviceMode = Waiting;
    Serial.println("Going back to Waiting mode");
  }
}

void loop() {
  switch (deviceMode) {
    case (Waiting):
      loopWaiting();
      break;
    case (Pairing):
      loopPairing();
      break;
    case (Paired):
      loopPaired();
      break;
  }
}