#include <Arduino.h>

#define PIN_BUTTON    34 // GPIO34
#define PIN_LED_RED   23 // GPIO23
#define PIN_LED_BLUE  22 // GPIO22

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

inline ButtonState getButtonState() {
  return digitalRead(PIN_BUTTON) == HIGH ? ButtonDown : ButtonUp;
}

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <esp_wifi.h>
#include <WiFi.h>

#define UUID_SERVICE          "d91fdc86-46f8-478f-8dec-ebdc0a1188b2"
#define UUID_CHARACTERISTIC   "56100987-749a-4014-bc22-0be2f5af59d0"

BLEServer *bleServer;
BLEService *bleService;
BLECharacteristic *bleCharacteristic;
BLEAdvertising *bleAdvertising;
bool bleClientConnected = false;
unsigned long pairedAt;

class BLECallbacks: public BLEServerCallbacks {
   void onConnect(BLEServer* pServer) {
      Serial.println("BLE Client Connected!");
      bleClientConnected = true;
      digitalWrite(PIN_LED_BLUE, HIGH); // Keep the Blue LED On!
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("BLE Client Disconnected!");
      bleClientConnected = false;
      deviceMode = Paired;
      pairedAt = 0;
    } 
};

inline void startPairing() {
  if (bleServer == nullptr) {
    Serial.println("First Time Pairing");
    // Get the MAC Address
    WiFi.mode(WIFI_MODE_STA);
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);

    // Prepare our BLE Server
    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new BLECallbacks());

    // Prepare our Service
    bleService = bleServer->createService(UUID_SERVICE);

    // A Characteristic is what we shall use to provide Clients/Slaves with our MAC Address.
    bleCharacteristic = bleService->createCharacteristic(UUID_CHARACTERISTIC, BLECharacteristic::PROPERTY_READ);

    // Provide our Characteristic with the MAC Address "Payload"
    bleCharacteristic->setValue(&mac[0], 6);
    // Make the Property visible to Clients/Slaves.
    bleCharacteristic->setBroadcastProperty(true);

    // Start the BLE Service
    bleService->start();
  
    // Advertise it!
    bleAdvertising = BLEDevice::getAdvertising();
    bleAdvertising->addServiceUUID(UUID_SERVICE);
    bleAdvertising->setScanResponse(true);
    bleAdvertising->setMinPreferred(0x06);
    bleAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    return;
  }
  // Start the BLE Service
  bleService->start();

  // Advertise it!
  bleAdvertising = BLEDevice::getAdvertising();
  BLEDevice::startAdvertising();
}

inline void stopPairing() {
  BLEDevice::stopAdvertising();
  bleService->stop();
  Serial.println("Stopped BLE");
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

  BLEDevice::init("Flowduino Auto-Discovery Demo - Master");

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

// The Loop routine when our Device is in Pairing Mode
inline void loopPairing() {
  if (bleClientConnected) { return; }
  flashBlueLED();

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