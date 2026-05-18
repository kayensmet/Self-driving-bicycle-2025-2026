#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

//doit devkit esp32 (niet bluepad esp32)
//pin 35 en 34 geven error (kan niet als output pin gebruikt worden enkel input)
//controlleer wifi kanaal

// ================= DAC =================
const int throttlePin = 25;
const int brakePin = 26;

const int controllerPin = 32; //voorlicht 
const int brakeLight = 33; //remlicht 
const int buzzer = 27;

const int WIFI_CHANNEL = 11; //eerst kijken welke wifi channel dat de zender selecteert en dan dat invullen hier

struct SensorData {
  int   counter;
  float l2Volt;
  float r2Volt;
  bool  r1Pressed;
  bool  controllerConnected;
};

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(SensorData)) {
    SensorData d;
    memcpy(&d, data, sizeof(d));

    Serial.print("ESP-NOW DEBUG | R1=");
    Serial.print(d.r1Pressed ? "TRUE" : "FALSE");

    Serial.print(" | Controller=");
    Serial.println(d.controllerConnected ? "CONNECTED" : "DISCONNECTED");

    // converteer spanning 0-3.3V naar 0-255 voor DAC
    int throttleDAC = (int)(d.r2Volt / 3.3 * 255);
    int brakeDAC    = (int)(d.l2Volt / 3.3 * 255);

    dacWrite(throttlePin, throttleDAC);
    dacWrite(brakePin, brakeDAC);

    //Achterlicht (L2)
    if (d.l2Volt > 0.1) {           // drempel om “ingedrukt” te detecteren
        digitalWrite(brakeLight, HIGH);
    } else {
        digitalWrite(brakeLight, LOW);
    }

    //Voorlicht (R2)
    if (d.r2Volt > 0.1) {
        digitalWrite(controllerPin, HIGH);
    } else {
        digitalWrite(controllerPin, LOW);
    }

    Serial.print("counter="); Serial.print(d.counter);
    Serial.print("  throttleDAC="); Serial.print(throttleDAC);
    Serial.print("  brakeDAC="); Serial.println(brakeDAC);


  if (d.r1Pressed) {
    digitalWrite(buzzer, HIGH);
} else {
    digitalWrite(buzzer, LOW);
}

/*
if (d.controllerConnected) {
    // Optioneel: toon status
    Serial.println("Controller verbonden");
}
*/

  } else {
    Serial.print("Onbekend pakket, len="); Serial.println(len);
  }

}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);           // ESP-NOW werkt in station mode

    pinMode(controllerPin, OUTPUT); // voorlicht
    pinMode(brakeLight, OUTPUT);    // achterlicht
    pinMode(buzzer, OUTPUT);    // buzzer

    //kannaal afhankelijk van de zender... test eerst wat het kanaal is van de zender
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);  // aanpassen indien nodig


    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init fout");
        return;
    }

    esp_now_register_recv_cb(onReceive); // registreer callback
}

void loop() {
  
    // niks nodig, callback doet alles
}