//SAFETY ESP

#include <esp_now.h>
#include <WiFi.h>


const uint8_t PIN_SSR_MAIN = 2;   
const uint8_t PIN_RESET_BTN = 25;


unsigned long lastOkTime = 0;
const unsigned long okTimeout = 4000; 


bool isFailsafe = false; //vergrendeling


typedef struct struct_message {
  uint8_t sender; uint8_t command; int throttle; int brake; int sturen; float value; bool buzzer; bool balansAan;
} struct_message;

struct_message incoming;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

//ini ESP-NOW en WiFi
void connectESPNow() {
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  
  Serial.println("Netwerk info:");
  Serial.print(" - MAC-adres: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("!!! Fout bij ESP-NOW start. Herstarten over 2 sec...");
    delay(2000);
    ESP.restart();
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println(" - ESP-NOW status: OK");
}

// failsafe functie: hardware uitschakelen en wachten op fysieke reset
void enterFailsafe(String reden) {

  Serial.println("KRITIEKE STOP: " + reden);
  
  // 1. Hardware direct afschakelen
  digitalWrite(PIN_SSR_MAIN, LOW); 
  isFailsafe = true; // ← NIEUW: vergrendel het systeem
  
  Serial.println("STATUS: Systeem vergrendeld.");
  Serial.println("ACTIE: Druk op de fysieke reset-knop...");


  // 2. Blokkerende loop: wacht tot knop van HIGH naar LOW gaat (PULLUP)
  while (digitalRead(PIN_RESET_BTN) == HIGH) {
      delay(50); 
  }
  //debounce
  delay(50);
  while (digitalRead(PIN_RESET_BTN) == LOW) {
      delay(50);
  }
  delay(200);

  Serial.println("\nKnop ingedrukt! Hardware herstart...");
  delay(500);
  ESP.restart(); // hardware restart van de ESP
}

//callback functie voor ontvangen data via ESP-NOW
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));

  if (incoming.sender == 0) { 
    if (incoming.command == 0x01) { 
      lastOkTime = millis();
      if (!isFailsafe) {               
        digitalWrite(PIN_SSR_MAIN, HIGH); 
      }
    } 
    else if (incoming.command == 0x02) { 
      enterFailsafe("Handmatige noodstop via controller");
    }
  }
}


void setup() {
  Serial.begin(115200);
  
  delay(1000);
  Serial.println("\n--- SAFETY ESP OPSTARTSEQUENCE ---");
  
  // Hardware configuratie
  pinMode(PIN_SSR_MAIN, OUTPUT);
  digitalWrite(PIN_SSR_MAIN, LOW); // Altijd uit bij start
  
  pinMode(PIN_RESET_BTN, INPUT_PULLUP); 

  // Netwerk configuratie
  connectESPNow();
  
  lastOkTime = millis();
  Serial.println("Systeem gereed. Wacht op hartslag...");
}


void loop() {
  // Check of de controller nog in leven is momenteel uit (zie documentatie)
  // if (millis() - lastOkTime > okTimeout) {
  //   enterFailsafe("Verbinding met controller verloren (timeout)");
  // }
  
  // delay(10);
}
