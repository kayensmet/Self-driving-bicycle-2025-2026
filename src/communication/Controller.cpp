// CONTROLLER FIETS
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

//pins voor display
#define TFT_CS         5
#define TFT_RST        4 
#define TFT_DC         2
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

//andere pins
const uint8_t PIN_NOODKNOP    = 13;
const uint8_t PIN_BUZZ_BUTTON = 14;
const int PIN_STUUR           = 34;
const int PIN_THROTTLE        = 32;
const int PIN_BRAKE           = 33;
const int PIN_BALANS_AAN      = 12;

//macs
uint8_t safetyAddress[]  = {0xA0, 0xB7, 0x65, 0x46, 0xF7, 0x18};
uint8_t ioAddress[]      = {0x7C, 0x9E, 0xBD, 0x2A, 0xFC, 0x1C};
uint8_t servoAddress[]   = {0xD4, 0x8C, 0x49, 0xE3, 0x68, 0x30}; 
uint8_t balansAddress[] = {0xAC, 0x67, 0xB2, 0x31, 0x96, 0x38};

// struct. Niet geoptimaliseerd, zie documentatie
enum Commands : uint8_t { CMD_OK = 0x01, CMD_STOP = 0x02, CMD_BUZZ = 0x05, CMD_SPEED_DATA = 0x0A, CMD_BALANS = 0x03};
typedef struct struct_message {
  uint8_t sender; uint8_t command; int throttle; int brake; int sturen; float value; bool buzzer; bool balansAan;
} struct_message;
struct_message msg;

//variabelen
unsigned long lastHeartbeat = 0;
const unsigned long heartbeatInterval = 200;
float lastSpeed = 0;
bool vorigeBalansAan = false; 

void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  struct_message incomingMsg;
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));
}


void setup() {
  Serial.begin(115200);

  pinMode(PIN_NOODKNOP,    INPUT_PULLDOWN);
  pinMode(PIN_BUZZ_BUTTON, INPUT_PULLDOWN);
  pinMode(PIN_STUUR,       INPUT);
  pinMode(PIN_THROTTLE,    INPUT);
  pinMode(PIN_BRAKE,       INPUT);
  pinMode(PIN_BALANS_AAN,  INPUT_PULLDOWN);

  //tft ini
  tft.init(170, 320);           
  tft.setRotation(1); 
  tft.fillScreen(ST77XX_BLACK);

  //static ui
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 10);
  tft.println("FIETS CONTROLLER");
  tft.drawFastHLine(0, 35, 320, 0x7BEF);

  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 55);  tft.print("Throttle:");
  tft.setCursor(10, 85);  tft.print("Brake:");
  tft.setCursor(10, 115); tft.print("Steer:");
  tft.setCursor(10, 145); tft.print("Balans:");

  //espnow ini
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(onDataRecv);

  auto addPeer = [](uint8_t* addr) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, addr, 6);
    esp_now_add_peer(&peerInfo);
  };
   addPeer(ioAddress); addPeer(servoAddress); addPeer(balansAddress);
}


void loop() {
  //Input lezen
  int valThrottle = analogRead(PIN_THROTTLE);
  int valBrake    = analogRead(PIN_BRAKE);
  int valStuur    = analogRead(PIN_STUUR);
  bool isBuzzing  = digitalRead(PIN_BUZZ_BUTTON) == HIGH;
  bool balansAan  = digitalRead(PIN_BALANS_AAN) == HIGH;

  //NOODSTOP
  if (digitalRead(PIN_NOODKNOP) == HIGH) {
    msg.command = CMD_STOP;
    esp_now_send(safetyAddress, (uint8_t*)&msg, sizeof(msg));
  }

  //BALANS AAN/UIT
  if (balansAan != vorigeBalansAan) {
    msg.sender  = 0;
    msg.command = CMD_BALANS;
    esp_now_send(balansAddress, (uint8_t*)&msg, sizeof(msg));
    vorigeBalansAan = balansAan;
  }

//HEARTBEAT VERZENDEN
  if (millis() - lastHeartbeat > heartbeatInterval) {
    msg.sender   = 0;
    msg.command  = CMD_OK;
    msg.throttle = valThrottle;
    msg.brake    = valBrake;
    msg.sturen   = valStuur;
    msg.buzzer   = isBuzzing;
    msg.balansAan = balansAan;
    esp_now_send(ioAddress,      (uint8_t*)&msg, sizeof(msg));
    esp_now_send(servoAddress,   (uint8_t*)&msg, sizeof(msg));
    //esp_now_send(safetyAddress, (uint8_t*)&msg, sizeof(msg)); // Wanneer het noodstopcircuit niet verbonden is mag deze regel niet actief zijn. Anders probeert de controller connectie te maken met deze esp, die hij niet vindt en loopt de andere communicatie een delay op.
    esp_now_send(balansAddress, (uint8_t*)&msg, sizeof(msg));
    lastHeartbeat = millis();
  }

  // 4. TFT Update
if (digitalRead(PIN_NOODKNOP) == HIGH) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 10);
  tft.println("FIETS CONTROLLER");
  tft.drawFastHLine(0, 35, 320, 0x7BEF);
  
  tft.setTextColor(ST77XX_RED);
  tft.setCursor(10, 80);
  tft.println("IN NOODSTOP");
  tft.setCursor(10, 110);
  tft.setTextSize(1); 
  tft.println("Reset op de fiets om verder te gaan");
  
  while(digitalRead(PIN_NOODKNOP) == HIGH) { delay(100); } // Wacht tot knop losgelaten wordt
  tft.fillScreen(ST77XX_BLACK); // Wis scherm na reset om statische UI terug te zetten
  setup(); // Herstart setup om de statische labels (Throttle, Brake, etc.) opnieuw te tekenen
} else {
  
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  
  tft.setCursor(140, 55);
  tft.printf("%4d", valThrottle);
  
  tft.setCursor(140, 85);
  tft.printf("%4d", valBrake);

  tft.setCursor(140, 115);
  if (valStuur > 2000) tft.print("Links    "); 
  else if (valStuur < 1910) tft.print("Rechts   ");
  else tft.print("Rechtdoor");

  tft.setCursor(140, 145);
  tft.print(balansAan ? "Aan  " : "Uit  ");
  }
}
