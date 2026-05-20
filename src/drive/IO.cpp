// IO 
#include <esp_now.h>
#include <WiFi.h>

//pins
const uint8_t PIN_HALL       = 14;
const uint8_t PIN_BUZZER     = 27;
const uint8_t PIN_THROTTLE   = 25;   // DAC
const uint8_t PIN_BRAKE      = 26;   // DAC
const uint8_t PIN_FRONTLIGHT = 32;
const uint8_t PIN_BRAKELIGHT = 33;

//controller mac (voor snelheid doorsturen (nog uit te breiden))
uint8_t controllerAddress[] = {0xEC, 0x62, 0x60, 0x9D, 0x28, 0xF8};


enum DeviceType : uint8_t {
  SERVER = 0,
  SAFETY = 1,
  IO_ESP = 2,
  SERVO_ESP = 3
};

enum Commands : uint8_t {
  CMD_OK         = 0x01,
  CMD_STOP       = 0x02,
  CMD_BUZZ       = 0x05,
  CMD_SPEED_DATA = 0x0A
};

// struct. Niet geoptimaliseerd, zie documentatie
typedef struct struct_message {
  uint8_t sender;
  uint8_t command;
  int     throttle;
  int     brake;
  int     sturen;
  float   value;
  bool    buzzer;
} struct_message;

struct_message msg;

//variabelen
const float maxVoltage = 3.3;

// Failsafe hold-time
unsigned long lastValidTime  = 0;
const unsigned long holdTime = 200;
float lastThrottle = 0;
float lastBrake    = 0;

// Hall sensor
volatile unsigned long pulseCount = 0;
unsigned long lastSpeedTime = 0;

void IRAM_ATTR hallISR() {
  pulseCount++;
}

// ESP-NOW receive callback
void onDataRecv(const uint8_t* mac,
                const uint8_t* incomingData,
                int len) {

  struct_message incomingMsg;
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));

  if (incomingMsg.sender != SERVER) return;

  // BUZZER
  if (incomingMsg.buzzer) {
    digitalWrite(PIN_BUZZER, HIGH);
    } else {
    digitalWrite(PIN_BUZZER, LOW);
    }

  // ADC waarden (0–4095) omzetten naar spanning (0–3.3V)
  float throttleEff = (incomingMsg.throttle / 4095.0) * maxVoltage;
  float brakeEff    = (incomingMsg.brake    / 4095.0) * maxVoltage;

  //failsafe: als er korte tijd geen valide input is, houd de laatste waarde vast (om abrupt stoppen te voorkomen bij tijdelijke verbindingsproblemen)
  if (throttleEff != 0 || brakeEff != 0) {
    lastValidTime = millis();
  } else {
    if (millis() - lastValidTime < holdTime) {
      throttleEff = lastThrottle;
      brakeEff    = lastBrake;
    }
  }

  // grenzen aanhouden (om onverwachte waarden door ruis of fouten te voorkomen)
  throttleEff = constrain(throttleEff, 0, maxVoltage);
  brakeEff    = constrain(brakeEff,    0, maxVoltage);

  // DAC output (0–255) op basis van de effectieve spanning
  dacWrite(PIN_THROTTLE, (throttleEff / maxVoltage) * 255);
  dacWrite(PIN_BRAKE,    (brakeEff    / maxVoltage) * 255);

  // laatste waarden bijhouden voor failsafe
  lastThrottle = throttleEff;
  lastBrake    = brakeEff;

  //remlicht aan als remmen boven een bepaalde drempel is (om visuele feedback te geven bij het remmen)
  digitalWrite(PIN_BRAKELIGHT, brakeEff >= 1.5 ? HIGH : LOW);

  Serial.print("Throttle: "); Serial.print(throttleEff);
  Serial.print(" | Brake: ");  Serial.println(brakeEff);
}


void setup() {
  Serial.begin(115200);

  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_FRONTLIGHT, OUTPUT);
  pinMode(PIN_BRAKELIGHT, OUTPUT);
  pinMode(PIN_HALL,       INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, RISING);

  WiFi.mode(WIFI_STA);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, controllerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  esp_now_register_recv_cb(onDataRecv);

  digitalWrite(PIN_FRONTLIGHT, HIGH);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.println("IO ESP Ready");
}

//loop enkel voor snelheid doorsturen, rest gebeurt in onDataRecv (om directe reactie op ontvangen data mogelijk te maken)
void loop() {

  if (millis() - lastSpeedTime > 500) {

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float speed = pulses * 0.5;

    msg.sender  = IO_ESP;
    msg.command = CMD_SPEED_DATA;
    msg.value   = speed;

    esp_now_send(controllerAddress, (uint8_t*)&msg, sizeof(msg));

    Serial.print("Speed: ");
    Serial.println(speed);

    lastSpeedTime = millis();
  }
}
