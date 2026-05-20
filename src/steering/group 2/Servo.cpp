// SERVO
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>


const uint8_t PIN_SERVO = 2;


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


typedef struct struct_message {
  uint8_t sender;
  uint8_t command;
  int     throttle;
  int     brake;
  int     sturen;
  float   value;
} struct_message;

struct_message msg;


Servo stuurServo;
const int SERVO_MIDDEN = 90;
const int SERVO_MAX    = 30;   // graden afwijking van midden

// ESP-NOW receive callback
void onDataRecv(const uint8_t* mac,
                const uint8_t* incomingData,
                int len) {

  struct_message incomingMsg;
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));

  if (incomingMsg.sender != SERVER) return;

  // STOP: servo terug naar midden
  if (incomingMsg.command == CMD_STOP) {
    stuurServo.write(SERVO_MIDDEN);
    return;
  }

  // Alleen stuurdata verwerken bij CMD_OK
  if (incomingMsg.command != CMD_OK) return;

  // Dode zone rond joystick-midden (±100 rondom 2048)
  if (incomingMsg.sturen > 1920 && incomingMsg.sturen < 2000) {
    stuurServo.write(SERVO_MIDDEN);
    return;
  }

  // Joystick (0–4095) -> Servo hoek (SERVO_MIDDEN ± SERVO_MAX)
  int hoek = map(incomingMsg.sturen, 0, 4095,
                 SERVO_MIDDEN - SERVO_MAX,
                 SERVO_MIDDEN + SERVO_MAX);

  hoek = constrain(hoek, SERVO_MIDDEN - SERVO_MAX, SERVO_MIDDEN + SERVO_MAX);

  stuurServo.write(hoek);

  Serial.print("Sturen: "); Serial.print(incomingMsg.sturen);
  Serial.print(" | Hoek: ");  Serial.println(hoek);
}


void setup() {
  Serial.begin(115200);

  stuurServo.attach(PIN_SERVO);
  stuurServo.write(SERVO_MIDDEN);   // centreer bij opstart

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

  Serial.println("Servo ESP Ready");
}


void loop() {
  // Alles via callback
}
