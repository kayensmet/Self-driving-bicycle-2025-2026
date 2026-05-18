#include <WiFi.h>
#include <esp_now.h>
#include <PubSubClient.h>
#include <Bluepad32.h>

//board doit esp32 devkit v1 bluepad kiezen
//staat mac adress correct ? lees de mac adress van de ontvanger pcb eerst
// wifikanaal checken na de boot van de zender pcb en dit invullen in de ontvanger
//check wifi, mqtt,  credentials 

// --- WiFi / MQTT ---
const char* ssid     = "IB3";
const char* password = "ingenieursbeleving3";
//const char* broker   = "192.168.0.33";
const int mqttPort   = 1883;

const char* broker   = "leenux.local";

//ander wifi
// const char* ssid = "";
// const char* password = "";

//const char* ssid = "";
//const char* password = "";


//hotspot 4g
 //const char* ssid = "";
 //const char* password = "";

const char* mqtt_topic_l2 = "controller/l2";
const char* mqtt_topic_r2 = "controller/r2";

bool r1Pressed = false;          // R1 ingedrukt
bool controllerConnected = false; // controller verbonden

// Struct voor data
struct SensorData {
  int   counter;
  float l2Volt; //idee int * 10 om naar een kleiner datatype te gaan en dus mindere data te sturen (naar float)
  float r2Volt;
  bool  r1Pressed;           // 
  bool  controllerConnected; // 
};

SensorData sensorData = {0, 0.0f, 0.0f};

// MQTT setup
WiFiClient espClient;
PubSubClient client(espClient);

// ESP-NOW peer (ontvanger) lees mac uit van de ontvanger
uint8_t receiverMAC[] = {0xE4, 0x65, 0xB8, 0x83, 0x66, 0xC4}; //


// Bluepad32 controller
ControllerPtr myController = nullptr;

// ======== FUNCTIES ========

// WiFi verbinden
void setupWifi() {
    Serial.print("Verbinden met WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi verbonden, IP: " + String(WiFi.localIP()));
    Serial.println("\nWiFi verbonden, IP: " + String(WiFi.localIP()));
    Serial.println("WiFi kanaal: " + String(WiFi.channel()));  // kanaal keuze
}

// MQTT verbinden
void connectMQTT() {
    while (!client.connected()) {
        Serial.print("MQTT verbinden... state: ");
        Serial.println(client.state());
        if (client.connect("ESP32_01")) {
            Serial.println("MQTT verbonden!");
        } else {
            Serial.println("Retry over 5s...");
            delay(5000);
        }
    }
}

// ESP-NOW send callback
void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("ESP-NOW Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ESP-NOW receive callback (van andere nodes, optioneel)
void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
    int receivedValue;
    if (len == sizeof(receivedValue)) {
        memcpy(&receivedValue, data, sizeof(receivedValue));
        Serial.print("Ontvangen via ESP-NOW: ");
        Serial.println(receivedValue);
    }
}

// Bluepad32 callbacks
void onDisconnectedController(ControllerPtr ctl) {
    myController = nullptr;
    controllerConnected = false;
    Serial.println("Controller verbroken");
}

void onConnectedController(ControllerPtr ctl) {
    myController = ctl;
    controllerConnected = true;
    Serial.println("Controller verbonden");
}

// Process controller input
void processControllers() {
    if (myController && myController->isConnected() && myController->hasData()) {
        // buttons
        if (myController->x()) Serial.println("X");
        if (myController->y()) Serial.println("Y");
        if (myController->b()) Serial.println("B");
        if (myController->a()) Serial.println("A");
        
  
        // L2 en R2 triggers (0-1023)
        int l2Value = myController->brake();    // L2
        int r2Value = myController->throttle(); // R2

        r1Pressed = myController->r1();
        if (r1Pressed) {
            Serial.println("R1 ingedrukt");
        } 


        //  Als beide ingedrukt, negeer R2
        int threshold = 50; // pas aan indien nodig
        if (l2Value > threshold && r2Value > threshold) {
            r2Value = 0;
        }

        // Debug: check of R2 ooit != 0
        Serial.print("Throttle raw: "); Serial.println(r2Value);

        sensorData.l2Volt = l2Value * 3.3f / 1023.0f;
        sensorData.r2Volt = r2Value * 3.3f / 1023.0f;

        sensorData.r1Pressed = r1Pressed;
        sensorData.controllerConnected = controllerConnected;

        // MQTT publish
        char l2Str[8], r2Str[8];
        itoa(l2Value, l2Str, 10);
        itoa(r2Value, r2Str, 10);
        client.publish(mqtt_topic_l2, l2Str);
        client.publish(mqtt_topic_r2, r2Str);

        Serial.print("L2: "); Serial.print(l2Value);
        Serial.print(" | R2: "); Serial.println(r2Value);
    }
}

// SETUP 
void setup() {
    Serial.begin(115200);
    setupWifi();
    Serial.println("WiFi kanaal: " + String(WiFi.channel()));

    // Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();

    // MQTT
    client.setServer(broker, mqttPort);
    connectMQTT();

    // ESP-NOW init
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init fout");
        return;
    }

    // Register callbacks
    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSent);

    // Peer info
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = WiFi.channel();  // zender en ontvanger hetzelfde kanaal gebruiken
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Peer toevoegen mislukt");
        return;
    }
}

// ======== LOOP ========
void loop() {
    delay(100);
    if (BP32.update()) processControllers();
    
    //sensorData.controllerConnected = (myController != nullptr); //om te weten of de controller nog verbonden is (niet doen geeft bugs)

    if (!client.connected()) connectMQTT();
    client.loop();

    // Verstuur SensorData elke 1s
    static unsigned long last = 0;
    if (millis() - last > 100) { //verlangen? bij meer data? 
        sensorData.counter++;

        // Lokale copy maken om race conditions te voorkomen
        SensorData toSend = sensorData;
        esp_now_send(receiverMAC, (uint8_t*)&toSend, sizeof(toSend));

        Serial.print("Verstuurd: counter="); Serial.print(sensorData.counter);
        Serial.print(" l2="); Serial.print(sensorData.l2Volt);
        Serial.print(" r2="); Serial.println(sensorData.r2Volt);

        // Optioneel: ook via MQTT
        char buf[10];
        itoa(sensorData.counter, buf, 10);
        client.publish("espnow/zender", buf);

        last = millis();
    }
}