#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

const char* mqtt_server = "broker.hivemq.com";
const int port = 1883;
const char* topic = "v3d/robot/gesture";

StaticJsonDocument<200> doc;
const char* command_last_l="";
const char* command_last_r="";

const char* ssid = "EpsonPrinter-4800";
const char* password = "enderdisgrace15";
WiFiClient espClient;
PubSubClient client(espClient);

/* ---------- PWM ---------- */
const int PWM_FREQ = 5000;
const int PWM_RES = 8;
const int VELOCIDAD = 190;

// ----------- WIFI -----------
void setup_wifi() {
  Serial.begin(115200);
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado");
}

// ----------- CALLBACK MQTT -----------
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en topic: ");
  Serial.println(topic);
  
  // Parsear JSON directamente desde payload
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("Error al parsear JSON: ");
    Serial.println(error.c_str());
    return;
  }
  // Leer campos
  const char* command_cur_l = doc["command_left"];
  const char* command_cur_r = doc["command_right"];
  
  long timestamp_cur = doc["timestamp_ms"];
  Serial.println(command_cur_l);
  Serial.print(" ");
  Serial.println(command_cur_r);
  if (command_cur_l!= command_last_l && command_cur_r!= command_last_r)
  handleCommand(String(command_cur_l),String(command_cur_r));
  command_last_l=command_cur_l;
  command_last_r=command_cur_r;

}

// ----------- RECONEXIÓN MQTT -----------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");

    if (client.connect("ArduinoSubscriber")) {
      Serial.println("conectado");

      // Suscribirse al topic
      client.subscribe(topic);

    } else {
      Serial.print("error, rc=");
      Serial.print(client.state());
      Serial.println(" reintentando en 5s");
      delay(5000);
    }
  }
}


void handleCommand(String left, String right){

  if (left="forward_left") {
    ledcWrite(IN1, VELOCIDAD);
    ledcWrite(IN2, 0);
  } else if (left="backward_left") {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, VELOCIDAD);
  } else {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, 0);
  }

  // Motor derecho
  if (right="forward_right") {
    ledcWrite(IN3, VELOCIDAD);
    ledcWrite(IN4, 0);
  } else if (right="backward_right") {
    ledcWrite(IN3, 0);
    ledcWrite(IN4, VELOCIDAD);
  } else {
    ledcWrite(IN3, 0);
    ledcWrite(IN4, 0);
  }

}


// ----------- SETUP -----------
void setup() {
  ledcAttach(IN1, PWM_FREQ, PWM_RES);
  ledcAttach(IN2, PWM_FREQ, PWM_RES);
  ledcAttach(IN3, PWM_FREQ, PWM_RES);
  ledcAttach(IN4, PWM_FREQ, PWM_RES);

  setup_wifi();
  client.setServer(mqtt_server, port);
  client.setCallback(callback);
}


// ----------- LOOP -----------
void loop() {
  if (!client.connected()) {
    reconnect();
  }


  client.loop();
}