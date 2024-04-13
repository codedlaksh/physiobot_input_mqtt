#include <ESP32_Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp32-hal-ledc.h"
#include <SPIFFS.h>
#include<string.h>
#include <PubSubClient.h>


#define SERVO_PIN_1 26 // Change this to your servo's connected pin
#define SERVO_PIN_2 27 /
#define LEDC_CHANNEL 0
#define LEDC_CHANNEL_1 1
#define LEDC_FREQUENCY 50
#define LEDC_RESOLUTION 16
#define DEAD_MAN 5

// VARIABLES
Servo myservo; // create a servo object
WebServer server(80);
WebSocketsServer webSocket(81);
int angle_calc(int pulse_width_us);
int pulse_width_calculator(int angle);
void initial_setup();
// WiFi AP settings
const char *ssid = "Physiobot";
const char *password = "123456789";
volatile bool jog_btn1_state = false;
volatile bool jog_btn2_state = false;
volatile int hold_time = 3;
volatile int start_pos = 0;
volatile int start_pos_1=0;
volatile int stop_pos = 0;
volatile int stop_pos_1 = 0;
volatile int speed = 0;
volatile int count = 0;
// int servo_min_pulse = 0;  // minimum pulse time in microseconds
// int servo_max_pulse = 0; // maximum pulse time in microseconds
// volatile int pulse_width_us = 0;
// volatile int pulse_width_us_1 = 0;

int servo_min_pulse = 500;  // minimum pulse time in microseconds
int servo_max_pulse = 2500; // maximum pulse time in microseconds
volatile int pulse_width_us = 500;
volatile int pulse_width_us_1 = 2500;
volatile bool session_running = false;
volatile int session_count = 5;
 char *ssid_new;
 char *password_new;
volatile int flag = 0;
volatile int flag2 = 0;
volatile int init_flag=0;
volatile int pulse=0;
volatile int angle_calculator=0;
volatile int input_angle=0;
volatile int input_angle_min=0;
volatile int input_angle_max=0;
volatile int angle2=0;
volatile int angle3=0;
bool ok=0;
  char name[]={0};
  char start_angle[]={0};
   char stop_angle[]={0};
    char reps[]={0};

  int led = 2; // led on board
  int jogger = 0;
  int starter = 1;
  int jogged_angle = 0;
// MQTT Server details
const char* mqtt_server = "f72b8337ec8b4c9ea2a1da1effbced2f.s1.eu.hivemq.cloud";
const int mqtt_port = 8883; // Use port 8883 for TLS

// MQTT authentication
const char* mqtt_client_id = "ESP32Client"; // Unique client ID
const char* mqtt_username = "info.lakshmigopi@gmail.com";
const char* mqtt_password = "Laksh@2024";

// Topics
const char* publish_topics[] = {"physiobot/sessionnumber", "physiobot/date", "physiobot/duration",
                                "physiobot/high_pos", "physiobot/low_pos", "physiobot/speed",
                                "physiobot/repitions"};
const char* subscribe_topics[] = {"physiobot/patientName1", "physiobot/startPos1", "physiobot/stopPos1", "physiobot/countPos1"};

// CA Certificate
const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);


  // Task handles
  TaskHandle_t servoTaskHandle;

  // FUNCTIONS

  void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    // Serial.print("Message arrived [");
    Serial.print(topic);
    if(topic=="physiobot/patientName1"){
      for (int i = 0; i < length; i++) {
        name[i]=(char)payload[i];
    }
    }else if(topic=="physiobot/startPos1"){
      for (int i = 0; i < length; i++) {
        start_angle[i]=(char)payload[i];
    }
    }
    else if(topic=="physiobot/stopPos1"){
      for (int i = 0; i < length; i++) {
        stop_angle[i]=(char)payload[i];
    }
    }
    else if(topic=="physiobot/countPos1"){
      for (int i = 0; i < length; i++) {
        reps[i]=(char)payload[i];
    }
    }
    
}

void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT Broker!");
            // Subscribe to topics
            for (auto& topic : subscribe_topics) {
                mqttClient.subscribe(topic);
                Serial.print("Subscribed to ");
                Serial.println(topic);
            }
        } else {
            Serial.print("Failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}


  void servoTask(void *pvParameters) {
    for (;;) {
     if (jog_btn1_state && pulse_width_us < servo_max_pulse && pulse_width_us_1> servo_min_pulse){

      if(digitalRead(DEAD_MAN)==HIGH){
         digitalWrite(led, HIGH);
        servoWriteMicroseconds(pulse_width_us);
        servo2WriteMicroseconds(pulse_width_us_1);
        delay(hold_time);
        pulse_width_us += 1;
        pulse_width_us_1-=1;
        
      
      }
      else {
        while(!digitalRead(DEAD_MAN)==HIGH){
           vTaskDelay(pdMS_TO_TICKS(10));
      }

      }
        // servoWriteMicroseconds(pulse_width_us);// reset motor to 0°
        // servo2WriteMicroseconds(pulse_width_us_1);
        
      } else if (jog_btn2_state && pulse_width_us > servo_min_pulse && pulse_width_us_1<servo_max_pulse) {
        if(digitalRead(DEAD_MAN)==HIGH){
          digitalWrite(led, HIGH);
        servoWriteMicroseconds(pulse_width_us);
        servo2WriteMicroseconds(pulse_width_us_1);
        delay(hold_time);
        pulse_width_us -= 1;
        pulse_width_us_1+=1;
        
      
      }
      else {
        while(!digitalRead(DEAD_MAN)==HIGH){
           vTaskDelay(pdMS_TO_TICKS(10));
      }
      }
        // servoWriteMicroseconds(pulse_width_us);// reset motor to 0°
        // servo2WriteMicroseconds(pulse_width_us_1);
       
        
      } else {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
  }

  void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
      case WStype_TEXT:
      
        if (length > 0) {

          if (payload[0] == 'd' && payload[1] == 'o' && payload[2] == 'w' && payload[3] == 'n' && payload[4] == '1') {
            jog_btn1_state = true;
            int angle=angle_calc(pulse_width_us);
            Serial.println(angle);
                String message = "motorPosition1:" + String(angle); // Already converted to String, send as integer
            webSocket.sendTXT(num, message.c_str());
            // digitalWrite(led, HIGH);
            Serial.println("Motor jog clockwise");
            // angle_calculator=angle_calc(pulse_width_us);
            // Serial.println(angle_calculator);
            // Serial.println(hold_time);
          } else if (payload[0] == 'u' && payload[1] == 'p' && payload[2] == '1') {
            jog_btn1_state = false;
            digitalWrite(led, LOW);
            Serial.println("Motor stop");
          //  angle_calculator=angle_calc(pulse_width_us);
            // Serial.println(angle_calculator);
          } else if (payload[0] == 'd' && payload[1] == 'o' && payload[2] == 'w' && payload[3] == 'n' && payload[4] == '2') {
            jog_btn2_state = true;
            int angle=angle_calc(pulse_width_us);
             Serial.println(angle);
                String message = "motorPosition1:" + String(angle); // Already converted to String, send as integer
            webSocket.sendTXT(num, message.c_str());
            // digitalWrite(led, HIGH);
            Serial.println("Motor jog counter_clockwise");
            // //  angle_calculator=angle_calc(pulse_width_us);
            // Serial.println(angle_calculator);
          } else if (payload[0] == 'u' && payload[1] == 'p' && payload[2] == '2') {
            jog_btn2_state = false;
            digitalWrite(led, LOW);
            Serial.println("Motor stop");
            //  angle_calculator=angle_calc(pulse_width_us);
            // Serial.println(angle_calculator);
          } 
          
          
          
          
          
          
          else if (payload[0] == 'i' && payload[1] == 'n' && payload[2] == 'c' && payload[3] == '_' && payload[4] == 's' && payload[5] == 'p' && payload[6] == 'e' && payload[7] == 'e' && payload[8] == 'd') {
            if (hold_time >0) {//updated max speed for test
              hold_time =hold_time-1;;
             String message = "motorPosition4:" + String(hold_time); // Already converted to String, send as integer
            webSocket.sendTXT(num, message.c_str());
              Serial.println("Speed increased");
               Serial.println(hold_time);
            }
          } else if (payload[0] == 'd' && payload[1] == 'e' && payload[2] == 'c' && payload[3] == '_' && payload[4] == 's' && payload[5] == 'p' && payload[6] == 'e' && payload[7] == 'e' && payload[8] == 'd') {
            if (hold_time < 10) {
               hold_time = hold_time+1;
                String message = "motorPosition5:" + String(hold_time); // Already converted to String, send as integer
            webSocket.sendTXT(num, message.c_str());
              Serial.println("Speed decreased");
               Serial.println(hold_time);
            }
          } else if (payload[0] == 's' && payload[1] == 't' && payload[2] == 'o' && payload[3] == 'r' && payload[4] == 'e' && payload[5] == '_' && payload[6] == 's' && payload[7] == 't' && payload[8] == 'a' && payload[9] == 'r' && payload[10] == 't' && payload[11] == '_' && payload[12] == 'p' && payload[13] == 'o' && payload[14] == 's') {
            start_pos = pulse_width_us;
            start_pos_1=pulse_width_us_1;
             angle2=angle_calc(start_pos);
         Serial.println(angle2);
                String message = "motorPosition2:" + String(angle2); // Already converted to String, send as integer
            webSocket.sendTXT(num, message.c_str());
            
            Serial.print("Start Position stored: ");
            //  angle_calculator=angle_calc(pulse_width_us);
            // Serial.println(angle_calculator);
          } else if (payload[0] == 's' && payload[1] == 't' && payload[2] == 'o' && payload[3] == 'r' && payload[4] == 'e' && payload[5] == '_' && payload[6] == 's' && payload[7] == 't' && payload[8] == 'o' && payload[9] == 'p' && payload[10] == '_' && payload[11] == 'p' && payload[12] == 'o' && payload[13] == 's') {
            stop_pos = pulse_width_us;
            stop_pos_1= pulse_width_us_1;
             angle3=angle_calc(stop_pos);
         Serial.println(angle3);
                String message = "motorPosition3:" + String(angle3); // Already converted to String, send as integer
            webSocket.sendTXT(num, message.c_str());
            
            Serial.print("Stop Position stored: ");
            //  angle_calculator=angle_calc(pulse_width_us);
            // Serial.println(angle_calculator);
          } else if (payload[0] == 's' && payload[1] == 't' && payload[2] == 'o' && payload[3] == 'r' && payload[4] == 'e' && payload[5] == '_' && payload[6] == 's' && payload[7] == 'p' && payload[8] == 'e' && payload[9] == 'e' && payload[10] == 'd') {
            speed=hold_time;
            Serial.print("Speed stored: ");
            Serial.println(speed);
          } else if (payload[0] == 's' && payload[1] == 't' && payload[2] == 'o' && payload[3] == 'r' && payload[4] == 'e' && payload[5] == '_' && payload[6] == 'c' && payload[7] == 'o' && payload[8] == 'u' && payload[9] == 'n' && payload[10] == 't') {
            session_count = atoi("count");
            Serial.print("Count stored: ");
            Serial.println(count);
          } else if (payload[0] == 's' && payload[1] == 't' && payload[2] == 'a' && payload[3] == 'r' && payload[4] == 't' && payload[5] == '_' && payload[6] == 's' && payload[7] == 'e' && payload[8] == 's' && payload[9] == 's' && payload[10] == 'i' && payload[11] == 'o' && payload[12] == 'n') {
            session_running = true;
            flag2=1;
            // count=0;
            session_count = count;
           
            Serial.println("Session started");
          } else if (payload[0] == 's' && payload[1] == 't' && payload[2] == 'o' && payload[3] == 'p' && payload[4] == '_' && payload[5] == 's' && payload[6] == 'e' && payload[7] == 's' && payload[8] == 's' && payload[9] == 'i' && payload[10] == 'o' && payload[11] == 'n') {
            session_running = false;
            String message = "motorPosition6:" + String(count); // Already converted to String, send as integer
            webSocket.sendTXT(num, message.c_str());
            Serial.println("Session stopped");
          }

          else if (payload[0] == 'g' && payload[1] == 'e' && payload[2] == 't' && payload[3] == '_' && payload[4] == 'p' && payload[5] == 'r' && payload[6] == 'e' && payload[7] == 's' && payload[8] == 'c' && payload[9] == 'r' && payload[10] == 'i' && payload[11] == 'p') {
            session_running = false;
            String message1 = "data1:" + String(name); // Already converted to String, send as integer
            webSocket.sendTXT(num, message1.c_str());
           String message2 = "data2:" + String(start_angle); // Already converted to String, send as integer
            webSocket.sendTXT(num, message2.c_str());
            String message3 = "data3:" + String(stop_angle); // Already converted to String, send as integer
            webSocket.sendTXT(num, message3.c_str());
             String message4 = "data4:" + String(reps); // Already converted to String, send as integer
            webSocket.sendTXT(num, message4.c_str());
            
          }
          else if (payload[0] == 'S' && payload[1] == 'S' && payload[2] == 'I' && payload[3] == 'D') {
        free(ssid_new); // Free the old ssid_new memory if it was previously allocated
        ssid_new = (char *)malloc(length + 1); // Allocate memory for the new SSID plus null terminator
        memcpy(ssid_new, payload + 4, length - 4); // Skip the 'SSID' part
        ssid_new[length - 4] = '\0'; // Null terminate the string
    }
    
    // Password case
    else if (payload[0] == 'P' && payload[1] == 'a' && payload[2] == 's' && payload[3] == 's') {
        free(password_new); // Free the old password_new memory if it was previously allocated
        password_new = (char *)malloc(length + 1); // Allocate memory for the new password plus null terminator
        memcpy(password_new, payload + 8, length - 8); // Skip the 'Password' part
        password_new[length - 8] = '\0'; // Null terminate the string
        flag = 1;
        Serial.println("Credentials received, attempting to connect...");
        if (flag == 1) {
            connect_to_wifi();
        }
    }
     
        }
    break;
          }
        }
        
    
  

void servoWriteMicroseconds(int pulseUs) {
  long duty = (65536L * pulseUs) / 20000; // Convert microseconds to duty cycle
  ledcWrite(LEDC_CHANNEL, duty);
}

void servo2WriteMicroseconds(int pulseUs2) {
  long duty = (65536L * pulseUs2) / 20000; // Convert microseconds to duty cycle
  ledcWrite(LEDC_CHANNEL_1, duty);
}

void connect_to_wifi() {
    WiFi.begin(ssid_new, password_new);
    
    Serial.println(F("Attempting to connect to WiFi..."));
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) { // Wait for 10 seconds
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected. My IP address is: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println(F("Failed to connect."));
    }

    espClient.setCACert(ca_cert); // Set the CA certificate

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqtt_callback);

    reconnect(); // Ensure we're connected and subscribed on setup
}
  void setup() {
   ledcSetup(LEDC_CHANNEL, LEDC_FREQUENCY, LEDC_RESOLUTION);
   ledcSetup(LEDC_CHANNEL_1, LEDC_FREQUENCY, LEDC_RESOLUTION);
   ledcAttachPin(26, LEDC_CHANNEL); // Connect LEDC channel to pin 26
    ledcAttachPin(27, LEDC_CHANNEL_1); // Connect LEDC channel to pin 27
    pinMode(led, OUTPUT);
    pinMode(DEAD_MAN, INPUT_PULLDOWN);
    Serial.begin(9600);

    Serial.println(">>SERVO TEST:");
    WiFi.softAP(ssid, password);
      
     
    Serial.println("Access Point Started");
    Serial.println("IP Address: ");
    Serial.println(WiFi.softAPIP());
   

    // Serve the HTML page when the root URL is accessed
if (!SPIFFS.begin(true)) {
  Serial.println("An Error has occurred while mounting SPIFFS");
  return;
}

   
    server.on("/", HTTP_GET, []() {
  File file = SPIFFS.open("/index.html", "r");
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
});
server.on("/Connect_to_internet.html", HTTP_GET, []() {
  File file = SPIFFS.open("/Connect_to_internet.html", "r");
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
});
server.on("/style.css", HTTP_GET, []() {
  File file = SPIFFS.open("/style.css", "r");
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.streamFile(file, "text/css");
  file.close();
});


    // Start the web server
    server.begin();

    // Initialize WebSocket server
    webSocket.begin();
    webSocket.onEvent(handleWebSocketEvent);

    // Create servo task
    xTaskCreatePinnedToCore(
      servoTask,          // Task function
      "ServoTask",        // Task name
      10000,              // Stack size (bytes)
      NULL,               // Task parameters
      1,                  // Task priority
      &servoTaskHandle,   // Task handle
      0                   // Core ID (0 or 1)
    );

    // while(init_flag==0){
      // initial_setup();
      //  pulse_width_us =servo_min_pulse;
      //  pulse_width_us_1=servo_max_pulse;
  //  servoWriteMicroseconds(pulse_width_us);// reset motor to 0°
  //  servo2WriteMicroseconds(pulse_width_us_1);
  //  Serial.println(pulse_width_us);
  //   Serial.println(pulse_width_us_1);

    // }
     //attachInterrupt(digitalPinToInterrupt(dead_man_pin), deadManInterrupt, FALLING);
  }

  void loop() {
    server.handleClient();
    webSocket.loop();
     
        
      
        
     if (session_running ) {
      //  myservo.writeMicroseconds(start_pos);
      //  delay(hold_time);
      
        for((pulse_width_us = stop_pos) &&( pulse_width_us_1 = stop_pos_1); (pulse_width_us >= start_pos) && (pulse_width_us_1 <=start_pos_1); (pulse_width_us -= 1) && (pulse_width_us_1+=1)) {
          if(digitalRead(DEAD_MAN)==HIGH){
        servoWriteMicroseconds(pulse_width_us);
        servo2WriteMicroseconds(pulse_width_us_1);
        delay(hold_time);
      
      }
      else {
        while(!digitalRead(DEAD_MAN)==HIGH){

      }

      }
      
        } 
      
      delay(1000);
      
      
        for((pulse_width_us = start_pos )&&( pulse_width_us_1 = start_pos_1) ; (pulse_width_us <= stop_pos) && (pulse_width_us_1 >=stop_pos_1); (pulse_width_us += 1 )&& (pulse_width_us_1-=1 )) {
          if(digitalRead(DEAD_MAN)==HIGH){
        servoWriteMicroseconds(pulse_width_us);
        servo2WriteMicroseconds(pulse_width_us_1);
        delay(hold_time);
      }
      else {
        while(!digitalRead(DEAD_MAN)==HIGH){

      }
      }
      
       
      // count++;
      }
      delay(1000);
      count++;
         }
  if(flag==1){
 if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    // Example publish logic (this should be tailored to your application needs)
    static unsigned long lastPublishTime = 0;
    if (millis() - lastPublishTime > 10000) {  // Publish every 10 seconds
        mqttClient.publish(publish_topics[0], "2");
        mqttClient.publish(publish_topics[1], "2024-04-17");
        mqttClient.publish(publish_topics[2], "5");
        mqttClient.publish(publish_topics[3],String(angle2).c_str() );
        mqttClient.publish(publish_topics[4], String(angle3).c_str());
        mqttClient.publish(publish_topics[5], String(hold_time).c_str());
        mqttClient.publish(publish_topics[6], String(count).c_str());
        // Add additional publishes as needed
        lastPublishTime = millis();
    }
  }
        
  }
      
      // delay(1000);
    
      // if (pulse_width_us ==start_pos ) {
      //   pulse_width_us += 1;
      //   digitalWrite(led,HIGH);
      //   myservo.writeMicroseconds(pulse_width_us);
      //   delay(hold_time);
      // } else if (pulse_width_us ==stop_pos ){
      //  pulse_width_us --;
      //  digitalWrite(led,LOW);
      //   myservo.writeMicroseconds(pulse_width_us);
      //   delay(hold_time);
      //   session_count--;
      // }
      
//     void deadManInterrupt() {
//   detachInterrupt(digitalPinToInterrupt(dead_man_pin));
//   Serial.print("Current pulse_width_us: ");
//   Serial.println(pulse_width_us);
//   Serial.print("Current pulse_width_us_1: ");
//   Serial.println(pulse_width_us_1);
//   Serial.print("Current count: ");
//   Serial.println(count);
//   while (1) {
//     // Exit from every loop
//   }
// }

int angle_calc(int pulse_width_us){
  
  return map(pulse_width_us,servo_min_pulse,servo_max_pulse,0,270);
}
  
//  int pulse_width_calculator(int angle){
//   int pulse_width_calculated = (angle/0.135)+500;
//   return pulse_width_calculated;
//  } 

// void initial_setup(){
// Serial.println("Enter min angle: ");
// while(Serial.available()==0){
 
// }
// String input_str = Serial.readStringUntil('\n');
// input_angle_min = input_str.toInt();
// servo_min_pulse=pulse_width_calculator(input_angle_min);
// Serial.println(servo_min_pulse);

// Serial.println("Enter Max angle: ");

// while(Serial.available()==0){
 
// }
// input_str = Serial.readStringUntil('\n');
// input_angle_max = input_str.toInt();
// servo_max_pulse=pulse_width_calculator(input_angle_max);
// Serial.println(servo_max_pulse);


// init_flag=1;
// Serial.println(init_flag);
// return;
// }




