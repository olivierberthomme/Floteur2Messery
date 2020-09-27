#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266Ping.h>
#include <TaskScheduler.h>
#include <secrets.h>
#include <UniversalTelegramBot.h>

#define HOSTNAME "FloteurDeMessery"
#define CONTACT_GPIO 5 // GPIO 5 == D1
#define WIFI_GPIO 13 // GPIO 13 == D7
#define LED_GPIO 12 // GPIO 12 == D6

// Initialize Telegram BOT
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

// Wifi variables
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
// IP to ping
IPAddress ip (8,8,8,8);

// Scheduler variables
#define _TASK_SLEEP_ON_IDLE_RUN

void lecture_contact(){
  // Read GPIO and print it on Serial port
  Serial.print("State of GPIO");
  Serial.print(CONTACT_GPIO);
  Serial.print(": ");
  Serial.println(digitalRead(CONTACT_GPIO));

  if(digitalRead(CONTACT_GPIO) == 0){
    digitalWrite(LED_GPIO,LOW);
    bot.sendMessage(CHAT_ID, "Le floteur ne fait plus contact.", "");
  }else{
    digitalWrite(LED_GPIO,HIGH);
    bot.sendMessage(CHAT_ID, "Le floteur fait contact !", "");
  }
}

void lecture_wifi(){
  int avg_ms = 2000;
  Ping.ping(ip, 2);
  avg_ms = Ping.averageTime();

  Serial.print("ping:");
  Serial.print(avg_ms);
  Serial.println("ms");
  // digitalWrite(15,LOW);

  if(avg_ms < 300){
    digitalWrite(WIFI_GPIO,HIGH);
  }else{
    digitalWrite(WIFI_GPIO,LOW);
    Serial.println("Connection Failed! Rebooting...");
    delay(2000);
    ESP.restart();
  }
}

// Timer/Scheduler
Scheduler runner;

// List tasks
Task verifie_contact(1000, TASK_FOREVER, &lecture_contact, &runner, true);  // Vérifie le contact
Task verifie_wifi(1000, TASK_FOREVER, &lecture_wifi, &runner, true);  // Vérifie le wifi

void setup() {
  // Region WiFi
  Serial.begin(115200);
  Serial.println("\nBooting\n");

  pinMode(CONTACT_GPIO, INPUT);
  pinMode(WIFI_GPIO, OUTPUT);
  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(WIFI_GPIO,LOW);
  digitalWrite(LED_GPIO,LOW);


  WiFi.mode(WIFI_STA);
  WiFi.hostname(HOSTNAME);
  WiFi.begin(ssid, password);
  delay(2000);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(2000);
    ESP.restart();
  }
  Serial.println("Ready");
  Serial.print("IP address:");
  Serial.println(WiFi.localIP().toString());
  // Region WiFi end

  // Region Timer
  runner.startNow();  // set point-in-time for scheduling start
  // Region Timer end

  // Region TelegramBot
  client.setInsecure();
  bot.sendMessage(CHAT_ID, "ESP8266 connecté", "");
  bot.sendMessage(CHAT_ID, WiFi.localIP().toString(), "");
  // Region TelegramBot end
}

void loop() {
  // Timer
  runner.execute();
}
