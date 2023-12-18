#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266Ping.h>
#include <TaskScheduler.h>
#include <secrets.h>
#include <UniversalTelegramBot.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>

#define HOSTNAME "FloteurDeMessery"
#define CONTACT_GPIO 5 // GPIO 5 == D1
#define WIFI_GPIO 13 // GPIO 13 == D7
#define LED_GPIO 12 // GPIO 12 == D6
#define MAX_FAILING_WIFI 10 // 10 x 1mn == 10mn
int previous_gpio_state = 0; // Previous state of CONTACT_GPIO

// Initialize Telegram BOT
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

// Wifi variables
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
int failingWifi = 0;
// IP to ping
IPAddress ip (8,8,8,8);

// NTP param
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, TASK_MINUTE*60);

// Scheduler variables
#define _TASK_SLEEP_ON_IDLE_RUN

time_t getArduinoDueTime(){
  return timeClient.getEpochTime();
}

void lecture_contact(){
  int current_gpio_state = digitalRead(CONTACT_GPIO);

  // Read GPIO and print it on Serial port
  Serial.print("State of GPIO");
  Serial.print(CONTACT_GPIO);
  Serial.print(": ");
  Serial.println(current_gpio_state);

  if(current_gpio_state == 0){
    digitalWrite(LED_GPIO,LOW);
  }else{
    digitalWrite(LED_GPIO,HIGH);
  }
  
  if(current_gpio_state != previous_gpio_state){
    if(current_gpio_state == 0){
      Serial.println("Pompes a nouveau OK");
      bot.sendMessage(CHAT_ID, "Pompes à nouveau OK", "");
    }else{
      Serial.println("Probleme pompes detecte !");
      bot.sendMessage(CHAT_ID, "Problème pompes détecté !", "");
    }
  }

  // Send status during daytime
  if(timeClient.getHours() >= 8 && timeClient.getHours() <= 20){
    // When all good, send OK message at 10am
    if(current_gpio_state == 0 && timeClient.getHours() >= 10 && timeClient.getHours() < 14){
      bot.sendMessage(CHAT_ID, "Pompes OK", "");
      if(month() == 4 && day() == 3){
        bot.sendMessage(CHAT_ID, ",.-~*´¨¯¨`*·~-.-(  Bon anniversaire !  )-,.-~*´¨¯¨`*·~-.", "");
      }
    }

    // When something wrong, send message
    if(current_gpio_state == 1){
      bot.sendMessage(CHAT_ID, "Problème pompes !", "");
    }
  }
  previous_gpio_state = current_gpio_state;
}

void lecture_wifi(){
  // wait 1 minute before WiFi check
  if(millis()>TASK_MINUTE){
    int avg_ms = 2000;
    Ping.ping(ip, 9);
    avg_ms = Ping.averageTime();

    Serial.print(timeClient.getFormattedTime());
    Serial.print("-");
    Serial.print("ping:");
    Serial.print(avg_ms);
    Serial.println("ms");

    if(avg_ms < 2000){
      digitalWrite(WIFI_GPIO,HIGH);
      failingWifi=0;
    }else{
      digitalWrite(WIFI_GPIO,LOW);
      Serial.println("Connection Failed! Reconnecting...");
      WiFi.reconnect();
      delay(2000);
      while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection failed again...");
        failingWifi=failingWifi+1;
        if (failingWifi>MAX_FAILING_WIFI){
          ESP.restart();
        }
      }
      Serial.println("Connection fails but reconnected :)");
      // bot.sendMessage(CHAT_ID, "Connection fails but reconnected :)", "");
      digitalWrite(WIFI_GPIO,HIGH);
    }
  }
}

// Timer/Scheduler
Scheduler runner;

// List tasks
Task verifie_contact(TASK_MINUTE*60*3, TASK_FOREVER, &lecture_contact, &runner, true);  // Vérifie le contact
Task verifie_wifi(TASK_MINUTE, TASK_FOREVER, &lecture_wifi, &runner, true);  // Vérifie le wifi

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
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
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
  digitalWrite(WIFI_GPIO,HIGH);
  // Region WiFi end

  // Region Timer
  runner.startNow();  // set point-in-time for scheduling start
  timeClient.begin(); // start NTP client
  // Region Timer end

  // Region TelegramBot
  client.setInsecure();
  bot.sendMessage(CHAT_ID, "ESP8266 connecté", "");
  bot.sendMessage(CHAT_ID, WiFi.localIP().toString(), "");
  // Region TelegramBot end

  setSyncProvider(getArduinoDueTime);
}

void loop() {
  // Timer
  runner.execute();
  // NTP update
  timeClient.update();

  delay(50);
}
