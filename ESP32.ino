#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// WiFi credentials
const char* ssid = "A04";
const char* password = "jbzc4737";

// Firebase project API Key
#define API_KEY "AIzaSyCgyCXHMuQvI-ilCOVQIyjUuR464qNqE4s"

// Firebase project ID
#define FIREBASE_PROJECT_ID "npk-data-tracker-655de"

// Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// User Email and Password
#define USER_EMAIL "nethmal123@gmail.com"
#define USER_PASSWORD "123654789"

// Define LCD address and dimensions
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// Define RS485 pin numbers
#define RE 25 // Receiver Enable
#define DE 26 // Driver Enable
#define pin 12 // buzzer pin
#define VOLTAGE_SENSOR_PIN 34 // voltage sensor pin

// Define button pin
#define BUTTON_PIN 14

// Create ModbusMaster instance
ModbusMaster node;

// SD card setup
File myFile;
int pinCS = 5; // Change this according to your ESP32 setup
uint16_t nitrogen;
uint16_t phosphorus; 
uint16_t potassium;

// NTP Client setup
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

void preTransmission() {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
}

void postTransmission() {
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(4800);

  Wire.begin(21, 22);  // SDA, SCL - adjust these pins if necessary

  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  pinMode(pin, OUTPUT);
  pinMode(VOLTAGE_SENSOR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);

  node.begin(1, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("NPK Sensor");
  lcd.setCursor(0, 1);
  lcd.print("Initializing");
  delay(3000);

  connectWiFi();

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  if (SD.begin(pinCS)) {
    Serial.println("SD card is ready to use.");
  } else {
    Serial.println("SD card initialization failed");
    return;
  }
}

void loop() {
  readNPKSensor();
  delay(800);
  displayBatteryLevel();
  delay(500);
  if (digitalRead(BUTTON_PIN) == HIGH) {
      uploadToFirestore();
  }else{
     WriteDataToSD(nitrogen, phosphorus, potassium);
  }
}

void readNPKSensor() {
  uint16_t val1 = readSensor(0x001E);
  delay(250);
  uint16_t val2 = readSensor(0x001F);
  delay(250);
  uint16_t val3 = readSensor(0x0020);
  delay(250);

  Serial.print("Nitrogen: ");
  Serial.print(val1);
  Serial.println(" mg/kg");
  Serial.print("Phosphorus: ");
  Serial.print(val2);
  Serial.println(" mg/kg");
  Serial.print("Potassium: ");
  Serial.print(val3);
  Serial.println(" mg/kg");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("N: ");
  lcd.print(val1);
  lcd.print(" mg/kg");

  lcd.setCursor(0, 1);
  lcd.print("P: ");
  lcd.print(val2);
  lcd.print(" mg/kg");

  lcd.setCursor(8, 1);
  lcd.print("K: ");
  lcd.print(val3);
  lcd.print(" mg/kg");
}

uint16_t readSensor(uint16_t address) {
  uint8_t result;
  uint16_t data;

  result = node.readHoldingRegisters(address, 1);
  if (result == node.ku8MBSuccess) {
    data = node.getResponseBuffer(0);
    return data;
  } else {
    Serial.print("Failed to read from sensor, error code: ");
    Serial.println(result);
    return 0;
  }
}


//void buzzeron(int delaytime) {
//  tone(pin, 1500);
 // delay(delaytime);
 // noTone(pin);
 // delay(delaytime);
//}

void buzzeron(int delaytime) {
  digitalWrite(pin, HIGH); 
  delay(delaytime); 
  digitalWrite(pin, LOW); 
  delay(delaytime); 
}

void displayBatteryLevel() {
    int sensorValue = analogRead(VOLTAGE_SENSOR_PIN);
    float voltage = sensorValue * (3.3 / 4095.0) * 5;
    
    Serial.print("Analog Read: "); Serial.println(sensorValue);
    Serial.print("Voltage: "); Serial.println(voltage);

    int batteryLevel = map(voltage * 100, 0, 1200, 0, 100);
    if ((batteryLevel > 67) && (batteryLevel < 82)) batteryLevel= 75;
    if ((batteryLevel > 80 )&&  (batteryLevel <100)){
       batteryLevel= 90;
      


    } 
    if (batteryLevel > 100) batteryLevel = 100;
    if (batteryLevel < 0) batteryLevel = 0;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Battery:");
    lcd.print(batteryLevel);
    lcd.print("%");

    Serial.print("Battery Level:");
    Serial.print(batteryLevel);
    Serial.println("%");

    if (batteryLevel < 50) {
        lcd.setCursor(0, 1);
        lcd.print("Battery Low");
        buzzeron(700);
        buzzeron(700);
    }
}

void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting");
  lcd.setCursor(0,1);
  lcd.print("To WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connecting...");
  }

  Serial.println("WiFi connected.");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi connected");
}

void uploadToFirestore() {
   nitrogen = readSensor(0x001E);
   phosphorus = readSensor(0x001F);
   potassium = readSensor(0x0020);

  // Update NTP time
  timeClient.update();

  FirebaseJson content;
  content.set("fields/nitrogen/integerValue", String(nitrogen));
  content.set("fields/phosphorus/integerValue", String(phosphorus));
  content.set("fields/potassium/integerValue", String(potassium));
  content.set("fields/timestamp/integerValue", String(timeClient.getEpochTime()));

  // Generate a unique document ID using NTP time
  String documentId = String(timeClient.getEpochTime());
  String documentPath = "NPK/" + documentId;

  if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw())) {
    Serial.println("New document created in Firestore successfully.");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Data Uploaded ");
    lcd.setCursor(1,1);
    lcd.print("Successfully.");
    Serial.println("Document ID: " + documentId);
  } else {
    Serial.println("Failed to create new document in Firestore.");
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Data upload ");
    lcd.setCursor(1,0);
    lcd.print("Failed");

    Serial.println("Reason: " + fbdo.errorReason());
    logDataToSD(nitrogen, phosphorus, potassium);
  }
}
void logDataToSD(uint16_t nitrogen, uint16_t phosphorus, uint16_t potassium) {
  myFile = SD.open("/npk_data.txt", FILE_APPEND);
  if (myFile) {
    Serial.println("Writing to file...");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Wifi");
    lcd.setCursor(1,1);
    lcd.print("Disconnected.");
    delay(500);
    lcd.clear();
    lcd.print("Writing to SD ...");
    myFile.print("Nitrogen: ");
    myFile.print(nitrogen);
    myFile.print(" mg/kg, Phosphorus: ");
    myFile.print(phosphorus);
    myFile.print(" mg/kg, Potassium: ");
    myFile.print(potassium);
    myFile.println(" mg/kg");
    myFile.close();
    Serial.println("Done.");
    lcd.setCursor(1,1);
    lcd.print("Done.");
    
  } else {
    Serial.println("Error opening npk_data.txt");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Data writting");
    lcd.setCursor(1, 1);
    lcd.print("Failed.");
  }
}
void WriteDataToSD(uint16_t nitrogen, uint16_t phosphorus, uint16_t potassium) {
  myFile = SD.open("/npk_data.txt", FILE_APPEND);
  if (myFile) {
    Serial.println("Writing to file...");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Writing to SD ...");
    delay(500);
    myFile.print("Nitrogen: ");
    myFile.print(nitrogen);
    myFile.print(" mg/kg, Phosphorus: ");
    myFile.print(phosphorus);
    myFile.print(" mg/kg, Potassium: ");
    myFile.print(potassium);
    myFile.println(" mg/kg");
    myFile.close();
    Serial.println("Done.");
    lcd.setCursor(1,1);
    lcd.print("Done.");

    
  } else {
    Serial.println("Error opening npk_data.txt");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Data writting");
    lcd.setCursor(1, 1);
    lcd.print("Failed.");
  }
}