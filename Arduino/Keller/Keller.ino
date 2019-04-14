//Versionsbezeichner "homeduino_21.ino / Stand: 2014.09.13";
// fuer Arduino Mega 2560 mit Arduino 1.5.6r2
//Verfasser: Eugen Stall
// diese Software erlaubt die Steuerung der Pinfunktionen mit einfachen Browserbefehlen
//verwendete Quellen fuer libraires und Programme siehe Beschreibung und Erläuterungen im Quelltext
//die aktuellste Version ist immer hier:
//http://www.stall.biz/?project=homeduino-der-universelle-lanwlan-arduino-fur-die-hausautomation

#include <Ethernet.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <dht.h>
#include <Time.h>
#include <TimeAlarms.h>

////////////////////////////////////////////////////////////////////////
//Netzwerk-Konfiguration muss individuell hier eingegeben werden
const byte ip[] = {192, 168, 85, 10}; //das ist die IP des Arduino //<<user-eingabe<<
const byte mac[] = { 0xDE, 0xAF, 0xEE, 0xEF, 0xEE, 0xFE }; //nur 1x im Netz //<<user-eingabe<<
const byte ioBroker[] = {192, 168, 85, 22 }; //das ist die IP der CCU //<<user-eingabe<<
const byte gateway[] = { 192, 168, 85, 1 }; // Gateway
const byte dnsserver[] = { 1, 0, 0, 1 }; // DNS Server
const byte subnet[] = { 255, 255, 255, 0 }; // Subnet
const byte nrfAddress[][6] = {"METEO"};
String hm_systemvariable = "arduino_1_";
int32_t WetterstationTXNr = 0;

int RETRY = 5; // Anzahl versuche etwas an die CCU zu senden.
EthernetClient client;
dht DHT;

RF24 radio(7, 6);

// Variablen fuer die Strommessung
volatile int sumPower;;
volatile float maxPower;
volatile long milli;
volatile long diff;
volatile byte index;
volatile uint16_t maxPowerAVG[5];
AlarmId power;

struct RFData {
  int16_t internalTemp;
  uint16_t sealevelPres;
  uint16_t voltage;
  uint32_t lux;
  uint8_t uvIndex;
  uint16_t rain;
  int16_t windDir;
  int16_t externalTemp;
  int16_t externalTemp2;
  uint8_t externalHum;
  uint16_t windSpeed;
  uint16_t windSpeedMax;
  uint8_t retransmissionCounter;
};

unsigned const long delta_time = 3600000; // jede Stunde werden alle Inputs aktualisiert
unsigned const long delta_tx = 500;  //in ms, minimaler Abstand der Telegramme an die CCU
unsigned long next_tx = 0;
boolean maxTimeExpired = false;

float const deltaT = 0.2;
float const deltaH = 1;

String I;


int DHT_PINS[] = {9};
int OneWire_Pin = 8;
float tempC_DHT[1];
float hum_DHT[1];
DeviceAddress TempSensor[] = //Adress-Array definieren
{
  {
    0x28, 0xFF, 0xC0, 0x96, 0x3E, 0x04, 0x00, 0x93
  }
  , //<<Heizung / Boiler<<
  {
    0x28, 0xFF, 0x34, 0x5E, 0x3D, 0x04, 0x00, 0xEB
  }
  , //<<Heizung / Boiler<<
  {
    0x28, 0xFF, 0xE3, 0x99, 0x3C, 0x04, 0x00, 0xF0
  }
};
float tempC_DS18[3];
boolean radioReceived;

void setup()
{
  Serial.begin(57600); //Pins 10,11,12 & 13 fuer ethernet shield verwendet
  // Fix fuer fehlenden printf
  delay(2000);
  fdevopen( &my_putc, 0);
  Serial.println("Init");
  setTime(1, 1, 1, 1, 1, 1); // set any dummy Time... (01:01:01 1.1.2001)
  Ethernet.begin(mac, ip, dnsserver, gateway, subnet);
  delay(1000);
  int result = client.connect(ioBroker, 8087);
  IPAddress myIp = Ethernet.localIP();
  Serial.print("LocalIP:");
  Serial.println(Ethernet.localIP());
  char myIpString[24];  //IP auslesen
  sprintf(myIpString, "%d.%d.%d.%d", myIp[0], myIp[1], myIp[2], myIp[3]);
  I = myIpString;
  // Update der IP in Homematic, dann sieht man am Timestamp die Startzeit des Arduino
  client.println("GET /set/hm-rega.0.11614?value=" + I);
  client.println();
  delay(1000);
  Serial.println("Starting Radio Setup");
  radio.begin();
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(15, 15);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(0, 0xF0F0F0F0D2LL);
  radio.openReadingPipe(1, nrfAddress[0]);
  radio.startListening();
  radio.printDetails();
  Alarm.timerRepeat(60 * 60, UpdateAll); // Ein volles Update alle 60 min.
  power = Alarm.timerOnce(60 * 5, sendPower); // Alle 5 Min Strom Updates schicken.
  index = 0;
  radioReceived = false;
  //  attachInterrupt(digitalPinToInterrupt(3), checkRadio, LOW);
  // client.println();
  // Stromzaehler Blinken registrieren
  attachInterrupt(digitalPinToInterrupt(2), blink, FALLING);
}

// Hack to enable printf Funktion fuer PrintDetails
int my_putc( char c, FILE *t) {
  Serial.write( c );
}

void loop()
{
  // Hier sind keine Digitalen Trigger vorhanden, also koennen die Eingaenge einfach alle 10 Secs abgefragt werden.
  checkDS18();
  checkDHT22();
  maxTimeExpired = false;
  for (int i = 0; i < 600; i++) {
    if (radio.available()) {
      checkRadio();
    }
    Alarm.delay(100);
  }
}

// ISR fuer Zaehlerblinken
void blink() {
  diff = millis() - milli;
  milli = millis();
  sumPower++;
  float current = (60.0 * 60.0 * 1000.0) / diff;
  maxPowerAVG[index] = current;
  index ++;
  // Averaging ueber 5 Messwerte. Dabei den kleinsten und den groessten verwerfen.
  if (index > 4) {
    uint16_t mini = 65535;
    uint16_t maxi = 0;
    current = 0.0;
    for (int i = 0; i < 5; i++) {
      if (maxPowerAVG[i] > maxi)
        maxi = maxPowerAVG[i];
      if (maxPowerAVG[i] < mini)
        mini = maxPowerAVG[i];
      current = current + maxPowerAVG[i];
    }
    current = (current - (mini + maxi)) / 3.0;
    if (current > maxPower)
      maxPower = current;
    index = 0;
  }
  if (Serial) {
    Serial.print("Aktuell=");
    Serial.print(current);
    Serial.print("Wh / Summe=");
    Serial.print(sumPower);
    Serial.println("W");
  }
}

void UpdateAll() {
  maxTimeExpired = true;
}

// Ziemliches Chaos... Try & Error fuer Connection Wieder-/Aufbau
boolean set_sysvar(String befehl) {
  detachInterrupt(digitalPinToInterrupt(2));
  for (int j = 0; j < RETRY; j++) {
    Ethernet.begin(mac, ip);
    while (millis() < next_tx) {
      delay(100);
    } //warten bis time > next_tx oder timeout
    int result = 1;
    next_tx = millis() + delta_tx;
    if (!client.connected()) {
      if (Serial) {
        Serial.print("Stopping closed Connection.");
        Serial.print("LocalIP:");
        Serial.print(Ethernet.localIP());
      }
      client.flush();
      client.stop();
      result = client.connect(ioBroker, 8087);
    }
    if (result == 1 || client.connected())
    {
      client.println(befehl);
      client.println();
      if (Serial)
        Serial.println(". Disconnecting...");
      client.stop();
      attachInterrupt(digitalPinToInterrupt(2), blink, FALLING);
      return true;
    } else {
      if (Serial) {
        Serial.print("Problem Connecting. Got Error Code:");
        Serial.println(result);
      }
    }
  }
  attachInterrupt(digitalPinToInterrupt(2), blink, FALLING);
  return false;
}

String constructCommand(String name, String value) {
  return "GET /set/" + name + "?value=" + value + "";
}

void checkRadio() {
  if (Serial)
    Serial.println("Reading Data!");
    WetterstationTXNr ++;
    readNewNRF();
}

void readNewNRF() {
  RFData rfData;
  radio.read(&rfData, sizeof(rfData));
  float maxWind = float(rfData.windSpeedMax) / 10.0;
  float avgWind = float(rfData.windSpeed) / 10.0;
  float pressure = float(rfData.sealevelPres) / 10.0;
  float battery = float(rfData.voltage) / 10.0;
  float ext_temp = float(rfData.externalTemp) / 10.0;
  uint8_t ext_hum = uint8_t(rfData.externalHum);
  float rain = float(rfData.rain) / 1000.0;
  float int_temp = float(rfData.internalTemp) / 10.0;
  uint8_t windDir = uint8_t(rfData.windDir);
  uint32_t lux = rfData.lux;
  uint8_t uvIndex = rfData.uvIndex;
  float ext_temp2 = float(rfData.externalTemp2) / 10.0;
  uint8_t retransCounter = rfData.retransmissionCounter;
  if (Serial) {
    Serial.print("Maximaler Wind : ");
    Serial.print(maxWind);
    Serial.println(" km/h");
    Serial.print("Durchschnittlicher Wind : ");
    Serial.print(avgWind);
    Serial.println(" km/h");
    Serial.print("Barometer : ");
    Serial.print(pressure);
    Serial.println(" hPa");
    Serial.print("AussenTemp : ");
    Serial.print(ext_temp);
    Serial.println(" °C");
    Serial.print("Aussenfeuchte : ");
    Serial.print(ext_hum);
    Serial.println(" %");
    Serial.print("Regen : ");
    Serial.print(rain);
    Serial.println(" ml/m2");
    Serial.print("Interne Temperatur : ");
    Serial.print(int_temp);
    Serial.println(" °C");
    Serial.print("Windrichtung : ");
    Serial.println(windDir);
  }
  set_sysvar(constructCommand("hm-rega.0.11615", String(maxWind, 1)));
  if (battery > 0.0) {
    set_sysvar(constructCommand("hm-rega.0.11616", String(avgWind, 1)));
    set_sysvar(constructCommand("hm-rega.0.11617", String(pressure, 1)));
    set_sysvar(constructCommand("hm-rega.0.11618", String(battery, 1)));
    set_sysvar(constructCommand("hm-rega.0.14944", String(ext_temp, 1)));
    set_sysvar(constructCommand("hm-rega.0.14945", String(ext_hum, DEC)));
    set_sysvar(constructCommand("hm-rega.0.14948", String(rain, DEC)));
    set_sysvar(constructCommand("hm-rega.0.14946", String(int_temp, 1)));
    float dir;
    switch (windDir) {
      case 0: dir = 0; break;
      case 1: dir = 22.5; break;
      case 2: dir = 45; break;
      case 3: dir = 67.5; break;
      case 4: dir = 90; break;
      case 5: dir = 112.5; break;
      case 6: dir = 135; break;
      case 7: dir = 157.5; break;
      case 8: dir = 180; break;
      case 9: dir = 202.5; break;
      case 10: dir = 225; break;
      case 11: dir = 247.5; break;
      case 12: dir = 270; break;
      case 13: dir = 292.5; break;
      case 14: dir = 315; break;
      case 15: dir = 337.5; break;
      default : Serial.println("EXCEPT : Ungültige Richtung erhalten");
    }
    set_sysvar(constructCommand("hm-rega.0.14947", String(dir, 1)));
    set_sysvar(constructCommand("hm-rega.0.14949", String(WetterstationTXNr, DEC)));
  }
}


void sendPower() {
  set_sysvar(constructCommand("hm-rega.0.13174", String(sumPower * 12)));
  set_sysvar(constructCommand("hm-rega.0.13088", String(sumPower)));
  set_sysvar(constructCommand("hm-rega.0.13089", String(maxPower, 1)));
  sumPower = 0;
  maxPower = 0.0;
  Alarm.free(power);
  power = Alarm.timerOnce(60 * 5, sendPower);
}

void checkDS18() {
  //Setup onewire//
  OneWire oneWire(OneWire_Pin);
  DallasTemperature sensors(&oneWire);
  // Start up the library
  sensors.begin();
  // aufloesung 10 bit fuer alle Sensoren
  for (int i = 0; i < (sizeof(tempC_DS18) / sizeof(float)); i++) {
    sensors.setResolution(TempSensor[i], 10);
  }
  for (int i = 0; i < (sizeof(tempC_DS18) / sizeof(float)); i++) {
    if (Serial) {
      Serial.print("Checking 1-Wire Sensor ");
      Serial.println(i);
    }
    sensors.requestTemperatures();
    float oldTemp = tempC_DS18[i];
    float newTemp = sensors.getTempC(TempSensor[i]);
    if (Serial) {
      Serial.print("Got Temp Reading of :");
      Serial.print(newTemp);
      Serial.println(" deg C");
    }
    int id;
    switch (i) {
      case 0: id = 10185; break;
      case 1: id = 10187; break;
      case 2: id = 10186; break;
    }
    if (abs(oldTemp - newTemp) > deltaT || maxTimeExpired) {
      if (set_sysvar(constructCommand("hm-rega.0." + String(id), String(newTemp)))) {
        tempC_DS18[i] = newTemp;
      }
    }
  }
}

void checkDHT22() {
  for (int i = 0; i < (sizeof(DHT_PINS) / sizeof(int)); i++) {
    int pin = DHT_PINS[i];
    if (Serial) {
      Serial.print("Checking DHT 22 Sensor connected to Pin:");
      Serial.println(pin);
    }
    int chk = DHT.read22(pin);
    float oldTemp = tempC_DHT[i];
    float newTemp = DHT.temperature;
    float oldHum = hum_DHT[i];
    float newHum = DHT.humidity;
    switch (chk)
    {
      case DHTLIB_OK:
        if (Serial) {
          Serial.print("Data from Sensor ok. Temp=");
          Serial.print(newTemp);
          Serial.print("C / Humidity=");
          Serial.print(newHum);
          Serial.println("%");
        }
        if (abs(oldTemp - newTemp) > deltaT || maxTimeExpired) {
          if (set_sysvar(constructCommand("hm-rega.0.5014", String(newTemp)))) {
            tempC_DHT[i] = newTemp;
          }
        }
        if (abs(oldHum - newHum) > deltaH || maxTimeExpired) {
          if (set_sysvar(constructCommand("hm-rega.0.5015", String(newHum)))) {
            hum_DHT[i] = newHum;
          }
        }
        break;
      default:
        if (Serial)
          Serial.println("Error");
        break;
    }
  }
}



