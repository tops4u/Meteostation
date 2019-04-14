/* ===================================================================================================================
   Der folgende Code wurde fuer einen Arduino Pro Mini mit 3.3V geschrieben. Mit einigen Anpassungen
   sollte der Code auch auf anderen Arduino Boards laufen.
   !! ACHTUNG : Versuche den Code (Interrupthandling des Windmessers im Zusammenspiel mit dem Stromsparmodus)
                auf einem Arduino MEGA 2560 zu testen schlugen fehl. Ich vermute ein anderes IRQ Handling auf dem MEGA

   Hilfreiche Links zum Thema Stromsparmodus :
   - http://donalmorrissey.blogspot.ch/2010/04/putting-arduino-diecimila-to-sleep-part.html
   - http://forum.arduino.cc/index.php?topic=199576.0

   Der vorliegende Code ist Open Source nach GPL

   Hardware Voraussetzungen / verwendete Hardware:
   - ELTAKO Windmesser WS
   - Arduino Mini Pro 3.3V -> http://www.arduino.cc/en/Main/ArduinoBoardProMini
   - NRF24L01 Funkmodul
   - SM5611 Luftdruck Modul
   - OPT 3001 Lichtsensor
   - VEML 6075 UV Index
   - SI 7021 Hygro- und Thermometer
   - tmp102 Thermometer
   ==================================================================================================================*/

// Library Includes
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <printf.h>
#include <Wire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <MS5611.h>
#include <SparkFunTMP102.h>
#include <SI7021.h>
#include <VEML6075.h>
#include "Adafruit_SI1145.h"


#include <ClosedCube_OPT3001.h>


// IO Pin Konfiguration
#define BATT_PIN 0
#define WDIR_PIN 1
#define WSPEED_PIN 2
#define RAIN_PIN 3
#define ANALOG_ENABLE 4
#define I2C_ENABLE 5
#define NRF_CE 8
#define NRF_CSN 9

// Programm Konstanten
// Serial Uebertragungsrate
#define BAUD 9600

// Hoehe der Mess-Station damit der Druck auf Meereshoehe normalisiert werden kann.
#define STATION_ALTITUDE 462

#define SI7021_ADDRESS 0x40
#define OPT3001_ADDRESS 0x44
#define TMP102_ADDRESS 0x48
#define VEML6075_ADDRESS 0x10
#define MS5611_ADDRESS 0x76
#define SI1145_ADDRESS 0x40

struct SIData {
  float temperature;
  uint8_t humidity;
};

// RFData ist 31 Bytes - Lucky, da der NRF Frame Size = 32 Byte ist.
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

const byte nrfAddress[][6] = {"METEO"};

// Ab welcher Windboen Geschwindigkeit soll eine Funkuebertragung stattfinden?
#define MAX_WIND_GUST 35

// Alle n Sekunden wird eine regulaere AVG Wind Messung uebertragen, zusammen mit allen anderen Werten
#define MEASUREMENT_INTERVALL 300

// Wieviele Regulaere Datenuebertragungen muessen zwischen zwei Boen Windwarnungen vergehen. WIND_GUST_HOLDOFF x MEASUREMENT_INTERVALL
#define WIND_GUST_HOLDOFF 3

// Timeout Multiplikator fuer die Watchdog Timeouts. Muss zu den verwendeten Watchdog Einstellungen passen!
#define WDT_TIMEOUT 4000

// AD-Converter Controll Register Zwischenspeicher, damit der ADC waehrend dem Sleep deaktiviert werden kann.
byte adcsra;

// Zaehler ab wann wieder Windboeen Warnungen uebertragen werden.
unsigned long nextWindGustTx = 0;

// Aktuelle Uebertragungsnummer
unsigned long txNumber = 0;

// Variablen zur Windmessung. Werte welche innerhalb der InterruptServiceRoutinen (ISR) veraendert werden muessen volatile markiert werden.
// Zaehler der Windsensor Impulse waehrend dem regulaeren Messintervall
volatile uint32_t windCount = 0L;

// Variable zur Regenmessung, welche innerhalb der ISR verändert wird.
volatile uint16_t rainCount = 0;

// Zaehler der Windsensor Impulse waehrend dem Watchdog -> Boen
volatile uint32_t windGustCount = 0L;

// Marker welcher Interrupt ausgeloest hat. TRUE = Watchdog / FALSE = WindSensor/Regen
volatile boolean WDT_INT = false;
volatile boolean WIND_INT = false;
volatile boolean RAIN_INT = false;

// Globale Variable fuer die Max Windboen waehrend der Messperiode
float maxWind;
boolean rainState;

// Anzahl durchgefuehrter Boen Messungen
uint16_t windCounter;

uint16_t retransmissionCounter = 0;

// NRF24 Variablen zur Funkuebertragung
// NRF24 Modul benoetigt den SPI Bus (Pins auf Mini Pro 11-MOSI / 12-MISO / 13-SCK) plus 2 Pins.
RF24 radio(8, 9);

// I2C Devices
SI7021 si7021;
TMP102 tmp102(TMP102_ADDRESS);
MS5611 ms5611;
ClosedCube_OPT3001 opt3001;
VEML6075 veml6075 = VEML6075();
Adafruit_SI1145 si1145 = Adafruit_SI1145();

// I2C Availabaility
boolean si7021_avail = false;
boolean tmp102_avail = false;
boolean ms5611_avail = false;
boolean opt3001_avail = false;
boolean veml6075_avail = false;
boolean si1145_avail = false;

void setup()
{
  Serial.begin(BAUD);
  // I/O Pin Config
  fdevopen( &my_putc, 0);
  pinMode(I2C_ENABLE, OUTPUT);
  pinMode(ANALOG_ENABLE, OUTPUT);
  pinMode(WSPEED_PIN, INPUT_PULLUP);
  pinMode(RAIN_PIN, INPUT_PULLUP);
  for (int i = 7; i < 13; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, false);
  }
  if (Serial)
    Serial.println("Setup V3.0 - 2019_01_02");
  windCount = 0;
  windGustCount = 0;
  windCounter = 1;
  WDT_INT = false;
  analogReference(EXTERNAL);
  adcsra = ADCSRA;
  initRFCom();
  delay(500);

  powerUp();
  delay(500);

  initI2C();
  delay(500);

  I2C_Scanner();
  initSensors();
  ms5611_avail = true;
  setup_watchdog();
  if (Serial)
    Serial.println("Watchdog - Setup done");
  attachIRQs();
  powerSaver();
  powerDown();
}

// Hack to enable printf Funktion fuer PrintDetails
int my_putc( char c, FILE *t) {
  Serial.write( c );
}

void initI2C() {
  Wire.begin();
  Wire.setClock(10000);
}

void powerSaver() {
  DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins
  DIDR1 = (1 << AIN1D) | (1 << AIN0D); //Disable digital input buffer on AIN1/0
}

// Watchdog Vorbereiten und zu verwendenden Timeout einstellen.
void setup_watchdog(void) {
  MCUSR = 0;
  MCUSR &= ~bit(WDRF);
  WDTCSR |= bit(WDCE) | bit(WDE);
  WDTCSR = bit (WDIE) | bit(WDP3) ; /* Bit 3 = 4.0 Sekunden */
  wdt_reset();
}

void I2C_Scanner() {
  if (Serial)
    Serial.println("Scaning for I2C Devices");
  for (int address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    int error = Wire.endTransmission();
    if (error == 0)
    {
      String hexAddress = "0";
      if (address < 16)
        hexAddress = hexAddress.concat(String(address, HEX));
      else
        hexAddress = String(address, HEX);
      // !! ADDRESS NEEDS TO BE IN DECIMAL !!
      switch (address) {
        case 68: {
            opt3001_avail = true;
            if (Serial)
              Serial.println("OPT 3001 Light Sensor Found");
            break;
          }
        case 72: {
            tmp102_avail = true;
            if (Serial)
              Serial.println("TMP 102 Temperature Sensor Found");
            break;
          }
        case 118: {
            ms5611_avail = true;
            if (Serial)
              Serial.println("MEAS 5611 Pressure Sensor Found");
            break;
          }
        case 64: {
            si7021_avail = true;
            if (Serial)
              Serial.println("SI7021 Temperature/Humidity Sensor Found");
            break;
          }
        case 16: {
            veml6075_avail = true;
            if (Serial)
              Serial.println("VEML6075 UV Index Sensor Found");
            break;
          }
        case 96: {
            si1145_avail = true;
            if (Serial)
              Serial.println("SI1145 UV Index Sensor Found");
            break;
          }
        default: {
            if (Serial) {
              Serial.print("!!UNKNOWN I2C DEVICE FOUND! Device address = ");
              Serial.println(hexAddress);
            }
          }
          break;
      }
    }
  }
}

boolean initopt3001() {
  opt3001.begin(OPT3001_ADDRESS);
  OPT3001_Config newConfig;
  newConfig.RangeNumber = B1100;
  newConfig.ConvertionTime = B0;
  newConfig.Latch = B1;
  newConfig.ModeOfConversionOperation = B11;
  OPT3001_ErrorCode errorConfig = opt3001.writeConfig(newConfig);
  return errorConfig != NO_ERROR;
}

float getLight() {
  float result = opt3001.readResult().lux;
  if (Serial) {
    Serial.print("Lux:");
    Serial.println(result);
  }
}

boolean initUVIndex() {
  return veml6075.begin();
}

float getUVIndex() {
  if (veml6075_avail) {
    veml6075.poll();
    if (Serial)  {
      Serial.print("UV-Index:");
      Serial.println(veml6075.getUVIndex());
    }
    return veml6075.getUVIndex();
  } else if (si1145_avail) {
    float UVindex = si1145.readUV();
    if (Serial) {
      Serial.print("UV_Index:");
      Serial.println(UVindex / 100.0);
    }
    return UVindex / 100.0;
  }
  else return 0.0f;
}

boolean initBarometer() {
  ms5611.begin();
  Serial.println("MS5611 Init");
  return true;
}

double getPressure(double temp) {
  double realPressure = ms5611.getPressure();
  double pressure = realPressure * pow(1-(0.0065 * STATION_ALTITUDE / (temp + 273.15 + 0.0065 * STATION_ALTITUDE)), -5.257) / 100.0;
  if (Serial) {
    Serial.print("Absolute Pressure (QFE)=");
    Serial.print(realPressure / 100.0);
    Serial.print(" / Sealevel Pressure (QNH)=");
    Serial.println(pressure);
  }
  return pressure;
}

double getInternalTemp() {
  double result = ms5611.getTemperature() / 100.0;
  if (Serial) {
    Serial.print("Internal Temp=");
    Serial.println(result);
  }
  return result;
}

boolean initTemperature() {
  tmp102.begin();
  return true;
}

float getTemperature() {
  tmp102.setConversionRate(2);
  tmp102.setExtendedMode(0);
  tmp102.wakeup();
  float result = tmp102.readTempC();
  tmp102.sleep();
  if (Serial) {
    Serial.print("Temperature Read:");
    Serial.println(result);
  }
  return result;
}

boolean initHumidity() {
  si7021.begin();
  uint16_t deviceId = si7021.getDeviceId();
  if (deviceId == 20 || deviceId == 21)
    return true;
  else
    return false;
}

SIData getSiData() {
  SIData result;
  result.temperature = si7021.getCelsiusHundredths() / 100.0;
  result.humidity = si7021.getHumidityPercent();
  if (Serial) {
    Serial.print("Humidity:");
    Serial.println(result.humidity);
    Serial.print("Temperature:");
    Serial.println(result.temperature);
  }
  return result;
}

boolean initUV2() {
  return si1145.begin();
}

void initSensors() {
  if (Serial)
    Serial.println("Initializing Sensors");
  if (opt3001_avail && initopt3001())
    if (Serial)
      Serial.println("Light Sensor Initialized.");
  if (veml6075_avail && initUVIndex())
    if (Serial)
      Serial.println("UV-Index Sensor Initialized.");
  if (ms5611_avail && initBarometer())
    if (Serial)
      Serial.println("Barometer Initialized.");
  if (tmp102_avail && initTemperature())
    if (Serial)
      Serial.println("Temperature Sensor Initialized.");
  if (si7021_avail && initHumidity())
    if (Serial)
      Serial.println("Humidity/Temperature Sensor Initialized.");
  if (si1145_avail && initUV2())
    if (Serial)
      Serial.println("UV-Index Sensor Initialized");
}

// This is done via a 1.8V Zener Diode
float readVoltage() {
  int16_t in = analogRead(0);
  float result = map(in, 856, 575, 180, 330) / 100.0;
  if (Serial) {
    Serial.print("Voltage Read V=");
    Serial.println(result);
  }
  return result;
}

/* Interrupt Service Routinen. Hier wird der Schlafmodus des Arduino unterbrochen wenn ein solcher ausgeloest wird */

// Interrupt Service Routine fuer den Watchdog.
ISR(WDT_vect)
{
  WDT_INT = true;
}

// Interrupt Service Routine fuer den Windmesser
void windUp() {
  windCount ++;
  windGustCount++;
  WIND_INT = true;
  WDT_INT = false;
}

void rainUp() {
  rainCount ++;
}

int8_t getWindDir() {
  // Wind Direction Calculation goes here
  int in = analogRead(1);
  Serial.print("Wind-Dir RAW=");
  Serial.println(in);
  if (checkDir(in, 80))
    return 13;
  else if (checkDir(in, 101))
    return 11;
  else if (checkDir(in, 111))
    return 12;
  else if (checkDir(in, 151))
    return 15;
  else if (checkDir(in, 218))
    return 14;
  else if (checkDir(in, 284))
    return 1;
  else if (checkDir(in, 331))
    return 0;
  else if (checkDir(in, 456))
    return 9;
  else if (checkDir(in, 512))
    return 10;
  else if (checkDir(in, 648))
    return 3;
  else if (checkDir(in, 677))
    return 2;
  else if (checkDir(in, 745))
    return 7;
  else if (checkDir(in, 820))
    return 8;
  else if (checkDir(in, 857))
    return 5;
  else if (checkDir(in, 909))
    return 6;
  else if (checkDir(in, 959))
    return 4;
  else
    return 0;
}

boolean checkDir(int dir, int soll) {
  if (abs(soll - dir) < 5)
    return true;
  else
    return false;
}

// Berechnung für MI-SOL WH-SP-RG Regenmengenmesser. 1l Wasser ergeben ca 400 Impulse = 1 Puls => 2.5ml. Die Fläche beträgt 5 x 11cm
// Rückgabe erfolgt in ml/m^2 während dem Messintervall.
uint16_t calcRain(uint16_t count) {
  Serial.print("Rain Counts=");
  Serial.println(count);
  uint16_t rain =  count * 454 / 2;  // 1 Count = 0.0025 ml bezogen auf den Messtrichter. 1 Count = 454ml/m^2 oder 0.454mm
  if (Serial) {
    Serial.print("Berechnete Regenmenge (ml/m^2): ");
    Serial.println(rain);
  }
  return rain;
}

// For ELTAKO Windmesser
// Windgeschwindigkeit aus der Anzahl Impulse und der vergangenen Zeit berechnen
float calcWind(uint32_t count, uint32_t time) {
  // Bei sehr tiefen Geschwindigkeiten ist die Formel ungenau, deshalb muss im Schnitt mindestens
  // 1u/sec erfasst worden sein. Die Erfassunglimite liegt somit bei 3km/h
  if (count <= time / 2) {
    return 0.0;
  }
  float windSpeedAlt = (count / time + 2) / ( 3.0) * 3.6;
  if (Serial) {
    Serial.print("Berchnete Windgeschwindigkeit (kmh): ");
    Serial.println(windSpeedAlt);
  }
  return windSpeedAlt;
}

// Stromspar Modus verwenden
void enterSleep(void) {
  WIND_INT = false;
  // Sleep Modus einstellen. Max Stromersparnis bei POWER DOWN
  if (WDT_INT)
    setup_watchdog(); // Reset Watchdog.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts();
  attachIRQs();
  sleep_enable();
  MCUCR = bit (BODS) | bit (BODSE);  // turn on brown-out enable select
  MCUCR = bit (BODS);
  interrupts();
  sleep_cpu();
  // Nach dem Aufwachen geht es hier weiter.
  // Als erstes weitere Interrupts verhindern und Sleep Mode deaktivieren
  noInterrupts();
  sleep_disable();
  detachIRQs();
  interrupts();
}

void attachIRQs() {
  attachInterrupt(digitalPinToInterrupt(WSPEED_PIN), windUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainUp, CHANGE);
}

void detachIRQs() {
  detachInterrupt(digitalPinToInterrupt(WSPEED_PIN));
  detachInterrupt(digitalPinToInterrupt(RAIN_PIN));

}

/* Ende der ISR Methoden */
void initRFCom() {
  radio.begin();
  radio.powerUp();
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(15, 15);
  radio.stopListening();
  radio.openWritingPipe(nrfAddress[0]);
  if (Serial) {
    printf_begin();
    radio.printDetails();
    Serial.println("Funkuebertragung Initialisiert");
  }
}

// Uebertragen der Daten
void transmit(RFData data) {
  if (Serial) {
    Serial.print("Start NRF Transmission with Size of:");
    Serial.print(sizeof(data));
  }
  if (radio.write(&data, sizeof(data))) {
    retransmissionCounter = 0;
    if (Serial)
      Serial.println("Byte. Transmission ok");
  } else {
    retransmissionCounter ++;
    if (Serial)
      Serial.println("Byte. Transmission failed");
  }
}

// Wind Gust ist nur eine Warnung bei > MAX_WIND_GUST (default = 25kmh)
void transmitGust() {
  RFData rfData;
  rfData.windSpeedMax = maxWind * 10;
  rfData.voltage = 0;
  radio.powerUp();
  radio.write(&rfData, sizeof(rfData));
  radio.txStandBy();
  radio.powerDown();
}

void collectAndTrasmit() {
  if (Serial)
    Serial.println("Collecting all Data...");
  long start = millis();
  RFData rfData;
  powerUp();
  initSensors();
  // Alle Messwerte bestimmen
  rfData.voltage = readVoltage() * 10;
  rfData.rain = calcRain(rainCount);
  rfData.windSpeed = calcWind(windCount, windCounter * (WDT_TIMEOUT / 1000)) * 10;
  rfData.windSpeedMax = maxWind * 10;
  rfData.windDir = getWindDir();
  double temp = 15.0;
  if (si7021_avail) {
    SIData siData = getSiData();
    rfData.externalTemp = siData.temperature * 10;
    rfData.externalHum = siData.humidity;
    temp = siData.temperature;
  }
  if (ms5611_avail) {
    rfData.internalTemp = getInternalTemp() * 10;
    rfData.sealevelPres = getPressure(temp) * 10;
  }
  if (opt3001_avail)
    rfData.lux = getLight();
  if (veml6075_avail || si1145_avail)
    rfData.uvIndex = getUVIndex();
  if (tmp102_avail)
    rfData.externalTemp2 = getTemperature() * 10;
  powerDown();
  rfData.retransmissionCounter = retransmissionCounter;
  // Messwerte Uebertragen
  if (Serial) {
    Serial.println(rfData.internalTemp);
    Serial.println(rfData.sealevelPres);
    Serial.println(rfData.voltage);
    Serial.println(rfData.lux);
    Serial.println(rfData.uvIndex);
    Serial.println(rfData.rain);
    Serial.println(rfData.windDir);
    Serial.println(rfData.externalTemp);
    Serial.println(rfData.externalTemp2);
    Serial.println(rfData.externalHum);
    Serial.println(rfData.windSpeed);
    Serial.println(rfData.windSpeedMax);
    Serial.println(rfData.retransmissionCounter);
  }
  radio.powerUp();
  if (radio.write(&rfData, sizeof(rfData))) {
    if (Serial)
      Serial.println("Transmission ok");
  } else {
    if (Serial)
      Serial.println("Transmission failed");
  }
  radio.txStandBy();
  radio.powerDown();
  txNumber++;
  // Alle Messvariablen zuruecksetzen
  windCounter = 0L;
  windGustCount = 0L;
  windCount = 1;
  maxWind = 0L;
  rainCount = 0;
  if (Serial) {
    Serial.print("Data Collection & Transmission took :");
    Serial.print(millis() - start);
    Serial.println("ms");
  }
}

// Strom zu den Elementen leiten welche beim Transmit ausgelesen werden resp Baugruppen des Arduino mit Strom versorgen (ADC)
void powerUp() {
  ADCSRA = adcsra;
  digitalWrite(I2C_ENABLE, HIGH);
  digitalWrite(ANALOG_ENABLE, HIGH);
  power_usart0_enable();
  Serial.begin(BAUD);
  if (Serial) {
    Serial.println("Powering Up Auxillaries");
  }
  //power_spi_enable();
  power_twi_enable();
  initI2C();
  //power_timer0_enable();
  //power_timer1_enable();
  //power_timer2_enable();
}

// Zur normalen Messung via IRQ nicht bentigte Baugruppen und Elemente ausschalten
void powerDown() {
  if (Serial) {
    Serial.println("Powering Down Auxillaries");
  }
  digitalWrite(ANALOG_ENABLE, LOW);
  ADCSRA = 0;
  power_twi_disable();
  // Set I2C into a defined state
  TWCR = 0;
  digitalWrite(I2C_ENABLE, LOW);
  pinMode(SDA, OUTPUT);
  pinMode(SCL, OUTPUT);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  //power_spi_disable();
  //power_timer0_disable();
  //power_timer1_disable();
  //power_timer2_disable();
  // Serial Comm flushen, da es sonst zu Stoerungen auf der Seriellen Schnittstelle kommt
  Serial.flush();
  Serial.end();
  power_usart0_disable();
}

// Main Loop
void loop()
{
  if (WDT_INT) { // Falls dies ein Watchdog Timeout war
    // Boen Wind waehrend der Messperiode bestimmen.
    float tempMaxWind = calcWind(windGustCount, WDT_TIMEOUT / 1000);
    // Falls neue Boenspitze in dieser Messperiode den Max Wert neu setzen
    if (tempMaxWind > maxWind)
      maxWind = tempMaxWind;
    // Boenzaehler reset
    windGustCount = 0;
    windCounter ++;
    // Falls der regulaere Messintervall abgelaufen ist
    if (MEASUREMENT_INTERVALL / (WDT_TIMEOUT / 1000) <= windCounter) {
      // Alle Werte auslesen und die Daten uebertragen
      collectAndTrasmit();
    }
    // Wenn es eine Windboe gab welche ueber der Alarm Schwelle lag und es schon laenger keine solche Meldung mehr gab
    else if (maxWind > MAX_WIND_GUST && nextWindGustTx < txNumber) {
      // Bei der Alarm Meldung sind alle Werte ausser dem Boenwert = 0
      transmitGust();
      // Die naechste Boen Alarmmeldung darf erst nach WIND_GUST_HOLDOFF regulaeren Uebermittlungen stattfinden
      nextWindGustTx = txNumber +  WIND_GUST_HOLDOFF + 1;
    }
  } if (WIND_INT) {
    // Bei einem Windmesser Interrupt noch etwas warten, damit Schalterprellen nicht gleich wieder einen IRQ ausloesst.
    // Dies passt fuer meinen ELTAKO Windmesser WS, andere Reedkontakt basierte Windmesser bentigen ggf andere Werte
    delayMicroseconds(5);
    // Bei einem Auslöser durch den Regensensor diesen erhöhen.
  }
  // Verarbeitung Fertig, CPU wieder schlafen legen.
  enterSleep();
}


