#include <SoftwareSerial.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include "SparkFunBME280.h"
#include <FastLED.h>
#include <avr/sleep.h>     //AVR MCU power management
#include <avr/power.h>     //disable/anable AVR MCU peripheries (Analog Comparator, ADC, USI, Timers/Counters)
#include <avr/wdt.h>       //AVR MCU watchdog timer
#include <avr/io.h>        //includes the apropriate AVR MCU IO definitions
#include <avr/interrupt.h> //manipulation of the interrupt flags

#define LED_DATA_PIN 13
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_OLED_ADDRESS 0x3C
#define BUTTON_PIN 2
#define BATTERY_PIN A6

CRGB leds[1];
BME280 bme280; //Uses I2C address 0x76 in my case
SoftwareSerial mySerial(7, 6); // mySerial (rxPin, txPin, inverse_logic);
SSD1306AsciiWire oled;

byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}; // 0x86 = 134 in dec
unsigned char response[9];

void setup()
{
  digitalWrite (BUTTON_PIN, LOW); //D2 - button
  pinMode (BUTTON_PIN, INPUT);

  attachInterrupt (0, btnPress, RISING );

  Serial.println("  --  Start  --");
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, 1);

  Serial.begin(9600);
  Serial.println("Start");

  mySerial.begin(9600);
  mySerial.write(cmd, 9);

  Wire.begin();
  Wire.setClock(400000L);

  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS);
  oled.setFont(Stang5x7);

  bme280.setI2CAddress(0x76);
  uint8_t mode = 1;
  while (true) {
    if (bme280.beginI2C() == false) {
      Serial.print("bme280 I2C connect failed, sensor mod:");
      Serial.println(bme280.getMode());
      bme280.setMode(mode);
      delay(100);
    }
    else {
      break;
    }
  }
  while (mySerial.available()) mySerial.read();
}


volatile byte display_mod = 1;
volatile bool calibration_flag = false, change_mod_flag = false;
volatile unsigned long press_time = 0;

void loop()
{
  while(press_time); // this needed to increase value in millis() when you hold button
  if(change_mod_flag)
  {
    // тут можно не на долго запретить прерывания чтобы избежать дребезжания контактов кнопки
    if(display_mod < 4) display_mod++;
    else display_mod = 1;
    Serial.println("change_mod_flag");
    change_mod_flag = false;
  }
  if(calibration_flag)
  {
    Serial.println("calibration_flag");
    calibration_flag = false;
  }
  float humidity = bme280.readFloatHumidity();
  float pressure = bme280.readFloatPressure();
  float temp = bme280.readTempC();

  unsigned int co2_ppm = getCO2Value();
  float battery_voltage = getBatteryVoltage();

  int charge_percentage = getChargePercentage(battery_voltage);

  printInfoToSerialPort(co2_ppm, humidity, pressure, temp, battery_voltage, charge_percentage);
  drawInfo(co2_ppm, humidity, pressure, temp, battery_voltage, charge_percentage);
  setLedColor(co2_ppm);
  myWatchdogEnable (0b100001);
}

// процедура обработки прерывания по нажатию кнопки
void btnPress ()
{ 
  noInterrupts ();
  if(press_time == 0) // this will execute when button pressed
  {
    press_time = millis();
    attachInterrupt(0, btnPress, FALLING);
  }
  else  // this will execute when button released
  {
    if(millis() - press_time >= 3000) // long pressing
    { 
      calibration_flag = true; // means that need calibration
    }
    else // short pressing
    {  
      change_mod_flag = true;
    }
    press_time = 0;
    attachInterrupt(0, btnPress, RISING);
  }
  interrupts ();
}

// прерывание сторожевого таймера
ISR (WDT_vect)
{
  return;
}

void myWatchdogEnable (const byte interval)
{
  noInterrupts ();

  MCUSR = 0;                          // сбрасываем различные флаги
  WDTCSR |= 0b00011000;               // устанавливаем WDCE, WDE
  WDTCSR =  0b01000000 | interval;    // устанавливаем WDIE, и соответсвующую задержку
  wdt_reset();

  byte adcsra_save = ADCSRA;
  ADCSRA = 0;  // запрещаем работу АЦП
  power_all_disable ();   // выключаем все модули
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);   // устанавливаем режим сна
  sleep_enable();
  interrupts ();
  sleep_cpu ();            // переходим в сон и ожидаем прерывание

  ADCSRA = adcsra_save;  // останавливаем понижение питания
  power_all_enable ();   // включаем все модули
}

void setLedColor(unsigned int co2)
{
  memset(leds, 0, 1 * sizeof(struct CRGB));
  if(display_mod % 2 == 0) {
    leds[0].r = 0;
    leds[0].g = 0;
    leds[0].b = 0;
  }
  else {
    co2 = constrain(co2, 400, 3000);
    co2 = map(co2, 400, 3000, 0, 128);
    byte c = (byte) co2;
    leds[0].r = c;
    leds[0].g = 128 - c;
    leds[0].b = 0;
  }
  FastLED.show();
}

float getBatteryVoltage()
{
  float val = 0.0;
  for (int i = 0; i < 5; i++) {
    delay(50);
    val += analogRead(BATTERY_PIN);
  }
  val = val / 5;
  val = val / 1024.0 * 4.97;
  return val;
}

unsigned int getCO2Value()
{
  unsigned int co2_ppm = 0;

  if (mySerial.isListening()) {
    mySerial.write(cmd, 9);
    memset(response, 0, 9);
  } else {
    Serial.println("mySerial is not listening");
    return co2_ppm;
  }
  delay(50);
  Serial.print("Bites available:");
  Serial.println(mySerial.available());
  if (mySerial.available() >= 9) {
    mySerial.readBytes(response, 9);
    byte crc = 0;
    for (int i = 1; i < 8; i++) crc += response[i];
    crc = 255 - crc;
    crc++;

    if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) )
    {
      Serial.println("CRC error: " + String(crc) + " / " + String(response[8]));
    }
    else
    {
      unsigned int responseHigh = (unsigned int) response[2];
      unsigned int responseLow = (unsigned int) response[3];
      co2_ppm = (256 * responseHigh) + responseLow;
    }
  } else {
    Serial.println("mySerial is not available");
  }
  return co2_ppm;
}



int getChargePercentage(float battery_voltage)
{
  int charge_percentage = 0;
  battery_voltage *= 100;
  charge_percentage = constrain(battery_voltage, 320, 415);   // 3.2V - 0%, 4.15V - 100%
  charge_percentage = map(charge_percentage, 320, 415, 0, 100);
  return charge_percentage;
}

void printInfoToSerialPort(unsigned int co2_ppm, float humidity, float pressure, float temp, float battery_voltage, int charge_percentage)
{
  Serial.print(" Humidity, %: ");
  Serial.println(humidity, 0);

  Serial.print(" Pressure, kPa: ");
  Serial.println(pressure);

  Serial.print(" Temp, C: ");
  Serial.println(temp);

  Serial.print(" CO2, ppm:");
  Serial.println(co2_ppm);

  Serial.print(" Voltage, V: ");
  Serial.print(battery_voltage);
  Serial.print(" , ");
  Serial.println(charge_percentage);

  return;
}

void drawInfo(unsigned int co2_ppm, float humidity, float pressure, float temp, float battery_voltage, int charge_percentage)
{
  oled.clear();
  if(display_mod <= 2)
  {
    oled.print("V:");
    oled.print(battery_voltage);
    oled.print(", ");
    oled.print(charge_percentage);
    oled.print("%");
    oled.print(", mod:");
    oled.println(display_mod);

    oled.print("CO2:");
    oled.println(co2_ppm);

    oled.print("hdty:");
    oled.println(humidity);
    oled.print("%");

    oled.print("pre:");
    oled.println(pressure);

    oled.print("temp:");
    oled.println(temp);
  }

  return;
}
