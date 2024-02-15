#include <Arduino.h>
#include <CAN.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#if ( defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)  || \
        defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) ||    defined(ARDUINO_AVR_ETHERNET) || \
        defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT)   || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO)      || \
        defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) || \
        defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
        defined(ARDUINO_AVR_PROTRINKET3FTDI) )
  #define USE_TIMER_1     true
  #warning Using Timer1
#else          
  #define USE_TIMER_3     true
  #warning Using Timer3
#endif

#ifndef LED_BUILTIN
	#define LED_BUILTIN       13
#endif

#include "TimerInterrupt_Generic.h"
#include "ISR_Timer_Generic.h"

LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display

#define ICM_MSG_ID  0x13
#define TEMP_MSG_ID 0x12

#define ICM_INTR_PIN  2   //from  UNO to MEGA
#define TEMP_INTR_PIN 3   //from MEGA TX to MEGA RX
#define REQ_MSG_PIN 5     //from MEGA to UNO 

uint8_t icm_first=0;
uint8_t temp_first=0;

volatile long sentTimeMicros_ICM=0;
volatile long sentTimeMicros_TEMP=0;
volatile long receivedTimeMicros_ICM=0;
volatile long receivedTimeMicros_TEMP=0;

//Globals
ISR_Timer ISR_timer;
#define TIMER_INTERVAL_MS 1L
#define LED_TOGGLE_INTERVAL_MS        1000L

void TimerHandler()
{
  static bool toggle  = false;
	static int timeRun  = 0;

  ISR_timer.run();

	// Toggle LED every LED_TOGGLE_INTERVAL_MS = 2000ms = 2s
	if (++timeRun == ((LED_TOGGLE_INTERVAL_MS) / TIMER_INTERVAL_MS) )
	{
		timeRun = 0;

		//timer interrupt toggles pin LED_BUILTIN
		digitalWrite(LED_BUILTIN, toggle);
		toggle = !toggle;
	}
}

float accx=0;
uint16_t temp_data=0;

void requestMessage() {
  Serial.println("Requesting message from slaves...");
  digitalWrite(REQ_MSG_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(REQ_MSG_PIN, HIGH);
}

void delayICM_interrupt() {
  sentTimeMicros_ICM = micros();
}

void delayTEMP_interrupt() {
  sentTimeMicros_TEMP = micros();
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(REQ_MSG_PIN, OUTPUT);
  digitalWrite(REQ_MSG_PIN, HIGH);

  Serial.println("CAN Receiver");
  CAN.setPins(53, 10); //CS, INT (INT unused)

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  //Init lcd
  lcd.init();       //initialize i2c LCD
  lcd.backlight();
  delay(1000);

  //Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ICM_INTR_PIN), delayICM_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(TEMP_INTR_PIN), delayTEMP_interrupt, RISING);

#if USE_TIMER_1

  ITimer1.init();

  // Using ATmega328 used in UNO => 16MHz CPU clock ,

  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
    
#elif USE_TIMER_3

  ITimer3.init();

  if (ITimer3.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
  {
    Serial.print(F("Starting  ITimer3 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer3. Select another freq. or timer"));

#endif

  ISR_timer.setInterval(5000L, requestMessage);
}

float reconstructFloat(unsigned char *bytes) {
    float num;
    unsigned char *ptr = (unsigned char *)&num;
    
    // Copy each byte back to the floating point number
    for (int i = 0; i < sizeof(float); ++i) {
        *(ptr + i) = bytes[i];
    }
    return num;
}

uint16_t reconstructUnsignedShort(uint8_t byte1, uint8_t byte2) {
    // Shift the first byte 8 bits to the left and combine it with the second byte using bitwise OR
    uint16_t reconstructed = (uint16_t )(byte1 << 8) | byte2;
    return reconstructed;
}

void loop() {
  int packetSize = CAN.parsePacket();

  if (packetSize || CAN.packetId() != -1) {
    receivedTimeMicros_ICM = micros();
    receivedTimeMicros_TEMP = micros();

    // received a packet
    Serial.print("Received ");

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if(CAN.packetId()==ICM_MSG_ID) {
      //ICM sensor data received 

      //At this point, message is already received and stored in buffer 
      Serial.print("\nICM message received with latency: ");
      Serial.print(receivedTimeMicros_ICM - sentTimeMicros_ICM);
      Serial.print("us");
      Serial.println("\n");

      unsigned char bytes[4];
      bytes[0]=CAN.read();
      bytes[1]=CAN.read();
      bytes[2]=CAN.read();
      bytes[3]=CAN.read();
      accx = reconstructFloat(bytes);
    }
    else if(CAN.packetId()==TEMP_MSG_ID) {
      Serial.print("\nTEMP message received with latency: ");
      Serial.print(receivedTimeMicros_TEMP - sentTimeMicros_TEMP);
      Serial.print("us");
      Serial.println("\n");

      uint8_t val_h=0;
      uint8_t val_l=0;
      val_h = CAN.read();
      val_l = CAN.read();

      temp_data = reconstructUnsignedShort(val_h, val_l);
    }

    CAN.flush();
  }
  lcd.setCursor(0,0);
  lcd.print("AccX: ");
  lcd.setCursor(6,0);
  lcd.print(accx);
  lcd.setCursor(0,1);
  lcd.print("Temp: ");
  lcd.setCursor(6,1);
  lcd.print(temp_data, DEC);
}