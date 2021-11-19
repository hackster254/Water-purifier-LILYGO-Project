
// Please select the corresponding model

#define SIM800L_IP5306_VERSION_20190610
//#define SIM800L_AXP192_VERSION_20200327
// #define SIM800C_AXP192_VERSION_20200609
// #define SIM800L_IP5306_VERSION_20200811

// Define the serial console for debug prints, if needed
#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG          SerialMon
#include<string.h>
#include "utilities.h"

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800          // Modem is SIM800
#define TINY_GSM_RX_BUFFER      1024   // Set RX buffer to 1Kb

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

const char apn[] = "safaricom";
const char user[] = "saf";
const char pass[] = "data";

char* lower = "";
char* response = " ";
String res = "";
bool new_data = 0;
int relay = 0;

#define SMS_TARGET  "+254789814231"
char* msg = "+cmt: \"+254789814231\"";
char* ring = "+clip: \"+254789814231\"";

char* value1 = "relay on";
char* value0 = "relay off";

#define PUMP_SIGNAL 14
#define BULB_SIGNAL 13 //4 // this is second pump
#define BTN_SWITCH 2
#define relay_pin 14
#define pump_pin 4

int btn_status = 0;

int pumpState = HIGH;
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// for multitasking operations
unsigned long prevTime_T1 = millis();

unsigned long prevTime_T2 = millis();

long interval_T1 = 1000;

long interval_T2 = 5000;

void setup()
{

  // Set console baud rate
  SerialMon.begin(115200);
  pinMode(PUMP_SIGNAL,OUTPUT);
  pinMode(BULB_SIGNAL, OUTPUT);
  delay(10);

  digitalWrite(PUMP_SIGNAL, pumpState);
  digitalWrite(BULB_SIGNAL, pumpState);

  // Start power management
  if (setupPMU() == false) {
    Serial.println("Setting power error");
  }

  // Some start operations
  setupModem();

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  delay(6000);

  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  modem.simUnlock("4019");
  delay(1000);

  SerialAT.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();

  SerialAT.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  SerialAT.println("AT+CNMI=1,2,0,0,0"); // Decides how newly arrived SMS messages should be handled
  //updateSerial();
}

void loop()
{
  //updateSerial();

  unsigned long currentTime = millis();

  if ((currentTime - prevTime_T1) > 100)
  {
    pumpLedCon();

    prevTime_T1 = currentTime;
  }

  if ((currentTime - prevTime_T2) > interval_T2)
  {
    updateSerial();

    prevTime_T2 = currentTime;
  }
}

void pumpLedCon()
{
  int reading = digitalRead(BTN_SWITCH);
  if (reading != lastButtonState)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading != buttonState)
    {
      buttonState = reading;
      if (buttonState == HIGH)
      {
        pumpState = !pumpState;
      }
    }
  }

  digitalWrite(BULB_SIGNAL, pumpState);
  digitalWrite(PUMP_SIGNAL, pumpState);
  lastButtonState = reading;
}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    SerialAT.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (SerialAT.available())
  {
    //Serial.write(SerialAT.read());//Forward what Software Serial received to Serial Port

    char add = SerialAT.read();
    res = res + add;
    delay(1);
    new_data = 1;
  }

  if (new_data) {
    response = &res[0];
    //------------------------------------------------- Converting every character of the String in lower form
    const int length = strlen( response ); // get the length of the text
    lower = ( char* )malloc( length + 1 ); // allocate 'length' bytes + 1 (for null terminator) and cast to char*
    lower[ length ] = 0; // set the last byte to a null terminator
    //------------------------------------------------- copy all character bytes to the new buffer using tolower
    for ( int i = 0; i < length; i++ )
    {
      lower[ i ] = tolower( response[ i ] );
    }
    Serial.println(lower);// printin the String in lower character form
    Serial.println("\n");
    relay_control( lower);
    response = "";
    res = "";
    lower = "";
    new_data = 0;

  }
}

void relay_control(char* lower)
{

  if (strstr(lower, msg))
  {
  

    String source = (String)lower;
    

    char* desti = &source[0];
    int i = 0;
    while (desti[i+2] != '\n' )
      i++;
    String destin = (String)desti;
    source = source.substring(51);
    
    Serial.print("Your message is = ");Serial.print(source);
    char* relay1 = &source[0];
    if (strstr(relay1, value1))
    {
      if(pumpState ==LOW ){
        digitalWrite(PUMP_SIGNAL, HIGH);
        digitalWrite(BULB_SIGNAL, HIGH);
        res = modem.sendSMS(SMS_TARGET, String("Relay is turned on"));
        DBG("SMS:", res ? "OK" : "fail");
        relay = 1;
        return; 
      }
    
      
      
    }
    if (strstr(relay1, value0)){
      if(pumpState == HIGH){
        digitalWrite(PUMP_SIGNAL, LOW);
        digitalWrite(BULB_SIGNAL, LOW);
        res = modem.sendSMS(SMS_TARGET, String("Relay is turned off"));
        DBG("SMS:", res ? "OK" : "fail");
        relay = 0;
        return; 
    }
    
  }

     if (strstr(lower, ring))
      {
        
        bool res = modem.callHangup();
        DBG("Hang up:", res ? "OK" : "fail"); 
        if(relay == 1)
        {
          digitalWrite(relay_pin, LOW);
          relay = 0;
          res = modem.sendSMS(SMS_TARGET, String("Relay is turned off"));
          DBG("SMS:", res ? "OK" : "fail");
          return;    
        }
        if(relay == 0)
        {
          digitalWrite(relay_pin, HIGH);
          relay = 1;
          res = modem.sendSMS(SMS_TARGET, String("Relay is turned on"));
          DBG("SMS:", res ? "OK" : "fail");
          return;
        }
      }
  }

}
