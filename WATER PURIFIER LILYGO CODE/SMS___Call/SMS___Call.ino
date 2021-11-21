
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

#include "secrets.h"

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

#include "ThingSpeak.h"
TinyGsmClient client(modem); // create a client

unsigned long myChannelNumber = SECRET_CH_ID;
const char *myWriteAPIKey = SECRET_WRITE_APIKEY;

const char *myReadAPIKey = SECRET_READ_APIKEY;
//CODE FOR FLOW METER

#define SENSOR 13 //4 // to edit

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
//boolean ledState = LOW;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned long flowMilliLitres;
unsigned int totalMilliLitres;
float flowLitres;
float totalLitres;

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

// Initialize our values
int number1 = 0;
int number2 = random(0, 100);
int number3 = random(0, 100);
int number4 = random(0, 100);
String myStatus = "";


#define SMS_TARGET  "+254789814231"
char* msg = "+cmt: \"+254789814231\"";
char* ring = "+clip: \"+254789814231\"";

char* value1 = "relay on";
char* value0 = "relay off";

#define PUMP_SIGNAL 14
#define BULB_SIGNAL  0// supposed to be 13 //4 // this is second pump
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
unsigned long prevTime_T1 = 0;

unsigned long prevTime_T2 = 0;

unsigned long prevTime_T3 = 0;

unsigned long prevTime_T4 = 0;

long interval_T1 = 1000;

long interval_T2 = 5000;

long interval_T3 = 50000;

long interval_T4 = 50000;

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
  SerialMon.println(" OK");
  delay(1000);

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }
  SerialMon.print(F("Connecting to APN: "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(" fail");
    delay(1000);
    return;
  }
  SerialMon.println(" OK");


  SerialAT.println("AT"); //Once the handshake test is successful, it will back to OK
  //updateSerial();

  SerialAT.println("AT+CMGF=1"); // Configuring TEXT mode
  //updateSerial();
  SerialAT.println("AT+CNMI=1,2,0,0,0"); // Decides how newly arrived SMS messages should be handled
  //updateSerial();

  ThingSpeak.begin(client); // Initialize ThingSpeak

  pinMode(SENSOR, INPUT_PULLUP);

  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
}

void loop()
{
  //updateSerial();
 pumpLedCon();
//digitalWrite(PUMP_SIGNAL, LOW);
//thingspeakActions();
//sendReminder();
  unsigned long currentTime = millis();

/*
  if ((currentTime - prevTime_T1) > 10000)
  {
    pumpLedCon();
    //delay(5000);
    Serial.println("carying out task1");
    prevTime_T1 = currentTime;
  }

  */
/*  

 if ((currentTime - prevTime_T2) > interval_T2)
  {
    updateSerial();
  Serial.println("carying out task2 sms to control everything");
    prevTime_T2 = currentTime;
  }
  
*/
  if((currentTime -prevTime_T3)> interval_T3){ // interval_T3
    if(pumpState  == LOW){
      thingspeakActions();
Serial.println("carying out task3 sending data to thingspeak");
      prevTime_T3 = currentTime;
    }
  }
  

  if((currentTime- prevTime_T4)>interval_T4){
    sendReminder();
    Serial.println("carying out task4 sending reminder incase volume reaches 100l");

    prevTime_T4 = currentTime;
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
      if(pumpState ==HIGH ){
        Serial.println("pump state is:" +pumpState);
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

void thingspeakActions()
{

  int statusCode = 0;
  currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {

    pulse1Sec = pulseCount;
    pulseCount = 0;

    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();

    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;
    flowLitres = (flowRate / 60);

    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
    totalLitres += flowLitres;

    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(float(flowRate)); // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t"); // Print tab space

    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity: ");
    Serial.print(totalMilliLitres);
    Serial.print("mL / ");
    Serial.print(totalLitres);
    Serial.println("L");
  }
  //CHANGED IN ORDER TO TEST IF GSM CAN SEND DATA
  
  ThingSpeak.setField(1, flowRate);
  ThingSpeak.setField(2, totalLitres);
/*
  ThingSpeak.setField(1, number2);
  ThingSpeak.setField(2, number3);
*/
  //ThingSpeak.setField(3, number3);
  //ThingSpeak.setField(4, number4);

  // figure out the status message
  if (number1 > number2)
  {
    myStatus = String("field1 is greater than field2");
  }
  else if (number1 < number2)
  {
    myStatus = String("field1 is less than field2");
  }
  else
  {
    myStatus = String("field1 equals field2");
  }

  // set the status
  ThingSpeak.setStatus(myStatus);

  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200)
  {
    Serial.println("Channel update successful.");
  }
  else
  {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  /*
  // change the values
  number1++;
  if(number1 > 99){
    number1 = 0;
  }
  number2 = random(0,100);
  //number3 = random(0,100);
  //number4 = random(0,100);
  */

  delay(2000); // Wait 20 seconds to update the channel again

  float volumeRead = ThingSpeak.readFloatField(myChannelNumber, 2, myReadAPIKey);

  // Check the status of the read operation to see if it was successful
  statusCode = ThingSpeak.getLastReadStatus();
  if (statusCode == 200)
  {
    Serial.println("VOLUME at MathWorks HQ: " + String(volumeRead) + " deg F");
  }
  else
  {
    Serial.println("Problem reading channel. HTTP error code " + String(statusCode));
  }
  Serial.print("volume read is: ");
  Serial.println(volumeRead);

  /*

  if(flowRate > 0.1){
digitalWrite(BULB_SIGNAL, HIGH);
Serial.println("PUMP going on... ");
    
  }else {
    digitalWrite(BULB_SIGNAL, LOW);
    Serial.println("PUMP going OFF ...");
  }
  */
}

void sendReminder()
{
  float currenttotalvolume = ThingSpeak.readFloatField(myChannelNumber, 2, myReadAPIKey);
  Serial.println("CURRENT TOTAL VOLUME TO BE USED FOR THE REMINDER :");
  Serial.print(currenttotalvolume);
  if (currenttotalvolume > 0.0001)
  {

    res = modem.sendSMS(SMS_TARGET, String("Time to change the filter "));
    DBG("SMS:", res ? "OK" : "fail");
    return;
  }
}
