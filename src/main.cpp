#define DEBUG 0

// PIN FOR RFID
#define RST_PIN 22
#define SS_PIN 21


#define SerialToPC Serial
#define DebugSerial Serial

#define BAUDRATE_PC_SERIAL 9600
#define BAUDRATE_DEBUG_SERIAL 115200


#define COMMAND_FOR_PRESS SerialToPC.println("P")
#define COMMAND_FOR_RIGHT SerialToPC.println("R")
#define COMMAND_FOR_LEFT SerialToPC.println("L")

#define ENCODER_P_PIN 33
#define ENCODER_L_PIN 26
#define ENCODER_R_PIN 25

#define SENSITIVITY 2
#define DELAY_TIME_IN_ms 500


#define TAG1 "48B422"
#define TAG2 "5CE4C4B"
#define TAG3 "585DD99"
#define TAG4 "FFC29C26"

#define CMD_FOR_TAG1 SerialToPC.println("A")
#define CMD_FOR_TAG2 SerialToPC.println("B")
#define CMD_FOR_TAG3 SerialToPC.println("C")
#define CMD_FOR_TAG4 SerialToPC.println("D")

#include <Arduino.h>

#include <SPI.h>
#include <MFRC522.h>

uint8_t encoder_l_counter = 0;
uint8_t encoder_r_counter = 0;

bool currentState;
bool lastState;
TaskHandle_t Task1;
TaskHandle_t Task2;

byte readCard[4];
String tagID = "";

// Create instances
MFRC522 mfrc522(SS_PIN, RST_PIN);

 void read_encoder() 
 { 
   currentState = digitalRead(ENCODER_L_PIN);
   if (currentState != lastState)
    {     
     if (digitalRead(ENCODER_R_PIN) != currentState) { encoder_l_counter++; encoder_r_counter =0; if(encoder_l_counter > SENSITIVITY){ COMMAND_FOR_LEFT; encoder_l_counter=0; delay(DELAY_TIME_IN_ms);} } 
     else {encoder_r_counter ++; encoder_l_counter =0; if(encoder_r_counter > SENSITIVITY) { COMMAND_FOR_RIGHT; encoder_r_counter =0; delay(DELAY_TIME_IN_ms);} }
    }
  //  else if(!digitalRead(ENCODER_P_PIN)){ COMMAND_FOR_PRESS; while(!digitalRead(ENCODER_P_PIN)){yield();}}
   lastState = currentState; 
 }

//Read new tag if available
boolean getID() 
{
  // Getting ready for Reading PICCs
  if ( ! mfrc522.PICC_IsNewCardPresent()) { //If a new PICC placed to RFID reader continue
  return false;
  }
  if ( ! mfrc522.PICC_ReadCardSerial()) { //Since a PICC placed get Serial and continue
  return false;
  }
  tagID = "";
  for ( uint8_t i = 0; i < 4; i++) { // The MIFARE PICCs that we use have 4 byte UID
  //readCard[i] = mfrc522.uid.uidByte[i];
  tagID.concat(String(mfrc522.uid.uidByte[i], HEX)); // Adds the 4 bytes in a single String variable
  }
  tagID.toUpperCase();
  mfrc522.PICC_HaltA(); // Stop reading
  return true;
}

void checkID() 
{
  while (getID()) 
  {
    if (tagID == TAG1) 
    {
      CMD_FOR_TAG1;
      if (DEBUG){ DebugSerial.println("TAG 1");}
      }
    else if (tagID == TAG2) 
    {
      CMD_FOR_TAG2;
      if (DEBUG){ DebugSerial.println("TAG 2");}
    }
    else if (tagID == TAG3) 
    {
      CMD_FOR_TAG3;
      if (DEBUG){ DebugSerial.println("TAG 3");}
    }
    else if (tagID == TAG4) 
    {
      CMD_FOR_TAG4;
      if (DEBUG){ DebugSerial.println("TAG 4");}
    }
    else {
    DebugSerial.println(" ID : " + String (tagID));
    if (DEBUG) { DebugSerial.println("Unknown Card");}}

    if (DEBUG){DebugSerial.println(" ID : " + String (tagID));}  
  }
}
void core_zero_loop()
{
  checkID();
  delay(1);
}

void Task1code( void * pvParameters ){ for(;;){ core_zero_loop(); yield();} }
void Task2code( void * pvParameters ){ for(;;){yield();} }


void dual_core_setup()
{
  delay(500);
  xTaskCreatePinnedToCore(Task1code,"Task1",10000,NULL,1,&Task1,0);                       
  delay(500); 
  xTaskCreatePinnedToCore(Task2code,"Task2",10000,NULL,1,&Task2,1);      
  delay(500); 
}

void setup() 
{
  // Initiating

  SerialToPC.begin(BAUDRATE_PC_SERIAL);
  SPI.begin(); // SPI bus
  mfrc522.PCD_Init(); // MFRC522

  pinMode(ENCODER_P_PIN,INPUT_PULLUP);
  pinMode(ENCODER_L_PIN,INPUT_PULLUP);
  pinMode(ENCODER_R_PIN,INPUT_PULLUP);
  dual_core_setup();
  
}



void loop() 
{
  read_encoder();
}

