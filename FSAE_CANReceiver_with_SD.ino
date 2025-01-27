// FSAEA AV DATA LOGGER
// DEC 2024
// N.Owen, Nuvotion P/L
//
// Utilising the following:

// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
// https://github.com/sandeepmistry/arduino-CAN/blob/master/API.md

#include <CAN.h>

#include <TinyGPSPlus.h>


// The TinyGPSPlus object
TinyGPSPlus gps;

#define HSPI_SCK 18
#define HSPI_MISO 19
#define HSPI_MOSI 23
#define SD_CS 22 //22

#include "FS.h"
#include "SD.h"
#include "SPI.h"

SPIClass * hspi = NULL;

bool SDloaded = 0;
bool SDLOGFILECREATED = 0;
String SDFILENAME = "";
String TIME = "";

//CAR ID
//String ID = "A86"; // QUT
String ID = "A46"; // MONASH


//GPS MESSAGE SUB STRING COMPONANTS
String UTC_TIME = "0";
String LAT = "LAT=0";
String LNG = "LNG=0";
String HFA = "HFA=0";
String HDOP = "HDOP=0";
String HVAL = "HVAL=0";
String SFA = "SFA=0";
String NOS = "NOS=0";
String AFA = "AFA=0";
String ALM = "ALM=0";
String AKM = "AKM=0";
String SPFA = "SPFA=0";
String MPS = "MPS=0";
String KMH = "KMH=0";
String CFA = "CFA=0";
String CMR = "CMR=0";
String CCD = "CCD=0";

String GPS_MESSAGE = "";

/*
   AV STATUS MESSAGE STRINGS
*/
String SA = "SA=0";
String ST = "ST=0";
String STA = "STA=0";
String STT = "STT=0";
String BRA = "BRA=0";
String BRT = "BRT=0";
String MMT = "MMT=0";
String MMA = "MMA=0";
String ALAT = "ALAT=0";
String ALON = "ALON=0";
String YAW = "YAW=0";



String AST = "AST=0";
String EBS = "EBS=0";
String AMI = "AMI=0";
String STS = "STS=0";
String SBS = "SBS=0";
String LAP = "LAP=0";
String CCA = "CCA=0";
String CCT = "CCT=0";




String AV_STATUS = "";
String AV_STATUS_MESSAGE = "";
String LAST_AV_STATUS = "";

/*
   RES MESSAGE STRING
*/
String RES = "RES=0";
String K2T = "K2T=0";
String K3B = "K3B=0";
String RRQ = "RRQ=0";
String LSSN = "LSSN=0";

String RES_MESSAGE = "";
String RESG_STATUS_MESSAGE = "";
String RESG_LAST_AV_STATUS = "";


#define RXD1 21  // External Uart
#define TXD1 2   // External Uart
#define GLED 12 // Green LED
#define BLED 13 // Blue LED


void LEDflash(int x) {
  // Alternativily flash the green and blue leds x amount of times
  for (int i = 0; i <= x; i++) {
    digitalWrite(GLED, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(BLED, LOW);    // turn the LED off by making th
    delay(100);                       // wait for a second
    digitalWrite(GLED, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(BLED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
  }
  digitalWrite(GLED, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(BLED, LOW);    // turn the LED off by making th
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(115200);


  hspi = new SPIClass(HSPI);

  hspi->begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, SD_CS);
  //hspi->begin();


  if (!SD.begin(SD_CS, *hspi, 40000000)) {
    Serial.println("Card Mount Failed");
    //return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    //return;
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
    SDloaded = 1;
  } else {
    Serial.println("UNKNOWN");
  }

  SDloaded = 0;

  while (!Serial);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  LEDflash(4);

  Serial.println("FSAE AV CAN DATA LOGGER");

  // start the CAN bus at 1000 kbps
  if (!CAN.begin(1000E3)) {
    //if (!CAN.begin(500E3)) { //500 kbps
    Serial.println("Starting CAN failed!");
    while (1);
  }
  digitalWrite(GLED, HIGH);   // turn the Green LED on (HIGH is the voltage level)

}

void loop() {

  // try to parse packet
  int packetSize = CAN.parsePacket();
  //Serial.print(" packetsize = ");
  //Serial.println(packetSize, HEX);


  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read()))
      displayInfo();
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }



  //Serial.println("stuck in loop");


  //  if (packetSize || CAN.packetId() != -1) {
  if (packetSize > 4) {
    // received a packet

    int bytecnt = 1;
    while (CAN.available()) {

      char CANRX = CAN.read();

      if (CAN.packetId() == 1282) { // hex id 502

        if (bytecnt == 1) {
          //AS State
          byte CANRXmasked = 0b00000111 & CANRX; // mask
          //Serial1.print("AS STATE  = ");
          //Serial1.println(CANRXmasked, HEX);
          AST = "AST=";
          AST += CANRXmasked;

          //EBS State
          char CANRXshift = CANRX >> 3; // shift first to observe bits 3 and 4
          CANRXmasked = 3 & CANRXshift; //mask out all bits except first 2 bits
          //Serial1.print("EBS STATE  = ");
          //Serial1.println(CANRXmasked, HEX);
          EBS = "EBS=";
          EBS += CANRXmasked;

          //AMI State
          CANRXshift = CANRX >> 5; // shift first to observe bits 5 -> 7
          CANRXmasked = 0b00000111 & CANRXshift; //mask out all bits except first 3 bits
          //Serial1.print("AMI STATE  = ");
          //Serial1.println(CANRXmasked, HEX);
          AMI = "AMI=";
          AMI += CANRXmasked;






        }

        else if (bytecnt == 2) {
          //Steering State
          char CANRXshift = CANRX ; // shift first to observe bit 8
          byte CANRXmasked = 1 & CANRXshift; //mask out all bits except first  bit
          //Serial1.print("STEERING STATE  = ");
          //Serial1.println(CANRXmasked, HEX);
          STS = "STS=";
          STS += CANRXmasked;

          //Service Brake State
          CANRXshift = CANRX >> 1; // shift first to observe bits 5 -> 7
          CANRXmasked = 0b00000111 & CANRXshift; //mask out all bits except first 3 bits
          //Serial1.print("SERVICE BRAKE STATE  = ");
          //Serial1.println(CANRXmasked, HEX);
          SBS = "SBS=";
          SBS += CANRXmasked;

          //LAP COUNT
          CANRXshift = CANRX >> 3; // shift first to observe bits 5 -> 7
          CANRXmasked = 0b00001111 & CANRXshift; //mask out all bits except first 3 bits
          //Serial1.print("LAP COUNT  = ");
          //Serial1.println(CANRXmasked, DEC);
          LAP = "LAP=";
          LAP += CANRXmasked;


        }

        bytecnt++;
      } // End 502 loop

      else if (CAN.packetId() == 1280) { // hex id 500
        // Serial.println("CAN packet 500");
        if (bytecnt == 1) {
          //AS Speed Actual
          byte CANRXmasked = 0b11111111 & CANRX; // mask
          //  Serial1.print("SPEED ACTUAL = ");
          //  Serial1.println(CANRXmasked, DEC);
          SA = "SA=";
          SA += CANRXmasked;


        }
        if (bytecnt == 2) {
          //AS Speed Target
          byte CANRXmasked = 0b11111111 & CANRX; // mask
          //Serial1.print("SPEED TARGET = ");
          // Serial1.println(CANRXmasked, DEC);
          ST = "ST=";
          ST += CANRXmasked;


        }

        if (bytecnt == 3) {
          //AS Steering Angle Actual
          byte CANRXmasked = 0b11111111 & CANRX; // mask
          // Serial1.print("STEERING ANGLE ACTUAL = ");
          //Serial1.println(CANRXmasked, DEC);
          STA = "STA=";
          STA += CANRXmasked;


        }
        if (bytecnt == 4) {
          //AS Steering Angle Target
          byte CANRXmasked = 0b11111111 & CANRX; // mask
          //Serial1.print("STEERING ANGLE TARGET = ");
          //Serial1.println(CANRXmasked, DEC);
          STT = "STA=";
          STT += CANRXmasked;

        }

        if (bytecnt == 5) {
          //AS Brake Hydraulic Actual
          byte CANRXmasked = 0b11111111 & CANRX; // mask
          //Serial1.print("BRAKE HYDRAULIC ACTUAL = ");
          //Serial1.println(CANRXmasked, DEC);
          BRA = "BRA=";
          BRA += CANRXmasked;


        }

        if (bytecnt == 6) {
          //AS Brake Hydraulic Target
          byte CANRXmasked = 0b11111111 & CANRX; // mask
          // Serial1.print("BRAKE HYDRAULIC TARGET = ");
          // Serial1.println(CANRXmasked, DEC);
          BRT = "BRT=";
          BRT += CANRXmasked;


        }

        if (bytecnt == 7) {
          //AS Motor Moment Actual
          byte CANRXmasked = 0b11111111 & CANRX; // mask
          //Serial1.print("MOTOR MOMENT ACTUAL = ");
          //Serial1.println(CANRXmasked, DEC);
          MMT = "MMT=";
          MMT += CANRXmasked;


        }
        if (bytecnt == 8) {
          //AS Motor Moment Target
          byte CANRXmasked = 0b11111111 & CANRX; // mask
          //Serial1.print("MOTOR MOMENT TARGET = ");
          //Serial1.println(CANRXmasked, DEC);
          MMA = "MMA=";
          MMA += CANRXmasked;


        }



        bytecnt++;

      }



      else if (CAN.packetId() == 401) { // hex id 191
        //Serial.println("CAN packet 191");
        if (bytecnt == 1) {
          //AS RES E-STOP
          //byte CANRXmasked = 0b11111111 & CANRX; // mask
          byte CANRXmasked = 0b00000001 & CANRX; // mask
          //Serial1.print("RES ESTOP = ");
          //Serial1.println(CANRXmasked, DEC);
          RES += CANRXmasked;



          byte CANRXshift = CANRX >> 1; // shift first to observe bits 2
          CANRXmasked = 0b00000001 & CANRXshift; //mask out all bits except first 3 bits
          //Serial1.print("RES SW K2 = ");
          //Serial1.println(CANRXmasked, DEC);
          K2T += CANRXmasked;


          //AS BT K3
          CANRXshift = CANRX >> 2; // shift first to observe bits 3
          CANRXmasked = 0b00000001 & CANRXshift; //mask out all bits except first 3 bits
          //Serial1.print("RES BT K3 = ");
          //Serial1.println(CANRXmasked, DEC);
          K3B += CANRXmasked;


        }

        if (bytecnt == 7) {

          byte CANRXmasked =  CANRX; // mask
          //Serial1.print("RES RADIO QUALITY = ");
          // Serial1.println(CANRXmasked, DEC);
          RRQ += CANRXmasked;


        }

        if (bytecnt == 8) {
          //AS RES LoSSN
          byte CANRXshift = CANRX >> 6; // shift first to observe bits 6
          byte CANRXmasked = 0b11111111 & CANRXshift; //mask out all bits except first 3 bits
          //Serial1.print("RES LoSSN = ");
          //Serial1.println(CANRXmasked, DEC);
          LSSN += CANRXmasked;


        }



        bytecnt++;
        //Serial.print("Byte cnt = ");
        //Serial1.println(bytecnt);
      }


    }  // End While loop

    //send AV messages
    AVS_MESSAGE();
    RESG_MESSAGE();



  }



  if (SDloaded == 1) {
    //append messages to SD CARD
    if ((UTC_TIME != 0) && (SDLOGFILECREATED == 0))
    {
      digitalWrite(BLED, HIGH);   // turn the LED on (HIGH is the voltage level)
      SDLOGFILECREATED = 1;
      SDFILENAME = "/";
      SDFILENAME += "FILENAME";

      // SDFILENAME += UTC_TIME;
      SDFILENAME += ".csv";

      Serial.print("SD_FILENAME=");

      Serial.println(SDFILENAME);
      writeFile(SD, SDFILENAME.c_str(), UTC_TIME.c_str());

    }
  }

}

void AVS_MESSAGE() {
  // Example
  //#ID=A46|UTC=P2024820T06:56:04.00|SA=###|ST=###|STA=###|STT=###|BRA=###|BRT=###|MMT=###|MMA=###|ALAT=#########|ALON=#########|YAW=#########|AST=###|EBS=###|AMI=###|STS=###|SBS=###|LAP=###|CCA=###|CCT=###\r\n

  //AV_STATUS MESSAGE
  AV_STATUS += SA;
  AV_STATUS += "|";
  AV_STATUS += ST;
  AV_STATUS += "|";
  AV_STATUS += STA;
  AV_STATUS += "|";
  AV_STATUS += STT;
  AV_STATUS += "|";
  AV_STATUS += BRA;
  AV_STATUS += "|";
  AV_STATUS += BRT;
  AV_STATUS += "|";
  AV_STATUS += MMT;
  AV_STATUS += "|";
  AV_STATUS += MMA;
  AV_STATUS += "|";
  AV_STATUS += ALAT;
  AV_STATUS += "|";
  AV_STATUS += ALON;
  AV_STATUS += "|";
  AV_STATUS += YAW;
  AV_STATUS += "|";
  AV_STATUS += AST;
  AV_STATUS += "|";
  AV_STATUS += EBS;
  AV_STATUS += "|";
  AV_STATUS += STS;
  AV_STATUS += "|";
  AV_STATUS += SBS;
  AV_STATUS += "|";
  AV_STATUS += LAP;
  AV_STATUS += "|";
  AV_STATUS += CCA;
  AV_STATUS += "|";
  AV_STATUS += CCT;
  AV_STATUS += "|";

  if ((AV_STATUS == LAST_AV_STATUS) || (AV_STATUS == "SA=|ST=|STA=|STT=|BRA=|BRT=|MMT=|MMA=|ALAT=0|ALON=0|YAW=0|AST=|EBS=|STS=|SBS=|LAP=|CCA=0|CCT=0|"))
  {

  }
  else {
    AV_STATUS_MESSAGE += "ID=";
    AV_STATUS_MESSAGE += ID;
    AV_STATUS_MESSAGE += "|";
    AV_STATUS_MESSAGE += UTC_TIME;
    AV_STATUS_MESSAGE += "|";
    AV_STATUS_MESSAGE += AV_STATUS;
    Serial1.println(AV_STATUS_MESSAGE);
    AV_STATUS_MESSAGE += "\r\n";
    if (SDloaded == 1) {
      appendFile(SD, SDFILENAME.c_str(), AV_STATUS_MESSAGE.c_str()); // write message to SDCARD
    }
  }

  //UTC_TIME = "UTC=1";
  /*
    SA = "SA=";
    ST = "ST=";
    STA = "STA=";
    STT = "STT=";


    BRA = "BRA=";
    BRT = "BRT=";
    MMT = "MMT=";
    MMA = "MMA=";
  */
  ALAT = "ALAT=0"; //not yet implmented
  ALON = "ALON=0"; //not yet implmented
  YAW = "YAW=0"; //not yet implmented
  /*
    AST = "AST=";
    EBS = "EBS=";
    AMI = "AMI=";
    STS = "STS=";
    SBS = "SBS=";
    LAP = "LAP=";
  */
  CCT = "CCT=0";
  CCA = "CCA=0";

  LAST_AV_STATUS = AV_STATUS;
  AV_STATUS = "";
  AV_STATUS_MESSAGE = "";


}





void RESG_MESSAGE() {
  // Example
  //#ID=A46|UTC=P2024820T06:56:04.00|RES=0|K2T=0|K3B=0|RRQ=255|LSSN=203445\r\n



  //RES_MESSAGE
  RES_MESSAGE += RES;
  RES_MESSAGE += "|";
  RES_MESSAGE += K2T;
  RES_MESSAGE += "|";
  RES_MESSAGE += K3B;
  RES_MESSAGE += "|";
  RES_MESSAGE += RRQ;
  RES_MESSAGE += "|";
  RES_MESSAGE += LSSN;
  RES_MESSAGE += "|";

  if ((RES_MESSAGE == RESG_LAST_AV_STATUS) || (RES_MESSAGE == "RES=|K2T=|K3B=|RRQ=|LSSN=|"))
  {

  }
  else {
    RESG_STATUS_MESSAGE += "ID=";
    RESG_STATUS_MESSAGE += ID;
    RESG_STATUS_MESSAGE += "|";
    RESG_STATUS_MESSAGE += UTC_TIME;
    RESG_STATUS_MESSAGE += "|";
    RESG_STATUS_MESSAGE += RES_MESSAGE;
    Serial1.println(RESG_STATUS_MESSAGE);

    RESG_STATUS_MESSAGE += "\r\n";
    if (SDloaded == 1) {
      appendFile(SD, SDFILENAME.c_str(), RESG_STATUS_MESSAGE.c_str()); // write message to SDCARD
    }
  }

  //UTC_TIME = "UTC=1";
  RES = "RES=";
  K2T = "K2T=";
  K3B = "K3B=";
  RRQ = "RRQ=";
  LSSN = "LSSN=";

  RESG_LAST_AV_STATUS = RES_MESSAGE;
  RES_MESSAGE = "";
  RESG_STATUS_MESSAGE = "";

}





void displayInfo()

{
  //Serial.print(F("Location: "));

  // get date drom GPS module
  if (gps.date.isValid())
  {

    /*
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
        Serial.print(" ");
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    */

    //Serial.print("UTC=");
    //UTC_TIME += "UTC=P";
    UTC_TIME = "UTC=P";

    UTC_TIME += gps.date.year();
    UTC_TIME += gps.date.month();
    UTC_TIME += gps.date.day();
    UTC_TIME += "T";


    //Serial.print("P"); Serial.print(gps.date.year()); Serial.print(gps.date.month()); Serial.print(gps.date.day()); Serial.print("T");

    if (gps.time.hour() < 10) //Serial.print(F("0"));
      UTC_TIME += "0";
    UTC_TIME += gps.time.hour();
    UTC_TIME += ":";
    //Serial.print(gps.time.hour());
    //Serial.print(F(":"));
    if (gps.time.minute() < 10) UTC_TIME += "0"; //Serial.print(F("0"));
    UTC_TIME += "0";
    UTC_TIME += gps.time.minute();
    UTC_TIME += ":";
    //Serial.print(gps.time.minute());
    //Serial.print(F(":"));
    if (gps.time.second() < 10) UTC_TIME += "0"; //Serial.print(F("0"));
    UTC_TIME += gps.time.second();
    UTC_TIME += ".";

    //Serial.print(gps.time.second());
    //Serial.print(F("."));

    if (gps.time.centisecond() < 10) UTC_TIME += "0"; //Serial.print(F("0"));
    UTC_TIME += gps.time.centisecond();
    //Serial.println(gps.time.centisecond());


  }

  if (gps.location.isValid()) {

    /*
        Serial.print("LAT=");
        Serial.print(gps.location.lat(), 8);
        Serial.print(F("|"));
        Serial.print("LNG=");
        Serial.print(gps.location.lng(), 8);
        Serial.println();
    */

    // LAT += "LAT=";
    LAT += String(gps.location.lat(), 8);
    //LNG += "LNG=";
    LNG += String(gps.location.lng(), 8);

    /*
        Serial.print(F("HDOP       Fix Age="));
        Serial.print(gps.hdop.age());
        Serial.print(F("ms raw="));
        Serial.print(gps.hdop.value());
        Serial.print(F(" hdop="));
        Serial.println(gps.hdop.hdop());
    */

    //Serial.print(F("HFA=")); Serial.print(gps.hdop.age()); Serial.print(F("|")); Serial.print(F("HDOP=")); Serial.print(gps.hdop.hdop()); Serial.print(F("|")); Serial.print(F("HVAL=")); Serial.println(gps.hdop.value());
    //HFA += "HFA=";
    HFA += gps.hdop.age();
    //HDOP += "HDOP=";
    HDOP += gps.hdop.hdop();
    //HVAL += "HVAL=";
    HVAL += gps.hdop.value();


    /*
        Serial.print(F("SATELLITES Fix Age="));
        Serial.print(gps.satellites.age());
        Serial.print(F(" ms Value="));
        Serial.println(gps.satellites.value());

    */
    //Serial.print(F("SFA=")); Serial.print(gps.satellites.age()); Serial.print(F("|")); Serial.print(F("NOS=")); Serial.println(gps.satellites.value());
    //SFA += "SFA=";
    SFA += gps.satellites.age();
    //NOS += "NOS=";
    NOS += gps.satellites.value();

    /*
      Serial.print(F("SPEED      Fix Age="));
      Serial.print(gps.speed.age());
      Serial.print(F("ms Raw="));
      Serial.print(gps.speed.value());
      Serial.print(F(" Knots="));
      Serial.print(gps.speed.knots());
      Serial.print(F(" MPH="));
      Serial.print(gps.speed.mph());
      Serial.print(F(" m/s="));
      Serial.print(gps.speed.mps());
      Serial.print(F(" km/h="));
      Serial.println(gps.speed.kmph());
    */

    // Serial.print(F("SPFA=")); Serial.print(gps.speed.age()); Serial.print(F("|")); Serial.print(F("MPS=")); Serial.print(gps.speed.mps()); Serial.print(F("|")); Serial.print(F("KMH=")); Serial.println(gps.speed.kmph());


    /*
        Serial.print(F("ALTITUDE   Fix Age="));
        Serial.print(gps.altitude.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.altitude.value());
        Serial.print(F(" Meters="));
        Serial.print(gps.altitude.meters());
        Serial.print(F(" Miles="));
        Serial.print(gps.altitude.miles());
        Serial.print(F(" KM="));
        Serial.print(gps.altitude.kilometers());
        Serial.print(F(" Feet="));
        Serial.println(gps.altitude.feet());
    */
    // Serial.print(F("AFA=")); Serial.print(gps.altitude.age()); Serial.print(F("|")); Serial.print(F("ALM=")); Serial.print(gps.altitude.meters()); Serial.print(F("|")); Serial.print(F("AKM=")); Serial.println(gps.altitude.kilometers());
    // AFA += "AFA=";
    AFA += gps.altitude.age();
    //ALM += "ALM=";
    ALM += gps.altitude.meters();
    //AKM += "AKM=";
    AKM += gps.altitude.kilometers();


  }
  if (gps.speed.isValid())
  {
    /*
      Serial.print(F("SPEED      Fix Age="));
      Serial.print(gps.speed.age());
      Serial.print(F("ms Raw="));
      Serial.print(gps.speed.value());
      Serial.print(F(" Knots="));
      Serial.print(gps.speed.knots());
      Serial.print(F(" MPH="));
      Serial.print(gps.speed.mph());
      Serial.print(F(" m/s="));
      Serial.print(gps.speed.mps());
      Serial.print(F(" km/h="));
      Serial.println(gps.speed.kmph());
    */
    //Serial.print(F("SPFA=")); Serial.print(gps.speed.age()); Serial.print(F("|")); Serial.print(F("MPS=")); Serial.print(gps.speed.mps()); Serial.print(F("|")); Serial.print(F("KMH=")); Serial.println(gps.speed.kmph());
    //SPFA += "SPFA=";
    SPFA += gps.speed.age();
    //MPS += "MPS=";
    MPS += gps.speed.mps();
    //KMH += "KMH=";
    KMH += gps.speed.kmph();

  }

  if (gps.course.isUpdated())
  {
    /*
      Serial.print(F("COURSE     Fix Age="));
      Serial.print(gps.course.age());
      Serial.print(F("ms Raw="));
      Serial.print(gps.course.value());
      Serial.print(F(" Deg="));
      Serial.println(gps.course.deg());
    */
    CFA = "CFA=";
    CFA += gps.course.age();
    CMR = "CMR=";
    CMR += gps.course.value();
    CCD = "CCD=";
    CCD += gps.course.deg();



  }





  //GPS MESSAGE
  //Serial.print(UTC_TIME);
  GPS_MESSAGE += "ID=";
  GPS_MESSAGE += ID;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += UTC_TIME;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += LAT;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += LNG;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += HFA;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += HDOP;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += HVAL;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += SFA;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += NOS;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += SPFA;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += MPS;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += KMH;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += AFA;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += ALM;
  GPS_MESSAGE += "|";
  GPS_MESSAGE += AKM;
  GPS_MESSAGE += "|";
  /*
    GPS_MESSAGE += CFA;
    GPS_MESSAGE += "|";
    GPS_MESSAGE += CMR;
    GPS_MESSAGE += "|";
    GPS_MESSAGE += CCD;
    GPS_MESSAGE += "|";
  */

  GPS_MESSAGE += "\r\n";

  //Serial1.print(GPS_MESSAGE);
  if (SDloaded == 1) {
    appendFile(SD, SDFILENAME.c_str(), GPS_MESSAGE.c_str()); // write message to SDCARD
  }

  GPS_MESSAGE = "";
  LAT = "LAT=";
  LNG = "LNG=";
  HFA = "HFA=";
  HDOP = "HDOP=";
  HVAL = "HVAL=";
  SFA = "SFA=";
  NOS = "NOS=";
  SPFA = "SPFA=";
  MPS = "MPS=";
  KMH = "KMH=";
  AFA = "AFA=";
  ALM = "ALM=";
  AKM = "AKM=";
  CFA = "CFA=";
  CMR = "CMR=";
  CCD = "CCD=";

}






/*
    SD CARD FUNCTIONS
*/


void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }

  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}
