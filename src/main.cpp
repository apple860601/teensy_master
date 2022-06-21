#include "SPI.h"

#include "SPI_MSTransfer_T4/SPI_MSTransfer_MASTER.h"
SPI_MSTransfer_MASTER<&SPI, 10, 0x1234> mySPI1234;
SPI_MSTransfer_MASTER<&SPI, 9, 0x4567> mySPI4567;

void myCB(uint16_t *buffer, uint16_t length, AsyncMST info) {
  for ( int i = 0; i < length; i++ ) {
    Serial.print(buffer[i]); Serial.print(" ");
  }
  Serial.print(" --> Length: "); Serial.print(length);
  Serial.print(" --> PacketID: "); Serial.println(info.packetID, HEX);
  
}

void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(10, 1);
  digitalWrite(9, 1);
  mySPI1234.begin();
  mySPI4567.begin();
  mySPI4567.onTransfer(myCB);
  mySPI1234.onTransfer(myCB);
  mySPI4567.pinMode(7, OUTPUT);
  
}
bool test=true;

void loop() {
  // mySPI1234.events();

  static uint32_t t = millis();
  if ( millis() - t > 1000 ) {
    Serial.println(millis());

    uint16_t buf[5] = { 123, -1, 'TEST', 0xF4, true };
    uint16_t buf2[5] = { 0xBEEF, 0xF7, 0xF8, 0xF9, 0xDEAD };
    // digitalWrite(10, 1);
    mySPI1234.transfer16(buf2, 5, random(0x1000, 0x8000));
    mySPI1234.events();
    // digitalWrite(10, 0);
    // digitalWrite(9, 1);
    mySPI4567.transfer16(buf, 5, random(0x1000, 0x8000));
    mySPI4567.events();
    // digitalWrite(9, 0);
    // mySPI1234.onTransfer(myCB);
    
    

    static bool flip = 0;
    flip = !flip;
    mySPI1234.digitalWrite(6, flip);
    //    mySPI1234.pinMode(5, INPUT);
    bool moo = mySPI1234.digitalRead(6);
    Serial.print("State: "); Serial.println(moo);
    mySPI1234.detectSlaves();
    mySPI4567.detectSlaves();
    test=!test;

    mySPI4567.digitalWrite(7,test);

    mySPI1234.pinMode(5, INPUT);
    t = millis();
  }

}