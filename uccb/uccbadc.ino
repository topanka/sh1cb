#include <SPI.h>                         // Include the SPI library
 
SPISettings MCP3008(2000000,MSBFIRST,SPI_MODE0);
 
const int  CS_MCP3008=53;          // ADC Chip Select
 
void adc_setup()
{
  SPI.begin();
  pinMode(CS_MCP3008,OUTPUT);
  digitalWrite(CS_MCP3008,LOW);        // Cycle the ADC CS pin as per datasheet
  digitalWrite(CS_MCP3008,HIGH);
 
}
 
/* 
void loop()
{
  
  double vRef        = 4.91;
  int    adc_reading = 0;
 
  adc_reading        = adc_single_channel_read (adc_single_ch7);
 
  Serial.print       ("ADC Ch ");
  Serial.print       (adc_single_ch7 & 0x07);
  Serial.print       (" Voltage: ");
  Serial.println     ((adc_reading * vRef) / 1024, 2);
 
  delay(50);
  
}
*/
 
 
int adc_single_channel_read(byte readAddress)
{
  byte dataMSB=0;
  byte dataLSB=0;
  byte JUNK=0x00;

  SPI.beginTransaction(MCP3008);
  digitalWrite(CS_MCP3008,LOW);
  SPI.transfer(0x01);                             // Start Bit
  dataMSB=SPI.transfer(readAddress << 4) & 0x03;  // Send readAddress and receive MSB data, masked to two bits
  dataLSB=SPI.transfer(JUNK);                     // Push junk data and get LSB byte return
  digitalWrite(CS_MCP3008,HIGH);
  SPI.endTransaction();
 
  return(dataMSB << 8 | dataLSB);
}

