#include <SPI.h>

void setup() {
	/* initiate serial communication */

	SPI.begin();
	/* use SPI mode 3 */
	SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  for (int i = 0; i < 8; i++) {
		SPI.transfer(0xFF);
	}
	

}


void loop() {
  /* check if the ID register of the ADC is valid */
  
	digitalWrite(SS,LOW);
  SPI.transfer(0x00);
	/* send read command to the desired register 0x00 - 0xFF */
	SPI.transfer(0x40 | 0x07);
  byte value[2];
  for (int i = 0; i < 2; i++) {
		value[i] = SPI.transfer(0x00);
	}
  Serial.print(value[0],HEX);
  Serial.println(value[1],HEX);
  digitalWrite(SS,HIGH);



  
  delay(1000);



}

