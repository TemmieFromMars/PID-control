//editted from https://github.com/brain-duino/AD7173-Arduino
#include <SPI.h>
#include <stdio.h>

/* registers */
typedef enum {
	/* other ADC registers */
	COMMS_REG = 0x00,
	STATUS_REG = 0x00,
	ADCMODE_REG = 0x01,
	IFMODE_REG = 0x02,
	REGCHECK_REG = 0x03,
	DATA_REG = 0x04,
	GPIOCON_REG = 0x06,
	ID_REG = 0x07,
	/* ADC channel registers */
	CH0 = 0x10,
	CH1 = 0x11,
	CH2 = 0x12,
	CH3 = 0x13,
	CH4 = 0x14,
	CH5 = 0x15,
	CH6 = 0x16,
	CH7 = 0x17,
	CH8 = 0x18,
	CH9 = 0x19,
	CH10 = 0x1A,
	CH11 = 0x1B,
	CH12 = 0x1C,
	CH13 = 0x1D,
	CH14 = 0x1E,
	CH15 = 0x1F,
	/* ADC setup config register */
	SETUP0 = 0x20,
	SETUP1 = 0x21,
	SETUP2 = 0x22,
	SETUP3 = 0x23,
	SETUP4 = 0x24,
	SETUP5 = 0x25,
	SETUP6 = 0x26,
	SETUP7 = 0x27,
	/* ADC filter config registers */
	FILTER0 = 0x28,
	FILTER1 = 0x29,
	FILTER2 = 0x2A,
	FILTER3 = 0x2B,
	FILTER4 = 0x2C,
	FILTER5 = 0x2D,
	FILTER6 = 0x2E,
	FILTER7 = 0x2F,
	/* ADC offset registers */
	OFFSET0 = 0x30,
	OFFSET1 = 0x31,
	OFFSET2 = 0x32,
	OFFSET3 = 0x33,
	OFFSET4 = 0x34,
	OFFSET5 = 0x35,
	OFFSET6 = 0x36,
	OFFSET7 = 0x37,
	/* ADC gain registers */
	GAIN0 = 0x38,
	GAIN1 = 0x39,
	GAIN2 = 0x3A,
	GAIN3 = 0x3B,
	GAIN4 = 0x3C,
	GAIN5 = 0x3D,
	GAIN6 = 0x3E,
	GAIN7 = 0x3F
} adc7173_register_t;

/* ADC analog inputs */
typedef enum {
	AIN0 = 0x00,
	AIN1 = 0x01,
	AIN2 = 0x02,
	AIN3 = 0x03,
	AIN4 = 0x04,
	AIN5 = 0x05,
	AIN6 = 0x06,
	AIN7 = 0x07,
	AIN8 = 0x08,
	AIN9 = 0x09,
	AIN10 = 0x0A,
	AIN11 = 0x0B,
	AIN12 = 0x0C,
	AIN13 = 0x0D,
	AIN14 = 0x0E,
	AIN15 = 0x0F,
	AIN16 = 0x10,
	/* other ADC analog inputs */
	TEMP_SENSOR_POS = 0x11,
	TEMP_SENSOR_NEG = 0x12,
  AVDD1_AVSS_5_POS = 0x13,
  AVDD1_AVSS_5_NEG = 0x14,
	REF_POS = 0x15,
	REF_NEG = 0x16
} analog_input_t;

/* ADC filter data rates (samples per second) */
/* some are are rounded down, the data rates are for sinc5 + sinc1 */
typedef enum {
	SPS_250000 = 0x00,
	SPS_125000 = 0x01,
  SPS_62500 = 0x02,
  SPS_50000 = 0x03,
  SPS_31250 = 0x04,
  SPS_25000 = 0x05,
  SPS_15625 = 0x06,
  SPS_10000 = 0x07,
  SPS_5000 = 0x08,
  SPS_2500 = 0x09,
  SPS_1000 = 0x0A,
  SPS_500 = 0x0B,
  SPS_397 = 0x0C,
  SPS_200 = 0x0D,
  SPS_100 = 0x0E,
  SPS_60 = 0x0F,
  SPS_50 = 0x10,
  SPS_20 = 0x11,
  SPS_17 = 0x12,
  SPS_10 = 0x13,
  SPS_5 = 0x14,
	
} data_rate_t;

/* ADC setup coding modes */
typedef enum {
	UNIPOLAR = 0x00,
	BIPOLAR = 0x01
} coding_mode_t;

/* ADC data conversion modes */
typedef enum {
	CONTINUOUS_CONVERSION_MODE = 0x00,
	SINGLE_CONVERSION_MODE = 0x01,
	STANDBY_MODE = 0x02,
  POWER_DOWN_MODE = 0x03,
  INTERNAL_OFFSET_CALI = 0x04,
  SYSTEM_OFFSET_CALI = 0x06,
  SYSTEM_GAIN_CALI = 0x07,

} data_mode_t;

/* clock mode */
/*
	00 Internal oscillator
	01 Internal oscillator output on XTAL2/CLKIO pin
	10 External clock input on XTAL2/CLKIO pin
	11 External crystal on XTAL1 and XTAL2/CLKIO pins
*/
typedef enum {
	INTERNAL_CLOCK = 0x00,
	INTERNAL_CLOCK_OUTPUT = 0x01,
	EXTERNAL_CLOCK_INPUT = 0x02,
	EXTERNAL_CRYSTAL = 0x03
} clock_mode_t;

/* ADC internal reference modes */
typedef enum {
  //in ADC mode register
	REF_DISABLE = 0x00,
	REF_ENABLE = 0x01
} ref_mode_t;

/* ADC channel buffer setting */
typedef enum {
	AIN_BUF_DISABLE = 0x00,
	AIN_BUF_ENABLE = 0x01
} ain_buf_mode_t;

/* ADC ref input buffer setting */
typedef enum {
	REF_BUF_DISABLE = 0x00,
	REF_BUF_ENABLE = 0x01
} ref_buf_mode_t;

/* setup to select reference source */
/*
	00 External reference source
	01 AIN1/REF2+ and AIN0/REF2−
	10 Internal reference source
	11 External AVDD1 – AVSS
*/
typedef enum {
	REF_EXT = 0x00,
	REF_AIN = 0x01,
	REF_INT = 0x02,
	REF_PWR = 0x03
} setup_ref_source_t;

/* ADC data ready indicator */
#define DATA_READY digitalRead(MISO) == LOW

/* enable or disable debug */
#define DEBUG_ENABLED 1

/* delay for reading and writing registers  */
#define READ_WRITE_DELAY 1

#define DATA_MODE 0//1 for continuous mode
#define APPEND_DATA 1//1 for appended data to include register information

void setup() {
	/* initiate serial communication */
	Serial.begin(230400);
  delay(5000);
  Serial.println("Begin setting");
	SPI.begin();
	/* use SPI mode 3 */
	SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  //reset chip
  for (int i = 0; i < 8; i++) {
		SPI.transfer(0xFF);
	}
  digitalWrite(SS,LOW);
  
	set_channel_config(CH0, true, SETUP0, AIN4, AIN3);
  digitalWrite(SS,HIGH);
  digitalWrite(SS,LOW);

  
  set_setup_config(SETUP0, BIPOLAR,0,0, AIN_BUF_ENABLE,AIN_BUF_ENABLE, REF_INT);
  digitalWrite(SS,HIGH);
  digitalWrite(SS,LOW);
  
  set_adc_mode_config(CONTINUOUS_CONVERSION_MODE, INTERNAL_CLOCK, REF_ENABLE);
  digitalWrite(SS,HIGH);
  digitalWrite(SS,LOW);
  
  set_interface_mode_config(DATA_MODE, APPEND_DATA);
  digitalWrite(SS,HIGH);
  /* wait for ADC */
	delay(10);
}


void loop() {
  
	digitalWrite(SS,LOW);
  byte data[4];
  get_data(data);
  Serial.print(data[0], HEX);
	Serial.print(data[1], HEX);
	Serial.println(data[2], HEX);
	//Serial.println(data[3], HEX);
  long readvalue = 0;
  readvalue += (long)data[0] << 16;
  readvalue += (long)data[1] << 8;
  readvalue += (long)data[2];
  Serial.println(readvalue,HEX);
  
  digitalWrite(SS,HIGH);
  float voltage = ConvertVoltage(readvalue);
  Serial.println(voltage);
  delay(1000);

}

void print_byte(byte value) {
  //print byte
	char format[10];
	sprintf(format, "0x%.2X ", value);
	Serial.print(format);
}

int get_data(byte *value) {
	/* Address: 0x04, Reset: 0x000000, Name: DATA */

	/* when not in continuous read mode, send the read command */
	if (DATA_MODE != 1) {
		/* send communication register id 0x00 */
		SPI.transfer(0x00);
		/* send read command 0x40 to the data register 0x04 */
		SPI.transfer(0x40 | DATA_REG);
	}
	/* get the ADC conversion result (24 bits) */
	value[0] = SPI.transfer(0x00);
	value[1] = SPI.transfer(0x00);
	value[2] = SPI.transfer(0x00);
	/* when status register appending is enabled */
	if (APPEND_DATA) {
		value[3] = SPI.transfer(0x00);
	}
	
	/* when debug enabled */
	if (DEBUG_ENABLED) {
		Serial.print("get_data: read [ ");
		print_byte(value[0]);
		print_byte(value[1]);
		print_byte(value[2]);
		/* when status register appending is enabled */
		if (APPEND_DATA) {
			print_byte(value[3]);
		}
		Serial.println("] from reg [ 0x04 ]");
	}
	/* return error code */
	return 0;
}

int get_register(adc7173_register_t reg, byte *value, int value_len) {
  //read register from AD7175
	/* send communication register id 0x00 */
	SPI.transfer(0x00);
	/* send read command to the desired register 0x00 - 0xFF */
	SPI.transfer(0x40 | reg);
	/* receive the desired amount of bytes */
	for (int i = 0; i < value_len; i++) {
		value[i] = SPI.transfer(0x00);
	}
	/* when debug enabled */
	if (DEBUG_ENABLED) {
    char format[10];
		Serial.print("get_register: got [ ");
		for (int i = 0; i < value_len; i++) {

	    print_byte(value[i]);
		}
		Serial.print("] from reg [ ");
		print_byte(reg);
		Serial.println(" ]");
	}
	/* TODO: find out correct delay */
	delay(READ_WRITE_DELAY);
	/* return error code */
	return 0;
}

int set_register(adc7173_register_t reg, byte *value, int value_len) {
	//write register to AD7175
	SPI.transfer(0x00);
	/* send write command to the desired register 0x00 - 0xFF */
	SPI.transfer(0x00 | reg);
	/* send the desired amount of bytes */
	for (int i = 0; i < value_len; i++) {
		SPI.transfer(value[i]);
	}
	/* when debug enabled */
	if (DEBUG_ENABLED) {
		Serial.print("set_register: set [ ");
		for (int i = 0; i < value_len; i++) {
			print_byte(value[i]);
		}
		Serial.print("] to reg [ ");
		print_byte(reg);
		Serial.println("]");
	}
	/* TODO: find out correct delay */
	delay(READ_WRITE_DELAY);
	/* return error code */
	return 0;
}

int set_adc_mode(data_mode_t data_mode, clock_mode_t clock_mode, ref_mode_t ref_mode) {
	/* Address: 0x01, Reset: 0x2000, Name: ADCMODE */

	/* prepare the configuration value */
	/* REF_EN [15], HIDE_DELAY [14], SING_CYC [13], RESERVED [12:11], DELAY [10:8], RESERVED [7], MODE [6:4], CLOCKSEL [3:2], RESERED [1:0] */
	byte value[2] = {0x00, 0x00};
	value[1] = (data_mode << 4) | (clock_mode << 2);
	value[0] = (ref_mode << 7);

	/* update the configuration value */
	set_register(ADCMODE_REG, value, 2);

	/* verify the updated configuration value */
	get_register(ADCMODE_REG, value, 2);

	/* return error code */
	return 0;
}

int set_channel_config(adc7173_register_t channel, bool enable, adc7173_register_t setup, analog_input_t ain_pos, analog_input_t ain_neg) {
	/* Address: 0x10, Reset: 0x8001, Name: CH0 */
	/* Address Range: 0x11 to 0x1F, Reset: 0x0001, Name: CH1 to CH15 */

	/* prepare the configuration value */
	/* CH_EN0 [15], SETUP_SEL0 [14:12], RESERVED [11:10], AINPOS0 [9:5], AINNEG0 [4:0] */
	byte value[2] = {0x00, 0x00};
	value[0] = (enable << 7) | (setup << 4) | (ain_pos >> 3);
	value[1] = (ain_pos << 5) | ain_neg;


	
  byte value_r[2] = {0x00, 0x00};
	/* verify the updated configuration value */
  while(!(value_r[0]==value[0] and value_r[1]==value[1])){
    Serial.println("Channel config");
    set_register(channel, value, 2);
    get_register(channel, value_r, 2);
  }

	/* return error code */
	return 0;
}

int set_setup_config(adc7173_register_t setup, coding_mode_t coding_mode,ref_buf_mode_t refpos_buf,ref_buf_mode_t refneg_buf,ain_buf_mode_t ainpos_buf,ain_buf_mode_t ainneg_buf , setup_ref_source_t setup_ref_source) {
	/* Address Range: 0x20 to 0x27, Reset: 0x1000, Name: SETUPCON0 to SETUPCON7 */
  
	/* prepare the configuration value */
	byte value[2] = {0x00, 0x00};
	value[0] = (coding_mode << 4) | (refpos_buf<<3)| (refneg_buf<<2)| (ainpos_buf<<1)|ainneg_buf;
	value[1] = (setup_ref_source << 4);
  
  byte value_r[2] = {0x00, 0x00};
	/* verify the updated configuration value */
  while(!(value_r[0]==value[0] and value_r[1]==value[1])){
    Serial.println("Setup config");
    set_register(setup, value, 2);
    get_register(setup, value_r, 2);
  }
	/* return error code */
	return 0;
}

int set_filter_config(adc7173_register_t filter, data_rate_t data_rate) {
	/* Address Range: 0x28 to 0x2F, Reset: 0x0000, Name: FILTCON0 to FILTCON7 */

	/* prepare the configuration value */
	byte value[2] = {0x00, 0x00};
	/* SINC3_MAP0 [15], RESERVED [14:12], ENHFILTEN0 [11], ENHFILT0 [10:8], RESERVED [7], ORDER0 [6:5], ORD0 [4:0] */
	value[1] = data_rate;

	byte value_r[2] = {0x00, 0x00};
	/* verify the updated configuration value */
  while(!(value_r[0]==value[0] and value_r[1]==value[1])){
    set_register(filter, value, 2);
    get_register(filter, value_r, 2);
  }

	/* return error code */
	return 0;
}

int set_offset_config(adc7173_register_t offset, uint32_t offset_value) {
	/* Address Range: 0x30 to 0x37, Reset: 0x0000, Name: OFFSET0 to OFFSET7 */

	/* add the default offset value */
	offset_value += 8388608;
	/* prepare the configuration value */
	byte value[3] = {0x00, 0x00, 0x00};
	value[0] = offset_value >> 16;
	value[1] = offset_value >> 8;
	value[2] = offset_value;

  byte value_r[3] = {0x00, 0x00,0x00};
	/* verify the updated configuration value */
  while(!(value_r[0]==value[0] and value_r[1]==value[1]and value_r[2]==value[2])){
    set_register(offset, value, 3);
    get_register(offset, value_r, 3);
  }
	/* return error code */
	return 0;
}

int set_interface_mode_config(bool continuous_read, bool append_status_reg) {
	/* Address: 0x02, Reset: 0x0000, Name: IFMODE */
  
	/* prepare the configuration value */
	/* RESERVED [15:13], ALT_SYNC [12], IOSTRENGTH [11], RESERVED [10:9], DOUT_RESET [8], CONTREAD [7], DATA_STAT [6], REG_CHECK [5], RESERVED [4], CRC_EN [3:2], RESERVED [1], WL16 [0] */
	byte value[2] = {0x00, 0x00};
	value[1] = (continuous_read << 7) | (append_status_reg << 6);

  byte value_r[2] = {0x00, 0x00};
	/* verify the updated configuration value */
  while(!(value_r[0]==value[0] and value_r[1]==value[1])){
    Serial.println("interface mode config");
    set_register(IFMODE_REG, value, 2);
    get_register(IFMODE_REG, value_r, 2);
  }

	/* return error code */
	return 0;
}

int set_adc_mode_config(data_mode_t data_mode, clock_mode_t clock_mode, ref_mode_t ref_mode) {
	/* Address: 0x01, Reset: 0x2000, Name: ADCMODE */
  
	/* prepare the configuration value */
	/* REF_EN [15], HIDE_DELAY [14], SING_CYC [13], RESERVED [12:11], DELAY [10:8], RESERVED [7], MODE [6:4], CLOCKSEL [3:2], RESERED [1:0] */
	byte value[2] = {0x00, 0x00};
	value[1] = (data_mode << 4) | (clock_mode << 2);
	value[0] = (ref_mode << 7);
  byte value_r[2] = {0x00, 0x00};
	/* verify the updated configuration value */
  while(!(value_r[0]==value[0] and value_r[1]==value[1])){
    Serial.println("ADC mode config");
    set_register(ADCMODE_REG, value, 2);
    get_register(ADCMODE_REG, value_r, 2);
  }
  
	/* return error code */
	return 0;
}

float ConvertVoltage(long readvalue) {
  //convert readvalue to Voltage
  float voltage;
  if (readvalue>8388608) {
    voltage=(readvalue-8388608)*2.5/8388608.0; 
  }
  else {
    readvalue=~readvalue+1; // Flip bits and add one
    readvalue = readvalue&(0x7FFFFF);
    voltage=readvalue*2.5/8388608.0;
    voltage=-voltage;
  }
  return voltage;  
}