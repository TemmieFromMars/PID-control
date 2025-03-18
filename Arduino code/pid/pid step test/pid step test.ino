/*driver editted from https://github.com/brain-duino/AD7173-Arduino*/
/*Serial communication from https://forum.arduino.cc/t/serial-input-basics-updated/382007*/
#include <SPI.h>
#include <stdio.h>

/* registers for ADC7178-5*, details in its datasheet*/
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
} adc7175_register_t;

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

#define DATA_READY digitalRead(MISO) == LOW

/* enable or disable debug */
#define DEBUG_ENABLED 0

/* delay for reading and writing registers  */
#define READ_WRITE_DELAY 0

#define DATA_MODE 0//1 for continuous mode
#define APPEND_DATA 1//1 for appended data to include register information

/*set temperature control parameters*/
float T_set = 23.5; //set Temperature in C degree
float T_set_face[6] = {T_set+0, T_set+0.22, T_set+0.12, T_set+0.33, T_set+0.33,T_set+0.22};
//PID Parameters
//Kp (err+ Ingetal(err)/Ti + Td/Derivative(err))
float Kp=900;
float Ti=600.0; //Integrator time constant 
float Td=0; //Derivative time constant 
bool PIDenable =0;
bool integral_enable = 1;
const int outmax = 255;
const int ADCspeed = SPS_5;// ADC reading speed, this will change settling time and reading accuracy.  faster than 1000 will cause problem
/*variables for PID control*/
float Tshield[6]{0}; //averaged temperature for each face
float err[6]{0};//err
float prev_error[6]{0}; //previous err
float delta_err[6]{0};//err_k - err_k-1
float prev_delta_err[6]{0};
int output[6]{0}; //output to PWM to control range 0-255
bool pidflag = 0;


/*parameters for fit R-T curve*/
const float R0 = 10000.0; //precision resistor
const float vref = 5.0; //reference voltage
const double SH_A = 0.00112199583; //SH equation A coefficient
const double SH_B = 0.000235375345; //SH equation B coefficient
const double SH_C = 0.0000000819736340; //SH equation C coefficient

/*pin configuration*/
const int CS1 = 12; //select pin for ADC1,  SS is default in code
const int CS2 = 9; // select pin for ADC2
const int NUM_CH = 12; //number of channnels
const int PWMpin[6] = {5,3,6,11,10,13}; //use power width modulation to manipulate output.
const int ADC1_select = CS1;
const int ADC2_select = CS2;

/*ADC channel connection*/
const adc7175_register_t Channels1[13] = {CH0,CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8,CH9,CH10,CH11,CH12}; //channel 0 is disabled, to make sure all data reading will not be 0.
const analog_input_t AINports1[13] = {AIN1,AIN12,AIN13,AIN14,AIN15,AIN10,AIN11,AIN6,AIN7,AIN8,AIN9,AIN4,AIN5};
const adc7175_register_t Channels2[13] = {CH0,CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8,CH9,CH10,CH11,CH12};
const analog_input_t AINports2[13] = {AIN1,AIN12,AIN13,AIN14,AIN15,AIN10,AIN11,AIN6,AIN7,AIN8,AIN9,AIN4,AIN5};
const analog_input_t AINref1 = AIN16; 
const analog_input_t AINref2 = AIN16;

/*variables for reading ADC*/
byte data[4]; //data array, 3byte read value and 1byte status register, in status register can check if data available and chceck the channel.
unsigned long readvalue = 0;
const int N_cycle = 1; // cycles of reading per update
int i_cycle = 0;
unsigned long tstart=0; //timestamp when begin reading
unsigned long deltat=0; //time used
/*
Tshield[i] corresponding to face i+1.
ADC1 
AIN 6,7 FACE 4 
AIN 4,5 FACE 6 
AIN 8,9 FACE 5 
AIN 12,13 FACE 1
AIN 10,11 FACE 3 
AIN 14,15 FACE 2

ADC2
AIN 6,7 FACE 4 
AIN 4,5 FACE 6 
AIN 8,9 FACE 5 
AIN 12,13 FACE 1
AIN 10,11 FACE 3 
AIN 14,15 FACE 2
*/


/*variables for reading from serial*/
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;
boolean newData = false;

int cycles = 0;

void setup() {
	/* initiate serial communication */
	Serial.begin(9600);
  delay(5000);
  
  //set pin mode for ADC select
  pinMode(ADC1_select, OUTPUT);
  pinMode(ADC2_select, OUTPUT);
  //set pin mode for output
  int i = 0;
  while (i<6){
    pinMode(PWMpin[i], OUTPUT);
    analogWrite(PWMpin[i],0);
    i+=1;
  }
  

  //Serial.println("Begin setting");
  SPI.begin();
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE3));
	//SPI.begin();
	/* use SPI mode 3 */
	//SPI.setDataMode(SPI_MODE3);
  //SPI.setBitOrder(MSBFIRST);

  delay(1);
  i = 0;
  //reset ADCs
  for (i = 0; i < 8; i++) {
		SPI.transfer(0xFF);
	}
  
  digitalWrite(ADC1_select,HIGH);
  digitalWrite(ADC2_select,HIGH);


  /*set ADCs, use channel from 1, ch0 not used to avoid 0 in reading*/

  /*set ADC1*/
	digitalWrite(ADC1_select,LOW); //select ADC1
  delay(10);
  //set channels
  i = 0;
  set_channel_config(CH0, false, SETUP0, AINports1[i], AINref1); // disable ch0  
  while (i<NUM_CH){
    i+=1;
    set_channel_config(Channels1[i], true, SETUP0, AINports1[i], AINref1);
  }
  //set other operation parameters
  set_setup_config(SETUP0, BIPOLAR,0,0, AIN_BUF_ENABLE,AIN_BUF_ENABLE, REF_INT);
  set_filter_config(FILTER0, ADCspeed);
  set_adc_mode_config(CONTINUOUS_CONVERSION_MODE, INTERNAL_CLOCK, REF_ENABLE);
  set_interface_mode_config(DATA_MODE, APPEND_DATA);
  digitalWrite(ADC1_select,HIGH); //deselect ADC1

  /*set ADC2*/
  digitalWrite(ADC2_select,LOW); //select ADC2
  delay(10);
  //set channels
  i = 0;
  set_channel_config(CH0, false, SETUP0, AINports2[i], AINref2); // disable ch0  
  while (i<NUM_CH){
    i+=1;
    set_channel_config(Channels2[i], true, SETUP0, AINports2[i], AINref2);
  }  
  //set other operation parameters
  set_setup_config(SETUP0, BIPOLAR,0,0, AIN_BUF_ENABLE,AIN_BUF_ENABLE, REF_INT);
  set_filter_config(FILTER0, ADCspeed);
  set_adc_mode_config(CONTINUOUS_CONVERSION_MODE, INTERNAL_CLOCK, REF_ENABLE);
  set_interface_mode_config(DATA_MODE, APPEND_DATA);
  digitalWrite(ADC2_select,HIGH); //deselect ADC2
  //Serial.println("End setting");

}




void loop() {
  /*variables for reading*/
  float Tread1[NUM_CH] {0};//save reading sum for ADC1
  float Tread2[NUM_CH] {0};//save reading sum for ADC2
  float Tmean1[NUM_CH] {0};//save reading mean for ADC1
  float Tmean2[NUM_CH] {0};//save reading mean for ADC2
  float cycles1[NUM_CH] {0};//save reading num of each channel for ADC1, in case sometimes ADC will miss channels because bad timing.
  float cycles2[NUM_CH] {0};//save reading num of each channel for ADC2

  /*wait from serial to see if any parameters are changed*/
  //serial must send <T_set,integral_enable, Kp, Ti, Td>.
  
  recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData();
        newData = false;
    }

  


  float voltage;
  float tem;
  int i = 0;
  int j = 0;
  int channel; //which channel the reading belonging to
  int i_cycle=0; // cycles of reading. should depend on ADC sample speed. Max num is defined as N_cycle
  tstart = micros();
  while (i_cycle < N_cycle){

    /*reading adc1 start*/
    i=0;
    //Serial.println("ADC1");
    
    digitalWrite(ADC1_select,LOW);
    while (i<NUM_CH){
      i+=1;
      

      while (!adc_ready()){
        //Serial.println("not ready");
        //delay(1);
      }
      get_data(data);
      readvalue = 0;
      readvalue += (long)data[0] << 16;
      readvalue += (long)data[1] << 8;
      readvalue += (long)data[2];
      channel = (data[3]&15)-1;
      voltage = convert_voltage(readvalue);
      tem = convert_tem(voltage);
      if (i==1){
        //Serial.println(voltage,7);
      }
      //Serial.println(voltage);
      Tread1[channel] += tem;
      cycles1[channel] += 1; // in case missing in some channel. this will cause problem when averaging.
      
      //Serial.println(micros()-tstart);
    }
    digitalWrite(ADC1_select,HIGH);
    /*read adc1 end*/
    
    /*read adc2 start*/
    i=0;
    //Serial.println("ADC2");
    digitalWrite(ADC2_select,LOW);
    while (i<NUM_CH){
      i+=1;
      while (!adc_ready()){
        //Serial.println("not ready");
        //delay(1);
      }
      get_data(data);
      readvalue = 0;
      readvalue += (long)data[0] << 16;
      readvalue += (long)data[1] << 8;
      readvalue += (long)data[2] ;
      channel = (data[3]&15)-1;
      voltage = convert_voltage(readvalue);
      tem = convert_tem(voltage);
      //Serial.println(voltage);
      
      Tread2[channel] += tem;
      //Serial.println(Tread2[channel]); 
      if (0){
        Serial.println(tem,7);
      }

      cycles2[channel] += 1; // in case missing in some channel. this will cause problem when averaging.
      //Serial.println(cycles2[channel]); 
      if  (i == NUM_CH){
        break;
      }

    }
    digitalWrite(ADC2_select,HIGH);
    /*read adc2 end*/
    i_cycle += 1;
  }
  deltat = (micros()-tstart); // to seconds
  //Serial.print("time spent: ");
  //Serial.println(deltat);

  /*average over cycles and faces*/
  i=0;
  while (i<NUM_CH){
    Tmean1[i] = Tread1[i]/cycles1[i];
    Tmean2[i] = Tread2[i]/cycles2[i];
    i+=1;
  }
  Tshield[1-1] = (Tmean1[0]+Tmean1[1]+Tmean2[0]+Tmean2[1])/4;
  Tshield[2-1] = (Tmean1[2]+Tmean1[3]+Tmean2[2]+Tmean2[3])/4;
  Tshield[3-1] = (Tmean1[4]+Tmean1[5]+Tmean2[4]+Tmean2[5])/4;
  Tshield[4-1] = (Tmean1[6]+Tmean1[7]+Tmean2[6]+Tmean2[7])/4;
  Tshield[5-1] = (Tmean1[8]+Tmean1[9]+Tmean2[8]+Tmean2[9])/4;
  Tshield[6-1] = (Tmean1[10]+Tmean1[11]+Tmean2[10]+Tmean2[11])/4;

  /*update servo*/
  i=0;
  char strBuf[90];
  char TreadingBuf[6][10];
  float outbuf;
  while (i<6){
    err[i] = Tshield[i]-T_set_face[i];
    if (prev_error[i]-err[i] > 1 || prev_error[i]-err[i]<-1){
      if (pidflag == 0){
        //avoid any error in reading can affect control
        err[i] = prev_error[i];
        pidflag = 1;
      }
    }
    dtostrf(Tshield[i],7,5, TreadingBuf[i]); //write to serial buffer

    delta_err[i] = err[i] - prev_error[i];
    

    
    if (cycles > 100){
      output[i] = 0;
      analogWrite(PWMpin[i],output[i]);//range 0-255, output to PWMpin
    }
    i+=1;
  }
  sprintf(strBuf, "%s,%d,%s,%d,%s,%d,%s,%d,%s,%d,%s,%d", TreadingBuf[0],output[0], TreadingBuf[1],output[1], TreadingBuf[2],output[2], TreadingBuf[3],output[3], TreadingBuf[4],output[4], TreadingBuf[5],output[5]);
  Serial.println(strBuf);
  cycles+=1;
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts
    //<T_set,integral_enable, Kp, Ti, Td>
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    T_set = atof(strtokIndx); // copy it to messageFromPC
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integral_enable = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    Kp = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    Ti = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    Td = atof(strtokIndx);     // convert this part to a float

}

//============

void showParsedData() {
    //print paratemers to see
  Serial.print("T_set = ");
  Serial.println(T_set);
  Serial.print("integral enable = ");
  Serial.println(integral_enable);
  Serial.print("Kp = ");
  Serial.println(Kp);
  Serial.print("Ti = ");
  Serial.println(Ti);
  Serial.print("Td = ");
  Serial.println(Td);
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

int get_register(adc7175_register_t reg, byte *value, int value_len) {
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
	//delay(READ_WRITE_DELAY);
	/* return error code */
	return 0;
}

int set_register(adc7175_register_t reg, byte *value, int value_len) {
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
	//delay(READ_WRITE_DELAY);
	/* return error code */
	return 0;
}

bool is_valid_id() {
	/* Address: 0x07, Reset: 0x30DX, Name: ID */
	/* get the ADC device ID */
	byte id[2];
	get_register(ID_REG, id, 2);
	/* check if the id matches 0x3CDX, where X is don't care */
	id[1] &= 0xF0;
	bool valid_id = id[0] == 0x3C && id[1] == 0xD0;

	/* return validity of ADC device ID */
	return valid_id;
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

int set_channel_config(adc7175_register_t channel, bool enable, adc7175_register_t setup, analog_input_t ain_pos, analog_input_t ain_neg) {
	/* Address: 0x10, Reset: 0x8001, Name: CH0 */
	/* Address Range: 0x11 to 0x1F, Reset: 0x0001, Name: CH1 to CH15 */

	/* prepare the configuration value */
	/* CH_EN0 [15], SETUP_SEL0 [14:12], RESERVED [11:10], AINPOS0 [9:5], AINNEG0 [4:0] */
	byte value[2] = {0x00, 0x00};
	value[0] = (enable << 7) | (setup << 4) | (ain_pos >> 3);
	value[1] = (ain_pos << 5) | ain_neg;

  //Serial.print(value[0],BIN);
  //Serial.print('|');
  //Serial.println(value[1],BIN);
	
  byte value_r[2] = {0x00, 0x00};
	/* verify the updated configuration value */
  //while(!(value_r[0]==value[0] and value_r[1]==value[1])){
  //  Serial.println("Channel config");
  //  set_register(channel, value, 2);
  //  get_register(channel, value_r, 2);
  //}
  set_register(channel, value, 2);
  get_register(channel, value_r, 2);
	/* return error code */
	return 0;
}

int set_setup_config(adc7175_register_t setup, coding_mode_t coding_mode,ref_buf_mode_t refpos_buf,ref_buf_mode_t refneg_buf,ain_buf_mode_t ainpos_buf,ain_buf_mode_t ainneg_buf , setup_ref_source_t setup_ref_source) {
	/* Address Range: 0x20 to 0x27, Reset: 0x1000, Name: SETUPCON0 to SETUPCON7 */
  
	/* prepare the configuration value */
	byte value[2] = {0x00, 0x00};
	value[0] = (coding_mode << 4) | (refpos_buf<<3)| (refneg_buf<<2)| (ainpos_buf<<1)|ainneg_buf;
	value[1] = (setup_ref_source << 4);
  set_register(setup, value, 2);
  byte value_r[2] = {0x00, 0x00};
  get_register(setup, value_r, 2);
  //byte value_r[2] = {0x00, 0x00};
	/* verify the updated configuration value */
  //while(!(value_r[0]==value[0] and value_r[1]==value[1])){
  //  Serial.println("Setup config");
   // set_register(setup, value, 2);
   // get_register(setup, value_r, 2);
  //}
	/* return error code */
	return 0;
}

int set_filter_config(adc7175_register_t filter, data_rate_t data_rate) {
	/* Address Range: 0x28 to 0x2F, Reset: 0x0000, Name: FILTCON0 to FILTCON7 */

	/* prepare the configuration value */
	byte value[2] = {0x00, 0x00};
	/* SINC3_MAP0 [15], RESERVED [14:12], ENHFILTEN0 [11], ENHFILT0 [10:8], RESERVED [7], ORDER0 [6:5], ORD0 [4:0] */
	value[1] = data_rate;

	byte value_r[2] = {0x00, 0x00};
	/* verify the updated configuration value */
  set_register(filter, value, 2);
  get_register(filter, value_r, 2);

	/* return error code */
	return 0;
}

int set_offset_config(adc7175_register_t offset, uint32_t offset_value) {
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
    //Serial.println("interface mode config");
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
    //Serial.println("ADC mode config");
    set_register(ADCMODE_REG, value, 2);
    get_register(ADCMODE_REG, value_r, 2);
  }
  
	/* return error code */
	return 0;
}

int adc_ready(){
  byte value[1] = {0x00};
  get_register(STATUS_REG, value, 1);
  return (value[0]>>7) == 0;
}

float convert_voltage(long readvalue) {
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

float convert_tem(float voltage) {
  //use Steinhart–Hart equation to fit. parameter SH_A SH_B SH_C
  float rther;//resistance of thermistor
  rther = R0/(vref/(voltage+0.5*vref)-1);

  float temp;
  temp = 1/(SH_A+SH_B*log(rther)+SH_C*log(rther)*log(rther)*log(rther))-273.15;
  return temp;
}

