/*From https://forum.arduino.cc/t/serial-input-basics-updated/382007*/

// Example 5 - Receive with start- and end-markers combined with parsing

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;
boolean newData = false;

float T_set = 25; //set Temperature in C degree
bool integral_enable = 0;
float Kp=1.0;
float Ti=1.0; //Integrator time constant (seconds) 35
float Td=0; //Derivative time constant (seconds) 23




//============

void setup() {
    Serial.begin(115200);
    Serial.println("This demo expects 3 pieces of data - text, an integer and a floating point value");
    Serial.println("Enter data in this style <25.0,0,1.0,100.0,0>"); //<T_set,integral_enable, Kp, Ti, Td>
    Serial.println();
}

//============

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData();
        newData = false;
    }
    //Serial.println(integerFromPC*floatFromPC);
}

//============

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