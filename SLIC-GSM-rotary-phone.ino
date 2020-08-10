////////////////////////////////////////////////////////////////////////////
//                                                                        //
//  ROTARY PHONE GSM CONVERTER WITH SUBSCRIBER LINE INTERFACE CIRCUIT     //
//                                                                        //
//  By Johan Berglund, November 2016                                      //
//                                                                        //
//  For Arduino, QCX601 SLIC board and SIM900 or SIM800 breakout board    //
//                                                                        // 
////////////////////////////////////////////////////////////////////////////
#include "Adafruit_FONA.h"

#define SIMCOM_7000 // SIM7000A/C/E/G

#define SLIC_TEST

#define FONA_PWRKEY 6
#define FONA_RST 7

#define FONA_TX 10 // Microcontroller RX
#define FONA_RX 11 // Microcontroller TX

// this is a large buffer for replies
char replybuffer[255];

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

SoftwareSerial *fonaSerial = &fonaSS;

// Use this one for LTE CAT-M/NB-IoT modules (like SIM7000)
// Notice how we don't include the reset pin because it's reserved for emergencies on the LTE module!
if defined(SIMCOM_7000) || defined(SIMCOM_7500)
  Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();
#end if

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!


#define IDLE_WAIT 1                
#define RINGING 2                  
#define ACTIVE_CALL 3               
#define GETTING_NUMBER 4             



#define hzPin 7         // 25Hz pin to QCX601 pin J1-2 (25Hz signal for ringing) _-_-_-_-_
#define rcPin 8         // RC pin to QCX601 pin J1-3, ring control, HIGH when ringing --___--___--___
#define shkPin 9        // switch hook pin from QCX601 pin J1-4

                        // Change the pins above as is suits your project
                        // For connection to GSM board, pin 2 goes to TX and pin 3 to RX
                        // To change this, edit GSM.cpp in the GSM library
                        
#if defined(SLIC_TEST)
#define ringTestPin 12 
#endif                       

#define oscInterval 20
#define ringInterval 6000
#define ringDuration 1200
#define statusCheckInterval 1000

#define tNewDig 500     // time since last SHK rising edge before counting for next digit
#define tHup 2000       // time since last SHK falling edge before hanging up/flushing number
#define tComplete 6000  // time since last SHK rising edge before starting call
#define tDebounce 15    // debounce time

unsigned long currentMillis = 0L;
unsigned long oscPreviousMillis = 0L;
unsigned long ringPreviousMillis = 0L;
unsigned long statusPreviousMillis = 0L;
unsigned long lastShkDebounce = 0L;
unsigned long lastShkRise = 0L;
unsigned long lastShkFall = 0L;

int shkState = 0;           // LOW is on hook, HIGH is off hook
int edge;
int lastShkReading = 0;
int currentShkReading = 0;

#if defined(SLIC_TEST)
int ringTest; 
#endif  

byte pulses = 0;
byte digit = 0;
String number = "";
byte digits = 0;            // the number of digits we have collected
char numArray[10];
byte lteStatus;            
byte state;



////////////////////////////////////////////////////////////////////////////
// 
//  SETUP
//

void setup() {
 
  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state

  pinMode(FONA_PWRKEY, OUTPUT);

  // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  powerOn(); // See function definition at the very end of the sketch

  Serial.begin(9600);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take several seconds)"));

  // SIM7000 takes about 3s to turn on but SIM7500 takes about 15s
  // Press reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset
  
  fonaSS.begin(115200); // Default SIM7000 shield baud rate

  Serial.println(F("Configuring to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    while (1); // Don't proceed if it couldn't find the device
  }
  
   type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM5320A:
      Serial.println(F("SIM5320A (American)")); break;
    case SIM5320E:
      Serial.println(F("SIM5320E (European)")); break;
    case SIM7000A:
      Serial.println(F("SIM7000A (American)")); break;
    case SIM7000C:
      Serial.println(F("SIM7000C (Chinese)")); break;
    case SIM7000E:
      Serial.println(F("SIM7000E (European)")); break;
    case SIM7000G:
      Serial.println(F("SIM7000G (Global)")); break;
    case SIM7500A:
      Serial.println(F("SIM7500A (American)")); break;
    case SIM7500E:
      Serial.println(F("SIM7500E (European)")); break;
    default:
      Serial.println(F("???")); break;
  }

  // Print module IMEI number.
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
  
    // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1

  // Configure a GPRS APN, username, and password.
  // You might need to do this to access your network's GPRS/data
  // network.  Contact your provider for the exact APN, username,
  // and password values.  Username and password are optional and
  // can be removed, but APN is required.
  //fona.setNetworkSettings(F("your APN"), F("your username"), F("your password"));
  //fona.setNetworkSettings(F("m2m.com.attz")); // For AT&T IoT SIM card
  //fona.setNetworkSettings(F("telstra.internet")); // For Telstra (Australia) SIM card - CAT-M1 (Band 28)
  fona.setNetworkSettings(F("hologram")); // For Hologram SIM card

  // Optionally configure HTTP gets to follow redirects over SSL.
  // Default is not to follow SSL redirects, however if you uncomment
  // the following line then redirects over SSL will be followed.
  //fona.setHTTPSRedirect(true);

  /*
  // Other examples of some things you can set:
  fona.setPreferredMode(38); // Use LTE only, not 2G
  fona.setPreferredLTEMode(1); // Use LTE CAT-M only, not NB-IoT
  fona.setOperatingBand("CAT-M", 12); // AT&T uses band 12
//  fona.setOperatingBand("CAT-M", 13); // Verizon uses band 13
  fona.enableRTC(true);
  
  fona.enableSleepMode(true);
  fona.set_eDRX(1, 4, "0010");
  fona.enablePSM(true);
  // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
  fona.setNetLED(true, 2, 64, 3000); // on/off, mode, timer_on, timer_off
  fona.setNetLED(false); // Disable network status LED
  */
}
  
  
  pinMode(shkPin, INPUT);
  pinMode(hzPin, OUTPUT);
  pinMode(rcPin, OUTPUT);
  digitalWrite(hzPin, 0);
  #if defined(SLIC_TEST)
  pinMode(ringTestPin, INPUT_PULLUP);
  #endif

byte pulses = 0;
byte digit = 0;
String number = "";
byte digits = 0;            // the number of digits we have collected
char numArray[10];
byte lteStatus;
byte state;
bool incoming

  Serial.println("\nHello, I am your telephone.");

  state = IDLE_WAIT;
}

////////////////////////////////////////////////////////////////////////////
// 
//  MAIN LOOP
//

void loop() {

  currentMillis = millis(); // get snapshot of time for debouncing and timing

  // read and debounce hook pin
  currentShkReading = digitalRead(shkPin);
  if (currentShkReading != lastShkReading) {
    // reset debouncing timer
    lastShkDebounce = currentMillis;
  }
  if ((unsigned long)(currentMillis - lastShkDebounce) > tDebounce) {
    // debounce done, set shk state to debounced value if changed
    if (shkState != currentShkReading) {
      shkState = currentShkReading;
      if (shkState == HIGH) {
        lastShkRise = millis();
      } else {
        lastShkFall = millis();  
      }
    }
  }
  lastShkReading = currentShkReading;

  if ((unsigned long)(currentMillis - statusPreviousMillis) >= statusCheckInterval) {
    // Time to check status of GSM board
    #if defined(SIMCOM_7000)        

    if (!digitalRead(rcPin) && (incoming != GETTING_NUMBER) { // or software serial will interfere with ringing and dialing
      lteStatus = getCallStatus();
    }
    #endif
    #if defined(SLIC_TEST)
    ringTest = !digitalRead(ringTestPin);
    #endif
    statusPreviousMillis = currentMillis;
  }


  ////////////////////////////////////////////////////////////////////////////
  // 
  //  STATE IDLE_WAIT         ///READY
  //
  if (state == IDLE_WAIT) {
    // wait for incoming call or picking up reciever
    #if defined(SIMCOM_7000)
       onIncomingCall()= incoming
    if (incoming == true) {  
      Serial.println("Incoming call. Ringing.");
      state = RINGING;  
    }
    if (lteStatus == 4) {
      // this should not happen
      // hanging up must have failed, try again
      HangUp();
      delay(2000); 
    }
    #endif
    #if defined(SLIC_TEST)
    if (ringTest) {
      Serial.println("Incoming call. Ringing.");
      state = RINGING;
    }
    #endif    
    if (shkState == HIGH) {
      Serial.println("Off hook. Dial your number.");
      state = GETTING_NUMBER;
    }
  }
  ////////////////////////////////////////////////////////////////////////////
  // 
  //  STATE RINGING
  //
  if (state == RINGING) {  
    // ring until reciever is off hook or call disconnected
    // Ringing interval 
    // How much time has passed, accounting for rollover with subtraction!
    if ((unsigned long)(currentMillis - ringPreviousMillis) >= ringInterval) {
      digitalWrite (rcPin,1); // Ring
      // Use the snapshot to set track time until next event
      ringPreviousMillis = currentMillis;
    }
    if (digitalRead(rcPin) && ((unsigned long)(currentMillis - ringPreviousMillis) >= ringDuration)) {
        digitalWrite(rcPin, 0); // Silent after ring duration
    }
    // 25Hz oscillation      
    // How much time has passed, accounting for rollover with subtraction!
    if ((unsigned long)(currentMillis - oscPreviousMillis) >= oscInterval) {
      // It's time to do something!
      if (digitalRead(rcPin)) {
        digitalWrite(hzPin, !digitalRead(hzPin)); // Toggle the 25Hz pin
      }    
      // Use the snapshot to set track time until next event
      oscPreviousMillis = currentMillis;
    }
    if (shkState == HIGH) {
      digitalWrite(rcPin, 0); // stop ringing
      digitalWrite(hzPin, 0);
      // tell GSM board to pick up
      Serial.println("Picking up. Call initiated.");
      #if defined(SIMCOM_7000)
      pickUp();
      #endif   
      state = ACTIVE_CALL; 
    }
    #if defined(SIMCOM_7000)
    
    if (lteStatus != 4) {
      digitalWrite(rcPin, 0); // stop ringing
      digitalWrite(hzPin, 0);
      Serial.println("Caller gave up. Going back to idle.");
      state = IDLE_WAIT;
    }
    #endif
    #if defined(SLIC_TEST)
    if (!ringTest) {
      digitalWrite(rcPin, 0); // stop ringing
      digitalWrite(hzPin, 0);
      Serial.println("Caller gave up. Going back to idle.");
      state = IDLE_WAIT;
    }
    #endif
  }
  ////////////////////////////////////////////////////////////////////////////
  // 
  //  STATE ACTIVE_CALL
  //
  if (state == ACTIVE_CALL) {
    // keep connection until on-hook or call disconnected
    if ((shkState == LOW) && ((unsigned long)(currentMillis - lastShkFall) >= tHup)) {
      // tell GSM board to disconnect call, flush everything, then go idle
      Serial.println("Hanging up. Going idle.");
      #if defined(SIMCOM_7000)
      hangUp();
      #endif
      flushNumber();
      delay(1000); // wait a sec before going to next state to make sure GSM module keeps up
      state = IDLE_WAIT;
    }
    #if defined(SIMCOM_7000)
    if (lteStatus == 1) {
      Serial.println("Call disconnected. Going idle.");
      flushNumber();
      state = IDLE_WAIT;
    }
    #endif
    #if defined(SLIC_TEST)
    if (!ringTest) {
      Serial.println("Call disconnected. Going idle.");
      flushNumber();
      state = IDLE_WAIT;
    }
    #endif
  }
  ////////////////////////////////////////////////////////////////////////////
  // 
  //  STATE GETTING_NUMBER
  //
  if (state == GETTING_NUMBER) {
    // count groups of pulses on SHK (loop disconnect) until complete number
    // if single digit, fetch stored number
    // then make call

    if (pulses && (unsigned long)(currentMillis - lastShkRise) > tNewDig) {
      // if there are pulses, check rising edge timer for complete digit timeout
      digit = pulses - 1; // one pulse is zero, ten pulses is nine (swedish system)
      // for systems where ten pulses is zero, use code below instead:
      // digit = pulses % 10;
      Serial.println(digit); // just for debug
      // add digit to number string
      number += (int)digit;
      digits++;
      pulses = 0;
    }

    if ((shkState == LOW) && (edge == 0)) {
      edge = 1;
    } else if ((shkState == HIGH) && (edge == 1)) {
      pulses++;
      Serial.print(". "); // just for debug . . . . .
      edge = 0;
    }
    
    if ((digits && (shkState == HIGH) && ((unsigned long)(currentMillis - lastShkRise) > tComplete)) || digits == 10) {
      // if completed number (full 10 digits or timeout with at least one digit)
      // check if shortnumber/fave and then tell GSM board to initiate call 
      if (digits == 1) getFave();
      Serial.print("Number complete, calling: ");
      Serial.println(number);
      number.toCharArray(numArray, 10);
      #if defined(SIMCOM_7000)
      callPhone(numArray);
      #endif
      delay(1000); // wait a sec before going to next state to make sure GSM module keeps up
      state = ACTIVE_CALL;
    }
    if ((shkState == LOW) && (unsigned long)(currentMillis - lastShkFall) > tHup) {
      // reciever on hook, flush everything and go to idle state
      flushNumber();
      Serial.println("On hook. Flushing everything. Going idle.");
      state = IDLE_WAIT;
    }
  }
}  // END OF MAIN LOOP


////////////////////////////////////////////////////////////////////////////
// 
//  FUNCTIONS
//

void getFave() {
  if (number == "1\0"){
    number = "9876543210"; // Significant other
    digits = 10;
  }
  if (number == "2\0"){
    number = "0123456789"; // Best friend
    digits = 10;
  }  
  if (number == "3\0"){
    number = "1234567890"; // fave #3... and so on...
    digits = 10;
  }
  // just add more faves here
}

void flushNumber() {
  digits = 0;
  number = "";
  pulses = 0;
  edge = 0;
}


