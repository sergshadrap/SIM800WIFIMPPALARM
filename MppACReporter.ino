/*
                The device works as ESP8266 based wifi reporter equipped with GSM interface serviced by SIM800 module and it own LION battery.
                Initially connects to a wifi router and setup one analog reporter intended reporting battery voltage, one momentary - standard relay controller providing on/off switching (10A max),
                Two MppSensors, that intended to report the 220 power voltage and IntA - that supposed to detect any firing on pin 13, two virtual flag devices - one 
                GAlarmF - is intended to follow to the global Alarm flag on AM Server and syncronized with it by rules, AlarmF - virtual flag is intended to be switched by server rules , following to alarm devices.
                The default settings are: Relay pin- 15, Modem reset pin - 0( normal high , when low - reboot the SIM800) , internal alarm pin -13 (for external alarm system, normal closed and in high state).
                P_PHONE = "Contact phone number" - is important setting for enabling SMS message, if no phone number installed SMS won't work.
                Period- is the time in seconds using for many routins ,like checking interval , GPIO refresh , modem reboots and etc.
                GPIO 12,14 are used for voltage,charging, battery conditions control. 
                The current date and time updating from cellular provider networks and can be ajusted manually in P_TIMESET = "D&Time: YY/MM/DD,HH:MM:SS" ex 22/03/30,13:45:00
                Here's how it works:
                The device checking current AC power voltage over battery charger if the status is different than CHARGED/CHARGING the device change the status of power sensor and send message to server "POWER FAILED".
                After AC power disapears the device remaing functionality on battery, checking wifi signal too,  if power failed and wifi host signal disapears , the device rebooting in "no-wifi mode" and send SMS to report number.
                When the power restores it reboots again in wifi mode.
                You can manage the device either over common ethernet pathway(AM Server remote, http page) or by SMS commands.  The format of command : OKXPASSXXXXX , where OK - fixed prefix , 
                X - mode (0- switch relay off, 1 - switch relay on , 2 - re-switch relay off than on , 3- get report , 4- report and balance , 5 -reboot ), PASS - fixed prefix and 5 digits code of password.
                During the first start and after each hardware reboot the device sends the password to the host phone number. 
                For first setup you have to tune the server rule for virtual flag UID_GAlarmF - syncronize it with main server Alarm flag. It allows the device react in alarm mode only.
                The device will send SMS to host phone when UID_GAlarmF is on and any incoming signal comes from any of IntA or AlarmF devices.
                UID_AlarmF - is a MppSwitch that can be swithed by any severs rules as reaction to any issue. 
                UID_IntA - GPIO 13 normaly closed and grounded , fires when opened .
                UID_GAlarmF - virtual flag device that can be synconized with server flag. 
                SMS balance comes in UCS format ,RUS Megafon.
*/
#include <Arduino.h>
#include <ESP.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <MppServer.h>
#include <MppDevice.h>
#include <MppDevices.h>
#include <MppHTTPCLient.h>



#define SIMTIMEOUT  5000              //timeot in millsec wait response SIM800 with x sec timeout
#define SIMCHECK 20               // check SIM for operator registration and etc every 20 min
#define DEFAULT_BUFFER_SIZE 64   // default size for serial buffer reading

ADC_MODE(ADC_VCC);


float getBatteryVoltage() {
  return (float) ESP.getVcc() / 1024.0;
}
const char *DeviceVersion = "MppACReporter 2.5"; // New - REporting GSM and wifi signal power

static const char *P_PERIOD = "Period";
static const char *P_BATTERY_TRESHOLD = "BatteryLowerTreshold";
static const char *P_TIMEOUT_SIM = "SIM Timeout msec";
// static const char *P_SMS_PASSWORD = "SMS Password ";
static const char *P_PHONE = "Contact phone number";
static const char *P_TIMESET = "D&Time: YY/MM/DD,HH:MM:SS";
static const char *P_MODEM_PIN = "MODEM Reboot GPIO";
static const char *P_INT_ALARM = "External Alarm GPIO";

//static const char *P_AMSERVER_IP="AM Server IP";
static struct rst_info *resetInfo;

// properties handled by MppSensor
static const char *properties[] = { //
		P_PERIOD, //
//				P_SENSOR_PIN, //
//				P_CYCLE_RECOVERY, // restarts for wifi recovery mode (0 to disable)
//				P_WIFI_RESTART, // restart if wifi not connected (minutes)
        P_TIMEOUT_SIM, // of the AM Server
        P_BATTERY_TRESHOLD, // Battery lower treshold when device is doing to sleep for MAX time
        P_INT_ALARM, // external alarm sensor PIN
        P_PHONE,    // Contact phone only allowed for SMS +79999999999
        P_INITIAL, // set state after power on
        P_USE_LAST, // persist and use last state on startup
        P_RELAY_PIN, //  relay pin
        P_MOMENTARY, // ms,default pulse period
        P_IP_CHECK, //
        P_TIMESET, //time in format YY/MM/DD,HH:MM:SS
        P_MODEM_PIN, // GPIO for switch on SIM800 feeding
//        P_ALRM_FLAG_NAME, // Watering flag UDN name for http GET query       
				NULL };

struct {          //RTC structure
  uint32_t crc32;                //  4 byte
  uint32_t counter;               // 4 byte
  float b_voltage;               // 4 byte battery voltage
  uint32_t RunningTime;            //  4 byte
  uint32_t BootMode=0;              //  4 byte Boot mode = 0- normal mode; 1- no wifi ; 2-reboot to normal ; 3-bad battery- long sleep
  boolean power_fault=false;              // 1 byte if power was down
  uint8_t RelayPin;               // relay pin in no wifi mode
  uint16_t password;                //2 byte
  char  phone[16]={0,};                  //16 byte
    } MyRtcData; // 40 byte 

 SoftwareSerial SIM800(4, 5);        //TX= GPIO4 , RX=GPIO5
MppServer mppServer(DeviceVersion, properties);
static const char *G_ALARM_FLAG="GAlarmF";  // Global alarm follover flag name
class MppDevice battery;
class MppDevice ACsensor;
class MppSensor* Int_alarm; // internal alarm sensor
class MppRelay *relay; // for this relay


// unsigned long alarm_time;     //timer for defeat false triggering 
bool GlobalAlarm =false;
static int RelayPin;
unsigned long next = millis();
unsigned long check_networks= millis();
unsigned long Sms_reported = millis();
bool reboot_flag=false; // if device has already been rebooted after mode 2
bool sensor_state=false; // power sensor state
bool Int_alarm_flag=false; // Internal alarm firing flag
unsigned mode,oldmode,t_reboot,sim_check;

String phones = "";                   // One contact mobile phone number
String _response = "";                          // Value for reply of SIM800
// char *_responsen;                            // Value for reply of SIM800

class CustomFlag : public MppDevice {
 public: CustomFlag() {
   MppSerial.printf("CustomFlag in %d state\n", state);
   };

public: void setState(bool state) {
                this->state = state;
                put(STATE, state ? "on" : "off");
};

public: bool getState() {
    return state;
};

public: bool handleAction(String action, MppParameters parms) {
                boolean handled = false;
                if (action == "state") {
                              if (parms.hasParameter("state")) {
                                             setState(parms.getBoolParameter("state"));
                                             handled = true;
                              } else if (parms.hasParameter("toggle")) {
                                                setState(!state);
                                             handled = true;
                              }
               }
                return handled ? true : MppDevice::handleAction(action, parms);
};

public:void begin() {
  // capture the current state

  MppDevice::begin();
    setState(false);
};
 
private: bool state = false;  

} Alarm_signal;

class PersistedCustomFlag : public MppDevice {
 public: PersistedCustomFlag(const char *name) {
//   Serial.printf("CustomFlag in %d state\n", getState());
    this->flagName = name;
    put(STATE,mppServer.isProperty(name) ? "on" : "off"); // set the initial value for the device
   };

public: void setState(bool NewState) {
  //         if (getUdn()!=NULL) { mppServer.putProperty(getUdn(),NewState ? "true" : "false");
  mppServer.putProperty(flagName,NewState ? "true" : "false"); // save the value as an MppServer device property
    put(STATE,NewState ? "on" : "off"); // report the value for the device
};

public: bool getState() {
// if (getUdn()!=NULL) return mppServer.isProperty(getUdn());
 //     else return false;
 return mppServer.isProperty(flagName); 
};

public: bool handleAction(String action, MppParameters parms) {
                boolean handled = false;
                if (action == "state") {
                              if (parms.hasParameter("state")) {
                                             setState(parms.getBoolParameter("state"));
                                             handled = true;
                              } else if (parms.hasParameter("toggle")) {
                                                setState(!getState());
                                             handled = true;
                              }
               }
                return handled ? true : MppDevice::handleAction(action, parms);
};

public:void begin() {
  MppDevice::begin();
  };
 
 private: 
   const char *flagName;
} Alarm_flag(G_ALARM_FLAG);

int simReset;      
long updatePeriod = 60000;                                        /* Update one time per minute                 */
bool disable_SMS;
bool Relay_On=false;
bool Relay_Off=false;
bool Relay_OFFON=false;
bool Report=false;
bool Balance=false;
bool NewPass=false;
bool SMS_sent= false;   // if SMS was sent or not
bool RebootDevice=false;


bool check_SIM(void)    {
  if ( sendATCommand("AT+CREG?", true)) {
    if(_response.indexOf("+CREG: 0,1"))  return true;
    else return false;
  }
  
}

bool  setupTime(char const *dtime) { 
   String settm="AT+CCLK=\"";
settm+=dtime;
 settm+="+03\""; 
// Serial.printf("First String of time:%s\n",dtime);
Serial.println("Final string: "+settm);

  if(sendATCommand(settm,true))  return true;
  else  {
    if(sendATCommand(settm,true)) return true;
  }
 
 return false;
}


bool SIM800init(void)  {
  if ( sendATCommand("AT", true)&&     
 sendATCommand("AT+CSCLK=0", true) &&          // wakeup module if return from sleep mode
  sendATCommand("AT+IPR=9600", true)&&        // set internal port baud rate 
    sendATCommand("AT+CNMI=2,2,0,0", true) &&  // "AT+CNMI=1,2,0,0,0" - wouldn't store incoming SMS in memory
      sendATCommand("AT+CMGF=1;&W", true))  //SMS TEXT and store it in memory
  return true;
  else return false;
}

bool SendSMSReport(String report)  {

String Date="Date:";
String Voltage=" Bat.voltg:"; 
String Relay=" Relay status:";
String Power=" Power state:";
String Wifist="  Wifi state:";

    if(sendATCommand("AT+CCLK?",true)) 
    Date+= _response.substring(_response.indexOf("+CCLK:")+7,_response.lastIndexOf("\"\r")+1);
    Voltage.concat(getBatteryVoltage());
    if(MyRtcData.BootMode==0)     {
    Relay.concat(relay->getRelay() ? "on" : "off");
    Power.concat(sensor_state ? "on" : "off");
    Wifist.concat(isWifiReady() ? "on" : "off");
    }
    if(MyRtcData.BootMode==1)     {
//      Power.concat(sensor_state ? "on" : "off");
      if(check_power()==0) Power+="off "; else Power+="on ";
      if(digitalRead(MyRtcData.RelayPin)==HIGH) Relay+="on ";
      if(digitalRead(MyRtcData.RelayPin)==LOW) Relay+="off ";
  //    Relay.concat(digitalRead(MyRtcData.RelayPin) ? "on" : "off");
      Wifist+="off ";
    }
      if(!disable_SMS) {
        if(sim800SendSMS(phones,(report+Date+Voltage+Relay+Power+Wifist))) return true; 
        else { MppSerial.println("SMS service stopped!"); return false;}
      }
      MppSerial.println("SMS disabled");
      return false;
}

void sim800Reset() {
  /* Reset SIM800L */
  MppSerial.println(F("Reset SIM800L ..."));

  digitalWrite(mppServer.getUnsignedProperty(P_MODEM_PIN), LOW);
  delay(200);
  digitalWrite(mppServer.getUnsignedProperty(P_MODEM_PIN), HIGH);
}

bool sendATCommand(String cmd, bool waiting) {
                       
//  MppSerial.println(cmd);                          
  SIM800.println(cmd);                          
  if (waiting) {                                
   if(!waitResponse(SIMTIMEOUT)) return false;  
   
    if(_response.indexOf("OK")) { 
      MppSerial.println("Response:"+_response);                
      return true;
    }
     if(_response.indexOf("ERROR")) { 
      MppSerial.println("ERROR Response:"+_response);                
      return false;
  }
  
 }                               
}

bool waitResponse(int timeout) {                         // Waiting response from SIM800
                      
//  long _timeout = millis() + (long)timeout;             //*1000 timeout in sec to millisec
  unsigned long timeOld = millis();
 // unsigned int pCount;
 // char in, *p = NULL;
//  char *_resp;

 while (!SIM800.available() && (millis() - timeOld) < (timeout * 1000)) { delay(10); } 
 //  while (!SIM800.available() && millis() < _timeout)  {yield();}
  if (SIM800.available()) {                     
    _response = SIM800.readString();                //store response 
//   MppSerial.println(_response);
//   MppSerial.println(_response);
    return true;
  } else {
    MppSerial.println("Timeout...");   
    return false; }         
//  Serial.println(_response);
   MppSerial.println(_response);
    return true;  
}

/*char* waitResponse(unsigned long timeout) { 
 unsigned int pCount;
  char in, *p = NULL;

  unsigned long timeOld = millis();

  while (!SIM800.available() && (millis() - timeOld) < (timeout * 1000)) {
    delay(10);
  }

  if (SIM800.available()) {
    for (pCount = 0; SIM800.available(); pCount++) {
      if (pCount == 0 || (pCount % DEFAULT_BUFFER_SIZE) == 0) {
        p = (char*)realloc(_responsen, DEFAULT_BUFFER_SIZE + pCount);
        if (!p) {
          Serial.println(F("Error realloc in sim800WaitResponse()"));
          free(_responsen);
          _responsen = NULL;
          return NULL;
        }
      }
      _responsen = p;
      in = SIM800.read();
      if (pCount == 0 && (in == '\r' || in == '\n' || in == ' ')) {
        pCount--;
        continue;
      }
      p[pCount] = in;
      delay(1);
    }
    p[pCount] = 0;
  } else {
    Serial.println(F("Timeout..."));
    sim800Reset();
  }

  return p;
}
*/
bool sim800SendSMS(String phone, String message) {
 if (sendATCommand("AT+CMGS=\"" + phone + "\"", true) &&             // Set text mode 
  sendATCommand(message + "\r\n" + (String)((char)26), true)) return true; //send message
    else return false;
}

int GeneratePassword(void) {
int currentNumberOfDigits = 0,temppaswd;
beg:
  srand(millis());
  int paswd=temppaswd=rand();
  while (temppaswd != 0) {
 temppaswd = temppaswd/10;
 currentNumberOfDigits++;
}
  if(currentNumberOfDigits<5) goto beg;
  return paswd;
}

String UCS2ToString(String s) {                       // DEcode UCS string
  String result = "";
  unsigned char c[5] = "";                            // 
  for (int i = 0; i < s.length() - 3; i += 4) {       // Get 4 simbols from code 
    unsigned long code = (((unsigned int)HexSymbolToChar(s[i])) << 12) +    // GET UNICODE from HEX
                         (((unsigned int)HexSymbolToChar(s[i + 1])) << 8) +
                         (((unsigned int)HexSymbolToChar(s[i + 2])) << 4) +
                         ((unsigned int)HexSymbolToChar(s[i + 3]));
    if (code <= 0x7F) {                               // Make regular simbol
      c[0] = (char)code;                              
      c[1] = 0;                                       // add 0
    } else if (code <= 0x7FF) {
      c[0] = (char)(0xC0 | (code >> 6));
      c[1] = (char)(0x80 | (code & 0x3F));
      c[2] = 0;
    } else if (code <= 0xFFFF) {
      c[0] = (char)(0xE0 | (code >> 12));
      c[1] = (char)(0x80 | ((code >> 6) & 0x3F));
      c[2] = (char)(0x80 | (code & 0x3F));
      c[3] = 0;
    } else if (code <= 0x1FFFFF) {
      c[0] = (char)(0xE0 | (code >> 18));
      c[1] = (char)(0xE0 | ((code >> 12) & 0x3F));
      c[2] = (char)(0x80 | ((code >> 6) & 0x3F));
      c[3] = (char)(0x80 | (code & 0x3F));
      c[4] = 0;
    }
    result += String((char*)c);                       // add simbol to result string
  }
  return (result);
}

unsigned char HexSymbolToChar(char c) {
  if      ((c >= 0x30) && (c <= 0x39)) return (c - 0x30);
  else if ((c >= 'A') && (c <= 'F'))   return (c - 'A' + 10);
  else                                 return (0);
}

String getFiguresFromString(String str) {            // Get figures from USSD balance
  bool   flag     = false;
  String result   = "";
  str.replace(",", ".");                          // Change coma to dot
  for (int i = 0; i < str.length(); i++) {
    if (isDigit(str[i]) || (str[i] == (char)46 && flag)) { // If figures appears 
      result += str[i];                           // gathering the string 
      if (!flag) flag = true;                     // 
    }
    else {                                        // 
      if (flag) break;                            // 
    }
  }
  return result;                        // 
}

bool getSIMBalance(void)   {
  if(sendATCommand("AT+CUSD=1,\"*100#\"", true)) return true;
  else return false;
  
}

String parseSimBalance(String balancestring)
{
  String balUCS="";
   balUCS= balancestring.substring(balancestring.indexOf("\"")+1,balancestring.lastIndexOf("\"")+1);
 return getFiguresFromString(UCS2ToString(balUCS))+" ";
}


bool parseSMS(String msg) {                                   /// Parse incoming SMS 

  String msgbody    = "";
  String msgphone   = "";


if(msg.startsWith("+CMT: \"")) {
    msgphone = msg.substring(msg.indexOf("\"")+1,msg.indexOf("\",\""));
    msgbody = msg.substring(msg.indexOf("\r")+1);
  MppSerial.println("Phone: " + msgphone);                       
  MppSerial.println("Message: " + msgbody);   
} else {MppSerial.println("Wrong SMS!"); return false;    }              

  if (msgphone.length() < 10 || phones.indexOf(msgphone) == -1) {
        MppSerial.println("Unknown phone or broken number");
    return false;
    }

     msgbody = msgbody.substring(msgbody.indexOf("PASS")+4,msg.lastIndexOf("\r")+1);
      if((int)msgbody.toInt()!=MyRtcData.password) {
        MppSerial.println("Password wrong!:"+msgbody);
        MppSerial.printf("RTC password:%d\n",MyRtcData.password);
        sim800SendSMS(phones,"Wrong password!");
        return false;
      }
      else MppSerial.println("Password  matched !:"+msgbody);

  msgbody = msg.substring(msg.indexOf("OK")+2,msg.lastIndexOf("PASS")); //Parse command ofter OK and before PASS(1 symbol)
     int command=(int)msgbody.toInt();
     MppSerial.printf("Command :%d or :",command);
     MppSerial.println(msg);
   
 switch (command) {   // switch relay off - 0
  case 0:
    Relay_Off=true;
    break;
   case 1:
    Relay_On=true;     // switch relay on -1 
    break;
   case 2:
    Relay_OFFON=true;     // switch relay OFF and then ON -2 
    break;
    case 3:
    Report=true;    // send a report 3;
    break;
    case 4:
    Balance=true;   //get and send balance
    break;
    case 5:         // reboot device command
    RebootDevice=true;
    break;
 }
    return true;
 }


uint32_t calculateCRC32(const uint8_t *data, size_t length) {// CRC of RTC data calculation 
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;  // unit_32 polynomials
      }
    }
  }
  return crc;
}

unsigned check_power(void)
{
  pinMode(14,INPUT);pinMode(12,INPUT);  // Reserved for reading TP4056 
  
    if(digitalRead(12)==HIGH&&digitalRead(14)==HIGH) return 0;  // no power 
      if(digitalRead(12)==LOW&&digitalRead(14)==HIGH) return 1; // power OK charging
        if(digitalRead(12)==HIGH&&digitalRead(14)==LOW) return 2; // power OK charged
          if(digitalRead(12)==LOW&&digitalRead(14)==LOW) return 3; // power OK battery failed or disconnected
}

void goToSleep(unsigned stime, unsigned bmode) {
 MyRtcData.RunningTime += (uint32_t) stime / 600000ull + millis()/1000;
   MyRtcData.counter++;
    MyRtcData.b_voltage=getBatteryVoltage(); //
    MyRtcData.crc32 = calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4);
      ESP.rtcUserMemoryWrite(16, (uint32_t*) &MyRtcData, sizeof(MyRtcData));
   switch(bmode)   {
         case 0:
      MppSerial.printf("Falling asleep for %d msec! Voltage:%4.2f counter:%d boot mode:%d\n",stime,(MyRtcData.b_voltage),MyRtcData.counter,MyRtcData.BootMode);
       delay(100); // short delay before sleep
        ESP.deepSleep( (uint64) stime ,WAKE_RF_DISABLED ); //wake up after P_SLEEP with wifi
    break;

 //  ESP.deepSleep(20000000, WAKE_RF_DISABLED); // test 20s  WAKE_RF_DEFAULT - WAKE_RF_DISABLED
    case 1:
       if(sendATCommand("AT+CSCLK=2",true)) Serial.println("SIM800 fall asleep");
        MppSerial.printf("Falling max sleep ! Voltage:%f\n",stime,(MyRtcData.b_voltage));
         delay(100); // short delay before sleep
           ESP.deepSleep( ESP.deepSleepMax(), WAKE_RF_DISABLED ); //  sleep for 3H wake up no wifi (uint64)6480000000000ull
    break;
    case 2:     // Restart in wifi mode 
      MppSerial.println("Restart in normal mode!");
        delay(50);
          //ESP.reset(); 
          ESP.restart();
     break;
  }
}


// otherwise causes a problem when properties are cleared...
int getPeriod() {
  if (MyRtcData.BootMode==1) return 10; // set 10 sec time interval in battery mode
  else 	return mppServer.getUnsignedProperty(P_PERIOD) < 1 ?
			20 : mppServer.getUnsignedProperty(P_PERIOD);
}



//The setup function is called once at startup of the sketch
void setup() {
  String pwdmsg="User:admin Device password:";
  bool stm=false;

  Serial.begin(115200);  
  resetInfo = ESP.getResetInfoPtr();
 if (ESP.rtcUserMemoryRead(16, (uint32_t*) &MyRtcData, sizeof(MyRtcData))) {
    MppSerial.printf("\n RTC Data Counter:%d, Timer:%d CRC saved:%Lu, CRC calculated:%Lu\n",MyRtcData.counter, MyRtcData.RunningTime,
          MyRtcData.crc32,calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4));
  }
  if (MyRtcData.crc32 != calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4)||MyRtcData.counter==0) {  //RTC data initialisation if CRC doesn't math or Initialisation for first start
    MppSerial.printf("CRC Data Corrupted! Or first Start CRC: %Lu Stored CRC:%Lu Calculated. Initialisation!\n",MyRtcData.crc32,calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4));
    MyRtcData.RunningTime = 0;      
    MyRtcData.counter = 0;
    MyRtcData.b_voltage = getBatteryVoltage();
    MppSerial.printf("!voltage:%f\n",(MyRtcData.b_voltage));
    MyRtcData.power_fault = false;
    MyRtcData.password =GeneratePassword();
    NewPass=true;
    MyRtcData.BootMode=0;
      MyRtcData.crc32 = calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4);
      ESP.rtcUserMemoryWrite(16, (uint32_t*) &MyRtcData, sizeof(MyRtcData));
  }
    else MppSerial.println("CRC Data OK"); 

  MppSerial.printf("Wakes up in setup,voltage:%f  fault flag:%d reset reason:%d\n",getBatteryVoltage(),MyRtcData.power_fault,resetInfo->reason);
  MppSerial.printf("reboot counter:%d running time%d  Current power:%d Reboot reason:%d Boot mode:%d \n",MyRtcData.counter,MyRtcData.RunningTime,check_power(),resetInfo->reason,MyRtcData.BootMode);

   if(check_power()>0 && MyRtcData.BootMode==3)     {    // Wake up after endless sleep by reset button and trying to continue in normal mode
     MppSerial.println("Wakes up endless sleep , power restored!");
           MyRtcData.power_fault==false; //Reset RTC counters for bad power
            MyRtcData.RunningTime = 0;     
            MyRtcData.counter = 0;
            MyRtcData.BootMode==0;
            MyRtcData.crc32 = calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4);
            ESP.rtcUserMemoryWrite(16, (uint32_t*) &MyRtcData, sizeof(MyRtcData)); 
   }

   if (resetInfo->reason ==1 || resetInfo->reason ==0 || resetInfo->reason ==6) // regenerate password at each hard reboot //
    {
      MyRtcData.password =GeneratePassword();
      NewPass=true;
      MppSerial.printf("New password generated! rebot reason:%d/%s\n",resetInfo->reason,ESP.getResetReason().c_str());
      MyRtcData.crc32 = calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4);
      ESP.rtcUserMemoryWrite(16, (uint32_t*) &MyRtcData, sizeof(MyRtcData));
    }
  
    
   if(check_power()>0 && MyRtcData.BootMode==1)     {      //Power has returned in no wifi mode time to reboot in normal mode
    MyRtcData.BootMode=0;
     MppSerial.println("Power restored ,trying to restart in normal mode..."); 
       delay(100);   // delay before falling asleep
        goToSleep(0,2); // go to reboot in wifi mode    
    }

   if( resetInfo->reason == 5 && MyRtcData.BootMode==0 && check_power()>0)   { //Wakes up after deep sleep , power restored! Continue normal cycle
      MppSerial.println("Wakes up after deep sleep , power restored!");
           MyRtcData.power_fault==false; //Reset RTC counters for bad power
            MyRtcData.RunningTime = 0;     
            MyRtcData.counter = 0;
            MyRtcData.BootMode==0;
            MyRtcData.crc32 = calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4);
            ESP.rtcUserMemoryWrite(16, (uint32_t*) &MyRtcData, sizeof(MyRtcData)); 
  
    }


 
if (MyRtcData.BootMode==0)    {            // Normal boot mode or return to normal mode
                 

 mppServer.setPropertyDefault(P_MODEM_PIN, "0");   // GPIO 0 for modem reset
  if (mppServer.hasProperty(P_MODEM_PIN)){
  pinMode(mppServer.getUnsignedProperty(P_MODEM_PIN), OUTPUT);
  digitalWrite(mppServer.getUnsignedProperty(P_MODEM_PIN),HIGH);
  }
 
    pinMode(14,INPUT);pinMode(12,INPUT);  // Reserved for reading TP4056 

if (mppServer.hasProperty(P_TIMEOUT_SIM)){      // set timeout for SIM800 from user 
  #undef  SIMTIMEOUT
  #define SIMTIMEOUT mppServer.getUnsignedProperty(P_TIMEOUT_SIM)
}

   // set a default for the relay 
 mppServer.setPropertyDefault(P_RELAY_PIN, "15");
 RelayPin = mppServer.getUnsignedProperty(P_RELAY_PIN);
 pinMode(mppServer.getUnsignedProperty(P_RELAY_PIN), OUTPUT);

  relay = new class MppRelay(RelayPin,
      mppServer.getUnsignedProperty(P_MOMENTARY),
      mppServer.isProperty(P_INITIAL));
if(MyRtcData.RelayPin!=RelayPin)  {
MyRtcData.RelayPin=mppServer.getUnsignedProperty(P_RELAY_PIN);
MppSerial.printf("Relay pin stored:%d\n",MyRtcData.RelayPin);
MyRtcData.crc32 = calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4);
      ESP.rtcUserMemoryWrite(16, (uint32_t*) &MyRtcData, sizeof(MyRtcData));
}      
  mppServer.setPropertyDefault(P_BATTERY_TRESHOLD, "3.0"); // the lowest value of the battery voltage when device is going to sleep for MAX time
//  mppServer.setPropertyDefault(P_SLEEP_TIME, "2"); // Deep sleep for 2 min
  mppServer.setPropertyDefault(P_PERIOD, "10");

mppServer.setPropertyDefault(P_INT_ALARM, "13");
int Al_Sen_Pin = mppServer.getUnsignedProperty(P_INT_ALARM);
 Int_alarm = new class MppSensor(Al_Sen_Pin,false,false);
  
 // pinMode(mppServer.getUnsignedProperty(P_EXT_ALARM), INPUT);
 
  
//   mppServer.setPropertyDefault(P_WIFI_RESTART, "5");  // restarts device after n minutes if wifi connection  failed

      // identify the managed devices
  mppServer.manageDevice(&battery, getDefaultUDN(MppAnalog)); // Battery voltage device
  mppServer.manageDevice(&ACsensor, getDefaultUDN(MppSensor)); // AC power sensor
  mppServer.manageDevice(relay, getDefaultUDN(MppMomentary)); // relay
  mppServer.manageDevice(Int_alarm, getDefaultUDN(MppSensor)+ "_" + "IntA");   // register internal sensor for alarm system
  mppServer.manageDevice(&Alarm_signal,getDefaultUDN(MppSwitch)+"_AlarmF"); // register external flag device for other Mpp devices
  mppServer.manageDevice(&Alarm_flag, getDefaultUDN(MppSwitch) + G_ALARM_FLAG); // register the flag device with the MppServer for AM to see it 
  
if (mppServer.isProperty(P_USE_LAST))
    relay->setRelayHandler([](bool relayState, unsigned relayPin) {
      (void) relayPin;
      mppServer.putProperty(P_INITIAL, relayState ? "true" : "false");
    });

  if (mppServer.hasProperty(P_INITIAL))
    relay->setRelay(mppServer.isProperty(P_INITIAL), 0);

  MppSerial.printf("\nMppServer booting: %s, mode=%d, version=%d\n",
      ESP.getResetReason().c_str(), ESP.getBootMode(),
      ESP.getBootVersion());


  // start the web and mpp server
  mppServer.begin();


      
t_reboot=5*getPeriod();       // set time for refresh GPIO state as 5 CHECKIN cycles but no more than 10s
if(t_reboot>5) t_reboot=5;
}
                        
  SIM800.begin(9600);                           


  
  if(SIM800init()) {disable_SMS = false; Serial.println("SIM800 Started!");}
  else { 
    sim800Reset();
    delay(1000);
    if(SIM800init()) {disable_SMS = false; Serial.println("SIM800 Started!");}
    else {
    MppSerial.println("SIM800 Can't be initalised!"); 
    disable_SMS = true; 
    }
  }

if (MyRtcData.BootMode==0)    {   
   if(mppServer.hasProperty(P_PHONE)&& !disable_SMS) {
     phones=mppServer.getProperty(P_PHONE);   // Phone number from device http page for storing as contact
     int phlenght=phones.length();
     MppSerial.printf("P_PHONE :%s  flag:%d RTC count:%d Password:%d lenght of phone:%d\n",mppServer.getProperty(P_PHONE), disable_SMS,MyRtcData.counter,MyRtcData.password,phones.length());
       if(strcmp(mppServer.getProperty(P_PHONE),MyRtcData.phone)==0)  {
          disable_SMS=false;
            MppSerial.println("Phone not changed"+phones);
   //         if(!mppServer.hasProperty(P_PASSWORD)) mppServer.putProperty(P_PASSWORD,String(MyRtcData.password).c_str());// setup password protection for device's page
              if(NewPass)   {
                pwdmsg+=MyRtcData.password;
                  sim800SendSMS(phones,pwdmsg);
                    MppSerial.println("New password has sent for phone:"+phones);
                      MppSerial.println("New password:"+pwdmsg);
                      NewPass=false;
              }
       }
       else   {
      memcpy(MyRtcData.phone,phones.c_str(),phlenght+1); // Store phone in RTC memory 
    if(sizeof(phones)>10)          {         // if phone number long enought (more 10 bytes) to be true 
     MyRtcData.crc32 = calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4);
      ESP.rtcUserMemoryWrite(16, (uint32_t*) &MyRtcData, sizeof(MyRtcData));
          pwdmsg+=MyRtcData.password;
                sim800SendSMS(phones,pwdmsg);
                  MppSerial.println("New password has sent for phone:"+phones);
                  NewPass=true;
                    disable_SMS=false;
 //                     mppServer.putProperty(P_PASSWORD,String(MyRtcData.password).c_str());
                        delay(50);
    }
      else      {               // new phone number too short to be true 
           disable_SMS=true;
           MppSerial.println("Phone not valid password hasn't send ");
      }
     }
   }
    else { 
      if(memcmp(MyRtcData.phone,"+",1)==0) {      // if AM server hasn't phone number but some old number stored in RTC
        MppSerial.println("Old telefon was in RTC- CLear!");
         phones=mppServer.getProperty(P_PHONE);     // get 0 phone number 
         strncpy(MyRtcData.phone,phones.c_str(),sizeof(phones)); // Clear phone in RTC memory  by empty 
     MyRtcData.crc32 = calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4);
      ESP.rtcUserMemoryWrite(16, (uint32_t*) &MyRtcData, sizeof(MyRtcData));
      mppServer.removeProperty(P_PASSWORD);  // REmove password property if phone deleted
      }
      disable_SMS=true; Serial.println("SMS are disabled , Contact phone didn't set");Serial.printf("RTC.PHONE:%s\n",MyRtcData.phone); 
      }

  


  if (mppServer.hasProperty(P_TIMESET ))   stm =setupTime(mppServer.getProperty(P_TIMESET));
  if(stm)  mppServer.removeProperty(P_TIMESET); 
 
 mode=oldmode=check_power();
 }
 if (MyRtcData.BootMode==1)       {  // copy phone number from RTC memory in noWifi mode
  mode=oldmode=3; // mode 3 - no power 
  phones=MyRtcData.phone;  // copy relevant phone number from RTC to phones
  if(phones.length()>=12) disable_SMS=false;  
  if(!disable_SMS)       {
    pwdmsg+=MyRtcData.password;
      pwdmsg+=" Power down , WIFI down, battery OK. System rebooted in safe mode";
          delay(1000); // allow SIM800 initialise after reboot
             if(resetInfo->reason !=2 && NewPass) // awoid sending SMS after exeptional reboot and only when new password has been generated
              sim800SendSMS(phones,pwdmsg); // send password once in no power no wifi mode
                    NewPass=false;
MppSerial.println("Message for phone:"+phones+pwdmsg);
   }
 }
}


void loop() {
  bool battery_state=true;
  bool notify = false;
	unsigned long now = millis(),nextsim;
  String url;

  if (SIM800.available())   {                   // if something where send by SIM800
     if(waitResponse(SIMTIMEOUT)){     // wait response SIM800 with X sec timeout
// MppSerial.println(_responsen);
//strcpy(const_cast<char*>(_response.c_str()),waitResponse(SIMTIMEOUT));
    _response.trim();                           // eliminate spaces 
    MppSerial.println(_response);}
    else Serial.println("No response from modem!");
   
    if (_response.startsWith("+CMT")&&!disable_SMS) {       // New SMS comes
      if (parseSMS(_response))                   // Parse unreaded SMS   
          sendATCommand("AT+CMGDA=\"DEL ALL\"", true); // Remove ALL SMS from SIM if successful read
    }
     if(_response.startsWith("RING")) {
    //   if(_response.indexOf(mppServer.getProperty(P_PHONE))) 
      sendATCommand("ATH0", true); // hangs off if somebody calls
     }  
        if(_response.startsWith("+CUSD"))   {                   
         SendSMSReport("Balance:"+parseSimBalance(_response));
        }
         if(_response.startsWith("UNDER"))   {                //UNDER-VOLTAGE Warning from SIM800 means voltage down at 3.5V 
            battery_state= false;
            disable_SMS=true;
            MppSerial.println("Battery dead! Ready to sleep!");
        }
   
   }
 if (Serial.available())  {                    
    SIM800.write(Serial.read());                
  }

 if(RebootDevice) {
  RebootDevice=false;
  goToSleep(0,2); // Restart device by SMS command
 }
  if(Report) {
       SendSMSReport("");
        Report=false; 
        }
  if(Relay_On) { 
    MppSerial.printf("Relay ON  , relaypin:%d\n",MyRtcData.RelayPin); 
    if(MyRtcData.BootMode==0)
          relay->setRelay(true,0);
    if(MyRtcData.BootMode==1)   {
      pinMode(MyRtcData.RelayPin,OUTPUT);
      digitalWrite(MyRtcData.RelayPin,HIGH);
    }
    Relay_On=false; 
    SendSMSReport("OK,");
              }
  if(Relay_Off) { 
    MppSerial.printf("Relay OFF , relaypin:%d\n",MyRtcData.RelayPin); 
     if(MyRtcData.BootMode==0) 
        relay->setRelay(false,0);
     if(MyRtcData.BootMode==1)    {
        pinMode(MyRtcData.RelayPin,OUTPUT);
        digitalWrite(MyRtcData.RelayPin,LOW);
     }
    Relay_Off=false; 
    SendSMSReport("OK,");
              }
    if(Relay_OFFON) { 
    MppSerial.printf("Relay ON and than OFF , relaypin:%d\n",MyRtcData.RelayPin); 
     if(MyRtcData.BootMode==0)  {
          relay->setRelay(true,0);
          delay(1000);
          relay->setRelay(false,0);
     }
      if(MyRtcData.BootMode==1)     {
        pinMode(MyRtcData.RelayPin,OUTPUT);
        digitalWrite(MyRtcData.RelayPin,HIGH);
        delay(1000);
        digitalWrite(MyRtcData.RelayPin,LOW);
      }
        
    Relay_OFFON=false; 
    SendSMSReport("RELAY RESWITCHED,");
              }
 if(Balance) {
       getSIMBalance();
        Balance=false; 
        }  

if(MyRtcData.BootMode==0)
	mppServer.handleClients(); // let the server handle any incoming requests
 
//	mppServer.handleCommand(); // optional, handle user Serial input
  
/*  if(now >=nextsim*SIMCHECK*60000) {  // Reset SIM if problem with cellular network registration.
    MppSerial.printf("Simcheck nextsim: %lus now:%lus \n",nextsim*SIMCHECK*60000 , now);
 //   Serial.printf("Simcheck: %lus now:%lus \n",next*SIMCHECK , now);
    if(!check_SIM()) {
      Serial.println("SIM doesn't registerd in network!Reset SIM!");
      sim800Reset(); 
    }
    nextsim=now;
  }
*/
  if(!battery_state&&!sensor_state){     // if battery discharged and power down
     MyRtcData.power_fault=true;          // remember power fault 
      MyRtcData.crc32 = calculateCRC32(((uint8_t*)&MyRtcData) + 4, sizeof(MyRtcData) - 4);
        ESP.rtcUserMemoryWrite(16, (uint32_t*) &MyRtcData, sizeof(MyRtcData));
        MyRtcData.BootMode==3;
            delay(100);   // delay before falling asleep
              goToSleep(0,1);  // sleep mode 1 - long sleep 3h.
       }


///*****************External ALARM BY FLAG***********************
if(Alarm_signal.getState()   ) {     // External Flag firing reaction 
  MppSerial.printf("Remote Alarm by flag: %s \n",Alarm_signal.getState()? "true" : "false");
//  Serial.printf("Remote Alarm by flag: %s \n",Alarm_signal.getState()? "true" : "false");
     Alarm_signal.setState(false);
     if(GlobalAlarm) {
      if(Sms_reported<millis()+ 60000){     // Don't send SMS alarm report often than 1 minute
      SendSMSReport("External Sensor ALARM! CHECK EMAIL! ");
      Sms_reported=millis();
      }
//        Alarm_signal.setState(false);
  MppSerial.printf("Remote Alarm in Global Alarm State by flag: %s \n",Alarm_signal.getState()? "true" : "false");
     }
 }
///**************Internal Alarm ****************************
  
     if(digitalRead(mppServer.getUnsignedProperty(P_INT_ALARM))==HIGH && GlobalAlarm && !Int_alarm_flag) { 
  //    if(millis()-alarm_time>20000) {     // Don't react alarm if less than 20 sec between next incoming alarm
  MppSerial.printf("Entrance Sensor ALARM!  GPIO13:%d\n",digitalRead(mppServer.getUnsignedProperty(P_INT_ALARM)));
   SendSMSReport("Entrance Sensor ALARM! CHECK EMAIL! ");
        Int_alarm_flag=true;
  //        alarm_time=millis();
 //     } else MppSerial.printf("Empty Entrance Sensor ALARM!  GPIO13:%d\n",digitalRead(mppServer.getUnsignedProperty(P_INT_ALARM)));
} 

///******************************************************************************************************
 
  if (now >= next) {
      if( Alarm_flag.getState())  GlobalAlarm=true;
      else { GlobalAlarm=false ;  Int_alarm_flag=false; }
/*   if(digitalRead(mppServer.getUnsignedProperty(P_EXT_ALARM)==LOW)) Int_alarm.put(STATE,"on");
    if(digitalRead(mppServer.getUnsignedProperty(P_EXT_ALARM)==HIGH)) Int_alarm.put(STATE,"off"); */
//    Serial.printf("Entrance Sensor  GPIO13:%d Sensor Int state:%s\n",digitalRead(13),Int_alarm->get(STATE)? "true" : "false");
   MppSerial.printf("G_Alarm_Flag %s  heap=%d mode:%d Reset info:%d Alarm Signal:%s\n",Alarm_flag.getState()? "true" : "false",ESP.getFreeHeap(),mode,resetInfo->reason,Alarm_signal.getState()? "true" : "false");


switch(check_power())    {           // power failed
    case 0:        
        sensor_state=false;
           mode=3;  
            if(oldmode!=mode) notify =true; else notify =false;
              MyRtcData.b_voltage=getBatteryVoltage(); 
    break;
    case 1:     // power  ok charging 
      mode=1;
          if(oldmode!=mode) notify =true; else notify =false;
              sensor_state=true;
  
      t_reboot=5*getPeriod();   // recharge GPIO reboot period
       if(t_reboot>5) t_reboot=5;  
          break;
    case 2:    // power ok battery charged
    mode=2;
      if(oldmode!=mode) notify =true; else notify =false;
      if(oldmode==3) mode=5;
      sensor_state=true; 
        t_reboot--;
         if(t_reboot==0) {
           pinMode(14,OUTPUT);pinMode(12,OUTPUT);  // Refresh GPIO state
          delay(20);
          digitalWrite(12,HIGH);
          digitalWrite(14,HIGH);
          delay(100);
           pinMode(14,INPUT);pinMode(12,INPUT);  // Return state of GPIO 
           MppSerial.println("GPIO rebooted!");
           t_reboot=5*getPeriod();
        if(t_reboot>5) t_reboot=5;  
         }
         break;
     case 3:      // battery not connected or failed but power On
     mode=4;
     MppSerial.println("Battery disconnected or failed!");
       break;
}        
 
 if(MyRtcData.BootMode==0)  {        
  if(isWifiReady())     {       // WiFi.status()==WL_CONNECTED&&WiFi.localIP()

 
    if(millis()-check_networks>600000)  {    // Check Cellular & Wifi status , signal strench every hour 3600000 ms ,setOutputPower(float dBm) - might work when signal is low , 20.5 -is max value
   String Cellular="Cellular:";
        if(check_SIM()) Cellular+="Registered,";
       else {Cellular+="Failed"; sim800Reset(); }
       if(sendATCommand("AT+CSQ",true)) // Asking GSM signal level
       Cellular+="Level:"+_response.substring(_response.indexOf("+CSQ:")+5,_response.indexOf(","));
        MppSerial.println("Whole string:"+Cellular+",WIFI:"+(String)WiFi.RSSI());
       relay->update("message", Cellular);  
       check_networks=millis();

    }
    
// MppSerial.printf("WIFI steel ready: now:%lus \n", now);
 switch (mode) {   
  case 3:    
    notify |=  ACsensor.update("message", String("POWER FAILED!").c_str());  
        break;
  case 1:  
    notify |=  ACsensor.update("message", String("POWER OK/CHARGING").c_str()); 
        break;
 case 2:       
    notify |=   ACsensor.update("message", String("POWER OK/CHARGED").c_str());
         break;
  case 4:
   notify |=   battery.update("message",String("POWER OK/BATTERY FAILED OR DISCONN.").c_str());
         break;
   case 5:
   notify |=   battery.update("message",String("POWER RESTORED/CHARGING.").c_str());
         break;       
 }

     if(!battery_state)     { 
       ACsensor.update("message", String("Power down/Battery discharged/Sleep!").c_str()); 
        ACsensor.notifySubscribers();
      }
      
// Serial.printf("Pin12:%d Pin14:%d notify:%d sensorState:%d mode:%d oldmode:%d T_reboot:%d battery_state:%d old_voltage:%f new_voltage:%f\n",digitalRead(12),digitalRead(14),notify,
//                                  sensor_state,mode, oldmode,t_reboot,battery_state,(MyRtcData.b_voltage),getBatteryVoltage());
 // Serial.printf("Wifi ready:%d , Wifi status: %d , Wifi mode:%d \n",isWifiReady(),WiFi.status(),WiFi.getMode());
if (notify) {
        ACsensor.notifySubscribers();
        notify=false;
    }
    
  	notify |= battery.update(VALUE, String(getBatteryVoltage()).c_str());
    notify |= ACsensor.update(STATE,sensor_state ? "on" : "off");



if (notify) {
   battery.update(STATE,battery_state ? "on" : "off");
      battery.notifySubscribers();
        ACsensor.notifySubscribers();
        notify=false;
    }

                                 

 }    else       {       // If WIFI not connected or disapeared
  if(WiFi.status()==1|| WiFi.status()!=3 || WiFi.getMode()!=2)   {                       // if wifi status = NO wifi or NO_AP found  or wifi mode != AP mode
    MppSerial.printf("WIFI disapeared!\n");
    if(!sensor_state&&!disable_SMS)   {            // if no wifi and no power , good battery  and SMS service enabled - try to send last SMS 
               if(MyRtcData.BootMode==0)   {
                SendSMSReport("SYSTEM FAILED! WIFI-OFF, POWER-OFF BATTERY OK "); // Send message when wifi is off
                  MppSerial.printf("SMS about SYSTEM FAILED! has sent, reboot in NOWIFI mode  boot mode:%d sensor:%d SMS:%d\n",MyRtcData.BootMode,sensor_state,disable_SMS);
                     MyRtcData.BootMode=1; 
                      delay(100);   // delay before falling asleep
                        goToSleep(0.1*60000000ull,0); // go to short sleep 10sec and reboot in no wifi mode
        } else  MppSerial.printf(" heap=%d ",ESP.getFreeHeap()); 
      }
   if(MyRtcData.BootMode==0 && sensor_state&&!disable_SMS && !SMS_sent)   {            // if no wifi but power on , good battery  and SMS service enabled - try to send last SMS    
     MppSerial.println("SMS SYSTEM FAILED! has sent, power ON but wifi disapeared ");
                 SendSMSReport("SYSTEM FAILED! WIFI-OFF, POWER-OK BATTERY OK "); // Send message when wifi is off
                      SMS_sent= true;
      }
    }
 }
}
    if(sensor_state && MyRtcData.BootMode==1)     {      //Power has returned in no wifi mode
    MyRtcData.BootMode=0;
     MppSerial.println("Power restored ,trying to restart in normal mode..."); 
  SendSMSReport("Power returned! Reboot for normal service"); // Send message when wifi is off but power returned
      delay(100);   // delay before falling asleep
        goToSleep(0.1*60000000ull,2); // go to short sleep 10 sec and reboot in wifi mode
    }

     
   	 MppSerial.printf("Current Mode:%d sensor:%d bat_voltg:%f heap=%d\n",MyRtcData.BootMode,sensor_state,getBatteryVoltage(), ESP.getFreeHeap()); 
     if(ESP.getFreeHeap()<1000 && MyRtcData.BootMode==1) goToSleep(0,2); // Reboot device if memory leaks
    next = now + getPeriod() * 1000;
    oldmode=mode;
   	
   	}
}
