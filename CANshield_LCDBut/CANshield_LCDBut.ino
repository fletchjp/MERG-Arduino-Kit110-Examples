// CANshield_LCDBut
// This example is designed to work with the Arduino CAN shield (MERG Kit 110) with an Arduino UNO.
// The Kit 110 shield is placed on the Arduino UNO and a second shield placed on top.
// https://www.dfrobot.com/product-51.html (DFR0009)
// This shield has a 2 by 16 character LCD display and a set of buttons.
// The code allows for CBUS events to be sent using the buttons.
// What the button actions do can be configured.
// Currently the display shows the button actions.
// It can be extended to respond to incoming CBUS events.
//////////////////////////////////////////////////////////////////////////////////
// NOTE: This code does not support the Button and LEDs for CBUS configuration.
//       This is because the display uses the same pins.
//       In any case, the button and LEDs are hidden behind the display shield.
///////////////////////////////////////////////////////////////////////////////////
// Pin Use map UNO:
// Digital pin 2          Interupt CAN
// Digital pin 3 (PWM)    Not used
// Digital pin 4          LCD pin_d4
// Digital pin 5 (PWM)    LCD pin_d5
// Digital pin 6 (PWM)    LCD pin_d6
// Digital pin 7          LCD pin_d7
// Digital pin 8          LCD pin_RS
// Digital pin 9 (PWM)    LCD pin_EN
// Digital pin 10         LCD backlight pin - this would normally be for the CS
// Digital pin 11 (MOSI)  SI    CAN
// Digital pin 12 (MISO)  SO    CAN
// Digital pin 13 (SCK)   Sck   CAN

// Digital pin 14 / Analog pin 0  Analog input from buttons
// Digital pin 15 / Analog pin 1 (SS)    CS    CAN
// Digital pin 16 / Analog pin 2  Not used
// Digital pin 17 / Analog pin 3  Not used
// Digital / Analog pin 4     Not Used - reserved for I2C
// Digital / Analog pin 5     Not Used - reserved for I2C
//////////////////////////////////////////////////////////////////////////

// IoAbstraction libraries
#include <IoAbstraction.h>
#include <DfRobotInputAbstraction.h>
#include <TaskManagerIO.h>
#include <DeviceEvents.h>
#include <LiquidCrystalIO.h>

/// This uses the default settings for analog ranges.
IoAbstractionRef dfRobotKeys = inputFromDfRobotShield();

/// It is in fact set as default defining dfRobotKeys.
#define ANALOG_IN_PIN A0

// 3rd party libraries
#include <Streaming.h>
#include <Bounce2.h>

////////////////////////////////////////////////////////////////////////////
// CBUS library header files
#include <CBUS2515.h>    // CAN controller and CBUS class
#include <CBUSconfig.h>  // module configuration
//#include <CBUSLED.h>             // CBUS LEDs
#include <CBUSconfig.h>  // module configuration
#include <cbusdefs.h>    // MERG CBUS constants
#include <CBUSParams.h>  // CBUS parameters

////////////DEFINE MODULE/////////////////////////////////////////////////
/// Use these values for the CBUS outputs from the display shield buttons
/// These values give intial values which are not used elsewhere.
int button = -1;
int prevbutton = -1;

/// module name
unsigned char mname[7] = { 'L', 'C', 'D', 'B', 'u', 't', ' ' };

/// constants
const byte VER_MAJ = 2;     // code major version
const char VER_MIN = 'a';   // code minor version
const byte VER_BETA = 1;    // code beta sub-version
const byte MODULE_ID = 81;  // CBUS module type for CANshield

const unsigned long CAN_OSC_FREQ = 16000000UL;  // Oscillator frequency on the Kit 110 shield

#define NUM_LEDS 0      // How many LEDs are there?
#define NUM_SWITCHES 0  // How many switchs are there?


#define NUM_NVS 10
// This defines an array to hold the state of each button.
byte buttonState[NUM_NVS];

//////////////////////////////////////////////////////////////////////////

//CBUS pins
const byte CAN_INT_PIN = 2;
const byte CAN_CS_PIN = 15;  // Changed from 10 because of the LCD shield.

// CBUS objects
CBUSConfig modconfig;       // configuration object
CBUS2515 CBUS(&modconfig);  // CBUS object
CBUSLED ledGrn, ledYlw;     // two LED objects
CBUSSwitch pb_switch;       // switch object

// forward function declarations
void eventhandler(byte, CANFrame*);
void processSerialInput(void);
void printConfig(void);


// Index values for errors
enum errorStates {
  blankError,
  noError,
  emergencyStop,
  CANbusError,
  invalidError
};

// Index values for incoming event processing
// enum base changed to avoid other events.
// These are ideas at the moment.
enum eventTypes {
  nonEvent = 100,  // not used
  testEvent,
  emergencyEvent,
  sendFailureEvent,
  dataEvent,
  requestEvent,
  invalidEvent
};

//////////////////////////////////////////////////////////////////////////////////////////////

// LCD shield with buttons example code

/*
Arduino 2x16 LCD - Detect Buttons
modified on 18 Feb 2019
by Saeed Hosseini @ Electropeak
https://electropeak.com/learn/
Using data from here:
https://wiki.dfrobot.com/Arduino_LCD_KeyPad_Shield__SKU__DFR0009_
I have also played with the values.
*/
//#include <LiquidCrystal.h>
//LCD pin to Arduino
const int pin_RS = 8;
const int pin_EN = 9;
const int pin_d4 = 4;
const int pin_d5 = 5;
const int pin_d6 = 6;
const int pin_d7 = 7;
const int pin_BL = 10;
LiquidCrystal lcd(pin_RS, pin_EN, pin_d4, pin_d5, pin_d6, pin_d7);

int x;
int prevx = 0;
int range;
int prevrange = 0;
int y = 0;

// This is following the methods in EzyBus_master to provide error messages.
// These have been limited to 16 chars to go on an LCD 2 by 16 display.
// blank_string is used to cancel an error message.
const char blank_string[] PROGMEM = "                ";
const char error_string_0[] PROGMEM = "no error";
const char error_string_1[] PROGMEM = "Test message";
const char error_string_2[] PROGMEM = "Emergency Stop";
const char error_string_3[] PROGMEM = "CANbus error";
const char error_string_4[] PROGMEM = "invalid error";

const char* const error_string_table[] PROGMEM = {
  blank_string, error_string_0, error_string_1, error_string_2, error_string_3, error_string_4
};

#define MAX_ERROR_NO 5

// Buffer for string output.
// This has been made safe for line termination.
#define MAX_LENGTH_OF_STRING 16
#define LENGTH_OF_BUFFER (MAX_LENGTH_OF_STRING + 1)
char error_buffer[LENGTH_OF_BUFFER];

void getErrorMessage(int i);

// This is new in this version of the code and may be useful elsewhere.
// It is used to transfer error details to the DrawingEvent
// which is why it is declared here.
struct Error {
  int i;
  byte x;
  byte y;
  Error()
    : i(0), x(0), y(0) {}
  Error(int ii, byte xx, byte yy)
    : i(ii), x(xx), y(yy) {}
  Error(const Error& e)
    : i(e.i), x(e.x), y(e.y) {}
};

/**
 * Here we create an event that handles all the drawing for an application, in this case printing out readings
 * of a sensor when changed. It uses polling and immediate triggering to show both examples
 */
class DrawingEvent : public BaseEvent
{
private:
  volatile bool emergency;  // if an event comes from an external interrupt the variable must be volatile.
  bool hasChanged;
  bool hasKey;
  char key[7];
  bool hasError;
  Error error;
public:
  /** This constructor sets the initial values for various variables. */
  DrawingEvent()
  {
    hasChanged = false;
    hasKey = false;
    //key = "      ";
    hasError = false;
  }
  /**
     * This is called by task manager every time the number of microseconds returned expires, if you trigger the
     * event it will run the exec(), if you complete the event, it will be removed from task manager.
     * @return the number of micros before calling again. 
     */
  uint32_t timeOfNextCheck() override
  {
    setTriggered(hasChanged);
    return millisToMicros(500);  // no point refreshing more often on an LCD, as its unreadable
  }

  /**
     * This is called when the event is triggered, it prints all the data onto the screen.
     * Note that each source of input has its own bool variable.
     * This ensures that only the items needing output are executed.
     */
  void exec() override
  {
    hasChanged = false;

    if (hasKey) {
      hasKey = false;
      lcd.setCursor(10, 1);
      lcd.print(key);
    }
    if (hasError) {
      getErrorMessage(error.i);
      lcd.setCursor(error.x, error.y);
      lcd.write("E: ");
      lcd.write(error_buffer);
      hasError = false;
    }
  }

  /* This provides for the logging of the key information
       This is an example of something coming from an internal event. */
  void drawKey(const char* whichKey)
  {
    memcpy(key, whichKey, 7);  //= whichKey;
    hasKey = true;
    hasChanged = true;  // we are happy to wait out the 500 millis
  }
  /* This provides for the logging of the error information.
     * This is an example of something coming from an external event.
     * The Error object holds the data for plotting. */
  void displayError(const Error& e)
  {
    error = e;
    hasError = true;
    hasChanged = true;  // we are happy to wait out the 500 millis
  }
  /**
     * Triggers an emergency that requires immediate update of the screen
     * @param isEmergency if there is an urgent notification
     * This is not used at present and is included from the source example.
     */
  void triggerEmergency(bool isEmergency)
  {
    emergency = isEmergency;
    markTriggeredAndNotify();  // get on screen asap.
  }
};

// create an instance of the above class
DrawingEvent drawingEvent;

// Add check for invalid error
void getErrorMessage(int i)
{
  if (i >= 0 && i <= MAX_ERROR_NO) {
    strncpy_P(error_buffer, (char*)pgm_read_word(&(error_string_table[i])), MAX_LENGTH_OF_STRING);
  } else {
    strncpy_P(error_buffer, (char*)pgm_read_word(&(error_string_table[MAX_ERROR_NO])), MAX_LENGTH_OF_STRING);
  }
}

void serialPrintError(int i)
{
  getErrorMessage(i);
  Serial.print(error_buffer);
}
void serialPrintErrorln(int i)
{
  getErrorMessage(i);
  Serial.println(error_buffer);
}


//
/// setup CBUS - runs once at power on from setup()
//
void setupCBUS()
{

  // set config layout parameters
  modconfig.EE_NVS_START = 10;
  modconfig.EE_NUM_NVS = NUM_NVS;
  modconfig.EE_EVENTS_START = 50;
  modconfig.EE_MAX_EVENTS = 32;
  modconfig.EE_NUM_EVS = 1;
  modconfig.EE_BYTES_PER_EVENT = (modconfig.EE_NUM_EVS + 4);

  // initialise and load configuration
  modconfig.setEEPROMtype(EEPROM_INTERNAL);
  modconfig.begin();

  Serial << F("> mode = ") << ((modconfig.FLiM) ? "FLiM" : "SLiM") << F(", CANID = ") << modconfig.CANID;
  Serial << F(", NN = ") << modconfig.nodeNum << endl;

  // show code version and copyright notice
  printConfig();

  // set module parameters
  CBUSParams params(modconfig);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setModuleId(MODULE_ID);
  params.setFlags(PF_FLiM | PF_COMBI);

  // assign to CBUS
  CBUS.setParams(params.getParams());
  CBUS.setName(mname);

  // register our CBUS event handler, to receive event messages of learned events
  CBUS.setEventHandler(eventhandler);

  // configure and start CAN bus and CBUS message processing
  CBUS.setNumBuffers(2, 2);               // more buffers = more memory used, fewer = less
  CBUS.setOscFreq(CAN_OSC_FREQ);          // select the crystal frequency of the CAN module
  CBUS.setPins(CAN_CS_PIN, CAN_INT_PIN);  // select pins for CAN bus CE and interrupt connections

  if (!CBUS.begin()) {
    Serial << F("> error starting CBUS") << endl;
  }
}

void setupModule()
{
  for (int i = 0; i < NUM_NVS; i++) {
    buttonState[i] = false;
  }
}

void logKeyPressed(int pin, const char* whichKey, bool heldDown)
{
  drawingEvent.drawKey(whichKey);
  Serial.print("Key ");
  Serial.print(whichKey);
  Serial.println(heldDown ? " Held" : " Pressed");
  button = pin + 1;  // Increment to avoid event 0
}

// Serial IO
#define SERIAL_SPEED 115200  // Speed of the serial port.


/**
 * Along with using functions to receive callbacks when a button is pressed, we can
 * also use a class that implements the SwitchListener interface. Here is an example
 * of implementing that interface. You have both choices, function callback or
 * interface implementation.
 */
class MyKeyListener : public SwitchListener
{
private:
  const char* whatKey;
public:
  // This is the constructor where we configure our instance
  MyKeyListener(const char* what)
  {
    whatKey = what;
  }

  // when a key is pressed, this is called
  void onPressed(pinid_t pin, bool held) override
  {
    logKeyPressed(pin, whatKey, held);
    button = pin + 1;
  }

  // when a key is released this is called.
  void onReleased(pinid_t pin, bool held) override
  {
    Serial.print("Release ");
    logKeyPressed(pin, whatKey, held);
  }
};

MyKeyListener selectKeyListener("SELECT");

void setup1602()
{
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("CANshield LCDBut");
  lcd.setCursor(0, 1);
  lcd.print("Press Key:");
}

void setupSwitches()
{
  // initialise the switches component with the DfRobot shield as the input method.
  // Note that switches is the sole instance of SwitchInput
  switches.initialise(dfRobotKeys, false);  // df robot is always false for 2nd parameter.

  // now we add the switches, each one just logs the key press, the last parameter to addSwitch
  // is the repeat frequency is optional, when not set it implies not repeating.
  switches.addSwitch(
    DF_KEY_DOWN, [](pinid_t pin, bool held) {
      logKeyPressed(pin, "DOWN  ", held);
    },
    20);
  switches.addSwitch(
    DF_KEY_UP, [](pinid_t pin, bool held) {
      logKeyPressed(pin, "UP    ", held);
    },
    20);
  switches.addSwitch(
    DF_KEY_LEFT, [](pinid_t pin, bool held) {
      logKeyPressed(pin, "LEFT  ", held);
    },
    20);
  switches.addSwitch(
    DF_KEY_RIGHT, [](pinid_t pin, bool held) {
      logKeyPressed(pin, "RIGHT ", held);
    },
    20);
  //switches.onRelease(DF_KEY_RIGHT, [](pinid_t /*pin*/, bool) { Serial.println("RIGHT has been released");});

  switches.addSwitchListener(DF_KEY_SELECT, &selectKeyListener);
}

void setup()
{
  // put your setup code here, to run once:

  // Initialise
  while (!Serial)
    ;
  Serial.begin(SERIAL_SPEED);  // Start Serial IO.
  Serial << endl
         << endl
         << F("> ** CBUS LCD Button Arduino Shield ** ") << __FILE__ << endl;
  //analogWrite(pin_d6,50);
  setup1602();
  setupCBUS();
  setupModule();
  setupSwitches();

  // Schedule tasks to run every 250 milliseconds.
  taskManager.scheduleFixedRate(250, processSerialInput);
  taskManager.scheduleFixedRate(250, processButtons);

  // create any other tasks that you need here for your sketch

  taskManager.registerEvent(&drawingEvent);

  // end of setup
  Serial << F("> ready") << endl
         << endl;
}


void loop()
{
  // put your main code here, to run repeatedly:

  //
  /// do CBUS message, switch and LED processing
  //

  CBUS.process();

  //
  /// process console commands
  //

  processSerialInput();

  //
  /// check CAN message buffers
  //

  if (CBUS.canp->receiveBufferPeakCount() > CBUS.canp->receiveBufferSize()) {
    // Serial << F("> receive buffer overflow") << endl;
  }

  if (CBUS.canp->transmitBufferPeakCount(0) > CBUS.canp->transmitBufferSize(0)) {
    // Serial << F("> transmit buffer overflow") << endl;
  }

  //
  /// check CAN bus state
  //

  byte s = CBUS.canp->errorFlagRegister();

  if (s != 0) {
    // Serial << F("> error flag register is non-zero = ") << s << endl;
  }

  // Run IO_Abstraction tasks.
  // This replaces actions taken here in the previous version.
  taskManager.runLoop();
}

//
/// user-defined event processing function
/// called from the CBUS library when a learned event is received
/// it receives the event table index and the CAN frame
//

void eventhandler(byte index, CANFrame* msg)
{

  // as an example, display the opcode and the first EV of this event
  byte opc = msg->data[0];

  Serial << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(opc) << endl;
  Serial << F("> EV1 = ") << modconfig.getEventEVval(index, 1) << endl;

  unsigned int node_number = (msg->data[1] << 8) + msg->data[2];
  unsigned int event_number = (msg->data[3] << 8) + msg->data[4];
  Serial << F("> NN = ") << node_number << F(", EN = ") << event_number << endl;
  Serial << F("> op_code = ") << opc << endl;

  // Experimental code to display a message index on the event_number.
  if (event_number >= nonEvent) {
    switch (opc) {

      case OPC_ACON:
      case OPC_ASON:
        Serial << "Display error " << event_number - nonEvent << endl;
        drawingEvent.displayError(Error(event_number - nonEvent, 0, 0));
        break;

      case OPC_ACOF:
      case OPC_ASOF:
        Serial << "Display blank error " << endl;
        drawingEvent.displayError(Error(blankError, 0, 0));
        break;
      
      case OPC_ASON1:
        byte errorNo = msg->data[5];
        Serial << "Display error " << errorNo << endl;
        drawingEvent.displayError(Error(errorNo, 0, 0));
    }
  }

  return;
}
//
/// print code version config details and copyright notice
//

void printConfig(void)
{

  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Duncan Greenwood (MERG M5767) 2019") << endl;
  Serial << F("> © Martin Da Costa (MERG M6223) 2023") << endl;
  Serial << F("> © John Fletcher (MERG M6777) 2023") << endl;
  return;
}


void processButtons(void)
{
  // Send an event corresponding to the button.
  // Note: There could be a use of NVs to change the action depending on the setting.
  byte nvval;
  byte opCode;
  nvval = modconfig.readNV(button);
  // Allow consecutive button pushes for NV = 2 (on/off case)
  if (button != prevbutton /*|| nvval == 2 */) {
    Serial << F("Button ") << button << F(" changed") << endl;
    switch (nvval) {
      case 0:  // ON
        opCode = OPC_ACON;
        break;
      case 1:  // OFF
        opCode = OPC_ACOF;
        break;
      case 2:  // ON/OFF
        if (buttonState[button]) {
          opCode = OPC_ACOF;
        } else {
          opCode = OPC_ACON;
        }
        buttonState[button] = !buttonState[button];
        break;
      default:
        opCode = OPC_ACON;
    }
    sendEvent(opCode, button);
    prevbutton = button;
  }
}

// Send an event routine according to Module Switch
bool sendEvent(byte opCode, unsigned int eventNo)
{
  CANFrame msg;
  msg.id = modconfig.CANID;
  msg.len = 5;
  msg.data[0] = opCode;
  msg.data[1] = highByte(modconfig.nodeNum);
  msg.data[2] = lowByte(modconfig.nodeNum);
  msg.data[3] = highByte(eventNo);
  msg.data[4] = lowByte(eventNo);

  bool success = CBUS.sendMessage(&msg);
  if (success) {
    Serial << F("> sent CBUS message with Event Number ") << eventNo << endl;
  } else {
    Serial << F("> error sending CBUS message with ") << eventNo << endl;
  }
  return success;
}

//
/// command interpreter for serial console input
//

void processSerialInput(void)
{

  byte uev = 0;
  char msgstr[32], dstr[32];

  if (Serial.available()) {

    char c = Serial.read();

    switch (c) {

      case 'n':

        // node config
        printConfig();

        // node identity
        Serial << F("> CBUS node configuration") << endl;
        Serial << F("> mode = ") << (modconfig.FLiM ? "FLiM" : "SLiM") << F(", CANID = ") << modconfig.CANID << F(", node number = ") << modconfig.nodeNum << endl;
        Serial << endl;
        break;

      case 'e':

        // EEPROM learned event data table
        Serial << F("> stored events ") << endl;
        Serial << F("  max events = ") << modconfig.EE_MAX_EVENTS << F(" EVs per event = ") << modconfig.EE_NUM_EVS << F(" bytes per event = ") << modconfig.EE_BYTES_PER_EVENT << endl;

        for (byte j = 0; j < modconfig.EE_MAX_EVENTS; j++) {
          if (modconfig.getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        Serial << F("  stored events = ") << uev << F(", free = ") << (modconfig.EE_MAX_EVENTS - uev) << endl;
        Serial << F("  using ") << (uev * modconfig.EE_BYTES_PER_EVENT) << F(" of ") << (modconfig.EE_MAX_EVENTS * modconfig.EE_BYTES_PER_EVENT) << F(" bytes") << endl
               << endl;

        Serial << F("  Ev#  |  NNhi |  NNlo |  ENhi |  ENlo | ");

        for (byte j = 0; j < (modconfig.EE_NUM_EVS); j++) {
          sprintf(dstr, "EV%03d | ", j + 1);
          Serial << dstr;
        }

        Serial << F("Hash |") << endl;

        Serial << F(" --------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < modconfig.EE_MAX_EVENTS; j++) {

          if (modconfig.getEvTableEntry(j) != 0) {
            sprintf(dstr, "  %03d  | ", j);
            Serial << dstr;

            // for each data byte of this event
            for (byte e = 0; e < (modconfig.EE_NUM_EVS + 4); e++) {
              sprintf(dstr, " 0x%02hx | ", modconfig.readEEPROM(modconfig.EE_EVENTS_START + (j * modconfig.EE_BYTES_PER_EVENT) + e));
              Serial << dstr;
            }

            sprintf(dstr, "%4d |", modconfig.getEvTableEntry(j));
            Serial << dstr << endl;
          }
        }

        Serial << endl;

        break;

      // NVs
      case 'v':

        // note NVs number from 1, not 0
        Serial << "> Node variables" << endl;
        Serial << F("   NV   Val") << endl;
        Serial << F("  --------------------") << endl;

        for (byte j = 1; j <= modconfig.EE_NUM_NVS; j++) {
          byte v = modconfig.readNV(j);
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j, v, v);
          Serial << msgstr << endl;
        }

        Serial << endl
               << endl;

        break;

      // CAN bus status
      case 'c':

        CBUS.printStatus();
        break;

      case 'h':
        // event hash table
        modconfig.printEvHashTable(false);
        break;

      case 'y':
        // reset CAN bus and CBUS message processing
        CBUS.reset();
        break;

      case '*':
        // reboot
        modconfig.reboot();
        break;

      case 'm':
        // free memory
        Serial << F("> free SRAM = ") << modconfig.freeSRAM() << F(" bytes") << endl;
        break;

      case 'r':
        // renegotiate
        // Serial << F("> Start renegotiation ") << endl;
        CBUS.renegotiate();
        break;

      case 'z':
        // Reset module, clear EEPROM
        static bool ResetRq = false;
        static unsigned long ResWaitTime;
        if (!ResetRq) {
          // start timeout timer
          Serial << F(">Reset & EEPROM wipe requested. Press 'z' again within 2 secs to confirm") << endl;
          ResWaitTime = millis();
          ResetRq = true;
        } else {
          // This is a confirmed request
          // 2 sec timeout
          if (ResetRq && ((millis() - ResWaitTime) > 2000)) {
            Serial << F(">timeout expired, reset not performed") << endl;
            ResetRq = false;
          } else {
            //Request confirmed within timeout
            Serial << F(">RESETTING AND WIPING EEPROM") << endl;
            modconfig.resetModule();
            ResetRq = false;
          }
        }
        break;


      case '\r':
      case '\n':
        Serial << endl;
        break;

      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }
}
