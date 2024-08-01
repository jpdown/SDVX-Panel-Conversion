/**
 * Originally written by Skogaby
 * Modified by SpeedyPotato
 */

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#include <Bounce2.h>
//#include <FastLED.h>
#include <Joystick.h>
#include <ResponsiveAnalogRead.h>
#include "PluggableUSB.h"
#include "HID.h"

// Pin setup
#define ADC_MAX 1024
#define NUM_BUTTONS 9
#define NUM_KNOBS 2
#define NUM_LED_STRIPS 2
#define NUM_LEDS_PER_STRIP 31
#define LED_TYPE WS2812B
#define LED_ORDER GRB

// HID lighting setup
#define NUMBER_OF_SINGLE 7
#define NUMBER_OF_RGB 1

bool lightStates[NUMBER_OF_SINGLE] = { false };
unsigned long lightTimestamp = 0;

typedef struct {
  uint8_t brightness;
} SingleLED;

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} RGBLed;

// Main output
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD, NUM_BUTTONS, 0,
 true, true, false, false, false, false, false, false, false, false, false);

// Button pins
const byte buttonLightPins[NUM_BUTTONS] = { 2, 3, 4, 5, 19, 18, 1, 1, 15 };
const byte buttonInputPins[NUM_BUTTONS] = { 6, 7, 8, 9, 14, 16, 0, 0, 10 };
const byte knobPin1 = A3;
const byte knobPin2 = A2;
//const byte ledPin1 = A2;
//const byte ledPin2 = A3;
//CRGB leds[NUM_LED_STRIPS][NUM_LEDS_PER_STRIP];

// Button debounce objects
const Bounce buttons[NUM_BUTTONS] = {
  Bounce(),
  Bounce(),
  Bounce(),
  Bounce(),
  Bounce(),
  Bounce(),
  Bounce(),
  Bounce(),
  Bounce()
};

// Objects to read the analog values of the knobs responsively, but with smoothing
ResponsiveAnalogRead leftKnob(knobPin1, true);
ResponsiveAnalogRead rightKnob(knobPin2, true);

// Diva mode
#define ACTIVATION_TIME 50
#define ACTIVATION_THRESHOLD 10
#define CANCEL_THRESHOLD 3
#define SUSTAIN_TIME 175

typedef struct {
  unsigned long moveTime;
  int lastVal;
  int activationVal;
  bool activating;
  bool activated;
  int direction;
} DivaKnob;

bool divaMode = false;
DivaKnob divaLeft = {0};
DivaKnob divaRight = {0};

//void setAllLeds(CRGB color);

/**
 * Setup all the hardware pins, etc.
 */
void setup() {
  //Serial.begin(115200);
  //while (!Serial) { delay(10); }
  
  // Setup the ADC registers to run more quickly
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  // Joystick init
  Joystick.begin(false);
  Joystick.setXAxisRange(0, ADC_MAX);
  Joystick.setYAxisRange(0, ADC_MAX);

  // Initialize the IO pins
  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (buttonInputPins[i] == 20) continue;
    pinMode(buttonLightPins[i], OUTPUT);
    buttons[i].attach(buttonInputPins[i], INPUT_PULLUP);
    buttons[i].interval(5);
  }

  pinMode(knobPin1, INPUT);
  pinMode(knobPin2, INPUT);

  // Check if should enter Diva mode (holding FX-L, BT-A, Start upon plugging in)
  if (!digitalRead(buttonInputPins[0]) && !digitalRead(buttonInputPins[4]) && !digitalRead(buttonInputPins[8])) {
    divaMode = true;
    for (int i = 0; i < 3; i++) {
      digitalWrite(buttonLightPins[0], 1);
      digitalWrite(buttonLightPins[4], 1);
      digitalWrite(buttonLightPins[8], 1);
      delay(250);
      digitalWrite(buttonLightPins[0], 0);
      digitalWrite(buttonLightPins[4], 0);
      digitalWrite(buttonLightPins[8], 0);
      delay(250);
    }
  }

  // Initialize the LED strips
  //FastLED.addLeds<LED_TYPE, ledPin1, LED_ORDER>(leds[0], NUM_LEDS_PER_STRIP);
  //FastLED.addLeds<LED_TYPE, ledPin2, LED_ORDER>(leds[1], NUM_LEDS_PER_STRIP);

  // Boot LED sequence
  //FastLED.setBrightness(128);
  //setAllLeds(CRGB::Blue);
  //FastLED.show();
}

/**
 * Callback for HID lighting events.
 */
void light_update(SingleLED* single_leds, RGBLed* rgb_leds) {
  // Read the on/off non-RGB lights
  for(int i = 0; i < NUMBER_OF_SINGLE; i++) {
    bool isOn = single_leds[i].brightness > 128;
    lightStates[i] = isOn;
  }

  // Read the RGB lights
  //CRGB color = CRGB(rgb_leds[0].r, rgb_leds[0].g, rgb_leds[0].b);
  //setAllLeds(color);
  lightTimestamp = millis();
}

int knob_direction(int oldVal, int newVal) {
  if (abs(oldVal - newVal) > 768) {
    // Assume it wrapped around, so return the reverse
    return newVal < oldVal ? 1 : -1;
  }
  return newVal > oldVal ? 1 : -1;
}

int knob_difference(int oldVal, int newVal) {
  // If the difference is very large, assume it wrapped around
  int subtract = abs(oldVal - newVal) > 768 ? 1024 : 0;

  return newVal - oldVal - subtract;
}

int diva_idle(DivaKnob *knob, int knob_val) {
    if (knob_val != knob->lastVal) {
      knob->moveTime = millis();
      knob->direction = knob_direction(knob->lastVal, knob_val);
      knob->activationVal = knob_val;
      knob->activating = true;
    }

    return 512;
}

int diva_activating(DivaKnob *knob, int knob_val) {
    if (millis() - knob->moveTime > ACTIVATION_TIME) {
      knob->activating = false;
      knob->lastVal = knob_val;
    } else if(knob_direction(knob->lastVal, knob_val) != knob->direction && abs(knob_difference(knob->lastVal, knob_val))) {
      // Cancel this activation
      knob->activating = false;
    } else if (abs(knob_difference(knob->activationVal, knob_val)) > CANCEL_THRESHOLD) {
      knob->activating = false;
      knob->lastVal = knob_val;
      knob->moveTime = millis();
      knob->activated = true;
    }

    return 512;
}

int diva_activated(DivaKnob *knob, int knob_val) {
    if (knob_val != knob->lastVal) {
      if (knob_direction(knob->lastVal, knob_val) != knob->direction && abs(knob_difference(knob->lastVal, knob_val)) > CANCEL_THRESHOLD) {
        // Cancel this activation
        knob->activated = false;
      } else {
        knob->moveTime = millis();
      }
    } else if (millis() - knob->moveTime > SUSTAIN_TIME) {
      knob->activated = false;
    }

    return knob->direction == 1 ? 1024 : 0;
}

int diva_update(DivaKnob *knob, int knob_val) {
  int axis = 0;

  // You can progress between all states in one iteration, so check every state in sequence
  if (!knob->activating && !knob->activated) {
    axis = diva_idle(knob, knob_val);
  } 
  if (knob->activating) {
    axis = diva_activating(knob, knob_val);
  } 
  if (knob->activated) {
    axis = diva_activated(knob, knob_val);
  }

  knob->lastVal = knob_val;
  return axis;
}

/**
 * Main runtime loop.
 */
void loop() {
  // Read the button states
  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (buttonInputPins[i] == 20) continue;
    buttons[i].update();
    Joystick.setButton(i, !buttons[i].read());
  }

  // Read the knobs
  leftKnob.update();
  rightKnob.update();

  // Output the joystick states
  if (divaMode) {
    Joystick.setXAxis(diva_update(&divaLeft, leftKnob.getValue()));
    Joystick.setYAxis(diva_update(&divaRight, rightKnob.getValue()));
  } else {
    Joystick.setXAxis(leftKnob.getValue());
    Joystick.setYAxis(rightKnob.getValue());
  }
  
  Joystick.sendState();
  
  if (lightTimestamp + 1000 < millis()) {
    // Update the lights as reactive
    for (int i = 0; i < NUMBER_OF_SINGLE - 1; i++) {
      digitalWrite(buttonLightPins[i], !buttons[i].read());
    }
    digitalWrite(buttonLightPins[NUM_BUTTONS - 1], !buttons[NUM_BUTTONS - 1].read());
  } else {
    // Update the lights as necessary
    for (int i = 0; i < NUMBER_OF_SINGLE - 1; i++) {
      digitalWrite(buttonLightPins[i], lightStates[i]);
    }
    digitalWrite(buttonLightPins[NUM_BUTTONS - 1], lightStates[NUMBER_OF_SINGLE - 1]);
  }
  //FastLED.show();
}

/**
 * Helper function to set all the LEDs across both strips the same color.
 */
//void setAllLeds(CRGB color) {
//  for (int strip = 0; strip < NUM_LED_STRIPS; strip++) {
//    for (int led = 0; led < NUM_LEDS_PER_STRIP; led++) {
//      leds[strip][led] = color;
//    }
//  }
//}

// ******************************
// don't need to edit below here

#define NUMBER_OF_LIGHTS (NUMBER_OF_SINGLE + NUMBER_OF_RGB*3)
#if NUMBER_OF_LIGHTS > 63
  #error You must have less than 64 lights
#endif

union {
  struct {
    SingleLED singles[NUMBER_OF_SINGLE];
    RGBLed rgb[NUMBER_OF_RGB];
  } leds;
  uint8_t raw[NUMBER_OF_LIGHTS];
} led_data;

static const uint8_t PROGMEM _hidReportLEDs[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x00,                    // USAGE (Undefined)
    0xa1, 0x01,                    // COLLECTION (Application)
    // Globals
    0x95, NUMBER_OF_LIGHTS,        //   REPORT_COUNT
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x05, 0x0a,                    //   USAGE_PAGE (Ordinals)
    // Locals
    0x19, 0x01,                    //   USAGE_MINIMUM (Instance 1)
    0x29, NUMBER_OF_LIGHTS,        //   USAGE_MAXIMUM (Instance n)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    // BTools needs at least 1 input to work properly
    0x19, 0x01,                    //   USAGE_MINIMUM (Instance 1)
    0x29, 0x01,                    //   USAGE_MAXIMUM (Instance 1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0xc0                           // END_COLLECTION
};

// This is almost entirely copied from NicoHood's wonderful RawHID example
// Trimmed to the bare minimum
// https://github.com/NicoHood/HID/blob/master/src/SingleReport/RawHID.cpp
class HIDLED_ : public PluggableUSBModule {

  uint8_t epType[1];
  
  public:
    HIDLED_(void) : PluggableUSBModule(1, 1, epType) {
      epType[0] = EP_TYPE_INTERRUPT_IN;
      PluggableUSB().plug(this);
    }

    int getInterface(uint8_t* interfaceCount) {
      *interfaceCount += 1; // uses 1
      HIDDescriptor hidInterface = {
        D_INTERFACE(pluggedInterface, 1, USB_DEVICE_CLASS_HUMAN_INTERFACE, HID_SUBCLASS_NONE, HID_PROTOCOL_NONE),
        D_HIDREPORT(sizeof(_hidReportLEDs)),
        D_ENDPOINT(USB_ENDPOINT_IN(pluggedEndpoint), USB_ENDPOINT_TYPE_INTERRUPT, USB_EP_SIZE, 16)
      };
      return USB_SendControl(0, &hidInterface, sizeof(hidInterface));
    }
    
    int getDescriptor(USBSetup& setup)
    {
      // Check if this is a HID Class Descriptor request
      if (setup.bmRequestType != REQUEST_DEVICETOHOST_STANDARD_INTERFACE) { return 0; }
      if (setup.wValueH != HID_REPORT_DESCRIPTOR_TYPE) { return 0; }
    
      // In a HID Class Descriptor wIndex contains the interface number
      if (setup.wIndex != pluggedInterface) { return 0; }
    
      return USB_SendControl(TRANSFER_PGM, _hidReportLEDs, sizeof(_hidReportLEDs));
    }
    
    bool setup(USBSetup& setup)
    {
      if (pluggedInterface != setup.wIndex) {
        return false;
      }
    
      uint8_t request = setup.bRequest;
      uint8_t requestType = setup.bmRequestType;
    
      if (requestType == REQUEST_DEVICETOHOST_CLASS_INTERFACE)
      {
        return true;
      }
    
      if (requestType == REQUEST_HOSTTODEVICE_CLASS_INTERFACE) {
        if (request == HID_SET_REPORT) {
          if(setup.wValueH == HID_REPORT_TYPE_OUTPUT && setup.wLength == NUMBER_OF_LIGHTS){
            USB_RecvControl(led_data.raw, NUMBER_OF_LIGHTS);
            light_update(led_data.leds.singles, led_data.leds.rgb);
            return true;
          }
        }
      }
    
      return false;
    }
};

HIDLED_ HIDLeds;
