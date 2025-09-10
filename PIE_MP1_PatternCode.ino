// Bike Light Arduino Program adapted from Brad Minch's example code, FSM04, found on the PIE GitHub.




// mechanical constants/mappings
const uint8_t GREEN_LED = 9; // note that all of these ports must be PWM compatible
const uint8_t BLUE_LED = 10;
const uint8_t RED_LED = 11;
const uint8_t SW1 = 8;       // switch is a button
const uint8_t LIGHT_SENSOR_PIN = A0; // the analog pin the phototransistor is connected to




// pattern constants
const uint16_t BLINK_INTERVAL      = 500; // time between on/off state on flash pattern
const uint16_t DEBOUNCE_TIME       = 200; // time between
const uint16_t BREATHING_INTERVAL = 20;  // how often to update brightness (ms)
const uint16_t BREATHING_STEPS     = 100; // number of steps from 0 to max brightness
const uint8_t LED_PINS[] = {RED_LED, BLUE_LED, GREEN_LED}; // ordered from top to bottom
const uint16_t BOUNCE_INTERVAL     = 200; // speed of bounce (switching time)




// state machine foundation
// this is done so that we create a pointer for our state, meaning we can call the variable
// as opposed to having to switch between the cases for each state
typedef void (*STATE_HANDLER_T)();




// forward declarations for all functions to avoid compile-time errors
void pattern_all_off();
void pattern_all_on();
void pattern_all_flash();
void pattern_breathing();
void pattern_bounce();
void pattern_transistor_dimmer(); // NEW: Forward declaration for the new state




// forward declarations for used variables to avoid compile-time errors
STATE_HANDLER_T prior_state, state;
uint32_t led_time;
bool last_button_state = LOW;
bool button_pressed = false;
uint8_t  current_led       = 0;   // current LED in bouncing pattern
int8_t   bounce_direction    = 1;   // 1 = going down, -1 = going up
uint8_t  brightness        = 0;   // measured from 0-255 as 0-100% respectively
int8_t   brightness_direction = 1;   // 1 for increasing, -1 for decreasing
uint16_t breathing_step      = 0;   // current step in breathing cycle












/*!
@brief This is a helper function to allow for easier switching between states. It treats a button
press as a toggle with a debounce time, preventing microsecond state switches, and ensures a
smoother and stable switching of states. You can only switch from your current state to the
provided new state. The unique switch allows for varying buttons to activate varying states.
@param switch_pin is the Digital Pin Number on the Arduino that is mapped to your switch cycle.
@param next_state is the next state the button press should switch to.
*/
void toggleButtonPressSwitch(uint8_t switch_pin, STATE_HANDLER_T next_state) {
  static uint32_t last_valid_press = 0;      // the last recorded time of a valid press
  bool current_button = digitalRead(switch_pin);   // get pin number for switch from argument
  uint32_t current_time = millis();              // elapsed time of run time in ms




  if (current_button == HIGH && (current_time - last_valid_press) > DEBOUNCE_TIME) {
    last_valid_press = current_time; // update time
    state = next_state;              // update state
  }
}




/*!
This is a function that exists as a state for the bike light LED pattern cycle. In this state, the LEDs will all be off. The next state is pattern_all_on.
*/
void pattern_all_off() {
  if (state != prior_state) { // inital state conditions
    prior_state = state;
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  }
 
  toggleButtonPressSwitch(SW1, pattern_all_on); // button press state switch
 
  if (state != prior_state) { // conditions when leaving state (clean up)
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  }
}




/*!
This is a function that exists as a state for the bike light LED pattern cycle. In this state, the LEDs will all be off. The next state is pattern_all_flash.
*/
void pattern_all_on() {
  if (state != prior_state) { // inital state conditions
    prior_state = state;
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(RED_LED, HIGH);
  }
 
  toggleButtonPressSwitch(SW1, pattern_all_flash); // button press state switch
 
  if (state != prior_state) { // conditions when leaving state (clean up)
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  }
}




/*!
This is a function that exists as a state for the bike light LED pattern cycle. In this state, all LEDs will flash together. The next state is pattern_breathing.
*/
void pattern_all_flash() {
  if (state != prior_state) { // initial state conditions
    prior_state = state;
    led_time = millis();
  }
 
  // state tasks - flash all LEDs together (alternate between on/off at the same time)
  uint32_t t = millis();
  if (t >= BLINK_INTERVAL + led_time){
    digitalWrite(RED_LED, !digitalRead(RED_LED));
    digitalWrite(BLUE_LED, !digitalRead(BLUE_LED));
    digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
    led_time = t;
  }
 
  toggleButtonPressSwitch(SW1, pattern_breathing); // button press state switch
 
  if (state != prior_state) { // conditions when leaving state (clean up)
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  }
}




/*!
This is a function that exists as a state for the bike light LED pattern cycle. In this state, the LEDs will smoothly fade in and out like breathing. The next state is pattern_bounce.
@note this is only possible through the PWM ports, so make sure your LEDs fit here. PWM essentially takes a digital signal (0/1) and translates it to analog (variable) by measuring the
time between intervals and the duration of the state, meaning in a 1-second interval, being on 1 for .5 seconds would translate to 50%, and thus be 50% brightness. For Arduino's purposes,
0% maps to 0 and 100% maps to 255.
*/
void pattern_breathing() {
  if (state != prior_state) { // initial state conditions
    prior_state = state;
    brightness = 0;
    brightness_direction = 1;
    breathing_step = 0;
    led_time = millis();
  }
 
  // state tasks - update breathing effect
  uint32_t t = millis();
  if (t >= led_time + BREATHING_INTERVAL) {   // check if enough time has passed for next brightness update
    breathing_step += brightness_direction;  // move to next step (down +1 or up -1)
   
    // map() converts breathing_step range (0 to BREATHING_STEPS) into PWM range (0 to 255)
    // example: if breathing_step=50 and BREATHING_STEPS=100, map returns 127 (half brightness)
    brightness = map(breathing_step, 0, BREATHING_STEPS, 0, 255);
   
    // apply calculated brightness to all LEDs using PWM (0 = off, 255 = max brightness)
    analogWrite(RED_LED, brightness);
    analogWrite(GREEN_LED, brightness);
    analogWrite(BLUE_LED, brightness);
   
    // reverse direction when we hit the limits to create a bouncing effect
    if (breathing_step >= BREATHING_STEPS) { // reached maximum brightness
      brightness_direction = -1;             // start fading down
    } else if (breathing_step <= 0) {       // reached minimum brightness
      brightness_direction = 1;              // start fading up
    }
   
    led_time = t; // update time for next interval
  }
 
  toggleButtonPressSwitch(SW1, pattern_bounce); // button press state switch
 
  if (state != prior_state) { // conditions when leaving state (clean up)
    analogWrite(RED_LED, 0);
    analogWrite(GREEN_LED, 0);
    analogWrite(BLUE_LED, 0);
  }
}




void pattern_bounce() {
  // sizeof(LED_PINS) gives total bytes, sizeof(LED_PINS[0]) gives bytes per element
  // dividing gives the number of elements in the array (e.g., 3 LEDs)
  uint8_t NUM_LEDS = sizeof(LED_PINS) / sizeof(LED_PINS[0]);
 
  if (state != prior_state) {   // initial state conditions
    prior_state = state;
    current_led = 0;           // start with first LED (index 0 = RED_LED)
    bounce_direction = 1;      // +1 means moving forward through array (RED→BLUE→GREEN)
    led_time = millis();
   
    // ensure all LEDs start in the off state
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      digitalWrite(LED_PINS[i], LOW);
    }
    digitalWrite(LED_PINS[current_led], HIGH); // illuminate only the starting LED
  }
 
  // state tasks - bounce between LEDs
  uint32_t t = millis();
  if (t >= led_time + BOUNCE_INTERVAL) {         // check if enough time passed for next bounce step
    digitalWrite(LED_PINS[current_led], LOW);    // turn off currently lit LED before moving
   
    current_led += bounce_direction;             // move index: +1 goes forward, -1 goes backward
   
    // boundary checking - prevent array out of bounds and create bounce effect
    if (current_led >= NUM_LEDS - 1) {       // hit the end of array (GREEN LED)
      current_led = NUM_LEDS - 1;            // clamp to last valid index
      bounce_direction = -1;               // reverse to go backward (GREEN→BLUE→RED)
    } else if (current_led <= 0) {           // hit the beginning of array (RED LED)
      current_led = 0;                       // clamp to first index
      bounce_direction = 1;                  // reverse to go forward (RED→BLUE→GREEN)
    }
   
    digitalWrite(LED_PINS[current_led], HIGH); // illuminate the new LED position
    led_time = t;                            // update timing for next bounce interval
  }
 
  toggleButtonPressSwitch(SW1, pattern_transistor_dimmer); // button press state switch (NEW)
 
  if (state != prior_state) { // conditions when leaving state (clean up)
    // ensure all LEDs are off when exiting this pattern
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      digitalWrite(LED_PINS[i], LOW);
    }
  }
}




/*!
@brief This is a function that exists as a state on the bike light that uses the phototransistor to control the LED brightness inversely.
When there is no light detected, the LEDs will be at maximum brightness. When there is light, they will dim.
*/
void pattern_transistor_dimmer() {
  if (state != prior_state) { // initial state conditions
    prior_state = state;
    analogWrite(RED_LED, 0);
    analogWrite(GREEN_LED, 0);
    analogWrite(BLUE_LED, 0);
  }
 
  // state tasks - read the sensor output (from voltage divider circuit) and set LED brightness
  int lightSensorValue = analogRead(LIGHT_SENSOR_PIN);
 
  // map() converts the light sensor range (0-1023) to an inverted PWM range (255-0); this makes the LEDs brighter with less light and dimmer with more light. We chose a value of light sensor range ending at 700 to make the effect more obvious using the constrain macro.
  lightSensorValue = constrain(lightSensorValue, 0, 700);
  int ledBrightness = map(lightSensorValue, 0, 700, 255, 0);
 
  // apply the brightness to all LEDs
  analogWrite(RED_LED, ledBrightness);
  analogWrite(GREEN_LED, ledBrightness);
  analogWrite(BLUE_LED, ledBrightness);
 
  toggleButtonPressSwitch(SW1, pattern_all_off); // button press state switch
 
  if (state != prior_state) { // conditions when leaving state (clean up)
    analogWrite(RED_LED, 0);
    analogWrite(GREEN_LED, 0);
    analogWrite(BLUE_LED, 0);
  }
}



/*
@brief This function is always called initially when the sketch is loaded. This will be run initially and define our pins as either inputs or outputs while stating  the initial state they should be at.
*/
void setup() {
  pinMode(GREEN_LED, OUTPUT); // configure green LED pin as output (send voltage vs. read voltage)
  pinMode(BLUE_LED, OUTPUT);  // configure blue LED pin as output
  pinMode(RED_LED, OUTPUT);   // configure red LED pin as output
  pinMode(SW1, INPUT);        // configure switch pin as input
 
  // set initial LED states
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);
 
  // initialize state machine
  prior_state = NULL;
  state = pattern_all_off; // start with first pattern
}




void loop() {
  state(); // execute current pattern function
}



