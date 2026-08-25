#include <USB.h>
#include <USBHIDKeyboard.h> 
USBHIDKeyboard Keyboard;

#define CONTAINER "docker?connect"
#define MIN_TIME 1500 /* minimal time required between presses*/
int timer = 0;

/** Button 1 configuration*/
#define BUTTON1_PIN D5      /* pin for the button                 */
#define LED_PIN1 D3         /* pin for the led bulb               */
#define LED1_BLINK_TIME 500 /* time in miliseconds for the blink  */
#define COMMAND1 "ros2 launch what?could?go?wrong macro?launch.xml"
bool ledState1 = false;
bool button1Toggled = false;
int timeSinceLastPress1 = 0;


/** Button 2 configuration*/
#define BUTTON2_PIN D6      /* pin for the button                 */
#define LED_PIN2 D2         /* pin for the led bulb               */
#define LED2_BLINK_TIME 100 /* time in miliseconds for the blink  */
#define COMMAND2 "ros2 bag record /a"
bool ledState2 = false;
bool button2Toggled = false;
int timeSinceLastPress2 = 0;

void updateLED(int led_pin, int* time, int blink_time, bool toggle_state, bool* ledState) {
  if (!toggle_state) {
    digitalWrite(led_pin, LOW); /* turn off the bulb*/
    return;
  }
  if (*time % blink_time == 0) {
    *ledState = !*ledState;
    digitalWrite(led_pin, *ledState ? HIGH : LOW);
  }
}

void inputCommand(const char* text) {
  while (*text) {
    Keyboard.print(*text);
    text++;
    delay(8);
  }
  Keyboard.write(KEY_RETURN);
}

void pressShortcut(uint8_t mod, uint8_t key) {
  Keyboard.press(mod);
  Keyboard.press(key);
  delay(80);
  Keyboard.releaseAll();
  delay(150);
}

void pressShortcut2(uint8_t mod1, uint8_t mod2, uint8_t key) {
  Keyboard.press(mod1);
  Keyboard.press(mod2);
  Keyboard.press(key);
  delay(80);
  Keyboard.releaseAll();
  delay(150);
}

void openTerminal() {
  pressShortcut2(KEY_LEFT_CTRL, KEY_LEFT_ALT, 't');
  delay(1000);
  inputCommand(CONTAINER);
  delay(500);
}

void openNewTerminalTab() {
  pressShortcut2(KEY_LEFT_CTRL, KEY_LEFT_SHIFT, 't');
  delay(1000);
  inputCommand(CONTAINER);
  delay(500);
}

void changeTerminalTab() {
  pressShortcut(KEY_LEFT_CTRL, KEY_PAGE_UP);
}

void killRunningCommand() {
  pressShortcut(KEY_LEFT_CTRL, 'c');
}


void setup() {
  // Set buttons as inputs
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  // Set bulbs as outputs and turn off the bulbs
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);

  Serial.begin(115200);
  delay(10000); /* give jetson for jetson to setup*/

  // Initialize ESP as a keyboard
  USB.begin();
  Keyboard.begin();
  delay(700);

  // Open two terminals, one for each command
  openTerminal();
  delay(700);
  openNewTerminalTab();
  delay(700);
}

void updateButton() {
}

void loop() {
  bool button1State = digitalRead(BUTTON1_PIN); /* read current input from button 1*/
  bool button2State = digitalRead(BUTTON2_PIN); /* read current input from button 2*/

  if (button1State && timeSinceLastPress1 >= MIN_TIME) {
    timeSinceLastPress1 = 0;
    button1Toggled = !button1Toggled;
    Serial.println("Button 1 toggled");
    if (button1Toggled) {
      inputCommand(COMMAND1);
    } else {
      killRunningCommand();
    }
  }

  if (button2State && timeSinceLastPress2 >= MIN_TIME) {
    timeSinceLastPress2 = 0;
    button2Toggled = !button2Toggled;
    Serial.println("Button 2 toggled");
    if (button2Toggled) {
      changeTerminalTab();
      inputCommand(COMMAND2);
      changeTerminalTab();
    } else {
      changeTerminalTab();
      killRunningCommand();
      changeTerminalTab();
    }
  }

  updateLED(LED_PIN1, &timer, LED1_BLINK_TIME, button1Toggled, &ledState1);
  updateLED(LED_PIN2, &timer, LED2_BLINK_TIME, button2Toggled, &ledState2);

  // Update time
  timer += 10;
  timeSinceLastPress1 += 10;
  timeSinceLastPress2 += 10;
  delay(10);
}
