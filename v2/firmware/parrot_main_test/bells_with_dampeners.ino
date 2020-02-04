
// state machine, 0 is inactive
// 1 is dampener is up but solenoid has not struck
// 2 is dampener is up and solenoid has struck
#define BELL_INACTIVE 0
#define BELL_DAMPENER_UP 1
#define BELL_STRIKING 2
#define BELL_STRUCK 3

uint8_t bell_state = 0;
// is a bell strike queued for striking
bool bell_strike_queue = false;

// this is how long in MS to wait after the dampener is removed before striking with solenoid
#define DAMPENER_DELAY 10
// this is how long in MS to actuate the striking solenoid for the bell
#define BELL_STRIKE_LENGTH 30
#define BELL_RING_LENGTH 30

elapsedMillis last_bell_activity;
unsigned long bell_strike_delay = 4000; // how often to strike the bell

void strikeBell() {
  // use a switch case statement to determine what to do
  // Serial.print(bell_state);
  // Serial.print("\t");
  switch (bell_state) {
    case BELL_INACTIVE:
      // if it has been long enough since the last bell strike and the bell it not currently active
      if (last_bell_activity > bell_strike_delay) {
        Serial.println("removing bell dampener");
        // turn on the dampener
        digitalWrite(s_pins[BELL_DAMPENER], HIGH);
        // reset the last bell activity timer
        last_bell_activity = 0;
        // change bell state to reflect status of dampener
        bell_state = BELL_DAMPENER_UP;
        break;
      }
      break;
    case BELL_DAMPENER_UP:
      // if the dampener has been up for long enough then strike the solenoid
      if (last_bell_activity > DAMPENER_DELAY) {
        Serial.println("striking the bell");
        digitalWrite(s_pins[BELL_SOL], HIGH);
        // reset the last bell activity timer
        last_bell_activity = 0;
        // change bell state to reflect status of dampener
        bell_state = BELL_STRIKING;
        break;
      }
      break;
    case BELL_STRIKING:
      // if the bell has been struck for long enough deactivate the striking solenoid
      if (last_bell_activity > BELL_STRIKE_LENGTH) {
        Serial.println("stopping bell strike");
        digitalWrite(s_pins[BELL_SOL], LOW);
        // reset the last bell activity timer
        last_bell_activity = 0;
        // change bell state to reflect status of dampener
        bell_state = BELL_STRUCK;
        break;
      }
      break;
    case BELL_STRUCK:
      // if the bell has rings for long enough then allow the dampener to reengague
      if (last_bell_activity > BELL_RING_LENGTH) {
        Serial.println("reapplying the dampener");
        digitalWrite(s_pins[BELL_DAMPENER], LOW);
        // reset the last bell activity timer
        last_bell_activity = 0;
        // change bell state to inactive
        bell_state = BELL_INACTIVE;
        break;
      }
      break;
  }
}
