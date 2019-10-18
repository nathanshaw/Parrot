///////////////////////// WASHBOARD / PECKER MODULE /////////////////////////////

// 0 is off, 1 is forward, -1 is backward
int8_t washboard_state = 0;
uint16_t washboard_forward_remaining = 0;
uint16_t washboard_backward_remaining = 0;

elapsedMillis washboard_update;
#define WASHBOARD_F_SPEED 200
#define WASHBOARD_R_SPEED -50

void shakeWashboard(uint16_t len) {
  washboard_forward_remaining = len * 0.8;
  washboard_backward_remaining = len * 0.2;
  Serial.print("Shake washboard: ");
  Serial.print(washboard_forward_remaining);
  Serial.print(" - ");
  Serial.println(washboard_backward_remaining);
  washboard_state = 1;
  washboard_update = 0;
  motor.enableDrivers();
  updateWashboard();
}

void updateWashboard() {
  if (washboard_state == 0) {
    dbPrintln("wb state == 0", 2);
    return;
  } else if (washboard_state == 1) {
    dbPrintln("wb state = 1", 2);
    if (washboard_update > washboard_forward_remaining) {
      washboard_state = -1;
      washboard_update = 0;
    } else {
      motor.setM1Speed(WASHBOARD_F_SPEED);  
      // Serial.println("forward");
    }
  } else if (washboard_state == -1) {
    dbPrintln("wb state = -1", 2);
    if (washboard_update > washboard_backward_remaining) {
      washboard_state = 0;
      washboard_update = 0;
      motor.setM1Speed(0);
      motor.disableDrivers();
    }
    else {
      // Serial.println("reverse");
      motor.setM1Speed(WASHBOARD_R_SPEED);
    }
  }
}

void testPeckerModule() {
  // test the H-Bridge and the solenoid peckers
  Serial.println("TESTING PECKER");
  motor.enableDrivers();
  motor.setM1Speed(200);
  // digitalWrite(s_pins[1], HIGH);
  delay(40);
  motor.setM1Speed(-50);
  // digitalWrite(s_pins[1], LOW);
  delay(20);
  motor.setM1Speed(0);
  delay(100);
  motor.disableDrivers();
  
  /*for (int i =0; i < 4; i++) {
  digitalWrite(s_pins[i], HIGH);
  delay(50);
  digitalWrite(s_pins[i], LOW);
  delay(250);
  }*/
}
