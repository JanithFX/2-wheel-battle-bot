// M1 left
int enA = 5;
int in1 = 3;
int in2 = 2;
// M2 eight
int enB = 6;
int in3 = 8;
int in4 = 7;

// New motor driver pins
int enC = 10;
int in5 = 9;
int in6 = 11;

int receiver_pins[] = {A0, A1, A2, A3, A4, A5}; // Channel 5 on A4, Channel 6 on A5
int receiver_values[] = {0, 0, 0, 0, 0, 0};
int res_min = 1100;
int res_max = 1900;

int working_range = 255; // motor driver range

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Setup for the new motor driver
  pinMode(enC, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);

  Serial.begin(115200);

  // Make sure motors are off on startup
  mpower(1, 0);
  mpower(2, 0);
  mpower(3, 0); // Initialize the new motor driver to off
}

void loop() {
  receive();

  int m1 = 0;
  int m2 = 0;
  int m3 = 0; // New motor speed
  int rot = receiver_values[0]; // Steering input (Channel 1)

  // Only process motor control if valid signals are received
  if (isValidSignal()) {
    // Throttle (forward/backward movement) input (Channel 2)
    m1 = receiver_values[1] + rot / 2;
    m2 = receiver_values[1] - rot / 2;

    // Control new motor using channel 5 (A4) for speed
    if (receiver_values[4] > 0) { // Channel 5 for turning the motor on
      m3 = 255; // Full speed or set to desired value
    } else {
      m3 = 0; // Turn off the motor
    }

    // Change direction of new motor using channel 6 (A5)
    if (receiver_values[5] > 0) { // Positive value for one direction
      mpower(3, m3); // Forward
    } else if (receiver_values[5] < 0) { // Negative value for the opposite direction
      mpower(3, -m3); // Reverse
    } else {
      mpower(3, 0); // No movement if neutral
    }
  } else {
    // If the signal is not valid, stop the motors
    m1 = 0;
    m2 = 0;
    m3 = 0; // Ensure the new motor is off
  }

  mpower(1, m1);
  mpower(2, m2);
  // Control the new motor direction already handled above
}

int rp = 0;
void receive() {
  receiver_values[rp] = map(pulseIn(receiver_pins[rp], HIGH), res_min, res_max, -1 * working_range, working_range);
  rp++;
  if (rp == 6) {
    rp = 0;
  }

  for (int i = 0; i < 6; i++) {
    Serial.print("CH");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(receiver_values[i]);
    Serial.print(",\t");
  }

  Serial.println("");
}

// Function to check if signal from receiver is valid
boolean isValidSignal() {
  boolean valid = true;
  for (int i = 0; i < 2; i++) {  // Only check channels 1 and 2 (steering and throttle)
    if (receiver_values[i] == -working_range || receiver_values[i] == working_range) {
      valid = false;  // If any channel is out of range, mark the signal as invalid
    }
  }
  return valid;
}

void mpower(int motor, int spd) {
  int rotation = 0;
  if (spd > 0) {
    rotation = 1;
  } else if (spd < 0) {
    rotation = -1;
    spd *= -1;
  }

  if (spd > 255) {
    spd = 255;
  }

  int pwm;
  int pA;
  int pB;

  if (motor == 1) {
    pwm = enA;
    pA = in1;
    pB = in2;
  } else if (motor == 2) {
    pwm = enB;
    pA = in3;
    pB = in4;
  } else if (motor == 3) { // New motor
    pwm = enC;
    pA = in5;
    pB = in6;
  } else {
    return;
  }

  if (rotation == 0) {
    digitalWrite(pA, LOW);
    digitalWrite(pB, LOW);
  } else if (rotation == 1) {
    digitalWrite(pA, HIGH);
    digitalWrite(pB, LOW);
  } else if (rotation == -1) {
    digitalWrite(pA, LOW);
    digitalWrite(pB, HIGH);
  }

  analogWrite(pwm, spd);
}
