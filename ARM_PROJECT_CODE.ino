#define ENCODER_A_PIN 1      // Rotary encoder pin A
#define ENCODER_B_PIN 2      // Rotary encoder pin B
#define SERVO_PIN 7          // Servo motor pin

volatile int encoderPos = 0;
volatile int lastEncoded = 0;
int servoAngle = 0;

void updateEncoder() {
  int MSB = digitalRead(ENCODER_A_PIN);  // Most Significant Bit
  int LSB = digitalRead(ENCODER_B_PIN);  // Least Significant Bit

  int encoded = (MSB << 1) | LSB;  // Convert the 2 binary inputs to a single decimal number
  int sum = (lastEncoded << 2) | encoded;  // Calculate the sum of the last and current encoded value

  // Check the lookup table to determine the direction of rotation
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos--;
  }

  lastEncoded = encoded;
}

void setup() {
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), updateEncoder, CHANGE);

  pinMode(SERVO_PIN, OUTPUT);
}

void loop() {
  // Map the encoder position to the servo angle range (0-180)
  int angle = map(encoderPos, 0, 100, 0, 180);
  
  // Set the servo angle using PWM signal
  analogWrite(SERVO_PIN, map(angle, 0, 180, 0, 255));
  Serial.println(encoderPos);

  delay(10);  // Adjust the delay as per your requirements
}
