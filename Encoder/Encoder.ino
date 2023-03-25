const int encoder_pin1 = 2;
const int encoder_pin2 = 3;
volatile int count = 0;
float position = 0.0;

void setup() {
  pinMode(encoder_pin1, INPUT);
  pinMode(encoder_pin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_pin1), encoder_callback, CHANGE);
  Serial.begin(9600);
}

void loop() {
  Serial.print("Motor position: ");
  Serial.print(position);
  Serial.println(" degrees");
  delay(100);
}

void encoder_callback() {
  if (digitalRead(encoder_pin2)) {
    count++;
  } else {
    count--;
  }
  position = count / 25.9 * 360;
}
