
//yellow
const int encoder_pin1 = 2;
//green
const int encoder_pin2 = 3;
volatile int count = 0;
float position = 0.0;
bool lastA;

void setup() {
  pinMode(encoder_pin1, INPUT_PULLUP);
  pinMode(encoder_pin2, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_pin1), encoder_callback, CHANGE);
  lastA = digitalRead(encoder_pin1);
  Serial.begin(9600);
}

void loop() {
  Serial.print("Motor position: ");
  Serial.print(position);
  Serial.println(" degrees");
  digitalWrite(9, HIGH);
  delay(100);
}

void encoder_callback() {
  bool newA = digitalRead(encoder_pin1);
  bool newB = digitalRead(encoder_pin2);

  if (newA != lastA) {
    if (newB != lastA) {
      count++;
    } else {
      count--;
    }
    lastA = newA;
  }
  position = count * 13.89/2;
}
