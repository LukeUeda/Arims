float scissor_angle;
float lift_height;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  scissor_angle = 120.0f - analogRead(A0)*89.5f/1023.0f;
  lift_height = 300*sin(scissor_angle * 3.1415/360)+ 32.5;

  Serial.println(lift_height);
  delay(50);
}


// MAX HEIGHT = 268.74
// MIN HEIGHT = 114.94