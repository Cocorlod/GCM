#define STBY 16
#define AIN1 4 
#define AIN2 8
#define BIN1 18
#define BIN2 17 
#define PWMA 9
#define PWMB 10

const int freq = 1000;
const int resolution = 8;

void setup() {
  // put your setup code here, to run once:
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  ledcAttach(PWMA, freq, resolution);
  ledcAttach(PWMB, freq, resolution);

  digitalWrite(STBY, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, 200);
  delay(2000);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWMA, 200);
  delay(2000);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWMA, 0);
  delay(2000);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMB, 200);
  delay(2000);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(PWMB, 200);
  delay(2000);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  ledcWrite(PWMB, 0);
  delay(2000);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  for (int speed = 0; speed <= 255; speed++) {
    ledcWrite(PWMA, speed);
    ledcWrite(PWMB, speed);
    delay(20);
  }

  for (int speed = 255; speed >= 0; speed--) {
    ledcWrite(PWMA, speed);
    ledcWrite(PWMB, speed);
    delay(20);
  }

  digitalWrite(STBY, LOW);   
  delay(2000);

  digitalWrite(STBY, HIGH);  
  delay(2000);
}
