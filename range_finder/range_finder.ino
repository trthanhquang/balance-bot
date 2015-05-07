#include <NewPing.h>
#include <Servo.h> 

#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define SERVO_PIN 9
#define MAX_DISTANCE 500

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo myservo;
int servo_pos = 0;

unsigned long start_time;

unsigned long prev_sensor_ms = 0;
unsigned long sensor_period_ms = 30;

unsigned long prev_remote_ms = 0;
unsigned long remote_period_ms = 200;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);

  myservo.attach(SERVO_PIN);
  myservo.write(0);
  prev_sensor_ms  = millis();
  prev_remote_ms  = millis();
}

void loop() {
  if((millis() - prev_sensor_ms) >= sensor_period_ms) {
    start_time = millis();
    
    unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
    Serial.print(" d_cm=");
    Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
    Serial.println("");
    
    prev_sensor_ms = start_time;
  }

  if((millis() - prev_remote_ms) >= remote_period_ms) {
    prev_remote_ms = millis();
    read_servo_config();
  }
}

void read_servo_config(){
  if (Serial.available() > 0) {
    servo_pos = Serial.parseFloat();
    Serial.print(" servo=");
    Serial.println(servo_pos, DEC);
    myservo.write(servo_pos);
  }
}

