#include <Wire.h>

#define SLAVE 0x01

bool motorON = false;
int pwm_a = 3;   //PWM control for motor outputs 1 and 2 is on digital pin 3
int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12
int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13

byte SPEED1 = 255;
byte SPEED2 = 255;
bool DIR1 = false;
bool DIR2 = false; // false is forward true is reverse

void setup() {
  // put your setup code here, to run once:
  Wire.begin(SLAVE);
  
  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, HIGH);

  Wire.onReceive(recEvent);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void recEvent(int DATA)
{
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  int i = 0;
  while (1 <= Wire.available())
  {
    i += Wire.read();
  }
  //if (i>255)
  //  return;
  Serial.println((uint8_t) i);
  updateMotors((uint8_t) i);
  delay(100);
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
}

void updateMotors(uint8_t VALUE)
{
  /*
   * This function will not function if a byte is not passed.
   * String must be supported.
   */
  String sp1 = "", sp2 = "";

  String s = (String)((VALUE >> 0) & 0b1);
  if (s == "1")
    digitalWrite(dir_a, HIGH);
  else
    digitalWrite(dir_a, LOW);

  for (int b = 1; b <= 3; b++)
  {
     sp1 += (String)((VALUE >> b) & 0b1);
  }

  s = (String)((VALUE >> 4) & 0b1);
  if (s == "1")
    digitalWrite(dir_b, HIGH);
  else
    digitalWrite(dir_b, LOW);
   
  for (int b = 5; b <= 7; b++)
  {
     sp2 += (String)((VALUE >> b) & 0b1);
  }

  /*

  Serial.println(sp1);
  Serial.println(sp2);
  Serial.println(s+" "+DIR1+" "+DIR2);

  */

  SPEED1 = setSpeeds(sp1);
  SPEED2 = setSpeeds(sp2);

  analogWrite(pwm_a, SPEED1);
  analogWrite(pwm_b, SPEED2);
}

byte setSpeeds(String input)
{
  byte spd;
  if (input == "001")
    spd = 38;
  else if (input == "010")
    spd = 72;
  else if (input == "011")
    spd = 136;
  else if (input == "100")
    spd = 170;
  else if (input == "101")
    spd = 200;
  else if (input == "110")
    spd = 230;
  else if (input == "111")
    spd = 255;
  else
    spd = 0;
  return spd;
}

