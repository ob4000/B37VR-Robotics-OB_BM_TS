
#include <NewPing.h>

#define TRIGGER_PIN 11//dist sensor
#define ECHO_PIN 12//   dist sensor
#define MAX_DISTANCE 200

#define MOT_A1_PIN 5//right wheel
#define MOT_A2_PIN 6//right wheel
#define MOT_B1_PIN 9//left wheel
#define MOT_B2_PIN 10//left wheel

// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

/* variables for the speed of motors */
int leftServoSpeed = 200-23+25-3+1;
int rightServoSpeed = 200-25-10;
 //guess what
void setup()
{
  // Initialize the stepper driver control pins to output drive mode.
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  // Start with drivers off, motors coasting.
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

  //Initialise the serian UART at 9600 bps (bits per second).
  Serial.begin(9600);
}

/// \param pwm
/// \param IN1_PIN
/// \param IN2_PIN

void setMotorPwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255

void setMotorCurrents(int pwm_A, int pwm_B)
{
  setMotorPwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
  setMotorPwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);
}

/// \param pwm_A  motor A PWM, -255 to 255
/// \param pwm_B  motor B PWM, -255 to 255
/// \param duration delay in milliseconds
//chicken 🐔 butt

void spin(int pwm_A, int pwm_B)
{
  setMotorCurrents(pwm_A, pwm_B);
}

unsigned int look()
{
  unsigned int uS = sonar.ping_cm();
  Serial.print(uS);
  Serial.println("cm");
  return uS;
}

void loop()
{
  unsigned int dist = look();
  unsigned int current = look();

  if(dist > 23 || dist == 0)
  {
    spin(leftServoSpeed,rightServoSpeed); // sets speed of motors to value entered above for 0.5 sec and keeps repeating
  }
  else
  {
    while(true)
    {
      spin(-leftServoSpeed, -rightServoSpeed);
      delay(1000);
      spin(leftServoSpeed, -rightServoSpeed);
      delay(500);
      unsigned int dif = current - look();
      if(dif > 23)
      {
        break;
      }
    }
  }
}
