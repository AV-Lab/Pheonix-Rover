#include <Arduino.h>
#include <Encoder.h>
#include <math.h>

// Defining Pins
#define right_encoder_pin 2
#define left_encoder_pin 21

#define right_pwm_pin 4
#define left_pwm_pin 5

#define right_relay_pin 52
#define left_relay_pin 24

// Initialization of encoders
Encoder rightEncoder(right_encoder_pin, right_encoder_pin);
Encoder leftEncoder(left_encoder_pin, left_encoder_pin);

// Variables for encoder readings
volatile int countA = 0;
volatile int countF = 0;

// Variables for RPM, speed & distance
unsigned long previousMillis = 0;
float interval = 100.0;

float leftRpm = 0.0;
int leftPwm = 0;
float leftSpeed = 0.0;
float measuredLeftSpeed = 0.0;

float rightRpm = 0.0;
int rightPwm = 0;
float rightSpeed = 0.0;
float measuredRightSpeed = 0.0;

float leftDistance = 0.0;
float rightDistance = 0.0;

// Robot position variables
float deltaDistance = 0.0;
float deltaTheta = 0.0;
float x = 0.0;
float y = 0.0;
float theta = 0.0;
float imuYaw = 0.0;

// Motor control variables
float targetLinearVelocity = 0.0;
float targetAngularVelocity = 0.0;

// Variables for filtering
#define BUFFER_SIZE 10
float leftEncoderBuffer[BUFFER_SIZE] = {0};
float rightEncoderBuffer[BUFFER_SIZE] = {0};
float leftEncoderSum = 0;
float rightEncoderSum = 0;
int bufferIndex = 0;
float spikeThreshold = 0.0;

// Function to calculate the variance
float calculateVariance(float* buffer, int size) {
    float mean = 0.0, variance = 0.0;
    for (int i = 0; i < size; i++) {
        mean += buffer[i];
    }
    mean /= size;
    
    for (int i = 0; i < size; i++) {
        variance += pow(buffer[i] - mean, 2);
    }
    variance /= size;
    return variance;
}

void setup() {
    // Serial setup
    Serial.begin(115200);

    // Speed PWM pin setup
    pinMode(right_pwm_pin, OUTPUT);
    pinMode(left_pwm_pin, OUTPUT);

    // Relay pin setup
    pinMode(right_relay_pin, OUTPUT);
    pinMode(left_relay_pin, OUTPUT);

    // Using pin 25 to power Relay
    pinMode(25, OUTPUT);
    digitalWrite(25, HIGH);

    // Using pin 26 to power 5v Intake Fan Power
    pinMode(26, OUTPUT);
    digitalWrite(26, HIGH);

    // Wait 2 seconds for start-up stability
    analogWrite(right_pwm_pin, 0);
    analogWrite(left_pwm_pin, 0);
    digitalWrite(right_relay_pin, HIGH);
    digitalWrite(left_relay_pin, HIGH);
    delay(2000);
}

void loop() {
    // Check for new commands from serial
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command.length() > 0) {
            int firstCommaIndex = command.indexOf(',');
            if (firstCommaIndex > 0) {
                int secondCommaIndex = command.indexOf(',', firstCommaIndex + 1);
                if (secondCommaIndex > 0) {
                    targetLinearVelocity = command.substring(0, firstCommaIndex).toFloat();
                    targetAngularVelocity = command.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
                    imuYaw = command.substring(secondCommaIndex + 1).toFloat();           
                    controlMotors(targetLinearVelocity, targetAngularVelocity);
                }
            }
        }
    }

    // Calculating RPM, speed & distance
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        unsigned long leftPulseCount = leftEncoder.read();
        unsigned long rightPulseCount = rightEncoder.read();
        leftEncoder.write(0);
        rightEncoder.write(0);

        // Update buffer with the current readings
        leftEncoderSum -= leftEncoderBuffer[bufferIndex];
        rightEncoderSum -= rightEncoderBuffer[bufferIndex];
        leftEncoderBuffer[bufferIndex] = leftPulseCount;
        rightEncoderBuffer[bufferIndex] = rightPulseCount;
        leftEncoderSum += leftPulseCount;
        rightEncoderSum += rightPulseCount;

        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

        // Calculate variance and adaptive threshold
        float leftVariance = calculateVariance(leftEncoderBuffer, BUFFER_SIZE);
        float rightVariance = calculateVariance(rightEncoderBuffer, BUFFER_SIZE);
        float adaptiveThresholdLeft = 3 * sqrt(leftVariance); // 3-sigma rule
        float adaptiveThresholdRight = 3 * sqrt(rightVariance);

        // Filtering encoder readings
        if (abs(leftPulseCount - leftEncoderBuffer[bufferIndex]) > adaptiveThresholdLeft) {
            leftPulseCount = leftEncoderBuffer[bufferIndex];  // Discard spike and use previous average
        }
        if (abs(rightPulseCount - rightEncoderBuffer[bufferIndex]) > adaptiveThresholdRight) {
            rightPulseCount = rightEncoderBuffer[bufferIndex];
        }

        // Calculating speed depending on direction (+/-)
        const float wheelD = 0.143;
        if (digitalRead(left_relay_pin) == HIGH) {
            leftDistance = (leftPulseCount * PI * wheelD) / 230;
        } else {
            leftDistance = ((leftPulseCount * PI * wheelD) / 230) * -1;
        }

        if (digitalRead(right_relay_pin) == HIGH) {
            rightDistance = (rightPulseCount * PI * wheelD) / 230;
        } else {
            rightDistance = ((rightPulseCount * PI * wheelD) / 230) * -1;
        }

        float linearVelocity = ((rightDistance + leftDistance) / 2.0) / 0.1;
        float angularVelocity = ((rightDistance - leftDistance) / 0.327) / 0.1;

        deltaDistance = (leftDistance + rightDistance) / 2.0;
        deltaTheta = (rightDistance - leftDistance) / 0.327;

        x += deltaDistance * cos(imuYaw);
        y += deltaDistance * sin(imuYaw);
        theta += deltaTheta;

        // Send odometry data to serial
        Serial.print(x);
        Serial.print(",");
        Serial.print(y);
        Serial.print(",");
        Serial.print(theta);
        Serial.print(",");
        Serial.print(linearVelocity);
        Serial.print(",");
        Serial.println(angularVelocity);
    }
}

void controlMotors(float linear, float angular) {

    bool unidirectional = ((angular == 0 && linear != 0) || (angular != 0 && linear == 0));
    bool leftZero = ((angular > 0 && linear > 0)||(angular < 0 && linear < 0));
    bool rightZero = ((angular < 0 && linear > 0)||(angular > 0 && linear < 0));
    bool stop = (angular == 0 && linear == 0);

    leftSpeed = (linear - (angular * 0.1635)) * 100;
    rightSpeed = (linear + (angular * 0.1635)) * 100;

    if (stop) {
        leftPwm = 0;
        rightPwm = 0;
        analogWrite(left_pwm_pin, leftPwm);
        analogWrite(right_pwm_pin, rightPwm);
    }

    if (unidirectional) {
        if (leftSpeed > 0) {
            leftPwm = int((leftSpeed + 8.14) / 0.56) + 1;
            leftPwm = constrain(leftPwm, 40, 255);
            digitalWrite(left_relay_pin, HIGH);
            delay(200);
            analogWrite(left_pwm_pin, leftPwm);
        } else if (leftSpeed < 0) {
            leftPwm = (int((leftSpeed - 8.14) / 0.56) - 1 ) * -1;
            leftPwm = constrain(leftPwm, 40, 255);
            digitalWrite(left_relay_pin, LOW);
            delay(200);
            analogWrite(left_pwm_pin, leftPwm);
        } else {
            leftPwm = 0;
            analogWrite(left_pwm_pin, leftPwm);
        }

        if (rightSpeed > 0) {
            rightPwm = int((rightSpeed + 8.14) / 0.56);
            rightPwm = constrain(rightPwm, 40, 255);
            digitalWrite(right_relay_pin, HIGH);
            delay(200);
            analogWrite(right_pwm_pin, rightPwm);
        } else if (rightSpeed < 0) {
            rightPwm = int((rightSpeed - 8.14) / 0.56) * -1;
            rightPwm = constrain(rightPwm, 40, 255);
            digitalWrite(right_relay_pin, LOW);
            delay(200);
            analogWrite(right_pwm_pin, rightPwm);
        } else {
            rightPwm = 0;
            analogWrite(right_pwm_pin, rightPwm);
        }
    }

    if (leftZero) {
        if (rightSpeed > 0) {
            rightPwm = int((rightSpeed + 8.14) / 0.56);
            rightPwm = constrain(rightPwm, 40, 255);
            leftSpeed = constrain(leftSpeed, 0.0, 1.4);
            leftPwm = int((leftSpeed + 8.14) / 0.56) + 1;
            leftPwm = constrain(leftPwm, 40, 255);
            digitalWrite(right_relay_pin, HIGH);
            digitalWrite(left_relay_pin, HIGH);
            delay(200);
            analogWrite(right_pwm_pin, rightPwm);
            analogWrite(left_pwm_pin, leftPwm);
        } else if (rightSpeed < 0) {
            rightPwm = int((rightSpeed - 8.14) / 0.56) * -1;
            rightPwm = constrain(rightPwm, 40, 255);
            leftSpeed = constrain(leftSpeed, -1.4, 0.0);
            leftPwm = (int((leftSpeed - 8.14) / 0.56) - 1 ) * -1;
            leftPwm = constrain(leftPwm, 40, 255);
            digitalWrite(right_relay_pin, LOW);
            digitalWrite(left_relay_pin, LOW);
            delay(200);
            analogWrite(right_pwm_pin, rightPwm);
            analogWrite(left_pwm_pin, leftPwm);
        } else {
            rightPwm = 0;
            leftPwm = 0;
            analogWrite(right_pwm_pin, rightPwm);
            analogWrite(left_pwm_pin, leftPwm);
        }
    }

    if (rightZero) {
        if (leftSpeed > 0) {
            leftPwm = int((leftSpeed + 8.14) / 0.56) + 1;
            leftPwm = constrain(leftPwm, 40, 255);
            rightSpeed = constrain(rightSpeed, 0.0, 1.4);
            rightPwm = int((rightSpeed + 8.14) / 0.56) + 1;
            rightPwm = constrain(rightPwm, 40, 255);
            digitalWrite(left_relay_pin, HIGH);
            digitalWrite(right_relay_pin, HIGH);
            delay(200);
            analogWrite(left_pwm_pin, leftPwm);
            analogWrite(right_pwm_pin, rightPwm);
        } else if (leftSpeed < 0) {
            leftPwm = (int((leftSpeed - 8.14) / 0.56) - 1 ) * -1;
            leftPwm = constrain(leftPwm, 40, 255);
            rightSpeed = constrain(rightSpeed, -1.4, 0.0);
            rightPwm = (int((rightSpeed - 8.14) / 0.56) - 1 ) * -1;
            rightPwm = constrain(rightPwm, 40, 255);
            digitalWrite(left_relay_pin, LOW);
            digitalWrite(right_relay_pin, LOW);
            delay(200);
            analogWrite(left_pwm_pin, leftPwm);
            analogWrite(right_pwm_pin, rightPwm);
        } else {
            leftPwm = 0;
            rightPwm = 0;
            analogWrite(left_pwm_pin, leftPwm);
            analogWrite(right_pwm_pin, rightPwm);
        }
    }
}
