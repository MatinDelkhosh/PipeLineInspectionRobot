// Define ultrasonic sensor pins
#define TRIG1 8
#define ECHO1 10
#define TRIG2 9
#define ECHO2 11

// Define motor control pins
#define MOTOR_LEFT_IN1 1
#define MOTOR_LEFT_IN2 2
#define MOTOR_LEFT_ENA 5
#define MOTOR_RIGHT_IN1 4
#define MOTOR_RIGHT_IN2 3
#define MOTOR_RIGHT_ENB 6

void setup() {
    pinMode(TRIG1, OUTPUT);
    pinMode(ECHO1, INPUT);
    pinMode(TRIG2, OUTPUT);
    pinMode(ECHO2, INPUT);
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);
    pinMode(MOTOR_LEFT_ENA, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);
    pinMode(MOTOR_RIGHT_ENB, OUTPUT);

    pinMode(A0, INPUT);
    Serial.begin(9600);
}

long getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2; // Convert to cm
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    if (leftSpeed > 0) {
        digitalWrite(MOTOR_LEFT_IN1, HIGH);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
    } else {
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, HIGH);
        leftSpeed = -leftSpeed;
    }
    
    if (rightSpeed > 0) {
        digitalWrite(MOTOR_RIGHT_IN1, HIGH);
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
    } else {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);
        digitalWrite(MOTOR_RIGHT_IN2, HIGH);
        rightSpeed = -rightSpeed;
    }
    
    analogWrite(MOTOR_LEFT_ENA, constrain(leftSpeed, 0, 255));
    analogWrite(MOTOR_RIGHT_ENB, constrain(rightSpeed, 0, 255));
}

void loop() {
    long distance1 = getDistance(TRIG1, ECHO1);
    long distance2 = getDistance(TRIG2, ECHO2);
    int pwmInput = analogRead(A0);
    int baseSpeed = 10;

    int difference = distance1 - distance2;
    int turnFactor = map(abs(difference), 0, 10, 0, baseSpeed);
    
    int leftSpeed = baseSpeed;
    int rightSpeed = baseSpeed;
    
    if (difference > 0) {
        // Turn right
        leftSpeed += turnFactor;
        rightSpeed -= turnFactor;
    } else if (difference < 0) {
        // Turn left
        leftSpeed -= turnFactor;
        rightSpeed += turnFactor;
    }
    
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    setMotorSpeed(leftSpeed, rightSpeed);
    
    Serial.print("PWM Input: "); Serial.print(pwmInput);
    Serial.print("Distance1: "); Serial.print(distance1);
    Serial.print(" cm, Distance2: "); Serial.print(distance2);
    Serial.print(" cm, Left Speed: "); Serial.print(leftSpeed);
    Serial.print(", Right Speed: "); Serial.println(rightSpeed);
    
    delay(100);
}