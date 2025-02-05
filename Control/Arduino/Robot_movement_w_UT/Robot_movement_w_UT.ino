// Define ultrasonic sensor pins
#define TRIG1 1
#define ECHO1 2


void setup() {
    pinMode(TRIG1, OUTPUT);
    pinMode(ECHO1, INPUT);

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


void loop() {
    long distance1 = getDistance(TRIG1, ECHO1);
    Serial.print("Distance1: "); Serial.print(distance1);
    
    delay(100);
}