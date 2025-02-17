// ESP32 Pin Locations - Motors 1 and 2
int motor1Pin1 = 33; 
int motor1Pin2 = 32; 
int enable1Pin = 4; 
int motor2Pin1 = 26; 
int motor2Pin2 = 25; 
int enable2Pin = 5; 

// ESP32 Pin Locations - Ultrasonic Sensors
const int trigPinA = 12;
const int echoPinA = 13;
const int trigPinB = 19;
const int echoPinB = 18;

// PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int speed = 200;

void setup() {
    Serial.begin(115200);

    // Set motor pins as outputs
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(enable1Pin, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(enable2Pin, OUTPUT);

    // Set ultrasonic sensor pins
    pinMode(trigPinA, OUTPUT);
    pinMode(echoPinA, INPUT);
    pinMode(trigPinB, OUTPUT);
    pinMode(echoPinB, INPUT);

    // Attach PWM channels using your original syntax
    ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);
    ledcAttachChannel(enable2Pin, freq, resolution, pwmChannel);
}

// Function to measure distance from Sensor A
float getDistanceA() {
    digitalWrite(trigPinA, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinA, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinA, LOW);

    long duration = pulseIn(echoPinA, HIGH);
    return (duration * 0.034) / 2; // Convert to cm
}

// Function to measure distance from Sensor B
float getDistanceB() {
    digitalWrite(trigPinB, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinB, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinB, LOW);

    long duration = pulseIn(echoPinB, HIGH);
    return (duration * 0.034) / 2; // Convert to cm
}

// Function to stop motors
void StopMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, 0);
    ledcWrite(enable2Pin, 0);
}

// Function to move forward
void ForwardMotors() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, speed);
    ledcWrite(enable2Pin, speed);
}

// Function to move backward
void ReverseMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    ledcWrite(enable1Pin, speed);
    ledcWrite(enable2Pin, speed);
}

// Function to turn left
void TurnLeft() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(enable1Pin, speed);
    ledcWrite(enable2Pin, speed);
}

// Function to turn right
void TurnRight() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    ledcWrite(enable1Pin, speed);
    ledcWrite(enable2Pin, speed);
}

void loop() {
    // Read ultrasonic sensor values
    float distanceA = getDistanceA();
    float distanceB = getDistanceB();

    // Send sensor data to Raspberry Pi over serial
    Serial.print("DIST ");
    Serial.print(distanceA);
    Serial.print(" ");
    Serial.println(distanceB);

    // Check if a command was received over serial
    if (Serial.available() > 0) {
        char command = Serial.read();
        switch (command) {
            case 'F': ForwardMotors(); break;
            case 'B': ReverseMotors(); break;
            case 'L': TurnLeft(); break;
            case 'R': TurnRight(); break;
            default: StopMotors(); break;
        }
    }
    
    delay(100); // Short delay to prevent overloading serial communication
}
