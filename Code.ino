#include <Wire.h>
#include <LiquidCrystal_AIP31068_I2C.h>
#include <DHT.h>

// Pin Definitions
const int RX_PIN = 0;                   
const int TX_PIN = 1;                   
const int WITHIN_SETPOINT_PIN = 4;     
const int DHT_PIN = 5;                 
const int TRIG_PIN = 7;                 
const int ECHO_PIN = 8;                 
const int CONTROLLER_OUTPUT_PIN = 9;    
const int ABOVE_SETPOINT_PIN = 10;      
const int BELOW_SETPOINT_PIN = 11;      
const int MOTOR_PIN = 12;               
const int LCD_SCL_PIN = A5;            
const int LCD_SDA_PIN = A4;            
const int SETPOINT_PIN = A1;            
const int SYSTEM_OUTPUT_PIN = A0;       

// Constants
const int DHT_TYPE = DHT22;                  
const float TEMP_THRESHOLD_HIGH = 50.0;      
const float TEMP_THRESHOLD_LOW = 30.0;       
const float DISTANCE_TOLERANCE = 0.05;       
const unsigned long CONTROL_INTERVAL = 10;    
const int SERIAL_BAUD = 9600;                 
const int MOVING_AVG_SAMPLES = 10;          
const int MAX_DISTANCE = 400;                 
const float VOLTAGE_REF = 5.0;                

// LCD Configuration
const int LCD_ADDR = 0x3E;                   
const int LCD_COLS = 16;                      
const int LCD_ROWS = 3;                       


// Initialize Objects
LiquidCrystal_AIP31068_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
DHT dht(DHT_PIN, DHT_TYPE);

// PID Control Variables - Adjusted for small overshoot
double Setpoint, Input, Output;

// Control variables for PI controller
float Kp = 0.14; // Proportional gain (adjusted for slight overshoot)
float Ki = 5.00; // Integral gain (adjusted for slight overshoot)
float integralTerm = 0.0;
float previousError = 2.0;
float controlOutput = 0.0; // Store the control output

// Global Variables
unsigned long previousMillis = 0; 
float distanceBuffer[MOVING_AVG_SAMPLES];
int bufferIndex = 0;
float temperature = 0.0;
float humidity = 0.0;
bool motorState = false;

// Variables for LED flashing
unsigned long flashInterval = 3;       // Interval in milliseconds for flashing
unsigned long steadyOnDuration = 0;   // Initial delay before flashing, in milliseconds
unsigned long lastFlashTime = 0;         // Timestamp for the last LED change
unsigned long aboveSetpointStartTime = 0; // Timestamp when distance first exceeded the setpoint
bool ledState = false;                   // Current state of the LED
bool aboveSetpoint = false;              // Track if above setpoint

void setup() {
    Serial.begin(SERIAL_BAUD);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(WITHIN_SETPOINT_PIN, OUTPUT);
    pinMode(ABOVE_SETPOINT_PIN, OUTPUT);
    pinMode(BELOW_SETPOINT_PIN, OUTPUT);

    Wire.begin();
    lcd.init();
    lcd.on();
    lcd.clear();

    dht.begin();

    // Initialize moving average buffer
    float initialDistance = readDistance();
    for (int i = 0; i < MOVING_AVG_SAMPLES; i++) {
        distanceBuffer[i] = initialDistance;
    }
    Serial.println("Reading Data");
}

// Function to read distance
float readDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 23529); // Timeout after ~4m round trip
    if (duration == 0) return MAX_DISTANCE;
    return duration * 0.034 / 2.4; // Convert duration to distance in cm
}

// Moving average filter
float getFilteredDistance() {
    float newReading = readDistance();
    distanceBuffer[bufferIndex] = newReading;
    bufferIndex = (bufferIndex + 1) % MOVING_AVG_SAMPLES;

    float sum = 0;
    for (int i = 0; i < MOVING_AVG_SAMPLES; i++) {
        sum += distanceBuffer[i];
    }
    return sum / MOVING_AVG_SAMPLES;
}

// LED control 
void updateLEDs(float distance, float setpoint) {
    float error = abs(distance - setpoint);
    float tolerance = setpoint * DISTANCE_TOLERANCE;

    // Check if the distance is above the setpoint
    if (distance > setpoint + tolerance) {
        // If just entering the above setpoint range, capture the start time
        if (!aboveSetpoint) {
            aboveSetpoint = true;
            aboveSetpointStartTime = millis(); // Record when we first exceeded the setpoint
            digitalWrite(ABOVE_SETPOINT_PIN, HIGH); // Turn LED on initially
            ledState = true; // Set LED state to on
        }
        // Check if we've been above the setpoint for the steadyOnDuration
        unsigned long currentMillis = millis();
        if (currentMillis - aboveSetpointStartTime >= steadyOnDuration) {
            // Start flashing the LED
            if (currentMillis - lastFlashTime >= flashInterval) {
                lastFlashTime = currentMillis; // Update last flash time
                ledState = !ledState; // Toggle LED state
                digitalWrite(ABOVE_SETPOINT_PIN, ledState ? HIGH : LOW); // Set LED state
            }
        }
    } else {
        // Reset LED and variables when below setpoint
        digitalWrite(ABOVE_SETPOINT_PIN, LOW);
        ledState = false;
        aboveSetpoint = false;
    }

    // Control the other LEDs
    digitalWrite(WITHIN_SETPOINT_PIN, error <= tolerance);
    digitalWrite(BELOW_SETPOINT_PIN, distance < setpoint - tolerance);
}

// PI control update
void updatePIControl() {
    // Read the distance as the system output and calculate the error
    float setpoint = analogRead(SETPOINT_PIN) * (MAX_DISTANCE / 1023.0);
    float actualOutput = getFilteredDistance();
    float error = setpoint - actualOutput;

    // Proportional control
    float proportionalTerm = Kp * error;
 
    // Integral control
    integralTerm += error * (CONTROL_INTERVAL / 1495.0); // Accumulate error over time
    float integralControl = Ki * integralTerm;

    // Combine proportional and integral terms to compute the control output
    controlOutput = proportionalTerm + integralControl;

    // Constrain the control output to fit within PWM limits (0-255)
    controlOutput = constrain(controlOutput, 0, 255);

    // Apply control output to the PWM pin
    analogWrite(CONTROLLER_OUTPUT_PIN, controlOutput);
}

// Update LCD
void updateLCD() {
    //lcd.clear();

    // Display and print distance
    float filteredDistance = getFilteredDistance() / 0.8007;
    lcd.setCursor(0, 0);
    lcd.print("Distance:");
    lcd.print(filteredDistance, 2);
    lcd.setCursor(15, 0);
    lcd.print("cm");

    Serial.print("Distance: ");
    Serial.print(filteredDistance, 2);
    Serial.print(" cm |");

    // Display and print setpoint
    float setpointVoltage = ((analogRead(SETPOINT_PIN) * (MAX_DISTANCE / 1023.0)) * 0.01) / 0.8007;
    lcd.setCursor(0, 1);
    lcd.print("Setpoint:");
    lcd.print(setpointVoltage, 2);
    lcd.setCursor(14, 1);
    lcd.print("V");

    Serial.print("Setpoint: ");
    Serial.print(setpointVoltage, 2);
    Serial.print(" V |");

    // Display and print temperature
    lcd.setCursor(0, 2);
    lcd.print("Temp:");
    lcd.print(temperature, 2);
    lcd.setCursor(11, 2);
    lcd.print("C");

    Serial.print("Temp: ");
    Serial.print(temperature, 2);
    Serial.print(" C |");

    // Display and print humidity
    lcd.setCursor(0, 3);
    lcd.print("Hum:");
    lcd.print(humidity, 2);
    lcd.setCursor(10, 3);
    lcd.print("%");

    Serial.print("Humidity: ");
    Serial.print(humidity, 2);
    Serial.print(" % |");

    //print motor status
    Serial.print("Motor Status: ");
    Serial.println(motorState ? "ON" : "OFF");
}

// Update motor
void updateMotor() {
    if (temperature > TEMP_THRESHOLD_HIGH && !motorState) {
        motorState = true;
        digitalWrite(MOTOR_PIN, HIGH);
    } else if (temperature < TEMP_THRESHOLD_LOW && motorState) {
        motorState = false;
        digitalWrite(MOTOR_PIN, LOW);
    }
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - previousMillis >= CONTROL_INTERVAL) {
        previousMillis = currentTime;

        // Read sensors
        temperature = dht.readTemperature();
        humidity = dht.readHumidity();


        if (isnan(temperature) || isnan(humidity)) {
            Serial.println("Failed to read from DHT sensor!"); 
        } else { 
            // Perform control updates
            updatePIControl();
            updateMotor();
            updateLEDs(getFilteredDistance(), analogRead(SETPOINT_PIN) * (MAX_DISTANCE / 1023.0));

            // Update LCD every 400 ms
            static unsigned long lastDisplayUpdate = 0;
            if (currentTime - lastDisplayUpdate >= 400) {
                updateLCD();
                lastDisplayUpdate = currentTime;
            }
        }
    }
}