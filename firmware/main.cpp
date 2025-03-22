// Library
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BleKeyboard.h>
#include <driver/adc.h>

// Function Declarations for BLE Keyboard
void calibrateArmPositions();
void countdown(int sec);
void recordReference(int index);
void executeBLEAction(int position);

// Redefined I2C Pins
#define SDA_PIN 22  // Your new SDA pin
#define SCL_PIN 23  // Your new SCL pin

// LED Definition
#define LED_BUILTIN 2
#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// ADC Definition
#define ADC_PIN 39

// BLE Keyboard instance
BleKeyboard bleKeyboard("Motion BLE Controller");

// Accelerometer instance
Adafruit_MPU6050 mpu;

// Voltage Threshold
const float VOLTAGE_THRESHOLD = 0.1;

// ADC Vars
const unsigned long COOLDOWN_TIME = 1500;
unsigned long lastCommandTime = 0;

// Arrays to store calibration data for 5 positions
float accel_data[5][3]; // Stores (x, y, z) accelerometer readings

bool is_calibrated = false;

void setup() {

    // LED Setup
    pinMode(LED_BUILTIN, OUTPUT);
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(LED_BUILTIN, PWM_CHANNEL);

    // Serial Terminal Setup
    Serial.begin(115200); // PuTTY Baud Rate
    Serial.print("\033[2J"); // Clear terminal for clarity
    Serial.println("Starting BLE and MPU6050...\n");

    // Initialize BLE Keyboard
    bleKeyboard.begin();

    // Initialize MPU6050
    Wire.begin(SDA_PIN, SCL_PIN); 
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found. Check wiring!\n");
        while (1);
    }
    Serial.println("MPU6050 Initialized!\n");

    // Start Calibration
    delay(3000); // Gives user a moment before sudden start
    calibrateArmPositions();
}

// Generate User Profile
void calibrateArmPositions() {
    Serial.println("== CALIBRATION START ==\n");

    for (int i = 0; i < 5; i++) {
        String pos_name;
        
        switch (i) {
            case 0: pos_name = "Straight Arm (No Action)"; 
            break;
            
            case 1: pos_name = "Bent Arm (Next Track)"; 
            break;
            
            case 2: pos_name = "Random Position 1 (Volume Up)"; 
            break;
            
            case 3: pos_name = "Random Position 2 (Volume Down)"; 
            break;

            case 4: pos_name = "Random Position 3 (Pause/Play)"; 
            break;
        }

        Serial.println("Place your arm in: " + pos_name);
        countdown(5);
        recordReference(i);
        Serial.println(pos_name + " recorded!\n");
        delay(3000); // Between calibration tests
    }

    is_calibrated = true;
    Serial.println("== CALIBRATION COMPLETE ==");
}

void recordReference(int index) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Store accelerometer readings
    accel_data[index][0] = a.acceleration.x;
    accel_data[index][1] = a.acceleration.y;
    accel_data[index][2] = a.acceleration.z;
}

void countdown(int sec) {
    int dutyCycle = 0;
    for (int i = sec; i > 0; i--) {
        dutyCycle = dutyCycle + 51;
        Serial.print(i);
        Serial.println("...");
        ledcWrite(PWM_CHANNEL, dutyCycle);
        delay(1000);
    }
    ledcWrite(PWM_CHANNEL, 0); // Between tests
}

// Fist Boolean
bool isFistClosed() {
    int raw_Value = analogRead(ADC_PIN);
    float voltage = (raw_Value * 3.3) / 4095.0;
    return voltage > VOLTAGE_THRESHOLD;
}

// Compare current readings with the calibrated positions
int detectArmPosition() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    int best_match_index = 0;
    float min_difference = 1e6; // Large number to find min value

    for (int i = 0; i < 5; i++) {
        float accel_diff = abs(a.acceleration.x - accel_data[i][0]) + abs(a.acceleration.y - accel_data[i][1]) + abs(a.acceleration.z - accel_data[i][2]);

        if (accel_diff < min_difference) {
            min_difference = accel_diff;
            best_match_index = i;
        }
    }

    return best_match_index; // Return index of the best-matching position
}

void loop() {
    if (is_calibrated && bleKeyboard.isConnected()) {
        if (isFistClosed()) {
            unsigned long currentTime = millis();
            if (currentTime - lastCommandTime >= COOLDOWN_TIME) { // prevent repeating actions in small time window
                int position = detectArmPosition();
                executeBLEAction(position);
                lastCommandTime = currentTime;
            }
        }
    } 
    else {
        Serial.println("Waiting for BLE connection...");
    }
    delay(1);
}

// Map positions to BLE actions
void executeBLEAction(int position) {

    switch (position) {

        case 0:
            Serial.println("Straight Arm - No action.\n");
            break;

        case 1:
            Serial.println("Bent Arm - Next Track.\n");
            bleKeyboard.write(KEY_MEDIA_NEXT_TRACK);
            break;

        case 2:
            Serial.println("Random Position 1 - Volume Up.\n");
            for (int i = 0; i < 2; i++) {
                bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
                delay(100);
            }
            break;

        case 3:
            Serial.println("Random Position 2 - Volume Down.\n");
            for (int i = 0; i < 2; i++) {
                bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
                delay(100);
            }
            break;

        case 4:
            Serial.println("Random Position 3 - Pause/Play.\n");
            bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE);
            break;

        default:
            Serial.println("Unknown Position - No action.\n");
            
    }
}