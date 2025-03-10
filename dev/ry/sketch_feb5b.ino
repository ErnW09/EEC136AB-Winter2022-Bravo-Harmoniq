#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BleKeyboard.h>

// BLE Keyboard instance
BleKeyboard bleKeyboard("Motion BLE Controller");

// MPU6050 instance
Adafruit_MPU6050 mpu;

// Arrays to store calibration data for 4 positions
float accel_data[4][3]; // Stores (x, y, z) accelerometer readings

bool is_calibrated = false;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE and MPU6050...");

    // Initialize BLE Keyboard
    bleKeyboard.begin();

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found. Check wiring!");
        while (1);
    }
    Serial.println("MPU6050 Initialized!");

    // Start Calibration
    calibrateArmPositions();
}

void calibrateArmPositions() {
    Serial.println("== CALIBRATION START ==");

    for (int i = 0; i < 4; i++) {
        String pos_name;
        switch (i) {
            case 0: pos_name = "Straight Arm (No Action)"; break;
            case 1: pos_name = "Bent Arm (Next Track)"; break;
            case 2: pos_name = "Random Position 1 (Volume Up)"; break;
            case 3: pos_name = "Random Position 2 (Volume Down)"; break;
        }

        Serial.println("Place your arm in: " + pos_name);
        countdown(3);
        recordReference(i);
        Serial.println(pos_name + " recorded!");
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
    for (int i = sec; i > 0; i--) {
        Serial.print(i);
        Serial.println("...");
        delay(1000);
    }
}

// Compare current readings with the calibrated positions
int detectArmPosition() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    int best_match_index = 0;
    float min_difference = 1e6; // Large number to find min value

    for (int i = 0; i < 4; i++) {
        float accel_diff = abs(a.acceleration.x - accel_data[i][0]) + 
                           abs(a.acceleration.y - accel_data[i][1]) + 
                           abs(a.acceleration.z - accel_data[i][2]);

        if (accel_diff < min_difference) {
            min_difference = accel_diff;
            best_match_index = i;
        }
    }

    return best_match_index; // Return index of the best-matching position
}

void loop() {
    if (is_calibrated && bleKeyboard.isConnected()) {
        Serial.println("Detecting arm position...");
        int position = detectArmPosition();
        executeBLEAction(position);
        delay(1500); // Sample every 3 seconds
    } else {
        Serial.println("Waiting for BLE connection...");
        delay(1000);
    }
}

// Map positions to BLE actions
void executeBLEAction(int position) {
    switch (position) {
        case 0:
            Serial.println("Straight Arm - No action.");
            break;
        case 1:
            Serial.println("Bent Arm - Next Track.");
            bleKeyboard.write(KEY_MEDIA_NEXT_TRACK);
            break;
        case 2:
            Serial.println("Random Position 1 - Volume Up.");
            for (int i = 0; i < 2; i++) { // Increase volume 3 steps
                bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
                delay(100);
            }
            break;
        case 3:
            Serial.println("Random Position 2 - Volume Down.");
            for (int i = 0; i < 2; i++) { // Decrease volume 3 steps
                bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
                delay(100);
            }
            break;
        default:
            Serial.println("Unknown Position - No action.");
    }
}


