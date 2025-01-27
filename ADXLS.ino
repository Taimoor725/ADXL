#include <Wire.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Preferences.h>

// Wi-Fi credentials
const char* ssid = "Tanveer";
const char* password = "Tanveer5890";

// Firebase configuration
#define FIREBASE_HOST "https://fyp-sensor-readings-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "AIzaSyCt56rUgXAWDcAOEFwUmO5WXI_Je6GE4ik"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

Preferences preferences;

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

// Multiplexer and ADXL345 I2C addresses
#define PCA9548A_ADDRESS 0x70
#define ADXL345_ADDRESS 0x53

// ADXL345 register addresses
#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_REG_DATAX0    0x32

// Sensitivity in g/LSB (assuming ±2g)
float sensitivity = 0.0039;  // 0.0039 g per LSB for ±2g range

// Last processed sensor index and reset cycle count
uint8_t lastSensorIndex = 0;
uint8_t resetValue = 0;  // Track the reset value (i.e., how many full cycles of all sensors)

void initADXL345() {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(ADXL345_REG_POWER_CTL);  
  Wire.write(0x08);  // Set the device to measure mode
  Wire.endTransmission();
}

void readADXL345(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(ADXL345_REG_DATAX0);  // Start reading from the data registers
  Wire.endTransmission();
  
  Wire.requestFrom(ADXL345_ADDRESS, 6);  // Request 6 bytes of data
  
  if (Wire.available() == 6) {
    x = Wire.read() | (Wire.read() << 8);  // Combine 2 bytes for X axis
    y = Wire.read() | (Wire.read() << 8);  // Combine 2 bytes for Y axis
    z = Wire.read() | (Wire.read() << 8);  // Combine 2 bytes for Z axis
  }
}

void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1 << channel);  // Select the channel on the multiplexer
  Wire.endTransmission();
  
  delay(100);
}

void connectWiFi() {
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void sendDataToFirebase(uint8_t sensorNumber, float ax, float ay, float az) {
  String path = "/SensorData/ADXL" + String(sensorNumber) + "/" + String(resetValue);
  FirebaseJson json;

  json.set("X", ax);
  json.set("Y", ay);
  json.set("Z", az);

  // Push data to Firebase with sequential keys
  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    Serial.print("Sensor ");
    Serial.print(sensorNumber);
    Serial.print(" data pushed to Firebase (Cycle: ");
    Serial.print(resetValue);
    Serial.println(") successfully.");
  } else {
    Serial.print("Firebase Error: ");
    Serial.println(fbdo.errorReason());
  }
}

void updateResetValue() {
  resetValue++;  // Increment reset value after all sensors are processed
  Firebase.RTDB.setInt(&fbdo, "/reset", resetValue);  // Save the reset value to Firebase
  Serial.print("Reset value updated to: ");
  Serial.println(resetValue);
}

void loadLastSensorIndex() {
  lastSensorIndex = preferences.getUInt("lastSensor", 0);  // Default to 0 if no data found
  Serial.print("Resuming from sensor index: ");
  Serial.println(lastSensorIndex);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize preferences storage
  preferences.begin("sensor_data", false);

  connectWiFi();

  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("Initializing PCA9548A and ADXL345 accelerometers...");

  loadLastSensorIndex();

  // Initialize multiplexer and ADXL345
  for (uint8_t i = 0; i < 8; i++) {
    selectMuxChannel(i);  // Select channel i
    initADXL345();        // Initialize ADXL345 on the current channel
    delay(100);
  }
  
  Serial.println("All ADXL345 accelerometers initialized.");
}

void loop() {
  int16_t x, y, z;
  float ax, ay, az;
  
  // Read data from all ADXL345 accelerometers starting from the last index
  for (uint8_t i = lastSensorIndex; i < 8; i++) {
    selectMuxChannel(i);  // Select the channel on the multiplexer
    delay(50);  // Allow time for channel to settle

    // Read the accelerometer data
    readADXL345(x, y, z);  
    
    // Convert the data to m/s²
    ax = x * sensitivity * 9.81;  // Convert to m/s² for X-axis
    ay = y * sensitivity * 9.81;  // Convert to m/s² for Y-axis
    az = z * sensitivity * 9.81;  // Convert to m/s² for Z-axis

    // Display the sensor data on the Serial Monitor
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" - X: ");
    Serial.print(ax);
    Serial.print(" m/s², Y: ");
    Serial.print(ay);
    Serial.print(" m/s², Z: ");
    Serial.print(az);
    Serial.println(" m/s²");

    // Send data to Firebase
    sendDataToFirebase(i + 1, ax, ay, az);

    // Update the last sensor index
    lastSensorIndex = i + 1;
    preferences.putUInt("lastSensor", lastSensorIndex);
  }

  // After completing a cycle of all sensors, increment reset value
  updateResetValue();

  // Reset index to 0 for next loop cycle
  lastSensorIndex = 0;
  preferences.putUInt("lastSensor", lastSensorIndex);

  delay(500);  
}
