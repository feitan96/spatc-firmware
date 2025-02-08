#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

// WiFi credentials
#define WIFI_SSID "konek"
#define WIFI_PASSWORD "passwords"

// Firebase Setup
#define FIREBASE_HOST "https://spatc-46809-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "AIzaSyDLwSbPMUnfDZn2kpgJ32cVeC4n4lnZ3TI"

FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// GPS Setup
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);
#define RXD2 16
#define TXD2 17

// Ultrasonic Sensor Setup
#define TrigPin 5   // Trigger pin
#define EchoPin 18  // Echo pin

String gpsData = "GPS Not Initialized";
float distanceCm = 0.0;

// Function declarations
void updateGPSData();
float measureDistance();
void sendToFirebase();

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nConnected to WiFi!");

  // Configure Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH; // ✅ Correct
  auth.user.email = "";   // ✅ Needed to avoid empty auth issue
  auth.user.password = ""; 
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Connected to Firebase!");

  // GPS Initialization
  GPS_Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Initializing GPS Module...");

  // Ultrasonic Sensor Initialization
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
}

void loop() {
  updateGPSData();
  distanceCm = measureDistance();
  
  // Print data
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  // Send data to Firebase
  sendToFirebase();

  delay(2000); // Update every 2 seconds
}

// Update GPS Data
void updateGPSData() {
  static bool gpsLocked = false;
  static bool gpsInitialized = false;

  while (GPS_Serial.available() > 0) {
    if (gps.encode(GPS_Serial.read())) {
      if (gps.location.isValid()) {
        if (!gpsInitialized) {
          Serial.println("GPS Powered On and Searching for Satellites...");
          gpsInitialized = true;
        }

        // If a new GPS fix is available
        if (gps.location.isUpdated()) {
          double latitude = gps.location.lat();
          double longitude = gps.location.lng();
          double altitude = gps.altitude.meters();
          int satellites = gps.satellites.value();

          // Update GPS Data
          gpsData = "Latitude: " + String(latitude, 6) +
                    "  Longitude: " + String(longitude, 6) +
                    "  Altitude: " + String(altitude) + "m" +
                    "  Satellites: " + String(satellites);
          
          Serial.println(gpsData);
          gpsLocked = true;
        }
      } else {
        if (gpsLocked) {
          gpsLocked = false;
          gpsData = "GPS Searching for satellites...";
          Serial.println("GPS Searching for satellites...");
        }
      }
    }
  }
}

// Measure distance using ultrasonic sensor
float measureDistance() {
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  long duration = pulseIn(EchoPin, HIGH);
  float distance = duration * 0.0343 / 2; // Convert to cm

  return distance;
}

// Send GPS & Ultrasonic data to Firebase
void sendToFirebase() {
  Serial.println("Sending data to Firebase...");

  // GPS Data
  if (Firebase.setFloat(firebaseData, "bin/gps/latitude", gps.location.lat())) {
    Serial.println("Latitude sent!");
  } else {
    Serial.print("Firebase Error: ");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, "bin/gps/longitude", gps.location.lng())) {
    Serial.println("Longitude sent!");
  } else {
    Serial.print("Firebase Error: ");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, "bin/gps/altitude", gps.altitude.meters())) {
    Serial.println("Longitude sent!");
  } else {
    Serial.print("Firebase Error: ");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, "bin/distance(cm)", distanceCm)) {
    Serial.println("Distance sent!");
  } else {
    Serial.print("Firebase Error: ");
    Serial.println(firebaseData.errorReason());
  }
}
