#include <TinyGPS++.h>
#include <HardwareSerial.h>

// GPS Setup
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);
#define RXD2 16
#define TXD2 17

// Ultrasonic Sensor Setup
#define TrigPin 5  // Define trigger pin
#define EchoPin 18 // Define echo pin

String gpsData = "GPS Not Initialized";
float distanceCm = 0.0;

// Function declarations
void handleGPS();
void updateGPSData();
float measureDistance();

void setup() {
  Serial.begin(115200);

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

  // Print both GPS and Ultrasonic Data
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  delay(1000); // Delay for stability
}

// Handle GPS data request
void handleGPS() {
  Serial.println("GPS Data Request: " + gpsData);
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
