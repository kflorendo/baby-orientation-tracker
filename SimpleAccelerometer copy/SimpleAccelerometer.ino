/*
  Arduino LSM6DS3 - Simple Accelerometer

  This example reads the acceleration values from the LSM6DS3
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>
#include <Firebase_Arduino_WiFiNINA.h>

//Firebase settings
#define FIREBASE_HOST "baby-orientation-tracker.firebaseio.com"
#define FIREBASE_AUTH "SLGasL5enTeVGqfRRASjweU2JPSKhShWuj3w1Dwb"

FirebaseData firebaseData;

//Wi-Fi settings
#define WIFI_SSID "FiOS-3KFVU"
#define WIFI_PASS "hut0044tom7882try"
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);

//orientation variables
int cur_pos;
int last_pos;
int diff_count = 0;
int change_threshold = 15;

bool debugMode = false;

void setup() {
  pinMode(13, OUTPUT);
  if (debugMode) {
    Serial.begin(9600);
    while (!Serial);
  }

  /* ------------- INITIALIZING IMU ------------- */
  if (!IMU.begin()) {
    if (debugMode) {
      Serial.println("Failed to initialize IMU!");
    }

    while (1);
  }

  if (debugMode) {
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Acceleration in G's");
    Serial.println("X\tY\tZ");
  }

  /* ------------- INITIALIZING WIFI ------------- */
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    if (debugMode) {
      Serial.println("Communication with WiFi module failed!");
    }
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    if (debugMode) {
      Serial.println("Please upgrade the firmware");
    }
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    if (debugMode) {
      Serial.print("Attempting to connect to Network named: ");
      Serial.println(WIFI_SSID);                   // print the network name (SSID);
    }

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();                           // start the web server on port 80
  if (debugMode) {
    printWifiStatus();                        // you're connected now, so print out the status
  }
  digitalWrite(13, HIGH);                   // turn on LED to indicate connection
  
  /* ------------- INITIALIZING FIREBASE ------------- */
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASS);

  /* ------------- INITIALIZING POSITION ------------- */
  cur_pos = -1;
  last_pos = -1;
}

void loop() {
  float x, y, z;
  float side_delta = 0.1;
  float flat_delta = 0.3;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
//    double angle = atan(y/x);
//    Serial.println((angle * 180/3.14));
//    Serial.print(x);
//    Serial.print('\t');
//    Serial.print(y);
//    Serial.print('\t');
//    Serial.println(z);
//    float roll = atan(y / sqrt(pow(x,2) + pow(z,2))) * 180 / PI;
//    float pitch = atan(-1 * x / sqrt(pow(y, 2) + pow(z, 2))) * 180 / PI;
//    Serial.print("roll: ");
//    Serial.print(roll, 8);
//    Serial.print('\t');
//    Serial.print("pitch: ");
//    Serial.println(pitch, 8);

    /*
     * 0 = on back
     * 1 = left and up
     * 2 = left
     * 3 = left and down
     * 4 = on stomach
     * 5 = right and down
     * 6 = right
     * 7 = right and up
     */
     
    // logic to decide which orientation
    if(y <= flat_delta && y >= -flat_delta){
          if (z > 0) {
            cur_pos = 0;
            //Serial.println("on back");
          } else {
            cur_pos = 4;
            //Serial.println("on stomach");
          }
    } else if(y > side_delta && y < 1 - side_delta) {
          if (z > 0) {
            cur_pos = 1;
            //Serial.println("left and up");
          } else {
            cur_pos = 3;
            //Serial.println("left and down");
          }
    } else if(y >= 1 - side_delta) {
          cur_pos = 2;
          //Serial.println("left");
    } else if(y < -side_delta && y > side_delta - 1) {
          if (z > 0) {
            cur_pos = 7;
            //Serial.println("right and up");
          } else {
            cur_pos = 5;
            //Serial.println("right and down");
          }
    } else {
          cur_pos = 6;
          //Serial.println("right");
    }

    // compare last_pos against cur_pos, update count if different
    if (last_pos == -1) { // first time setting pos
      last_pos = cur_pos;
      if (debugMode) {
        Serial.print("start orientation\t");
        Serial.println(cur_pos);
      }
      sendPosToFirebase(cur_pos);
    } else if (cur_pos != last_pos) {
      diff_count += 1;
    }

    // check count against threshold to determine if orientation changed
    if (diff_count > change_threshold) {
      if (debugMode) {
        Serial.print("orientation changed\t");
        Serial.println(cur_pos);
      }
      //send change to firebase
      sendPosToFirebase(cur_pos);
      last_pos = cur_pos;
      diff_count = 0;
    }
    
  }
  delay(20);
}

void sendPosToFirebase(int data) {
  if (Firebase.setInt(firebaseData, "/position", data)){
    //Success, then read the payload value return from server
    //This confirmed that your data was set to database as an int
    if (firebaseData.dataType() == "int") {
      if (debugMode) {
        Serial.println(firebaseData.intData());
      }
    }
  
  } else {
    //Failed, then print out the error detail
    if (debugMode) {
      Serial.println(firebaseData.errorReason());
    }
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
