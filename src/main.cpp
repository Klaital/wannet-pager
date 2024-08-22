#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <LSM6DSOSensor.h>
#include <LSM6DSOXSensor.h>
#include <WiFiNINA.h>
#include "config.h"

// onboard sensors
float Ax, Ay, Az;
constexpr float AccelThreshhold = 1.1f;
float Gx, Gy, Gz;
float temperature;
LSM6DSOXSensor accGyr(&Wire);

// external peripherals
constexpr pin_size_t BUZZER = 3;
constexpr pin_size_t TOUCH_SENSOR = 2;

// WiFi connection
constexpr char ssid[] = WIFI_SSID;
constexpr char wifi_pass[] = WIFI_PASS;
int wifiStatus = WL_IDLE_STATUS;

// MQTT setup
#include <ArduinoMqttClient.h>
constexpr char mqttBrokerHost[] = MQTT_BROKER;
WiFiClient mqttWifi;
MqttClient mqttClient(mqttWifi);


volatile bool pagerActivated = false;
volatile unsigned long lastPagerActivation = 0;
constexpr unsigned long pagerActivationCooldown = 10000;
volatile bool imuActivated = false;
void IMU_callback();
void HandleTouch();
void printMLCStatus(uint8_t status);

void setup() {
// write your initialization code here
    Serial.begin(9600);
    // while(!Serial);

    Serial.print("connecting to wifi...");

    Serial.println(" [DONE]");

    Serial.print("initializing IMU...");
    if (!IMU.begin()) {
        Serial.println(" [FAIL]");
        while(true)
            ; // halt and catch fire
    }

    Wire.begin(); // initialize I2C bus
    attachInterrupt(digitalPinToInterrupt(INT_IMU), IMU_callback, RISING);
    accGyr.begin();
    accGyr.Enable_X();
    accGyr.Enable_G();
    // accGyr.Enable_Single_Tap_Detection(LSM6DSOX_INT1_PIN);
    // accGyr.Enable_Free_Fall_Detection(LSM6DSO_INT1_PIN);
    Serial.println(" [DONE]");


    Serial.print("Accelerometer sample rate: ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println("Hz");
    Serial.print("Gyro sample rate: ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println("Hz");
    Serial.println();

    Serial.print("initializing touch sensor...");
    pinMode(TOUCH_SENSOR, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(TOUCH_SENSOR), HandleTouch, RISING);
    Serial.println(" [DONE]");
    Serial.print("initializing buzzer...");
    pinMode(BUZZER, OUTPUT);
    noTone(BUZZER);
    Serial.println(" [DONE]");
}

void beep(const pin_size_t pin, const int freq, const unsigned long duration) {
    tone(pin, freq);
    delay(duration);
    noTone(pin);
}
void loop() {
    if (pagerActivated) {
        pagerActivated = false;
        // Check if the cooldown has expired
        const unsigned long now = millis();
        if (now - lastPagerActivation > pagerActivationCooldown) {
            lastPagerActivation = now;
            Serial.println("Pager activated!");
            beep(BUZZER, 4000, 100);
            // TODO: send a pager notice on MQTT
            delay(500);
        }
    }

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(Ax, Ay, Az);

        if (Ax > AccelThreshhold || Ay > AccelThreshhold || Az > 1.80) {
            Serial.print("Accelerometer driven pager: ");
            Serial.print(Ax);
            Serial.print("\t");
            Serial.print(Ay);
            Serial.print("\t");
            Serial.print(Az);
            Serial.println();
            pagerActivated = true;
        }
    }

}

void IMU_callback() {
    // Serial.println("IMU data available!");
    imuActivated = true;
}

void HandleTouch() {
    // Serial.println("Touch detected");
    pagerActivated = true;
}


void printMLCStatus(const uint8_t status) {
    switch(status) {
        case 0:
            Serial.println("Activity: Stationary");
        break;
        case 1:
            Serial.println("Activity: Walking");
        break;
        case 4:
            Serial.println("Activity: Jogging");
        break;
        case 8:
            Serial.println("Activity: Biking");
        break;
        case 12:
            Serial.println("Activity: Driving");
        break;
        default:
            Serial.println("Activity: Unknown");
        break;
    }
}
void enable_WiFi() {
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
        // don't continue
        while (true)
            ;
    }

    const String fv = WiFi.firmwareVersion();
    if (fv < "1.0.0") {
        Serial.println("Please upgrade the firmware");
    }

    WiFi.setHostname(HOSTNAME);
}
void connect_WiFi() {
    // attempt to connect to Wifi network:
    while (wifiStatus != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        wifiStatus = WiFi.begin(ssid, wifi_pass);
        Serial.print("WiFi Status: ");
        Serial.println(wifiStatus);
        // wait 10 seconds for connection:
        delay(5000);
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
    const long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
    Serial.print("IP: ");
    Serial.println(ip);
}
