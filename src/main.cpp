#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// NTP-related
#include <ezTime.h>

#define SAMPLE_INTERVAL 1000UL       // orientation updates every second
#define STATUS_INTERVALL 10000UL     // status updates every 10 seconds
#define CALIBRATION_INTERVAL 60000UL // calibration data every minute
#define MQTT_PORT 1883
#define DECLINATION_DEG 5.07f // Berlin
// #define DECLINATION_DEG 4.98f        // Graz
#define TIMEZONE "Etc/UTC" // or "Europe/Berlin", "Europe/Vienna" etc.

// ---------- WiFi ----------
const char *SSID = "things@37";
const char *PASSWORD = "C0nn3ct10n_l05t!";

// ---------- MQTT ----------
const char *MQTT_SERVER = "192.168.37.21";
const char *MQTT_CLIENT_ID = "rotator_positioner_dev";
const char *MQTT_ORIENTATION_TOPIC = "antenna/orientation";
const char *MQTT_STATUS_TOPIC = "antenna/status";
const char *MQTT_CALIBRATION_TOPIC = "antenna/calibration";

// ---------- Globals ----------
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Adafruit_BNO055 bno(55);

unsigned long lastSample = 0;      // orientation interval timing
unsigned long lastStatus = 0;      // status interval timing
unsigned long lastCalibration = 0; // calibration interval timing
unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;

bool wifiConnected = false;
bool mqttConnected = false;

float lastHeading = 0.0f;
long revolutions = 0;

Timezone tz;

// ---------- WiFi ----------
void ensureWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
        return;

    unsigned long now = millis();
    if (now - lastWifiAttempt > 5000)
    {
        lastWifiAttempt = now;
        WiFi.begin(SSID, PASSWORD);
    }
}

// ---------- MQTT ----------
void ensureMQTT()
{
    if (mqtt.connected())
        return;

    unsigned long now = millis();
    if (now - lastMqttAttempt > 3000)
    {
        lastMqttAttempt = now;
        mqtt.connect(MQTT_CLIENT_ID);
    }
}

// ---------- Setup ----------
void setup()
{
    Serial.begin(115200);

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);

    mqtt.setServer(MQTT_SERVER, MQTT_PORT);

    if (!bno.begin())
    {
        Serial.println("BNO055 not detected");
        while (true)
            yield();
    }

    waitForSync();
    delay(1000);
    setServer("192.168.37.1");
    setInterval(3600);
    setDebug(INFO);
    delay(1000);
    tz.setLocation(F(TIMEZONE));

    delay(1000);
    bno.setExtCrystalUse(true);
}

// ---------- Main Loop ----------
void loop()
{
    ensureWiFi();
    ensureMQTT();
    mqtt.loop();
    events();

    String timeStr = tz.dateTime();
    time_t ts = tz.now() * 1000 + tz.ms();
    unsigned long now = millis();

    if (now - lastSample >= SAMPLE_INTERVAL)
    {
        lastSample = now;

        if (!mqtt.connected())
            return;

        // ---- Quaternion ----
        imu::Quaternion q = bno.getQuat();

        float w = q.w();
        float x = q.x();
        float y = q.y();
        float z = q.z();

        // ---- Yaw (Magnetic Azimuth) ----
        float yaw = atan2(2.0f * (w * z + x * y),
                          1.0f - 2.0f * (y * y + z * z));

        float heading = yaw * 180.0f / PI;

        if (heading < 0)
            heading += 360.0f;

        // ---- True North Correction ----
        heading += DECLINATION_DEG;

        if (heading >= 360.0f)
            heading -= 360.0f;
        if (heading < 0.0f)
            heading += 360.0f;

        // ---- Continuous Rotation Tracking ----
        if (heading < lastHeading - 180.0f)
            revolutions++;
        if (heading > lastHeading + 180.0f)
            revolutions--;

        float extendedHeading = heading + revolutions * 360.0f;
        lastHeading = heading;

        // ---- Elevation from Gravity ----
        imu::Vector<3> gravity =
            bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

        float gx = gravity.x();
        float gy = gravity.y();
        float gz = gravity.z();

        float norm = sqrt(gx * gx + gy * gy + gz * gz);

        float elevation = 0.0f;

        if (norm > 0.0001f)
        {
            gz /= norm;
            elevation = asin(gz) * 180.0f / PI;
            elevation = fabs(elevation);
        }

        // ---- Publish JSON ----
        char payload[512];

        snprintf(payload, sizeof(payload),
                 "{"
                 "  \"dateTime\": \"%s\","  
                 "  \"ts\": %i," 
                 "  \"azimuth\": %.2f," 
                 "  \"elevation\": %.2f," 
                 "  \"units\": {" 
                 "    \"ts\": \"milliseconds\"," 
                 "    \"orientation\": \"decimal degrees\"" 
                "  }" 
                "}",
                 timeStr.c_str(),
                 ts,
                 extendedHeading,
                 elevation);

        mqtt.publish(MQTT_ORIENTATION_TOPIC, payload);
        Serial.println(String(payload));
    }
    else if (now - lastStatus >= STATUS_INTERVALL)
    {
        lastStatus = now;
    }
    else if (now - lastCalibration >= CALIBRATION_INTERVAL)
    {
        lastCalibration = now;
    }
}
