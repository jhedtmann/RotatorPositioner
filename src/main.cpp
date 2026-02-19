#define VERSION "2.0.1b"
#define DEV_DEBUG 1

#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// NTP-related
#include <ezTime.h>

#define SAMPLE_INTERVAL 1000UL       // orientation updates every second
#define STATUS_INTERVALL 60000UL     // status updates every 10 seconds
#define CALIBRATION_INTERVAL 10000UL // calibration data every minute
#define MQTT_PORT 1883
#define DECLINATION_DEG 5.07f // Berlin
// #define DECLINATION_DEG 4.98f        // Graz
#define TIMEZONE "Etc/UTC" // or "Europe/Berlin", "Europe/Vienna" etc.
#define USE_NMEA true

#define SDA_PIN D2
#define SCL_PIN D1

// ---------- WiFi ----------
const char *SSID = "things@37";
const char *PASSWORD = "C0nn3ct10n_l05t!";

// ---------- MQTT ----------
const char *MQTT_SERVER = "192.168.37.21";
const char *MQTT_CLIENT_ID = "rotator_positioner_dev";
const char *MQTT_ORIENTATION_TOPIC = "antenna/orientation";
const char *MQTT_STATUS_TOPIC = "antenna/status";
const char *MQTT_CALIBRATION_TOPIC = "antenna/calibration";

// ---------- NTP --------------
const char *NTP_SERVER_IP = "192.168.37.1";

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
int revolutions = 0;

Timezone tz;

// ---------- I2C -----------
void recoverI2C()
{
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);

    if (digitalRead(SDA_PIN) == LOW)
    {
        pinMode(SCL_PIN, OUTPUT);
        for (int i = 0; i < 9; i++)
        {
            digitalWrite(SCL_PIN, HIGH);
            delayMicroseconds(5);
            digitalWrite(SCL_PIN, LOW);
            delayMicroseconds(5);
        }
    }
}

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

// ----------- NMEA ----------
uint8_t calcNMEAChecksum(const char *sentence)
{
    if (*sentence == '$')
        sentence++;

    uint8_t sum = 0;

    while (*sentence && *sentence != '*')
        sum ^= (uint8_t)*sentence++;

    return sum;
}

bool appendChecksum(char *buffer, size_t bufSize)
{
    size_t len = strlen(buffer);

    // Need space for "*XX\r\n\0" = 6 bytes
    if (len + 6 >= bufSize)
        return false;

    uint8_t cs = calcNMEAChecksum(buffer);

    sprintf(buffer + len, "*%02X\r\n", cs);

    return true;
}

// ---------- Setup ----------
void setup()
{
    delay(1500); // allow sensor power stabilization
    recoverI2C();
    Wire.begin();
    Wire.setClock(400000);

    Serial.begin(115200);

    if (!bno.begin())
    {
        Serial.println("Initial begin() failed");
        delay(1000);
        bno.begin(); // second attempt after boot settles
    }

    delay(100); // allow mode switch
    bno.setExtCrystalUse(true);
    delay(10);

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);

    mqtt.setServer(MQTT_SERVER, MQTT_PORT);

    waitForSync();
    delay(1000);
    setServer(NTP_SERVER_IP);
    setInterval(3600);
    setDebug(INFO);
    delay(1000);
    tz.setLocation(F(TIMEZONE));

    delay(1000);
    bno.setExtCrystalUse(true);
    delay(1000);
}

time_t ts;
char payload[512];
char nmeaStr[83];

// ---------- Main Loop ----------
void loop()
{
#if DEV_DEBUG
    Serial.printf("Heap: %u\n", ESP.getFreeHeap());
#endif

    ensureWiFi();
    ensureMQTT();
    mqtt.loop();
    events();

    ts = tz.now() * 1000 + tz.ms();
    unsigned long now = millis();

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
        // elevation = asin(gz) * 180.0f / PI;

        elevation = 90.0f - (asin(gz) * 180.0f / PI);
        // elevation = fabs(elevation);
    }

    uint8_t system, gyro, accelerometer, magnetometer;
    system = 0;
    gyro = 0;
    accelerometer = 0;
    magnetometer = 0;
    bno.getCalibration(&system, &gyro, &accelerometer, &magnetometer);

    if (now - lastSample >= SAMPLE_INTERVAL)
    {
        lastSample = now;

        // ---- Publish JSON ----
        snprintf(payload, sizeof(payload),
                 "{"
                 "  \"dateTime\": \"%s\","
                 "  \"ts\": \"%lld\","
                 "  \"azimuth\": %.2f,"
                 "  \"elevation\": %.2f,"
                 "  \"revolutions\": %i,"
                 "  \"units\": {"
                 "    \"ts\": \"milliseconds\","
                 "    \"orientation\": \"decimal degrees\""
                 "  }"
                 "}",
                 tz.dateTime().c_str(),
                 ts,
                 extendedHeading,
                 elevation,
                 revolutions);

        if (mqtt.connected())
            mqtt.publish(MQTT_ORIENTATION_TOPIC, payload);

        // $PATRK,1771493857399,ms,47.33,dd,19.13,dd,-1,revs,0,3,1,0*<HH>\r\n
        snprintf(nmeaStr,
                 sizeof(nmeaStr),
                 "$PANT,TRK,%lld,ms,%.2f,dd,%.2f,dd,%i,revs,%i,%i,%i,%i",
                 ts,
                 extendedHeading,
                 elevation,
                 revolutions,
                 system,
                 gyro,
                 accelerometer,
                 magnetometer);
        appendChecksum(nmeaStr, sizeof(nmeaStr));
        if (USE_NMEA)
        {
            Serial.print(nmeaStr);
        }
        else
        {
            Serial.println(String(payload));
        }
    }
    else if (now - lastStatus >= STATUS_INTERVALL)
    {
        lastStatus = now;

        // ---- Publish JSON ----
        snprintf(payload, sizeof(payload),
                 "{"
                 "  \"dateTime\": \"%s\","
                 "  \"ts\": \"%lld\","
                 "  \"version\": \"%s\""
                 "  \"bssid\": \"%s\""
                 "}",
                 tz.dateTime().c_str(),
                 ts,
                 VERSION,
                 WiFi.BSSIDstr().c_str());

        if (mqtt.connected())
            mqtt.publish(MQTT_STATUS_TOPIC, payload);

        if (!USE_NMEA)
            Serial.println(String(payload));
    }
    else if (now - lastCalibration >= CALIBRATION_INTERVAL)
    {
        lastCalibration = now;

        // ---- Publish JSON ----
        snprintf(payload, sizeof(payload),
                 "{"
                 "  \"dateTime\": \"%s\","
                 "  \"ts\": \"%lld\","
                 "  \"system\": %i,"
                 "  \"gyro\": %i,"
                 "  \"accelerometer\": %i,"
                 "  \"magnetometer\": %i"
                 "}",
                 tz.dateTime().c_str(),
                 ts,
                 system,
                 gyro,
                 accelerometer,
                 magnetometer);

        if (mqtt.connected())
            mqtt.publish(MQTT_CALIBRATION_TOPIC, payload);

        if (!USE_NMEA)
            Serial.println(String(payload));
    }
}
