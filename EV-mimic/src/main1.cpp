#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Configuration Constants
#define MEASUREMENT_INTERVAL 1000 // ms
#define BATTERY_CAPACITY 2600.0   // mAh
#define TOTAL_CYCLES 1000.0       // Estimated cycles at 80% capacity
#define SOC_LOW_THRESHOLD 20      // SOC threshold for cycle counting
#define SOC_HIGH_THRESHOLD 90     // SOC threshold for cycle counting

// Pin Definitions
#define RELAY_PIN D3
#define ONE_WIRE_BUS D0

// Battery voltage to SOC lookup table (adjust these values for your specific battery)
const float voltageSocTable[][2] = {
    {3.00, 0},  // 0% at 3.00V
    {3.30, 5},  // 5% at 3.30V
    {3.50, 10}, // 10% at 3.50V
    {3.60, 20}, // 20% at 3.60V
    {3.70, 30}, // 30% at 3.70V
    {3.75, 40}, // 40% at 3.75V
    {3.80, 50}, // 50% at 3.80V
    {3.85, 60}, // 60% at 3.85Vread
    {3.90, 70}, // 70% at 3.90V
    {3.95, 80}, // 80% at 3.95V
    {4.00, 90}, // 90% at 4.00V
    {4.10, 95}, // 95% at 4.10V
    {4.20, 100} // 100% at 4.20V
};
const int voltageSocTableSize = sizeof(voltageSocTable) / sizeof(voltageSocTable[0]);

enum VehicleType
{
    PORSCHE = 0x01,
    TESLA = 0x02,
    AUDI = 0x03,
    BMW = 0x04,
    // Add more as needed
};

uint8_t vehicle_type_code = PORSCHE; // Set during initialization

// Hardware Configuration
MCP2515 can(D8); // CS on D8 (GPIO15)
Adafruit_INA219 ina219_ev_battery(0x40);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// WiFi Credentials
const char *ssid = "OPTIMUS";
const char *password = "qqwweeaaaa";

// Web Server
ESP8266WebServer server(80);

// Battery State Variables
struct BatteryState
{
    float voltage;
    float current;
    float temperature;
    float soc;
    float soh;
    float remainingCapacity;
    float cycleCount;
    bool charging;
    bool commsActive;
    unsigned long lastCommTime;
};

BatteryState battery = {
    .voltage = 0,
    .current = 0,
    .temperature = 0,
    .soc = 100,
    .soh = 100,
    .remainingCapacity = BATTERY_CAPACITY,
    .cycleCount = 0,
    .charging = false,
    .commsActive = false,
    .lastCommTime = 0};

// Function Prototypes
void initializeHardware();
void connectToWiFi();
void setupWebServer();
float readTemperature();
void measureBatteryParameters();
float calculateSoC();
float calculateSoH();
void updateCycleCount();
void handleRoot();
void handleData();
void sendEVStatus();
void checkChargerCommands();
uint8_t calculateChecksum(uint8_t *data);
void updateRelayState();
void checkCommunicationTimeout();


    float calculateSoC()
{
    float measuredVoltage = battery.voltage;

    // Handle voltage below minimum
    if (measuredVoltage <= voltageSocTable[0][0])
    {
        return voltageSocTable[0][1];
    }

    // Handle voltage above maximum
    if (measuredVoltage >= voltageSocTable[voltageSocTableSize - 1][0])
    {
        return voltageSocTable[voltageSocTableSize - 1][1];
    }

    // Find the interval where the voltage lies
    for (int i = 0; i < voltageSocTableSize - 1; i++)
    {
        if (measuredVoltage >= voltageSocTable[i][0] && measuredVoltage < voltageSocTable[i + 1][0])
        {
            // Linear interpolation between the two points
            float x0 = voltageSocTable[i][0];
            float y0 = voltageSocTable[i][1];
            float x1 = voltageSocTable[i + 1][0];
            float y1 = voltageSocTable[i + 1][1];

            return y0 + (measuredVoltage - x0) * ((y1 - y0) / (x1 - x0));
        }
    }

    // Default return (shouldn't reach here)
    return 50.0;
}

void measureBatteryParameters()
{
    // Read voltage and current
    battery.voltage = ina219_ev_battery.getBusVoltage_V() +
                      (ina219_ev_battery.getShuntVoltage_mV() / 1000);
    battery.current = ina219_ev_battery.getCurrent_mA();

    // Read temperature
    battery.temperature = readTemperature();

    // Update SOC and SOH
    battery.soc = calculateSoC();
    updateCycleCount();
    battery.soh = calculateSoH();

    // Update remaining capacity based on SOC
    battery.remainingCapacity = (battery.soc / 100.0) * BATTERY_CAPACITY;

    // Diagnostic output
    Serial.printf("V: %.2fV, I: %.2fmA, Temp: %.1f°C, SOC: %.1f%%, SOH: %.1f%%\n",
                  battery.voltage, battery.current, battery.temperature, battery.soc, battery.soh);
}

void initializeHardware()
{
    // Initialize I2C and SPI
    Wire.begin();
    SPI.begin();
    can.reset();
    can.setBitrate(CAN_125KBPS, MCP_8MHZ);
    can.setNormalMode();

    // Initialize DS18B20
    ds18b20.begin();

    // Initialize INA219
    if (!ina219_ev_battery.begin())
    {
        Serial.println("Failed to initialize INA219");
        while (1)
            delay(10);
    }

    // Initialize relay pin
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    Serial.println("Hardware initialization complete");
}

float readTemperature()
{
    ds18b20.requestTemperatures();
    float tempC = ds18b20.getTempCByIndex(0);

    // Check if reading was successful
    if (tempC == DEVICE_DISCONNECTED_C)
    {
        Serial.println("Error reading DS18B20 temperature");
        return battery.temperature; // Return last known value
    }

    return tempC;
}

void updateRelayState()
{
    // Only turn on relay if charging is enabled AND communication is active
    bool relayState = battery.charging && battery.commsActive;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);

    // if (relayState != digitalRead(RELAY_PIN))
    // {
    //     Serial.printf("Relay %s\n", relayState ? "ON" : "OFF");
    // }
}

void checkCommunicationTimeout()
{
    const unsigned long COMM_TIMEOUT = 2500; // 5 seconds timeout

    if (battery.commsActive && (millis() - battery.lastCommTime > COMM_TIMEOUT))
    {
        battery.commsActive = false;
        Serial.println("Communication timeout - disabling relay");
        updateRelayState();
    }
}

void sendEVStatus()
{
    can_frame msg;
    msg.can_id = 0x100;
    msg.can_dlc = 8;

    // Pack measurements
    uint16_t voltage = battery.voltage * 100;
    uint16_t current = abs(battery.current);

    msg.data[0] = voltage >> 8;
    msg.data[1] = voltage & 0xFF;
    msg.data[2] = current >> 8;
    msg.data[3] = current & 0xFF;
    msg.data[4] = (uint8_t)constrain(battery.soc, 0, 100);
    msg.data[5] = (uint8_t)constrain(battery.soh, 0, 100);
    msg.data[6] = (uint8_t)constrain(round(battery.temperature), -128, 127);

    msg.data[7] = (vehicle_type_code << 2) | (battery.charging ? 0x02 : 0x00) | 0x01;
    msg.data[7] ^= calculateChecksum(msg.data);

    can.sendMessage(&msg);
}

void checkChargerCommands()
{
    if (can.checkReceive())
    {
        can_frame rxMsg;
        if (can.readMessage(&rxMsg) == MCP2515::ERROR_OK)
        {
            if (rxMsg.can_id == 0x101 && rxMsg.can_dlc == 2)
            {
                if (rxMsg.data[1] == (rxMsg.data[0] ^ 0xFF))
                {
                    battery.charging = (rxMsg.data[0] == 0x01);
                    Serial.println(battery.charging ? "Charging STARTED" : "Charging STOPPED");
                    updateRelayState();

                    // digitalWrite(RELAY_PIN,battery.charging);
                }
            }

            // Check for ACK of our status message
            if (rxMsg.can_id == 0x102)
            {   

                battery.lastCommTime = millis();
                if (!battery.commsActive)
                {
                    battery.commsActive = true;
                    Serial.println("Received ACK - communication active");
                    updateRelayState();
                }
            }

            
        }
    }
}

void connectToWiFi()
{
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}



void setupWebServer()
{
  server.enableCORS(true);
  server.on("/", handleRoot);
  server.on("/data", handleData);

  server.begin();
  Serial.println("HTTP server started");
}

uint8_t calculateChecksum(uint8_t *data)
{
  uint8_t checksum = 0;
  for (int i = 0; i < 7; i++)
  {
    checksum ^= data[i];
  }
  return checksum;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\nStarting EV Battery Monitor");

    initializeHardware();
    connectToWiFi();
    setupWebServer();

    Serial.println("System initialization complete");
}

void loop()
{
    server.handleClient();

    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate >= MEASUREMENT_INTERVAL)
    {
        measureBatteryParameters();
        sendEVStatus();
        lastUpdate = millis();
    }

    checkChargerCommands();
    checkCommunicationTimeout();
}



float calculateSoH()
{
    // Simple linear degradation model
    float degradation = min(battery.cycleCount / TOTAL_CYCLES, 1.0);
    return (1.0 - degradation) * 100;
}

void updateCycleCount()
{
    static bool wasBelowThreshold = false;

    if (battery.soc < SOC_LOW_THRESHOLD && !wasBelowThreshold)
    {
        wasBelowThreshold = true;
    }
    else if (battery.soc > SOC_HIGH_THRESHOLD && wasBelowThreshold)
    {
        wasBelowThreshold = false;
        battery.cycleCount += 0.5; // Half cycle (discharge + charge = full cycle)
        Serial.printf("Cycle count updated: %.1f\n", battery.cycleCount);
    }
}

void handleRoot()
{
    String html = "Hey, I am " + (String)vehicle_type_code + "....... Wanna Ride it!";
    server.send(200, "text/html", html);
}

void handleData()
{
    String json = "{";
    json += "\"vehicle_name\":\"" + String(vehicle_type_code) + "\",";
    json += "\"battery_voltage\":" + String(battery.voltage, 2) + ",";
    json += "\"battery_current\":" + String(battery.current, 2) + ",";
    json += "\"soc\":" + String(battery.soc, 1) + ",";
    json += "\"soh\":" + String(battery.soh, 1) + ",";
    json += "\"temperature\":" + String(battery.temperature, 1) + ",";
    json += "\"charging\":" + String(battery.charging ? "true" : "false");
    json += "}";

    server.send(200, "application/json", json);
}
