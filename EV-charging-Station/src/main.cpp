#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_INA219.h>
#include <MFRC522.h>
#include <SPI.h>
#include <EEPROM.h>
#include <mcp2515.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Update.h>
#include <ESPmDNS.h>

// Hardware Definitions (same as before)
#define DAC_GPIO 25
#define WIRELESS_CHARGE_PIN 26
#define WIRED_CHARGE_PIN 27
#define BUCK_PIN 4
#define CHARGER_PIN 2
#define SS_PIN 14
#define RST_PIN 17

#define NTC_NOMINAL_RESISTANCE 10000
#define NTC_NOMINAL_TEMP 25.0
#define NTC_B_VALUE 3950
#define SERIES_RESISTOR 10000
#define EEPROM_SIZE 512
#define ENERGY_EEPROM_ADDR 300 // Use an address after your user data
#define ENERGY_PRICE_ADDR 200 // Use an address after your user data
#define MAX_USERS 4
#define CAN_TIMEOUT 1800

// User storage configuration
#define USER_STORAGE_SIZE 32 // Bytes per user
#define MAX_UID_LENGTH 14    // 7 bytes UID in HEX (2 chars per byte) + null terminator
#define OTA_USER "admin"
#define OTA_PASS "admin123"
#define VERSION "1.3"

// FreeRTOS Handles
TaskHandle_t xWebServerTask = NULL;
TaskHandle_t xCANHandlerTask = NULL;
TaskHandle_t xRFIDTask = NULL;
TaskHandle_t xSensorTask = NULL;
QueueHandle_t xCANQueue;
QueueHandle_t xRFIDQueue;
SemaphoreHandle_t xI2CSemaphore;
SemaphoreHandle_t xSPISemaphore;
static SemaphoreHandle_t xSOCMutex = NULL;
SemaphoreHandle_t xSensorDataMutex = NULL;
SemaphoreHandle_t xEEPROMMutex = NULL;
SemaphoreHandle_t xUserMutex = NULL;

// Global Objects 
MCP2515 can(5);
MFRC522 mfrc522(SS_PIN, RST_PIN);
WebServer server(80);
Adafruit_INA219 ina219_lm2596(0x40);
Adafruit_INA219 ina219_battery(0x41);

// ///////////////////////       Global Variables       /////////////////////

// WiFi Credentials
const char *ssid = "OPTIMUS";
const char *password = "qqwweeaaaa";

struct CANMessage
{
    uint16_t voltage;     // 0.1V resolution (0-6553.5V range)
    uint16_t current;     // 0.1A resolution (0-6553.5A range)
    uint8_t soc;          // 0-100%
    uint8_t soh;          // 0-100%
    int8_t evtemperature; // -128°C to +127°C
    uint8_t status;       // Bitmask (0x01=connected, 0x02=charging, 0x04=error)
    uint8_t vehicle_id;   // Encoded vehicle identifier (0-255 possible vehicles)
    uint8_t checksum;     // XOR of all previous bytes
};

struct User
{
    char uid[MAX_UID_LENGTH + 1]; // Fixed size buffer for HEX UID
    float balance;
};

// Shared variables
User currentAuthUser;
bool deleteUser = false;
unsigned long lastCANMessageTime = 0;

// NTC Thermistor
const int ntcPin = 34;
// float temperature;

// DAC Control (GPIO 25)
int dacValue = 128; // Default midpoint (0-255)

// CAN Variables
bool evConnected = false;
float evVoltage = 0;
float evCurrent = 0;
uint8_t evSOC = 0;
uint8_t evSOH = 0;
uint8_t ev_Temperature = 0;
String evVehicleName;
bool evCharging;
bool chargingStatus = false;
bool otaInProgress = false;

// Global variables
float g_lm2596_voltage = 0;
float g_lm2596_current = 0;
float g_lm2596_power = 0;
float g_battery_voltage = 0;
float g_battery_current = 0;
float g_battery_power = 0;
float g_temperature = 0;

// Pricing
bool chargeRateRead = false;
float chargingRate = 15; // Price per kWh (adjust as needed)
float batterySOH = 100.0;
// unsigned long lastSOHUpdate = 0;
float energyDelivered = 0; // Track kWh delivered
bool isCharging = false;
unsigned long chargeStartTime = 0;
float startEnergy = 0;   // kWh at session start
float sessionEnergy = 0; // kWh delivered in current session

// Battery configuration (adjust for your specific battery)
#define BATTERY_CELLS 3        // Number of series cells
#define MIN_VOLTAGE 3.0f        // Fully discharged voltage per cell
#define MAX_VOLTAGE 4.2f        // Fully charged voltage per cell
#define NOMINAL_VOLTAGE 3.7f    // Nominal voltage per cell
#define TEMP_COEFFICIENT 0.005f // Voltage compensation per °C from 25°C

//////////////////////////////////////////////////////////////////////////

// Task Priorities
#define WEBSERVER_PRIORITY 1
#define CAN_HANDLER_PRIORITY 2
#define RFID_PRIORITY 2
#define SENSOR_PRIORITY 3

// Task Stack Sizes
#define WEBSERVER_STACK_SIZE 8192
#define CAN_HANDLER_STACK_SIZE 4096
#define RFID_STACK_SIZE 4096
#define SENSOR_STACK_SIZE 4096

//////////////////////////////////////////////////////////////////////////

// Voltage-SOC lookup table (Li-ion example, adjust for your battery)
typedef struct
{
    float voltage;
    float soc;
} VoltageSocPair;

static const VoltageSocPair voltageSocTable[] = {
    {3.0f, 0.0f}, // Fully discharged
    {3.3f, 10.0f},
    {3.6f, 30.0f},
    {3.8f, 50.0f},
    {3.9f, 70.0f},
    {4.1f, 90.0f},
    {4.2f, 100.0f} // Fully charged
};
static const size_t voltageSocTableSize = sizeof(voltageSocTable) / sizeof(VoltageSocPair);

///////  Function definition used    //////////////////////////////

void vWebServerTask(void *pvParameters);
void vCANHandlerTask(void *pvParameters);
void vRFIDTask(void *pvParameters);
void vSensorTask(void *pvParameters);

// Endpoints function
void handleRoot();
void handleData();
void handleSetDAC();
void handleChipEnable();
void handleAddCard();
void handleListUsers();
void handleTopUp();
void handleDeleteUser();
void handleEVData();
// void handleCharge();
void handleReboot();
void handleSetRate();
void handleAuthCard();
void handleStartCharging();
void handleEndCharging();
void handleUpdate();
void handleUpdatePage();

void initSOCEstimator();
float estimateSOC(float voltage, float temperature);
float getEstimatedSOC(float temperature);
float readNTC();
void updateSOH(float currentVoltage, float temperature);
bool saveUser(int index, const User &u);
User readUser(int index);
int findUserIndex(const char *uid);
void sendCANCommand(uint8_t cmd);
void updateEnergyMeasurement();
void saveEnergyData();

    ///////  Function definition end    //////////////////////////////

    // Setup Function (modified for FreeRTOS)
void setup()
{
    Serial.begin(115200);

    // Initialize hardware
    pinMode(WIRED_CHARGE_PIN, OUTPUT);
    digitalWrite(WIRED_CHARGE_PIN, LOW);
    pinMode(WIRELESS_CHARGE_PIN, OUTPUT);
    digitalWrite(WIRELESS_CHARGE_PIN, LOW);
    pinMode(BUCK_PIN, OUTPUT);
    digitalWrite(BUCK_PIN, HIGH); //Active LOW
    pinMode(CHARGER_PIN, OUTPUT);
    digitalWrite(CHARGER_PIN, LOW);    

    // Initialize communication interfaces
    Wire.begin();
    SPI.begin();
    if (!ina219_lm2596.begin())
    {
        Serial.println("Failed to initialize INA219 LM2596");
        while (1)
            delay(100);
    }
    if (!ina219_battery.begin())
    {
        Serial.println("Failed to initialize INA219 Battery");
        while (1)
            delay(100);
    }

    // Initialize FreeRTOS objects
    xSensorDataMutex = xSemaphoreCreateMutex();
    xCANQueue = xQueueCreate(10, sizeof(CANMessage));
    // xRFIDQueue = xQueueCreate(5, sizeof(String));
    xRFIDQueue = xQueueCreate(5, sizeof(User));
    xI2CSemaphore = xSemaphoreCreateMutex();
    xSPISemaphore = xSemaphoreCreateMutex();
    xEEPROMMutex = xSemaphoreCreateMutex();
    xUserMutex = xSemaphoreCreateMutex();
    xRFIDQueue = xQueueCreate(5, sizeof(char[16])); // Queue for UID strings

    // In setup(), after creating mutexes/queues:
    if (xRFIDQueue == NULL || xI2CSemaphore == NULL || xSPISemaphore == NULL ||
        xEEPROMMutex == NULL || xUserMutex == NULL || xSensorDataMutex == NULL)
    {
        Serial.println("Failed to create FreeRTOS objects");
        while (1)
            delay(1000);
    }

    // Initialize EEPROM
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        Serial.println("Failed to initialize EEPROM");
        while (1)
            delay(100);
    }

    // Connect to WiFi
    WiFi.begin(ssid, password);
    WiFi.setAutoReconnect(true);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Set up server endpoints (same as before)
    server.enableCORS(true);
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/setdac", HTTP_GET, handleSetDAC);
    server.on("/setChipStatus", HTTP_POST, handleChipEnable);
    server.on("/addcard", HTTP_POST, handleAddCard);
    server.on("/listusers", handleListUsers);
    server.on("/topup", HTTP_POST, handleTopUp);
    server.on("/deleteuser", HTTP_POST, handleDeleteUser);
    server.on("/evdata", handleEVData);
    // server.on("/charge", handleCharge);
    server.on("/reboot", HTTP_POST, handleReboot);
    server.on("/setrate", handleSetRate);
    server.on("/authCard", HTTP_POST, handleAuthCard);
    server.on("/startCharging", HTTP_POST, handleStartCharging);
    server.on("/endCharging", HTTP_POST, handleEndCharging);
    server.on("/update", HTTP_GET, handleUpdatePage);
    server.on("/update", HTTP_POST, []()
              { server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK"); }, handleUpdate);

    // Enable mDNS 
    if (!MDNS.begin("ampiq"))
    {
        Serial.println("Error setting up MDNS responder!");
    }
    MDNS.addService("http", "tcp", 80);
    // ... all other endpoint handlers ...

    // Create tasks
    xTaskCreatePinnedToCore(
        vWebServerTask,
        "WebServer",
        WEBSERVER_STACK_SIZE,
        NULL,
        WEBSERVER_PRIORITY,
        &xWebServerTask,
        0); // Run on core 0

    xTaskCreatePinnedToCore(
        vCANHandlerTask,
        "CANHandler",
        CAN_HANDLER_STACK_SIZE,
        NULL,
        CAN_HANDLER_PRIORITY,
        &xCANHandlerTask,
        1); // Run on core 1

    xTaskCreatePinnedToCore(
        vRFIDTask,
        "RFID",
        RFID_STACK_SIZE,
        NULL,
        RFID_PRIORITY,
        &xRFIDTask,
        1); // Run on core 1

    xTaskCreatePinnedToCore(
        vSensorTask,
        "Sensors",
        SENSOR_STACK_SIZE,
        NULL,
        SENSOR_PRIORITY,
        &xSensorTask,
        1); // Run on core 1

    Serial.println("FreeRTOS tasks started");
}

// Loop function is empty since everything is handled by tasks
void loop()
{
    // Empty - all work is done in tasks
    vTaskDelete(NULL); // Delete the loop task
}

///////////////////////////////////////////////

///////////////////////////////////////////////

// Web Server Task
void vWebServerTask(void *pvParameters)
{
    server.begin();

    for (;;)
    {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield to other tasks
    }
}

// CAN task handler
void vCANHandlerTask(void *pvParameters)
{
    can.reset();
    can.setBitrate(CAN_125KBPS, MCP_8MHZ);
    can.setNormalMode();

 
    for (;;)
    {
        if (can.checkReceive())
        {
            // Serial.println("Message received on CAN Bus");
            can_frame rxMsg;
            if (can.readMessage(&rxMsg) == MCP2515::ERROR_OK)
            {
                // Serial.println("Reading msg on CAN Bus");

                if (rxMsg.can_id == 0x100)
                {
                    if (rxMsg.can_dlc < 8)
                    {
                        Serial.println("CAN frame too short!");
                        continue;
                    }

                    // Calculate checksum from bytes 0–6
                    uint8_t calculated_checksum = 0;
                    for (int i = 0; i < 7; i++)
                    {
                        calculated_checksum ^= rxMsg.data[i];
                    }

                    // Remove checksum from byte 7 to get original value
                    uint8_t original_byte7 = rxMsg.data[7] ^ calculated_checksum;

                    // Extract status bits
                    uint8_t vehicle_id = (original_byte7 >> 2) & 0x1F; // Bits [6:2]
                    bool charging = original_byte7 & 0x02;             // Bit 1
                    bool connected = original_byte7 & 0x01;            // Bit 0 (optional)

                    // Extract values
                    float voltage = (rxMsg.data[0] << 8 | rxMsg.data[1]) / 100.0f;  // Divide by 100 to match sender
                    float current = (rxMsg.data[2] << 8 | rxMsg.data[3]) / 1000.0f; // Convert to Amps
                    uint8_t soc = rxMsg.data[4];
                    uint8_t soh = rxMsg.data[5];
                    int8_t evtemperature = rxMsg.data[6];

                    // Convert vehicle ID to name
                    String vehicle_name;
                    switch (vehicle_id)
                    {
                    case 0x01:
                        vehicle_name = "Porsche";
                        break;
                    case 0x02:
                        vehicle_name = "Tesla";
                        break;
                    // Add more cases as needed
                    default:
                        vehicle_name = "Unknown";
                        break;
                    }

                    // Update global/shared variables
                    evConnected = connected;
                    evVoltage = voltage;
                    evCurrent = current;
                    evSOC = soc;
                    evSOH = soh;
                    ev_Temperature = evtemperature;
                    evVehicleName = vehicle_name;
                    evCharging = charging;

                    // Debug output
    
                    // Serial.println("---- CAN Message Decoded ----");
                    // Serial.print("Vehicle: ");
                    // Serial.println(vehicle_name);
                    // Serial.print("Voltage: ");
                    // Serial.print(voltage);
                    // Serial.println(" V");
                    // Serial.print("Current: ");
                    // Serial.print(current);
                    // Serial.println(" A");
                    // Serial.print("SOC: ");
                    // Serial.print(soc);
                    // Serial.println(" %");
                    // Serial.print("SOH: ");
                    // Serial.print(soh);
                    // Serial.println(" %");
                    // Serial.print("Temperature: ");
                    // Serial.print(evtemperature);
                    // Serial.println(" °C");
                    // Serial.print("Charging: ");
                    // Serial.println(charging ? "Yes" : "No");
                    // Serial.print("Connected: ");
                    // Serial.println(connected ? "Yes" : "No");
                    // Serial.println("-----------------------------");
                    // --- Update timestamp and connection status ---
                    lastCANMessageTime = millis();
                }
                else if (rxMsg.can_id == 0x102)
                {
                    lastCANMessageTime = millis();
                }
            }
            else
            {
                Serial.println("Error reading CAN message");
                return;
            }
        }


        // Check for timeout
        if (evConnected && (millis() - lastCANMessageTime > CAN_TIMEOUT))
        {
            evConnected = false;
            handleEndCharging();
            memset(currentAuthUser.uid, 0, sizeof(currentAuthUser.uid));
            currentAuthUser.balance = 0.00;
        }

        // Send periodic ACK (if needed by sender)
        static unsigned long lastAckTime = 0;
        if (millis() - lastAckTime > 1000)
        {
            can_frame ackMsg;
            ackMsg.can_id = 0x102;
            ackMsg.can_dlc = 1;
            ackMsg.data[0] = 0xAA; // Simple ACK pattern
            can.sendMessage(&ackMsg);
            lastAckTime = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// RFID Task (optimized)
void vRFIDTask(void *pvParameters)
{
    mfrc522.PCD_Init();
    User tag;

    for (;;)
    {
        if (xSemaphoreTake(xSPISemaphore, portMAX_DELAY) == pdTRUE)
        {
            // Only check for new cards, don't read them here
            if (mfrc522.PICC_IsNewCardPresent())
            {
                vTaskDelay(pdMS_TO_TICKS(50)); // Small delay for card stabilization
                if (mfrc522.PICC_ReadCardSerial())
                {
                    memset(tag.uid, 0, sizeof(tag.uid));
                    for (byte i = 0; i < mfrc522.uid.size && (i * 2) < MAX_UID_LENGTH; i++)
                    {
                        sprintf(tag.uid + i * 2, "%02X", mfrc522.uid.uidByte[i]);
                    }
                    tag.balance = 0;
                    xQueueSend(xRFIDQueue, &tag, portMAX_DELAY);
                    mfrc522.PICC_HaltA();
                }
            }
            xSemaphoreGive(xSPISemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Sensor Reading Task
void vSensorTask(void *pvParameters)
{
    initSOCEstimator();
    for (;;)
    {
        if (xSemaphoreTake(xI2CSemaphore, portMAX_DELAY) == pdTRUE)
        {
            float lm2596_voltage = ina219_lm2596.getBusVoltage_V();
            float lm2596_current = ina219_lm2596.getCurrent_mA();
            float lm2596_power = ina219_lm2596.getPower_mW();

            float battery_voltage = ina219_battery.getBusVoltage_V();
            float battery_current = ina219_battery.getCurrent_mA();
            float battery_power = ina219_battery.getPower_mW();

            xSemaphoreGive(xI2CSemaphore);

            // Update global variables with protection
            if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE)
            {
                g_lm2596_voltage = lm2596_voltage;
                g_lm2596_current = lm2596_current;
                g_lm2596_power = lm2596_power;

                g_battery_voltage = battery_voltage;
                g_battery_current = battery_current;
                g_battery_power = battery_power;

                g_temperature = readNTC();

                xSemaphoreGive(xSensorDataMutex);
            }

            updateEnergyMeasurement(); 
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

///////////////////////////////////////////////

void handleRoot()
{
    String html = "<!DOCTYPE html><html><head><title>AmpIQ Charging Station</title></head>";
    html += "<body><h1>Welcome to AmpIQ Charging Station</h1>";
    html += "<p>IP Address: " + WiFi.localIP().toString() + "</p>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}
void handleData()
{
    // Local copies of sensor data
    float lm2596_voltage, lm2596_current, lm2596_power, battery_voltage, battery_current, battery_power, temperature;

    // Get protected data
    if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE)
    {
        lm2596_voltage = g_lm2596_voltage;
        lm2596_current = g_lm2596_current;
        lm2596_power = g_lm2596_power;

        battery_voltage = g_battery_voltage;
        battery_current = g_battery_current;
        battery_power = g_battery_power;

        temperature = g_temperature;
        xSemaphoreGive(xSensorDataMutex);
    }

    // Rest of your function remains the same
    float soc = estimateSOC(battery_voltage, temperature);
    updateSOH(battery_voltage, temperature);

    if (isnan(energyDelivered))
        energyDelivered = 0.0f;
    float calculatedCost = energyDelivered * chargingRate;
    if (isnan(calculatedCost))
        calculatedCost = 0.0f;

    if (!chargeRateRead)
    {
        EEPROM.get(ENERGY_PRICE_ADDR, chargingRate); // Read from EEPROM
        if (isnan(chargingRate))
        {
            chargingRate = 0.15;
        }

        EEPROM.get(ENERGY_EEPROM_ADDR, energyDelivered);
        if (isnan(energyDelivered))
        {
            energyDelivered = 0.0f;
            EEPROM.put(ENERGY_EEPROM_ADDR, energyDelivered);
            EEPROM.commit();
        }

        chargeRateRead = true;
    }

    // Create JSON response
    String json = "{";
    json += "\"lm2596_voltage\":" + String(lm2596_voltage, 2);
    json += ",\"lm2596_current\":" + String(lm2596_current, 2);
    json += ",\"lm2596_power\":" + String(lm2596_power, 2);
    json += ",\"battery_voltage\":" + String(battery_voltage, 2);
    json += ",\"battery_current\":" + String(battery_current, 2);
    json += ",\"battery_power\":" + String(battery_power, 2);
    json += ",\"soc\":" + String(soc, 1);
    json += ",\"temp\":" + String(temperature, 1);
    json += ",\"soh\":" + String(batterySOH, 1);
    json += ",\"total_energy\":" + String(energyDelivered, 3);
    json += ",\"rate\":" + String(chargingRate, 2);
    json += ",\"version\":" + String(VERSION);
    json += "}"; // No trailing comma!

    server.send(200, "application/json", json);
}

void handleSetDAC()
{
    if (server.hasArg("value"))
    {
        dacValue = server.arg("value").toInt();
        dacWrite(DAC_GPIO, dacValue);
        Serial.print("DAC set to ");
        Serial.println(dacValue);
        server.send(200, "text/plain", "DAC set to " + String(dacValue));
    }
    else
    {
        server.send(400, "text/plain", "Missing 'value' parameter");
    }
}

void handleChipEnable()
{
    bool lm2596Processed = false;
    bool cn3303Processed = false;
    String response = "";

    // Process LM2596 if argument exists
    if (server.hasArg("lm2596"))
    {
        String buck_converter_state = server.arg("lm2596");

        if (buck_converter_state == "on")
        {
            digitalWrite(BUCK_PIN, LOW); // active low
            lm2596Processed = true;
            response += "LM2596: HIGH";
        }
        else if (buck_converter_state == "off")
        {
            digitalWrite(BUCK_PIN, HIGH);
            lm2596Processed = true;
            response += "LM2596: LOW";
        }
        else
        {
            response += "LM2596: Invalid state";
        }
    }

    // Process CN3303 if argument exists
    if (server.hasArg("cn3303"))
    {
        if (!response.isEmpty())
            response += ", ";

        String charger_state = server.arg("cn3303");

        if (charger_state == "on")
        {
            digitalWrite(CHARGER_PIN, HIGH);
            cn3303Processed = true;
            response += "CN3303: HIGH";
        }
        else if (charger_state == "off")
        {
            digitalWrite(CHARGER_PIN, LOW);
            cn3303Processed = true;
            response += "CN3303: LOW";
        }
        else
        {
            response += "CN3303: Invalid state";
        }
    }

    // Determine the appropriate response
    if (lm2596Processed || cn3303Processed)
    {
        server.send(200, "text/plain", response);
        Serial.println(response);
    }
    else
    {
        server.send(400, "text/plain", "Invalid parameters - requires 'lm2596' or 'cn3303'");
    }
}

void handleAddCard()
{
    if (server.method() != HTTP_POST)
    {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    User detectedCard;
    memset(&detectedCard, 0, sizeof(detectedCard));

    // Wait for card from queue (1 second timeout)
    if (xQueueReceive(xRFIDQueue, &detectedCard, pdMS_TO_TICKS(5000)) == pdTRUE)
    {
        // Check if exists
        for (int i = 0; i < MAX_USERS; i++)
        {
            User existing = readUser(i);
            if (strcmp(existing.uid, detectedCard.uid) == 0)
            {
                char response[100];
                snprintf(response, sizeof(response),
                         "User exists! UID: %s\nBalance: $%.2f",
                         existing.uid, existing.balance);
                server.send(200, "text/plain", response);
                return;
            }
        }

        // Add new user
        for (int i = 0; i < MAX_USERS; i++)
        {
            User u = readUser(i);
            if (strlen(u.uid) == 0)
            {
                strncpy(u.uid, detectedCard.uid, MAX_UID_LENGTH);
                u.balance = 10.0;
                saveUser(i, u);
                server.send(200, "text/plain", "User added!\nUID: " + String(detectedCard.uid));
                return;
            }
        }
        server.send(507, "text/plain", "Storage full");
    }
    else
    {
        server.send(400, "text/plain", "No card detected(Timeout)");
    }
}

void handleListUsers()
{
    JsonDocument doc;
    JsonArray users = doc.to<JsonArray>();

    for (int i = 0; i < MAX_USERS; i++)
    {
        User u = readUser(i);
        if (strlen(u.uid) > 0)
        {
            JsonObject user = users.add<JsonObject>();
            user["uid"] = u.uid;
            user["balance"] = u.balance;
        }
    }

    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
}

void handleTopUp()
{
    if (!server.hasArg("uid") || !server.hasArg("amount"))
    {
        server.send(400, "text/plain", "Missing UID or amount");
        return;
    }

    String uidArg = server.arg("uid");
    String amountArg = server.arg("amount");

    // Input validation
    if (uidArg.length() == 0 || uidArg.length() > MAX_UID_LENGTH)
    {
        server.send(400, "text/plain", "Invalid UID format");
        return;
    }

    float amount = amountArg.toFloat();
    if (amount <= 0 || isnan(amount))
    {
        server.send(400, "text/plain", "Invalid amount");
        return;
    }

    if (xSemaphoreTake(xUserMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        bool userFound = false;

        for (int i = 0; i < MAX_USERS; i++)
        {
            User u = readUser(i);
            if (strcmp(u.uid, uidArg.c_str()) == 0)
            {
                userFound = true;
                float newBalance = u.balance + amount;

                // Create modified user
                User modified = u;
                modified.balance = newBalance;

                if (saveUser(i, modified))
                {
                    char response[128];
                    snprintf(response, sizeof(response),
                             "Success: Added $%.2f to %s\nNew Balance: $%.2f",
                             amount, u.uid, newBalance);
                    server.send(200, "text/plain", response);
                }
                else
                {
                    server.send(500, "text/plain", "Failed to save user data");
                }
                break;
            }
        }

        xSemaphoreGive(xUserMutex);

        if (!userFound)
        {
            server.send(404, "text/plain", "User not found");
        }
    }
    else
    {
        server.send(503, "text/plain", "Service busy, try again");
    }
}

void handleDeleteUser()
{
    if (server.method() != HTTP_POST)
    {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    if (!server.hasArg("uid"))
    {
        server.send(400, "text/plain", "Missing UID parameter");
        return;
    }

    String uidArg = server.arg("uid");
    if (uidArg.length() == 0 || uidArg.length() > MAX_UID_LENGTH)
    {
        server.send(400, "text/plain", "Invalid UID format");
        return;
    }

    if (xSemaphoreTake(xUserMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        bool userFound = false;

        for (int i = 0; i < MAX_USERS; i++)
        {
            User u = readUser(i);
            if (strcmp(u.uid, uidArg.c_str()) == 0)
            {
                userFound = true;

                // Create empty user
                User emptyUser;
                memset(emptyUser.uid, 0, sizeof(emptyUser.uid));
                emptyUser.balance = 0.0f;
                deleteUser = true;

                if (saveUser(i, emptyUser))
                {
                    server.send(200, "text/plain", "User deleted successfully");
                }
                else
                {
                    server.send(500, "text/plain", "Failed to delete user");
                }
                break;
            }
        }

        xSemaphoreGive(xUserMutex);

        if (!userFound)
        {
            server.send(404, "text/plain", "User not found");
        }
    }
    else
    {
        server.send(503, "text/plain", "Service busy, try again");
    }
}

// New endpoint on ESP32
void handleEVData()
{
    // if (!can.checkError())
    // { // Add this check
    //     server.send(503, "application/json", "{\"error\":\"CAN not initialized\"}");
    //     return;
    // }
    String json = "{";
    json += "\"connected\":" + String(evConnected ? "true" : "false") + ",";

    if (evConnected)
    {
        json += "\"voltage\":" + String(evVoltage, 1) + ",";
        json += "\"current\":" + String(evCurrent, 1) + ",";
        json += "\"soc\":" + String(evSOC) + ",";
        json += "\"soh\":" + String(evSOH) + ",";
        json += "\"Temperature\":" + String(ev_Temperature) + ",";
    }
    else
    {
        json += "\"voltage\":0,";
        json += "\"current\":0,";
        json += "\"soc\":0,";
        json += "\"soh\":0,";
        json += "\"Temperature\":0,";
    }

    json += "\"charging\":" + String(chargingStatus ? "true" : "false");
    json += "}";

    server.send(200, "application/json", json);
}

// void handleCharge()
// {
//     if (server.hasArg("cmd"))
//     {
//         String cmd = server.arg("cmd");
//         if (cmd == "start")
//         {
//             sendCANCommand(0x01); // Start charging
//             chargingStatus = true;
//         }
//         else
//         {
//             sendCANCommand(0x00); // Stop charging
//             chargingStatus = false;
//         }
//         server.send(200, "text/plain", "Command accepted");
//     }
// }

void handleReboot()
{
    server.send(200, "application/json", "{\"status\":\"rebooting\"}");
    delay(1000); // Give time for the response to be sent
    ESP.restart();
}

// Endpoint for pricing
void handleSetRate()
{

    if (server.hasArg("rate"))
    {
        if (server.arg("rate").toFloat() > 100.00 || server.arg("rate").toFloat() < 5.00)
        { // Max $1/kWh
            server.send(403, "text/plain", "Invalid Rate");
            return;
        }
        chargingRate = server.arg("rate").toFloat();
        EEPROM.put(ENERGY_PRICE_ADDR, chargingRate); // Store in EEPROM
        EEPROM.commit();
        server.send(200, "text/plain", "Rate set: $" + String(chargingRate, 3) + "/kWh");
    }
}

// Authentication Handler
void handleAuthCard()
{
    User receivedTag;
    memset(&receivedTag, 0, sizeof(receivedTag)); // Initialize

    // Wait for new RFID (with timeout)
    if (xQueueReceive(xRFIDQueue, &receivedTag, pdMS_TO_TICKS(5000)) == pdTRUE)
    {
        if (xSemaphoreTake(xUserMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            for (int i = 0; i < MAX_USERS; i++)
            {
                User u = readUser(i);
                if (strcmp(u.uid, receivedTag.uid) == 0)
                {
                    // Create response
                    // Create response
                    JsonDocument doc;      // Prefer StaticJsonDocument<size> if possible (e.g., StaticJsonDocument<256>)
                    doc["success"] = true; // Root-level "success"

                    // Create a nested "user" object (modern syntax)
                    JsonObject user = doc["user"].to<JsonObject>();
                    user["uid"] = u.uid;
                    user["balance"] = u.balance;

                    // Serialize and send
                    String output;
                    serializeJson(doc, output);
                    server.send(200, "application/json", output);

                    strncpy(currentAuthUser.uid, u.uid, MAX_UID_LENGTH);
                    currentAuthUser.balance = u.balance;

                    xSemaphoreGive(xUserMutex);
                    return;
                }
            }
            xSemaphoreGive(xUserMutex);
        }
    }

    // Fallback response
    server.send(200, "application/json",
                "{\"success\":false,\"message\":\"User not found or timeout\"}");
}

// Charging Control
void handleStartCharging()
{

    if (server.hasArg("configCharge"))
    {
        String cmd = server.arg("configCharge");
        if (cmd == "wired")
        {
            digitalWrite(WIRELESS_CHARGE_PIN, LOW);
            digitalWrite(WIRED_CHARGE_PIN, HIGH);

            // Start energy tracking
            if (!isCharging)
            {
                chargeStartTime = millis();
                startEnergy = energyDelivered;
                isCharging = true;
            }

            sendCANCommand(0x01); // Start charging
            chargingStatus = true;
            server.send(200, "text/plain", "DC Charging started");
            return;
        }
        else if (cmd == "wireless")
        {
            digitalWrite(WIRED_CHARGE_PIN, LOW);
            digitalWrite(WIRELESS_CHARGE_PIN, HIGH);
            sendCANCommand(0x01); // Start charging
            chargingStatus = true;
            server.send(200, "text/plain", "Wireless Charging started");
            return;
            
        }
        else
        {
            digitalWrite(WIRELESS_CHARGE_PIN, LOW);
            digitalWrite(WIRED_CHARGE_PIN, LOW);
            sendCANCommand(0x00); // Stop charging
            chargingStatus = false;
            server.send(200, "text/plain", "Invalid Commmand");
            return;
        }
        // server.send(200, "text/plain", "Command accepted");
    }
    
    else
    {
        server.send(200, "text/plain", "Invalid Argument"); 
        return;
    }

    // Serial.println("Charging started");
}

void handleEndCharging()
{
    digitalWrite(WIRELESS_CHARGE_PIN, LOW);
    digitalWrite(WIRED_CHARGE_PIN, LOW);
    sendCANCommand(0x00); // Stop charging
    chargingStatus = false;

    // Calculate final cost
    float cost = 0;
    if (isCharging)
    {
        
        updateEnergyMeasurement(); // Final update
        saveEnergyData();
        cost = sessionEnergy * chargingRate;

        // Deduct from user balance if authenticated
        if (strlen(currentAuthUser.uid) > 0)
        {
            int userIndex = findUserIndex(currentAuthUser.uid);
            if (userIndex >= 0)
            { // Only proceed if user was found
                currentAuthUser.balance -= cost;
                saveUser(userIndex, currentAuthUser);
            }
            else
            {
                Serial.println("Failed to find user index for UID: " + String(currentAuthUser.uid));
            }
        }
        else{
            Serial.print("UId is   ");
            Serial.println(currentAuthUser.uid);
        }

        isCharging = false;
    }
    else{
        Serial.println("Not Charging");
    }
    // Serial.println(sessionEnergy);
    // Serial.println(cost);
    // Serial.println(chargingRate);

        // Create response
        JsonDocument doc;
    doc["status"] = "stopped";
    doc["energy_kwh"] = sessionEnergy;
    doc["cost"] = cost;
    doc["rate"] = chargingRate;
    if (strlen(currentAuthUser.uid) > 0)
    {
        doc["balance_remaining"] = currentAuthUser.balance;
    }

    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
   

    // Reset session
    sessionEnergy = 0;
    chargeStartTime = 0;
}

void initSOCEstimator()
{
    xSOCMutex = xSemaphoreCreateMutex();
    if (xSOCMutex == NULL)
    {
        // Handle error - critical system failure
    }
}

float estimateSOC(float voltage, float temperature)
{
    // Validate inputs
    if (isnan(voltage) || isnan(temperature))
    {
        return 0.0f; // Safe default
    }

    // Calculate per-cell voltage
    float cellVoltage = voltage / BATTERY_CELLS;

    // Apply temperature compensation
    float compensatedVoltage = cellVoltage + ((25.0f - temperature) * TEMP_COEFFICIENT);

    // Constrain to valid range
    compensatedVoltage = constrain(compensatedVoltage, MIN_VOLTAGE, MAX_VOLTAGE);

    // Find SOC using lookup table with linear interpolation
    for (size_t i = 1; i < voltageSocTableSize; i++)
    {
        if (compensatedVoltage <= voltageSocTable[i].voltage)
        {
            float voltageRange = voltageSocTable[i].voltage - voltageSocTable[i - 1].voltage;
            float socRange = voltageSocTable[i].soc - voltageSocTable[i - 1].soc;
            float voltageDelta = compensatedVoltage - voltageSocTable[i - 1].voltage;

            float soc = voltageSocTable[i - 1].soc + ((voltageDelta / voltageRange) * socRange);

            // Protect against potential floating point errors
            return constrain(soc, 0.0f, 100.0f);
        }
    }

    return 100.0f; // If voltage is above maximum table value
}

// Thread-safe version to use from other tasks
float getEstimatedSOC(float temperature)
{
    float soc = 0.0f;

    if (xSemaphoreTake(xSOCMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        float battery_voltage;

        // Get latest voltage reading (protected by mutex)
        if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            battery_voltage = g_battery_voltage;
            xSemaphoreGive(xSensorDataMutex);
        }

        soc = estimateSOC(battery_voltage, temperature);
        xSemaphoreGive(xSOCMutex);
    }

    return soc;
}

float readNTC()
{
    // Average multiple readings
    const int samples = 5;
    float average = 0;
    // analogReadResolution(12); // Switch to 12-bit (0–4095)
    for (int i = 0; i < samples; i++)
    {
        average += analogRead(ntcPin);
        delay(10);
    }
    average /= samples;

    // Convert ADC value to voltage (ESP32 ADC reference voltage is 3.3V)
    float voltage = average * (3.3 / 4095.0);

    // Calculate NTC resistance
    float ntcResistance = SERIES_RESISTOR / ((3.3 / voltage) - 1.0);

    // Serial.println(ntcResistance);

    // Steinhart-Hart equation to convert resistance to temperature
    float steinhart;
    steinhart = ntcResistance / NTC_NOMINAL_RESISTANCE; // (R/Ro)
    steinhart = log(steinhart);                         // ln(R/Ro)
    steinhart /= NTC_B_VALUE;                           // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NTC_NOMINAL_TEMP + 273.15);     // + (1/To)
    steinhart = 1.0 / steinhart;                        // Invert
    steinhart -= 273.15;                                // Convert to Celsius

    return steinhart;
}


void updateSOH(float currentVoltage, float temperature)
{
    if ((currentVoltage / BATTERY_CELLS) < (1.5))
    {
        batterySOH = 0;
        return;
    }
    // Only update SOH once per hour (adjust as needed)
    static unsigned long lastUpdateTime = 0;
    const unsigned long updateInterval = 3600000; // 1 hour in ms

    if (millis() - lastUpdateTime < updateInterval)
    {
        return;
    }
    lastUpdateTime = millis();

    // Safety checks
    if (isnan(currentVoltage) || isnan(temperature))
    {
        Serial.println("Invalid voltage or temperature for SOH calculation");
        return;
    }

    // Calculate per-cell voltage
    float cellVoltage = currentVoltage / BATTERY_CELLS;

    // 1. Voltage-based SOH estimation
    // Normalized voltage factor (0-1 range)
    float voltageFactor = (cellVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE);
    voltageFactor = constrain(voltageFactor, 0.0f, 1.0f);

    // 2. Temperature-based factor (0-1 range)
    // Ideal temperature is 25°C, deviation reduces SOH
    float tempDeviation = fabs(temperature - 25.0f);
    float tempFactor = 1.0f - constrain(tempDeviation * 0.01f, 0.0f, 0.3f); // 1% reduction per °C, max 30%

    // 3. Capacity fade over time (simplified)
    static float capacityFade = 0.0f;
    capacityFade += 0.0001f;                            // 0.01% per hour (~10% per year)
    capacityFade = constrain(capacityFade, 0.0f, 0.3f); // Max 30% capacity fade

    // Combine factors with weights
    float combinedSOH = (voltageFactor * 0.4f) + (tempFactor * 0.3f) + ((1.0f - capacityFade) * 0.3f);

    // Apply smoothing to prevent rapid changes
    static float smoothedSOH = 100.0f;
    smoothedSOH = (smoothedSOH * 0.9f) + (combinedSOH * 100.0f * 0.1f);

    // Constrain to valid range
    batterySOH = constrain(smoothedSOH, 70.0f, 100.0f); // Never show below 70%

    Serial.printf("SOH Update - Voltage: %.2fV (factor: %.2f), Temp: %.1f°C (factor: %.2f), Fade: %.2f%%, SOH: %.1f%%\n",
                  cellVoltage, voltageFactor,
                  temperature, tempFactor,
                  capacityFade * 100.0f,
                  batterySOH);
}

// SaveUser function (thread-safe with validation)
bool saveUser(int index, const User &u)
{
    if (index < 0 || index >= MAX_USERS)
    {
        Serial.println("Invalid user index");
        return false;
    }

    // Validate UID
    if ((strlen(u.uid) == 0 || strlen(u.uid) > MAX_UID_LENGTH) && !deleteUser)
    {
        Serial.println("Invalid UID length");
        return false;
    }
    deleteUser = false;

    // Validate balance
    if (isnan(u.balance))
    {
        Serial.println("Invalid balance");
        return false;
    }

    int addr = index * USER_STORAGE_SIZE;
    bool success = false;

    if (xSemaphoreTake(xEEPROMMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        // Debug print before write
        Serial.printf("Saving to addr %d: UID='%s', Balance=%.2f\n",
                      addr, u.uid, u.balance);

        // Write UID
        EEPROM.writeBytes(addr, u.uid, MAX_UID_LENGTH + 1);

        // Write balance
        EEPROM.put(addr + MAX_UID_LENGTH + 1, u.balance);

        // Calculate checksum
        uint8_t checksum = 0;
        for (size_t i = 0; i < sizeof(User) - 1; i++)
        {
            checksum ^= EEPROM.read(addr + i);
        }
        EEPROM.write(addr + USER_STORAGE_SIZE - 1, checksum);

        // Commit with verification
        success = EEPROM.commit();

        if (!success)
        {
            Serial.println("EEPROM commit failed!");
            // Try one more time
            success = EEPROM.commit();
            if (!success)
            {
                Serial.println("Second commit attempt failed!");
            }
        }

        xSemaphoreGive(xEEPROMMutex);

        // Verify write
        if (success)
        {
            User verify = readUser(index);
            if (strcmp(verify.uid, u.uid) != 0 || abs(verify.balance - u.balance) > 0.01)
            {
                Serial.println("Write verification failed!");
                success = false;
            }
        }
    }
    else
    {
        Serial.println("Failed to acquire EEPROM mutex");
    }

    return success;
}

// Modified readUser function
User readUser(int index)
{
    User u;
    memset(u.uid, 0, sizeof(u.uid)); // Initialize uid

    if (index < 0 || index >= MAX_USERS)
        return u;

    int addr = index * USER_STORAGE_SIZE;

    if (xSemaphoreTake(xEEPROMMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        // Read UID
        EEPROM.readBytes(addr, u.uid, MAX_UID_LENGTH + 1);

        // Read balance
        EEPROM.get(addr + MAX_UID_LENGTH + 1, u.balance);

        // Verify checksum
        uint8_t storedChecksum = EEPROM.read(addr + USER_STORAGE_SIZE - 1);
        uint8_t calculatedChecksum = 0;
        for (size_t i = 0; i < USER_STORAGE_SIZE - 1; i++)
        {
            calculatedChecksum ^= EEPROM.read(addr + i);
        }

        xSemaphoreGive(xEEPROMMutex);

        // Validate data
        if (calculatedChecksum != storedChecksum ||
            strlen(u.uid) == 0 ||
            isnan(u.balance))
        {
            memset(u.uid, 0, sizeof(u.uid));
            u.balance = 0.0f;
        }
    }

    return u;
}


int findUserIndex(const char *uid)
{
    if (uid == nullptr || strlen(uid) == 0)
    {
        return -1; // Invalid UID
    }

    for (int i = 0; i < MAX_USERS; i++)
    {
        User u = readUser(i);
        if (strcmp(u.uid, uid) == 0)
        {
            return i; // Found matching user
        }
    }

    return -1; // User not found
}

void sendCANCommand(uint8_t cmd)
{
    can_frame txMsg;
    txMsg.can_id = 0x101;
    txMsg.can_dlc = 2;
    txMsg.data[0] = cmd;
    txMsg.data[1] = cmd ^ 0xFF; // Simple checksum
    can.sendMessage(&txMsg);
}

void handleUpdate()
{
    HTTPUpload &upload = server.upload();

    if (upload.status == UPLOAD_FILE_START)
    {
        if (otaInProgress)
        {
            server.send(503, "text/plain", "OTA update already in progress");
            return;
        }

        otaInProgress = true;
        Serial.println("Suspended background tasks for OTA");

        // Suspend non-critical tasks
        if (xSensorTask)
            vTaskSuspend(xSensorTask);
        if (xRFIDTask)
            vTaskSuspend(xRFIDTask);
        if (xCANHandlerTask)
            vTaskSuspend(xCANHandlerTask);

        Serial.printf("OTA Started: %s\n", upload.filename.c_str());
        Update.abort();

        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH))
        {
            Serial.println("OTA Begin failed:");
            Update.printError(Serial);

            // Resume tasks on failure
            if (xSensorTask)
                vTaskResume(xSensorTask);
            if (xRFIDTask)
                vTaskResume(xRFIDTask);
            Serial.println("Resumed background tasks");

            otaInProgress = false;
            return;
        }
    }
    else if (upload.status == UPLOAD_FILE_WRITE)
    {
        if (!otaInProgress)
            return;

        // Progress reporting
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 1000)
        {
            Serial.printf("OTA Progress: %d%%\r", (upload.totalSize * 100) / Update.size());
            lastPrint = millis();
        }

        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
        {
            Serial.println("OTA Write failed:");
            Update.printError(Serial);
        }
    }
    else if (upload.status == UPLOAD_FILE_END)
    {
        if (!otaInProgress)
            return;

        if (Update.end(true))
        {
            Serial.printf("\nOTA Success: %u bytes\nRebooting...\n", upload.totalSize);
            server.send(200, "text/plain", "Update complete. Rebooting...");
            server.client().stop();
            delay(100);
            ESP.restart();
        }
        else
        {
            Serial.println("OTA End failed:");
            Update.printError(Serial);
            server.send(500, "text/plain", "Update failed. Check serial logs.");

            // Resume tasks on failure
            if (xSensorTask)
                vTaskResume(xSensorTask);
            if (xRFIDTask)
                vTaskResume(xRFIDTask);
            Serial.println("Resumed background tasks");
        }
        otaInProgress = false;
    }
    else
    { // UPLOAD_FILE_ABORTED or other status
        if (otaInProgress)
        {
            Serial.println("OTA Aborted");
            Update.abort();

            // Resume tasks on abort
            if (xSensorTask)
                vTaskResume(xSensorTask);
            if (xRFIDTask)
                vTaskResume(xRFIDTask);
            Serial.println("Resumed background tasks");

            otaInProgress = false;
        }
        server.send(500, "text/plain", "Upload failed");
    }
}

void handleUpdatePage()
{
    if (!server.authenticate(OTA_USER, OTA_PASS))
    {
        return server.requestAuthentication();
    }
    String html = "<form method='POST' action='/update' enctype='multipart/form-data'>"
                  "<input type='file' name='update'>"
                  "<input type='submit' value='Update'>"
                  "</form>";
    server.send(200, "text/html", html);
}

void updateEnergyMeasurement()
{
    static unsigned long lastSaveTime = 0;
    const unsigned long saveInterval = 30000; // Save every 30 seconds

    if (!isCharging)
        return;

    // Get current power (convert mA to A and V to kW)
    // float power_kW = (g_battery_current / 1000.0) * (g_battery_voltage / 1000.0);
    // float power_kW = (evCurrent / 1000.0) * (evVoltage / 1000.0);
    float power_kW = (evCurrent ) * (evVoltage );    //for testing only
    
    // Calculate time in hours
    // float hours = (millis() - chargeStartTime) / 3600000.0;
    float hours = (millis() - chargeStartTime) / 1000.0; // 1 hour assumed 1 sec for testing only
    
    // Update energy delivered (kWh)
    sessionEnergy = power_kW * hours;
    energyDelivered = startEnergy + sessionEnergy;


    // Serial.print("Power_kW : ");
    // Serial.print(power_kW);
    // Serial.print(" Hours : ");
    // Serial.print(hours);
    // Serial.print(" Session Energy : ");
    // Serial.print(sessionEnergy);
    // Serial.print(" EnergyDelivered : ");
    // Serial.println(energyDelivered);

    // Periodic save to EEPROM
    if (millis() - lastSaveTime > saveInterval)
    {
        saveEnergyData();
        lastSaveTime = millis();
    }
}

void saveEnergyData()
{
    if (xSemaphoreTake(xEEPROMMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        EEPROM.put(ENERGY_EEPROM_ADDR, energyDelivered);
        bool success = EEPROM.commit();
        xSemaphoreGive(xEEPROMMutex);

        if (!success)
        {
            Serial.println("Failed to save energy data!");
        }
    }
}