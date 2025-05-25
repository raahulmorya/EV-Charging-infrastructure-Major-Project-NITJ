// #include <ESP8266WiFi.h>
// #include <ESP8266WebServer.h>
// #include <Wire.h>
// #include <SPI.h>
// #include <mcp2515.h>
// #include <Adafruit_INA219.h>

// // Configuration Constants
// #define MEASUREMENT_INTERVAL 1000 // ms
// #define BATTERY_CAPACITY 2600.0   // mAh
// #define TOTAL_CYCLES 1000.0       // Estimated cycles at 80% capacity
// #define NTC_SAMPLES 5             // Number of NTC readings to average
// #define SOC_LOW_THRESHOLD 20      // SOC threshold for cycle counting
// #define SOC_HIGH_THRESHOLD 90     // SOC threshold for cycle counting

// enum VehicleType
// {
//   PORSCHE = 0x01,
//   TESLA = 0x02,
//   AUDI = 0x03,
//   BMW = 0x04,
//   // Add more as needed
// };

// uint8_t vehicle_type_code = PORSCHE; // Set during initialization

// // Hardware Configuration
// MCP2515 can(D8); // CS on D8 (GPIO15)
// Adafruit_INA219 ina219_ev_battery(0x40);

// // WiFi Credentials
// const char *ssid = "OPTIMUS";
// const char *password = "qqwweeaaaa";

// // Web Server
// ESP8266WebServer server(80);

// // NTC Thermistor Configuration
// const int ntcPin = A0;
// const float ntcNominalResistance = 10000.0; // Resistance at 25°C
// const float ntcNominalTemp = 25.0;          // Temperature for nominal resistance
// const float ntcBCoefficient = 3950.0;       // B coefficient
// const float seriesResistor = 10000.0;       // Value of the series resistor

// // Battery State Variables
// struct BatteryState
// {
//   float voltage;
//   float current;
//   float temperature;
//   float soc;
//   float soh;
//   float remainingCapacity;
//   float cycleCount;
//   bool charging;
// };

// BatteryState battery = {
//     .voltage = 0,
//     .current = 0,
//     .temperature = 0,
//     .soc = 100,
//     .soh = 100,
//     .remainingCapacity = BATTERY_CAPACITY,
//     .cycleCount = 0,
//     .charging = false};


// unsigned long lastMeasurementTime = 0;

// // Function Prototypes
// void initializeHardware();
// void connectToWiFi();
// void setupWebServer();
// float readTemperature();
// void measureBatteryParameters();
// float calculateSoC();
// float calculateSoH();
// void updateCycleCount();
// void handleRoot();
// void handleData();
// void sendEVStatus();
// void checkChargerCommands();
// uint8_t calculateChecksum(uint8_t *data);

// void initializeHardware()
// {
//   // Initialize I2C and SPI
//   Wire.begin();
//   SPI.begin();
//   can.reset();
//   can.setBitrate(CAN_125KBPS, MCP_8MHZ);
//   can.setNormalMode();

//   Serial.println("CAN Init Done");

//   // Initialize INA219
//   if (!ina219_ev_battery.begin())
//   {
//     Serial.println("Failed to initialize INA219");
//     while (1)
//       delay(10);
//   }

// }

// void connectToWiFi()
// {
//   WiFi.begin(ssid, password);
//   WiFi.setAutoReconnect(true);
//   WiFi.persistent(true);

//   Serial.print("Connecting to WiFi");
//   while (WiFi.status() != WL_CONNECTED)
//   {
//     delay(500);
//     Serial.print(".");
//   }

//   Serial.println("\nConnected!");
//   Serial.print("IP Address: ");
//   Serial.println(WiFi.localIP());
// }

// void setupWebServer()
// {
//   server.enableCORS(true);
//   server.on("/", handleRoot);
//   server.on("/data", handleData);
//   server.begin();
//   Serial.println("HTTP server started");
// }

// float readTemperature()
// {
//   float average = 0;

//   for (int i = 0; i < NTC_SAMPLES; i++)
//   {
//     average += analogRead(ntcPin);
//     delay(10);
//   }
//   average /= NTC_SAMPLES;

//   // Convert ADC value to resistance
//   float voltage = average * (3.3 / 1023.0);
//   float ntcResistance = seriesResistor * ((3.3 / voltage) - 1.0);

//   // Calculate temperature using Steinhart-Hart equation
//   float steinhart = ntcResistance / ntcNominalResistance;
//   steinhart = log(steinhart);
//   steinhart /= ntcBCoefficient;
//   steinhart += 1.0 / (ntcNominalTemp + 273.15);
//   steinhart = 1.0 / steinhart;
//   steinhart -= 273.15;

//   return steinhart;
// }

// void measureBatteryParameters()
// {
//   // Read voltage and current
//   battery.voltage = ina219_ev_battery.getBusVoltage_V() +
//                     (ina219_ev_battery.getShuntVoltage_mV() / 1000);
//   battery.current = ina219_ev_battery.getCurrent_mA();

//   // Read temperature
//   battery.temperature = readTemperature();

//   // Update SOC and SOH
//   battery.soc = calculateSoC();
//   updateCycleCount();
//   battery.soh = calculateSoH();

//   // Diagnostic output
//   Serial.printf("V: %.2fV, I: %.2fmA, Temp: %.1f°C, SOC: %.1f%%, SOH: %.1f%%\n",
//                 battery.voltage, battery.current, battery.temperature, battery.soc, battery.soh);
// }

// float calculateSoC()
// {
//   unsigned long currentTime = millis();

//   if (lastMeasurementTime == 0)
//   {
//     lastMeasurementTime = currentTime;
//     return (battery.remainingCapacity / BATTERY_CAPACITY) * 100;
//   }

//   // Calculate elapsed time in hours
//   float elapsedHours = (currentTime - lastMeasurementTime) / 3600000.0;

//   // Update remaining capacity (coulomb counting)
//   battery.remainingCapacity -= battery.current * elapsedHours;

//   // Apply bounds
//   battery.remainingCapacity = constrain(battery.remainingCapacity, 0, BATTERY_CAPACITY);

//   lastMeasurementTime = currentTime;
//   return (battery.remainingCapacity / BATTERY_CAPACITY) * 100;
// }

// float calculateSoH()
// {
//   // Simple linear degradation model
//   float degradation = min(battery.cycleCount / TOTAL_CYCLES, 1.0);
//   return (1.0 - degradation) * 100;
// }

// void updateCycleCount()
// {
//   static bool wasBelowThreshold = false;

//   if (battery.soc < SOC_LOW_THRESHOLD && !wasBelowThreshold)
//   {
//     wasBelowThreshold = true;
//   }
//   else if (battery.soc > SOC_HIGH_THRESHOLD && wasBelowThreshold)
//   {
//     wasBelowThreshold = false;
//     battery.cycleCount += 0.5; // Half cycle (discharge + charge = full cycle)
//     Serial.printf("Cycle count updated: %.1f\n", battery.cycleCount);
//   }
// }

// void handleRoot()
// {
//   String html = "Hey, I am " + (String)vehicle_type_code + "....... Wanna Ride it!";
//   server.send(200, "text/html", html);
// }

// void handleData()
// {
//   String json = "{";
//   json += "\"vehicle_name\":\"" + String(vehicle_type_code) + "\",";
//   json += "\"battery_voltage\":" + String(battery.voltage, 2) + ",";
//   json += "\"battery_current\":" + String(battery.current, 2) + ",";
//   json += "\"soc\":" + String(battery.soc, 1) + ",";
//   json += "\"soh\":" + String(battery.soh, 1) + ",";
//   json += "\"temperature\":" + String(battery.temperature, 1) + ",";
//   json += "\"charging\":" + String(battery.charging ? "true" : "false");
//   json += "}";

//   server.send(200, "application/json", json);
// }

// void sendEVStatus()
// {
//   can_frame msg;
//   msg.can_id = 0x100;
//   msg.can_dlc = 8;  

//   // Pack measurements
//   uint16_t voltage = battery.voltage * 100;
//   uint16_t current = abs(battery.current);

//   msg.data[0] = voltage >> 8;
//   msg.data[1] = voltage & 0xFF;
//   msg.data[2] = current >> 8;
//   msg.data[3] = current & 0xFF;
//   msg.data[4] = (uint8_t)constrain(battery.soc, 0, 100);
//   msg.data[5] = (uint8_t)constrain(battery.soh, 0, 100);
//   msg.data[6] = (uint8_t)constrain(round(battery.temperature), -128, 127);

//   // // Byte 7: [7]Checksum [6:2]VehicleType [1]Charging [0]Reserved
//   // msg.data[7] = (vehicle_type_code << 2) | (battery.charging ? 0x02 : 0x00);
//   // msg.data[7] ^= calculateChecksum(msg.data); // XOR checksum with MSB

//   msg.data[7] = (vehicle_type_code << 2) | (battery.charging ? 0x02 : 0x00) | 0x01;
//   msg.data[7] ^= calculateChecksum(msg.data);

//   can.sendMessage(&msg);
// }

// void checkChargerCommands()
// {
//   if (can.checkReceive())
//   {
//     can_frame rxMsg;
//     can.readMessage(&rxMsg);
    
//       if (rxMsg.can_id == 0x101 && rxMsg.can_dlc == 2)
//       {
//         if (rxMsg.data[1] == (rxMsg.data[0] ^ 0xFF))
//         {
//           battery.charging = (rxMsg.data[0] == 0x01);
//           Serial.println(battery.charging ? "Charging STARTED" : "Charging STOPPED");
//         }
//       }
      
//   }
// }

// uint8_t calculateChecksum(uint8_t *data)
// {
//   uint8_t checksum = 0;
//   for (int i = 0; i < 7; i++)
//   {
//     checksum ^= data[i];
//   }
//   return checksum;
// }



// void setup()
// {
//   Serial.begin(115200);
//   Serial.println("\nStarting EV Battery Monitor");

//   initializeHardware();
//   connectToWiFi();
//   setupWebServer();

//   Serial.println("System initialization complete");
// }

// void loop()
// {
//   server.handleClient();

//   static uint32_t lastUpdate = 0;
//   if (millis() - lastUpdate >= MEASUREMENT_INTERVAL)
//   {
//     measureBatteryParameters();
//     sendEVStatus();
//     lastUpdate = millis();
//   }

//   checkChargerCommands();
// }

