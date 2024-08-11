#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>

// Wi-Fi configuration
const char* ssid = "globalnet_abdallah";
const char* password = "51301579";

// WiFiClient object for Wi-Fi connection
WiFiClient espClient;

// MQTT server configuration
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_topic = "/firmware/blink_led_url";


102.219.178.133
// PubSubClient object for MQTT connection
PubSubClient client(espClient);

// UART configuration
HardwareSerial mySerial(1); // Use UART1
const int uartBaudRate = 115200;

// Variables to store URL and CRC received from MQTT message
String downloadUrl;
String receivedCRC;

// Function prototypes
void setup_wifi();
void connect_to_mqtt();
void callback(char* topic, byte* payload, unsigned int length);
bool downloadAndTransferFile(String url, String crcValue);

void setup() {
    Serial.begin(115200); // Initialize serial communication
    mySerial.begin(uartBaudRate, SERIAL_8N1, 16, 17); // Initialisation de UART1 (TX=16, RX=17)
    setup_wifi(); // Connect to Wi-Fi
    client.setServer(mqtt_server, mqtt_port); // Configure MQTT server
    client.setCallback(callback); // Set callback function for MQTT messages
    connect_to_mqtt(); // Connect to MQTT server
}

void loop() {
    client.loop(); // Maintain MQTT connection
}

// Function to connect to Wi-Fi
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

// Function to connect to MQTT server
void connect_to_mqtt() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) { // Connect to MQTT server
            Serial.println("connected");
            client.subscribe(mqtt_topic); // Subscribe to MQTT topic
            Serial.print("Subscribed to topic: ");
            Serial.println(mqtt_topic);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000); // Wait 5 seconds before retrying
        }
    }
}

// Callback function to handle received MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");

    // Convert payload to string
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println(message);

    // Parse the message for URL and CRC
    int urlIndex = message.indexOf("\"url\":");
    int crcIndex = message.indexOf("\"crc\":");

    if (urlIndex != -1 && crcIndex != -1) {
        // Extract URL
        int urlStart = message.indexOf("\"", urlIndex + 6) + 1;
        int urlEnd = message.indexOf("\"", urlStart);
        downloadUrl = message.substring(urlStart, urlEnd);

        // Extract CRC
        int crcStart = message.indexOf("\"", crcIndex + 6) + 1;
        int crcEnd = message.indexOf("\"", crcStart);
        receivedCRC = message.substring(crcStart, crcEnd);

        Serial.print("Received URL: ");
        Serial.println(downloadUrl);
        Serial.print("Received CRC: ");
        Serial.println(receivedCRC);

        // Trigger file download and transfer process
        if (downloadAndTransferFile(downloadUrl, receivedCRC)) {
            Serial.println("File transferred successfully");
        } else {
            Serial.println("Failed to download and transfer file");
        }
    } else {
        Serial.println("Failed to parse message");
    }
}

bool downloadAndTransferFile(String url, String crcValue) {
    HTTPClient http;

    Serial.print("Connecting to URL: ");
    Serial.println(url);

    http.begin(url); // Initialize HTTP connection

    int httpCode = http.GET(); // Perform HTTP GET request

    if (httpCode == HTTP_CODE_OK) {
        Serial.println("File downloaded successfully");
        WiFiClient* stream = http.getStreamPtr(); // Get the response stream

        const size_t bufferSize = 1024;
        uint8_t buffer[bufferSize];

        // Indicate the start of file transfer
        Serial.println("Transferring file...");

        // Send the size of the file
        size_t fileSize = http.getSize();
        Serial.print("File size: ");
        Serial.println(fileSize);
        mySerial.write((uint8_t*)&fileSize, sizeof(fileSize));

        // Transfer file via UART
        while (stream->available()) {
            size_t bytesRead = stream->readBytes(buffer, bufferSize);
            mySerial.write(buffer, bytesRead); // Transfer file data via UART
        }
        // Transfer CRC via UART
        mySerial.write(crcValue.c_str(), crcValue.length());
        Serial.println("CRC transferred successfully");

        http.end(); // Close HTTP connection
        return true;
    } else {
        Serial.print("Error downloading file. HTTP code: ");
        Serial.println(httpCode);
        Serial.println(http.errorToString(httpCode));
        http.end(); // Close HTTP connection
        return false;
    }
}
