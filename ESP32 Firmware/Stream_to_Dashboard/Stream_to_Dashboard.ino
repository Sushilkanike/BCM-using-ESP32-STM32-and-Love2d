/*
 * ESP32 UDP Streaming Server
 *
 * This code turns the ESP32 into a data streaming server for the LÖVE dashboard.
 *
 * HOW IT WORKS:
 * 1. The ESP32 connects to WiFi and starts a UDP server, listening on a port.
 * 2. It waits for any packet from a client (the LÖVE dashboard).
 * 3. When it receives a packet, it saves the client's IP address and port.
 * 4. It then begins streaming sample JSON data packets to that client
 * at a fixed interval, without waiting for more requests.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <IPAddress.h> // For storing the client's IP

// --- WiFi Configuration ---
const char *ssid = "Sushil's Phone : )";
const char *password = "8712308542";
unsigned int localPort = 8888; // The port we will listen on

// --- UDP Objects ---
WiFiUDP udp;
char packetBuffer[255]; // Buffer for incoming packets

// --- Client State ---
IPAddress clientIP;        // Stores the IP of the dashboard
uint16_t clientPort;       // Stores the port of the dashboard
bool clientConnected = false; // Flag to know if we should start streaming

// --- Data Streaming ---
unsigned long lastSendTime = 0; // For timing the packets
const long sendInterval = 100;  // Send a packet every 100ms (10 FPS)
int currentPacketIndex = 0;     // Which packet to send next

// --- Array of Sample Data Packets (as JSON strings) ---
// These match the data structure our LÖVE dashboard expects
const char* samplePackets[] = {
  "SPEED=0,FUEL=75,GEAR=P,ODOMETER=12345.6,BLINK=NONE,HI_BEAM=false,CHK_ENG=false,OIL=false,BATT=false",
  "SPEED=10,FUEL=74,GEAR=D,ODOMETER=12345.7,BLINK=LEFT,HI_BEAM=false,CHK_ENG=false,OIL=false,BATT=false",
  "SPEED=20,FUEL=73,GEAR=D,ODOMETER=12345.8,BLINK=LEFT,HI_BEAM=false,CHK_ENG=false,OIL=false,BATT=false",
  "SPEED=30,FUEL=72,GEAR=D,ODOMETER=12345.9,BLINK=NONE,HI_BEAM=true,CHK_ENG=false,OIL=false,BATT=false",
  "SPEED=50,FUEL=71,GEAR=D,ODOMETER=12346.0,BLINK=NONE,HI_BEAM=true,CHK_ENG=true,BATT=true,OIL=false",
  "SPEED=70,FUEL=70,GEAR=D,ODOMETER=12346.1,BLINK=NONE,HI_BEAM=false,CHK_ENG=true,BATT=true,OIL=true",
  "SPEED=90,FUEL=69,GEAR=D,ODOMETER=12346.2,BLINK=RIGHT,HI_BEAM=false,CHK_ENG=false,BATT=false,OIL=true",
  "SPEED=110,FUEL=68,GEAR=D,ODOMETER=12346.3,BLINK=RIGHT,HI_BEAM=false,CHK_ENG=false,BATT=false,OIL=false",
  "SPEED=130,FUEL=67,GEAR=D,ODOMETER=12346.4,BLINK=NONE,HI_BEAM=false,CHK_ENG=false,BATT=false,OIL=false"
};
int numPackets = sizeof(samplePackets) / sizeof(samplePackets[0]);


void setup() {
  Serial.begin(115200);
  
  // Connect to Wifi network.
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(F("."));
  }
  
  Serial.println("\nWiFi connected.");
  
  // Start the UDP server
  udp.begin(localPort);
  Serial.printf("UDP server started. Listening on IP %s, Port %i\n", WiFi.localIP().toString().c_str(), localPort);
  Serial.println("Waiting for a packet from the dashboard client...");
}

void loop() {
  
  // --- 1. Check for an incoming packet ---
  // This is how the client "registers" itself
  int packetSize = udp.parsePacket();
  
  if (packetSize) {
    // Read the packet (we don't care what's in it, just who it's from)
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }

    // Save the client's IP and port
    clientIP = udp.remoteIP();
    clientPort = udp.remotePort();
    clientConnected = true;

    Serial.println("------------------------------------");
    Serial.print("Dashboard client connected from: ");
    Serial.print(clientIP);
    Serial.print(", Port: ");
    Serial.println(clientPort);
    Serial.printf("Received packet data: %s\n", packetBuffer);
    Serial.println("Starting data stream...");
    Serial.println("------------------------------------");

    // Optional: Send a confirmation packet back
    udp.beginPacket(clientIP, clientPort);
    udp.print("Connection OK. Starting stream...");
    udp.endPacket();
  }

  // --- 2. Stream data if a client is connected ---
  // This part runs continuously without blocking
  if (clientConnected && (millis() - lastSendTime > sendInterval)) {
    lastSendTime = millis(); // Reset the timer

    // Get the next packet from our sample array
    const char* packetToSend = samplePackets[currentPacketIndex];

    // Send the packet to the registered client
    udp.beginPacket(clientIP, clientPort);
    udp.print(packetToSend);
    udp.endPacket();

    // Print to Serial monitor for debugging
    Serial.print("Sent packet ");
    Serial.print(currentPacketIndex);
    Serial.print(": ");
    Serial.println(packetToSend);

    // Move to the next packet, looping back to 0
    currentPacketIndex++;
    if (currentPacketIndex >= numPackets) {
      currentPacketIndex = 0;
    }
  }
  
  // No delay() here, so the loop is fast and responsive
}