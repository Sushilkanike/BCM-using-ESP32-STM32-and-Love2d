#include <WiFi.h>
#include <WiFiUdp.h>
#include <IPAddress.h>

#include <mcp_can.h>
#include <SPI.h>

// ---------- WiFi Setup ----------
const char *ssid = "Sushil's Phone : )";
const char *password = "8712308542";
unsigned int localPort = 8888;

WiFiUDP udp;
IPAddress clientIP;
uint16_t clientPort;
bool clientConnected = false;


// ---------- CAN Setup ----------
#define CAN_CS 14
#define CAN0_INT 4
MCP_CAN CAN0(CAN_CS);

// Latest Values (Saved for Streaming)
volatile uint8_t latestPOT = 0;
volatile uint8_t latestBTN = 0;
volatile float latestAX = 0;
volatile float latestAY = 0;
volatile float latestAZ = 0;

// ---------- Streaming Timing ----------
unsigned long lastSendTime = 0;
const long sendInterval = 100; // 100ms = 10 Hz


// Convert button byte to 6-bit binary string
String buttonsToBinary6(uint8_t b) {
  String s = "";
  for (int i = 5; i >= 0; i--)
    s += ((b >> i) & 1) ? '1' : '0';
  return s;
}


// Create the final single output string
String formatDataString() {
  // String out = "POT=" + String(latestPOT);
  // out += ",BTN=" + buttonsToBinary6(latestBTN);
  // out += ",AX=" + String(latestAX, 2);
  // out += ",AY=" + String(latestAY, 2);
  // out += ",AZ=" + String(latestAZ, 2);
  // return out;

  //return format_for_BCM();
  return format_for_FCM();

}


static void setFilters() {
  CAN0.init_Mask(0, 0, 0x7FF);
  CAN0.init_Filt(0, 0, 0x100);
  CAN0.init_Filt(1, 0, 0x101);

  CAN0.init_Mask(1, 0, 0x7FF);
  CAN0.init_Filt(2, 0, 0x100);
  CAN0.init_Filt(3, 0, 0x101);
  CAN0.init_Filt(4, 0, 0x100);
  CAN0.init_Filt(5, 0, 0x101);
}

float speed_mps = 0;             // running integrated speed (m/s)
unsigned long lastCalcTime = 0;  // timestamp for DT

float altitude = 0;
float last_altitude = 0;
unsigned long lastAltTime = 0;
float heading = 0;   // placeholder until IMU yaw added

void setup() {
  Serial.begin(115200);

  // ---- WiFi ----
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected.");

  udp.begin(localPort);
  Serial.printf("UDP Ready. IP: %s Port: %d\n", WiFi.localIP().toString().c_str(), localPort);

  // ---- CAN ----
  SPI.begin(18,19,23,CAN_CS);
  pinMode(CAN0_INT, INPUT);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 OK");
  else Serial.println("MCP2515 FAIL");

  setFilters();
  CAN0.setMode(MCP_NORMAL);
  Serial.println("Listening CAN...");
}


void loop() {

  // -------- Check for UDP Client --------
  int packetSize = udp.parsePacket();
  if (packetSize) {
    udp.flush();
    clientIP = udp.remoteIP();
    clientPort = udp.remotePort();
    clientConnected = true;

    udp.beginPacket(clientIP, clientPort);
    udp.print("Connection OK. Streaming CAN Data...");
    udp.endPacket();

    Serial.printf("Client connected: %s:%d\n", clientIP.toString().c_str(), clientPort);
  }


  // -------- Read CAN Frames --------
  while (!digitalRead(CAN0_INT)) {
    long unsigned int id;
    byte len;
    byte buf[8];
    CAN0.readMsgBuf(&id, &len, buf);

    unsigned long sid = id & 0x7FF;

    if (sid == 0x100 && len == 2) {
      latestPOT = buf[0];
      latestBTN = buf[1];
    }
    else if (sid == 0x101 && len == 6) {
      latestAX = ((int16_t)(buf[0] << 8 | buf[1])) / 100.0f;
      latestAY = ((int16_t)(buf[2] << 8 | buf[3])) / 100.0f;
      latestAZ = ((int16_t)(buf[4] << 8 | buf[5])) / 100.0f;
    }
  }


  // -------- Stream to Client --------
  if (clientConnected && (millis() - lastSendTime >= sendInterval)) {
    lastSendTime = millis();

    String out = formatDataString();  // formatted string here

    udp.beginPacket(clientIP, clientPort);
    udp.print(out);
    udp.endPacket();

    //Serial.println(out);
  }
}

String format_for_BCM() {

  // --- Invert POT (because wired backward) ---
  uint8_t invPOT = 255 - latestPOT;
  int fuel = map(invPOT, 0, 255, 0, 100);

  // --- Compute SPEED from accelerometer (km/h) ---
  unsigned long now = millis();
  float dt = (now - lastCalcTime) / 1000.0f; // seconds
  lastCalcTime = now;

  // Horizontal acceleration magnitude (m/s²)
  float ax_mps2 = latestAX * 9.81;   // convert g → m/s²
  float ay_mps2 = latestAY * 9.81;

  float accel = sqrt(ax_mps2*ax_mps2 + ay_mps2*ay_mps2);

  // Integrate → speed (m/s)
  speed_mps += accel * dt;

  // Add damping so speed decreases when accel is small
  speed_mps *= 0.98; // tweak to feel right

  if (speed_mps < 0) speed_mps = 0; // prevent negative
  float speed_kmph = speed_mps * 3.6;


  // --- Determine GEAR (simple logic) ---
  String gear;
  if (speed_kmph < 1.0) gear = "P";
  else gear = "D";


  // --- Decode Blink & Indicators ---
  bool leftBlink  = (latestBTN & 0b000001) != 0;
  bool rightBlink = (latestBTN & 0b000010) != 0;
  String blink = (leftBlink && !rightBlink) ? "LEFT" :
                 (rightBlink && !leftBlink) ? "RIGHT" :
                 "NONE";

  bool hi_beam  = (latestBTN & 0b000100) != 0;
  bool chk_eng  = (latestBTN & 0b001000) != 0;
  bool batt      = (latestBTN & 0b010000) != 0;
  bool oil     = (latestBTN & 0b100000) != 0;


  // --- Final formatted message ---
  String out;
  out += "SPEED=";    out += String(speed_kmph, 1);
  out += ",FUEL=";    out += fuel;
  out += ",GEAR=";    out += gear;
  out += ",ODOMETER=0.0"; // ignored / placeholder
  out += ",BLINK=";   out += blink;
  out += ",HI_BEAM="; out += (hi_beam ? "true" : "false");
  out += ",CHK_ENG="; out += (chk_eng ? "true" : "false");
  out += ",OIL=";     out += (oil     ? "true" : "false");
  out += ",BATT=";    out += (batt    ? "true" : "false");

  return out;
}

String format_for_FCM() {

  // --- Time delta for integration ---
  unsigned long now = millis();
  float dt = (now - lastAltTime) / 1000.0f;
  lastAltTime = now;

  // --- Throttle ---
  uint8_t invPOT = 255 - latestPOT;
  int throttle = map(invPOT, 0, 255, 0, 100);

  // --- Airspeed already computed in BCM ---
  float airspeed = speed_mps * 3.6;  // km/h

  // --- Convert accel to flight attitude (rough placeholder) ---
  float roll_deg  = latestAX * 30.0f;   // tune scaling later
  float pitch_deg = latestAY * 30.0f;

  // --- Altitude integrate from Z accel ---
  float z_accel_mps2 = latestAZ * 9.81;
  altitude += z_accel_mps2 * dt;   // extremely rough, we’ll stabilize later

  // --- Vertical speed ---
  float VSI = (altitude - last_altitude) / dt;
  last_altitude = altitude;

  // --- Heading placeholder (until yaw added) ---
  float HDG = heading;

  // --- Blink Buttons ---
  String blink_str = "OFF";
  bool leftBlink  = (latestBTN & 0b000001);
  bool rightBlink = (latestBTN & 0b000010);
  if (leftBlink && !rightBlink) blink_str = "LEFT";
  else if (rightBlink && !leftBlink) blink_str = "RIGHT";

  // --- Other Buttons ---
  bool flaps = (latestBTN & 0b000100) != 0;
  bool nav_lights = (latestBTN & 0b001000) != 0;
  bool stall_warn = (latestBTN & 0b010000) != 0;
  bool engine_start = (latestBTN & 0b100000) != 0;

  // --- Format Output String ---
  String out = "";
  out += "SPEED=";   out += String(airspeed, 1);
  out += ",THROTTLE=";    out += throttle;
  out += ",ROLL=";    out += String(roll_deg, 1);
  out += ",PITCH=";   out += String(pitch_deg, 1);
  out += ",ALT=";     out += String(altitude, 1);
  out += ",HDG=";     out += String(HDG, 1);
  out += ",VSI=";     out += String(VSI, 1);
  out += ",Z_ACCEL="; out += String(latestAZ, 3);

  out += ",BLINK=";   out += blink_str;
  out += ",FLAPS="; out += (flaps ? "true" : "false");
  out += ",NAV_LIGHT="; out += (nav_lights ? "true" : "false");
  out += ",STALL_WRN=";    out += (stall_warn ? "true" : "false");
  out += ",ENG_STRT=";    out += (engine_start ? "true" : "false");

  return out;
}
