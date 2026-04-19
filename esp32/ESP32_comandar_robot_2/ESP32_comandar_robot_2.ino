#include <WiFi.h>
#include <WiFiUdp.h>

// =======================
// WIFI 
// =======================

const char* WIFI_SSID = "LAB ROBOTICA";
const char* WIFI_PASS = "robotica2021";


// =======================
// ROBOT ID 
// =======================
static const int ROBOT_ID = 2;   // 1, 2, 3

// =======================
// PUERTOS UDP
// =======================
static const uint16_t CMD_PORT       = 44444; 
static const uint16_t DISCOVERY_PORT = 37030;  
static const uint32_t COMMAND_TIMEOUT_MS = 500;

// =======================
// Pines TB6612FNG 
// =======================
#define STBY_PIN   13


#define IN1A_PIN   25  
#define IN2A_PIN   33  
#define PWMA_PIN   32  


#define IN1B_PIN   26  
#define IN2B_PIN   27  
#define PWMB_PIN   14  


// LEDs
#define LED1_PIN   16
#define LED2_PIN    5

// START externo 
#define START_IN_PIN 15   
#define USE_START_GATE 0

// Inversión motores
#define MOTOR_A_INVERT 0    
#define MOTOR_B_INVERT 1      

// =======================
// PWM 
// =======================
const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 10;
const int      DUTY_MAX = (1 << PWM_RES) - 1;

const int RAMP_STEP  = 12;
const int RAMP_DELAY = 2;

int targetA = 0, currentA = 0;
int targetB = 0, currentB = 0;

// =======================

// Escape simple
const uint16_t ESC_BACK_MS = 140;
const int      ESC_BACK_PCT = 60;
const uint16_t ESC_TURN_MS = 220;
const int      ESC_TURN_PCT = 55;

// =======================
// UDP
// =======================
WiFiUDP udpCmd;
WiFiUDP udpDisc;
char packetBuffer[256];

uint32_t lastCmdMs = 0;
int lastL = 0, lastR = 0;

// ---------------- Utilidades ----------------
inline int clampi(int v, int lo, int hi){ return (v < lo) ? lo : (v > hi) ? hi : v; }
inline int pct2duty(int p){ return clampi((p * DUTY_MAX) / 100, -DUTY_MAX, DUTY_MAX); }

void motorWriteSigned(int in1, int in2, int pwmPin, int dutySigned){
  dutySigned = clampi(dutySigned, -DUTY_MAX, DUTY_MAX);

  if (dutySigned == 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmPin, 0);
    return;
  }

  if (dutySigned > 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmPin, (uint32_t)dutySigned);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmPin, (uint32_t)(-dutySigned));
  }
}



void setMotorsPct(int leftPct, int rightPct){
  float factorL = 0.95; 
  float factorR = 0.34;
  
  
  leftPct  = (int)(leftPct  * factorL);
  rightPct = (int)(rightPct * factorR);


  leftPct  = clampi(leftPct, -100, 100);
  rightPct = clampi(rightPct, -100, 100);
 

  lastL = leftPct; lastR = rightPct;

  if (MOTOR_A_INVERT) leftPct  = -leftPct;
  if (MOTOR_B_INVERT) rightPct = -rightPct;

  targetA = pct2duty(leftPct);
  targetB = pct2duty(rightPct);
}




void stepRamp(){
  if (currentA < targetA) currentA = min(currentA + RAMP_STEP, targetA);
  else if (currentA > targetA) currentA = max(currentA - RAMP_STEP, targetA);

  if (currentB < targetB) currentB = min(currentB + RAMP_STEP, targetB);
  else if (currentB > targetB) currentB = max(currentB - RAMP_STEP, targetB);

  motorWriteSigned(IN1A_PIN, IN2A_PIN, PWMA_PIN, currentA);
  motorWriteSigned(IN1B_PIN, IN2B_PIN, PWMB_PIN, currentB);
}



bool lineaBlancaHys(){
}

void brakeActive(uint16_t ms){
  digitalWrite(IN1A_PIN, HIGH); digitalWrite(IN2A_PIN, HIGH);
  digitalWrite(IN1B_PIN, HIGH); digitalWrite(IN2B_PIN, HIGH);
  ledcWrite(PWMA_PIN, DUTY_MAX);
  ledcWrite(PWMB_PIN, DUTY_MAX);
  delay(ms);
  ledcWrite(PWMA_PIN, 0);
  ledcWrite(PWMB_PIN, 0);
}

void wifiConnect(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("[WIFI] Conectando");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED){
    delay(300);
    Serial.print(".");
    if (millis() - t0 > 20000){
      Serial.println("\n[WIFI] Timeout, reiniciando...");
      ESP.restart();
    }
  }
  Serial.println("\n[WIFI] OK");
  Serial.print("[WIFI] IP: "); Serial.println(WiFi.localIP());
}

void discoveryRespond(IPAddress rip, uint16_t rport){
  char reply[64];
  snprintf(reply, sizeof(reply), "ROBOT_HERE ID=%d CMDPORT=%u", ROBOT_ID, CMD_PORT);
  udpDisc.beginPacket(rip, rport);
  udpDisc.print(reply);
  udpDisc.endPacket();
}

void discoveryAnnounce(){
  char reply[64];
  snprintf(reply, sizeof(reply), "ROBOT_HERE ID=%d CMDPORT=%u", ROBOT_ID, CMD_PORT);
  udpDisc.beginPacket(IPAddress(255,255,255,255), DISCOVERY_PORT);
  udpDisc.print(reply);
  udpDisc.endPacket();
}

void setup(){
  Serial.begin(115200);

  pinMode(STBY_PIN, OUTPUT);
  pinMode(IN1A_PIN, OUTPUT); pinMode(IN2A_PIN, OUTPUT);
  pinMode(IN1B_PIN, OUTPUT); pinMode(IN2B_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT); pinMode(LED2_PIN, OUTPUT);

#if USE_START_GATE
  pinMode(START_IN_PIN, INPUT); 
#endif

  digitalWrite(STBY_PIN, HIGH);
  bool okA = ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RES);
  bool okB = ledcAttach(PWMB_PIN, PWM_FREQ, PWM_RES);
  if (!okA || !okB){
    Serial.println("[LEDC] Error: no se pudo adjuntar PWM a pines. Revisa pines/placa.");
    while(true){ delay(1000); }
  }


  wifiConnect();

  udpCmd.begin(CMD_PORT);
  udpDisc.begin(DISCOVERY_PORT);

  Serial.printf("[UDP] CMD_PORT=%u  DISCOVERY_PORT=%u  ROBOT_ID=%d\n", CMD_PORT, DISCOVERY_PORT, ROBOT_ID);

  setMotorsPct(0,0);
  lastCmdMs = millis();
}

void loop(){
#if USE_START_GATE
  if (digitalRead(START_IN_PIN) == LOW){
    setMotorsPct(0,0);
    stepRamp();

    // seguir respondiendo discovery aunque esté "parado"
    int ps = udpDisc.parsePacket();
    if (ps){
      int len = udpDisc.read(packetBuffer, sizeof(packetBuffer)-1);
      if (len > 0) packetBuffer[len] = 0;
      if (strcmp(packetBuffer, "DISCOVER_ROBOTS") == 0){
        discoveryRespond(udpDisc.remoteIP(), udpDisc.remotePort());
      }
    }
    delay(10);
    return;
  }
#endif

  // 1) Discovery
  int p2 = udpDisc.parsePacket();
  if (p2){
    int len2 = udpDisc.read(packetBuffer, sizeof(packetBuffer)-1);
    if (len2 > 0) packetBuffer[len2] = 0;
    if (strcmp(packetBuffer, "DISCOVER_ROBOTS") == 0){
      discoveryRespond(udpDisc.remoteIP(), udpDisc.remotePort());
    }
  }

  // 2) Anuncio periódico (resolver IP robusto)
  static uint32_t lastAnn = 0;
  if (millis() - lastAnn > 900){
    discoveryAnnounce();
    lastAnn = millis();
  }


  // 4) CMD UDP: "M L R"
  int p = udpCmd.parsePacket();
  if (p){
    int len = udpCmd.read(packetBuffer, sizeof(packetBuffer)-1);
    if (len > 0) packetBuffer[len] = 0;

    if (packetBuffer[0] == 'M'){
      int l, r;
      if (sscanf(packetBuffer, "M %d %d", &l, &r) == 2){
        setMotorsPct(l, r);
        lastCmdMs = millis();
      }
    } else if (strncmp(packetBuffer, "STOP", 4) == 0){
      setMotorsPct(0,0);
      lastCmdMs = millis();
    }
  }

  // 5) Timeout
  if (millis() - lastCmdMs > COMMAND_TIMEOUT_MS){
    setMotorsPct(0,0);
  }

  // 6) Aplicar rampa
  stepRamp();
  delay(RAMP_DELAY);
}
