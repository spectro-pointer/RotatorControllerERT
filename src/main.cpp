#include <Arduino.h>
#include <capsule.h>
#include <Bounce2.h>
#include "stepper.h"
#include "config.h"
#include "filters.h"
#include <SD.h>
#include <NativeEthernet.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <math.h>

static TrackerParameter globalParameter;
// ========================= PoV CONFIG =========================
// MOD: destinos UNICAST (ajusta a tu(s) PC(s) receptores). 0.0.0.0 = deshabilitado
static IPAddress trackletTarget1(0,0,0,0);
static IPAddress trackletTarget2(0,0,0,0);

// MOD: periodo y gating para tracklets (si algún día reactivás emitTracklet())
static const unsigned long trackletPeriodMs = 100;   // 10 Hz
static const float         trackletDeltaMinDeg = 0.02f;
static const unsigned long trackletHeartbeatMs = 1000; // 1 Hz

// MOD: metadatos PoV mínimos
static const char* SITE_ID       = "ARG-TDF-01";
static const char* INSTRUMENT_ID = "N300_QHY4040_A";
static uint32_t    EXP_MS        = 30;
static float       SEEING_ARCSEC = 1.8f;

// ========================= Tiempo UTC =========================
static int64_t g_utcOffsetMs = 0; // ajuste respecto de millis() para alinear a UTC real
unsigned long getUtcMillis() { return millis() + (unsigned long)(g_utcOffsetMs); }

String iso8601_from_utc_ms(unsigned long utc_ms) {
  time_t t = utc_ms / 1000;
  uint16_t ms = utc_ms % 1000;
  tm *ut = gmtime(&t);
  char buf[40];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03uZ",
    ut->tm_year + 1900, ut->tm_mon + 1, ut->tm_mday,
    ut->tm_hour, ut->tm_min, ut->tm_sec, (unsigned)ms);
  return String(buf);
}

// ========================= AltAz -> RA/Dec =========================
static inline double deg2rad(double d){ return d * 0.017453292519943295; }
static inline double rad2deg(double r){ return r * 57.29577951308232;   }

double jd_from_unix_ms(unsigned long utc_ms) {
  return 2440587.5 + (double)utc_ms / 86400000.0;
}
double gmst_rad_from_jd(double jd){
  double T = (jd - 2451545.0)/36525.0;
  double gmst = 280.46061837 + 360.98564736629*(jd-2451545.0) + 0.000387933*T*T - T*T*T/38710000.0;
  gmst = fmod(gmst, 360.0); if (gmst<0) gmst += 360.0;
  return deg2rad(gmst);
}

// Convención usada: Az=0° Norte, 90° Este; El=0° horizonte.
void altaz_to_radec(double az_deg, double el_deg, double lat_deg, double lon_deg,
                    unsigned long utc_ms, double &ra_deg, double &dec_deg) {
  double az = deg2rad(az_deg);
  double alt= deg2rad(el_deg);
  double lat= deg2rad(lat_deg);
  double lon= deg2rad(lon_deg);

  double sinDec = sin(lat)*sin(alt) + cos(lat)*cos(alt)*cos(az);
  double dec = asin(sinDec);
  double cosH = (sin(alt) - sin(lat)*sinDec) / (cos(lat)*cos(dec));
  if (cosH >  1.0) cosH = 1.0;
  if (cosH < -1.0) cosH = -1.0;
  double sinH = -sin(az)*cos(alt)/cos(dec);
  double H = atan2(sinH, cosH);

  double jd = jd_from_unix_ms(utc_ms);
  double gmst = gmst_rad_from_jd(jd);
  double lst = gmst + lon; // lon >0 Este
  double ra = lst - H;

  while (ra < 0)        ra += 2*M_PI;
  while (ra >= 2*M_PI)  ra -= 2*M_PI;

  ra_deg  = rad2deg(ra);
  dec_deg = rad2deg(dec);
}

bool joystickAvailable = false;
bool cameraAvailable = false;

unsigned long lastJoystickFrame = 0;
unsigned long lastCameraFrame = 0;

float azmManualPosition = 0;
float elvManualPosition = 0;

Adafruit_MMA8451 mma = Adafruit_MMA8451();

// MAC address for Teensy 1
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 100);

EthernetUDP udp;
EthernetServer server(80);

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len);
CapsuleStatic command(handleCommand);

void loopAutomatic();
void loopManual();

static conClass control;
unsigned long lastCmdTime = 0;

double internalAzmNorthEstimate = 0;
double internalElvHorizonEstimate = 0;

Bounce azmEndstop = Bounce();
Bounce elvEndstop = Bounce();

SecondOrderEstimator azmEstimator;
SecondOrderEstimator elvEstimator;

SecondOrderLowPassFilter azmFilter(0.1, 100);
SecondOrderLowPassFilter elvFilter(0.1, 100);

SecondOrderEstimator latEstimator;
SecondOrderEstimator lonEstimator;
SecondOrderEstimator altEstimator;

SecondOrderLowPassFilter latFilter(0.1, 100);
SecondOrderLowPassFilter lonFilter(0.1, 100);
SecondOrderLowPassFilter altFilter(0.1, 100);

void doHoming();
void printPosition();

// =====================================================================
// MOD: Telemetría extendida (paquete 34) — ωcmd/ωmeas + modo + flags
// =====================================================================
#pragma pack(push,1)
struct StatusPacket {
  float v_cmd_az;   // deg/s comandado
  float v_cmd_el;   // deg/s comandado
  float v_meas_az;  // deg/s medido (derivada)
  float v_meas_el;  // deg/s medido
  uint8_t mode;     // TRACKING_MODE actual (cast a uint8)
  uint8_t flags;    // bit0: cameraAvailable, bit1: joystickAvailable
};
#pragma pack(pop)

void sendStatusTelemetry(double v_cmd_az, double v_cmd_el) {
  static unsigned long lastSend = 0;
  const unsigned long periodMs = 100; // 10 Hz
  if (millis() - lastSend < periodMs) return;
  lastSend = millis();

  // Medición de velocidad a partir de la posición real
  static long lastPosAzStep = 0;
  static long lastPosElStep = 0;
  static unsigned long lastT = 0;

  long posAzStep = control.stepperAzm.currentPosition();
  long posElStep = control.stepperElv.currentPosition();
  unsigned long nowT = millis();

  double v_meas_az = 0.0;
  double v_meas_el = 0.0;

  if (lastT != 0) {
    double dt = (nowT - lastT) / 1000.0;
    if (dt < 1e-6) dt = 1e-6;

    double azNow  = control.stepToDegAzm(posAzStep);
    double azPrev = control.stepToDegAzm(lastPosAzStep);
    // wrap ΔAz a [-180,+180]
    double dAz = fmod(azNow - azPrev + 540.0, 360.0) - 180.0;
    double dEl = control.stepToDegElv(posElStep) - control.stepToDegElv(lastPosElStep);

    v_meas_az = dAz / dt;
    v_meas_el = dEl / dt;
  }

  lastPosAzStep = posAzStep;
  lastPosElStep = posElStep;
  lastT = nowT;

  StatusPacket pkt;
  pkt.v_cmd_az  = (float)v_cmd_az;
  pkt.v_cmd_el  = (float)v_cmd_el;
  pkt.v_meas_az = (float)v_meas_az;
  pkt.v_meas_el = (float)v_meas_el;
  pkt.mode      = (uint8_t)control.getMode();
  pkt.flags     = (cameraAvailable ? 0x01 : 0x00) | (joystickAvailable ? 0x02 : 0x00);

  // Mismos destinos que printPosition()
  static const IPAddress targets[] = {
    IPAddress(192,168,1,220),
    IPAddress(0,0,0,0)
  };
  static const size_t NUM_TARGETS = sizeof(targets)/sizeof(targets[0]);

  uint8_t* enc = command.encode(34, (uint8_t*)&pkt, sizeof(pkt)); // MOD: nuevo ID 34
  bool any = false; for (size_t i=0;i<NUM_TARGETS;i++) if (targets[i]!=IPAddress(0,0,0,0)) any=true;

  if (any) {
    for (size_t i=0;i<NUM_TARGETS;i++) {
      if (targets[i]==IPAddress(0,0,0,0)) continue;
      udp.beginPacket(targets[i], (unsigned int)globalParameter.ethernetParameter.port);
      udp.write(enc, command.getCodedLen(sizeof(pkt)));
      udp.endPacket();
    }
  } else {
    IPAddress bcast(255,255,255,255);
    udp.beginPacket(bcast, (unsigned int)globalParameter.ethernetParameter.port);
    udp.write(enc, command.getCodedLen(sizeof(pkt)));
    udp.endPacket();
  }
  delete[] enc;
}
// =====================================================================
// FIN telemetría extendida
// =====================================================================

// (Dejamos el prototipo de tracklet deshabilitado en el Teensy)
#if 0
void emitTracklet();
#endif




String readHTML();
void applyConfig();
void readConfigFile();
void saveConfigFile();
void setupWebserver();
void loopWebserver();
double distanceTo(double lat1, double lng1, double lat2, double lng2);
double azimuthTo(double lat1, double lng1, double lat2, double lng2);
AnglePacket computeAngle(double lat1, double lng1, double alt1, double lat2, double lng2, double alt2);

void setup() {  
  CMD_PORT.begin(CMD_BAUD);
  SERIAL_TO_PC.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  #ifdef AZM_ENCODER_PIN
    pinMode(AZM_ENCODER_PIN, INPUT);
  #endif

  #ifdef AZM_ENDSTOP_PIN
    azmEndstop.attach(AZM_ENDSTOP_PIN ,  INPUT); 
    azmEndstop.interval(25);
  #endif

  #ifdef ELV_ENDSTOP_PIN
    elvEndstop.attach(ELV_ENDSTOP_PIN ,  INPUT);
    elvEndstop.interval(25);
  #endif

  Ethernet.begin(mac, ip);
  udp.begin(8888);
  server.begin();

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Error initializing SD card");
    while (1);
  }

  readConfigFile();
  applyConfig();

  // doHoming();  // MOD: seguimos desactivado al arranque
  Serial.println("[setup] Homing desactivado en arranque (solo a demanda)");

  PositionPacket groundPosition;
  groundPosition.lat = 46.532308;
  groundPosition.lon = 6.590961;
  groundPosition.alt = 422;
  control.setGroundPosition(groundPosition);
}

void loop() {
  loopAutomatic();
  printPosition();         // id=33 (Az/El)
  // emitTracklet();       // MOD: seguimos sin usar en Teensy
  loopWebserver();
}

void loopAutomatic() {
  double speedAzm = 0;
  double speedElv = 0;

  int packetSize = udp.parsePacket();
  if (packetSize) {
    for (int i = 0; i < packetSize; i++) {
      command.decode(udp.read());
    }
  }

  static TRACKING_MODE lastMode = TRACKING_MODE::TRACKING_STOP;

  if (lastMode != control.getMode()) {
    Serial.print("Mode changed to ");
    Serial.println(control.getMode());
    lastMode = control.getMode();
  }

  switch(control.getMode()) {

    case TRACKING_MODE::TRACKING_MANUAL:
      if (millis()-control.lastManualTime<300) {
        azmManualPosition  = control.lastAngle.azm;
        elvManualPosition  = control.lastAngle.elv;

        azmManualPosition = constrain(azmManualPosition, globalParameter.azmParameter.limMin, globalParameter.azmParameter.limMax);
        elvManualPosition = constrain(elvManualPosition, globalParameter.elvParameter.limMin, globalParameter.elvParameter.limMax);

        control.stepperAzm.moveTo(azmManualPosition);
        control.stepperElv.moveTo(elvManualPosition);

        // Nota: en manual, AccelStepper reporta velocidad en pasos/s; mantenemos ωcmd a 0 aquí.
        speedAzm = control.stepperAzm.speed();
        speedElv = control.stepperElv.speed();
        // Enviamos ωcmd=0 en este modo (la Pi ya estima ωmeas).
        speedAzm = 0; 
        speedElv = 0;
      }
    break;

    case TRACKING_MODE::TRACKING_POSITION:
    {
      static unsigned long lastEstimationTime = millis();
      if ((millis()-lastEstimationTime) >= 1000.0/ESTIMATION_RATE) {
        lastEstimationTime = millis();

        double azmEstimation = azmEstimator.computeAngle(millis());
        double elvEstimation = elvEstimator.computeAngle(millis());

        double azmFiltered = azmEstimation;
        double elvFiltered = elvEstimation;

        PacketTrackerCmd newCmd;
        newCmd.azm = constrain(azmFiltered, globalParameter.azmParameter.limMin, globalParameter.azmParameter.limMax);
        newCmd.elv = constrain(elvFiltered, globalParameter.elvParameter.limMin, globalParameter.elvParameter.limMax);
        control.update(newCmd);
      }
      controlOutput output = control.computeOutputSpeedPosition();
      speedAzm = output.azmSpeed;  // deg/s comandados por posición
      speedElv = output.elvSpeed;
    }
    break;

    case TRACKING_MODE::TRACKING_ERROR:
      // MOD: aquí el lazo de la cámara/joystick define ωcmd
      speedAzm = control.outputSpeedError.azmSpeed; // deg/s
      speedElv = control.outputSpeedError.elvSpeed; // deg/s
    break;

    case TRACKING_MODE::TRACKING_STOP:
      speedAzm = 0;
      speedElv = 0;
    break;  

    case TRACKING_MODE::TRACKING_POINTER:         // <<< MOD: nuevo caso
    // lo tratamos como seguimiento por error (cámara/joystick)
     speedAzm = control.outputSpeedError.azmSpeed;
     speedElv = control.outputSpeedError.elvSpeed;
    // aplica límites, setSpeed, runSpeed como haces en otros casos o déjalo vacío si
    // ya usas las mismas variables fuera del switch
   break;
  }

  // Límites físicos
  if (globalParameter.azmParameter.limExist) {
    if (control.stepperAzm.currentPosition() < control.degToStepAzm(globalParameter.azmParameter.limMin)) {
      speedAzm = max(speedAzm, 0.0);
    }
    if (control.stepperAzm.currentPosition() > control.degToStepAzm(globalParameter.azmParameter.limMax)) {
      speedAzm = min(speedAzm, 0.0);
    }
  }
  if (globalParameter.elvParameter.limExist) {
    if (control.stepperElv.currentPosition() < control.degToStepElv(globalParameter.elvParameter.limMin)) {
      speedElv = max(speedElv, 0.0);
    }
    if (control.stepperElv.currentPosition() > control.degToStepElv(globalParameter.elvParameter.limMax)) {
      speedElv = min(speedElv, 0.0);
    }
  }

  control.stepperAzm.setSpeed(control.degToStepAzm(speedAzm));
  control.stepperElv.setSpeed(control.degToStepElv(speedElv));
  control.stepperAzm.runSpeed();
  control.stepperElv.runSpeed();

  // MOD: enviar telemetría extendida (ωcmd/ωmeas) a 10 Hz
  sendStatusTelemetry(speedAzm, speedElv);
}

void loopManual() {}

void printPosition() {
  static unsigned long lastPrint = 0;
  const unsigned long periodMs = 100;  // 10 Hz
  if (millis() - lastPrint < periodMs) return;
  lastPrint = millis();

  static const IPAddress targets[] = {
    IPAddress(192,168,1,220),
    IPAddress(0,0,0,0)
  };
  static const size_t NUM_TARGETS = sizeof(targets)/sizeof(targets[0]);

  AnglePacket angleToSend;
  angleToSend.azm = control.stepToDegAzm(control.stepperAzm.currentPosition());
  angleToSend.elv = control.stepToDegElv(control.stepperElv.currentPosition());

  static float lastAzm = -10000.0f, lastElv = -10000.0f;
  static unsigned long lastHeartbeat = 0;
  const float MIN_DELTA_DEG = 0.02f;
  const unsigned long HEARTBEAT_MS = 1000;

  const bool changed =
      (fabs(angleToSend.azm - lastAzm) > MIN_DELTA_DEG) ||
      (fabs(angleToSend.elv - lastElv) > MIN_DELTA_DEG);
  const bool heartbeat = (millis() - lastHeartbeat >= HEARTBEAT_MS);

  if (!changed && !heartbeat) return;
  if (heartbeat) lastHeartbeat = millis();
  lastAzm = angleToSend.azm;
  lastElv = angleToSend.elv;

  uint8_t packetData[sizeof(AnglePacket)];
  memcpy(packetData, &angleToSend, sizeof(AnglePacket));
  uint8_t* packetToSend = command.encode(33, packetData, sizeof(AnglePacket));

  bool anyTarget = false;
  for (size_t i = 0; i < NUM_TARGETS; ++i) {
    if (targets[i] != IPAddress(0,0,0,0)) { anyTarget = true; break; }
  }

  if (anyTarget) {
    for (size_t i = 0; i < NUM_TARGETS; ++i) {
      if (targets[i] == IPAddress(0,0,0,0)) continue;
      udp.beginPacket(targets[i], (unsigned int)globalParameter.ethernetParameter.port);
      udp.write(packetToSend, command.getCodedLen(sizeof(AnglePacket)));
      udp.endPacket();
    }
  } else {
    IPAddress ipBroadcast(255,255,255,255);
    udp.beginPacket(ipBroadcast, (unsigned int)globalParameter.ethernetParameter.port);
    udp.write(packetToSend, command.getCodedLen(sizeof(AnglePacket)));
    udp.endPacket();
  }

  delete[] packetToSend;
}

// ========================= (opcional) emitTracklet desactivado =========================
// Si quisieras reactivarlo, avísame y lo limpiamos para sólo UDP (sin SD).
void emitTracklet() { /* NO USAR: dejamos a la Pi generar JSONL */ }
// =======================================================================================

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch(packetId) {
    case 0x01: {
      control.lastCameraErrorTime = millis();
      memcpy(&control.lastCameraError, dataIn, sizeof(CameraErrorPacket));
      double posX = (control.lastCameraError.Cx)/100.0;
      double posY = (control.lastCameraError.Cy)/100.0;

      static unsigned long lastFoundTime = millis();

      if (control.lastCameraError.isVisible) {

        if (control.lastCameraError.camID == 33) {
          cameraAvailable = true;
          lastCameraFrame = millis();
        }
        else if (control.lastCameraError.camID == 99) {
          joystickAvailable = true;
          lastJoystickFrame = millis();
        }

        lastFoundTime = millis();

        bool shouldWeUsePacket = false;
        if (joystickAvailable && control.lastCameraError.camID == 99) {
          shouldWeUsePacket = true;
        }
        else if (!joystickAvailable && control.lastCameraError.camID == 33) {
          shouldWeUsePacket = true;
        }

        if (shouldWeUsePacket) {
          float KPx = control.lastCameraError.Kp;
          float KPy = control.lastCameraError.Kp;

          float maxSpeed = control.lastCameraError.maxSpeed;

          double controlX = KPx*posX;
          double controlY = KPy*posY;

          control.outputSpeedError.azmSpeed = constrain(controlX, -maxSpeed, maxSpeed);
          control.outputSpeedError.elvSpeed = constrain(controlY, -maxSpeed, maxSpeed);
        }

      } else {
        if (control.lastCameraError.camID == 99) {
          joystickAvailable = false;
        }
      }

      if ((millis()-lastFoundTime)>100) {
        control.setMode(TRACKING_MODE::TRACKING_STOP);
        posX = 0; posY = 0;
      } else {
        control.setMode(TRACKING_MODE::TRACKING_ERROR);
      }

      if ((millis()-lastCameraFrame)>50)  cameraAvailable = false;
      if ((millis()-lastJoystickFrame)>100) joystickAvailable = false;
    } break;

    case 0x02: {
      control.lastPositionTime = millis();
      memcpy(&control.lastPosition, dataIn, sizeof(PositionPacket));

      azmEstimator.update(control.lastAngle.azm,millis());
      elvEstimator.update(control.lastAngle.elv,millis());
      azmEstimator.setMaxTimeWindow(500);
      elvEstimator.setMaxTimeWindow(500);

      azmFilter.setCutoffFreq(globalParameter.control.freq);
      elvFilter.setCutoffFreq(globalParameter.control.freq);
    } break;

    case 0x03:
      control.lastAngleTime = millis();
      memcpy(&control.lastAngle, dataIn, sizeof(AnglePacket));
      Serial.println("Received ABS Packet!!");
      azmManualPosition = control.lastAngle.azm;
      elvManualPosition = control.lastAngle.elv;
    break;

    case 99:
      control.lastManualTime = millis();
      memcpy(&control.lastManual, dataIn, sizeof(ManualPacket));
      Serial.print("Manual cmd received  X: ");
      Serial.print(control.lastManual.joystickX);
      Serial.print(" Y: ");
      Serial.println(control.lastManual.joystickY);
    break;
  }
}

void doHoming() {
  if (globalParameter.elvParameter.limExist) {
  #ifdef DUMBO
    double avg = 0;
    if (mma.begin()) {
      for (int i = 0; i < 100; i++) {
        mma.read();
        double elv = atan2(mma.y, mma.z)*180.0/PI;
        avg += elv;
        delay(1);
      }
    }
    avg /= 100.0;
    float offset = 3;
    control.stepperElv.setCurrentPosition(control.degToStepElv(avg+offset));
    delay(1000);
    Serial.print("ELV: "); Serial.println(avg);
  #endif 

  #ifdef SPECTROBOT
    bool endstopReached = false;
    do {
      control.stepperElv.setSpeed(control.degToStepElv(ELV_HOMING_SPEED));
      control.stepperElv.runSpeed();
      elvEndstop.update();
      endstopReached = (elvEndstop.read()==ELV_ENDSTOP_ACTIVE_LEVEL);
      Serial.println("Waiting for ELV Endstop");
    } while (!endstopReached);
    Serial.println("ELV Endstop reached");
    control.stepperElv.setCurrentPosition(control.degToStepElv(ELV_ENDSTOP_POSITION));
  #endif
  } else {
    delay(1000);
    Serial.println("No ELV limit switch");
    control.stepperElv.setCurrentPosition(0);
  }

  if (globalParameter.azmParameter.limExist) {
    #ifdef DUMBO
      double avg = 0;
      for (int i = 0; i < 100; i++) { avg += analogRead(AZM_ENCODER_PIN); delay(1); }
      avg /= 100.0;
      double azmRead = map(avg,103,923, 0,360);
      azmRead = (int((azmRead + (360-200))*100.0)%36000)/100.0;
      if (azmRead>180) azmRead -= 360;
      control.stepperAzm.setCurrentPosition(control.degToStepAzm(azmRead));
      delay(1000);
      Serial.print("AZM: "); Serial.println(azmRead);
    #endif

    #ifdef SPECTROBOT
      bool endstopReached = false;
      do {
        control.stepperAzm.setSpeed(control.degToStepAzm(AZM_HOMING_SPEED));
        control.stepperAzm.runSpeed();
        azmEndstop.update();
        endstopReached = (azmEndstop.read()==AZM_ENDSTOP_ACTIVE_LEVEL);
        Serial.println("Waiting for AZM Endstop");
      } while (!endstopReached);
      Serial.println("AZM Endstop reached");
      control.stepperAzm.setCurrentPosition(control.degToStepAzm(AZM_ENDSTOP_POSITION));
    #endif
  } else {
    delay(1000);
    Serial.println("No AZM limit switch");
    control.stepperAzm.setCurrentPosition(0);
  }
}

void loopWebserver() {
  EthernetClient client = server.available();
  if (client) {
     Serial.println("new client");
    boolean currentLineIsBlank = true;
    String httpRequest = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        httpRequest += c;

        if (c == '\n' && currentLineIsBlank) {

          if (httpRequest.indexOf("GET /?") != -1) {
            int AmaxAccIndex = httpRequest.indexOf("AmaxAcc=");
            int AmaxSpeIndex = httpRequest.indexOf("&AmaxSpe=");
            int AstepRevIndex = httpRequest.indexOf("&AstepRev=");
            int AgearBoxIndex = httpRequest.indexOf("&AgearBox=");
            int AlimExistIndex = httpRequest.indexOf("&AlimExist=");
            int AlimMinIndex = httpRequest.indexOf("&AlimMin=");
            int AlimMaxIndex= httpRequest.indexOf("&AlimMax=");
            int AdirIndex = httpRequest.indexOf("&Adir=");
            int AbackIndex = httpRequest.indexOf("&Aback=");
            int EmaxAccIndex = httpRequest.indexOf("&EmaxAcc=");
            int EmaxSpeIndex = httpRequest.indexOf("&EmaxSpe=");
            int EstepRevIndex = httpRequest.indexOf("&EstepRev=");
            int EgearBoxIndex = httpRequest.indexOf("&EgearBox=");
            int ElimExistIndex = httpRequest.indexOf("&ElimExist=");
            int ElimMinIndex = httpRequest.indexOf("&ElimMin=");
            int ElimMaxIndex= httpRequest.indexOf("&ElimMax=");
            int EdirIndex = httpRequest.indexOf("&Edir=");
            int EbackIndex = httpRequest.indexOf("&Eback=");
            int portIndex = httpRequest.indexOf("&port=");
            int kpIndex = httpRequest.indexOf("&kp=");
            int kdIndex = httpRequest.indexOf("&kd=");
            int freqIndex = httpRequest.indexOf("&freq=");
            int maxTimeIndex = httpRequest.indexOf("&maxTime=");
            int modeIndex = httpRequest.indexOf("&mode=");

            if (AmaxAccIndex != -1 
                && AmaxSpeIndex != -1 
                && AstepRevIndex != -1 
                && AgearBoxIndex != -1 
                && AlimExistIndex != -1 
                && AlimMinIndex != -1
                && AlimMaxIndex != -1
                && AdirIndex != -1
                && AbackIndex != -1
                && EmaxAccIndex != -1
                && ElimMaxIndex != -1
                && EstepRevIndex != -1
                && EgearBoxIndex != -1
                && ElimExistIndex != -1
                && ElimMinIndex != -1
                && ElimMaxIndex != -1
                && EdirIndex != -1
                && EbackIndex != -1
                && portIndex !=-1
                && kpIndex != -1
                && kdIndex != -1
                && freqIndex !=-1
                && maxTimeIndex != -1
                && modeIndex != -1
                ) {
              globalParameter.azmParameter.maxAcc = httpRequest.substring(AmaxAccIndex + 8, AmaxSpeIndex).toFloat();
              globalParameter.azmParameter.maxSpe = httpRequest.substring(AmaxSpeIndex + 9, AstepRevIndex).toFloat();
              globalParameter.azmParameter.stepRev = httpRequest.substring(AstepRevIndex + 10, AgearBoxIndex).toFloat();
              globalParameter.azmParameter.gearBox = httpRequest.substring(AgearBoxIndex + 10, AlimExistIndex).toFloat();
              globalParameter.azmParameter.limExist = httpRequest.substring(AlimExistIndex + 11, AlimMinIndex).toFloat();
              globalParameter.azmParameter.limMin = httpRequest.substring(AlimMinIndex + 9, AlimMaxIndex).toFloat();
              globalParameter.azmParameter.limMax = httpRequest.substring(AlimMaxIndex + 9, AdirIndex).toFloat();
              globalParameter.azmParameter.dir = httpRequest.substring(AdirIndex + 6, AbackIndex).toFloat();
              globalParameter.azmParameter.back = httpRequest.substring(AbackIndex + 7, EmaxAccIndex).toFloat();

              globalParameter.elvParameter.maxAcc = httpRequest.substring(EmaxAccIndex + 9, EmaxSpeIndex).toFloat();
              globalParameter.elvParameter.maxSpe = httpRequest.substring(EmaxSpeIndex + 9, EstepRevIndex).toFloat();
              globalParameter.elvParameter.stepRev = httpRequest.substring(EstepRevIndex + 10, EgearBoxIndex).toFloat();
              globalParameter.elvParameter.gearBox = httpRequest.substring(EgearBoxIndex + 10, ElimExistIndex).toFloat();
              globalParameter.elvParameter.limExist = httpRequest.substring(ElimExistIndex + 11, ElimMinIndex).toFloat();
              globalParameter.elvParameter.limMin = httpRequest.substring(ElimMinIndex + 9, ElimMaxIndex).toFloat();
              globalParameter.elvParameter.limMax = httpRequest.substring(ElimMaxIndex + 9, EdirIndex).toFloat();
              globalParameter.elvParameter.dir = httpRequest.substring(EdirIndex + 6, EbackIndex).toFloat();
              globalParameter.elvParameter.back = httpRequest.substring(EbackIndex + 7, portIndex).toFloat();

              globalParameter.ethernetParameter.port = httpRequest.substring(portIndex + 6, kpIndex).toInt();
              globalParameter.control.kp = httpRequest.substring(kpIndex + 4, kdIndex).toFloat();
              globalParameter.control.kd = httpRequest.substring(kdIndex + 4, freqIndex).toFloat();
              globalParameter.control.freq = httpRequest.substring(freqIndex + 6, maxTimeIndex).toFloat();
              globalParameter.control.maxTime = httpRequest.substring(maxTimeIndex + 9, modeIndex).toFloat();
              globalParameter.mode = static_cast<TRACKING_MODE>(httpRequest.substring(modeIndex + 6).toInt());
              Serial.print("Received config with mode: ");
              Serial.println(globalParameter.mode);

              applyConfig();
              Serial.print("Config applied: ");
              Serial.println(globalParameter.control.maxTime);
              saveConfigFile();
            }
            else {
              Serial.println("Error parsing HTTP request");
            }
          }

          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");
          client.println();
          client.print(readHTML());
          break;
        }
        if (c == '\n') {
          currentLineIsBlank = true;
        } else if (c != '\r') {
          currentLineIsBlank = false;
        }
      }
    }
    client.stop();
  }
}

void readConfigFile() {
  delay(1000);
  File configFile = SD.open("/config.txt");
  if (configFile) {
    String line = configFile.readStringUntil('\n');
    configFile.close();

    int modeTmp = 0; // <<< MOD: temporal para el enum

    sscanf(line.c_str(),
      "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
      "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
      "%lf,%lf,%lf,%lf,%lf,%d",
      &globalParameter.azmParameter.maxAcc,
      &globalParameter.azmParameter.maxSpe,
      &globalParameter.azmParameter.stepRev,
      &globalParameter.azmParameter.gearBox,
      &globalParameter.azmParameter.limExist,
      &globalParameter.azmParameter.limMin,
      &globalParameter.azmParameter.limMax,
      &globalParameter.azmParameter.dir,
      &globalParameter.azmParameter.back,
      &globalParameter.elvParameter.maxAcc,
      &globalParameter.elvParameter.maxSpe,
      &globalParameter.elvParameter.stepRev,
      &globalParameter.elvParameter.gearBox,
      &globalParameter.elvParameter.limExist,
      &globalParameter.elvParameter.limMin,
      &globalParameter.elvParameter.limMax,
      &globalParameter.elvParameter.dir,
      &globalParameter.elvParameter.back,
      &globalParameter.ethernetParameter.port,
      &globalParameter.control.kp,
      &globalParameter.control.kd,
      &globalParameter.control.freq,
      &globalParameter.control.maxTime,
      &modeTmp // <<< MOD: aquí en lugar de &globalParameter.mode
    );
    Serial.println("Config file read");
    Serial.println(line);
    Serial.print("MaxAcc:"); Serial.println(globalParameter.azmParameter.maxAcc);
    Serial.print("MaxSpe:"); Serial.println(globalParameter.azmParameter.maxSpe);
    Serial.print("StepRev:"); Serial.println(globalParameter.azmParameter.stepRev);
    Serial.print("GearBox:"); Serial.println(globalParameter.azmParameter.gearBox);
    Serial.print("LimExist:"); Serial.println(globalParameter.azmParameter.limExist);
    Serial.print("LimMin:"); Serial.println(globalParameter.azmParameter.limMin);
    Serial.print("LimMax:"); Serial.println(globalParameter.azmParameter.limMax);
    Serial.print("Dir:"); Serial.println(globalParameter.azmParameter.dir);
    Serial.print("Back:"); Serial.println(globalParameter.azmParameter.back);
    Serial.print("MaxAcc:"); Serial.println(globalParameter.elvParameter.maxAcc);
    Serial.print("MaxSpe:"); Serial.println(globalParameter.elvParameter.maxSpe);
    Serial.print("StepRev:"); Serial.println(globalParameter.elvParameter.stepRev);
    Serial.print("GearBox:"); Serial.println(globalParameter.elvParameter.gearBox);
    Serial.print("LimExist:"); Serial.println(globalParameter.elvParameter.limExist);
    Serial.print("LimMin:"); Serial.println(globalParameter.elvParameter.limMin);
    Serial.print("LimMax:"); Serial.println(globalParameter.elvParameter.limMax);
    Serial.print("Dir:"); Serial.println(globalParameter.elvParameter.dir);
    Serial.print("Back:"); Serial.println(globalParameter.elvParameter.back);
    Serial.println(globalParameter.ethernetParameter.port);
    Serial.print("Kp:"); Serial.println(globalParameter.control.kp);
    Serial.print("Kd:"); Serial.println(globalParameter.control.kd);
    Serial.print("Freq:"); Serial.println(globalParameter.control.freq);
    Serial.print("MaxTime:"); Serial.println(globalParameter.control.maxTime);
    Serial.print("Mode:"); Serial.println(globalParameter.mode);
    globalParameter.mode = static_cast<TRACKING_MODE>(modeTmp);
  } else {
    Serial.println("Error opening config.txt");
  }
}

void saveConfigFile() {
  File configFile = SD.open("/config.txt", FILE_WRITE);
  configFile.truncate(0);
  
  if (configFile) {
    configFile.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d", 
      globalParameter.azmParameter.maxAcc,
      globalParameter.azmParameter.maxSpe,
      globalParameter.azmParameter.stepRev,
      globalParameter.azmParameter.gearBox,
      globalParameter.azmParameter.limExist,
      globalParameter.azmParameter.limMin,
      globalParameter.azmParameter.limMax,
      globalParameter.azmParameter.dir,
      globalParameter.azmParameter.back,
      globalParameter.elvParameter.maxAcc,
      globalParameter.elvParameter.maxSpe,
      globalParameter.elvParameter.stepRev,
      globalParameter.elvParameter.gearBox,
      globalParameter.elvParameter.limExist,
      globalParameter.elvParameter.limMin,
      globalParameter.elvParameter.limMax,
      globalParameter.elvParameter.dir,
      globalParameter.elvParameter.back,
      globalParameter.ethernetParameter.port,
      globalParameter.control.kp,
      globalParameter.control.kd,
      globalParameter.control.freq,
      globalParameter.control.maxTime,
      globalParameter.mode
    );
    configFile.close();
  }
}

void applyConfig() {
  control.stepperAzm.setMaxSpeed(control.degToStepAzm(globalParameter.azmParameter.maxSpe));
  control.stepperAzm.setAcceleration(control.degToStepAzm(globalParameter.azmParameter.maxAcc));
  control.stepperAzm.setPinsInverted(globalParameter.azmParameter.dir, false, true);
  control.azmGear = globalParameter.azmParameter.gearBox;
  control.azmSpr = unsigned(globalParameter.azmParameter.stepRev);

  control.stepperElv.setMaxSpeed(control.degToStepElv(globalParameter.elvParameter.maxSpe));
  control.stepperElv.setAcceleration(control.degToStepElv(globalParameter.elvParameter.maxAcc));
  control.stepperElv.setPinsInverted(globalParameter.elvParameter.dir, false, true);
  control.elvGear = globalParameter.elvParameter.gearBox;
  control.elvSpr = unsigned(globalParameter.elvParameter.stepRev);

  control.maxSpeedAzm = control.degToStepAzm(globalParameter.azmParameter.maxSpe);
  control.maxSpeedElv = control.degToStepElv(globalParameter.elvParameter.maxSpe);

  control.maxAccelAzm = control.degToStepAzm(globalParameter.azmParameter.maxAcc);
  control.maxAccelElv = control.degToStepElv(globalParameter.elvParameter.maxAcc);

  control.setMode(globalParameter.mode);
  Serial.print("Config applied: ");
}

String readHTML() {
  File htmlFile = SD.open("/index.html");
  String html = "";

  if (htmlFile) {
    while (htmlFile.available()) {
      String line = htmlFile.readStringUntil('\n');
      line.replace("{{AmaxAcc}}", String(globalParameter.azmParameter.maxAcc));
      line.replace("{{AmaxSpe}}", String(globalParameter.azmParameter.maxSpe));
      line.replace("{{AstepRev}}", String(globalParameter.azmParameter.stepRev));
      line.replace("{{AgearBox}}", String(globalParameter.azmParameter.gearBox));
      line.replace("{{AlimExist}}", String(globalParameter.azmParameter.limExist));
      line.replace("{{AlimMin}}", String(globalParameter.azmParameter.limMin));
      line.replace("{{AlimMax}}", String(globalParameter.azmParameter.limMax));
      line.replace("{{Adir}}", String(globalParameter.azmParameter.dir));
      line.replace("{{Aback}}", String(globalParameter.azmParameter.back));
      line.replace("{{EmaxAcc}}", String(globalParameter.elvParameter.maxAcc));
      line.replace("{{EmaxSpe}}", String(globalParameter.elvParameter.maxSpe));
      line.replace("{{EstepRev}}", String(globalParameter.elvParameter.stepRev));
      line.replace("{{EgearBox}}", String(globalParameter.elvParameter.gearBox));
      line.replace("{{ElimExist}}", String(globalParameter.elvParameter.limExist));
      line.replace("{{ElimMin}}", String(globalParameter.elvParameter.limMin));
      line.replace("{{ElimMax}}", String(globalParameter.elvParameter.limMax));
      line.replace("{{Edir}}", String(globalParameter.elvParameter.dir));
      line.replace("{{Eback}}", String(globalParameter.elvParameter.back));
      line.replace("{{port}}", String(globalParameter.ethernetParameter.port));
      line.replace("{{kp}}", String(globalParameter.control.kp));
      line.replace("{{kd}}", String(globalParameter.control.kd));
      line.replace("{{freq}}", String(globalParameter.control.freq));
      line.replace("{{maxTime}}", String(globalParameter.control.maxTime));
      line.replace("{{mode}}", String(globalParameter.mode));
      html += line;
    }
    htmlFile.close();
  } else {
    Serial.println("Error opening index.html");
    html = "Error loading HTML file";
  }
  return html;
}

double azimuthTo(double lat1, double lng1, double lat2, double lng2) {
  lat1 = lat1 * PI / 180.0;
  lng1 = lng1 * PI / 180.0;
  lat2 = lat2 * PI / 180.0;
  lng2 = lng2 * PI / 180.0;

  double dlon = lng2-lng1;
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0) a2 += 2*PI;
  return a2/PI*180.0;
}

double distanceTo(double lat1, double lng1, double lat2, double lng2) {
  double R = 6371000;
  lat1 = lat1 * PI / 180.0;
  lng1 = lng1 * PI / 180.0;
  lat2 = lat2 * PI / 180.0;
  lng2 = lng2 * PI / 180.0;

  double dlat = lat2-lat1;
  double dlng = lng2-lng1;

  double a = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlng/2) * sin(dlng/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double d = R * c;
  return d;
}

AnglePacket computeAngle(double lat1, double lng1, double alt1, double lat2, double lng2, double alt2) {
    AnglePacket command;
    command.azm = azimuthTo(lat1, lng1, lat2, lng2);
    command.elv = (atan((alt2 - alt1) / distanceTo(lat1, lng1, lat2, lng2))/PI)*180.0;
    return command;
}
