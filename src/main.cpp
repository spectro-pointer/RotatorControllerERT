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

bool joystickAvailable = false;
bool cameraAvailable = false;

unsigned long lastJoystickFrame = 0;
unsigned long lastCameraFrame = 0;

float azmManualPosition = 0;
float elvManualPosition = 0;

Adafruit_MMA8451 mma = Adafruit_MMA8451();

// MAC address for Teensy 1
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 100); // IP address for Teensy 1
// unsigned int localPort = 8888; // Local port to listen on

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

static TrackerParameter globalParameter;

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

  // Spectrobot
  #ifdef AZM_ENDSTOP_PIN
    azmEndstop.attach(AZM_ENDSTOP_PIN ,  INPUT); 
    azmEndstop.interval(25); // interval in ms
  #endif

  #ifdef ELV_ENDSTOP_PIN
    elvEndstop.attach(ELV_ENDSTOP_PIN ,  INPUT);
    elvEndstop.interval(25); // interval in ms
  #endif

  Ethernet.begin(mac, ip);
  udp.begin(8888);
  server.begin();

  // Initialize SD car
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Error initializing SD card");
    while (1);
  }

  // Read parameters from config file
  readConfigFile();
  applyConfig();

  doHoming();

  PositionPacket groundPosition;
  groundPosition.lat = 46.532308;
  groundPosition.lon = 6.590961;
  groundPosition.alt = 422;

  control.setGroundPosition(groundPosition);
}

void loop() {
  loopAutomatic();
  printPosition();
  loopWebserver();
  // double azmSinWave = 7*sin(millis()/10000.0*2*PI);
  // control.stepperAzm.moveTo(control.degToStepAzm(azmSinWave));
  // control.stepperAzm.run();

  // int packetSize = udp.parsePacket();
  // if (packetSize) {
  //   for (int i = 0; i < packetSize; i++) {
  //     command.decode(udp.read());
  //   }
  // }
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
    // if (control.getMode() == TRACKING_MODE::TRACKING_POSITION) {
    //     azmEstimator.reset(control.lastAngle.azm);
    //     elvEstimator.reset(control.lastAngle.elv);
    //     latEstimator.reset(control.lastPosition.lat);
    //     lonEstimator.reset(control.lastPosition.lon);
    //     altEstimator.reset(control.lastPosition.alt);
    // }
  }

  switch(control.getMode()) {

    case TRACKING_MODE::TRACKING_MANUAL:

      // azmEstimator.reset();
      // elvEstimator.reset();

      if (millis()-control.lastManualTime<300) {

        // speedAzm = control.lastManual.joystickX;
        // speedElv = control.lastManual.joystickY;

        azmManualPosition  = control.lastAngle.azm; //+= speedAzm/10000.0;
        elvManualPosition  = control.lastAngle.elv; //+= speedElv/10000.0;

        azmManualPosition = constrain(azmManualPosition, globalParameter.azmParameter.limMin, globalParameter.azmParameter.limMax);
        elvManualPosition = constrain(elvManualPosition, globalParameter.elvParameter.limMin, globalParameter.elvParameter.limMax);

        control.stepperAzm.moveTo(azmManualPosition);
        control.stepperElv.moveTo(elvManualPosition);

        speedAzm = control.stepperAzm.speed();
        speedElv = control.stepperElv.speed();

        // Serial.print("AZM: ");
        // Serial.print(azmManualPosition,10);
        // Serial.print(" ELV: ");
        // Serial.println(elvManualPosition,10);
      }
    break;

    case TRACKING_MODE::TRACKING_POSITION:
    {
      static unsigned long lastEstimationTime = millis();
      if ((millis()-lastEstimationTime) >= 1000.0/ESTIMATION_RATE) {
  
        double vrx = 0;
        double vry = 0;

        // internalAzmNorthEstimate -= (vrx/500.0)/ESTIMATION_RATE;
        // internalElvHorizonEstimate -= (vry/500.0)/ESTIMATION_RATE;

        lastEstimationTime = millis();

        double azmEstimation = azmEstimator.computeAngle(millis());
        double elvEstimation = elvEstimator.computeAngle(millis());

        // Serial.print("A");
        // Serial.print(azmEstimation);
        // Serial.print(" E");
        // Serial.println(elvEstimation);

        // double latEstimation = latEstimator.computeAngle(millis());
        // double lonEstimation = lonEstimator.computeAngle(millis());
        // double altEstimation = altEstimator.computeAngle(millis());

        // double latFiltered = latFilter.process(latEstimation);
        // double lonFiltered = lonFilter.process(lonEstimation);
        // double altFiltered = altFilter.process(altEstimation);

        // AnglePacket positionFiltered = computeAngle(control.groundPosition.lat, control.groundPosition.lon, control.groundPosition.alt, latFiltered, lonFiltered, altFiltered);

        // From now on all angle values are in the referential of the tracker, not north referenced

        double internalAzmValue = ((int(((azmEstimation-internalAzmNorthEstimate)+900.0)*100000.0)%36000000)/100000.0)-180.0;
        double internalElvValue = elvEstimation-internalElvHorizonEstimate;

        double azmFiltered = azmEstimation; // ;.process(azmEstimation);
        double elvFiltered = elvEstimation; //elvFilter.process(elvEstimation);

        // Serial.print("A");
        // Serial.print(azmFiltered);
        // Serial.print(" E");
        // Serial.println(elvFiltered);

        PacketTrackerCmd newCmd;

        newCmd.azm = azmFiltered;
        newCmd.elv = elvFiltered;

        newCmd.elv = constrain(newCmd.elv, globalParameter.elvParameter.limMin, globalParameter.elvParameter.limMax);
        newCmd.azm = constrain(newCmd.azm, globalParameter.azmParameter.limMin, globalParameter.azmParameter.limMax);

        control.update(newCmd);
      }
      controlOutput output = control.computeOutputSpeedPosition();

      speedAzm = output.azmSpeed;
      speedElv = output.elvSpeed;

      // control.stepperAzm.setSpeed(control.degToStepAzm(output.azmSpeed));
      // control.stepperElv.setSpeed(control.degToStepElv(output.elvSpeed));
      // control.stepperAzm.runSpeed();
      // control.stepperElv.runSpeed();
    }
    break;

    case TRACKING_MODE::TRACKING_ERROR:

      speedAzm = control.outputSpeedError.azmSpeed;
      speedElv = control.outputSpeedError.elvSpeed;

      // control.stepperAzm.setSpeed(degToStepAzm(control.outputSpeedError.azmSpeed));
      // control.stepperElv.setSpeed(control.degToStepElv(control.outputSpeedError.elvSpeed));
      // control.stepperAzm.runSpeed();
      // control.stepperElv.runSpeed();
    break;

    case TRACKING_MODE::TRACKING_STOP:
      speedAzm = 0;
      speedElv = 0;
    break;  

  }

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

  static unsigned long lastPrint = millis();
  if (millis()-lastPrint>100) {
    lastPrint = millis();
    // Serial.print(" AZM: ");
    // Serial.print(control.degToStepAzm(speedAzm));
    // Serial.print(" ELV: ");
    // Serial.println(control.degToStepElv(speedElv));
  }

  control.stepperAzm.setSpeed(control.degToStepAzm(speedAzm));
  control.stepperElv.setSpeed(control.degToStepElv(speedElv));
  control.stepperAzm.runSpeed();
  control.stepperElv.runSpeed();
}

void loopManual() {

}

void printPosition() {
  static unsigned long lastPrint = 0;
  if (millis()-lastPrint>100) {
    lastPrint = millis();
    IPAddress ipBroadcast(255,255,255,255);

    // We will send a capsule packet with the structure of a AnglePacket.
    // The capsule packet will be sent to the broadcast address on port 8888.

    AnglePacket angleToSend; 
    angleToSend.azm = control.stepToDegAzm(control.stepperAzm.currentPosition());
    angleToSend.elv = control.stepToDegElv(control.stepperElv.currentPosition());

    uint8_t packetData[sizeof(AnglePacket)];
    memcpy(packetData, &angleToSend, sizeof(AnglePacket));
    
    uint8_t packetId = 33; 

    uint8_t* packetToSend = command.encode(packetId,packetData,sizeof(AnglePacket));
    udp.beginPacket(ipBroadcast, 8888);
    udp.write(packetToSend,command.getCodedLen(sizeof(AnglePacket)));
    udp.endPacket();
    delete[] packetToSend;
  }
}

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

  // Serial.print("Received packet with packet ID: ");
  // Serial.println(packetId);

  switch(packetId) {

    case 0x01:
    {
      control.lastCameraErrorTime = millis();
      memcpy(&control.lastCameraError, dataIn, sizeof(CameraErrorPacket));
      // Camera error packet
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
        if (joystickAvailable and control.lastCameraError.camID == 99) {
          shouldWeUsePacket = true;
        }
        else if (!joystickAvailable and control.lastCameraError.camID == 33) {
          shouldWeUsePacket = true;
        }

        if (shouldWeUsePacket) {
          float KPx = control.lastCameraError.Kp; //= globalParameter.control.kp;
          float KPy = control.lastCameraError.Kp; //= globalParameter.control.kp;

          float maxSpeed = control.lastCameraError.maxSpeed;

          static float lastPosX = posX;
          static float lastPosY = posY;

          double controlX = KPx*posX;
          double controlY = KPy*posY;

          control.outputSpeedError.azmSpeed = controlX;
          control.outputSpeedError.elvSpeed = controlY;

          lastPosX = posX;
          lastPosY = posY;

          control.outputSpeedError.azmSpeed = constrain(control.outputSpeedError.azmSpeed, -maxSpeed, maxSpeed);
          control.outputSpeedError.elvSpeed = constrain(control.outputSpeedError.elvSpeed, -maxSpeed, maxSpeed);
        }

      }
      else {
        if (control.lastCameraError.camID == 99 and !control.lastCameraError.isVisible) {
          joystickAvailable = false;
        }
      }

      if ((millis()-lastFoundTime)>100) {
        control.setMode(TRACKING_MODE::TRACKING_STOP);
        posX = 0;
        posY = 0;
      }
      else {
        control.setMode(TRACKING_MODE::TRACKING_ERROR);
      }

      if ((millis()-lastCameraFrame)>50) {
        cameraAvailable = false;
      }
      if ((millis()-lastJoystickFrame)>100) {
        joystickAvailable = false;
      }
      
    }
    break;

    case 0x02:
    {
    control.lastPositionTime = millis();
    // Lat lon position packet
      memcpy(&control.lastPosition, dataIn, sizeof(PositionPacket));

      azmEstimator.update(control.lastAngle.azm,millis());
      elvEstimator.update(control.lastAngle.elv,millis());
      azmEstimator.setMaxTimeWindow(500);
      elvEstimator.setMaxTimeWindow(500);

      azmFilter.setCutoffFreq(globalParameter.control.freq);
      elvFilter.setCutoffFreq(globalParameter.control.freq);
    }
    break;

    // Absolute angle packet
    case 0x03:
      control.lastAngleTime = millis();
      memcpy(&control.lastAngle, dataIn, sizeof(AnglePacket));
      Serial.println("Received ABS Packet!!");
      // azmEstimator.update(control.lastAngle.azm,millis());
      // elvEstimator.update(control.lastAngle.elv,millis());
      // azmEstimator.setMaxTimeWindow(500);
      // elvEstimator.setMaxTimeWindow(500);

      // azmFilter.setCutoffFreq(globalParameter.control.freq);
      // elvFilter.setCutoffFreq(globalParameter.control.freq);
      azmManualPosition = control.lastAngle.azm;
      elvManualPosition = control.lastAngle.elv;
    break;

    // Manual control packet
    case 99:
      control.lastManualTime = millis();
      memcpy(&control.lastManual, dataIn, sizeof(ManualPacket));
      Serial.print("Manual cmd received ");
      Serial.print(" X: ");
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
    Serial.print("ELV: ");
    Serial.println(avg);
  #endif 

  #ifdef SPECTROBOT
    // Spectrobot
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
  }
  else {
    delay(1000);
    Serial.println("No ELV limit switch");
    control.stepperElv.setCurrentPosition(0);
  }


  if (globalParameter.azmParameter.limExist) {

    #ifdef DUMBO
      // Take 100 samples of the encoder value and do the average
      double avg = 0;
      for (int i = 0; i < 100; i++) {
        avg += analogRead(AZM_ENCODER_PIN);
        delay(1);
      }
      avg /= 100.0;
      double azmRead = map(avg,103,923, 0,360);
      azmRead = (int((azmRead + (360-200))*100.0)%36000)/100.0;
      if (azmRead>180) {
        azmRead -= 360;
      }
      control.stepperAzm.setCurrentPosition(control.degToStepAzm(azmRead));
      delay(1000);
      Serial.print("AZM: ");
      Serial.println(azmRead);
    #endif

    #ifdef SPECTROBOT
    // Spectrobot
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
  }

  else {
    delay(1000);
    Serial.println("No AZM limit switch");
    control.stepperAzm.setCurrentPosition(0);
  }
}

void loopWebserver() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
     Serial.println("new client");
    // an HTTP request ends with a blank line
    boolean currentLineIsBlank = true;
    String httpRequest = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        httpRequest += c;

        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {

          // parse the HTTP request to extract form data
          if (httpRequest.indexOf("GET /?") != -1) {
            // extract parameters from the HTTP request
            int AmaxAccIndex = httpRequest.indexOf("AmaxAcc=");
            // Serial.println(AmaxAccIndex);
            int AmaxSpeIndex = httpRequest.indexOf("&AmaxSpe=");
            // Serial.println(AmaxSpeIndex);
            int AstepRevIndex = httpRequest.indexOf("&AstepRev=");
            // Serial.println(AstepRevIndex);
            int AgearBoxIndex = httpRequest.indexOf("&AgearBox=");
            // Serial.println(AgearBoxIndex);
            int AlimExistIndex = httpRequest.indexOf("&AlimExist=");
            // Serial.println(AlimExistIndex);
            int AlimMinIndex = httpRequest.indexOf("&AlimMin=");
            // Serial.println(AlimMinIndex);
            int AlimMaxIndex= httpRequest.indexOf("&AlimMax=");
            // Serial.println(AlimMaxIndex);
            int AdirIndex = httpRequest.indexOf("&Adir=");
            // Serial.println(AdirIndex);
            int AbackIndex = httpRequest.indexOf("&Aback=");
            // Serial.println(AbackIndex);
            // elevacion  
            int EmaxAccIndex = httpRequest.indexOf("&EmaxAcc=");
            // Serial.println(EmaxAccIndex);
            int EmaxSpeIndex = httpRequest.indexOf("&EmaxSpe=");
            // Serial.println(EmaxSpeIndex);
            int EstepRevIndex = httpRequest.indexOf("&EstepRev=");
            // Serial.println(EstepRevIndex);
            int EgearBoxIndex = httpRequest.indexOf("&EgearBox=");
            // Serial.println(EgearBoxIndex);
            int ElimExistIndex = httpRequest.indexOf("&ElimExist=");
            // Serial.println(ElimExistIndex);
            int ElimMinIndex = httpRequest.indexOf("&ElimMin=");
            // Serial.println(ElimMinIndex);
            int ElimMaxIndex= httpRequest.indexOf("&ElimMax=");
            // Serial.println(ElimMaxIndex);
            int EdirIndex = httpRequest.indexOf("&Edir=");
            // Serial.println(EdirIndex);
            int EbackIndex = httpRequest.indexOf("&Eback=");
            // Serial.println(EbackIndex);
            //
            // int ipIndex = httpRequest.indexOf("&ip=");
            int portIndex = httpRequest.indexOf("&port=");
            // Serial.println(portIndex);
            //
            int kpIndex = httpRequest.indexOf("&kp=");
            // Serial.println(kpIndex);
            int kdIndex = httpRequest.indexOf("&kd=");
            // Serial.println(kpIndex);
            int freqIndex = httpRequest.indexOf("&freq=");
            // Serial.println(freqIndex);
            int maxTimeIndex = httpRequest.indexOf("&maxTime=");
            // Serial.println(maxTimeIndex);
            int modeIndex = httpRequest.indexOf("&mode=");
            //
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
                // && ipIndex != -1
                && portIndex !=-1
                && kpIndex != -1
                && kdIndex != -1
                && freqIndex !=-1
                && maxTimeIndex != -1
                && modeIndex != -1
                ) {
              // update variables with the values from the form            
              // Example pour modifier les paramètres
              globalParameter.azmParameter.maxAcc = httpRequest.substring(AmaxAccIndex + 8, AmaxSpeIndex).toFloat();
              globalParameter.azmParameter.maxSpe = httpRequest.substring(AmaxSpeIndex + 9, AstepRevIndex).toFloat();
              globalParameter.azmParameter.stepRev = httpRequest.substring(AstepRevIndex + 10, AgearBoxIndex).toFloat();
              globalParameter.azmParameter.gearBox = httpRequest.substring(AgearBoxIndex + 10, AlimExistIndex).toFloat();
              globalParameter.azmParameter.limExist = httpRequest.substring(AlimExistIndex + 11, AlimMinIndex).toFloat();
              globalParameter.azmParameter.limMin = httpRequest.substring(AlimMinIndex + 9, AlimMaxIndex).toFloat();
              globalParameter.azmParameter.limMax = httpRequest.substring(AlimMaxIndex + 9, AdirIndex).toFloat();
              globalParameter.azmParameter.dir = httpRequest.substring(AdirIndex + 6, AbackIndex).toFloat();
              globalParameter.azmParameter.back = httpRequest.substring(AbackIndex + 7, EmaxAccIndex).toFloat();
              // elevacion
              globalParameter.elvParameter.maxAcc = httpRequest.substring(EmaxAccIndex + 9, EmaxSpeIndex).toFloat();
              globalParameter.elvParameter.maxSpe = httpRequest.substring(EmaxSpeIndex + 9, EstepRevIndex).toFloat();
              globalParameter.elvParameter.stepRev = httpRequest.substring(EstepRevIndex + 10, EgearBoxIndex).toFloat();
              globalParameter.elvParameter.gearBox = httpRequest.substring(EgearBoxIndex + 10, ElimExistIndex).toFloat();
              globalParameter.elvParameter.limExist = httpRequest.substring(ElimExistIndex + 11, ElimMinIndex).toFloat();
              globalParameter.elvParameter.limMin = httpRequest.substring(ElimMinIndex + 9, ElimMaxIndex).toFloat();
              globalParameter.elvParameter.limMax = httpRequest.substring(ElimMaxIndex + 9, EdirIndex).toFloat();
              globalParameter.elvParameter.dir = httpRequest.substring(EdirIndex + 6, EbackIndex).toFloat();
              globalParameter.elvParameter.back = httpRequest.substring(EbackIndex + 7, portIndex).toFloat();
              //
              // globalParameter.ethernetParameter.ip = httpRequest.substring(ipIndex + 4, portIndex).toFloat();
              globalParameter.ethernetParameter.port = httpRequest.substring(portIndex + 6, kpIndex).toFloat();
              //
              globalParameter.control.kp = httpRequest.substring(kpIndex + 4, kdIndex).toFloat();
              globalParameter.control.kd = httpRequest.substring(kdIndex + 4, freqIndex).toFloat();
              globalParameter.control.freq = httpRequest.substring(freqIndex + 6, maxTimeIndex).toFloat();
              globalParameter.control.maxTime = httpRequest.substring(maxTimeIndex + 9, modeIndex).toFloat();
              globalParameter.mode = httpRequest.substring(modeIndex + 6).toInt();
              Serial.print("Received config with mode: ");
              Serial.print(globalParameter.mode);
              Serial.println();
              
              // Save parameters to config file
              applyConfig();
              Serial.print("Config applied: ");
              Serial.println(globalParameter.control.maxTime);
              saveConfigFile();
            }
            else {
              Serial.println("Error parsing HTTP request");
            }
          }

          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");
          client.println();

          // send the web page
          client.print(readHTML());
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    client.stop();
    // Serial.println("client disconnected");
  }
}

void readConfigFile() {
  delay(1000);
  File configFile = SD.open("/config.txt");
  
  if (configFile) {
    String line = configFile.readStringUntil('\n');
    configFile.close();

    // Parse the line to extract parameter values as parsed in loopWebserver()
    // The data is coherent with the data in the globalParameter structure.
    // There are 22 fields in the config.txt file, separated by commas
    sscanf(line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d", 
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
      &globalParameter.mode
    );
    Serial.println("Config file read");
    Serial.println(line);
    Serial.print("MaxAcc:");
    Serial.println(globalParameter.azmParameter.maxAcc);
    Serial.print("MaxSpe:");
    Serial.println(globalParameter.azmParameter.maxSpe);
    Serial.print("StepRev:");
    Serial.println(globalParameter.azmParameter.stepRev);
    Serial.print("GearBox:");
    Serial.println(globalParameter.azmParameter.gearBox);
    Serial.print("LimExist:");
    Serial.println(globalParameter.azmParameter.limExist);
    Serial.print("LimMin:");
    Serial.println(globalParameter.azmParameter.limMin);
    Serial.print("LimMax:");
    Serial.println(globalParameter.azmParameter.limMax);
    Serial.print("Dir:");
    Serial.println(globalParameter.azmParameter.dir);
    Serial.print("Back:");
    Serial.println(globalParameter.azmParameter.back);
    Serial.print("MaxAcc:");
    Serial.println(globalParameter.elvParameter.maxAcc);
    Serial.print("MaxSpe:");
    Serial.println(globalParameter.elvParameter.maxSpe);
    Serial.print("StepRev:");
    Serial.println(globalParameter.elvParameter.stepRev);
    Serial.print("GearBox:");
    Serial.println(globalParameter.elvParameter.gearBox);
    Serial.print("LimExist:");
    Serial.println(globalParameter.elvParameter.limExist);
    Serial.print("LimMin:");
    Serial.println(globalParameter.elvParameter.limMin);
    Serial.print("LimMax:");
    Serial.println(globalParameter.elvParameter.limMax);
    Serial.print("Dir:");
    Serial.println(globalParameter.elvParameter.dir);
    Serial.print("Back:");
    Serial.println(globalParameter.elvParameter.back);
    // Serial.print("IP:");
    Serial.println(globalParameter.ethernetParameter.port);
    Serial.print("Kp:");
    Serial.println(globalParameter.control.kp);
    Serial.print("Kd:");
    Serial.println(globalParameter.control.kd);
    Serial.print("Freq:");
    Serial.println(globalParameter.control.freq);
    Serial.print("MaxTime:");
    Serial.println(globalParameter.control.maxTime);
    Serial.print("Mode:");
    Serial.println(globalParameter.mode);
  } else {
    Serial.println("Error opening config.txt");
  }
}

void saveConfigFile() {
  File configFile = SD.open("/config.txt", FILE_WRITE);

  // Erase the content of the file
  configFile.truncate(0);
  
  if (configFile) {
    // Save parameters to config.txt
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
      // globalParameter.ethernetParameter.ip,
      globalParameter.ethernetParameter.port,
      globalParameter.control.kp,
      globalParameter.control.kd,
      globalParameter.control.freq,
      globalParameter.control.maxTime,
      globalParameter.mode
    );
    configFile.close();
  } else {
    // Serial.println("Error opening config.txt for writing");
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
      // Replace placeholders with actual parameter values
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
      // line.replace("{{ip}}", String(globalParameter.ethernetParameter.ip));
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
  if (a2 < 0.0)
  {
    a2 += 2*PI;
  }
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
