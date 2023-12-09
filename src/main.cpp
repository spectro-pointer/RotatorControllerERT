#include <Arduino.h>
#include <capsule.h>
#include <Bounce2.h>
#include "stepper.h"
#include "config.h"
#include "filters.h"
#include <SD.h>

// MAC address for Teensy 1
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 100); // IP address for Teensy 1
unsigned int localPort = 8888; // Local port to listen on

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

SecondOrderEstimator azmEstimator;
SecondOrderEstimator elvEstimator;

SecondOrderLowPassFilter azmFilter(0.1, 100);
SecondOrderLowPassFilter elvFilter(0.1, 100);

void doHoming();
void printPosition();

static TrackerParameter globalParameter;

static float speed = 0.0;
static float position = 0.0;
static float acceleration = 0.0;

String readHTML();
void readConfigFile();
void saveConfigFile();
void setupWebserver();
void loopWebserver();

void setup() {  

  CMD_PORT.begin(CMD_BAUD);
  SERIAL_TO_PC.begin(115200);
  Serial3.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(AZM_ENCODER_PIN, INPUT);

  doHoming();

  Ethernet.begin(mac, ip);
  udp.begin(localPort);
  server.begin();

  // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    // Serial.println("Error initializing SD card");
    while (1);
  }

  // Read parameters from config file
  readConfigFile();
}

void loop() {
  loopAutomatic();
  printPosition();
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

  static TRACKING_MODE lastMode = TRACKING_MODE::STATIONARY;

  if (lastMode != control.getMode()) {
    lastMode = control.getMode();
    if (control.getMode() == TRACKING_MODE::TRACKING_POSITION) {
        azmEstimator.reset(control.lastPosition.azm);
        elvEstimator.reset(control.lastPosition.elv);
    }
  }

  switch(control.getMode()) {

    case TRACKING_MODE::STATIONARY:
      azmEstimator.reset();
      elvEstimator.reset();
    break;
    case TRACKING_MODE::TRACKING_POSITION:
    {
      static unsigned long lastEstimationTime = millis();
      if ((millis()-lastEstimationTime) >= 1000.0/ESTIMATION_RATE) {
  
        double vrx = 0;
        double vry = 0;

        internalAzmNorthEstimate -= (vrx/500.0)/ESTIMATION_RATE;
        internalElvHorizonEstimate -= (vry/500.0)/ESTIMATION_RATE;

        lastEstimationTime = millis();

        double azmEstimation = azmEstimator.computeAngle(millis());
        double elvEstimation = elvEstimator.computeAngle(millis());

        // From now on all angle values are in the referential of the tracker, not north referenced

        double internalAzmValue = ((int(((azmEstimation-internalAzmNorthEstimate)+900.0)*100000.0)%36000000)/100000.0)-180.0;
        double internalElvValue = elvEstimation-internalElvHorizonEstimate;

        double azmFiltered = azmFilter.process(internalAzmValue);
        double elvFiltered = elvFilter.process(internalElvValue);

        PacketTrackerCmd newCmd;

        newCmd.azm = azmFiltered;
        newCmd.elv = elvFiltered;

        newCmd.elv = constrain(newCmd.elv, ELV_MIN_ANGLE, ELV_MAX_ANGLE);
        newCmd.azm = constrain(newCmd.azm, AZM_MIN_ANGLE, AZM_MAX_ANGLE);

        control.update(newCmd);
      }
      controlOutput output = control.computeOutputSpeedPosition();

      speedAzm = output.azmSpeed;
      speedElv = output.elvSpeed;

      // control.stepperAzm.setSpeed(degToStepAzm(output.azmSpeed));
      // control.stepperElv.setSpeed(degToStepElv(output.elvSpeed));
      // control.stepperAzm.runSpeed();
      // control.stepperElv.runSpeed();
    }
    break;
    case TRACKING_MODE::TRACKING_ERROR:

      speedAzm = control.outputSpeedError.azmSpeed;
      speedElv = control.outputSpeedError.elvSpeed;

      // control.stepperAzm.setSpeed(degToStepAzm(control.outputSpeedError.azmSpeed));
      // control.stepperElv.setSpeed(degToStepElv(control.outputSpeedError.elvSpeed));
      // control.stepperAzm.runSpeed();
      // control.stepperElv.runSpeed();
    break;
  }


  control.stepperAzm.setSpeed(degToStepAzm(speedAzm));
  control.stepperElv.setSpeed(degToStepElv(speedElv));
  control.stepperAzm.runSpeed();
  control.stepperElv.runSpeed();
}

void loopManual() {

}

void printPosition() {
  static unsigned long lastPrint = 0;
  if (millis()-lastPrint>1000) {
    lastPrint = millis();
    SERIAL_TO_PC.print(" AZM: ");
    SERIAL_TO_PC.print(stepToDegAzm(control.stepperAzm.currentPosition()));
    SERIAL_TO_PC.print(" ELV: ");
    SERIAL_TO_PC.println(stepToDegElv(control.stepperElv.currentPosition()));
  }
}

void handleCommand(uint8_t packetId, uint8_t *dataIn, uint32_t len) {

  Serial.println("Received packet");

  switch(packetId) {

    case 0x01:
    {
      memcpy(&control.lastCameraError, dataIn, sizeof(CameraErrorPacket));
      // Camera error packet
      double posX = float(control.lastCameraError.Cx)/(float(control.lastCameraError.Tx));
      double posY = float(control.lastCameraError.Cy)/(float(control.lastCameraError.Ty));

      static unsigned long lastFoundTime = millis();

      if (control.lastCameraError.isVisible) {

        posX = posX - 0.5;
        posY = -(posY - 0.5);
        lastFoundTime = millis();

        posX = constrain(posX, -1.0, 1.0);
        posY = constrain(posY, -1.0, 1.0);

        // Estimator Controller // 
        double angleX = posX*15.4;
        double angleY = posY*15.4;

        int Kx = 250;
        int Ky = 250;

        double controlX = Kx*posX;
        double controlY = Ky*posY;

        control.outputSpeedError.azmSpeed = controlX;
        control.outputSpeedError.elvSpeed = controlY;

        control.outputSpeedError.azmSpeed = constrain(control.outputSpeedError.azmSpeed, -AZM_MAX_SPEED_DEG/2.0, AZM_MAX_SPEED_DEG/2.0);
        control.outputSpeedError.elvSpeed = constrain(control.outputSpeedError.elvSpeed, -ELV_MAX_SPEED_DEG/2.0, ELV_MAX_SPEED_DEG/2.0);

      }
      else {
        if (millis()-lastFoundTime>300) {
          control.setMode(TRACKING_MODE::STATIONARY);
          posX = 0;
          posY = 0;
        }
        else {
          control.setMode(TRACKING_MODE::TRACKING_POSITION);
        }
      }
    }
    break;

    case 0x02:
    {
    // Absolute position packet
      memcpy(&control.lastPosition, dataIn, sizeof(PositionPacket));
      azmEstimator.update(control.lastPosition.azm,millis());
      elvEstimator.update(control.lastPosition.elv,millis());
      azmEstimator.setMaxTimeWindow(globalParameter.control.maxTimeWindow);
      elvEstimator.setMaxTimeWindow(globalParameter.control.maxTimeWindow);

      azmFilter.setCutoffFreq(globalParameter.control.cutOffFreq);
      elvFilter.setCutoffFreq(globalParameter.control.cutOffFreq);
    }
    break;

    case 0x03:
    // Pointer position packet
    break;
  }
}

void doHoming() {
}


void loopWebserver() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    // Serial.println("new client");
    // an HTTP request ends with a blank line
    boolean currentLineIsBlank = true;
    String httpRequest = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // Serial.write(c);
        httpRequest += c;

        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {

          // parse the HTTP request to extract form data
          if (httpRequest.indexOf("GET /?") != -1) {
            // extract parameters from the HTTP request
            int speedIndex = httpRequest.indexOf("speed=");
            int positionIndex = httpRequest.indexOf("&position=");
            int accelerationIndex = httpRequest.indexOf("&acceleration=");

            if (speedIndex != -1 && positionIndex != -1 && accelerationIndex != -1) {
              // update variables with the values from the form
              speed = httpRequest.substring(speedIndex + 6, positionIndex).toFloat();
              position = httpRequest.substring(positionIndex + 10, accelerationIndex).toFloat();
              acceleration = httpRequest.substring(accelerationIndex + 14).toFloat();
            }
          }

          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");
          client.println();

          // send the web page
          client.print(readHTML());

          // Save parameters to config file
          saveConfigFile();
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
  File configFile = SD.open("/config.txt");
  
  if (configFile) {
    String line = configFile.readStringUntil('\n');
    configFile.close();

    // Parse the line to extract parameter values
    sscanf(line.c_str(), "%f,%f,%f", &speed, &position, &acceleration);
  } else {
    // Serial.println("Error opening config.txt");
  }
}

void saveConfigFile() {
  File configFile = SD.open("/config.txt", FILE_WRITE);

  // Erase the content of the file
  configFile.truncate(0);
  
  if (configFile) {
    // Save parameters to config.txt
    configFile.printf("%.2f,%.2f,%.2f", speed, position, acceleration);
    configFile.close();
  } else {
    // Serial.println("Error opening config.txt for writing");
  }
}

String readHTML() {
  File htmlFile = SD.open("/index.html");
  String html = "";

  if (htmlFile) {
    while (htmlFile.available()) {
      String line = htmlFile.readStringUntil('\n');
      // Replace placeholders with actual parameter values
      line.replace("{{speed}}", String(speed));
      line.replace("{{position}}", String(position));
      line.replace("{{acceleration}}", String(acceleration));

      html += line;
    }
    htmlFile.close();
  } else {
    // Serial.println("Error opening index.html");
    html = "Error loading HTML file";
  }

  return html;
}
