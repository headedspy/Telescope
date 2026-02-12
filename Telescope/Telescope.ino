#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include <ESP8266WebServer.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>

#define STEPPER_OUTPUT1 13
#define STEPPER_OUTPUT2 15
#define STEPPER_OUTPUT3 0
#define STEPPER_OUTPUT4 2

#define QMC5883L_ADDR 0x0D
#define QMC5883L_X_LSB_ADDR 0x00

#define QMC5883L_STATUS_REG 0x06
#define QMC5883L_CONTROL_1_REG 0x09
#define QMC5883L_CONTROL_2_REG 0x0A
#define QMC5883L_SETRESET_PERIOD_REG 0x0B

#define QMC_5883L_SOFT_RST 7

// Bit offsets (Starting position in the 8-bit register)
#define QMC_5883L_MODE 0  // Bits 0-1
#define QMC_5883L_ODR  2  // Bits 2-3
#define QMC_5883L_RNG  4  // Bits 4-5
#define QMC_5883L_OSR  6  // Bits 6-7

// Mode (00 = Standby, 01 = Continuous)
#define QMC_5883L_MODE_STANDBY    (0x00 << QMC_5883L_MODE)
#define QMC_5883L_MODE_CONTINUOUS (0x01 << QMC_5883L_MODE)

// Output Data Rate (00=10Hz, 01=50Hz, 10=100Hz, 11=200Hz)
#define QMC_5883L_ODR_10HZ        (0x00 << QMC_5883L_ODR)
#define QMC_5883L_ODR_50HZ        (0x01 << QMC_5883L_ODR)
#define QMC_5883L_ODR_100HZ       (0x02 << QMC_5883L_ODR)
#define QMC_5883L_ODR_200HZ       (0x03 << QMC_5883L_ODR)

// Range (00 = 2G, 01 = 8G)
#define QMC_5883L_RNG_2G          (0x00 << QMC_5883L_RNG)
#define QMC_5883L_RNG_8G          (0x01 << QMC_5883L_RNG)

// Oversampling Ratio (00=512, 01=256, 10=128, 11=64)
#define QMC_5883L_OSR_512         (0x00 << QMC_5883L_OSR)
#define QMC_5883L_OSR_256         (0x01 << QMC_5883L_OSR)
#define QMC_5883L_OSR_128         (0x02 << QMC_5883L_OSR)
#define QMC_5883L_OSR_64          (0x03 << QMC_5883L_OSR)

#define SERVO_OFFSET 84

// 128x64 res, first 16 rows - yellow
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, 12, 14, U8X8_PIN_NONE);
String statusString = "";

AccelStepper stepper(AccelStepper::HALF4WIRE, STEPPER_OUTPUT1, STEPPER_OUTPUT3, STEPPER_OUTPUT2, STEPPER_OUTPUT4);
int16_t min_x, max_x;
int16_t min_y, max_y;
float hic_x, hic_y;

Servo servo;

const char* ssid = "az";
const char* password = "gonoreq7";

// set web server port number to 80
WiFiServer server(80);
String http_request;

// ntp server for date/time retrieving
WiFiUDP ntpUDP;

// bg is in UTC+02:00
// 2 * 60 * 60 = 7200
const long utcOffsetInSeconds = 7200;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

int t_year, t_month, t_day;
int t_hour, t_min, t_sec;

float alt = 0.0;

void compass_init()
{
  Wire.begin(12, 14);
  Wire.setClock(100000);

  // soft reset for clean start
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_CONTROL_2_REG);
  Wire.write(1 << QMC_5883L_SOFT_RST);
  Wire.endTransmission();
  delay(100);

  //define set/reset period
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_SETRESET_PERIOD_REG);
  Wire.write(0x01);
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_CONTROL_1_REG);
  Wire.write(QMC_5883L_MODE_CONTINUOUS | QMC_5883L_ODR_200HZ | QMC_5883L_RNG_8G | QMC_5883L_OSR_512);
  Wire.endTransmission();

  delay(100);
}

void readCompass(int16_t &x, int16_t&y, int16_t&z) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_STATUS_REG);
  Wire.endTransmission();

  Wire.requestFrom(QMC5883L_ADDR, 1);
  byte status = Wire.read();

  // bit 0 of Status Register is DRDY
  if (status & 0x01) {
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(QMC5883L_X_LSB_ADDR);
    Wire.endTransmission();

    Wire.requestFrom(QMC5883L_ADDR, 6);
    
    if (Wire.available() == 6) {
      uint8_t x_lsb = Wire.read();
      uint8_t x_msb = Wire.read();
      uint8_t y_lsb = Wire.read();
      uint8_t y_msb = Wire.read();
      uint8_t z_lsb = Wire.read();
      uint8_t z_msb = Wire.read();

      x = (int16_t)((x_msb << 8) | x_lsb);
      y = (int16_t)((y_msb << 8) | y_lsb);
      z = (int16_t)((z_msb << 8) | z_lsb);
    }
  }
}

float getHeading()
{
  int16_t x, y, z;
  readCompass(x, y, z);

  float heading = atan2(y - hic_y, x - hic_x);
  // rad to deg
  heading = heading * 180 / PI;

  // magnetic declination in sofia
  heading += 5.6833;
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;

  return heading;
}

void calibrateCompass()
{
  displayString("[CALIBRATING]");
  min_x = min_y = 32767;
  max_x = max_y = -32768;
  hic_x = 0;
  hic_y = 0;

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(2000); 

  // collect data over a full 360-degree sweep
  for (int i = 0; i <= 72; i++) {
    float deg = i * 5.0f;
    long targetSteps = round(deg * (4096.0f / 360.0f));

    stepper.moveTo(targetSteps);

    while (stepper.distanceToGo() != 0) {
      stepper.run();
      // force background processing to prevent WDT freeze
      optimistic_yield(1); 
    }
    
    // let the sensor settle before reading
    delay(20); 

    int16_t x, y, z;
    readCompass(x, y, z);

    min_x = min(min_x, x);
    min_y = min(min_y, y);
    max_x = max(max_x, x);
    max_y = max(max_y, y);
  }

  hic_x = (max_x + min_x) / 2.0f;
  hic_y = (max_y + min_y) / 2.0f;

  stepper.stop();
  stepper.disableOutputs();
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(100);
}

void rotateStepper(float targetDeg)
{
  const float stepsPerDegree = 4096.0f / 360.0f;
  const float tolerance = .5f;
  int maxAttempts = 5;

  for (int i = 0; i < maxAttempts; i++) {
    float currentHeading = getHeading();
    float diff = targetDeg - currentHeading;

    // shortest path wrap-around
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    if (abs(diff) <= tolerance) break;

    long stepsToMove = round(diff * stepsPerDegree) * -1;

    stepper.enableOutputs();
    stepper.move(stepsToMove);
    while (stepper.distanceToGo() != 0) {
      stepper.run();
      // force background processing to prevent WDT freeze
      optimistic_yield(1);
    }

    stepper.stop();
    stepper.disableOutputs();

    digitalWrite(STEPPER_OUTPUT1, LOW);
    digitalWrite(STEPPER_OUTPUT2, LOW);
    digitalWrite(STEPPER_OUTPUT3, LOW);
    digitalWrite(STEPPER_OUTPUT4, LOW);
    
    // delay to let the sensor settle before the next check
    delay(200); 
  }
}

int sizeofdata = 10;
int x[] = {-30, -20, -10, 0, 10, 20, 30, 40, 50, 60}; //given deg
int y[] = {-32, -23, -11, 0, 13, 25, 36, 48, 57, 67}; // actual deg

float servo_lerp(int xIn)
{
  //handle out-of-range input
  if (xIn < x[0] || xIn > x[sizeofdata - 1])
    return NAN;

  for (int i = 0; i < sizeofdata - 1; i++)
  {
    if (x[i] <= xIn && xIn <= x[i + 1])
    {
      float t = (float)(xIn - x[i]) / (float)(x[i + 1] - x[i]);
      return y[i] + t * (y[i + 1] - y[i]);
    }
  }

  return NAN; // should never reach here
}

void rotateServo(int deg)
{
  servo.write(0);
  delay(500);
  int servodata = SERVO_OFFSET + servo_lerp(deg);
  servo.write(servodata);
}

void display_prepare(void)
{
  display.setFont(u8g2_font_6x10_tf);
  display.setFontRefHeightExtendedText();
  display.setDrawColor(1);
  display.setFontPosTop();
  display.setFontDirection(0);
}

void displayStatusString(const char* str)
{
  display.clearBuffer();
  display_prepare();
  display.setFontDirection(0);
  display.drawStr(0, 0, str);

  display.sendBuffer();
}

void displayStatusString()
{
  display_prepare();
  display.setFontDirection(0);
  display.drawStr(0, 0, statusString.c_str());
  display.drawStr(95, 0, (String(t_hour) + ":" + (t_min <= 9 ? "0" : "") + String(t_min)).c_str());
}

void displayHeading(float heading)
{
  display.clearBuffer();
  display_prepare();
  display.setFontDirection(0);
  display.drawStr( 0, 16, ("azi " + String(heading)).c_str());
  display.drawStr( 0, 24, ("alt " + String(alt)).c_str());

  displayStatusString();
  display.sendBuffer();
}

void displayString(const char *string)
{
  display.clearBuffer();
  display_prepare();
  display.setFontDirection(0);
  display.drawStr(0, 16, string);

  displayStatusString();
  display.sendBuffer();
}

int extractAngle(String request)
{
  int start = request.indexOf("angle=") + 6;
  int end = request.indexOf(' ', start);  // end of GET path

  String valueStr = request.substring(start, end);
  return valueStr.toInt();
}

void updateTime()
{
  timeClient.update();

  unsigned long epochTime = timeClient.getEpochTime();

  time_t rawtime = epochTime;
  struct tm * ti;

  ti = localtime(&rawtime);

  t_year = ti->tm_year + 1900;
  t_month = ti->tm_mon + 1;
  t_day = ti->tm_mday;

  t_hour = ti->tm_hour;
  t_min = ti->tm_min;
  t_sec = ti->tm_sec;
}

void setup()
{
  Serial.begin(9600);
  compass_init();

  if(!display.begin()){
    Serial.println("Display failed!");
  }
  delay(100);

  pinMode(STEPPER_OUTPUT1, OUTPUT);
  pinMode(STEPPER_OUTPUT2, OUTPUT);
  pinMode(STEPPER_OUTPUT3, OUTPUT);
  pinMode(STEPPER_OUTPUT4, OUTPUT);
  
  digitalWrite(STEPPER_OUTPUT1, LOW);
  digitalWrite(STEPPER_OUTPUT2, LOW);
  digitalWrite(STEPPER_OUTPUT3, LOW);
  digitalWrite(STEPPER_OUTPUT4, LOW);


  servo.attach(4, 500, 2500);

  Serial.print("connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  displayStatusString("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  statusString = WiFi.localIP().toString();
  displayStatusString();

  timeClient.begin();
  
  updateTime();

  Serial.println("");
  Serial.println("IP: ");
  Serial.println(WiFi.localIP());
  server.begin();

  stepper.setMaxSpeed(500);
  stepper.setAcceleration(100);

  alt = 0.0;
  rotateServo(alt);
}


void loop() {
  float heading = getHeading();
  displayHeading(heading);
  delay(100);

  int pendingStepperAngle = -999; 
  int pendingServoAngle = -999;

  WiFiClient client = server.available();
  if (!client || !client.connected()) {
    delay(10); 
    return; 
  }

  if (client) {
    Serial.println("New Client.");
    unsigned long timeout = millis();
    // incoming data from the client
    String currentLine = "";

    while (client.connected() && millis() - timeout < 2000) {

      // read if there's bytes to read from the client
      if (client.available()) {
        char c = client.read();
        timeout = millis();
        Serial.write(c);
        http_request += c;
        
        // if the byte is a newline character and the current line is blank, you got two newline characters in a row.
        // that's the end of the client request, send a response
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            if(http_request.indexOf("/calibrate") >= 0){
              calibrateCompass();
            }
            else if (http_request.indexOf("GET /rotateServo?angle=") >= 0) {
              pendingServoAngle = extractAngle(http_request);
            }
            else if (http_request.indexOf("GET /rotateStepper?angle=") >= 0) {
              pendingStepperAngle = extractAngle(http_request);
            }

            String date = String(t_year) + "-" + (t_month <= 9 ? "0" : "") + String(t_month) + "-" + (t_day <= 9 ? "0" : "") +String(t_day);
            String time = (t_hour <= 9 ? "0" : "") + String(t_hour) + ":" + (t_min <= 9 ? "0" : "") + String(t_min) + ":" + (t_sec <= 9 ? "0" : "") + String(t_sec);

            client.println(R"=====(
            <!DOCTYPE html>
            <html lang="en">
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <title>Telescope</title>
                <link href="https://fonts.googleapis.com/css2?family=Orbitron:wght@500;700&display=swap" rel="stylesheet">
                <style>
                    body {
                        margin: 0;
                        padding: 0;
                        background-color: #000;
                        overflow-x: hidden;
                        font-family: 'Orbitron', sans-serif;
                        color: #aaddff;
                        display: flex;
                        justify-content: center;
                        align-items: center;
                        min-height: 100vh;
                    }
                    .stars, .stars2, .stars3 {
                        position: fixed;
                        top: 0; left: 0; right: 0; bottom: 0;
                        background: transparent;
                        z-index: -1;
                    }
                    .stars {
                        background-image: 
                            radial-gradient(1px 1px at 20px 30px, #eee, rgba(0,0,0,0)),
                            radial-gradient(1px 1px at 40px 70px, #fff, rgba(0,0,0,0)),
                            radial-gradient(1px 1px at 50px 160px, #ddd, rgba(0,0,0,0)),
                            radial-gradient(1px 1px at 90px 40px, #fff, rgba(0,0,0,0)),
                            radial-gradient(1px 1px at 130px 80px, #fff, rgba(0,0,0,0));
                        background-size: 200px 200px;
                        animation: moveStars 100s linear infinite;
                    }
                    .stars2 {
                        background-image: 
                            radial-gradient(2px 2px at 100px 150px, #fff, rgba(0,0,0,0)),
                            radial-gradient(2px 2px at 200px 50px, #aaa, rgba(0,0,0,0));
                        background-size: 300px 300px;
                        animation: moveStars 150s linear infinite;
                    }
                    @keyframes moveStars {
                        from { background-position: 0 0; }
                        to { background-position: 0 2000px; }
                    }
                    .main-container {
                        display: flex;
                        gap: 50px;
                        background: rgba(10, 15, 30, 0.85);
                        padding: 50px;
                        border: 3px solid #4488ff;
                        border-radius: 20px;
                        box-shadow: 0 0 50px rgba(68, 136, 255, 0.2), inset 0 0 100px rgba(0,0,0,0.8);
                        backdrop-filter: blur(5px);
                        min-width: 1200px; 
                        transform: scale(0.9); 
                    }
                    .left-panel { display: flex; flex-direction: column; gap: 30px; }
                    .right-panel { display: flex; flex-direction: column; gap: 40px; align-items: center; justify-content: flex-start; padding-top: 20px; }
                    button {
                        background: rgba(20, 40, 60, 0.9);
                        border: 2px solid #4488ff;
                        color: #4488ff;
                        font-family: 'Orbitron', sans-serif;
                        font-size: 1.1rem;
                        text-transform: uppercase;
                        letter-spacing: 2px;
                        cursor: pointer;
                        transition: 0.2s ease-in-out;
                        box-shadow: 0 0 10px rgba(68, 136, 255, 0.1);
                        border-radius: 5px;
                    }
                    button:hover { background: #4488ff; color: #000; box-shadow: 0 0 25px #4488ff; }
                    button:active { transform: scale(0.98); }
                    #calibrate-btn { width: 100%; padding: 25px; font-size: 1.5rem; font-weight: bold; background: rgba(68, 136, 255, 0.1); }
                    #calibrate-btn.uncalibrated { background: rgba(150, 0, 0, 0.2); border-color: #ff4444; color: #ff4444; animation: pulse-red 2s infinite; }
                    @keyframes pulse-red {
                        0% { box-shadow: 0 0 5px rgba(255, 68, 68, 0.4); }
                        50% { box-shadow: 0 0 25px rgba(255, 68, 68, 0.8); }
                        100% { box-shadow: 0 0 5px rgba(255, 68, 68, 0.4); }
                    }
                    .controls-row {
                        display: flex;
                        justify-content: space-between;
                        background: rgba(0,0,0,0.3);
                        padding: 20px;
                        border-radius: 10px;
                        border: 1px solid #335577;
                    }
                    .control-group { display: flex; flex-direction: column; align-items: center; width: 45%; }
                    .control-label { margin-top: 10px; font-size: 1.2rem; color: #fff; }
                    input[type=range] { -webkit-appearance: none; width: 100%; height: 10px; background: #112233; border-radius: 5px; outline: none; border: 1px solid #4488ff; }
                    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 25px; height: 25px; background: #4488ff; cursor: pointer; box-shadow: 0 0 15px #4488ff; }
                    .rotate-buttons { display: flex; gap: 20px; }
                    .rotate-btn { flex-grow: 1; padding: 15px; font-weight: bold; }
                    .compass-container {
                        position: relative;
                        width: 180px;
                        height: 180px;
                        border: 4px solid #4488ff;
                        border-radius: 50%;
                        background: radial-gradient(circle, #001122 0%, #000 100%);
                        box-shadow: 0 0 30px rgba(68, 136, 255, 0.3);
                    }
                    .compass-btn { position: absolute; width: 50px; height: 50px; border-radius: 50%; background: #000; z-index: 2; }
                    .btn-n { top: -25px; left: 50%; transform: translateX(-50%); }
                    .btn-s { bottom: -25px; left: 50%; transform: translateX(-50%); }
                    .btn-e { right: -25px; top: 50%; transform: translateY(-50%); }
                    .btn-w { left: -25px; top: 50%; transform: translateY(-50%); }
                    .bodies-list { display: flex; flex-direction: column; gap: 15px; width: 100%; }
                    .body-btn { width: 200px; padding: 15px; font-size: 1.1rem; text-align: center; }
                    .skymap-wrapper { border: 4px solid #4488ff; border-radius: 10px; overflow: hidden; background: #000; }
                </style>
            </head>
            <body>
                <div class="stars"></div>
                <div class="stars2"></div>
                <div class="stars3"></div>
                <div class="main-container">
                    <div class="left-panel">
                        <button id="calibrate-btn" class="uncalibrated" onclick="calibrateSystem();fetch('/calibrate').catch(()=>{}); return false;">CALIBRATE</button>
                        <div>
                            <div class="controls-row">
                                <div class="control-group">
                                    <input type="range" id="servo" min="-30" max="60" value="0" oninput="document.getElementById('servoVal').innerText = this.value; updateServoValue(this.value);">
                                    <div class="control-label">Altitude: <span id="servoVal">0</span>°</div>
                                </div>
                                <div class="control-group">
                                    <input type="range" id="stepper" min="0" max="360" value="0" oninput="document.getElementById('stepperVal').innerText = this.value; updateStepperValue(this.value);">
                                    <div class="control-label">Azimuth: <span id="stepperVal">0</span>°</div>
                                </div>
                            </div>
                            <br>
                            <div class="rotate-buttons">
                                <button class="rotate-btn" onclick="sendServoAngle()">ROTATE SERVO</button>
                                <button class="rotate-btn" onclick="sendStepperAngle();">ROTATE STEPPER</button>
                            </div>
                        </div>
                        <div class="skymap-wrapper">
                            <div id="av-skymap" style="position: relative; width: 900px; height: 900px;">
                                <script>
                                var avSkymapProperties = {
                                    location: { lon: 23.322, lat: 42.698, name: 'Sofia', tz: 'Europe/Sofia' },
                                    deco: 16399, size: 900, bgColor: '#000000', lang: 'en'
                                };
                                </script>
                                <script src="https://www.astroviewer.net/widgets/widgets/skymap.js"></script>
                            </div>
                        </div>
                    </div>
                    <div class="right-panel">
                        <div class="compass-container">
                            <button class="compass-btn btn-n" onclick="fetch('/rotateStepper?angle=0').catch(()=>{});">N</button>
                            <button class="compass-btn btn-e" onclick="fetch('/rotateStepper?angle=90').catch(()=>{});">E</button>
                            <button class="compass-btn btn-s" onclick="fetch('/rotateStepper?angle=180').catch(()=>{});">S</button>
                            <button class="compass-btn btn-w" onclick="fetch('/rotateStepper?angle=270').catch(()=>{});">W</button>
                            <div style="position:absolute; width: 100%; height: 2px; background: #4488ff; top: 50%; opacity: 0.3;"></div>
                            <div style="position:absolute; height: 100%; width: 2px; background: #4488ff; left: 50%; opacity: 0.3;"></div>
                        </div>
                        <div class="bodies-list">
                            <button class="body-btn" onclick="getCoords('sun')">Sun</button>
                            <button class="body-btn" onclick="getCoords('moon')">Moon</button>
                            <button class="body-btn" onclick="getCoords('mercury')">Mercury</button>
                            <button class="body-btn" onclick="getCoords('venus')">Venus</button>
                            <button class="body-btn" onclick="getCoords('mars')">Mars</button>
                            <button class="body-btn" onclick="getCoords('jupiter')">Jupiter</button>
                            <button class="body-btn" onclick="getCoords('saturn')">Saturn</button>
                            <button class="body-btn" onclick="getCoords('uranus')">Uranus</button>
                            <button class="body-btn" onclick="getCoords('neptune')">Neptune</button>
                        </div>
                    </div>
                </div>
                <script>
                function calibrateSystem() {
                    const btn = document.getElementById('calibrate-btn');
                    btn.classList.remove('uncalibrated');
                    btn.classList.add('calibrated');
                    btn.innerText = "CALIBRATE";
                }
                function getCoords(body) {
                    const xhr = new XMLHttpRequest();
                    xhr.onreadystatechange = function() {
                        if (this.readyState === XMLHttpRequest.DONE) {
                            try {
                                const json = JSON.parse(this.responseText);
                                const foundBody = json.data.table.rows.find(row => row.entry.name.toLowerCase() === body.toLowerCase());
                                if (foundBody) {
                                    const alt = Math.round(foundBody.cells[0].position.horizontal.altitude.degrees);
                                    const az = Math.round(foundBody.cells[0].position.horizontal.azimuth.degrees);
                                    updateServoValue(alt);
                                    updateStepperValue(az);
                                }
                            } catch (err) {}
                        }
                    };
                    )=====");
            
            client.println("xhr.open('GET','https://api.astronomyapi.com/api/v2/bodies/positions?longitude=-84.39733&latitude=33.775867&elevation=1&from_date=" + date + "&to_date=" + date + "&time=" + time + "');");

            client.println(R"=====(
                    xhr.setRequestHeader('Authorization', 'Basic NzU1MWFkOTItMWFlZC00OGZkLWFjOTgtMmY3ODA0ZmNlYjJkOmQ4MDg5MWY3MDU4YmI3ZWFkYzIzY2QzNWZhNzg0YWVmNDZmNmIzNTgwMjNkYzIwYTBjODU1MzljMTY5NzcxM2U5ZTFmZGJjM2RkMmFkMmY2OWE3YzM4MTM4NDFiNTE4YjMxN2VlNDc5ZDAxNzJmZTVlMzlhN2M1NzEwZTQyZWI0NGMzYzBkODRiMWZkMmNhY2RhYzBkZjhiMWZhZmMwNjEzOWEwNGExZDA0NWVkZTI3NTVlYTFmZDI4ZWZmMzZkOTUxYTM0MzVkMDA1YzE2N2QxZTg5MGVkMTFjMjkwY2Ey');
                    xhr.send();
                }
                let currentServoValue = 0;
                let currentStepperValue = 0;
                function updateServoValue(v) { currentServoValue = v; document.getElementById('servoVal').innerHTML = v; document.getElementById('servo').value = v; }
                function updateStepperValue(v) { currentStepperValue = v; document.getElementById('stepperVal').innerHTML = v; document.getElementById('stepper').value = v; }
                function sendServoAngle() { fetch('/rotateServo?angle=' + currentServoValue).catch(() => {}); }
                function sendStepperAngle() { fetch('/rotateStepper?angle=' + currentStepperValue).catch(() => {}); }
                </script>
            </body>
            </html>
            )=====");

            client.println();
            break;
          } else {
            // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {
          // if you got anything else but a carriage return character, add it to the end of the currentLine
          currentLine += c;
        }
      }
    }
    
    http_request = "";
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }

  if (pendingServoAngle != -999) {
    alt = (float)pendingServoAngle; // Update global alt for display
    rotateServo(pendingServoAngle);
    pendingServoAngle = -999;
    return;
  }

  if (pendingStepperAngle != -999) {
    rotateStepper((float)pendingStepperAngle);
    pendingStepperAngle = -999;
    return;
  }
}
