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

#define QMC_5883L_MODE 1
#define QMC_5883L_ODR 2
#define QMC_5883L_RNG 4
#define QMC_5883L_OSR 6

#define QMC_5883L_MODE_STANDBY 0 << QMC_5883L_MODE
#define QMC_5883L_MODE_CONTINUOUS 1 << QMC_5883L_MODE

#define QMC_5883L_ODR_10HZ 0 << QMC_5883L_ODR
#define QMC_5883L_ODR_50HZ 1 << QMC_5883L_ODR
#define QMC_5883L_ODR_100HZ 10 << QMC_5883L_ODR
#define QMC_5883L_ODR_200HZ 11 << QMC_5883L_ODR

#define QMC_5883L_RNG_2G 0 << QMC_5883L_RNG
#define QMC_5883L_RNG_8G 1 << QMC_5883L_RNG

#define QMC_5883L_OSR_64 11 << QMC_5883L_OSR
#define QMC_5883L_OSR_128 10 << QMC_5883L_OSR
#define QMC_5883L_OSR_256 1 << QMC_5883L_OSR
#define QMC_5883L_OSR_512 0 << QMC_5883L_OSR

#define SERVO_OFFSET 84

// 128x64 res, first 16 rows - yellow
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, 12, 14, U8X8_PIN_NONE);
String statusString = "";

AccelStepper stepper(AccelStepper::HALF4WIRE, STEPPER_OUTPUT1, STEPPER_OUTPUT3, STEPPER_OUTPUT2, STEPPER_OUTPUT4);
int16_t min_x, max_x;
int16_t min_y, max_y;
float hic_x, hic_y;

Servo servo;

const char* ssid = "VIVACOM_AAF7";
const char* password = "nQhrwXce6Wh4";

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

  // soft reset for clean start
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_CONTROL_2_REG);
  Wire.write(1 << QMC_5883L_SOFT_RST);
  Wire.endTransmission();
  
  delay(10); 

  //define set/reset period
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_SETRESET_PERIOD_REG);
  Wire.write(0x01);
  Wire.endTransmission();

  delay(10);

  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_CONTROL_1_REG);
  Wire.write(QMC_5883L_MODE_CONTINUOUS | QMC_5883L_ODR_200HZ | QMC_5883L_RNG_8G | QMC_5883L_OSR_512);
  Wire.endTransmission();
  
  delay(100);
}

void readCompass(int16_t &x, int16_t&y, int16_t&z)
{
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

  //heading += 5.6833;
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;

  return heading;
}

void calibrateCompass() {
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

void rotateStepper(float targetDeg) {
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

void display_prepare(void) {
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
  display.begin();

  pinMode(STEPPER_OUTPUT1, OUTPUT);
  pinMode(STEPPER_OUTPUT2, OUTPUT);
  pinMode(STEPPER_OUTPUT3, OUTPUT);
  pinMode(STEPPER_OUTPUT4, OUTPUT);

  compass_init();

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
  delay(10);

  WiFiClient client = server.available();

  if (client) {
    Serial.println("New Client.");
    // incoming data from the client
    String currentLine = "";

    while (client.connected()) {

      // read if there's bytes to read from the client
      if (client.available()) {
        char c = client.read();
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
              alt = extractAngle(http_request);
              rotateServo(alt);
            }
            else if (http_request.indexOf("GET /rotateStepper?angle=") >= 0) {
              int angle = extractAngle(http_request);
              rotateStepper(angle);
            }

            client.println("<!DOCTYPE html><html>");
            client.println("<head></head><body>");
            client.println("<h1>zdrasl");
            client.println("<a href=\"#\" onclick=\"fetch('/calibrate').catch(()=>{}); return false;\">[CALIBRATE]</a>");
            
            client.println("<a href=\"#\" onclick=\"fetch('/rotateStepper?angle=0').catch(()=>{}); return false;\">[NORTH]</a>");
            client.println("<a href=\"#\" onclick=\"fetch('/rotateStepper?angle=180').catch(()=>{}); return false;\">[SOUTH]</a>");
            client.println("<a href=\"#\" onclick=\"fetch('/rotateStepper?angle=90').catch(()=>{}); return false;\">[EAST]</a>");
            client.println("<a href=\"#\" onclick=\"fetch('/rotateStepper?angle=270').catch(()=>{}); return false;\">[WEST]</a>");

            client.println("<input type=\"range\" id=\"slider\" min=\"-30\" max=\"60\" value=\"0\" ");
            client.println("oninput=\"updateServoValue(this.value)\">");

            client.println("<input type=\"range\" id=\"slider\" min=\"0\" max=\"360\" value=\"0\" ");
            client.println("oninput=\"updateStepperValue(this.value)\">");

            client.println("<p>Servo: <span id=\"servoVal\">0</span></p>");
            client.println("<p>Stepper: <span id=\"stepperVal\">0</span></p>");

            client.println("<p><button onclick=\"var coords = getCoords('sun')\">Sun</button></p>");
            client.println("<p><button onclick=\"var coords = getCoords('moon')\">Moon</button></p>");
            client.println("<p><button onclick=\"var coords = getCoords('mercury')\">Mercury</button></p>");
            client.println("<p><button onclick=\"var coords = getCoords('venus')\">Venus</button></p>");
            client.println("<p><button onclick=\"var coords = getCoords('mars')\">Mars</button></p>");
            client.println("<p><button onclick=\"var coords = getCoords('jupiter')\">Jupiter</button></p>");
            client.println("<p><button onclick=\"var coords = getCoords('saturn')\">Saturn</button></p>");
            client.println("<p><button onclick=\"var coords = getCoords('uranus')\">Uranus</button></p>");
            client.println("<p><button onclick=\"var coords = getCoords('neptune')\">Neptune</button></p>");

            client.println("<button onclick=\"sendServoAngle()\">Servo</button>");
            client.println("<button onclick=\"sendStepperAngle()\">Stepper</button>");

            client.println("<div id=\"av-skymap\" style=\"position: relative; width: 1000px; height: 1000px;\">");
            client.println("<script>");
            client.println("var avSkymapProperties = {");
            client.println("location: {");
            client.println("lon:  23.322,");
            client.println("lat:  42.698,");
            client.println("name: 'Sofia',");
            client.println("tz:   'Europe/Sofia'");
            client.println("},");
            client.println("deco: 16399,");
            client.println("size: 1000,");
            client.println("bgColor: '#ffffff',");
            client.println("lang: 'en'");
            client.println("};");
            client.println("</script>");
            client.println("<script src=\"https://www.astroviewer.net/widgets/widgets/skymap.js\"></script>");
            client.println("</div>");

            client.println("<script>");
            client.println("function getCoords(body){");
            client.println("const authString=btoa(\"7551ad92-1aed-48fd-ac98-2f7804fceb2d:d80891f7058bb7eadc23cd35fa784aef46f6b358023dc20a0c85539c1697713e9e1fdbc3dd2ad2f69a7c3813841b518b317ee479d0172fe5e39a7c5710e42eb44c3c0d84b1fd2cacdac0df8b1fafc06139a04a1d045ede2755ea1fd28eff36d951a3435d005c167d1e890ed11c290ca2\");");
            client.println("document.addEventListener(\"DOMContentLoaded\",function(){var client=new AstronomyAPI({basicToken:authString});client.moonPhase();});");
            client.println("const xhr=new XMLHttpRequest();");
            client.println("xhr.onreadystatechange=function(){");
            client.println("if(this.readyState===XMLHttpRequest.DONE){");
            client.println("try{");
            client.println("const json=JSON.parse(this.responseText);");
            client.println("const foundBody=json.data.table.rows.find(row=>row.entry.name.toLowerCase()===body.toLowerCase());");
            client.println("if(foundBody){");
            client.println("const result={altitude:foundBody.cells[0].position.horizontal.altitude.degrees,azimuth:foundBody.cells[0].position.horizontal.azimuth.degrees};");
            client.println("console.log(\"Coordinates:\",result);");
            client.println("updateServoValue(Math.round(result.altitude));");
            client.println("updateStepperValue(Math.round(result.azimuth));");
            client.println("return result;");
            client.println("}else{console.log(\"Body not found:\",body);}");
            client.println("}catch(err){console.error(\"Failed to parse JSON:\",err);}");
            client.println("}");
            client.println("};");

            String date = String(t_year) + "-" + (t_month <= 9 ? "0" : "") + String(t_month) + "-" + (t_day <= 9 ? "0" : "") +String(t_day);
            String time = (t_hour <= 9 ? "0" : "") + String(t_hour) + ":" + (t_min <= 9 ? "0" : "") + String(t_min) + ":" + (t_sec <= 9 ? "0" : "") + String(t_sec);

            client.println("xhr.open('GET','https://api.astronomyapi.com/api/v2/bodies/positions?longitude=-84.39733&latitude=33.775867&elevation=1&from_date=" + date + "&to_date=" + date + "&time=" + time + "');");
            client.println("xhr.setRequestHeader('Authorization','Basic NzU1MWFkOTItMWFlZC00OGZkLWFjOTgtMmY3ODA0ZmNlYjJkOmQ4MDg5MWY3MDU4YmI3ZWFkYzIzY2QzNWZhNzg0YWVmNDZmNmIzNTgwMjNkYzIwYTBjODU1MzljMTY5NzcxM2U5ZTFmZGJjM2RkMmFkMmY2OWE3YzM4MTM4NDFiNTE4YjMxN2VlNDc5ZDAxNzJmZTVlMzlhN2M1NzEwZTQyZWI0NGMzYzBkODRiMWZkMmNhY2RhYzBkZjhiMWZhZmMwNjEzOWEwNGExZDA0NWVkZTI3NTVlYTFmZDI4ZWZmMzZkOTUxYTM0MzVkMDA1YzE2N2QxZTg5MGVkMTFjMjkwY2Ey');");
            client.println("xhr.send();");
            client.println("}");
            
            client.println("let currentServoValue = 0;");
            client.println("let currentStepperValue = 0;");

            client.println("function updateServoValue(v){");
            client.println("  currentServoValue = v;");
            client.println("  document.getElementById('servoVal').innerHTML = v;");
            client.println("}");
            
            client.println("function updateStepperValue(v){");
            client.println("  currentStepperValue = v;");
            client.println("  document.getElementById('stepperVal').innerHTML = v;");
            client.println("}");

            client.println("function sendServoAngle(){");
            client.println("  fetch('/rotateServo?angle=' + currentServoValue)");
            client.println(".catch(() => {});");
            client.println("}");
            
            client.println("function sendStepperAngle(){");
            client.println("  fetch('/rotateStepper?angle=' + currentStepperValue)");
            client.println(".catch(() => {});");
            client.println("}");

            client.println("</script>");

            client.println("</body>");
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
}
