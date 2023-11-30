#include <Arduino.h>
#include <Wire.h>  
#include <WiFi.h>
#include <Adafruit_GFX.h>
// install lib berikut di platformio (buka terminal dari menu platformio, jalankan pio lib -g install https://github.com .....)
#include <Adafruit_SH1106.h> //https://github.com/nhatuan84/esp32-sh1106-oled

#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "SPIFFS.h"

#define CHN_MTRDC 1
#define CHN_MTRDCR 2
#define CHN_MTRSRV 4
#define PWMRES 8

#define MLENCA 33
#define MLENCB 35
#define MRENCA 32
#define MRENCB 34

#define EN1 25
#define EN2 26

//#define MLEN 13
#define MLIN1 17 //5
#define MLIN2 18
//#define MREN 27
#define MRIN1 19
#define MRIN2 23

#define SSA 15
#define SSB 4
#define SSC 16
#define SSD 5 // 17
#define SZ 39

#define R 0.03
#define W 0.17

unsigned long nowtime;

Adafruit_SH1106 display(SDA, SCL);
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String ssid="loop-1";
String password="12345678";
IPAddress IP_ap = {192, 168, 1, 1};
IPAddress gateway_ap = {192, 168, 1, 1};
IPAddress NMask_ap = {255, 255, 255, 0};

int btn1,btn2,btn3;

int base = 0;
int dutycycle = 0;
int mtrSRV_freq = 50;
int mtrSRV_res = 16;
int mtrDC_freq = 50;
int mtrDC_res = 8;
int mtrDC_dutycycle = 0;
int mtrDC_dutycycleR = 0;
int mtrSRV_dutycycle = 0;
int mtrSRV_deg = 0;
int delayControl = 1000;
long countL=0,countR=0;

float integral_err = 0;
float integral_err_old = 0;
float deriv_err = 0;
float deriv_err_old = 0;
float err_old = 0;
float control = 0;

float t = 0;
float dt = 0.001;

// TUNING
float kp = 150/2.5;
float ki = 20/2.5;
float kd = 10/2.5;

int mode_run = 0;

bool running=false;

float err=0;

byte bacasensor(int index) {
  // if (index==8) index=16; 
  // else if(index==16) index = 8;
  digitalWrite(SSA, (index & 0x01));
  digitalWrite(SSB, ((index>>1) & 0x01));
  digitalWrite(SSC, ((index>>2) & 0x01));
  digitalWrite(SSD, ((index>>3) & 0x01));
  return (digitalRead(SZ))? 1:0;
}

void SetServoPos(float pos)
{
    uint32_t duty = (((pos/180.0)
              *2000)/20000.0*65536.0) + 1634;
         // convert 0-180 degrees to 0-65536

    ledcWrite(CHN_MTRSRV,duty);
        // set channel to pos
}

void handleWeb( void * pvParameters ){
  printf("handleWeb running on core %d\n",xPortGetCoreID()); // on Serial
  int i;

  //  Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("request web /");
    request->send(SPIFFS, "/index.html", "text/html",false);
  });
  server.on("/pavicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "Not found");
  });
  server.on("/nama", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "ABDUL MUIS");
  });
  server.on("/lastdata", HTTP_GET, [](AsyncWebServerRequest *request){
    String sensor="";
    err = 0;
    int baca=0;
    int temp =0;
    int n=0;
    for (int i=15; i>=0; i--) 
    { temp = bacasensor(i);
      if (temp) {
        n++;
        baca +=i+1;
      }
      // if ((i==7)||(i==15)) baca=0;
      sensor=sensor+" "+String(bacasensor(i));
    }
    err = (float)baca/(float)n - 8.5;

    StaticJsonDocument<2000> myArray;
    myArray["sensor"]=sensor;
    myArray["err"]=err;
    myArray["control"]=control;
    myArray["left"]["dutycycle"]=mtrDC_dutycycle;
    myArray["right"]["dutycycle"]=mtrDC_dutycycleR;
    myArray["left"]["enc"]=countL;
    myArray["right"]["enc"]=countR;
    
    String jsonString;
    serializeJson(myArray, jsonString);

    request->send(200, "application/json", jsonString);
  });
  server.on("/update", HTTP_PUT, [](AsyncWebServerRequest *request){
    if (request->hasParam("delay")) {
         delayControl = request->getParam("delay")->value().toInt();
         request->send(200, "text/plain", "Berhasil update delay "+String(delayControl));
    }
    if (request->hasParam("stop")) {
         running=false;
         request->send(200, "text/plain", "Berhasil stop ");
    }
    if (request->hasParam("start")) {
         running=true;
         request->send(200, "text/plain", "Berhasil start ");
    }
    if (request->hasParam("run")) {
         mode_run = request->getParam("mode")->value().toInt();
         running=true;
         request->send(200, "text/plain", "Mulai Jalan ");
    }
    if (request->hasParam("srvdeg")) {
         mtrSRV_deg = request->getParam("srvdeg")->value().toInt();
         SetServoPos(mtrSRV_deg);
         request->send(200, "text/plain", "Berhasil update servo Deg "+String(mtrSRV_deg));
    }
    if (request->hasParam("dcfreq")) {
         mtrDC_freq = request->getParam("dcfreq")->value().toInt();
         ledcWriteTone(CHN_MTRDC, mtrDC_freq);
         ledcWrite(CHN_MTRDC, mtrDC_dutycycle);
         ledcWrite(CHN_MTRDCR, mtrDC_dutycycleR);
         request->send(200, "text/plain", "Berhasil update dcfreq "+String(mtrDC_freq));
    }
    if (request->hasParam("dcdutycycle")) {
         dutycycle = request->getParam("dcdutycycle")->value().toInt();       
         request->send(200, "text/plain", "Berhasil update dcdutycycle "+String(dutycycle));
    }
    if (request->hasParam("dcdutycycleR")) {
         mtrDC_dutycycleR = request->getParam("dcdutycycleR")->value().toInt();
         ledcWrite(CHN_MTRDCR, mtrDC_dutycycleR);
         request->send(200, "text/plain", "Berhasil update dcdutycycleR "+String(mtrDC_dutycycleR));
    }
    request->send(200, "text/plain", "Tidak ada yang di update");
  });

  server.serveStatic("/", SPIFFS, "/");  
  server.onNotFound([](AsyncWebServerRequest *request){ request->send(404, "text/plain", "Not found"); });

  // Start ElegantOTA
  AsyncElegantOTA.begin(&server);

  server.begin();
  for(;;){
    delay(1000);
  }
}


void IRAM_ATTR encoderleft() {
  // countL++;return;
 if (digitalRead(MLENCB))
    countL += digitalRead(MLENCA)? 1:-1 ;
 else 
    countL -= digitalRead(MLENCA)? 1:-1 ;
}
void IRAM_ATTR encoderright() {
  // countR++;return;
 if (digitalRead(MRENCB))
    countR += digitalRead(MRENCA)? 1:-1 ;
 else 
    countR -= digitalRead(MRENCA)? 1:-1 ;
}

void inverse_jacobian(float &dqr, float &dql, float dx,float dy, float dshi, float shi) {
	dqr=1.0/R*(cos(shi)*dx + sin(shi)*dy +W/2.0*dshi);
	dql=1.0/R*(cos(shi)*dx + sin(shi)*dy -W/2.0*dshi);
}

void jacobian(float &dx, float &dy, float &dshi, float dqr, float dql, float shi) {
	dx=R/2.0*cos(shi)*(dqr+dql);
	dy=R/2.0*sin(shi)*(dqr+dql);
	dshi=R/W*(dqr-dql);
}


void controlLoop( void * pvParameters ){
  printf("controlLoop running on core %d\n",xPortGetCoreID()); // on Serial
  // setup PWM motor
  
  // init PWM kontrol motor kiri
  digitalWrite(MLIN2, LOW);
  ledcSetup(CHN_MTRDC, mtrDC_freq, mtrDC_res);
  ledcAttachPin(MLIN1, CHN_MTRDC);
  ledcWrite(CHN_MTRDC, mtrDC_dutycycle);
  
  // init PWM kontrol motor kanan
  digitalWrite(MRIN1, LOW);
  ledcSetup(CHN_MTRDCR, mtrDC_freq, mtrDC_res);
  ledcAttachPin(MRIN2, CHN_MTRDCR);
  ledcWrite(CHN_MTRDCR, mtrDC_dutycycleR);

  mtrDC_dutycycleR = 100;
  mtrDC_dutycycle = 100;

  ledcWrite(CHN_MTRDC, mtrDC_dutycycle);
  ledcWrite(CHN_MTRDCR, mtrDC_dutycycleR);

  for(;;){
    float errsp = 0;
    int baca=0;
    int temp =0;
    int n=0;
    for (int i=15; i>=0; i--) 
    { temp = bacasensor(i);
      if (temp) {
        n++;
        baca +=i+1;
      }
    }
    errsp = (float)baca/((float)n+0.001) - 8.5;

    err = errsp;

    // PID ==>  SP - PV -> PID -> CV -> Plant -> PV
    
    integral_err = integral_err_old + err * dt;
    
    integral_err_old = integral_err;

    deriv_err = (err - err_old)/dt;
    err_old = err;

    control = kp * err + ki * integral_err + kd * deriv_err;
    
    if (control > base-50){
      control = base-51;
    }

    if (control < -1*base+50){
      control = -1*base+51;
    }

    if(running = true){

    if(err >= 1 || err <= -1){
      base = 125;
      mtrDC_dutycycle = base + control;
      mtrDC_dutycycleR = (base - control);
    }
    else{
      mtrDC_dutycycle = (base+55 - abs(control));
      mtrDC_dutycycle = (base+55 - abs(control));
      if (control > 0){
        control = control - 10;
      }
      else{
        control = control + 10;
      }
    }

    if (mtrDC_dutycycle > 255){
      mtrDC_dutycycle = 255;
    }
    else{
      mtrDC_dutycycle = mtrDC_dutycycle;
    }

    if (mtrDC_dutycycleR > 255){
      mtrDC_dutycycleR = 255;
    }
    else {
      mtrDC_dutycycleR = mtrDC_dutycycleR;
    }
    
    ledcWrite(CHN_MTRDC, mtrDC_dutycycle*0.995);
    ledcWrite(CHN_MTRDCR, mtrDC_dutycycleR);
    
    
    if (millis() - nowtime > 500){
    nowtime = millis();
    Serial.print("c:");
    Serial.print(control);
    Serial.print("e:");
    Serial.print(err);
    Serial.print(", L:");
    Serial.print(mtrDC_dutycycle);
    Serial.print(", R:");
    Serial.println(mtrDC_dutycycleR);
    }

      //  if ((mode_run==1)&&(t<Tfinal)) {
      //   x_d=(x_final-x_awal)*t/Tfinal + x_awal;
      //   y_d=(y_final-y_awal)*t/Tfinal + y_awal;
      //   dx_d=(x_final-x_awal)/Tfinal; // kecepatan / gradient
      //   dy_d=(y_final-y_awal)/Tfinal;
      //   shi_d = atan2(dy_d,dx_d);
      //   dshi_ref= Kp/2.0 * (shi_d - shi);

      //   dx_ref = Kp*(x_d - x) ;
		  //   dy_ref = Kp*(y_d - y) ;

      //   inverse_jacobian(dqr_ref, dql_ref, dx_ref, dy_ref, dshi_ref, shi);

      //   mtrDC_dutycycle = (dql_ref>255) ? 255:dql_ref;
      //   mtrDC_dutycycleR = (dqr_ref>255) ? 255:dqr_ref;

		  //   jacobian(dx,dy, dshi, dqr, dql, shi);
		  //   // hitung x,y
		  //   v=dx*cos(shi)+dy*sin(shi);
      //   dx = dx_ref;
      //   dy = dy_ref;
      //   qr = countR * 0.5;
      //   ql = countL * 0.5;
		  //   shi=R/W*(qr-ql)+shi_init;
		  //   x=x+v*cos((shi+shi_old)/2.0)*dt;
		  //   y=y+v*sin((shi+shi_old)/2.0)*dt; 

      //  }

    vTaskDelay(dt/portTICK_PERIOD_MS);
    }

  else {
    ledcWrite(CHN_MTRDC, 0);
    ledcWrite(CHN_MTRDCR, 0);
  }
  }
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

void setup() {
  Serial.begin(115200);
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)

  initSPIFFS();
  WiFi.mode(WIFI_MODE_AP);
  uint64_t chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes).
  uint16_t chip = (uint16_t)(chipid >> 32);
  ssid = "ALAT-" + String(chip,HEX);
  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAP(ssid.c_str(), password.c_str(), 3,0,11); //ssid, passwd, ch, hidden, maxconn
  WiFi.softAPConfig(IP_ap, IP_ap, NMask_ap);
   
  // declare pin motor
  pinMode(MLIN1,OUTPUT);
  pinMode(MLIN2,OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(MRIN1,OUTPUT);
  pinMode(MRIN2,OUTPUT);
  pinMode(EN2, OUTPUT);

  digitalWrite(EN1,HIGH);
  digitalWrite(EN2,HIGH);

  // declare pin encoder
  pinMode(MLENCA,INPUT);
  pinMode(MLENCB,INPUT);
  pinMode(MRENCA,INPUT);
  pinMode(MRENCB,INPUT);

  // baca LINE deklarasi mux
   pinMode(SSA,OUTPUT);
   pinMode(SSB,OUTPUT);
   pinMode(SSC,OUTPUT);
   pinMode(SSD,OUTPUT);
   pinMode(SZ,INPUT);

  // attach encoder interrupt (odometry)
  attachInterrupt(MLENCA, encoderleft, CHANGE);
  attachInterrupt(MRENCA, encoderright, CHANGE);
  
  // send Torque Reference to CANbus
  xTaskCreatePinnedToCore(
      handleWeb, /* Function to implement the task */
      "handleWeb", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      NULL,  /* Task handle. */
      1); /* Core where the task should run */
  delay(1000);  // send Torque Reference to CANbus
  xTaskCreatePinnedToCore(
      controlLoop, /* Function to implement the task */
      "controlLoop", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      NULL,  /* Task handle. */
      0); /* Core where the task should run */
  delay(1000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  display.clearDisplay();
  display.setTextSize(1.1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,0);display.println("Status Counter");
  display.setCursor(0,20);display.println(String(countL)+" "+String(countR));
  display.display();
  delay(1000);
}
