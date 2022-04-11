#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <ESP8266httpUpdate.h>
#include <EEPROM.h> //导入Flash库文件
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include "DHT.h"
#define DHTTYPE DHT11

String X = "1.98";

// WiFi
const char *ssidp = "Mi 10"; // Enter your WiFi name
const char *password = "bisheng1579";  // Enter WiFi password
String ssid;
String psw;

String upUrl = "http://bin.bemfa.com/b/3BcMjJjZTA1NjNmMjI2YWFhYzljZjBhN2RiMGZmN2NlZjE=dogwitchA.bin";
//巴法云服务器地址默认即可
#define TCP_SERVER_ADDR "bemfa.com"
//服务器端口//TCP创客云端口8344//TCP设备云端口8340
#define TCP_SERVER_PORT "8344"

//用户私钥，可在控制台获取,修改为自己的UID
String UID = "22ce0563f226aaac9cf0a7db0ff7cef1";
//主题名字，可在控制台新建
String TOPIC = "dogwitchA"; 
int DHTPIN = 13;  //连接dht11的引脚

//设置上传速率2s（1s<=upDataTime<=60s）
//下面的2代表上传间隔是2秒
#define upDataTime 2*1000

struct config_type
{
  char stassid[32];//定义配网得到的WIFI名长度(最大32字节)
  char stapsw[64];//定义配网得到的WIFI密码长度(最大64字节)
};

config_type config;//声明定义内容



// for DHT11, 
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 2
DHT dht(DHTPIN, DHTTYPE);
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
uint32_t irBuffer[50]; //infrared LED sensor data
uint32_t redBuffer[50];  //red LED sensor data
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

//最大字节数
#define MAX_PACKETSIZE 512
/*********************************************************************/
/*********************************************************************/
//tcp客户端相关初始化，默认即可
WiFiClient TCPclient;
String TcpClient_Buff = "";
unsigned int TcpClient_BuffIndex = 0;
unsigned long TcpClient_preTick = 0;
unsigned long preHeartTick = 0;//心跳
unsigned long preTCPStartTick = 0;//连接
bool preTCPConnected = false;
/*********************************************************************/
/*********************************************************************/
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int32_t OS;
int beatAvg;
long irValue;
float temperatureMAX;

float t;
float h;
float f;
float hif;
float hic;


//相关函数初始化
//连接WIFI
void doWiFiTick();
void startSTA();
void updateBin();
void saveConfig();
void loadConfig();

//TCP初始化连接
void doTCPClientTick();
void startTCPClient();
void sendtoTCPServer(String p);

//传感器函数
 void temppts();
void  FUUTS();
void MAXSTOP();
void readMAX();
void MAXSET();
void MAXMAIN();
void MAXTMP();
void SPO();
/*
  *发送数据到TCP服务器
 */
void sendtoTCPServer(String p){
  
  if (!TCPclient.connected()) 
  {
    Serial.println("Client is not readly");
    return;
  }
  TCPclient.print(p);
  Serial.println("[Send to TCPServer]:String");
  Serial.println(p);
}


/*
  *初始化和服务器建立连接
*/
void startTCPClient(){
  if(TCPclient.connect(TCP_SERVER_ADDR, atoi(TCP_SERVER_PORT))){
    Serial.print("\nConnected to server:");
    Serial.printf("%s:%d\r\n",TCP_SERVER_ADDR,atoi(TCP_SERVER_PORT));
    String tcpTemp="";
    tcpTemp = "cmd=1&uid="+UID+"&topic="+TOPIC+"\r\n";

    sendtoTCPServer(tcpTemp);
    preTCPConnected = true;
    preHeartTick = millis();
    TCPclient.setNoDelay(true);
  }
  else{
    Serial.print("Failed connected to server:");
    Serial.println(TCP_SERVER_ADDR);
    TCPclient.stop();
    preTCPConnected = false;
  }
  preTCPStartTick = millis();
}



/*
  *检查数据，发送数据
*/
void doTCPClientTick(){
 //检查是否断开，断开后重连
   if(WiFi.status() != WL_CONNECTED) return;

  if (!TCPclient.connected()) {//断开重连

  if(preTCPConnected == true){

    preTCPConnected = false;
    preTCPStartTick = millis();
    Serial.println();
    Serial.println("TCP Client disconnected.");
    TCPclient.stop();
  }
  else if(millis() - preTCPStartTick > 1*1000)//重新连接
    startTCPClient();
  }
  else
  {
    if (TCPclient.available()) {//收数据
      char c =TCPclient.read();
      TcpClient_Buff +=c;
      TcpClient_BuffIndex++;
      TcpClient_preTick = millis();
      
      if(TcpClient_BuffIndex>=MAX_PACKETSIZE - 1){
        TcpClient_BuffIndex = MAX_PACKETSIZE-2;
        TcpClient_preTick = TcpClient_preTick - 200;
      }
      preHeartTick = millis();
    }
    if(millis() - preHeartTick >= upDataTime){//上传数据
      preHeartTick = millis();
/*****************获取数据*****************/
temppts();

   
      /*********************数据上传*******************/
      /*
        数据用#号包裹，以便app分割出来数据，&msg=#23#80#on#\r\n，即#温度#湿度#按钮状态#，app端会根据#号分割字符串进行取值，以便显示
        如果上传的数据不止温湿度，可在#号后面继续添加&msg=#23#80#data1#data2#data3#data4#\r\n,app字符串分割的时候，要根据上传的数据进行分割
      */
      String upstr = "";
      upstr = "cmd=2&uid="+UID+"&topic="+TOPIC+
      "&msg=#"+t+"#"+h+"#"+f+"#"+hif+"#"+hic+"#"
      +irValue+"#"+beatAvg+"#"+OS+"#"
      +temperatureMAX+"#"+X+"#"+"\r\n";
      sendtoTCPServer(upstr);
      upstr = "";
    }
  }
  
  if((TcpClient_Buff.length() >= 1) && (millis() - TcpClient_preTick>=200))
  {//data ready
    TCPclient.flush();
    Serial.println("Buff");
    Serial.println(TcpClient_Buff);
    //////字符串匹配，检测发了的字符串TcpClient_Buff里面是否包含&msg=on，如果有，则打开开关
    if((TcpClient_Buff.indexOf("&msg=update") > 0)) {
     updateBin();
    //////字符串匹配，检测发了的字符串TcpClient_Buff里面是否包含&msg=off，如果有，则关闭开关
    }
    else if((TcpClient_Buff.indexOf("&msg=onA") > 0)) {
      readMAX();
    }
    else if((TcpClient_Buff.indexOf("&msg=TEP") > 0)) {
      MAXTMP();
    }
    else if((TcpClient_Buff.indexOf("&msg=FUS") > 0)) {
      FUUTS();
    }
 else if((TcpClient_Buff.indexOf("&msg=SPO") > 0)) {
      SPO();
    }
 
   TcpClient_Buff="";//清空字符串，以便下次接收
   TcpClient_BuffIndex = 0;
  }
  
}

void startSTA(){
  //int s=0;
  WiFi.disconnect();
    WiFi.mode(WIFI_STA);
     loadConfig();//读取ROM是否包含密码
  if(ssid!=0&&psw!=0){
    WiFi.begin(ssid,psw);//如果有密码则自动连接
while (WiFi.status() != WL_CONNECTED)
  {
    WiFi.persistent(false);
     WiFi.beginSmartConfig();
      while(1)
  {
    Serial.print(".");
    delay(500);
 //WiFi.begin(ssid, password);
    
    if (WiFi.smartConfigDone())
    {
      WiFi.persistent(true);
      strcpy(config.stassid,WiFi.SSID().c_str());//名称复制
      strcpy(config.stapsw,WiFi.psk().c_str());//密码复制
      saveConfig();//调用保存函数
      WiFi.setAutoConnect(true);  // 设置自动连接
      break;
    }
    /*
    else if(s>=120&&WiFi.status() != WL_CONNECTED){
      for(int i=0;i<20;i++){
    WiFi.begin(ssid, password);
    Serial.print(".");
    delay(500);
    if(WiFi.status() == WL_CONNECTED){
      break;
      }
      }
      s=0;
    break;
      }
      s++;
      */
  }
  break;
  }
  }else{
    WiFi.persistent(false);
    WiFi.beginSmartConfig();
      while(1)
  {
    Serial.print(".");
    delay(500);
 //WiFi.begin(ssid, password);
    
    if (WiFi.smartConfigDone())
    {
      WiFi.persistent(true);
      strcpy(config.stassid,WiFi.SSID().c_str());//名称复制
      strcpy(config.stapsw,WiFi.psk().c_str());//密码复制
      saveConfig();//调用保存函数
      WiFi.setAutoConnect(true);  // 设置自动连接
      break;
    }
    }
  }
    WiFi.setAutoConnect(true);  // 设置自动连接
    Serial.println("Connected to AP");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}
/**************************************************************************
                                 MAX30105
***************************************************************************/
/*
  WiFiTick
  检查是否需要初始化WiFi
  检查WiFi是否连接上，若连接成功启动TCP Client
  控制指示灯
*/
void MAXSET(){
    Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}
void MAXSTOP(){
  //particleSensor.setPulseAmplitudeRed(0); //Turn Red LED to low to indicate sensor is running
  //particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  }
void MAXMAIN(){
  irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
}
void MAXTMP(){
  
  MAXSET();
  for(int i=0;i<30;i++){
  temperatureMAX= particleSensor.readTemperature()-9;
  }
  MAXSTOP();
}
void readMAX(){
  MAXSET();
  for(long i=0;i<=2800;i++){
    MAXMAIN();
  }
  MAXSTOP();
  }
 void temppts(){
    // read without samples.
      h = dht.readHumidity();
   // 读取温度或湿度大约需要250毫秒
   t = dht.readTemperature()-13;
   // 将温度读取为摄氏温度（默认值）
   f = dht.readTemperature(true)-55.4;
     //热量指数
    // 计算华氏温度 (默认)
    
     hif = dht.computeHeatIndex(f, h);
   // 计算摄氏温度 (Fahreheit = false)
   hic = dht.computeHeatIndex(t, h, false);

   /*
      if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println("没有从DHT1传感器上获取数据!");
      return;
   }   
   */
  }
  void SPO(){
      MAXSET();
      
  //read the first 50 samples, and determine the signal range
  for (byte i = 0 ; i < 50 ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 50 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, 50, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  for(int i=0;i<15;i++){
    //dumping the first 25 sets of samples in the memory and shift the last 25 sets of samples to the top
    for (byte i = 25; i < 50; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 25; i < 50; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);


      if(spo2>=100||spo2<90){
           Serial.print(F(", SPO2="));
      Serial.print("false!");
        }
        else{
            Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);
        OS=spo2;

          }
      
      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
 
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, 50, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
    }
 void FUUTS(){
for(int i=0;i<30;i++){
temppts();
}
MAXTMP();
readMAX();
SPO();
  }
  
/**************************************************************************
                                 WIFI
***************************************************************************/
/*
  WiFiTick
  检查是否需要初始化WiFi
  检查WiFi是否连接上，若连接成功启动TCP Client
  控制指示灯
*/
void doWiFiTick(){
  static bool startSTAFlag = false;
  static bool taskStarted = false;
  static uint32_t lastWiFiCheckTick = 0;

  if (!startSTAFlag) {
    startSTAFlag = true;
    startSTA();
    Serial.printf("Heap size:%d\r\n", ESP.getFreeHeap());
  }

  //未连接1s重连
  if ( WiFi.status() != WL_CONNECTED ) {
    if (millis() - lastWiFiCheckTick > 1000) {
      lastWiFiCheckTick = millis();
    }
  }
  //连接成功建立
  else {
    if (taskStarted == false) {
      taskStarted = true;
      Serial.print("\r\nGet IP Address: ");
      Serial.println(WiFi.localIP());
      startTCPClient();
    }
  }
}
void saveConfig()//保存函数
{
 EEPROM.begin(1024);//向系统申请1024kb ROM
 //开始写入
 uint8_t *p = (uint8_t*)(&config);
  for (int i = 0; i < sizeof(config); i++)
  {
    EEPROM.write(i, *(p + i)); //在闪存内模拟写入
  }
  EEPROM.commit();//执行写入ROM
}

void loadConfig()//读取函数
{
  EEPROM.begin(1024);
  uint8_t *p = (uint8_t*)(&config);
  for (int i = 0; i < sizeof(config); i++)
  {
    *(p + i) = EEPROM.read(i);
  }
  EEPROM.commit();
  ssid = config.stassid;
  psw = config.stapsw;
}
/**************************************************************************
                                 update
***************************************************************************/
/*
  云升级系统
*/
void updateBin(){
  WiFiClient UpdateClient;
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, upUrl);
  switch(ret) {
    case HTTP_UPDATE_FAILED:      //当升级失败
        Serial.println("[update] Update failed.");
        break;
    case HTTP_UPDATE_NO_UPDATES:  //当无升级
        Serial.println("[update] Update no Update.");
        break;
    case HTTP_UPDATE_OK:         //当升级成功
        Serial.println("[update] Update ok.");
        break;
  }
}
/**************************************************************************
                                 main
***************************************************************************/
/*
  主程序
*/
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
 doWiFiTick();
  doTCPClientTick();
}
