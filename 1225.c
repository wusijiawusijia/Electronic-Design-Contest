#include <LiquidCrystal.h>
#include <MsTimer2.h>
#include <Keypad.h>
#include <OneWire.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
//-----------连接超声波测距模块1---------------------------
#define Trig 2 //引脚Tring 连接 IO D2
#define Echo 3
//-----------连接超声波测距模块2---------------------------
#define Trig2 4 //引脚Tring 连接 IO D2
#define Echo2 5
//---------------------------------------------------------
#define LIGHT  45
#define BEEPER 6//引脚6接蜂鸣器
//---------------------------------------------------------
#define TIMEMULT 0.5
// connect SCL to I2C Clock
// connect SDA to I2C Data
// Serial1接功率传感器

const byte ROWS = 4;
const byte COLS = 4;
//-----------------------键盘-------
char keys[ROWS][COLS] = {
  {'1', '2', '3', '4'},
  {'5', '6', '7', '8'},
  {'9', 'A', 'B', 'C'},
  {'D', 'E', 'F', '0'}
};
byte rowPins[ROWS] = {32, 34, 36, 38};
byte colPins[COLS] = {30, 28, 26, 24};
String comdata = "";

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
unsigned char aixuexi[24] = {
  0XCE, 0XD2, 0XB0, 0XAE, 0XD1, 0XA7, 0XCF, 0XB0, 0XA3, 0XAC, 0XD1, 0XA7, 0XCF, 0XB0, 0XCA, 0XB9, 0XCE, 0XD2, 0XBF, 0XEC, 0XC0, 0XD6, 0XA3, 0XA1
};
unsigned char zhuyizishi[20] = {
  0XD7, 0XA2, 0XD2, 0XE2, 0XD7, 0XCB, 0XCA, 0XC6, 0XA3, 0XAC, 0XBE, 0XE0, 0XC0, 0XEB, 0XB2, 0XFA, 0XC9, 0XFA, 0XC3, 0XC0
};
unsigned char zuolehenjiule[24] = {
  0XD7, 0XF8, 0XBA, 0XDC, 0XBE, 0XC3, 0XC1, 0XCB, 0XA3, 0XAC, 0XC6, 0XF0, 0XC0, 0XB4, 0XD4, 0XCB, 0XB6, 0XAF, 0XD2, 0XBB, 0XCF, 0XC2, 0XB0, 0XC9
};
//--------------------屏幕接线-----------
const int rs = 12, en = 11, d4 = 7, d5 = 8, d6 = 9, d7 = 10;
/*

   LCD RS pin to digital pin 12
   LCD Enable pin to digital pin 11
   LCD D4 pin to digital pin 5
   LCD D5 pin to digital pin 4
   LCD D6 pin to digital pin 3
   LCD D7 pin to digital pin 2
   LCD R/W pin to ground
   LCD VSS pin to ground
   LCD VCC pin to 5V
   10K resistor:
   ends to +5V and ground
   wiper to LCD VO pin (pin 3)

*/
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//--------------------------------------
int sensorValue = 0;
float cm, cm1, cm2, alarmcm = 30.0;
float brightness;//灯亮度，0-255
int alarmcount = 1;//定时时间（2）
int lightmode = 0;//灯的模式
int tick = 0; //计数值,60为一分钟
char key ;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
// pass in a number for the sensor identifier (for your use later)
int distancetime = 0; int counttime = 1; int leavetime = 1;
int lastsecond1;
int autoflag = 0;
int sitlongflag = 0;
int leaveflag = 0;
int countflag = 0;
int distance_close_alarm_flag = 0;
int runtime;
int resttime;
int ifleave;//是否离开，是为1，否为0
int j, i , k = 0;

double V = 0, C = 0, P = 0;
//屏幕显示
int show_powerflag = 0;
int show_worktimeflag = 0;
int show_distanceflag = 0;
int show_modeflag = 1;
int showflag = 0;//1为报警距离，2为定时时间，3为离开时间
int halftime;
int tock;
int voiceflag = 0;

//============================================================
//============================================================
//============================================================
//============================================================
//============================================================
void setup(void)
{
  Serial.begin(9600);//启动总线0
  Serial1.begin(4800, SERIAL_8E1);
  //-------------------------配置启动光线传感器--------------
  Serial.println(F("Starting Adafruit TSL2591 Test!"));
  if (tsl.begin())
  {
    Serial.println(F("Found a TSL2591 sensor"));
  }
  else
  {
    Serial.println(F("No sensor found ... check your wiring?"));
  }
  configureSensor();
  keypad.addEventListener(keypadEvent);
  Serial2.begin(9600);

  //------------------------------------------------------------
  digitalWrite(BEEPER, HIGH); //初始化蜂鸣器
  lcd.begin(16, 2);

  //----------------------------配置启动超声波测距模块1---------
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  //------------------------------------------------------------

  //--------------------------配置启动超声波测距模块2---------
  pinMode(Trig2, OUTPUT);
  pinMode(Echo2, INPUT);
  //------------------------------------------------------------

  //----------------------------配置蜂鸣器----------------------
  pinMode(BEEPER, OUTPUT);
  //-------------------------------------------------------------

  MsTimer2::set(1000, onTimer); //设置中断，每1000ms进入一次中断服务程序 onTimer()
  MsTimer2::start();
}

//============================================================
//============================================================
//============================================================
//============================================================
//============================================================

void loop(void)
{
  key = keypad.getKey();
  /*if (key != NO_KEY) {
    Serial.println(key);
    }*/
  //GetPower();
  //DistanceTest();
  //Beep();
  //AutoLight();
  //LightControl(int lightmode);


  if (autoflag)  AutoLight();

  if ((cm < alarmcm) && (distance_close_alarm_flag == 1 )) {
    Beep();
    if (voiceflag == 1)Serial2.write(zhuyizishi, 20);
  }
  getworktime();
  if ((sitlongflag == 1) && (alarmcount < runtime)) {
    runtime = 0;
    alarmcount = 0;
    Beep();
    if (voiceflag == 1)Serial2.write(zhuyizishi, 24);
  }
  if ((leaveflag == 1) && ( leavetime == resttime)) {
    resttime = 0;
    leavetime = 0;
    if (voiceflag == 1)Serial2.write(zuolehenjiule, 24);
    key = '1' ;
  }
  //-----------------屏幕显示层
  if (show_modeflag == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mode:");
    lcd.setCursor(0, 1);
    lcd.print(lightmode);
  }
  if (show_powerflag == 1) {
    comdata = "";
    GetPower();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("U:");
    lcd.print(V, 0);
    lcd.print("V");
    lcd.print(" ");
    lcd.print("I:");
    lcd.print(C, 2);
    lcd.print("A");
    lcd.setCursor(0, 1);
    lcd.print("P:");
    lcd.print(P);
    lcd.print("W");
  };
  if (show_worktimeflag == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mode:");
    lcd.setCursor(0, 1);
    lcd.print(lightmode);
  };
  DistanceTest();
  if (show_distanceflag == 1) {

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Distance:");
    lcd.setCursor(0, 1);
    lcd.print(cm);
    lcd.setCursor(10, 1);
    lcd.print("cm");
    if (distance_close_alarm_flag == 1)
    { lcd.setCursor(15, 0);
      lcd.print("!");
    }
  };
  key = keypad.getKey();
  delay(30);
  key = keypad.getKey();
  if ((show_powerflag == 0) && (show_worktimeflag == 0) && (show_distanceflag == 0) && (show_modeflag == 0) && (showflag != 0))
  { lcd.clear();
    lcd.setCursor(0, 0);
    if (showflag == 1)
      lcd.print("Distance:");
    else
      lcd.print("Time:");
    lcd.setCursor(0, 1);
    if (showflag == 1)
      lcd.print(alarmcm);
    else if (showflag == 2)
      lcd.print(alarmcount);
    else if (showflag == 3)
      lcd.print(leavetime);
    else
      lcd.print(runtime);
    lcd.setCursor(15, 0);
    if ((showflag == 2 ) && ( sitlongflag == 1))
      lcd.print("!");
    if ((showflag == 3 ) && (leaveflag == 1))
      lcd.print("!");
    if ((showflag == 4 ) && ( sitlongflag == 1))
      lcd.print("!");
    delay(30);
    key = keypad.getKey();
  }
}


//============================================================
//============================================================
//============================================================
//============================================================
//============================================================
void onTimer()
{
  tick++;
  tock = !tock;
}
//============================================================
//============================================================
//============================================================
//============================================================
//============================================================

void configureSensor(void)
{

  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);

}

//============================================================
//============================================================
//============================================================
//============================================================
//============================================================
void AutoLight()
{
  sensors_event_t event;
  tsl.getEvent(&event);

  if ((event.light == 0) |
      (event.light > 4294966000.0) |
      (event.light < -4294966000.0))
  {

    Serial.println(F("Invalid data (adjust gain or timing)"));
  }
  else
  {
    Serial.print(event.light); Serial.println(F(" lux"));
  }

  float sensorValue = event.light;
  if (sensorValue > 800.0 || sensorValue == -1.0)
    sensorValue = 800.0;
  else
    brightness = 255.0 - 255.0 / 800.0 * sensorValue;
  Serial.print("Brightness=");
  if (brightness < 50.0)
    brightness = 0.0;
  if (brightness < 50)
    lightmode = 0;
  else if (( brightness >= 50) && (  brightness <= 60))
    lightmode = 1;
  else if (( brightness > 60) && ( brightness <= 90))
    lightmode = 2;
  else if (( brightness > 90) && ( brightness <= 120))
    lightmode = 3;
  else if (( brightness > 120) && ( brightness <= 150))
    lightmode = 4;
  else if (( brightness > 150) && ( brightness <= 180))
    lightmode = 5;
  else if (( brightness > 180) && ( brightness <= 220))
    lightmode = 6;
  else     lightmode = 7;

  Serial.println(brightness);
  analogWrite(LIGHT, brightness);
  //延时10ms,因为analogWrite执行瞬间完成，
}
//============================================================
//============================================================
//============================================================
//============================================================
//============================================================
void DistanceTest()
{ float temp;
  //-----------------------距离模块1--------------------
  digitalWrite(Trig, LOW); //给Trig发送一个低电平
  delayMicroseconds(2);    //等待 2微妙
  digitalWrite(Trig, HIGH); //给Trig发送一个高电平
  delayMicroseconds(10);    //等待 10微妙
  digitalWrite(Trig, LOW); //给Trig发送一个低电平

  temp = float(pulseIn(Echo, HIGH)); //存储回波等待时间,
  //pulseIn函数会等待引脚变为HIGH,开始计算时间,再等待变为LOW并停止计时
  //返回脉冲的长度

  //声速是:340m/1s 换算成 34000cm / 1000000μs => 34 / 1000
  //因为发送到接收,实际是相同距离走了2回,所以要除以2
  //距离(厘米)  =  (回波时间 * (34 / 1000)) / 2
  //简化后的计算公式为 (回波时间 * 17)/ 1000

  cm1 = (temp * 17 ) / 1000; //把回波时间换算成cm

  Serial.print("Echo=");
  Serial.print(temp);//串口输出等待时间的原始数据
  Serial.print(" | | Distance = ");
  Serial.print(cm1);//串口输出距离换算成cm的结果
  Serial.println("cm");
  delay(0);
  //------------------------------------------------------------

  temp = 0;

  //-----------------------------距离模块2--------------------
  //给Trig发送一个低高低的短时间脉冲,触发测距
  digitalWrite(Trig2, LOW); //给Trig发送一个低电平
  delayMicroseconds(2);    //等待 2微妙
  digitalWrite(Trig2, HIGH); //给Trig发送一个高电平
  delayMicroseconds(10);    //等待 10微妙
  digitalWrite(Trig2, LOW); //给Trig发送一个低电平

  temp = float(pulseIn(Echo2, HIGH)); //存储回波等待时间,
  //pulseIn函数会等待引脚变为HIGH,开始计算时间,再等待变为LOW并停止计时
  //返回脉冲的长度
  //声速是:340m/1s 换算成 34000cm / 1000000μs => 34 / 1000
  //因为发送到接收,实际是相同距离走了2回,所以要除以2
  //距离(厘米)  =  (回波时间 * (34 / 1000)) / 2
  //简化后的计算公式为 (回波时间 * 17)/ 1000

  cm2 = (temp * 17 ) / 1000; //把回波时间换算成cm
  Serial.print("Echo2=");
  Serial.print(temp);//串口输出等待时间的原始数据
  Serial.print(" | | Distance2= ");
  Serial.print(cm2);//串口输出距离换算成cm的结果
  Serial.println("cm");
  delay(10);
  //-------------------------------------------------
  cm1 < cm2 ? cm = cm1 : cm = cm2;

  Serial.print("cm= ");
  Serial.print(cm);//串口输出距离换算成cm的结果
  Serial.println("cm");

}
//============================================================
//============================================================
//============================================================
//============================================================
//============================================================
void GetPower()//数据处理
{
  int i;
  long VP_REG = 0, V_REG = 0, CP_REG = 0, C_REG = 0, PP_REG = 0, P_REG = 0;
  // unsigned int PF = 0, PF_COUNT = 0;
  //double V = 0, C = 0, P = 0;
  /*----------------------------------------------------数据读取------------------------------*/
  for (i = 0; i < 24; i++)
    while (Serial1.available() > 0)
    {
      {
        comdata += char(Serial1.read());
        delay(2);
      }
    }
  key = keypad.getKey();
  /*----------------------------------------------------------------------------------------------*/
  if (comdata[0] != 0xaa) //芯片误差修正功能正常，参数正常
  {
    VP_REG = comdata[2] * 65536 + comdata[3] * 256 + comdata[4]; //计算电压参数寄存器
    V_REG = comdata[5] * 65536 + comdata[6] * 256 + comdata[7]; //计算电压寄存器
    V = (VP_REG / V_REG) * 2.35; //计算电压值，2.35为电压系数，根据所采用的分压电阻大小来确定
    if (V <= 5)  V = 0;
    Serial.print("电压值：");
    Serial.print(V, 2);
    Serial.println("V");
    CP_REG = comdata[8] * 65536 + comdata[9] * 256 + comdata[10]; //计算电流参数寄存器
    C_REG = comdata[11] * 65536 + comdata[12] * 256 + comdata[13]; //计算电流寄存器
    C = ((CP_REG * 100) / C_REG) / 220.0; //计算电流值
    if (C < 0.07) C = 0;
    Serial.print("电流值：");
    Serial.print(C, 2);
    Serial.println("A");
    if (comdata[0] > 0xf0) //判断实时功率是否未溢出
    {
      Serial.print("No Device!");//功耗过低，未接用电设备
    }
    else
    {
      P = V * C;
      Serial.print("有效功率：");
      Serial.print(P, 2);
      Serial.println("W");
    }
  }
  else//芯片误差修正功能失效
  {
    Serial.print("data error\r\n");
    key = keypad.getKey();
  }
  comdata = "";

}
//============================================================
//============================================================
//============================================================
//============================================================
//============================================================
void Beep()
{ int i;
  for (i = 0; i < 3; i++)
  {
    digitalWrite(BEEPER, LOW);
    delay(100); key = keypad.getKey();
    digitalWrite(BEEPER, HIGH);
    delay(100); key = keypad.getKey();
  }
  for (i = 0; i < 7; i++)
  { delay(100);
    key = keypad.getKey();
  }
  for (i = 0; i < 3; i++)
  {
    digitalWrite(BEEPER, LOW);
    delay(100);
    key = keypad.getKey();
    digitalWrite(BEEPER, HIGH);
    delay(100); key = keypad.getKey();
  }
}
//============================================================
//============================================================
//============================================================
//============================================================
//============================================================
void LightControl(int lightmode)
{ switch (lightmode)
  { case 0 : brightness = 0; analogWrite(LIGHT, brightness); break;
    case 1 : brightness = 60; analogWrite(LIGHT, brightness); break;
    case 2 : brightness = 90; analogWrite(LIGHT, brightness); break;
    case 3 : brightness = 120; analogWrite(LIGHT, brightness); break;
    case 4 : brightness = 150; analogWrite(LIGHT, brightness); break;
    case 5 : brightness = 180; analogWrite(LIGHT, brightness); break;
    case 6 : brightness = 220; analogWrite(LIGHT, brightness); break;
    case 7 : brightness = 255; analogWrite(LIGHT, brightness); break;
  }
}


//============================================================
//============================================================
//============================================================
//============================================================
//============================================================
void getworktime()
{
  if ((cm < 50.0) && (brightness > 50.0))i++;
  if ((cm > 50.0) && (brightness > 50.0))j++;
  if (i == 250 * TIMEMULT) {
    runtime++;
    i = 0;
  }
  if (j == 250 * TIMEMULT) {
    resttime++;
    j = 0;
    if (tick = 10) {
      tick = 0;
      k = !k;
    }
  }
}
//============================================================
//============================================================
//============================================================
//============================================================
//============================================================
void keypadEvent(KeypadEvent key)
{
  switch (keypad.getState())      {
    case PRESSED:
      switch (key) {
        case '1':
          autoflag = 0;
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 1;
          lightmode = 0;
          LightControl(lightmode);
          break;
        case '2':
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 1;
          autoflag = 0;
          lightmode = 7;
          LightControl(lightmode);
          break;
        case '3':
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 1;
          autoflag = 0;
          lightmode--;
          if (lightmode < 0)lightmode = 0;
          LightControl(lightmode);
          break;
        case '4':
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 1;
          autoflag = 0;
          lightmode++; if (lightmode > 7)lightmode = 7;
          LightControl(lightmode);
          break;
        case '5':
          autoflag = 1;
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 1;
          break;
        case '6':
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 1;
          show_modeflag = 0;
          break;
        case '7':
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 1;
          show_modeflag = 0;
          distance_close_alarm_flag = !distance_close_alarm_flag;
          break;
        case '8':
          alarmcm++;
          if (alarmcm > 90)alarmcm = 0;
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 0;
          showflag = 1;
          break;
        case '9':
          sitlongflag = !sitlongflag;
          showflag = 4;
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 0;
          break;
        case 'A':  voiceflag = !voiceflag;   if (voiceflag == 1)Serial2.write(aixuexi, 24); break;
        case 'B': alarmcount--;
          if (alarmcount < 0)alarmcount = 0;
          showflag = 2;
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 0;
          break;
        case 'C': alarmcount++;
          if (alarmcount > 90)alarmcount = 90;
          showflag = 2;
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 0;
          break;
        case 'D': showflag = 0;
          show_powerflag = 1;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 0; break;
        case 'E': leaveflag = !leaveflag;
          showflag = 3;
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 0;
          break;
        case 'F': leavetime--; if (leavetime < 0)leavetime = 0;
          showflag = 3;
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 0; break;
        case '0': leavetime++; if (leavetime > 90)leavetime = 90;
          showflag = 3;
          show_powerflag = 0;
          show_worktimeflag = 0;
          show_distanceflag = 0;
          show_modeflag = 0; break;
      }
      break;

  }
}
