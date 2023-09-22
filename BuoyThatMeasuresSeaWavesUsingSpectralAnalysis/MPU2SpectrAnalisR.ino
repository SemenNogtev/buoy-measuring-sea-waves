#include <SPI.h>      // библиотека для работы с шиной SPI
#include "nRF24L01.h" // библиотека радиомодуля
#include "RF24.h"     // ещё библиотека радиомодуля

// библиотеки для работы с дисплеем TFT LCD 3.5"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library

// обозначение пинов дисплея
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// обозначение цветов дисплея
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// обозначение координат дисплея
#define HEIGHT_DISPLAY 320 // высота дисплея
#define WIDTH_DISPLAY 480 // ширина дисплея
#define START_X 30 // начало координат (30; 280)
#define START_Y 280 // начало координат (30; 280)
#define LENGTH_X 420 // длина Ox
#define LENGTH_Y 220 // длина Oy

#define T_OUT 200 // каждые 200 миллисекунд будем проводить вычисления 

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET); // объект экрана

RF24 radio(9, 10);  // "создать" модуль на пинах 9 и 10 Для Уно
byte pipeNo;
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб

// переменные, использующиеся в спектральном анализе
const int N = 32; // число измерений
const int Npos = 16; // число измерений в положительных частотах
float F[Npos], H[Npos];
int Hint[Npos];
float dt = T_OUT*0.001;
float fs = 1/dt;

// переменные, использующиеся при работе с дисплеем
float limitH = 5; // м
float limitF = 2.34; // Гц
float scaleFactorX = LENGTH_X/limitF;
float scaleFactorY = LENGTH_Y/limitH;
int countLCD = 0;

void setup() {
  // последовательный порт для связи с ПК
  Serial.begin(9600);
  
  // настройка радио
  radio.begin();              // активировать модуль
  radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);   // размер пакета, в байтах
  radio.openReadingPipe(1, address[0]);   // хотим слушать трубу 0
  radio.setChannel(0x60);     // выбираем канал (в котором нет шумов!)
  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!
  radio.powerUp();        // начать работу
  radio.startListening(); // начинаем слушать эфир, мы приёмный модуль

  // настройка дисплея
  tft.reset();
  uint16_t identifier = tft.readID();
  tft.begin(0x9341);
  tft.setRotation(1); // 0(rotate 0 degrees), 1, 2, 3 
  printDisplay();
}

void loop() {
  while (radio.available(&pipeNo)) {  // слушаем эфир со всех труб
    radio.read(&Hint, sizeof(Hint));  // чиатем входящий сигнал
    for(int k = 0; k < Npos; k++)
    {
      H[k] = 0.01*Hint[k]; // *0.01 для округления до сотых
      Serial.print(H[k]);
      Serial.print(" ");
      printChart();
    }
    Serial.println();
  }
}

// функция обновления дисплея
void printDisplay()
{
  // Color
  tft.fillScreen(YELLOW);
  tft.setTextColor(BLACK);
  
  // "Wave Spectrum"
  tft.setCursor(120, 10);  
  tft.setTextSize(3); // 1, 2, 3
  tft.print("Wave Spectrum");
  
  // "H,m"
  tft.setCursor(30, 30);
  tft.setTextSize(3); // 1, 2, 3
  tft.print("H,m");
  
  // "F,hz"
  tft.setCursor(380, 250);
  tft.setTextSize(3); // 1, 2, 3
  tft.print("F,hz");

  // Coordinate System
  for (int a=0; a<5; a++) tft.drawFastVLine(START_X+a, START_Y-LENGTH_Y, LENGTH_Y, BLACK);   
  for (int a=0; a<5; a++) tft.drawFastHLine(START_X, START_Y+a, LENGTH_X, BLACK);
  
  // Coordinate System Values
  for(int n = 1; n <= limitH; n++)
  { 
    tft.setCursor(START_X/2, START_Y-n*scaleFactorY);
    tft.setTextSize(2); // 1, 2, 3
    tft.print(n);
    tft.drawFastHLine(START_X, START_Y-n*scaleFactorY, LENGTH_X, BLACK);
  }  

  // float stepF = limitF/(Npos-1);
  float stepF = fs/2/(Npos-1);
  for(int k = 0; k<Npos-1; k++)
  {
    F[k] = stepF*k;
  }

  for(int n = 1; n<Npos-1; n++)
  {
    tft.setCursor(START_X+n*stepF*scaleFactorX-10, START_Y+(HEIGHT_DISPLAY-START_Y)/3); // 10 - поправка
    tft.setTextSize(1); // 1, 2, 3
    if((n+1)%2==0) tft.print(F[n]); 
    tft.drawFastVLine(START_X+n*stepF*scaleFactorX, START_Y-LENGTH_Y, LENGTH_Y, BLACK);
  }
}

// функция отрисовки графика
void printChart()
{
  countLCD++;
  if(countLCD>60)
  {
    printDisplay();
    countLCD = 0;
  }
  for(int k = 0; k < Npos-1; k++)
  { 
    for (int a=0; a<3; a++) tft.drawLine(START_X+F[k]*scaleFactorX+a, START_Y-H[k]*scaleFactorY, START_X+F[k+1]*scaleFactorX+a, START_Y-H[k+1]*scaleFactorY, BLUE);
    for (int a=0; a<3; a++) tft.drawLine(START_X+F[k]*scaleFactorX, START_Y-(H[k]*scaleFactorY+a), START_X+F[k+1]*scaleFactorX, START_Y-(H[k+1]*scaleFactorY+a), BLUE);
  }
}
