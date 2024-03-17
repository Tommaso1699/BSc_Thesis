#include <SoftwareSerial.h> //dołączenie biblioteki
#include<QTRSensors.h> //dołączenie biblioteki
QTRSensors qtr;//instancja klasy QTRSensors
int counter = 0; //inicjalizacja zmiennej pomocniczej wykorzystanej przy komunikacji z aplikacją mobilną
float Kp = 0.07;//inicjalizacja wzmocnienia części proporcjonalnej
float Kd = 0.014;//inicjalizacja wzmocnienia części różniczkującej
float Ki = 0.00001;//inicjalizacja wzmocnienia części całkującej
SoftwareSerial bluetooth(0, 1);//załączenie modułu Bluetooth na pinach Rx oraz Tx     
String stat;//definicja zmiennej odczytującej przesłany łańcuch znaków z aplikacji mobilnej przez moduł Bluetooth        
#define ADC_0 A0//definicja aliasu pinu  
#define ADC_1 A1//definicja aliasu pinu  
#define ADC_2 A2//definicja aliasu pinu  
#define ADC_3 A3//definicja aliasu pinu 
#define ADC_4 A4//definicja aliasu pinu  
#define ADC_5 A5//definicja aliasu pinu  
#define ADC_6 A6//definicja aliasu pinu 
#define ADC_7 A7//definicja aliasu pinu  
#define PWM_R 6//definicja aliasu pinu
#define PWM_L 5 //definicja aliasu pinu  
#define AIN1 3//definicja aliasu pinu  
#define AIN2 2//definicja aliasu pinu  
#define BIN1 7//definicja aliasu pinu 
#define BIN2 8//definicja aliasu pinu  
#define MOTOR1_R 9//definicja aliasu pinu 
#define MOTOR2_R 10//definicja aliasu pinu  
#define MOTOR1_L 11//definicja aliasu pinu  
#define MOTOR2_L 12//definicja aliasu pinu 
#define STBY 4//definicja aliasu pinu  
unsigned int ADC_x[8] = {ADC_0, ADC_1, ADC_2, ADC_3, ADC_4, ADC_5, ADC_6, ADC_7}; // inicjalizacja tablicy transoptorów
int lastError = 0; //inicjalizacja zmiennej odpowiedzialnej za ostatni odczyt z transoptorów
int P;//deklaracja zmiennej odpowiedzialnej za człon wzmacniający
int I;//deklaracja zmiennej odpowiedzialnej za człon całkujący
int D;//deklaracja zmiennej odpowiedzialnej za człon różniczkujący
const uint8_t maxspeed_a = 110;//inicjalizacja maksymalnej predkości dla prawego silnika
const uint8_t maxspeed_b = 110;//inicjalizacja maksymalnej predkości dla lewego silnika
const uint8_t basespeed_a = 80;//inicjalizacja minimalnej predkości dla prawego silnika
const uint8_t basespeed_b = 80;//inicjalizacja minimalnej predkości dla lewego silnika
uint16_t SensorValues[8];//deklaracja 8 elementowej tablicy
void drive(int a, int b) //funkcja zadająca predkość silnikom
{
   analogWrite(PWM_R, a);//zadanie prędkości na silnik prawy
   analogWrite(PWM_L, b);//zadanie prędkości na silnik lewy
}
void setup()
{
  pinMode(ADC_0, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za transoptor w stan wejścia
  pinMode(ADC_1, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za transoptor w stan wejścia
  pinMode(ADC_2, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za transoptor w stan wejścia
  pinMode(ADC_3, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za transoptor w stan wejścia
  pinMode(ADC_4, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za transoptor w stan wejścia
  pinMode(ADC_5, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za transoptor w stan wejścia
  pinMode(ADC_6, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za transoptor w stan wejścia
  pinMode(ADC_7, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za transoptor w stan wejścia
  pinMode(PWM_R, OUTPUT);//ustawienie pinu na Arduino odpowiedzialnego za silnik prawy w stan wyjścia
  pinMode(PWM_L, OUTPUT);//ustawienie pinu na Arduino odpowiedzialnego za silnik lewy w stan wyjścia
  pinMode(AIN1, OUTPUT);//ustawienie pinu na Arduino odpowiedzialnego za pin na mostku H w stan wyjścia
  pinMode(AIN2, OUTPUT);//ustawienie pinu na Arduino odpowiedzialnego za pin na mostku H w stan wyjścia
  pinMode(BIN1, OUTPUT);//ustawienie pinu na Arduino odpowiedzialnego za pin na mostku H w stan wyjścia
  pinMode(BIN2, OUTPUT);//ustawienie pinu na Arduino odpowiedzialnego za pin na mostku H w stan wyjścia
  pinMode(MOTOR1_R, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za pin na mostku H oraz wyprowadzenie enkodera w stan wejścia
  pinMode(MOTOR2_R, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za pin na mostku H oraz wyprowadzenie enkodera w stan wejścia
  pinMode(MOTOR1_L, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za pin na mostku H oraz wyprowadzenie enkodera w stan wejścia
  pinMode(MOTOR2_L, INPUT);//ustawienie pinu na Arduino odpowiedzialnego za pin na mostku H oraz wyprowadzenie enkodera w stan wejścia
  pinMode(STBY, OUTPUT);//ustawienie pinu na Arduino odpowiedzialnego za pin na mostku H w stan wyjścia
  digitalWrite(STBY, HIGH);//ustawienie pinu na Arduino odpowiedzialnego za pin na mostku H  w stan wysoki
  digitalWrite(AIN1, HIGH);//ustawienie pinu na mostku H w stan wysoki
  digitalWrite(AIN2, LOW);//ustawienie pinu na mostku H w stan niski
  digitalWrite(BIN1, HIGH);//ustawienie pinu na mostku H w stan wysoki
  digitalWrite(BIN2,LOW);//ustawienie pinu na mostku H w stan niski
  //ustawienie powyższych pinów AIN1, AIN2, BIN1 oraz BIN2 pozwala na kręcenie się silników z maksymalną prędkością zadaną poprzez PWM (różną od 0)
  //zgodnie ze wskazówkami zegara
  bluetooth.begin(9600);//określenie szybkości transmisji danych dla modułu Bluetooth               
  Serial.begin(9600); //określenie szybkości transmisji danych w monitorze portu szeregowego
 qtr.setTypeAnalog();//kalibracja transoptorów
 qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, 8);//deklaracja pinów odpowiedzialnych za odczyt z transoptorów 
    for (uint16_t i = 0; i <= 60; i++)//pętla, która pozwala na mapowanie wartości odczytanych przez transoptory
  {
    qtr.calibrate();
    delay(1);
  }
}
 
void loop()
{  
    PID_control();//wywołanie funkcji
    //odczyt danych przesłanych przez aplikację z wykorzystaniem modułu Bluetooth 
    while (bluetooth.available()) {                         
    char c = bluetooth.read();          
    stat += c; }    
  if (stat.length() > 0) {
     counter++;
     if(counter==1){
      float K =  stat.toFloat();
      Serial.print("K:");
      Serial.println(K, 3);
     }
      if(counter==2){
      float Ki =  stat.toFloat();
      Serial.print("Ki:");
      Serial.println(Ki, 3);
     }
      if(counter==3){
      float Kd =  stat.toFloat();
      Serial.print("Kd:");
      Serial.println(Kd, 3);
      counter = 0;
     }
    stat = "";                          
  }
    }
  void PID_control() { //funkcja ustawiająca wzmocnienia nastaw regulatora PID oraz wywołująca funkcję drive() z odpowiednimi parametrami
  int Position = qtr.readLineBlack(SensorValues);//aktualna pozycja robota na podstawie odczytu z transoptorów
  P = Position-3333;//obliczenie części proporcjonalnej
  I = I + Position;// obliczenie części całkującej
  D = Position - lastError;// obliczenie części różniczkującej
  lastError = Position;//przypisanie obecnego odczytu z transoptorów do zmiennej, która będzie wykorzystana w następnej iteracji
  int motorspeed = P*Kp + I*Ki + D*Kd;//obliczenie korekty za pomocą reuglatora PID
  int motorspeed_a = basespeed_a + motorspeed;// zadanie prędkości do silnika prawego
  int motorspeed_b = basespeed_b - motorspeed;// zadanie prędkości do silnika lewego
  if (motorspeed_a > maxspeed_a) {//ograniczenie maksymalnej prędkości dla prawego silnika
    motorspeed_a = maxspeed_a;
  }
  if (motorspeed_b > maxspeed_b) {//ograniczenie maksymalnej prędkości dla lewego silnika
    motorspeed_b = maxspeed_b;
  }
  if (motorspeed_a < 0) {//ograniczenie minimalnej prędkości dla prawego silnika
    motorspeed_a = 0;
  }
  if (motorspeed_b < 0) {//ograniczenie minimalnej prędkości dla lewego silnika
    motorspeed_b = 0;
  }   
  drive(motorspeed_a, motorspeed_b);// wywołanie funkcji sterującej silnikami
}
