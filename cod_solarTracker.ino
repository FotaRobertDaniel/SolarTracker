#include <Servo.h>
#include <LiquidCrystal.h>

// Definire servomotoare și variabile
Servo horizontal; // horizontal servo
int servoh = 180; 
int servohLimitHigh = 175;
int servohLimitLow = 5;

Servo vertical; // vertical servo
int servov = 45; 
int servovLimitHigh = 60;
int servovLimitLow = 1;

// Conexiuni fotorezistori
int ldrlt = A0; // LDR top left
int ldrrt = A2; // LDR top right
int ldrld = A1; // LDR down left
int ldrrd = A3; // LDR down right

int solarPin = A6; // Pin analogic pentru citirea curentului panoului solar

// Pini LED
int redLed = 5;
int yellowLed = 6;
int greenLed = 7;

// Inițializare LCD
LiquidCrystal lcd(12, 11, 4, 3, 2, 8);

// Variabile pentru temporizare
volatile bool timerFlag = false;

void setup() {
  // Inițializare servo
  horizontal.attach(9);   // Atașează servomotorul orizontal la pinul 9
  vertical.attach(10);    // Atașează servomotorul vertical la pinul 10
  horizontal.write(servoh);
  vertical.write(servov);

  // Inițializare pini LED ca ieșiri
  pinMode(redLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(greenLed, OUTPUT);

  // Inițializare LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Robert Fota");
  lcd.setCursor(0, 1);
  lcd.print("Solar Tracker");

  // Inițializare UART pentru debugging
  Serial.begin(9600);

  // Configurare Timer2 pentru întreruperi
  noInterrupts();           // Disable interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 249;              // Comparare la fiecare 1 ms (16MHz clock, prescaler 64)
  TCCR2A |= (1 << WGM21);   // Mod CTC
  TCCR2B |= (1 << CS22);    // Prescaler 64
  TIMSK2 |= (1 << OCIE2A);  // Enable timer compare interrupt
  interrupts();             // Enable interrupts
}

ISR(TIMER2_COMPA_vect) { // Funcția ISR pentru Timer2
  static unsigned long counter = 0;
  counter++;
  if (counter >= 1000) { // 1 secundă
    timerFlag = true;
    counter = 0;
  }
}

void loop() {
  if (timerFlag) {
    timerFlag = false;
    // Citirea curentului produs de panoul solar
    int solarCurrent = analogRead(solarPin);
    float percentage = (solarCurrent / 1023.0) * 100.0;

    // Controlul LED-urilor utilizând PWM
    if (percentage <= 30) {
      analogWrite(redLed, 255);
      analogWrite(yellowLed, 0);
      analogWrite(greenLed, 0);
    } else if (percentage > 30 && percentage <= 70) {
      analogWrite(redLed, 255);
      analogWrite(yellowLed, 255);
      analogWrite(greenLed, 0);
    } else {
      analogWrite(redLed, 255);
      analogWrite(yellowLed, 255);
      analogWrite(greenLed, 255);
    }

    // Actualizarea afișajului LCD
    lcd.setCursor(0, 0);
    lcd.print("Solar:");
    lcd.setCursor(7, 0);
    lcd.print(percentage);
    lcd.print("% ");

    lcd.setCursor(0, 1);
    lcd.print("Curent:");
    lcd.print(solarCurrent);
    lcd.print("W  ");

    // Trimitere date prin UART pentru debugging
    Serial.print("Solar Percentage: ");
    Serial.print(percentage);
    Serial.println("%");
    Serial.print("Solar Current: ");
    Serial.print(solarCurrent);
    Serial.println("W");
  }

  // Citirea valorilor fotorezistorilor
  int lt = analogRead(ldrlt); // top left
  int rt = analogRead(ldrrt); // top right
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down right

  int dtime = 10; 
  int tol = 90; // dtime=diffirence time, tol=tolerance

  int avt = (lt + rt) / 2; // average value top
  int avd = (ld + rd) / 2; // average value down
  int avl = (lt + ld) / 2; // average value left
  int avr = (rt + rd) / 2; // average value right

  int dvert = avt - avd; // check the difference of up and down
  int dhoriz = avl - avr;// check the difference of left and right

  // Ajustarea poziției panoului solar
  if (abs(dvert) > tol) {
    if (avt > avd) {
      servov++;
      if (servov > servovLimitHigh) {
        servov = servovLimitHigh;
      }
    } else if (avt < avd) {
      servov--;
      if (servov < servovLimitLow) {
        servov = servovLimitLow;
      }
    }
    vertical.write(servov);
  }

  if (abs(dhoriz) > tol) {
    if (avl > avr) {
      servoh--;
      if (servoh < servohLimitLow) {
        servoh = servohLimitLow;
      }
    } else if (avl < avr) {
      servoh++;
      if (servoh > servohLimitHigh) {
        servoh = servohLimitHigh;
      }
    }
    horizontal.write(servoh);
  }

  delay(dtime);
}
