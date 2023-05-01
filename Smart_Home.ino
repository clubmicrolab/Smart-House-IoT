// Sectiunea pentru Preprocesor.

#include <Wire.h>                     // Libraria de comunicare simplificata I2C dintre dispozitive.
#include <LiquidCrystal_I2C.h>   // LCD cu interfata de comunicare I2C.
#include <DHT.h>                     // Senzor Temperatura si Umiditate DHT11.
#include <SPI.h>                      // Interfata Serial Peripheral Interface (SPI).
#include <MFRC522.h>              // Modul-ul RFID - RC522.
#include <MPU6050_tockn.h>    // Senzorul Gyroscope MPU6050.
#include <Servo.h>                  // Motorul Servo SG90.

// Modulul Keyes-clr13 (LED RGB integrat pe Modul).
#define redLed 24
#define greenLed 26
#define blueLed 22

// Definirea conexiunilor pentru senzorul de Temperatura si Umiditate DHT11.
#define DHTPIN 10
#define DHTTYPE DHT11

// Definirea conexiunii pentru senzorul de Detectie a Gazului MQ2.
#define MQ2_PIN A3

#define TRUE_VALUE 1

// Definirea conexiunilor pentru butonul B1 si butonul B2.
#define buttonPin1 3
#define buttonPin2 6

// Definirea conexiunii pentru Buzzer-ul pasiv.
#define BUZZER_PIN 12

// Definirea pinilor pentru SDA si SCL ale senzorului MPU6050, respectiv adresa acestuia.
#define MPU_ADDRESS 0x68
#define SDA_PIN 20
#define SCL_PIN 21

// Definirea pinului pentru Releu de control al Motorului DC.
#define relayPin 7

// Definirea pinilor analogic si digital pentru Senzorul de detectare a moleculelor de H2O.
#define AnalogWaterSensor A0
#define DigitalWaterSensor 2

// end Preprocessor.

// object-creation, class-call pentru DHT, LCD_I2C, RFID-RC522, MPU6050, SG90.
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
MFRC522 RC522(53, 37);
MPU6050 mpu(Wire);
Servo servoMotor;

// Definirea variabilelor pentru controlul efectiv al butoanelor B1 si B2.
int buttonState1 = 0;
int buttonState2 = 0;
int lastButtonState1 = 0;
int lastButtonState2 = 0;

// Definirea variabilelor pentru afisare/control al valorilor passcode-ului.
int digit1 = -1;
int digit2 = -1;

// Variabila binara pentru sincronizarea procedurii de logare.
bool isLoggedIn = false;

// Definirea variabilelor pentru a afisa ora & minuta curenta.
int hour = 0; 
int minute = 0;
int seconds = 0;

// Definirea variabilei pentru controlul efectiv al valorii-nivel WaterDetector.
int WaterDetector = 0;

// Functia de setare a senzorilor, parametrilor, valorilor de IN/OUT.
void setup() {

  // Setarea Modulului Keyes-clr13 in calitate de OUTPUT.
  pinMode(redLed, OUTPUT);        
  pinMode(greenLed, OUTPUT);
  pinMode(blueLed, OUTPUT);

  // Setarea Modulelor Butoanelor in calitate de INPUT.
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);

  // Setarea Releului in calitate de OUTPUT.
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);  //Releul deconectat.

  Serial.begin(9600); // Portul Serial este capabil sa transfere capacitatea maximala de 9600 bits per secunda.

  // Initializarea dispozitivelor : LCD_I2C, RFID-RC522, MPU6050
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);
  lcd.init();
  dht.begin();
  SPI.begin(); 
  RC522.PCD_Init();
  lcd.backlight();

  // Setarea Buzzer-ului in calitate de OUTPUT.
  pinMode(BUZZER_PIN, OUTPUT);

  servoMotor.attach(9); // Conectarea Motorului Servo la pinul digital 9.

  // Setarea senzorului de detectare moleculelor H2O in calitate de OUTPUT.
  pinMode(DigitalWaterSensor, OUTPUT);
  digitalWrite(DigitalWaterSensor, LOW); // Senzorul de detectarea a apei deconectat.

     }

// Functia de afisare a passcode-ului.
void displayDigit(int digit, int pos) {
  lcd.setCursor(pos, 1);
  lcd.print(digit);
}

// Functia de logare cu modulul RFID-RC522 si Butoanele B1 si B2.
void login() {
  
  // Setarea LED-ului RGB la culoare Rosie.
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);
  digitalWrite(blueLed, LOW);

  // Mesaj Informativ.
  lcd.setCursor(0, 0);  
  lcd.print("To have access..");
  lcd.setCursor(0, 1);  
  lcd.print("<- Approach tag");

  if (RC522.PICC_IsNewCardPresent() && RC522.PICC_ReadCardSerial()) { 

    // Logare cu succes, apropiind TAG-ul.
    lcd.clear();  
    lcd.setCursor(0, 0);  
    lcd.print("Access granted");
    tone(BUZZER_PIN, 5000, 0);
    delay(1500);
    noTone(BUZZER_PIN);  
    delay(1500);
    lcd.clear();   
    RC522.PICC_HaltA();  
    RC522.PCD_StopCrypto1(); 

    // Mesaj Informativ.
    lcd.setCursor(0, 0);  
    lcd.print("Next step :");  
    lcd.setCursor(0, 1);  
    lcd.print("2-digit passcode");  
    delay(3000);

    // Mesaj Informativ.
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("(!) Key Code :");
    lcd.setCursor(0, 1);
    lcd.print("sel: 0->9 & 0->9");

  while (!isLoggedIn) {

    buttonState1 = digitalRead(buttonPin1);
    buttonState2 = digitalRead(buttonPin2);
    
    // Selectarea primului digit in intervalul [0...9].
    if (buttonState1 != lastButtonState1) {
      if (buttonState1 == HIGH) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("(!) Key Code :");
        digit1++;
        tone(BUZZER_PIN, 1000,0);
        delay(100);
        noTone(BUZZER_PIN);
        if (digit1 > 9) {
          digit1 = 0;
        }
        displayDigit(digit1, 0);
      }
      lastButtonState1 = buttonState1;
    }

    // Selectarea digit-ului nr.2 in intervalul [0...9].
    if (buttonState2 != lastButtonState2) {
      if (buttonState2 == HIGH) {
        digit2++;
        tone(BUZZER_PIN, 1000,0);
        delay(100);
        noTone(BUZZER_PIN);
        if (digit2 > 9) {
          digit2 = 0;
        }
        displayDigit(digit2, 1);
      }
      lastButtonState2 = buttonState2;
    }

    if (digit1 != -1 && digit2 != -1 && (digit1 * 10 + digit2) == 35) {
 
      // Daca digit1 = 3 si digit2 = 5, atunci logare cu succes.
      isLoggedIn = true;

            // Conectarea LED-ului RGB la culoare Verde.
            digitalWrite(redLed, LOW);
            digitalWrite(greenLed, HIGH);
            digitalWrite(blueLed, LOW);

            // Mesaj de salut pentru User-ul logat cu succes.
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Welcome Home !");
            lcd.setCursor(0, 1);
            lcd.print("User Mihai.");
            delay(3000);
            digit1 = -1;
            digit2 = -1;
            lcd.clear();
         }
       }
     }
   } 

// Bucla Infinita.
void loop() {

  if (!isLoggedIn) {

    // Daca utilizatorul nu a fost logat, se apeleaza functia de logare login().
    login();

  } else {
    
    // Citirea parametrilor gasLevel, temp(temperatura), hum(umiditatea).
    int gasLevel = analogRead(MQ2_PIN);
    float temp = dht.readTemperature();
    int hum = dht.readHumidity();

    // Citirea parametrului Gyro pe axele OX, OY, OZ.
    mpu.update();
    float gx = mpu.getGyroX();
    float gy = mpu.getGyroY();
    float gz = mpu.getGyroZ();

    // Parametrul de determinare/luare de decizie, daca a fost detectata miscare cinematica al Casei.
    float totalMovement = abs(gx) + abs(gy) + abs(gz);  

    // Parametrul de determinare/luare de decizie, daca au fost detectate molecule de H2O.
    int waterLevel = readWaterSensor();
  
// Daca a fost detectata miscare in stricta corespondenta cu centrul de masa...
  if (totalMovement > 100) { 
      
      // Afisarea mesajului, a fost detectat Cutremur de pamant ...
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("(!) Earthquake");
      lcd.setCursor(0, 1);
      lcd.print("Get out now ->>");
      servoMotor.write(-180);

      // Alarma sonora la detectarea cutremului cu o frecventa f = 5000.
      tone(BUZZER_PIN, 5000, 0);
      delay(1000);
      noTone(BUZZER_PIN);

      // Alarma luminiscenta prin conectarea LED-ului Rosu.
      digitalWrite(greenLed, LOW);
      digitalWrite(blueLed, LOW);
      digitalWrite(redLed, HIGH);
      delay(800);
      digitalWrite(redLed, LOW);

  } else {

      // Verificarea daca temp si hum nu sunt Not_a_Number (NaN).
      if (!isnan(temp) && !isnan(hum)) {

        // Conectarea culorii Albe.
        digitalWrite(redLed, HIGH);
        digitalWrite(greenLed, HIGH);
        digitalWrite(blueLed, HIGH);

        // Daca temperatura depaseste 30*C sau nivelul de CO2 mai mare sau egal decat 100 unitati fizice.
        if ((temp >= 30.0) || (gasLevel>=150)){

          // Fereastra fiind deschisa(rotirea SG90 la -180*)
          servoMotor.write(-180);

          // Ventilatorul pornit(Motorul DC pornit).
          digitalWrite(relayPin, HIGH);

          // Mesaj Informativ.
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("(!)High T.,GasL.");
          lcd.setCursor(0, 1);
          lcd.print("Vent.sys.started");

          // Daca moleculele H2O detectate pe range-ul [1 ... 500].
        } else if ((waterLevel>=1)&&(waterLevel<=500)){

           // Feereastra se inchide(SG90 rotit la 180*)
           servoMotor.write(180);

           // Mesaj Informativ.
           lcd.clear();
           lcd.setCursor(0, 0);
           lcd.print("(!)Rain started");
           lcd.setCursor(0, 1);
           lcd.print("[] Window Closed");

          }else {
        
        // Afisare in timp real al HH(orei) : MM(minutei), T*C, Umiditatii si nivelului de Gaz.
        servoMotor.write(180);
        digitalWrite(relayPin, LOW);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("T:");
        lcd.setCursor(2, 0);
        lcd.print(temp);
        lcd.print((char)223);
        lcd.print("C");

        lcd.setCursor(11, 0);
        lcd.print("H:");
        lcd.setCursor(13, 0);
        lcd.print(hum);
        lcd.print("%");
        
        lcd.setCursor(0, 1);
        lcd.print("AQI:");
        lcd.setCursor(4, 1);
        lcd.print(gasLevel);
        lcd.print("pu");
        
        //Apelarea functiei pentru setarea orei si minutelor de la Monitorul Serial.
        updateTime();
 
        //Apelarea functiei pentru afisarea orei si minutelor pe LCD.
        displayTime();  

          }
        }
         delay(1000);  //Data update la fiecare secunda.
     }
   }
  }

// Functia pentru citirea prezentei moleculelor de H2O.
int readWaterSensor(){

  digitalWrite(DigitalWaterSensor, HIGH);
  int readwaterLevel = analogRead(AnalogWaterSensor);
  delay(1000);
  return readwaterLevel;

}

// Functia pentru setarea orei si minutelor de la Monitorul Serial.
void updateTime() {

  if (Serial.available() > 0) { 
    String input = Serial.readStringUntil('\n'); 
    if (input.startsWith("set ")) { 
      input.remove(0, 4); 
      int colonIndex = input.indexOf(":"); 
      if (colonIndex >= 0) { 
        hour = input.substring(0, colonIndex).toInt(); 
        minute = input.substring(colonIndex + 1).toInt(); 
        Serial.println("Time set to " + String(hour) + ":" + String(minute)); 
      }
    }
  }
}

// Functia pentru afisarea orei si minutelor pe LCD.
void displayTime() {

  static unsigned long lastTime = 0; 
  unsigned long currentTime = millis(); 

  if (currentTime - lastTime >= 1000) { 
    lastTime = currentTime; 
    seconds++;
   if (seconds==60){
    seconds = 0;
    minute++; 
    if (minute == 60) { 
      minute = 0; 
      hour++; 
      if (hour == 24) { 
        hour = 0; 
      }
    }
  }
    lcd.setCursor(11, 1); 
    if (hour < 10) { 
      lcd.print("0");
    }
    lcd.print(hour); 
    lcd.print(":"); 
    if (minute < 10) { 
      lcd.print("0");
    }
    lcd.print(minute); 
  }
}
