// inclusion de toutes les librairies necessaire à la station
#include <BH1750.h> // Capteur de lumière 
#include <DHT.h>  // température et humidité
#include <RTClib.h> //horloge
#include <SdFat.h>  // Carte SD
#include <SPI.h> // protocole de la carte SD
#include <Wire.h> // bibliothèque lumière
#include <EEPROM.h> // Memoire morte
#include <SoftwareSerial.h> //Interruptions
#include <TinyGPS.h> // GPS

#define DHTPIN 4  // Pin digital connecté au capteur DHT

#define DHTTYPE DHT11  // Type DHT 11

// définition des pins de la led RGB
#define RLED 9
#define GLED 6
#define BLED 5

// définition des pins des boutons d'interruption (Bascule entre les modes)
#define GREENBUT 3
#define REDBUT 2

// définition des RX et TX du gps pour SoftwareSerial
#define GPSRX 7
#define GPSTX 8

// varibale de type enum pour basculer entre les modes
enum mode { standard,
            configuration,
            maintenance,
            economique };
mode Mode;
mode previousMode = 0;

// type enum pour les types d'erreurs
enum error {
  ertc,  // Erreur d’accès à l’horloge RTC
  egps,  // Erreur d’accès aux données du GPS
  ecap,  // Erreur accès aux données d’un capteur
  einc,  // Données  reçues  d’un  capteur  incohérentes  - vérification matérielle requise
  esdp,  // Carte SD pleine
  eesd,  // Erreur d’accès ou d’écriture sur la carte SD
};

// création des objets des capteurs
RTC_DS1307 RTC;
DHT dht(DHTPIN, DHTTYPE);
BH1750 LightSensor;

// creation objet fichier et sd
SdFat SD;

// parametres sauvegardés dans l'eeprom
int logInterval;           // 0 D:24
unsigned int fileMaxSize;  // 2 D:26
int timeout;               // 4 D:28
bool lumin;                // 6 D:30
unsigned char luminLow;    // 8 D:32
unsigned int luminHigh;    // 10 D:34
bool tempAir;              // 12 D:36
int minTempAir;            // 14 D:38
int maxTempAir;            // 16 D:40
bool hygr;                 // 18 D:42
int hygrMinT;              // 20 D:44
int hygrMaxT;              // 22 D:46

// variables pour la gestion des intervales de lecture
bool intervalEnd = true;
DateTime start;

// variables pour gerer les inerruptions
int flagRB = 0;
int flagGB = 0;
unsigned long rinterruptStart = 0;
unsigned long ginterruptStart = 0;
unsigned long time = 0;

// structure mesure
struct Mesure {
  float lux;
  float temp;
  float humi;
  String lightLevel = "";
  String temperature = "";
  String humidity = "";
};

// setup
void setup() {
  Wire.begin();
  // démarrage du port série
  Serial.begin(9600);
  while (!Serial)
    ;
  // démarrage de la carte SD et vérification
  if (!SD.begin(SS)) {
    Serial.println(F("er:SD"));
    // Erreur d’accès ou d’écriture sur la carte SD
    error(eesd);
    while (1)
      ;
  }

  if (!RTC.begin()) {
    Serial.println(F("er:RTC"));
    // Erreur d’accès à l’horloge RTC
    error(ertc);
    while (1)
      ;
  }

  if (!RTC.isrunning()) {
    // ajuste la RTC à la date et heure aux quelles le programme a été compilé
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  // démarrage des capteurs
  dht.begin();
  LightSensor.begin(BH1750::CONTINUOUS_LOW_RES_MODE);



  // initialisation des pins de la led rbg comme sorties
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT);
  rgbStartup();

  pinMode(GREENBUT, INPUT);
  pinMode(REDBUT, INPUT);

  start = RTC.now();

  if (digitalRead(REDBUT) == HIGH) Mode = configuration;
  else
    Mode = standard;
  ////Serial.println("setup");
  // définition des interruptions
  attachInterrupt(digitalPinToInterrupt(GREENBUT), greenToggle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REDBUT), redToggle, CHANGE);
  digitalWrite(GREENBUT, LOW);
  digitalWrite(REDBUT, LOW);
  EEPROM.get(0, logInterval);
  EEPROM.get(2 * sizeof(int), fileMaxSize);
  EEPROM.get(4 * sizeof(int), timeout);
  EEPROM.get(6 * sizeof(int), lumin);
  EEPROM.get(8 * sizeof(int), luminLow);
  EEPROM.get(10 * sizeof(int), luminHigh);
  EEPROM.get(12 * sizeof(int), tempAir);
  EEPROM.get(14 * sizeof(int), minTempAir);
  EEPROM.get(16 * sizeof(int), maxTempAir);
  EEPROM.get(18 * sizeof(int), hygr);
  EEPROM.get(20 * sizeof(int), hygrMinT);
  EEPROM.get(22 * sizeof(int), hygrMaxT);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  if (flagRB == 1) {
    time = millis();
    
    if (time - rinterruptStart >= 5000) {
      time = millis();
      rinterruptStart = time;
      intervalEnd = true;
      if (Mode == standard || Mode == economique) {     
        Mode = maintenance;
        Serial.println(F("M"));
      } else if (Mode == maintenance) {
        Mode = previousMode;
        switch (previousMode) {
          case standard:
            Serial.println(F("S"));
            break;
          case economique:
            Serial.println(F("E"));
            break;
        }
      }
    }
  }

  if (flagGB == 1) {
    time = millis();
    if (time - ginterruptStart >= 5000) {
      time = millis();
      ginterruptStart = time;
      
      intervalEnd = true;
      if (Mode == standard) {
        Mode = economique;
        Serial.println(F("E"));
      } else if (Mode == economique) {
        Mode = standard;
        Serial.println(F("S"));
      }
    }
  }
  
  switch (Mode) {
    case 0:
      mStandard();
      break;

    case 1:
      mConfig();
      break;

    case 2:
      mMaint();
      break;

    case 3:
      mEco();
      break;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Mode Standard
void mStandard() {
  setRgbColor(0, 255, 0);
  previousMode = standard;
  ////
  DateTime now = RTC.now();
  if (intervalEnd) {
    sdVerif();
    String dataString = "";
    readCaptors(&dataString);
    //getGpsData(&dataString);
    Serial.println(dataString);
    saveToSd(&dataString);
  }
  ////

  if (now.unixtime() - start.unixtime() < logInterval) {
    intervalEnd = false;
  } else {
    intervalEnd = true;
    start = RTC.now();
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Mode Config
void mConfig() {
  setRgbColor(255, 255, 0);
  //Serial.println("config");
  DateTime now = RTC.now();
  //Serial.println(now.unixtime()-start.unixtime());
  if (Serial.available() > 0) {

    start = RTC.now();
    String command = Serial.readString();
    command = command.substring(0, command.indexOf('\n'));
    command.replace(" ", "");
    Serial.println(command);
    int equalIndex = command.indexOf('=');

    if (equalIndex == -1) {  // commandes ne contenants pas de signe egale: RESET, VERSION
      if (command.equalsIgnoreCase(F("reset"))) {
        //Serial.println(F("Reset"));

        EEPROM.get(24 * sizeof(int), logInterval);
        EEPROM.get(26 * sizeof(int), fileMaxSize);
        EEPROM.get(28 * sizeof(int), timeout);
        EEPROM.get(30 * sizeof(int), lumin);
        EEPROM.get(32 * sizeof(int), luminLow);
        EEPROM.get(34 * sizeof(int), luminHigh);
        EEPROM.get(36 * sizeof(int), tempAir);
        EEPROM.get(38 * sizeof(int), minTempAir);
        EEPROM.get(40 * sizeof(int), maxTempAir);
        EEPROM.get(42 * sizeof(int), hygr);
        EEPROM.get(44 * sizeof(int), hygrMinT);
        EEPROM.get(46 * sizeof(int), hygrMaxT);

        EEPROM.put(0 * sizeof(int), logInterval);
        EEPROM.put(2 * sizeof(int), fileMaxSize);
        EEPROM.put(4 * sizeof(int), timeout);
        EEPROM.put(6 * sizeof(int), lumin);
        EEPROM.put(8 * sizeof(int), luminLow);
        EEPROM.put(10 * sizeof(int), luminHigh);
        EEPROM.put(12 * sizeof(int), tempAir);
        EEPROM.put(14 * sizeof(int), minTempAir);
        EEPROM.put(16 * sizeof(int), maxTempAir);
        EEPROM.put(18 * sizeof(int), hygr);
        EEPROM.put(20 * sizeof(int), hygrMinT);
        EEPROM.put(22 * sizeof(int), hygrMaxT);

      } else if (command.equalsIgnoreCase(F("version"))) {
        Serial.println(F("V1\nlot:1"));
      } else {
        Serial.println(F("Unknown"));
      }
    } else {
      String val = command.substring(equalIndex + 1);
      String parameter = command.substring(0, equalIndex);
      if (parameter.equalsIgnoreCase(F("lumin")) || parameter.equalsIgnoreCase(F("temp_air")) || parameter.equalsIgnoreCase(F("hygr"))) {  // adresse eeprom lumin: 6*sizeof(int)
        if ((!val.equals("0") && val.toInt() == 0) || (val.toInt() != 0 && val.toInt() != 1)) {
          Serial.println(F("er:{0,1}"));
        } else {
          if (parameter.equalsIgnoreCase(F("lumin"))) {
            EEPROM.put(6 * sizeof(int), val.toInt());
          } else if (parameter.equalsIgnoreCase(F("temp_air"))) {
            EEPROM.put(12 * sizeof(int), val.toInt());
          } else {
            EEPROM.put(18 * sizeof(int), val.toInt());
          }
        }
      } else if (parameter.equalsIgnoreCase(F("lumin_low")) || parameter.equalsIgnoreCase(F("lumin_high"))) {  //adresse eeprom lumin_low: 4*sizeof(int), lumin_high: 5*sizeof(int)
        if ((!val.equals("0") && val.toInt() == 0) || val.toInt() < 0 || val.toInt() > 1023) {
          Serial.println(F("er:[0-1023]"));
        } else {
          if (parameter.equalsIgnoreCase(F("lumin_low"))) {
            EEPROM.put(8 * sizeof(int), val.toInt());
          } else {
            EEPROM.put(10 * sizeof(int), val.toInt());
          }
        }
      } else if (parameter.equalsIgnoreCase(F("min_temp_air")) || parameter.equalsIgnoreCase(F("max_temp_air")) || parameter.equalsIgnoreCase(F("hygr_mint")) || parameter.equalsIgnoreCase(F("hygr_maxt"))) {
        if ((!val.equals("0") && val.toInt() == 0) || val.toInt() < -40 || val.toInt() > 80) {
          Serial.println(F("er:[-40-85]"));
        } else {
          if (parameter.equalsIgnoreCase(F("min_temp_air"))) {
            EEPROM.put(14 * sizeof(int), val.toInt());
          } else if (parameter.equalsIgnoreCase(F("max_temp_air"))) {
            EEPROM.put(16 * sizeof(int), val.toInt());
          } else if (parameter.equalsIgnoreCase(F("hygr_mint"))) {
            EEPROM.put(20 * sizeof(int), val.toInt());
          } else {
            EEPROM.put(22 * sizeof(int), val.toInt());
          }
        }
      } else if (parameter.equalsIgnoreCase(F("log_interval")) || parameter.equalsIgnoreCase(F("file_max_size")) || parameter.equalsIgnoreCase(F("timeout"))) {
        if ((!val.equals("0") && val.toInt() == 0) || val.toInt() < 0) {
          Serial.println(F("er:>0"));
        } else {
          if (parameter.equalsIgnoreCase(F("log_interval"))) {
            EEPROM.put(0, val.toInt());
          } else if (parameter.equalsIgnoreCase(F("file_max_size"))) {
            EEPROM.put(2 * sizeof(int), val.toInt());
          } else {
            EEPROM.put(4 * sizeof(int), val.toInt());
          }
        }
      } else if (parameter.equalsIgnoreCase(F("clock"))) {
        if (val.substring(0, 2).toInt() < 0 || val.substring(0, 2).toInt() > 23 || val.substring(3, 5).toInt() < 0 || val.substring(3, 5).toInt() > 59 || val.substring(6).toInt() < 0 || val.substring(6).toInt() > 59 || val.length() != 8) {
          Serial.println(F("er:hh[0-23]:mm[0-59]:ss[0-59]"));
        } else {
          RTC.adjust(DateTime(RTC.now().year(), RTC.now().month(), RTC.now().day(), val.substring(0, 2).toInt(), val.substring(3, 5).toInt(), val.substring(6).toInt()));
          Serial.println(RTC.now().timestamp(DateTime::TIMESTAMP_TIME));
        }
      } else if (parameter.equalsIgnoreCase(F("date"))) {
        if (val.substring(0, 2).toInt() < 0 || val.substring(0, 2).toInt() > 12 || val.substring(3, 5).toInt() < 0 || val.substring(3, 5).toInt() > 31 || val.substring(6).toInt() < 2000 || val.substring(6).toInt() > 2099 || val.length() != 10) {
          Serial.println(F("er:MM[0-12],JJ[0-31],AAAA[2000-2099]"));
          now = RTC.now();
          start = now;
        } else {
          RTC.adjust(DateTime(val.substring(6).toInt(), val.substring(0, 2).toInt(), val.substring(3, 5).toInt(), RTC.now().hour(), RTC.now().minute(), RTC.now().second()));
          Serial.println(RTC.now().timestamp(DateTime::TIMESTAMP_DATE));
          now = RTC.now();
          start = now;
        }
      } else {
        Serial.println(F("Unknown"));
      }
    }
    Serial.end();
    Serial.begin(9600);
  }

  if (now.unixtime() - start.unixtime() >= 15) {
    Mode = standard;
    start = RTC.now();
    for (int i = 0; i <= 23; i += 2) {
      int p;
      EEPROM.get(i * sizeof(int), p);
      Serial.println(p);
    }
    Serial.println(F("S"));
    EEPROM.get(0, logInterval);
    EEPROM.get(2 * sizeof(int), fileMaxSize);
    EEPROM.get(4 * sizeof(int), timeout);
    EEPROM.get(6 * sizeof(int), lumin);
    EEPROM.get(8 * sizeof(int), luminLow);
    EEPROM.get(10 * sizeof(int), luminHigh);
    EEPROM.get(12 * sizeof(int), tempAir);
    EEPROM.get(14 * sizeof(int), minTempAir);
    EEPROM.get(16 * sizeof(int), maxTempAir);
    EEPROM.get(18 * sizeof(int), hygr);
    EEPROM.get(20 * sizeof(int), hygrMinT);
    EEPROM.get(22 * sizeof(int), hygrMaxT);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Mode Maintenance
void mMaint() {
  setRgbColor(255, 100, 0);
  DateTime now = RTC.now();
  if (intervalEnd) {
    String dataString = "";
    readCaptors(&dataString);
    Serial.println(dataString);
  }
  ////

  if (now.unixtime() - start.unixtime() < 1) {
    intervalEnd = false;
  } else {
    intervalEnd = true;
    start = RTC.now();
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Mode Economique
void mEco() {
  previousMode = economique;
  setRgbColor(0, 0, 255);
  DateTime now = RTC.now();
  if (intervalEnd) {
    sdVerif();
    String dataString = "";
    readCaptors(&dataString);
    Serial.println(dataString);
    saveToSd(&dataString);
  }
  //// ne pas oublier d'implementer le fais que le gps n'est lu qu'une fois sur 2

  if (now.unixtime() - start.unixtime() < logInterval * 2) {
    intervalEnd = false;
  } else {
    intervalEnd = true;
    start = RTC.now();
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// animation des leds au démarrage
void rgbStartup() {
  setRgbColor(255, 0, 0);
  delay(200);

  setRgbColor(0, 255, 0);
  delay(200);

  setRgbColor(0, 0, 255);
  delay(200);
  setRgbColor(0, 0, 0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// fonction pour simplifier l'utilisation de la LED RGB
void setRgbColor(byte r, byte g, byte b) {
  analogWrite(RLED, r);
  analogWrite(GLED, g);
  analogWrite(BLED, b);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// fonction de verification de l'espace disponible dans la carte SD
void sdVerif() {
  //Serial.println("sdverif");
  // Donne le nombre de clusters disponibles dans la carte SD
  uint32_t freeClusters = SD.vol()->freeClusterCount();
  // Calcule l'espace disponible en KB
  float freeKB = .512 * freeClusters * SD.vol()->sectorsPerCluster();
  if (freeKB <= fileMaxSize) {  // si l'espace disponible est inferieur au
    Serial.println(F("Carte SD pleine"));
    // Carte SD pleine
    error(esdp);
    Mode = maintenance;
    return;
  } else {
    //Serial.print("Espace disponible ");
    //Serial.print(freeKB);
    //Serial.println(" kB");
    return;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// fonction de lecture et verification des capteurs
void readCaptors(String* pdata) {
  //Serial.println("readcaptors");
  DateTime timeoutStart;
  DateTime now;
  struct Mesure mesure;

  // mesure luminosité
  if (lumin) {  // verifier en eeprom si le capteur de luminosité est activé
    mesure.lux = LightSensor.readLightLevel();

    if (mesure.lux == -1) {
      timeoutStart = RTC.now();
      now = timeoutStart;
      while (mesure.lux == -1 && (now.unixtime() - timeoutStart.unixtime() < timeout)) {
        now = RTC.now();
        LightSensor.begin();
        mesure.lux = LightSensor.readLightLevel();
      }
      if (mesure.lux == -1) {
        mesure.lightLevel = "NA";
        error(ecap);
      }
    }
  }

  // mesure temperature
  if (tempAir) {
    mesure.temp = dht.readTemperature();

    if (isnan(mesure.temp)) {
      timeoutStart = RTC.now();
      now = timeoutStart;
      while ((isnan(mesure.temp)) && (now.unixtime() - timeoutStart.unixtime() < timeout)) {
        now = RTC.now();
        dht.begin();
        mesure.temp = dht.readTemperature();
      }
      if (isnan(mesure.temp)) {
        mesure.temperature = "NA";
        error(ecap);
      }
    }
  }

  // mesure humidité
  if (hygr) {
    mesure.humi = dht.readHumidity();
    if (isnan(mesure.humi)) {
      timeoutStart = RTC.now();
      now = timeoutStart;
      while ((isnan(mesure.humi)) && (now.unixtime() - timeoutStart.unixtime() < timeout)) {
        now = RTC.now();
        dht.begin();
        mesure.humi = dht.readHumidity();
      }
      if (isnan(mesure.humi)) {
        mesure.humidity = "NA";
        error(ecap);
      }
    }
  }

  // verification coherences
  checkCoherence(&mesure);
  DateTime readTime = RTC.now();
  *pdata += readTime.timestamp(DateTime::TIMESTAMP_TIME) + " ; ";
  if (lumin) *pdata += mesure.lightLevel + " ; ";
  if (tempAir) *pdata += mesure.temperature + "C ; ";
  if (hygr) *pdata += mesure.humidity + "%";
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// fonction de verification de la coherence des données
void checkCoherence(struct Mesure* input) {
  ////coherence temperature
  //Serial.println("checkcoherence");
  if (tempAir) {
    if (!((*input).temperature.equals("NA"))) {
      if ((*input).temp < minTempAir || (*input).temp > maxTempAir) {
        // erreur de coherence
        error(einc);
        // mise en pause/attente coupure du systeme
        while (1)
          ;
      } else {
        (*input).temperature = String((int)(*input).temp);
      }
    }
  }

  ////coherence humidité par rapport a la temperature
  if (hygr) {
    if (!((*input).humidity.equals("NA"))) {
      if (((*input).temp < hygrMinT || (*input).temp > hygrMaxT) && tempAir) {
        (*input).humidity = "NC";  // la valeur n'est pas prise en compte "Not Considered"
      } else {
        (*input).humidity = String((int)(*input).humi);
      }
    }
  }

  //// coherence luminosité
  if (lumin) {
    if (!((*input).lightLevel.equals("NA"))) {
      if ((*input).lux < luminLow && (*input).lux >= 0) {
        (*input).lightLevel = "LOW";
      } else if ((*input).lux <= luminHigh) {
        (*input).lightLevel = "MID";
      } else if ((*input).lux > luminHigh && (*input).lux < sizeof(int)) {
        (*input).lightLevel = "HIG";
      } else {
        error(einc);
        while (1)
          ;
      }
    }
  }
}

// fonction de sauvegarde sur la carte SD
void saveToSd(String* pdata) {
  //Serial.println("savetoSD");
  SdFile SaveFile;
  char filename[] = "000000_00.log";
  getFileName(&SaveFile, filename, RTC.now().year() - 2000, RTC.now().month(), RTC.now().day());

  SaveFile.println(*pdata);
  SaveFile.FatFile::flush();
  SaveFile.close();
}

// fonction pour recuperer le nom du fichier ou enregistrer et verifier s'il depasse ou non le file_max_size, en creeer un nouveau si oui
void getFileName(SdFile* SaveFile, char* filename, uint8_t y, uint8_t m, uint8_t d) {
  //Serial.println("getfilemame");
  static int filenum = 0;
  sprintf(filename, "%02u%02u%02u_%02u.log", y, m, d, filenum);
  (*SaveFile).open(filename, O_RDWR | O_CREAT | O_AT_END);
  //Serial.println((*SaveFile).fileSize());
  while ((*SaveFile).fileSize() >= fileMaxSize) {
    (*SaveFile).close();
    filenum++;
    sprintf(filename, "%02u%02u%02u_%02u.log", y, m, d, filenum);
    (*SaveFile).open(filename, O_RDWR | O_CREAT | O_AT_END);
  }
}

// fonction d'interruption bouton vert
void greenToggle() {
   if (flagGB == 1) {
    ginterruptStart = millis();
    flagGB = 0;
    Serial.println("GR");
    return;
  }
  if (flagGB == 0) {
    ginterruptStart = millis();
    flagGB = 1;
    Serial.println("GP");
    return;
  }
}

// fonction d'interruption bouton rouge
void redToggle() {
  if (flagRB == 1) {
    rinterruptStart = millis();
    flagRB = 0;
    Serial.println("RR");
    return;
  }
  if (flagRB == 0) {
    rinterruptStart = millis();
    flagRB = 1;
    Serial.println("RP");
    return;
  }
}



// fonction d'erreur
void error(enum error errorType) {
  //ertc, // Erreur d’accès à l’horloge RTC
  //egps, // Erreur d’accès aux données du GPS
  //ecap, // Erreur accès aux données d’un capteur
  //einc, // Données  reçues  d’un  capteur  incohérentes  - vérification matérielle requise
  //esdp, // Carte SD pleine
  //eesd, // Erreur d’accès ou d’écriture sur la carte SD
  switch (errorType) {
    case ertc:
      for (byte i = 0; i < 5; i++) {
        setRgbColor(255, 0, 0);
        delay(500);
        setRgbColor(0, 0, 255);
        delay(500);
      }
      break;
    case egps:
      for (byte i = 0; i < 5; i++) {
        setRgbColor(255, 0, 0);
        delay(500);
        setRgbColor(255, 255, 0);
        delay(500);
      }
      break;
    case ecap:
      for (byte i = 0; i < 5; i++) {
        setRgbColor(255, 0, 0);
        delay(500);
        setRgbColor(0, 255, 0);
        delay(500);
      }
      break;
    case einc:
      for (byte i = 0; i < 5; i++) {
        setRgbColor(255, 0, 0);
        delay(333);
        setRgbColor(0, 255, 0);
        delay(667);
      }
      break;
    case esdp:
      for (byte i = 0; i < 5; i++) {
        setRgbColor(255, 0, 0);
        delay(500);
        setRgbColor(255, 255, 255);
        delay(500);
      }
      break;
    case eesd:
      for (byte i = 0; i < 5; i++) {
        setRgbColor(255, 0, 0);
        delay(333);
        setRgbColor(255, 255, 0);
        delay(667);
      }
      break;
  }
}
/*
void getGpsData(String *pdata) {
  // creation des objets pour le gps
  SoftwareSerial gpsSerial(GPSRX,GPSTX);
  TinyGPS gps;
  // démarrage capteur GPS
  gpsSerial.begin(9600);
  float lat = 0.0, lon = 0.0;
  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      gps.f_get_position(&lat, &lon);
    }
  }
  *pdata += String(lat) + String(lon);
}*/