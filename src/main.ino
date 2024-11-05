#include <Arduino.h>         // Generic + Arduino
#include <Wire.h>            // Generic + Sensors
#include <SPI.h>             // Generic + SD Card
#include <SoftwareSerial.h>  // Generic + GPS
#include <stdlib.h>          // Generic + Malloc
#include <avr/pgmspace.h>    // Generic + PROGMEM
#include <EEPROM.h>          // Generic + EEPROM
#include <SD.h>              // Library + SD Card
#include <Adafruit_Sensor.h> // Library + Sensors
#include <Adafruit_BME280.h> // Library + Sensors
#include <ChainableLED.h>    // Library + LEDS
#include <DS1307.h>          // Library + RTC Battery

#define BOUTON_ROUGE 2     // Pin bouton rouge
#define BOUTON_VERT 3      // Pin bouton vert
#define LED_1 8            // Pin de la led n°1
#define LED_2 9            // Pin de la led n°2
#define NOMBRE_LEDS 1      // Nombre de led succesivement branchés
#define CAPTEUR_LUMIERE A0 // Pin capteur lumière, analogique
#define INTENSITY 255      // Utilisé pour les LEDs [0 - 255]

#define STAND 1
#define CONFIG 2
#define ECO 3
#define MAINT 4
#define ERR_RTC 5
#define ERR_GPS 6
#define ERR_CAP 7
#define ERR_CAP_INC 8
#define ERR_SD_PLEIN 9
#define ERR_SD 10
#define NUM_MODES 11

struct SData
{
  char latitude[16];
  char latDir[2];
  char longitude[16];
  char lonDir[2];
  char dGPS[32]; // Buffer pour stocker les données GPS formatées
  char uGPS[32];
  char dCap[64];
} sData; // Structure utilsié pour les données du GPS

struct SystemState
{
  byte mode;
  byte ledMode;
  bool ledSwitch;
  bool boutonRouge;
  bool boutonVert;
  bool messageDisplayed = false;
  bool gpsDataNeeded = true;
} systemState; // Structure utilisé pour tout ce qui est générique dans le code. Les flags etc...

struct Time
{
  signed char TLightFix;
  signed char TStdFix;
  signed char TRedFix;   // Temps du bouton rouge
  signed char TGreenFix; // Temps du bouton vert
} time;                  // Structure utilisé exlusivement pour les comparatifs de temps.

struct DConfiguration
{
  uint16_t LOG_INTERVAL;  // minute
  uint32_t FILE_MAX_SIZE; // octet
  uint8_t LUMIN;          // flag (why not boolean ?)
  uint16_t LUMIN_LOW;     // !
  uint16_t LUMIN_HIGH;    // !
  uint8_t TEMP_AIR;       // flag (why not boolean ?)
  uint8_t TIMEOUT;        // seconde
  int8_t MIN_TEMP_AIR;    // °C
  int8_t MAX_TEMP_AIR;    // °C
  uint8_t HYGR;           // flag (why not boolean ?)
  int8_t HYGR_MINT;       // °C
  int8_t HYGR_MAXT;       // °C
  uint8_t PRESSURE;       // flag (why not boolean ?)
  uint16_t PRESSURE_MIN;  // HPa
  uint16_t PRESSURE_MAX;  // HPa
} config;                 // Structure des variables principales

ChainableLED leds(LED_1, LED_2, NOMBRE_LEDS); // Objet : LED
DS1307 clock;                                 // Objet : Horloge RTC
File data;                                    // Objet : Fichier pour carte SD
File newFile;                                 // Objet : Fichier pour carte SD
Adafruit_BME280 bme;                          // Objet : capteurs
SoftwareSerial gps(5, 6);                     // Objet : gps

bool debugMode = true; // osef un peu de celui la

// Setup
void setup()
{
  systemState.ledSwitch = true;
  Serial.begin(9600);
  gps.begin(9600);
  EEPROM.get(0, config);
  pinMode(BOUTON_ROUGE, INPUT);    // bouton rouge
  pinMode(BOUTON_VERT, INPUT);     // bouton vert
  pinMode(CAPTEUR_LUMIERE, INPUT); // capteur lumière
  leds.setColorRGB(0, 0, 0, 0);
  clock.begin();
  clock.getTime();
  time.TStdFix = clock.second;   // en seconde, plus tard en minutes.
  time.TLightFix = clock.second; // en seconde, plus tard en minutes.
  while (!bme.begin(0x76))       // Initialisation du capteur température
  {
    Light(ERR_CAP);
  }
  while (!SD.begin(4))
  {
    clock.getTime();
    Light(ERR_SD);
  }
  Light(-1);
  systemState.mode = systemState.ledMode = digitalRead(BOUTON_ROUGE) ? STAND : CONFIG;
  time.TStdFix = digitalRead(BOUTON_ROUGE) ? clock.second : clock.minute;
}

// Boucle principale
void loop()
{
  clock.getTime();
  Light(systemState.ledMode);
  switch (systemState.mode)
  {
  case STAND:
    Standard(&time, &systemState);
    break;
  case ECO:
    Standard(&time, &systemState);
    break;
  case CONFIG:
    Configuration(&config);
    break;
  case MAINT:
    Maintenance(&time, &systemState);
    break;
  default:
    break;
  }
}

// Gestion des modes

// Standard et Eco:
void Standard(Time *time, SystemState *systemState)
{
  static char name[16];
  snprintf(name, sizeof(name), "%02d%02d%02d_0.LOG", clock.year, clock.month, clock.dayOfMonth);
  if (clock.second < time->TStdFix) // ou minute
  {
    time->TStdFix -= 60;
  }
  if (clock.second >= time->TStdFix + ((systemState->mode == ECO) ? config.LOG_INTERVAL * 2 : config.LOG_INTERVAL))
  {
    time->TStdFix = clock.second; // ou minute
    if (systemState->mode == ECO)
    {
      if (systemState->gpsDataNeeded)
      {
        systemState->gpsDataNeeded = false;
      }
      else
      {
        systemState->gpsDataNeeded = true;
      }
    }
    systemState->ledMode = ((systemState->mode == STAND) ? STAND : ECO);
    if (data)
    {
      data.close();
    
    File data = SD.open(name, FILE_WRITE);
    if (data)
    {
      if (data.size() >= config.FILE_MAX_SIZE)
      {
        {
          uint8_t revision = 0;
          char newName[16];
          snprintf(newName, sizeof(newName), "%02d%02d%02d_0.LOG", clock.year, clock.month, clock.dayOfMonth);
          while (SD.exists(newName))
          {
            snprintf(newName, sizeof(newName), "%02d%02d%02d_%d.LOG", clock.year, clock.month, clock.dayOfMonth, revision++);
          }
          File newFile = SD.open(newName, FILE_WRITE);

          if (newFile)
          {
            data.seek(0);
            while (data.available())
            {
              newFile.write(data.read());
            }
            Serial.println(F("Debug : Fichier copié"));
            data.close();
            newFile.close();
            SD.remove(name);
          }
          else
          {
            systemState->ledMode = ERR_SD;
            data.close();
            return;
          }
          data = SD.open(name, FILE_WRITE);
          if (!data)
          {
            systemState->ledMode = ERR_SD;
          }
        }
      }
      if (systemState->gpsDataNeeded)
      {
        // GetGPS(&sData);
      }
      GetSensorData(&sData);
      data.print("{");
      data.print(clock.hour);
      data.print(":");
      data.print(clock.minute);
      data.print(":");
      data.print(clock.second);
      data.print(sData.dCap);               // ";LIGHT;TEMP000;PRESS00;HUM00;" // sData.dCap
      data.print("4319.0757N;00018.5195W"); // "4319.0757N;00018.5195W" // sData.dGPS
      data.println("}");
      data.close();
    }
    else
    {
      systemState->ledMode = ERR_SD;
    }
  }

  handleButtons();
}
}

// Maintenance :
void Maintenance(Time *time, SystemState *systemState)
{
  if (clock.second < time->TStdFix) // ou minute
  {
    time->TStdFix -= 60;
  }
  // if (clock.minute >= time->TStdFix + ((systemState->mode == ECO) ? config.LOG_INTERVAL * 2 : config.LOG_INTERVAL))
  if (clock.second >= time->TStdFix + ((systemState->mode == STAND) ? 5 : 10)) // DEBUG
  {
    time->TStdFix = clock.second; // ou minute
    GetSensorData(&sData);
    Serial.print(F("{"));
    Serial.print(clock.hour);
    Serial.print(F(":"));
    Serial.print(clock.minute);
    Serial.print(F(":"));
    Serial.print(clock.second);
    Serial.print(sData.dCap);               // ";LIGHT;TEMP000;PRESS00;HUM00;" // sData.dCap
    Serial.print("4319.0757N;00018.5195W"); // "4319.0757N;00018.5195W" // sData.dGPS
    Serial.println(F("}"));
  }

  handleButtons();
}

// Configuration :
void Configuration(DConfiguration *config)
{

  if (clock.minute < time.TStdFix)
  {
    time.TStdFix -= 60;
  }
  // if (clock.minute >= time.TStdFix + (systemState.mode == ECO) ? config->LOG_INTERVAL * 2 : config->LOG_INTERVAL)
  if (clock.minute >= time.TStdFix + 5) // 1 -> 30
  {
    time.TStdFix = clock.minute;
    systemState.mode = STAND;
    systemState.ledMode = systemState.mode;
    SD.begin(4);
  }

  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println(command);

    if (command.startsWith("LOG_INTERVAL="))
    {
      uint16_t value = command.substring(13).toInt();
      if (value > 0)
      {
        config->LOG_INTERVAL = value;
        Serial.println(F("LOG_INTERVAL mis à jour."));
        EEPROM.put(0, *config);
      }
    }
    else if (command.startsWith("FILE_MAX_SIZE="))
    {
      uint16_t value = command.substring(14).toInt();
      if (value > 0)
      {
        config->FILE_MAX_SIZE = value;
        Serial.println(F("FILE_MAX_SIZE mis à jour."));
        EEPROM.put(0, *config);
      }
    }
    else if (command.startsWith("RESET"))
    {
      config->LOG_INTERVAL = 600;
      config->FILE_MAX_SIZE = 4096;
      config->LUMIN = 1;
      config->LUMIN_LOW = 255;
      config->LUMIN_HIGH = 768;
      config->TEMP_AIR = 1;
      config->TIMEOUT = 30;
      config->MIN_TEMP_AIR = -10;
      config->MAX_TEMP_AIR = 60;
      config->HYGR = 1;
      config->HYGR_MINT = 0;
      config->HYGR_MAXT = 50;
      config->PRESSURE = 1;
      config->PRESSURE_MIN = 850;
      config->PRESSURE_MAX = 1080;
    }
    else if (command.startsWith("EXIT"))
    {

      time.TStdFix = clock.minute;
      systemState.mode = STAND;
      systemState.ledMode = systemState.mode;
      SD.begin(4);
    }
    else if (command.startsWith("VERSION"))
    {
      Serial.println(F("Version 1.0 - Lot #12345"));
    }
    else if (command.startsWith("LUMIN="))
    {
      uint16_t value = command.substring(6).toInt();
      if (value == 0 || value == 1)
      {
        config->LUMIN = value;
        if (debugMode)
          Serial.println(F("LUMIN mis à jour."));
        EEPROM.put(0, *config);
        if (debugMode)
          Serial.println(F("Configuration sauvegardée en EEPROM."));
      }
    }
    else if (command.startsWith("LUMIN_LOW="))
    {
      uint16_t value = command.substring(10).toInt();
      if (value >= 0)
      {
        config->LUMIN_LOW = value;
        if (debugMode)
          Serial.println(F("LUMIN_LOW mis à jour."));
        EEPROM.put(0, *config);
        if (debugMode)
          Serial.println(F("Configuration sauvegardée en EEPROM."));
      }
    }
    else if (command.startsWith("LUMIN_HIGH="))
    {
      uint16_t value = command.substring(11).toInt();
      if (value >= config->LUMIN_LOW)
      {
        config->LUMIN_HIGH = value;
        if (debugMode)
          Serial.println(F("LUMIN_HIGH mis à jour."));
        EEPROM.put(0, *config);
        if (debugMode)
          Serial.println(F("Configuration sauvegardée en EEPROM."));
      }
    }
    else if (command.startsWith("CLOCK"))
    {
      // Demande de l'heure
      Serial.println(F("Entrez l'heure (0-23) :"));
      while (Serial.available() == 0)
        ; // Attendre que l'utilisateur saisisse une valeur
      uint8_t hour = Serial.readStringUntil('\n').toInt();

      // Validation de l'heure
      if (hour < 0 || hour > 23)
      {
        Serial.println(F("Heure invalide. Doit être entre 0 et 23."));
        return; // Sortir si l'heure est invalide
      }

      // Demande des minutes
      Serial.println(F("Entrez les minutes (0-59) :"));
      while (Serial.available() == 0)
        ; // Attendre que l'utilisateur saisisse une valeur
      uint8_t minute = Serial.readStringUntil('\n').toInt();

      // Validation des minutes
      if (minute < 0 || minute > 59)
      {
        Serial.println(F("Minutes invalides. Doit être entre 0 et 59."));
        return; // Sortir si les minutes sont invalides
      }

      // Demande des secondes
      Serial.println(F("Entrez les secondes (0-59) :"));
      while (Serial.available() == 0)
        ; // Attendre que l'utilisateur saisisse une valeur
      uint8_t second = Serial.readStringUntil('\n').toInt();

      // Validation des secondes
      if (second < 0 || second > 59)
      {
        Serial.println(F("Secondes invalides. Doit être entre 0 et 59."));
        return; // Sortir si les secondes sont invalides
      }

      // Mettre à jour l'horloge
      clock.fillByHMS(hour, minute, second);
      clock.setTime();
      clock.getTime();
      Serial.print(F("Horloge mise à jour : "));
      Serial.print(clock.hour);
      Serial.print(F(":"));
      Serial.print(clock.minute);
      Serial.print(F(":"));
      Serial.println(clock.second);
    }
    else
    {
      if (debugMode)
        Serial.println(F("Commande non reconnue."));
    }
  }
}

// Gestion des boutons / changement de mode :
void handleButtons()
{
  if (!digitalRead(BOUTON_ROUGE))
  {
    if (systemState.boutonRouge)
    {
      if (clock.second < time.TRedFix)
      {
        time.TRedFix -= 60;
      }
      if (clock.second >= time.TRedFix + 5)
      {
        time.TRedFix = clock.second;
        systemState.boutonRouge = false;
        systemState.messageDisplayed = false;
        systemState.mode = (systemState.mode == MAINT) ? STAND : MAINT;
        systemState.gpsDataNeeded = true;
        SD.begin(4);
        systemState.ledMode = systemState.mode;
      }
    }
    else
    {
      time.TRedFix = clock.second;
      systemState.boutonRouge = true;
    }
  }
  else if (!digitalRead(BOUTON_VERT))
  {
    if (systemState.boutonVert)
    {
      if (clock.second < time.TGreenFix)
      {
        time.TGreenFix -= 60;
      }
      if (clock.second >= time.TGreenFix + 5)
      {
        time.TGreenFix = clock.second;
        systemState.boutonVert = false;
        systemState.mode = (systemState.mode == ECO) ? STAND : ECO;
        systemState.gpsDataNeeded = true;
        SD.begin(4);
        systemState.ledMode = systemState.mode;
      }
    }
    else
    {
      time.TGreenFix = clock.second;
      systemState.boutonVert = true;
    }
  }
  else
  {
    systemState.boutonRouge = false;
    systemState.boutonVert = false;
  }
}

// Gestions des capteurs

void GetSensorData(SData *data)
{
  byte lumiere = analogRead(CAPTEUR_LUMIERE);
  float temperature = bme.readTemperature();
  float pression = bme.readPressure() / 100.0F;
  float humidite = bme.readHumidity();

  char tempStr[10], pressStr[10], humidStr[10], lumiereStr[10]; // Ajouter un tampon séparé pour 'lumiere'
  floatToString(temperature, tempStr, 2);
  floatToString(pression, pressStr, 2);
  floatToString(humidite, humidStr, 2);
  itoa(lumiere, lumiereStr, 10); // Conversion de la lumière dans un tampon séparé

  strcpy(data->dCap, ";");
  strcat(data->dCap, lumiereStr);
  strcat(data->dCap, ";");
  strcat(data->dCap, tempStr);
  strcat(data->dCap, ";");
  strcat(data->dCap, pressStr);
  strcat(data->dCap, ";");
  strcat(data->dCap, humidStr);
  strcat(data->dCap, ";");
}

void floatToString(float value, char *buffer, int precision)
{
  int intPart = (int)value;
  int decimalPart = (int)((value - intPart) * pow(10, precision));
  sprintf(buffer, "%d.%d", intPart, abs(decimalPart));
}

// Gestion du GPS :
void GetGPS(SData *sData)
{
  memset(sData->dGPS, 0, sizeof(sData->dGPS));
  byte i = 0;
  bool gpggaDetected = false;
  while (gps.available())
  {
    char c = gps.read();
    if (c == '$' && gps.peek() == 'G')
    {
      sData->uGPS[0] = c;
      i = 1;
      delay(1);
    }
    else if (i > 0)
    {
      sData->uGPS[i++] = c;
      if (i >= 6 && strncmp(sData->uGPS, "$GPGGA", 6) == 0)
      {
        gpggaDetected = true;
      }
      if (c == '\n')
        break;
    }
  }
  if (gpggaDetected)
  {
    char *lat = nullptr, *latDir = nullptr, *lon = nullptr, *lonDir = nullptr;
    char *token = strtok(sData->uGPS, ",");
    int fieldIndex = 0;

    while (token != nullptr)
    {
      fieldIndex++;
      if (fieldIndex == 3)
        lat = token;
      if (fieldIndex == 4)
        latDir = token;
      if (fieldIndex == 5)
        lon = token;
      if (fieldIndex == 6)
        lonDir = token;
      token = strtok(nullptr, ",");
    }
    if (lat && *lat && lon && *lon && latDir && *latDir && lonDir && *lonDir)
    {
      snprintf(sData->dGPS, sizeof(sData->dGPS), "%s%s;%s%s", lat, latDir, lon, lonDir);
      if (strcmp(sData->dGPS, "00;MM") == 0)
      {
        strncpy(sData->dGPS, "Données GPS incomplètes.", sizeof(sData->dGPS) - 1);
        sData->dGPS[sizeof(sData->dGPS) - 1] = '\0';
      }
    }
    else
    {
      strncpy(sData->dGPS, "Données GPS absentes ou incomplètes.", sizeof(sData->dGPS) - 1);
      sData->dGPS[sizeof(sData->dGPS) - 1] = '\0';
    }
  }
  else
  {
    strncpy(sData->dGPS, "Aucune ligne GPGGA trouvée.", sizeof(sData->dGPS) - 1);
    sData->dGPS[sizeof(sData->dGPS) - 1] = '\0';
  }
  memset(sData->uGPS, 0, sizeof(sData->uGPS));
}

// Gestion des lumières : modes et erreurs

inline void Light(int ledMode)
{
  switch (ledMode)
  {
  case STAND:                             // Mode standard
    leds.setColorRGB(0, 0, INTENSITY, 0); // LED verte continue
    break;

  case CONFIG:                                    // Mode configuration
    leds.setColorRGB(0, INTENSITY, INTENSITY, 0); // LED jaune continue
    break;

  case ECO:                               // Mode économique
    leds.setColorRGB(0, 0, 0, INTENSITY); // LED bleue continue
    break;

  case MAINT:                              // Mode maintenance
    leds.setColorRGB(0, INTENSITY, 60, 0); // LED orange continue
    break;

  case ERR_RTC:                                                  // Erreur d'accès à l'horloge RTC
    setLedIntermittente(INTENSITY, 0, 0, 0, 0, INTENSITY, 1, 1); // Rouge et bleu 1 seconde chacune
    break;

  case ERR_GPS:                                                          // Erreur d’accès aux données du GPS
    setLedIntermittente(INTENSITY, 0, 0, INTENSITY, INTENSITY, 0, 1, 1); // Rouge et jaune 1 seconde chacune
    break;

  case ERR_CAP:                                                  // Erreur accès aux données d’un capteur
    setLedIntermittente(INTENSITY, 0, 0, 0, INTENSITY, 0, 1, 1); // Rouge et vert 1 seconde chacune
    break;

  case ERR_CAP_INC:                                              // Données reçues d’un capteur incohérentes
    setLedIntermittente(INTENSITY, 0, 0, 0, INTENSITY, 0, 1, 2); // Rouge 1 seconde, vert 2 secondes
    break;

  case ERR_SD_PLEIN:                                                             // Carte SD pleine
    setLedIntermittente(INTENSITY, 0, 0, INTENSITY, INTENSITY, INTENSITY, 1, 1); // Rouge et blanc 1 seconde chacune
    break;

  case ERR_SD:                                                                   // Erreur d’accès ou d’écriture sur la carte SD
    setLedIntermittente(INTENSITY, 0, 0, INTENSITY, INTENSITY, INTENSITY, 1, 2); // Rouge 1 seconde, blanc 2 secondes
    break;

  default:
    leds.setColorRGB(0, 0, 0, 0); // Éteindre les LED en cas de mode non reconnu
    break;
  }
}

void setLedIntermittente(byte r1, byte g1, byte b1, byte r2, byte g2, byte b2, byte duree1, byte duree2)
{
  if (clock.second < time.TLightFix)
  {
    time.TLightFix -= 60;
  }

  if (clock.second >= time.TLightFix + (systemState.ledSwitch ? duree1 : duree2))
  {
    systemState.ledSwitch = !systemState.ledSwitch;
    time.TLightFix = clock.second;
  }

  leds.setColorRGB(0, systemState.ledSwitch ? r1 : r2, systemState.ledSwitch ? g1 : g2, systemState.ledSwitch ? b1 : b2);
}
