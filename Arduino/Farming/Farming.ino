#include<DHT.h>
#include<SoftwareSerial.h>
int LDRpin = A0;
int DHTpin = 2;
int Moistpin = A1;
DHT dht;
SoftwareSerial BTSerial(10, 11);
void setup()
{
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  Serial.begin(9600);
  BTSerial.begin(9600);
  dht.setup(DHTpin);
}
void loop()
{
  printHumidity();
  BTSerial.print(", ");
  printTemperature();
  BTSerial.println(", ");
  printIntensity();
  BTSerial.print(", ");
  printMoisture();
  BTSerial.println();
  delay(5000);
}
void printHumidity()
{
  BTSerial.print("Humidity: ");
  BTSerial.print(dht.getHumidity());
  BTSerial.print("%");
}
void printTemperature()
{
  BTSerial.print("Temperature: ");
  BTSerial.print(dht.getTemperature());
  BTSerial.print(" C");
}
void printIntensity()
{
  float LDRvalue=analogRead(LDRpin);
  float vout=LDRvalue*5/1024;
  float lux=50*(5-vout)/vout;
  BTSerial.print("Light Intensity: ");
  BTSerial.print(lux);
  BTSerial.print(" lux");
}
void printMoisture()
{
  float moisture = analogRead(Moistpin);
  moisture=map(moisture,1023,483,0,100);
  BTSerial.print("Moisture: ");
  BTSerial.print(moisture);
  BTSerial.print("%");
}

void DHT::setup(uint8_t pin, DHT_MODEL_t model)
{
  DHT::pin = pin;
  DHT::model = model;
  DHT::resetTimer();

  if ( model == AUTO_DETECT) {
    DHT::model = DHT22;
    readSensor();
    if ( error == ERROR_TIMEOUT ) {
      DHT::model = DHT11;
    }
  }
}

void DHT::resetTimer()
{
  DHT::lastReadTime = millis() - 3000;
}

float DHT::getHumidity()
{
  readSensor();
  return humidity;
}

float DHT::getTemperature()
{
  readSensor();
  return temperature;
}

#ifndef OPTIMIZE_SRAM_SIZE

const char* DHT::getStatusString()
{
  switch ( error ) {
    case DHT::ERROR_TIMEOUT:
      return "TIMEOUT";

    case DHT::ERROR_CHECKSUM:
      return "CHECKSUM";

    default:
      return "OK";
  }
}

#else

prog_char P_OK[]       PROGMEM = "OK";
prog_char P_TIMEOUT[]  PROGMEM = "TIMEOUT";
prog_char P_CHECKSUM[] PROGMEM = "CHECKSUM";

const char *DHT::getStatusString() {
  prog_char *c;
  switch ( error ) {
    case DHT::ERROR_CHECKSUM:
      c = P_CHECKSUM; break;

    case DHT::ERROR_TIMEOUT:
      c = P_TIMEOUT; break;

    default:
      c = P_OK; break;
  }

  static char buffer[9];
  strcpy_P(buffer, c);

  return buffer;
}

#endif

void DHT::readSensor()
{
  unsigned long startTime = millis();
  if ( (unsigned long)(startTime - lastReadTime) < (model == DHT11 ? 999L : 1999L) ) {
    return;
  }
  lastReadTime = startTime;

  temperature = NAN;
  humidity = NAN;


  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
  if ( model == DHT11 ) {
    delay(18);
  }
  else {
    delayMicroseconds(800);
  }

  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);

  word rawHumidity;
  word rawTemperature;
  word data;

  for ( int8_t i = -3 ; i < 2 * 40; i++ ) {
    byte age;
    startTime = micros();

    do {
      age = (unsigned long)(micros() - startTime);
      if ( age > 90 ) {
        error = ERROR_TIMEOUT;
        return;
      }
    }
    while ( digitalRead(pin) == (i & 1) ? HIGH : LOW );

    if ( i >= 0 && (i & 1) ) {
     
      data <<= 1;

     
      if ( age > 30 ) {
        data |= 1;
      }
    }

    switch ( i ) {
      case 31:
        rawHumidity = data;
        break;
      case 63:
        rawTemperature = data;
        data = 0;
        break;
    }
  }

  if ( (byte)(((byte)rawHumidity) + (rawHumidity >> 8) + ((byte)rawTemperature) + (rawTemperature >> 8)) != data ) {
    error = ERROR_CHECKSUM;
    return;
  }


  if ( model == DHT11 ) {
    humidity = rawHumidity >> 8;
    temperature = rawTemperature >> 8;
  }
  else {
    humidity = rawHumidity * 0.1;

    if ( rawTemperature & 0x8000 ) {
      rawTemperature = -(int16_t)(rawTemperature & 0x7FFF);
    }
    temperature = ((int16_t)rawTemperature) * 0.1;
  }

  error = ERROR_NONE;
}


