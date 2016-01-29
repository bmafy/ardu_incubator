#include <ArduinoJson.h>
#include <Average.h>
#include <dht11.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <SD.h>

const int xPin = A1;
//#define X_pin 1  // analog pin connected to X output
const int yPin = A2;
//#define Y_pin 2  // analog pin connected to Y output
const int Dht11BodyPin = 2;
//#define DHT11_BODY 2
const int Dht11AmbPin = 3;
//#define DHT11_AMB 3
const int soundPin = A0;
//#define heating 24
const int heatingPin = 24;
//#define humid 26
const int humidPin = 26;

long randNumber; // This variable is used only to test the code without any sensor plugged
const int chipSelect = 4;
File dataLog;

// Update these Value with the ones suitable for you
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // Ethernet MAC address (DO NOT TOUCH! IT IS STANDARDIZED!)
byte raspberry_ip[] = {169, 254, 210, 16}; // Rasp Pi IP address
byte arduino_ip[] = {169, 254, 210, 17}; // Arduino IP address
char raspberry_ip_str[] = "169.254.210.16";
char arduino_ip_str[] = "169.254.210.17";

// Variable related to --> Joystick
const byte size = 11;
Average<int> array_X(size);
Average<int> array_Y(size);
const int sampleWindowAcc = 20;
int startAcc;
int k = 0;
int k1 = 0;
int xDev = 0;
int yDev = 0;

// Variable related to --> Cry_sensor
const int sampleWindowS = 200; // Sample window width in ms (50ms = 20Hz --> 200ms --> 5Hz) for the microphone
unsigned int sample;
unsigned long startS; // Used to determine time starting in millis() for the microphone
unsigned int signalMax = 0; // Used to save the maximum in "sampleWindowS"
unsigned int signalMin = 1024; // Used to save the minimum in "sampleWindowS"
int j = 0;


// Variable related to --> DHT11
const int sampleWindowDHT11_1 = 1000; // Sample window width in mS (50ms = 20Hz --> 200ms --> 5Hz) for DHT11
const int sampleWindowDHT11_2 = 1000;
unsigned long startDHT11_1; // Used to determine time starting in millis() for DHT11
unsigned long startDHT11_2;
dht11 DHT; // Object DHT11

// Variable related to --> JSON
const int len = 500;
char buffer[len];
//StaticJsonBuffer<len> jsonBuffer;
DynamicJsonBuffer jsonBuffer;
JsonObject& sensor = jsonBuffer.createObject();
JsonObject& DHT11_1 = sensor.createNestedObject("DHT11_1");
JsonObject& DHT11_2 = sensor.createNestedObject("DHT11_2");
JsonObject& Acc = sensor.createNestedObject("Acc");
//JsonArray& TimeAcc = Acc.createNestedArray("TimeAcc");
JsonArray& Movement = Acc.createNestedArray("Movement");
JsonObject& Microphone = sensor.createNestedObject("Microphone");
//JsonArray& Time = Microphone.createNestedArray("Time");
JsonArray& dB = Microphone.createNestedArray("dB");
unsigned long startPrint;

//// Variable related to --> Networking
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

String unix_time;
String local_time;

boolean heating_state = LOW;
boolean humid_state = LOW;

void setup() {
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  else {
    Serial.println("Card initialized");
  }
  pinMode(humidPin, OUTPUT);
  pinMode(heatingPin, OUTPUT);
  digitalWrite(heatingPin, heating_state);
  digitalWrite(humidPin, humid_state);
  startAcc = millis();
  startS = millis();
  startDHT11_1 = millis();
  startDHT11_2 = millis();
  startPrint = millis();
  Microphone["ID"] = ("Microphone");
  DHT11_1["ID"] = ("DHT11 Sensor1");
  DHT11_2["ID"] = ("DHT11 Sensor2");
  Acc["ID"] = ("Accelerometer");
  for (int i = 0; i <= 5; i++) {
    //TimeAcc.add(0);
    Movement.add(0);
    //Time.add(0);
    dB.add(0);
  }
  Serial.begin(9600);
  while (!Serial) {
    // wait serial port initialization
  }
  mqttClient.setServer(raspberry_ip,1883);
  mqttClient.setCallback(callback);

  Ethernet.begin(mac, arduino_ip);
  // Allow the hardware to sort itself out
  delay(1500);
}

void loop() {
  if (!mqttClient.connected())
  {
    reconnect();
  }
  mqttClient.loop();

  readSound(Microphone);
  readDHT11(DHT11_1);
  readDHT11(DHT11_2);
  readAcc(Acc);
  digitalWrite(heatingPin, heating_state);
  digitalWrite(humidPin, humid_state);
  if (millis() - startPrint > 1010) {
    sensor.printTo(Serial);
    Serial.println();
    startPrint = millis();
  }
}
/*  ###################################
    #              SENSORS            #
    ###################################
*/

void readAcc(JsonObject & SubSensor) {
  //Read from the accelerometer and do somethins...
  mqttClient.publish("requesting", "unix");
  mqttClient.publish("requesting", "local");
  if (millis() - startAcc > sampleWindowAcc) {
    array_X.push(analogRead(X_pin));
    array_Y.push(analogRead(Y_pin));
    k++;
    startAcc = millis();
  }
  if (k >= 10) {
    k = 0;
    xDev = array_X.stddev();
    yDev = array_Y.stddev();

    SubSensor["Time"] = unix_time.toInt(); //Take hour from RPi
    Movement.set(k1, (xDev + yDev) / 2);
    k1++;
    if (k1 > 5) {
      k1 = 0;
      SubSensor.printTo(buffer, sizeof(buffer));
      mqttClient.publish("accelerometer", buffer); // Message published to the topic
//      Serial.println(buffer);
//      if (dataLog) {
//        String r1 = SubSensor["ID"];
//        dataLog.print("ID: ");
//        dataLog.println(r1);
//        dataLog.print("Value: ");
//        int a = Acc["Movement"][0];
//        Serial.println(a);
//        Movement[0].printTo(buffer,sizeof(buffer))
//        dataLog.print(Acc["Movement"][0]);
//        dataLog.print(", ");
//        dataLog.println(Acc[Movement][1]);
//        dataLog.print(", ");
//        dataLog.println(Movement[2]);
//        dataLog.print(", ");
//        dataLog.println(Movement[3]);
//        dataLog.print(", ");
//        dataLog.println(Movement[4]);
//        dataLog.print("Time: ");
//        dataLog.println(local_time);
//        dataLog.println();
//      }
//      else {
//        Serial.println("error opening datalog.txt");
//      }
    }

  }
}

void readSound(JsonObject & SubSensor) {
  unsigned int peakToPeak = 0;   // peak-to-peak level

  sample = analogRead(soundPin);
  if (sample < 1024)  // toss out spurious readings
    if (sample > signalMax)
      signalMax = sample;  // save just the max levels
    else if (sample < signalMin)
      signalMin = sample;  // save just the min levels

  if (millis() - startS > sampleWindowS) {
    peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
    double volts = (peakToPeak * 3.3);  // convert to volts
    signalMax = 0;
    signalMin = 1024;
    startS = millis();
    //Time.set(j, 999); //prendi l'ora da rasp
    SubSensor["Time"] = unix_time.toInt(); //Take hour from RPi
    dB.set(j, volts);
    j++;
    if (j > 5) {
      j = 0; 
      SubSensor.printTo(buffer, sizeof(buffer));
      mqttClient.publish("temperature", buffer); // Message published to the topic
    }
  }
}


void readDHT11(JsonObject & SubSensor) {
  if ((millis() - startDHT11_1 > sampleWindowDHT11_1) ||  (millis() - startDHT11_2 > sampleWindowDHT11_2)) {
    mqttClient.publish("requesting", "unix");
    mqttClient.publish("requesting", "local");
    //    int temp = random(25, 35); // This variable is used only to test the code without any sensor plugged
    //    int hum = random(10, 100); // This variable is used only to test the code without any sensor plugged
    int PIN;
    if (SubSensor["ID"] == "DHT11 Sensor1") {
      startDHT11_1 = millis();
      PIN = Dht11BodyPin;
    }
    else {
      startDHT11_2 = millis();
      PIN = Dht11AmbPin;
    }
    switch (DHT.read(PIN)) {
      case DHTLIB_OK:
        SubSensor["Status"] = "OK";
        break;
      case DHTLIB_ERROR_CHECKSUM:
        SubSensor["Status"] = "Checksum error";
        break;
      case DHTLIB_ERROR_TIMEOUT:
        SubSensor["Status"] = "Time out error";
        break;
      default:
        SubSensor["Status"] = "Unknown error";
        break;
    }
    int temp = DHT.temperature;
    int hum = DHT.humidity;
    SubSensor["Temp"] = temp;
    SubSensor["Hum"] = hum;
    SubSensor["Time"] = unix_time.toInt(); //Take hour from RPi
    //    const char *DHT11_1 = sensor["DHT11_1"].asString();
    //SubSensor.prettyPrintTo(Serial);
    if (dataLog) {
      String r1 = SubSensor["ID"];
      //String r4 = SubSensor["Time"];
      dataLog.print("ID: ");
      dataLog.println(r1);
      dataLog.print("Temperature: ");
      dataLog.println(temp);
      dataLog.print("Humidity: ");
      dataLog.println(hum);
      dataLog.print("Time: ");
      dataLog.println(local_time);
      dataLog.println();
    }
    else {
      Serial.println("error opening datalog.txt");
    }
    SubSensor.printTo(buffer, sizeof(buffer));
    mqttClient.publish("temperature", buffer); // Message published to the topic
  }
}

/*  ###################################
    #              ETHERNET           #
    ###################################
*/


// This functions is called when a message arrived (it acts as an "Interrupt" if you get me ;)  )
void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived on [");
  Serial.print(topic);
  Serial.print("]: ");
  if (dataLog) {
    dataLog.print("Message arrived on [");
    dataLog.print(topic);
    dataLog.print("]: ");
  }
  else {
    Serial.println("error opening datalog.txt");
  }
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]); // payload is the message coming from the broker
    dataLog.print((char)payload[i]);
  }
  dataLog.println();
  dataLog.println();
  dataLog.flush();
  Serial.println();
  if (strcmp (topic, "time_unix") == 0) {
    unix_time = ((String)(char *)payload).substring(0, (length - 3));
  }
  else if ((strcmp(topic, "moisturizing") == 0) || (strcmp(topic, "heating") == 0)) {
    String action = (char *)payload;
    action = action.substring(0, length);
    acting(topic, action);
  }
  else if (strcmp (topic, "time_local") == 0) {
    local_time = ((String)(char *)payload);
  }
  else {
    Serial.print("Topic ("); Serial.print(topic); Serial.println(") doesn't recognize!");
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(raspberry_ip_str))
    {
      Serial.println("Connected!");
      // Once connected, publish an announcement...
      //mqttClient.publish("outTopic","How r u, RPi?"); // Message published to the topic
      // ... and resubscribe
      //mqttClient.subscribe("time_unix");
      //Serial.println("Message published and inTopic subscribed in reconnect()...");
      mqttClient.subscribe("time_unix");
      mqttClient.subscribe("time_local");
      mqttClient.subscribe("heating");
      mqttClient.subscribe("moisturizing");
      dataLog = SD.open("datalog.txt", FILE_WRITE);
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.println(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      dataLog.close();
      delay(5000);
    }
  }
}


/*  ###################################
    #              ACTING             #
    ###################################
*/


void acting(char* topic, String action) {
  Serial.println("Entry in acting funcionalities...");
  if (strcmp(topic, "heating") == 0) {
    if (action.equals("ON")) {
      Serial.println("Heating ON");
      heating_state = HIGH;
      if (dataLog) {
        dataLog.println("Heating ON");
        dataLog.println();
        dataLog.flush();
      }
      else {
        Serial.println("error opening datalog.txt");
      }
      //HEATER ON
    }
    else {
      Serial.println("Heating OFF");
      heating_state = LOW;
      if (dataLog) {
        dataLog.println("Heating OFF");
        dataLog.println();
        dataLog.flush();
      }
      else {
        Serial.println("error opening datalog.txt");
      }
      //HEATER OFF
    }
  }
  if (strcmp(topic, "moisturizing") == 0) {
    if (action == "ON") {
      humid_state = HIGH;
      Serial.println("Moisturizing ON");
      if (dataLog) {
        dataLog.println("Moisturizing ON");
        dataLog.println();
        dataLog.flush();
      }
      else {
        Serial.println("error opening datalog.txt");
      }
      //HUMIDIFIER' ON
    }
    else {
      humid_state = LOW;
      Serial.println("Moisturizing OFF");
      if (dataLog) {
        dataLog.println("Moisturizing OFF");
        dataLog.println();
        dataLog.flush();
      }
      else {
        Serial.println("error opening datalog.txt");
      }
      //HUMIDIFIER OFF
    }
  }
}


