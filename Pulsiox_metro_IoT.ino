#include <Arduino.h>
#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include "WiFiManager.h"
#include <WebSocketsClient.h>
#include <Hash.h>

#include <Wire.h>                // I2C  Library
#include "MAX30105.h"            // MAX3010x library
#include "heartRate.h"           // Heart rate calculating algorithm

WebSocketsClient webSocket;
const char *websocket_server = "192.168.100.124"; //la direccion del servidor de nodered
const int websocket_port = 1880;
const char *url = "/corazon";

WiFiManager wifiManager;
void configModeCallback (WiFiManager *myWiFiManager)
{
 Serial.print ("Entered config mode");
 Serial.print (WiFi.softAPIP());
// si usas el autogenerar SSID, muestra
  Serial.print (myWiFiManager->getConfigPortalSSID());
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.printf("[WSc] Disconnected!\n");
    break;
  case WStype_CONNECTED:
  {
    Serial.printf("[WSc] Connected to url: %s\n", payload);
    webSocket.sendTXT("Connected");
  }
  break;
  }
}
  
MAX30105 particleSensor;

const byte RATE_SIZE = 6;        // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];           // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;               // Time at which the last beat occurred
unsigned long lastTime = 0;
float beatsPerMinute;
int beatAvg;
unsigned long timerDelay = 3000;  // espero 3 seg para generar lectura

void setup() {

Serial.begin(115200);
  // Initialize sensor
  particleSensor.begin(Wire, I2C_SPEED_FAST);    // Use default I2C port, 400kHz speed
  particleSensor.setup();                        // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);     // Turn Red LED to low to indicate sensor is running
Serial.println("BIENVENIDO AL PULSIOXIMETRO IoT ");
Serial.print("CONECTANDO AL WIFI...");

wifiManager.setAPCallback(configModeCallback);// configur las instancias de wifimanager
 //creo una funcion para reconetar en caso de que no se conecte cuando se conecto el dispositivo
  
if(!wifiManager.autoConnect("ESP8266_CORAZON")) {
  Serial.println("fallo la conexion y se agoto el tiempo de espera");

    // reseteo y vuelvo a intentar conectarme a wifi
   wifiManager.resetSettings();
   }
 
  Serial.println("Connected to Wifi");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

}

void sendToServer(){
  
 long irValue = particleSensor.getIR();              // Reading the IR value it will permit us to know if there's a finger on the sensor or not
 //Serial.println ("IR:  "+ String (irValue));
 
  if (irValue > 50000)  {

    if (checkForBeat(irValue) == true)                  //If a heart beat is detected, call checkForBeat as frequent as possible to get accurate value
    {
      unsigned long currentTime=millis();
      long delta = millis() - lastBeat;                 //Measure duration between two beats
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);           //Calculating the BPM
      if (beatsPerMinute < 255 && beatsPerMinute > 20)  //To calculate the average we strore some values (8) then do some math to calculate the average
      {
        rates[rateSpot++] = (byte)beatsPerMinute;       //Store this reading in the array
        rateSpot %= RATE_SIZE;                          //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
        
       if ((currentTime - lastTime) >= timerDelay) {
       lastTime = currentTime;

      Serial.println(" BPM: "+ String(beatAvg) );
      
      String payload = "{\"bpm\":";
      payload += beatAvg;
      payload += "}";
      webSocket.sendTXT(payload.c_str());
       }
      }
    }
  }
  
  else {
   Serial.println(" BPM: 0  " );
   Serial.println("PONGA EL DEDO EN EL SENSOR Y ESPERE.......");
   String payload = "{\"bpm\":";
      payload += "0";
      payload += "}";
      webSocket.sendTXT(payload.c_str());
     }  

}

void loop() {
  webSocket.loop();
  
  sendToServer();

}
