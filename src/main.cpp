/*
    Name:       Position-Motor
    Author:     YARIBAR
*/

#include <Arduino.h>
#include "PID_Control.h"
#include "PWM_ESP32.h"
#include <WiFi.h>
#include "H_BRIDGE.h"
#include <PubSubClient.h>
#include "Useful_Methods.h"
#include <BluetoothSerial.h>

#define SAMPLING_PERIOD 500
#define AIN1 12
#define AIN2 14
#define BIN1 18
#define BIN2 19
#define PPR 1632

const uint8_t channelPinADir = 25;
const uint8_t channelPinBDir = 26;
const uint8_t channelPinATrans = 33;
const uint8_t channelPinBTrans = 32;

uint16_t pulsos = 0;
uint32_t rev_actual=0;
uint32_t rev_anterior = 0;
float rev_velocidad = 0;
uint32_t start_time = 0;
float data [3] = {0.0, 0.0, 0.0};
uint32_t start_timePD = 0;
float desiredVelocity = 0;
float voltaje = 0;
float distancia_recorrida = 0;
float a_volante = 0;
String input;

int8_t QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
uint8_t old=0, new_value=0; 
byte out;

const uint16_t maxSteps = 1632;
volatile int16_t ISRCounter = 0;
static volatile byte isr_counter=0;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
int16_t counter = 0;

//*************************
//** MQTT CONFIGURATION ***
//*************************

const char* ssid = ""; //WiFi Network
const char* password = ""; //WiFi's password

const char *mqtt_server = ""; //Domain name or IP
const uint16_t mqtt_port = 1883; //MQTT port TCP
const char *mqtt_user = ""; //MQTT user
const char *mqtt_pass = ""; //MQTT password

//static char *input = char msg[25];

WiFiClient espClient; //Create an object WiFi to connect to internet
PubSubClient client(espClient); //The MQTT protocol will work with the connection done through WiFi

PIDCONTROL PID_DIR(10, 0, 0, 1,8);
PIDCONTROL PID_TRANS(2, 0, 0, 1,8);
HBRIDGE HBRIDGE_DIR(AIN1,AIN2,0, 1,8);
HBRIDGE HBRIDGE_TRANS(BIN1,BIN2,2, 3,8);

BluetoothSerial SerialBT;

float pidDataDir = 0.0 , pidDataTrans = 0.0;
float error_dir= 0, error_trans = 0, elapsedTime = 0;
float desiredAngleS = 0;
float currentPos=0;
float pidConstants[]={0.0,0.0,0.0};
boolean previous=false;

uint32_t lastMsgPID=0; //Variable to save last message
char msg[25]; //Character array to receive message

//*************************
//**FUNCTION DECLARATION***
//*************************

void callback(char* topic, byte* payload, uint length);
void reconnect();
void setup_wifi();

void  ISRencoderDir()
{   
    portENTER_CRITICAL_ISR(&spinlock);
    old = new_value;
    new_value = digitalRead (channelPinADir) * 2 + digitalRead (channelPinBDir); // Convert binary input to decimal value
    out = QEM [old * 4 + new_value];
    ISRCounter = ISRCounter + out;
    if (ISRCounter > maxSteps) ISRCounter=maxSteps;
    if (ISRCounter < -1*maxSteps) ISRCounter=-1*maxSteps;
    portEXIT_CRITICAL_ISR(&spinlock);
}

void ISRencoderTrans(){
  portENTER_CRITICAL_ISR(&spinlock);
  isr_counter++;
  portEXIT_CRITICAL_ISR(&spinlock);
}
 
//************************
//*******  SETUP  ********
//************************

void setup() {
    
    SerialBT.begin("Semanai"); //initialize serial bluetooth
    Serial.begin(115200); //initialize serial port
    setup_wifi();
    pinMode(AIN1, OUTPUT); 
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); 
    pinMode(BIN2, OUTPUT);
    pinMode(channelPinADir, INPUT_PULLUP); 
    pinMode(channelPinBDir, INPUT_PULLUP);  
    pinMode(channelPinATrans, INPUT_PULLUP); 
    pinMode(channelPinBTrans, INPUT_PULLUP); 
    attachInterrupt(digitalPinToInterrupt(channelPinADir), ISRencoderDir, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channelPinBDir), ISRencoderDir, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channelPinATrans), ISRencoderTrans, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channelPinBTrans), ISRencoderTrans, CHANGE);

    client.setServer(mqtt_server, mqtt_port); // MQTT Broker setup
    client.setCallback(callback); //Whenever a message arrives we call this fucntion
}

//*************************
//*******   LOOP   ********
//*************************

void loop() {
    if(!client.connected()){//Check if the board is connected to the server
        reconnect();
    }

    client.loop();

    if(millis() - lastMsgPID > 10){
        lastMsgPID = millis();
        //********************************* PD DIRECCION
        error_dir= desiredAngleS-currentPos;
        pidDataDir = PID_DIR.Data(error_dir,0.01);
        pidDataDir=pidDataDir*100/255;
        //Serial.println(pidData);
        HBRIDGE_DIR.setSpeed(pidDataDir);
        //****************************************** PD TRANSMISION

        error_trans= desiredVelocity - rev_velocidad;
        pidDataTrans = PID_TRANS.Data(error_trans,0.01);
        pidDataTrans=pidDataTrans*100/255;
        //Serial.println(pidData);
        HBRIDGE_TRANS.setSpeed(pidDataTrans);
    }
    
    if (counter != ISRCounter){
        counter = ISRCounter;
        currentPos=mapFloat(counter,0,PPR,0,360);
        //Serial.print("Count: ");
        //Serial.println(counter);
    }

    if(millis() - start_time > SAMPLING_PERIOD){
        start_time = millis();
        rev_velocidad = (rev_actual - rev_anterior)/0.5;
        rev_anterior = rev_actual;
        SerialBT.printf("%.1f,%.1f,%.1f,%.1f\r\n",rev_velocidad,voltaje,distancia_recorrida,a_volante);

        String msg_send = String(currentPos);
        msg_send.toCharArray(msg, 25);

        char topic[25];
        String topic_aux = "dashboard/pos_value";
        topic_aux.toCharArray(topic,25);

        client.publish(topic, msg);
        Serial.println(pidDataDir);
    }
    if(isr_counter){
        pulsos++; 
        portENTER_CRITICAL_ISR(&spinlock);
        isr_counter--;
        portEXIT_CRITICAL_ISR(&spinlock);
    }
    if(pulsos>=PPR){
        rev_actual++;
        pulsos=0;
    } 
    if (SerialBT.available()) { // Lectura de datos con Bluetooth
		input = Serial.readStringUntil('\n');
		parseString(input, ",", data);
        desiredVelocity = data[0];
        desiredAngleS = data[1];
		Serial.printf("%.1f,%.1f \r\n", data[0],data[1]);
	} 
}

//*************************
//*****WIFI CONNECTION*****
//*************************

void setup_wifi(){
    delay(10);

    //if(!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)){
    //    Serial.println("STA Failed to configure");
    //}

    Serial.println();
    Serial.printf("Connecing to %s",ssid);

    WiFi.begin(ssid,password);

    while (WiFi.status() != WL_CONNECTED){ //If there is no connection to the network the procces won't continue
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nConnected to WiFi network!\n");
    Serial.println("IP: ");
    Serial.println(WiFi.localIP());
}

//*************************
//******  LISTENER  *******
//*************************

void callback(char* topic, byte* payload, uint length){
    String incoming = "";
    Serial.print("Message received from -> ");
    Serial.print(topic);
	Serial.println("");

    //As date is send through char array this for concatenate the information in the String "incoming"
    for (int i = 0; i < length; i++)
    {
        incoming += (char)payload[i];
    }
    incoming.trim(); //Get a version of the String with any leading and trailing whitespace removed
    Serial.println("Mensaje ->" + incoming);

    String str_topic(topic);
    if(str_topic == "dashboard/constants"){
        parseString(incoming, ",", pidConstants);
        PID_DIR.changeConstants(pidConstants[0],pidConstants[1],pidConstants[2]);         
    }
    if(str_topic == "dashboard/pos"){
        desiredAngleS = incoming.toFloat(); 
    }
}

//*************************
//****CONNECTION MQTT******
//*************************

void reconnect(){
    while(!client.connected()){
        Serial.print("Trying connection MQTT...");
        //Create a client ID
        String clientId = "esp32_";
        clientId += String(random(0xffff),HEX);//If there are frequent disconnections this allow the board to connect with a different ID
                                
        if(client.connect(clientId.c_str(),mqtt_user,mqtt_pass)){//connects to MQTT
            Serial.println("Connected!");

            char topic[25];
            String topic_aux = "dashboard/constants";
            topic_aux.toCharArray(topic,25);
            client.subscribe(topic);//subscribe to topic constants

            char topic2[25];
            String topic_aux2 = "dashboard/pos";
            topic_aux2.toCharArray(topic2,25);
            client.subscribe(topic2);//subscribe to topic  pos
            
        }
        else{ //If the conection fails try again
            Serial.print("fail :( with error -> ");
            Serial.print(client.state());
            Serial.println(" Try again in 5 seconds");

            delay(5000);
        }
    }
}