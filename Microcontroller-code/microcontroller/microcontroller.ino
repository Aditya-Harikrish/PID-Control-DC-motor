// includes

#include "mbedtls/base64.h"
#include "mbedtls/md.h"
#include "analogWrite.h"
#include <time.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <base64.h>
#include <PubSubClient.h>
#include <ThingSpeak.h>
#include <WebServer.h>

// Details

  // Pin Details
  
    int IN1 = 27;     // Input Pin 1 [motor driver]
    int IN2 = 26;     // Input Pin   [motor driver]
    int PWM = 14;     // PWM Pin     [motor driver]
    int ENCA = 32;    // Encoder A   [DC motor]
    int ENCB = 34;    // Encoder B   [DC motor]
  
  // Wifi
    char ssid[] = "hackiiit";
    char password[] = "12344321";
  
  // Thingspeak 
    unsigned long writeChannelID = 1825221;
    char writeAPIKey[] = "XU89SKWS3ZIIHP52";
    const char* readAPIKey = "U29GEZH7BZ4ECF53";

// Clients

  HTTPClient http;
  WiFiClient client1;

// Variables

  // PID constants (can be taken as input also)
    float k_p = 10;
    float k_d = 0.025;
    float k_i = 5;

  // Strings for json formatting
    String topic_1 = "{\n\"write_api_key\": \"XU89SKWS3ZIIHP52\",\n\"updates\": [\n";
    String Temp = "";  
    String jsonBuffer = ""; //JSON Body

  // Time
    #define PID_TIMER 10000
    
    uint wifi_delay = 5000;
    uint startTime;
    uint lastTime;
    uint lastpubTime;
    
    int delta_pt =0 ;
    int SAMPLING_RATE = 100;
    long prevT = 0;
    long currT;
    float deltaT;
    
  // PID variables
    int dir;
    int md = 0;
    bool use_integral = false;
    float eprev = 0;
    float eintegral = 0;
    float e;
    float dedt;
    float u;
    float pwr;
    volatile int posi = 0;
 
  // Angles
    float input_target;
    float previous_target=0.0;
    float pos = 0;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

    pinMode(ENCA,INPUT);
    pinMode(ENCB,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

    analogWriteResolution(PWM, 8);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);

    WiFi.begin("hackiiit", "12344321");
    Serial.println("Connecting");
    while(WiFi.status() != WL_CONNECTED)
        delay(500);
    
    Serial.print("\nConnected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());

//  ThingSpeak.begin(client1);

}


void loop() {
  // put your main code here, to run repeatedly:
  input_target = 45.00;
  if(input_target != previous_target){
    PID_control(input_target,k_p,k_i,k_d);
    previous_target = input_target;
    setMotor(0,0,PWM,IN1,IN2); 
  }

}

//void mqttconnectloop()
//{
//  while(!mqttClient.connected())
//  {
//    
//    if (mqttClient.connect( clientID, mqttUserName, mqttPass )) {
//      Serial.println("Connected to MQTT broker");
//      break;
//    }
//    else {
//      Serial.println("MQTT connect failed");
//      Serial.println("Will reset and try again...");
//      delay(500);
//    }
//  }
//}

void PID_control(float target,float kp,float ki,float kd) 
{
    
    startTime = millis();
    lastTime = millis();
    lastpubTime = millis();
    use_integral = false;
    md = 0;
    
    jsonBuffer = topic_1;
    while(millis()- startTime < PID_TIMER){

      if(millis() - lastTime > SAMPLING_RATE){
//        setMotor(0,0,PWM,IN1,IN2);
        // time difference
        currT = micros();
        deltaT = ((float) (currT - prevT))/( 1.0e6 );
        prevT = currT;

        //Read the position
        pos = 0; 
        
        noInterrupts(); // disable interrupts temporarily while reading
        pos = (float)posi*0.85714285714//* 0.64056939501  ;
        interrupts(); // turn interrupts back on
        
//        pos = pos + 5;
        // error
        e = target - pos;

        // derivative
        dedt = (e-eprev)/(deltaT);

        // integral
        if(dedt == 0)
          use_integral = true;
        if (use_integral == true)
        {
            if (pos > target)
            {
              if (md == -1)
                eintegral = 0;
              md = 1;
            }
            else if (pos < target)
            {
              if (md == 1)
                eintegral = 0;
              md = -1;
            }
           
        
          eintegral = eintegral + e*deltaT;
        }

        // control signal
        u = kp*e + kd*dedt + ki*eintegral;

        // motor power
        pwr = fabs(u);
        if( pwr > 255 ){
            pwr = 255;
        }

        // motor direction
        dir = 1;
        if(u<0){
            dir = -1;
        }

        // signal the motor
        setMotor(dir,pwr,PWM,IN1,IN2);


        // store previous error
        eprev = e;

        Serial.print(pwr);
        Serial.print(" ");
        Serial.print(u);
        Serial.print(" ");
        Serial.print(target);
        Serial.print(" ");
        Serial.print(pos);
        Serial.println();
        
        
        lastTime = millis();
        delta_pt = int((lastTime - lastpubTime)/float(100));
        
        Temp="{\"delta_t\":\""+String(delta_pt)+"\",";
        jsonBuffer += Temp;
        
        Temp="\"field1\":\""+String(target)+"\",";
        jsonBuffer += Temp;

        Temp="\"field2\":\""+String(pos)+"\",";
        jsonBuffer += Temp;
        
        Temp="\"field3\":\""+String(u)+"\"";
        jsonBuffer += Temp;
        
        Temp="},\n";
        jsonBuffer += Temp;
        
        if(fabs(target-pos)<0.5){
          break;
        }
        
      }
      
   }
   setMotor(0,0,PWM,IN1,IN2);
   // Removing the final ','
   jsonBuffer[jsonBuffer.length()-2] = '\n';
   jsonBuffer[jsonBuffer.length()-1] = ']';
   
   Temp="}";
   jsonBuffer += Temp;

   // Doing a bulk update
   thingspeakPOST(jsonBuffer);
   
}

void thingspeakPOST(String rep) {
  http.begin("https://api.thingspeak.com/channels/1825221/bulk_update.json");

  http.addHeader("Content-Type", "application/json");
  
  Serial.println(rep);

  int code = http.POST(rep);
  if(code == 202)
  {
    Serial.println(code);
    Serial.println("Successfully added the reading");
  }
  else
  {
    Serial.println(code);
    Serial.println("Failed to add the reading");
  }
  http.end();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
    if(dir == 1){
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
    }
    else if(dir == -1){
        digitalWrite(in1,LOW);
        digitalWrite(in2,HIGH);
    }
    else{
        digitalWrite(in1,LOW);
        digitalWrite(in2,LOW);
    }  
    analogWrite(pwm,pwmVal);
}

void readEncoder(){
    int b = digitalRead(ENCB);
    if(b > 0){
        posi++;
    }
    else{
        posi--;
    }
}



/*
 * const char* server ="mqtt3.thingspeak.com"; 
 * const char mqttUserName[] = "MyAGLRksBQQGFhoLEC0NAx8"; 
 * const char mqttPass[] = "YCf3gwLtZG8kYglK/5/gTvGE";
 * const char clientID[] = "MyAGLRksBQQGFhoLEC0NAx8";
 * String topicString = "channels/" + String(writeChannelID) + "/publish";
 *   PubSubClient mqttClient(server, 1883, client1);
 * String f1 = "field1=";
 * String f2 = "&field2=";
 * String f3 = "&field3=";
 * String f4 = "&status=MQTTPUBLISH";
 * //        final_1 = f1 + String(target) + f2 + String(pos) + f3 +String(u) + f4;
//        if(lastTime-lastpubTime > 1500){
//             if(!mqttClient.connected()) {mqttconnectloop(); }
//             mqttClient.publish(topicString.c_str(), final_1.c_str());
//             lastpubTime = lastTime;
//             Serial.print(topicString.c_str());
//             Serial.print(" ");
//             Serial.print(final_1.c_str());
//             Serial.print(" ");
//             Serial.println();  
//        }
String final_1 = "";
//    mqttClient.setServer(server,1883);
//    mqttconnectloop();
    
//    delay(1000); 
//    pinMode(PWM,OUTPUT);

//char jsonBuffer[6500] = "";
 
 // String req_data = "{\"m2m:cin\": {\"con\": \"" + data + "\",\"cnf\": \"" + description + "\"}}";
  // Serial.println(req_data);
  */ 
