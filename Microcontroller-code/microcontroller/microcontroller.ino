// includes

#include "config.h"
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
    char ssid[] = WIFI_SSID;
    char password[] = WIFI_PASSWD;
  
  // Thingspeak 
          const char*   userApiKey            = USER_API_KEY;
      // Archive
          unsigned long archiveWriteChannelID = ARCHIVE_CHANNEL_ID;
          const char*   archiveWriteAPIKey    = ARCHIVE_WRITE_API_KEY;
          const char*   archiveReadAPIKey     = ARCHIVE_READ_API_KEY;
      // Live
          unsigned long liveWriteChannelID    = LIVE_CHANNEL_ID;
          const char*   liveWriteAPIKey       = LIVE_WRITE_API_KEY;
          const char*   liveReadAPIKey        = LIVE_READ_API_KEY;

// Clients

  HTTPClient http_Client;
  WiFiClient wifi_Client;

// Variables

    int Experiment_ID = 0;

  // PID constants (can be taken as input also)
    float k_p = K_P;
    float k_d = K_D;
    float k_i = K_I;

  // Strings for json formatting
    String archive_Json = "";
    String live_Json = "";
    String Temp = "";  
    String jsonBody = "";   //JSON body
    String onem2mBuffer = "";
    String onem2mrequest = "";

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

  // StatusCode
    int statusCode     = 0;
    int postStatusCode = 0;
    int onem2mcode     = 0;

void setup() {

    live_Json  = "\"write_api_key\": \"" ;
    live_Json +=  LIVE_WRITE_API_KEY     ;
    live_Json += "\",\n\"updates\": [\n" ;

    archive_Json  = "\"write_api_key\": \"";
    archive_Json += ARCHIVE_WRITE_API_KEY  ;
    archive_Json += "\",\n\"updates\": [\n";
    
  // put your setup code here, to run once:
    Serial.begin(9600);

    pinMode(ENCA,INPUT);
    pinMode(ENCB,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

    analogWriteResolution(PWM, 8);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);

    WiFi.begin(WIFI_SSID, WIFI_PASSWD);
    Serial.println("Connecting");
    while(WiFi.status() != WL_CONNECTED){
      delay(500);
      Serial.print(".");
    }
    
    Serial.print("\nConnected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());

    ThingSpeak.begin(wifi_Client);

}


void loop() {
  // put your main code here, to run repeatedly:
  //  input_target = 45.00;
  
  
  
  Experiment_ID++;
  Serial.println("Experiment ID: " + String(Experiment_ID));

  statusCode = 0;

  while(statusCode!=200){
    statusCode = ThingSpeak.readMultipleFields(LIVE_CHANNEL_ID);
    if(statusCode != 200)
        delay(15000);
  }

  input_target = ThingSpeak.getFieldAsFloat(1);
  k_p          = ThingSpeak.getFieldAsFloat(2);
  k_d          = ThingSpeak.getFieldAsFloat(3);
  k_i          = ThingSpeak.getFieldAsFloat(4);

  Serial.print(input_target);
  Serial.print(" ");
  Serial.print(k_p);
  Serial.print(" ");
  Serial.print(k_d);
  Serial.print(" ");
  Serial.print(k_i);
  Serial.print(" ");
  Serial.println();
  
  statusCode = 0;

  onem2mBuffer = "["+String(Experiment_ID)+","+String(input_target)+","+String(k_p)+","+String(k_d)+","+String(k_i);
  jsonBody = "";

  if(input_target != previous_target){
    PID_control(input_target,k_p,k_d,k_i,Experiment_ID);
    previous_target = input_target;
    setMotor(0,0,PWM,IN1,IN2); 
  }

  delay(240000);
  PID_reset(0,K_P,K_D,K_I);
  delay(120000);

}

void PID_reset(float target,float kp,float kd,float ki) 
{
    startTime = millis();
    lastTime = millis();
    while(millis()- startTime < PID_TIMER){
        currT = micros();
        deltaT = ((float) (currT - prevT))/( 1.0e6 );
        prevT = currT;

        //Read the position
        pos = 0; 
        
        noInterrupts(); // disable interrupts temporarily while reading
        pos = (float)posi*0.85714285714//* 0.64056939501  ;
        interrupts(); // turn interrupts back on
        
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

        if(fabs(target-pos)<0.05){
          break;
        }
    }
    setMotor(0,0,PWM,IN1,IN2);
}

void PID_control(float target,float kp,float kd,float ki,int ID) 
{
    
    startTime = millis();
    lastTime = millis();
    lastpubTime = millis();
    use_integral = false;
    md = 0;
    
    onem2mBuffer = onem2mBuffer +"(";
    while(millis()- startTime < PID_TIMER){

      if(millis() - lastTime > SAMPLING_RATE){
        // time difference
        currT = micros();
        deltaT = ((float) (currT - prevT))/( 1.0e6 );
        prevT = currT;

        //Read the position
        pos = 0; 
        
        noInterrupts(); // disable interrupts temporarily while reading
        pos = (float)posi*0.85714285714//* 0.64056939501  ;
        interrupts(); // turn interrupts back on
        
        // pos = pos + 5;  
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

        lastTime = millis();
        delta_pt = int((lastTime - lastpubTime)/float(100));

        onem2mBuffer = onem2mBuffer + "{";
        onem2mBuffer += String(deltaT) ;
        onem2mBuffer += ",";
        onem2mBuffer += String(u)      ;
        onem2mBuffer += ",";
        onem2mBuffer += String(pos)    ; 
        onem2mBuffer += ",";
        onem2mBuffer += String(pwr)    ;
        onem2mBuffer += "},";

        Temp="{\"delta_t\":\""+String(delta_pt)+"\",";
        jsonBody += Temp;
        
        Temp="\"field5\":\""+String(u)+"\",";
        jsonBody += Temp;
        
        Temp="\"field6\":\""+String(pos)+"\",";
        jsonBody += Temp;

        Temp="\"field7\":\""+String(pwr)+"\"";
        jsonBody += Temp;
        
        Temp="\"field8\":\""+String(ID)+"\"";
        jsonBody += Temp;

        Temp="},\n";
        jsonBody += Temp;
        
        if(fabs(target-pos)<0.5){
          break;
        }
    }
      
  }
   setMotor(0,0,PWM,IN1,IN2);
   // Removing the final ','
   jsonBody[jsonBody.length()-2] = '\n';
   jsonBody[jsonBody.length()-1] = ']';
   onem2mBuffer[onem2mBuffer.length()-1] = ')';
   
   Temp="}";
   jsonBody += Temp;
   onem2mBuffer += "]";

   // Doing a bulk update
   thingspeakPOST(live_Json +jsonBody, String(LIVE_CHANNEL_ID));

   delay(15500);
   
  
   thingspeakPOST(archive_Json + jsonBody, String(ARCHIVE_CHANNEL_ID));
   
   onem2mPOST(onem2mBuffer);

   jsonBody = "";
   Temp = "";
   onem2mBuffer = "";
}

void thingspeakPOST(String rep,String channel_id) {
  http_Client.begin("https://api.thingspeak.com/channels/"+channel_id+"/bulk_update.json");

  http_Client.addHeader("Content-Type", "application/json");
  
  Serial.println(rep);

  postStatusCode = 0;
  while(postStatusCode != 202){
    postStatusCode = http_Client.POST(rep);

    if(postStatusCode != 202)
      delay(15500);
  }

  http_Client.end();
}

void onem2mPOST(String rep) {
  
    http_Client.begin(Post_URL);
  
    http_Client.addHeader("X-M2M-Origin", USER_PASS);
    http_Client.addHeader("Content-Type", "application/json;ty=4");

    onem2mrequest= String() + "{\"m2m:cin\": {"

      +
      "\"con\": \"" + rep + "\","

      +
      "\"lbl\": \"" + "V1.0.0" + "\","

      //+ "\"rn\": \"" + "cin_"+String(i++) + "\","

      +
      "\"cnf\": \"text\""

      +
      "}}";
    onem2mcode = 0;
    while(onem2mcode != 202)
    {
      http_Client.POST(onem2mrequest);
      if(onem2mcode  != 202){
        delay(30000);
      }
    }
    http_Client.end();
    Serial.println(onem2mcode);
    onem2mcode = 0;
    onem2mrequest = "";
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
  
  statusCode = 0;
  while(statusCode!=200){
    input_target =  ThingSpeak.readFloatField(writeChannelID,1,readAPIKey);
    statusCode = ThingSpeak.getLastReadStatus();
     delay(16000);
  }
  Serial.print(input_target);
  Serial.print(" ");
  
  statusCode = 0;
  while(statusCode!=200){
    k_p =  ThingSpeak.readFloatField(writeChannelID,5,readAPIKey);
    statusCode = ThingSpeak.getLastReadStatus();
    delay(16000);
  }
  Serial.print(k_p);
  Serial.print(" ");
  
  statusCode = 0;
  while(statusCode!=200){
    k_d =  ThingSpeak.readFloatField(writeChannelID,6,readAPIKey);
    statusCode = ThingSpeak.getLastReadStatus();
    delay(15000);
  }
  Serial.print(k_d);
  Serial.print(" ");
   
  statusCode = 0;
  while(statusCode!=200){
    k_i =  ThingSpeak.readFloatField(writeChannelID,7,readAPIKey);
    statusCode = ThingSpeak.getLastReadStatus();
    delay(15000);
  }
*/
