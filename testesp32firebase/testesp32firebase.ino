//#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

//#define FIREBASE_HOST "https://testessp32-default-rtdb.firebaseio.com/"
//#define FIREBASE_AUTH "vKNXUkUDPr8wjK0of6u5CKYz6qLr0dMJJOq0vJzo"

#define FIREBASE_HOST "https://nckhapp-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "OBYdVlbiug1bcKbAYrgXyfq2oPSOsO0xpOualPeU"

#define WIFI_SSID "jupiter"
#define WIFI_PASSWORD "12345678"

int mtime;
String last_c;
#define RX2 16
#define TX2 17
FirebaseData firebaseData;
FirebaseJson json;
void setup() {
// put your setup code here, to run once:
Serial.begin(9600);
Serial2.begin(9600,SERIAL_8N1,RX2,TX2);
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
Serial.print("connecting");
while (WiFi.status() != WL_CONNECTED) {
Serial.print(".");
delay(500);
}
Serial.println();
Serial.print("connected with: ");
Serial.println(WIFI_SSID);
Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
Firebase.reconnectWiFi(true);
mtime = millis();
//Firebase.setString(firebaseData,"/test","0");
}

void loop() {
  //put your main code here, to run repeatedly:
  if(Serial2.available()>0){
    int a=Serial2.readString().toInt();
    Firebase.setInt(firebaseData,"/AngleNow",a);
  }
  if((int)(millis()-mtime)>100){
  //     int a=random(123,190);
   //    Firebase.setInt(firebaseData,"/AngleNow",a);
      Firebase.getString(firebaseData,"/Control");
      String c=firebaseData.stringData();
      if(c!=last_c){
       Serial2.println(c);
        }
     mtime=millis();
    }

  

  

}
