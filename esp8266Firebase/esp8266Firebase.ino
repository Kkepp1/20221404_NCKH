#include <SoftwareSerial.h>

#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <FirebaseESP8266.h>

#define MYPORT_TX D1
#define MYPORT_RX D2

#define WIFI_SSID "jupiter"
#define WIFI_PASSWORD "12345678"
//#define FIREBASE_HOST "v2-a30b4-default-rtdb.firebaseio.com"
//#define FIREBASE_AUTH "s8TTljCp8MepmgbmH55oFV9yT9YmWVfE41nB8IBR"
#define FIREBASE_HOST "nckhapp-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "OBYdVlbiug1bcKbAYrgXyfq2oPSOsO0xpOualPeU"
FirebaseData firebaseData;
FirebaseJson json;

String path = "/";
String a;
int b=1;
SoftwareSerial myPort;
void setup() {
  Serial.begin(9600);
  myPort.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
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
  if(!Firebase.beginStream(firebaseData, path))
  {
  Serial.println("reason: " + firebaseData.errorReason());
  Serial.println();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
//    if(myPort.available()){
//      String c = myPort.readString();
//      Serial.println(c);
//      Firebase.setString(firebaseData, path + "/AngleNow",c);
//      }
    b=-b;
    Firebase.setInt(firebaseData, path + "/AngleNow",b);
    if(Firebase.getString(firebaseData, path + "Control"))   a= firebaseData.stringData();
    Serial.print(a);
    myPort.print(a);
    delay(500);
}
