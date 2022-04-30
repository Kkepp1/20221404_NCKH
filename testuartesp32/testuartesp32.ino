#define RX2 16
#define TX2 17
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
Serial2.begin(9600,SERIAL_8N1,RX2,TX2);
}

void loop() {
  // put your main code here, to run repeatedly:
    if(Serial2.available()>0){
    String a=Serial2.readString();
    //Firebase.setString(firebaseData,"/test",a);
    Serial.println(a);
    Serial2.println(a);
     }
    //Serial2.println(99);
   //delay(1000);

}
