int mtime;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mtime=millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if((int)(millis()-mtime)>1000){
    Serial.println(1);
    mtime=millis();
  }

}
