unsigned timeStamp;
char buf[96];

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.read() != -1) {
    int val = analogRead(A0);
    sprintf(buf, "{\n\t\"timeStamp\": %u,\n\t\"type\": \"olfactory\",\n\t\"value\": %d\n}", timeStamp++, val);
    Serial.println(buf);
  }
}
