void setup() {
  // put your setup code here, to run once:
    pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
      digitalWrite(LED_BUILTIN,HIGH);

    digitalWrite(D1, HIGH);

    digitalWrite(D2, HIGH);

    digitalWrite(D0, LOW);

    digitalWrite(D3, LOW);

    delay(100);

    digitalWrite(LED_BUILTIN,LOW);

    delay(100);

}
