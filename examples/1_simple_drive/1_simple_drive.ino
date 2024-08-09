#define PIN_UH 6
#define PIN_UL 5
#define PIN_VH 1
#define PIN_VL 3
#define PIN_WH 0
#define PIN_WL 4

bool direction_of_rotation = 0;  // 1：顺时针；0：逆时针

void set_au_bd(void) {
  digitalWrite(PIN_UH, HIGH);
  digitalWrite(PIN_UL, LOW);
  digitalWrite(PIN_VH, LOW);
  digitalWrite(PIN_VL, HIGH);
  digitalWrite(PIN_WH, LOW);
  digitalWrite(PIN_WL, LOW);
}

void set_au_cd(void) {
  digitalWrite(PIN_UH, HIGH);
  digitalWrite(PIN_UL, LOW);
  digitalWrite(PIN_VH, LOW);
  digitalWrite(PIN_VL, LOW);
  digitalWrite(PIN_WH, LOW);
  digitalWrite(PIN_WL, HIGH);
}

void set_bu_cd(void) {
  digitalWrite(PIN_UH, LOW);
  digitalWrite(PIN_UL, LOW);
  digitalWrite(PIN_VH, HIGH);
  digitalWrite(PIN_VL, LOW);
  digitalWrite(PIN_WH, LOW);
  digitalWrite(PIN_WL, HIGH);
}

void set_bu_ad(void) {
  digitalWrite(PIN_UH, LOW);
  digitalWrite(PIN_UL, HIGH);
  digitalWrite(PIN_VH, HIGH);
  digitalWrite(PIN_VL, LOW);
  digitalWrite(PIN_WH, LOW);
  digitalWrite(PIN_WL, LOW);
}

void set_cu_ad(void) {
  digitalWrite(PIN_UH, LOW);
  digitalWrite(PIN_UL, HIGH);
  digitalWrite(PIN_VH, LOW);
  digitalWrite(PIN_VL, LOW);
  digitalWrite(PIN_WH, HIGH);
  digitalWrite(PIN_WL, LOW);
}

void set_cu_bd(void) {
  digitalWrite(PIN_UH, LOW);
  digitalWrite(PIN_UL, LOW);
  digitalWrite(PIN_VH, LOW);
  digitalWrite(PIN_VL, HIGH);
  digitalWrite(PIN_WH, HIGH);
  digitalWrite(PIN_WL, LOW);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);

  pinMode(PIN_UH, OUTPUT);  // AH
  pinMode(PIN_UL, OUTPUT);  // AL
  pinMode(PIN_VH, OUTPUT);  // BH
  pinMode(PIN_VL, OUTPUT);  // BL
  pinMode(PIN_WH, OUTPUT);  // CH
  pinMode(PIN_WL, OUTPUT);  // CL
}

int i = 0;
unsigned long previousMillis = 0;
void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();  

  if (currentMillis - previousMillis >= 5) {

    if(direction_of_rotation){
      switch (i) {
        case 0: set_cu_bd(); i = 1; break;
        case 1: set_cu_ad(); i = 2; break;
        case 2: set_bu_ad(); i = 3; break;
        case 3: set_bu_cd(); i = 4; break;
        case 4: set_au_cd(); i = 5; break;
        case 5: set_au_bd(); i = 0; break;
      }
    }else{
        switch (i) {
        case 0: set_au_bd(); i = 1; break;
        case 1: set_au_cd(); i = 2; break;
        case 2: set_bu_cd(); i = 3; break;
        case 3: set_bu_ad(); i = 4; break;
        case 4: set_cu_ad(); i = 5; break;
        case 5: set_cu_bd(); i = 0; break;
      }
    }
    previousMillis = currentMillis;  //更新时间记录
  }
}
