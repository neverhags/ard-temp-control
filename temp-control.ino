// termometro múltiple fácil de calibrar
float termometro1= 0;// entrada A0

int tiempo=0;
int cnt=0;
float t1=0;

/* time variables */
unsigned long actTime = 0;
unsigned long 

void setup() {
  Serial.begin(9600); 
  analogReference(INTERNAL);// pone como referencia iterna 1.1V
  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A1, INPUT);
  digitalWrite(A0, HIGH);
  digitalWrite(A2, LOW);
}
void loop() {
  
   t1 = map(analogRead(1),472,638,1000,0); // a 0°C la lectrura de analgRead es 140 y a 100°C es 99
   
   termometro1= t1/10;
   
   tiempo =millis()/1000;
   // para ver en el monitor
Serial.print (tiempo);
Serial.print (" Term 1= ");   
Serial.println (termometro1,1);

 delay(1000); 
}
