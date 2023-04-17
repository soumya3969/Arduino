#define trigPin 2
#define echoPin 3
#define led1 4
#define led2 5
#define led3 6
#define led4 7
#define buzzer 8
const int RELAY_PIN = A5;

void setup() 
{
    Serial.begin(9600);
    pinMode(trigPin,OUTPUT);
    pinMode(echoPin,INPUT);
    pinMode(led1,OUTPUT);
    pinMode(led2,OUTPUT);
    pinMode(led3,OUTPUT);
    pinMode(led4,OUTPUT);
    pinMode(buzzer,OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
}


void loop() 
{
    long duration,distance;
    digitalWrite(trigPin,LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin,HIGH);
    delayMicroseconds(10);

    digitalWrite(trigPin,LOW);
    
    duration=pulseIn(echoPin,HIGH);
    distance=(duration/2)/29.1;
    
if(distance<5)
  {
     digitalWrite(led1,HIGH);
     digitalWrite(led2,LOW);
     digitalWrite(led3,LOW);
     digitalWrite(led4,LOW);
     if(distance<5){
      digitalWrite(buzzer,HIGH);
      delay(500);
      digitalWrite(buzzer,LOW);
      delay(100);
     }
     digitalWrite(RELAY_PIN, HIGH);
     delay(500);     
  }

else if(distance<10)
  {
     digitalWrite(led1,LOW);
     digitalWrite(led2,HIGH);
     digitalWrite(led3,LOW);
     digitalWrite(led4,LOW);
     digitalWrite(buzzer,LOW);
     delay(500);
  } 
 
else if(distance<15)
  {
     digitalWrite(led1,LOW);
     digitalWrite(led2,LOW);
     digitalWrite(led3,HIGH);
     digitalWrite(led4,LOW);
     digitalWrite(buzzer,LOW);
     delay(500);
  }       
else
  {
     digitalWrite(led1,LOW);
     digitalWrite(led2,LOW);
     digitalWrite(led3,LOW);
     digitalWrite(led4,HIGH);
     digitalWrite(RELAY_PIN, LOW);
     digitalWrite(buzzer,LOW);
  }   
Serial.print(distance);
Serial.print("cm");
delay(200);
}
