int pin=9;
float alpha=0.1;
float smooth=0;

class PIDController{
  public:
    float Kp,Ki,Kd;
    float setpoint;       
    float integral;        
    float Error;         
    unsigned long Time; 

    PIDController(float Kp,float Ki,float Kd){
      this->Kp=Kp;
      this->Ki=Ki;
      this->Kd=Kd;
      this->integral=0;
      this->Error=0;
      this->Time=millis();
    }

    float compute(float value){
      unsigned long time=millis();
      float timeChange=(float)(time-Time);      
      float error=setpoint-value;
      integral+=error*timeChange;
      float derivative=(error-Error)/timeChange;
      float output=Kp*error+Ki*integral+Kd*derivative;
      Error=error;
      Time=time;
      return output;
    }
};

PIDController pid(2.0,0.5,1.0);

void setup(){
  pinMode(pin,OUTPUT); 
  pid.setpoint=100;        
}

void loop(){
  float speed=analogRead(A0);
  smooth=alpha*speed+(1-alpha)*smooth;
  float signal=pid.compute(smooth);
  analogWrite(pin,constrain(signal,0,255));
}