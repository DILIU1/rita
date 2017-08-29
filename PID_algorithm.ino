float SP=0.0, SP_pulse, SP_regime=0.0, pmax=100.0, pmin=-100.0, VSmax=2.5, VSmin=0.5, VinRange=3.3, VVmin=0.5, VVmax=1.0, VoutRange=2.8, PV, m, n, constrMin, constrMax, sigP=0.0, sigN=0.0, e0=0, e1=0, e2=0;
float kpp=0.05, kpi=1, kpd=1E-6, knp=1, kni=10, knd=1E-6;
double dt=1.0;
unsigned long PL=0, prev, curr;
unsigned int pos=DAC0, neg=DAC1;
int rem=0, bit_res=12, quant_levels=pow(2,bit_res);
String out, msg="";
bool active=true, pulse_regime=false;

void setup() {
  Serial.begin(9600);
  analogReadResolution(bit_res);
  analogWriteResolution(bit_res);
  m=(VinRange/pow(2,bit_res))*((pmax-pmin)/(VSmax-VSmin));
  n=-VSmin*((pmax-pmin)/(VSmax-VSmin))+pmin;
  constrMin=0;
  constrMax=1250;
}

void loop() {
  out="";
  prev=curr; 
  curr=millis();
  PV=analogRead(A0)*m+n;
  dt=0.001*constrain((curr-prev), 1, 100);
  e0=SP-PV;
  e1=e0;
  e2=e1;

  if (rem>0){
      SP=SP_pulse;
      if(!pulse_regime && (SP_regime-PV)/(SP_regime-SP_pulse)>0.5)
          pulse_regime=true;
      if(pulse_regime)
          rem-=(curr-prev);
  }
  else{
      SP=SP_regime;
      pulse_regime=false;
  }
  
  if(active){
    if(SP<0.0){
      sigN=constrain(sigN-((knp+dt*kni+knd/dt)*e0+(-knp-2*knd/dt)*e1+(knd/dt)*e2), constrMin, constrMax);
      sigP=constrMin;
      digitalWrite(2, HIGH);
    } 
    else {
      sigP=constrain(sigP+((kpp+dt*kpi+kpd/dt)*e0+(-kpp-2*kpd/dt)*e1+(kpd/dt)*e2), constrMin, constrMax);
      sigN=constrMin;
      digitalWrite(2, LOW);
    }
  }  
  else{
    sigP=0;
    sigN=0;
    digitalWrite(2, LOW);
  }
 
  analogWrite(pos, sigP);
  analogWrite(neg, sigN);

//Serial.println(sigP);
//  Serial.print(sigN);
//  Serial.print("\t");
//  Serial.println(sigP);
//  Serial.print("\t");
//  Serial.println(sigN);
}
    
void serialEvent() {

  char buf[64], *tmp=NULL, c;
  
    while (Serial.available()){
      
      msg += (c=(char)Serial.read());
      if(c=='\n'){
        msg.toCharArray(buf, 64);
        String cmd=(strtok_r(buf, " ,", &tmp));
        cmd.trim();

        if(cmd==String("SP")){
          SP_regime=atof(strtok_r(NULL, " ,", &tmp));
          Serial.println("SP = "+String(SP_regime));
        }
        else if (cmd=="PULSE"){      
          SP_pulse=atof(strtok_r(NULL, " ,", &tmp));    
          PL=atoi(strtok_r(NULL, " ,", &tmp));
          rem=PL;
          Serial.println("Pulse = "+String(PL)+" at "+SP_pulse);
        }
        else if (cmd=="ON" || cmd=="OFF"){
          (active=(cmd=="ON")) ? Serial.println("ACTIVE") : Serial.println("OFF");
          SP=0;
          rem=0;
          pulse_regime=false;
        }   
        else if (cmd=="PGAINS"){
          kpp=atof(strtok_r(NULL, " ,", &tmp));
          kpi=atof(strtok_r(NULL, " ,", &tmp));
          kpd=atof(strtok_r(NULL, " ,", &tmp));
          Serial.println("Gains(+) = "+String(kpp)+", "+kpi+", "+kpd);
        }
        else if (cmd=="NGAINS"){
          knp=atof(strtok_r(NULL, " ,", &tmp));
          kni=atof(strtok_r(NULL, " ,", &tmp));
          knd=atof(strtok_r(NULL, " ,", &tmp));
          Serial.println("Gains(-) = "+String(knp)+", "+kni+", "+knd);
        }
        else{
          Serial.println("ERR");
        }
        
        msg="";
        return;
      }
    }
    
}





