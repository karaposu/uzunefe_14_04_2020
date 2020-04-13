//egim var pwm hesaplanıp motorlara verild.  o momentum ile şaft düzeldi ve dedazone sınırları içerisine girdi. burda birden durmayacak. yavaşlayarak durması gerek.bunu
//bunu nasıl anlıcak.? eger açı 0 ile 5 arasında ise ve ivme - yönünde ve tolerans degerinden büyük ise yavaşlayarak dur. 
// eger açı 0 ile 5 arasında ise ve ivme + yönünde ve tolerans degerinden büyük ise + yönünde hızlanacak. bu hızlanmayı yaparken yavaşça gücü artırk


/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
//if(-3 <error <3)
//{
//  pid_i = pid_i+(ki*error);  
//}


//deadzone u iptal et. 0 a göre hesaplasın.
void pid_control()
{

//angle_error=0-gx;
//angle_difference= angle_error - angle_error_old;

//pid_calculate(float axis_angle  ,float set_point  , float kp,float ki ,float kd ,float dt , float old_axis_error,float current_axis_error)//raw_pid= pid_calculate(gx  ,0  ,  kp, ki , kd , dt , angle_error_old,angle_error);   angle_error_old=angle_error;

 // Input = gx;
// myPID.Compute();  raw_pid=Output;
//bluetooth_getvalue()



angle=gx;
angle=angle+direction_speed_change;
float error = angle*5.9 + error_slow + error_slower;
error_slow += error * .06; // learn better set point short run
  //error_slower += error * .005; // learn better set point long run
  
  error_slow *= 0.8;
   error_slower *= 0.99;
 float Output = error ;

raw_pid=Output*-1;



 
            

  sprint(error,error_slow,error_slower,Output );  
//-------------------------------------------------------------------------------------------REMOVE DEADBAND and CONSTRAİN MAX,MIN PWM VALUES 
//if (raw_pid < deadband_pwm_p) { if (0<raw_pid ) { raw_pid=deadband_pwm_p;} }
//if (raw_pid > deadband_pwm_n) {if (0>raw_pid ) { raw_pid=deadband_pwm_n; } }

pwm_cikis=raw_pid;


if (pwm_cikis>255){ pwm_cikis=255;}
if (pwm_cikis<-255){pwm_cikis=-255;}

      
//-------------------------------------------------------------------------------------------MOTOR BEHAVIOUR CONTROL 
         if(  inside_the_deadzone() ==0){
                      //  digitalWrite(LED_PIN, 0);inside_the_space_beyond_our_dimension=0;wtf=0;
                    
                        if(raw_pid>=0  ){   //gx is negative
                       digitalWrite(STBY, HIGH); 
                       digitalWrite(AIN1, LOW);  digitalWrite(AIN2,HIGH );  analogWrite(PWMA, pwm_cikis);
                       digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, pwm_cikis); }
                       
                         else {
                          pwm_cikis=-1*pwm_cikis;
                          digitalWrite(STBY, HIGH); 
                          digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, pwm_cikis);
                          digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, pwm_cikis); }
    
        
          }
          else{// hareketsiz alanda ise 
            
              //   digitalWrite(LED_PIN, 1);
                 if (pwm_cikis < 0) {pwm_cikis=-1*pwm_cikis;}
                  if(inside_the_space_beyond_our_dimension==0){
                       inside_the_space_beyond_our_dimension=1;
                    
                   pos=6;

                        
                   }
               }

 
        
 // sprint(kp,ki,kd,pwm_cikis );
 // sprint(gx, raw_pid, pos, pwm_cikis,  inside_the_space_beyond_our_dimension,  wtf ,   inside_the_deadzone()    );

      

 }



          int16_t pid_calculate(float axis_angle  ,float set_point  , float kp,float ki ,float kd ,float dt , float old_axis_error,float current_axis_error)
{
float axis_angle_error=0;                     float angle_difference=0;float iTerm=0;  dt=1;
  int16_t pwm_cikis=0;
  
axis_angle_error=set_point-axis_angle;
current_axis_error=axis_angle_error;

//------------------------------------------------------------------------------------------CALCULATE RAW_PID
float pTerm = kp * current_axis_error;
iTerm += ki * current_axis_error * dt;  // iTerm = constrain(iTerm, -250.0f, 250.0f); 
float dTerm = kd  * (current_axis_error - old_axis_error)/dt;


// raw_pid = pTerm + dTerm+ iTerm;
pwm_cikis = pTerm + dTerm+ iTerm;  

  return pwm_cikis;
 
                     
} 


void decrease_speed_from_positive_moment(int spd ){
      for(int i=spd; i>0;i--){
               digitalWrite(STBY, HIGH); 
    
                digitalWrite(AIN1, LOW);  digitalWrite(AIN2,HIGH );  analogWrite(PWMA, i);
                 digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, i); 
                  delay(1);

                  pwm_cikis=i;
                 }
               
      inside_the_space_beyond_our_dimension=1;
                }
      

void increase_speed_from_positive_moment(int spd ){
   for(int i=0; i<spd;i++){
   digitalWrite(STBY, HIGH); 
    digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, i);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, i); 
    delay(1); 
    }
          //   
 inside_the_space_beyond_our_dimension=1;      
}


void decrease_speed_from_negative_moment(int spd ){
   for(int i=spd; i>0;i--){
       digitalWrite(STBY, HIGH); 
       digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, i);
       digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, i); 
       delay(1);  
       
                  pwm_cikis=i;
    }
         
inside_the_space_beyond_our_dimension=1;                
}


void increase_speed_from_negative_moment(int spd ){
   for(int i=0; i<spd;i++){
    digitalWrite(STBY, HIGH); 
  digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, i);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, i); 
             delay(1);    
 inside_the_space_beyond_our_dimension=1;         
}
} 


void speedless_zone(){
   
    digitalWrite(STBY, LOW); 
  digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,HIGH ); 
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
             delay(1);    
 inside_the_space_beyond_our_dimension=1;  

       wtf=1;

} 


 
uint8_t inside_the_deadzone( ){
  uint8_t aaa=1;
     if( ( (aci_eksi_sinir>gx ) && (gx>aci_max_eksi_sinir) ) || ( (aci_arti_sinir<gx ) && (gx<aci_max_arti_sinir ) )  )  {
      aaa=0;
     }
    return aaa;
}



void realise()
{


  /* açının - oldugu yönden freefall yaparken 0
   * açının - oldugu yönden düzeltme  yaparken 1
   * açının + oldugu yönden freefall yaparken 3
   * açının + oldugu yönden düzeltme  yaparken 2
    aeod  iki interrupt arasında ölçülen açıların farkıdır.  eger 
    eger anlık  açı degeri - ise  ve aeod degeri - çıkarsa   
    mesela  ilk deger  -5  ikinci deger -10 ise  -10 - -5 =-5
    uzun efe - x e dogru freefall durumunda demektir
    eger anlık açı degeri + ise  aeod ise - ise mesela
    ilk önce 10 sonra 5 ise 5-10  = -5  
    uzunefe  +y eksenine yaklaşarak  düzeltme yapmaktadır
   */
float tolerance=0.3;
 aeod= gx - aeo;    // 5-10 =-5 çıktı
 aeo=gx;            // şimdiki gx degeri kaydedildi.
pos=5;                //
    if(  inside_the_deadzone() ==1){pos=6;}
     
if(gx<0){if(aeod<0){ aeod=aeod*-1;if(aeod>tolerance){    pos=0;} } 
    else if(aeod>0){if(aeod>tolerance){ pos=1;} }   }
                

 if(gx>0){if(aeod<0){ aeod=aeod*-1;if(aeod>tolerance){    pos=2;} } 
    else if(aeod>0){if(aeod>tolerance){ pos=3;} }   }
                
     




 // Serial.print(pos);   Serial.print(" ");   Serial.print(gx);    Serial.print(" ");   Serial.println(inside_the_space_beyond_our_dimension);  
  
  
}



void sprint(float a,float b ,float c ,float d,float e,float f,float g)
{
    Serial.print(a); Serial.print(" "); Serial.print(b);  Serial.print(" ");  Serial.print(c); Serial.print(" "); 
    Serial.print(d); Serial.print(" "); Serial.print(e); Serial.print(" "); Serial.print(f);Serial.print(" "); Serial.println(g);

  
}


void sprint(float a,float b ,float c ,float d,float e,float f)
{
    Serial.print(a); Serial.print(" "); Serial.print(b);  Serial.print(" ");  Serial.print(c); Serial.print(" "); 
    Serial.print(d); Serial.print(" "); Serial.print(e); Serial.print(" "); Serial.println(f);

  
}


void sprint(float a,float b ,float c ,float d)
{
    Serial.print(a); 
      Serial.print(" "); 
        Serial.print(b); 
        Serial.print(" ");  
        Serial.print(c); 
      Serial.print(" "); 
        Serial.println(d); 

  
}
void sprint(float a,float b ,float c ,float d,float e)
{
    Serial.print(a); 
      Serial.print(" "); 
        Serial.print(b); 
        Serial.print(" ");  
        Serial.print(c); 
      Serial.print(" "); 
        Serial.print(d); 
           Serial.print(" "); 
        Serial.println(e); 

  
}
void sprint(float a,float b ,float c)
{
    Serial.print(a); 
      Serial.print(" "); 
        Serial.print(b); 
        Serial.print(" ");  
        Serial.println(c); 

}
void sprint(float a,float b )
{
    Serial.print(a); 
      Serial.print(" "); 
        Serial.println(b); 
        

}

void sprint_char(char a,char b ,char c)
{
    Serial.print(a); 
      Serial.print(" "); 
        Serial.print(b); 
        Serial.print(" ");  
        Serial.println(c); 

}
