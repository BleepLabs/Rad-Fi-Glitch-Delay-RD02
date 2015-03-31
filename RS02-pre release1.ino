#include <avr/pgmspace.h>

//#define DIGITALIO_NO_MIX_ANALOGWRITE
#include "digitalIOPerformance.h"

#define DLY_BUF_LEN 1030

byte dly_buffer[DLY_BUF_LEN];


unsigned long d,t,prev,prev2,prevserial;
byte tickLED,tick2,pot_tick;
int in0,in1,amin;
int dds_rate=50;
long dds_tune;
byte dly_tick,fast_pot_tick,slow_sample;
uint16_t write_head,read_head,lerp_dly,fm,am,samp_c,freq_in,freq_in_s;
int dly_filter,feedback,fb_tmp,fb_tmp_inv,output,dly_input,dly_time,rn;
int in2;
byte f_mode;
int seq_step;
long seq_rate,seq_step_len;

int seq_stop,p1,dly_buf_in,p2,p2_inv;
int dly_mix,dly_mix_2,fpot1,mode_p,seq_rate_p;
byte mode,seq_on,seq_amp_out;
 int amp_lerp,fb_amt,fb_amt_inv; 
byte crush_amt;
int  dly_led_c;
int blink_inv,osc_rate;
byte tick4,reverse,freeze,noisecomb,lerp_tick;
int previnput[2]={};
int blinkc,blinklatch,blc;


void setup() {

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

 
 sbi (TIMSK1,TOIE1);
 
  sbi (TCCR1B, CS20);
  cbi (TCCR1B, CS21);
  cbi (TCCR1B, CS22);
  
  cbi (TCCR1A, COM2A0);  // clear Compare Match
  sbi (TCCR1A, COM2A1);
  sbi (TCCR1A, COM2B1);
  
  sbi (TCCR1A, WGM20);  // Mode 1  / Phase Correct PWM
  cbi (TCCR1A, WGM21);
  cbi (TCCR1B, WGM22);

  sbi (TCCR2B, CS20);
  cbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);
  
  cbi (TCCR2A, COM2A0);  // clear Compare Match
  sbi (TCCR2A, COM2A1);
  sbi (TCCR2A, COM2B1);
  
  //sbi (TCCR2A, WGM20);  // Mode 1  / Phase Correct PWM
  //cbi (TCCR2A, WGM21);
 // cbi (TCCR2B, WGM22);
  
  cbi(ADCSRA,ADPS2) ;
  sbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;

  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  pinMode(5, INPUT_PULLUP);

  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  
  pinMode(9, OUTPUT);

  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);



  //
  Serial.begin(9600); 

  delay(100);


}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int16_t comb;
ISR(TIMER1_OVF_vect)
{ 
  //digitalWrite(12,1);

  pot_tick++;
  if(pot_tick>3){
    pot_tick=0;
  }

  if(pot_tick==0){
    amin =(analogRead(A5)>>1); 
  }
  if(pot_tick==1){
    in0 =analogRead(A1);

  }

  if(pot_tick==2){
    amin =(analogRead(A5)>>1); 
  }

  if(pot_tick==3){
      in1 =analogRead(A0)>>2;  
    }


  amin-=255; 

  if (amin<-255) {
    amin=-255;
}

if (amin>255) {
  amin=255;
}

byte ttt=feedback_delay(amin>>1);

lerp_tick++;
if (lerp_tick>6){
 byte dly_step=1;

 if(lerp_dly < dly_time)  
 lerp_dly +=dly_step;
 if(lerp_dly > dly_time)
 lerp_dly -= dly_step;

 lerp_tick=0;
}

  OCR1A=(ttt);  //9
  //OCR1B=(comb);       //10


//  digitalWrite(12,0);


}


void loop() {

  if ((millis()-prevserial)>400){
    prevserial=millis();

    Serial.println();       
    Serial.println(lerp_dly);       
    Serial.println(freeRam()); 
    Serial.println(); 

          
  }



  freeze = (digitalRead(13));
  reverse = (digitalRead(12));

dly_time=(readchange(0,in0))+1;    // update the delay time value. readchange info bleow

//  fb_tmp=(in1);    // update the feedback value.
//  fb_tmp_inv=(fb_tmp-255)*-1;  //calculates an inverted value for feedback amount

if (in1<126){

 // fb_mode=1;
  fb_tmp=(in1<<1);    // update the feedback value.
  fb_tmp_inv=(fb_tmp-255)*-1;  //calculates an inverted value for feedback amount

}

if (in1>126 && in1<146){

 // fb_mode=1;
  fb_tmp=(254);    // update the feedback value.
  fb_tmp_inv=1;  //calculates an inverted value for feedback amount

}

if (in1>=146){

   fb_tmp=((in1-128)<<3);    // update the feedback value.
  fb_tmp_inv=(fb_tmp);  //calculates an inverted value for feedback amount

}


blink_inv=((dly_time-DLY_BUF_LEN)*-1);
blinkc++;
if (blinkc>blink_inv){
  blc=0;
  blinklatch=1;
  blinkc=0;
}

if(blinklatch==1){
  if (blc<100){
    blc++;
    digitalWrite(3, HIGH);
  }
  if (blc>=100){
    digitalWrite(3, LOW);
    blinklatch=0;
  }

}

byte bc3 =! digitalRead(7);  
byte bc4 =! digitalRead(8);
crush_amt=(bc3<<1)|bc4;




 
}



byte feedback_delay(int dly_input){

  byte dly_out;
  int fb_in,in_in,in_mix;
  int out_temp1;



  dly_tick++;
  if (dly_tick>crush_amt){

    write_head++;  
    dly_tick=0;

  }
  
  if (write_head > (DLY_BUF_LEN-2)){
    write_head=0;
  }


  read_head = (write_head + lerp_dly);


  if (read_head > (DLY_BUF_LEN-2)){
    read_head-=(DLY_BUF_LEN-2);
  }
if (reverse==1){ //OFF
  read_head = (write_head + lerp_dly);


  if (read_head > (DLY_BUF_LEN-2)){
    read_head-=(DLY_BUF_LEN-2);
  }
}



if (reverse==0){
  int inv_write_head=(write_head-DLY_BUF_LEN-2)*-1;
  read_head = (inv_write_head+lerp_dly);

  if (read_head > (DLY_BUF_LEN-2)){
    read_head-=(DLY_BUF_LEN-2);
  }

  if (read_head <0){
    read_head+=(DLY_BUF_LEN-2);
  }
}



if (freeze==1){



  dly_out = dly_buffer[read_head];



  fb_in = ((feedback-127)*fb_tmp)>>8;
  in_in = ((dly_input)*fb_tmp_inv)>>8;

  dly_mix_2=(((dly_input)*125)>>8);
  dly_mix=((dly_out-127)*(125))>>8;
  in_mix =  (fb_in+in_in)+127;

  if (in_mix>=253){
    in_mix=253;
  }

  if (in_mix<=2){
    in_mix=2;

  }


  dly_buffer[write_head] = in_mix;
  out_temp1 = dly_mix_2+dly_mix+127;



}


if (freeze==0){
  int lerp_dly_inv=(lerp_dly-DLY_BUF_LEN-2)*-1;


read_head=write_head % lerp_dly_inv;


  out_temp1 = (dly_buffer[read_head])>>1;
 //out_temp1 = 0;

}

feedback = out_temp1;

return out_temp1;
}


int readchange(byte n, int input){
  int diff=8;
  //int input = analogRead(n);
  int output;
  if ((input>(previnput[n]+diff))||(input<(previnput[n]-diff))){

    output= input;
    previnput[n]=input;
    //Serial.println("C");
  }


  else{
    output=  previnput[n];
    ///Serial.println("-");
  }

  if (input>1020){

    output=1024;
  }

  if (input<3){

    output=0;
  }

  return output;
}






int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



