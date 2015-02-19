/*
The Rad-Fi Patchable Synthesizer RS01
http://bleeplabs.com/store/the-rad-fi-system/ 

This works well but needs some work by the time the Rad-Fis ship


Todo:
commenting and moding instructions
PWM rate
general loop speedup

*/
#include <TimerOne.h>
#include "digitalIOPerformance.h"


#define DLY_BUF_LEN 1700

byte dly_buffer[DLY_BUF_LEN];

unsigned long prevserial;
byte tickLED,tick2,pot_tick;
byte dly_tick,fast_pot_tick,slow_sample;
uint16_t write_head,read_head;
int dly_filter,feedback,fb_tmp,fb_tmp_inv,output,dly_input,dly_time,lerp_dly;
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
int blink_inv;
byte tick4;
int previnput[2]={};
int in1,in0,in3;
byte AM_enable;
int dig_in,out_mix;
int mult_mix,lerp_tick,amin;
int dout,filter;
byte fb_mode,freeze,lofi_tick,lofi,reverse,time_mod,time_flip,time_cnt,bypass;
int serial_noise_c,raw_audio,wet_out;
int blinkc,blinklatch,blc,rate_led;
byte delay_rate_f,input_rate_high,input_rate_low,input_rate_f,rate_tick,dly_f;

void setup(void)
{

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);


  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);


  TCCR0B = TCCR0B & 0b11111000 | 0x01 ; //timer 0 62500Hz

  const unsigned char PS_16 = (1 << ADPS2);
  const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
  const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
  const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  ADCSRA &= ~PS_128;
  ADCSRA |=PS_16;


  Timer1.initialize(70); //65 =15.4 70=14.29 //75=13.33
  Timer1.attachInterrupt(DDS); 

  //Serial.begin(9600); // DLY_BUF_LEN needs to be loweres to 1200 is serial is on. 
}


int ledState = LOW;
volatile unsigned long blinkCount = 0; // use volatile for shared variables

int raw_length;

void DDS()
{

//
//digitalWrite(17,HIGH);

if (time_mod==0)
{

  time_cnt++;
  if (time_cnt>(raw_length>>4)){
time_cnt=0;

    if (time_flip==0){
        dly_time-=4;
        if (dly_time<2){
          time_flip=1;
        }

    }
    if (time_flip==1){
      dly_time+=4;
        if (dly_time>DLY_BUF_LEN-2){
          time_flip=0;
        }

    }


  }
}

lerp_tick++;
if (lerp_tick>6){
   lerp_tick=0;
 
 
 byte dly_step=1;

 if(lerp_dly < dly_time-2){  
 lerp_dly +=dly_step;
}
 if(lerp_dly > dly_time+2){
 lerp_dly -= dly_step;
}
}

  pot_tick=!pot_tick;

  if(pot_tick==1){
  raw_length =analogRead(A0)<<1;
  }

  if(pot_tick==0){
  in1 =analogRead(A1)>>2;  

  }


  rate_tick++;
  if (rate_tick>input_rate_f){
      raw_audio =(analogRead(A5)>>2); 
      amin=raw_audio-127; 
      rate_tick=0;
      if (amin<-125) {
          amin=-126;
      //  Serial.println("u");
      }

      if (amin>125) {
        amin=125;
      //  Serial.println("o");
      }
  }

dly_f=feedback_delay(amin);

if (dly_f>160)
{
  digitalWrite(3,HIGH);

}
if (dly_f<=160)
{
  digitalWrite(3,LOW);

}

if (bypass==1)
{
analogWrite(5, dly_f); 
}
if (bypass==0)
{
analogWrite(5, raw_audio>>1); 
}

//digitalWrite(17,LOW);




}

void loop(void){

  freeze = (digitalRead(9));
  reverse = (digitalRead(10));
  time_mod = (digitalRead(11));
  bypass = (digitalRead(13));
  input_rate_high =! (digitalRead(17));
  input_rate_low =! (digitalRead(16));
  input_rate_f=((input_rate_high<<1)|input_rate_low)<<2;//combine 2 bits into a byte

  byte delay_rate_high =! digitalRead(7);  
  byte delay_rate_low =! digitalRead(8);
  delay_rate_f=((delay_rate_high<<1)|delay_rate_low); 


if (time_mod==1)
{
  if (raw_length<DLY_BUF_LEN-2){
    dly_time=raw_length;
  }
  if (raw_length>=DLY_BUF_LEN-2){
    dly_time=DLY_BUF_LEN-2;
  }
}




if (in1<1278){

 // fb_mode=1;
  fb_tmp=(in1);    // update the feedback value.
  fb_tmp_inv=(fb_tmp-255)*-1;  //calculates an inverted value for feedback amount

}

  blink_inv=((dly_time-DLY_BUF_LEN)*-1*(crush_amt+1))>>4;

  rate_led++;

  if (rate_led>blink_inv)
  {
  blinklatch=1;
  rate_led=0;
  }


if(blinklatch==1){                                                                                                                                                 
  blc++;
  if (blc<15){
    digitalWrite(2, HIGH);
  }
  if (blc>=15){
    digitalWrite(2, LOW);
    blinklatch=0;
    blc=0;
  }

}



/*
    prevserial++;
  if (prevserial>200){
    prevserial=0;
//Serial.println(raw_length);
 //  Serial.println(time_cnt);
  // Serial.println(dly_time);

  //  Serial.print(fb_tmp); Serial.print(" ");  Serial.println(fb_tmp_inv);
  //  Serial.println();
 }



if (freeRam ()>800){
//digitalWrite(3,HIGH);
}

if (freeRam ()<800){
//digitalWrite(3,LOW);
}
*/

}

/////////////////////////////////////////////////////////////////////////////////////////////




byte feedback_delay(int dly_input){

  int dly_out;
  int fb_in,in_in,in_mix;
  int out_temp1;


  dly_tick++;
  if (dly_tick>delay_rate_f){
    write_head++;  
    dly_tick=0;
  }




  
  if (write_head > (DLY_BUF_LEN-2)){
    write_head=0;
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

  dly_out = dly_buffer[read_head]-127;
  wet_out=dly_out;

  if (fb_tmp<200)
  {
  filter=(filter+feedback)>>1;

  fb_in = ((filter)*fb_tmp)>>7;
  in_in = ((dly_input)*fb_tmp_inv)>>8;

  //dly_mix_2=(((dly_input)*125)>>8);
  //dly_mix=((dly_out-127)*(125))>>8;

  in_mix =  (fb_in+in_in);

   if (in_mix>125)
   {
     in_mix=125;
   }
   if (in_mix<-125)
   {
     in_mix=-125;
   }
  }

    if (fb_tmp>=200)
  {

  fb_in = ((feedback)*(fb_tmp-200))>>4;
  in_in = ((dly_input)*(fb_tmp_inv+200))>>7;

  //dly_mix_2=(((dly_input)*125)>>8);
  //dly_mix=((dly_out-127)*(125))>>8;

  in_mix =  (fb_in+in_in);
/*
     if (in_mix>125)
   {
     in_mix=125;
   }
   if (in_mix<-125)
   {
     in_mix=-125;
   }
  

   if (in_mix>125)
   {
    int fold_amt=(in_mix-125)<<1;
    in_mix-=fold_amt;
   }
   if (in_mix<-125)
   {
    int fold_amt=(in_mix+125)<<1;
    in_mix+=fold_amt;  
     }
  */

}

  dly_buffer[write_head] = in_mix+127;

  out_temp1 = ((dly_input+dly_out)>>1);

  feedback = out_temp1;

return out_temp1+127;

}


if (freeze==0){
  int lerp_dly_inv=(lerp_dly-DLY_BUF_LEN-2)*-1;


  read_head=write_head % (lerp_dly_inv);
  if (read_head<0)
  {
    read_head=0;
  }

  read_head=write_head % (lerp_dly_inv);
  if (read_head>DLY_BUF_LEN-2)
  {
    read_head=DLY_BUF_LEN-2;
  }


  out_temp1 = ((dly_buffer[read_head]));
 //out_temp1 = 0;


return out_temp1;

}



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

  if (input>DLY_BUF_LEN-2){

    output=DLY_BUF_LEN-2;
  }

  if (input<1){

    output=1;
  }

  return output;
}


int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



