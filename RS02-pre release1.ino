#include <TimerOne.h>


#define DLY_BUF_LEN 1030
byte dly_buffer[DLY_BUF_LEN]={};



unsigned long accumulator[3]={};
unsigned long increment[3]={};
uint16_t out[3]={};
uint16_t freq[3]={};
uint16_t waveindex[3]={};

unsigned long d,t,prev,prev2;
byte tickLED,tick2,pot_tick;
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
int blink_inv;
byte tick4;
int previnput[2]={};
int in1,in0,in3;
byte AM_enable;
int dig_in,out_mix;
int mult_mix,lerp_tick,amin;
int dout,fout;
byte fb_mode,freeze,lofi_tick,lofi,reverse;

void setup(void)
{

  pinMode(19, OUTPUT);  
  pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);

  pinMode(5, OUTPUT);

  pinMode(6, INPUT);
  digitalWrite(6, HIGH);

  pinMode(2, INPUT);
  digitalWrite(2, HIGH);

  pinMode(4, INPUT);   
  digitalWrite(4,HIGH);

  pinMode(3, INPUT);
  digitalWrite(3,HIGH);

  pinMode(7, INPUT);
  digitalWrite(7,HIGH);


  pinMode(8, INPUT);
  digitalWrite(8,HIGH);

  TCCR0B = TCCR0B & 0b11111000 | 0x01 ; //timer 0 62500Hz
  //TCCR1B = TCCR1B & 0b11111000 | 0x01 ; //timer 1 62500Hz


//http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
  const unsigned char PS_16 = (1 << ADPS2);
  const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
  const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
  const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  ADCSRA &= ~PS_128;
  ADCSRA |=PS_16;

  
  dds_rate=75;
  dds_tune=1/(dds_rate*.000001);

  DDS();

  Timer1.initialize(dds_rate);
  Timer1.attachInterrupt(DDS); 

  Serial.begin(9600);
}


int ledState = LOW;
volatile unsigned long blinkCount = 0; // use volatile for shared variables



void DDS()
{

  PORTB |= _BV(PORTB5);



  pot_tick=!pot_tick;

  if(pot_tick==1){
    in0 =analogRead(A0);

  }

  if(pot_tick==0){
    in1 =analogRead(A1)>>2;  
  }

volatile byte am = (digitalRead(6));

volatile int a3 =(analogRead(A3)>>2)-127; 
volatile int amx = a3*am;
volatile byte dly_in = amx+127;


volatile byte dly_outf=feedback_delay(dly_in);
analogWrite(5, dly_outf); 

lerp_tick++;
if (lerp_tick>6){
 byte dly_step=1;

 if(lerp_dly < dly_time)  
 lerp_dly +=dly_step;
 if(lerp_dly > dly_time)
 lerp_dly -= dly_step;

 lerp_tick=0;
}



PORTB &= ~_BV(PORTB5);



}

int blinkc,blinklatch,blc;

void loop(void){

  freeze = (digitalRead(8));
  reverse = (digitalRead(7));

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
    tick4=!tick4;
    digitalWrite(19,tick4);   // blinks with the delay rate
    blinkc=0;
}

byte bc3 =! digitalRead(3);  
byte bc4 =! digitalRead(4);
crush_amt=(bc3<<1)|bc4;

//   Serial.println(prev|dly_time);


  //Serial.print("blinkCount = ");
  if ((millis()-prev)>5000){
    prev=millis();
    


 //   Serial.println(waveindex[0]);
   Serial.println(fout);

  //  Serial.print(fb_tmp); Serial.print(" ");  Serial.println(fb_tmp_inv);
   // Serial.println();
 }

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
  in_in = ((dly_input-127)*fb_tmp_inv)>>8;

  dly_mix_2=(((dly_input-127)*100)>>8);
  dly_mix=((dly_out-127)*(255-100))>>8;
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

