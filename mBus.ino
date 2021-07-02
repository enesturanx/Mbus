#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#include <SoftwareSerial.h>


#define rxPin 13
#define txPin 33


byte dizi[]={0x10, 0x40, 0x02, 0x42, 0x16};
byte command = NULL;
SoftwareSerial mySerial(rxPin,txPin); // RX, TX

volatile int16_t abuf[11];
//volatile char rxDATA[162];
byte rxDATA[81];

int16_t abufPos = 0;
float value;
int adcValue;
bool alarmFlag = 1;;
bool startBit = 1;
int i,temp,j=0;
int ldata = 0; // RX data length 
int lcount = 0; // RX data
float lineVoltage=0;
int thVoltage=259; 

hw_timer_t * adcTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t * startTimer = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

int dec1, dec2;
char hex1, hex2;
char dec2hex(int dec){
  char hexdecnum = ' '; 
  int i=1, temp;  
  if (dec == 0) {return '0';}
  while(dec!=0) 
  { 
    temp=dec%16; 
    if(temp<10) 
    { 
      temp=temp+48; 
    } 
    else 
    { 
      temp=temp+55; 
    } 
    hexdecnum=temp; 
    dec=dec/16; 
  }
  return hexdecnum;
}

void local_adc1_read(int channel) { 
  if(abufPos < 11){
  SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
  //while (SENS.sar_slave_addr1.meas_status != 0);
  SENS.sar_meas_start1.meas1_start_sar = 0;
  SENS.sar_meas_start1.meas1_start_sar = 1;
  //while (SENS.sar_meas_start1.meas1_done_sar == 0);
  abuf[abufPos++] = SENS.sar_meas_start1.meas1_data_sar;
}

  else if(abufPos > 10) { 
        int dec1 = 0, dec2 = 0, dec;    
        abufPos = 0;
        alarmFlag = 1;
        startBit = 1;   
        timerAlarmDisable(adcTimer);
        for(i=8; i>=1;i--){
          if(i>4){
            dec1 += (!(abuf[i]/240))*pow(2,(i-5));
            //Serial.println(dec1);
            }
          else {
            dec2 += (!(abuf[i]/240))*pow(2,(i-1)); 
            //Serial.println(dec2);
            }         

      }
       dec = dec1*16+dec2;
      if((dec == 104) && (ldata == 0)) {
        ldata = 81;
        }
      else if((dec == 229) && (ldata == 0)) {
        //Serial.write(dec2);
        ldata = 1;
      }

      if(lcount < ldata){
               
    //  Serial.print(dec);
    //        Serial.print(' ');
       //     Serial.write(dec);

        rxDATA[lcount++] = dec;
       // rxDATA[lcount++] = hex2;

       }      
    }
}

void printReceivedData(){
  //Serial.println(2*ldata);
  //if((rxDATA[0] == '6') && (rxDATA[1] == '8') && (rxDATA[160] != '1') && (rxDATA[161] != '6')){
  //while(lcount>0) {
    //Serial.write(rxDATA[ldata-(lcount--)]);
    Serial.write(rxDATA,81);
    //Serial.print(' ');
    //Serial.print(rxDATA[2*ldata-(lcount--)]);
   // Serial.print(' ');
   // }
   lcount = 0;
    ldata = 0;
    command = NULL;
   // Serial.println();
  //  }
}
void IRAM_ATTR onTimer1() {
  portENTER_CRITICAL_ISR(&timerMux1);

  
  //local_adc1_read(ADC1_CHANNEL_6);
  timerAlarmEnable(adcTimer);
  timerAlarmDisable(startTimer);
  
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  local_adc1_read(ADC1_CHANNEL_6);

  portEXIT_CRITICAL_ISR(&timerMux);
}
void setup() {
  Serial.begin(2400,SERIAL_8E1);


  // put your setup code here, to run once:
  //xTaskCreate(complexHandler, "Handler Task", 8192, NULL, 1, &complexHandlerTask);
  adcTimer = timerBegin(0,80, true); // 80 MHz / 80 = 1 MHz hardware clock for easy figuring
  timerAttachInterrupt(adcTimer, &onTimer, true); // Attaches the handler function to the timer 
  timerAlarmWrite(adcTimer,417, true); // Interrupts when counter == 45, i.e. 22.222 times a second
  
  startTimer = timerBegin(1, 80, true); // 80 MHz / 80 = 1 MHz hardware clock for easy figuring
  timerAttachInterrupt(startTimer,&onTimer1, true); // Attaches the handler function to the timer 
  timerAlarmWrite(startTimer,300, true); // Interrupts when counter == 45, i.e. 22.222 times a second

  
  adc1_config_width(ADC_WIDTH_BIT_12 );
  adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_DB_0);

  

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  mySerial.begin(2400, SWSERIAL_8E1);
    while(j<10000){
    j++;
    lineVoltage += adc1_get_raw(ADC1_CHANNEL_6);  
    if(j >= 10000){
      lineVoltage = lineVoltage/10000;
      thVoltage = lineVoltage + thVoltage;
      delay(10);
      Serial.print("LINE VOLTAGE IS: ");
      Serial.println(lineVoltage);
      Serial.print("THRESHOLD VOLTAGE IS: ");
      Serial.print(thVoltage);
      Serial.println();
      delay(10);
    } 
  }
}

void loop() {
  // put your main code here, to run repeatedly:
 //if(startBit){
 //if(command != NULL){
 while(Serial.available() == 0 && command == NULL){};
 if(Serial.available() > 0){
   command = Serial.read();
    mySerial.write(command);
   // Serial.write(command);
  }
 
 

  adcValue = adc1_get_raw(ADC1_CHANNEL_6);
 //}
//Serial.write(adcValue);
  if(adcValue > 260 && alarmFlag) {
        

    startBit = 0;
    //Serial.println(adcValue);
    alarmFlag = 0; 
    delayMicroseconds(208);   
    //timerAlarmEnable(startTimer);
    timerAlarmEnable(adcTimer); 
 }

if((lcount >= ldata) && (ldata != 0)) {
  printReceivedData();
  }
}
