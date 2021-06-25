/* FNF Nijat Aliyev*/

#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

volatile int16_t abuf[11];
volatile char rxDATA[162];
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
        int dec1 = 0, dec2 = 0;
        
        abufPos = 0;
        alarmFlag = 1;
        startBit = 1;   
        timerAlarmDisable(adcTimer);
        for(i=8; i>=1;i--){
          if(i>4){
            dec1 += (!(abuf[i]/thVoltage))*pow(2,(i-5));
            //Serial.println(dec1);
            }
          else {
            dec2 += (!(abuf[i]/thVoltage))*pow(2,(i-1)); 
            //Serial.println(dec2);
            }         
        hex1 = dec2hex(dec1);
        hex2 = dec2hex(dec2);
      }
      if((hex1 == '6') && (hex2 = '8') && (ldata == 0)) {
        ldata = 81;
        }
      else if((hex1 == 'E') && (hex2 = '5') && (ldata == 0)) {
        ldata = 1;
      }
      if(lcount < 2*ldata){
        rxDATA[lcount++] = hex1;
        rxDATA[lcount++] = hex2;
       }      
    }
}

void printReceivedData(){
  //Serial.println(2*ldata);
  //if((rxDATA[0] == '6') && (rxDATA[1] == '8') && (rxDATA[160] != '1') && (rxDATA[161] != '6')){
  while(lcount>0) {
    Serial.print(rxDATA[2*ldata-(lcount--)]);
    Serial.print(rxDATA[2*ldata-(lcount--)]);
    Serial.print(' ');
    }
    ldata = 0;
    Serial.println();
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
  Serial.begin(115200);


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

  

  pinMode(33, OUTPUT);
  digitalWrite(33, HIGH);

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
  adcValue = adc1_get_raw(ADC1_CHANNEL_6);
 //}

  if(adcValue > 260 && alarmFlag) {
    startBit = 0;
    //Serial.println(adcValue);
    alarmFlag = 0; 
    delayMicroseconds(206);   
    //timerAlarmEnable(startTimer);
    timerAlarmEnable(adcTimer); 
 }

if((lcount >= 2*ldata) && (ldata != 0)) {
  printReceivedData();
  }
}
