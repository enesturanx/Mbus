#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>


volatile int16_t abuf[11];
int16_t abufPos = 0;
float value;
int adcValue;
bool alarmFlag = 1;
bool startBit = 1;
int i,temp,j=0;
float lineVoltage=0;
int thVoltage=259; 

hw_timer_t * adcTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t * startTimer = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

void local_adc1_read(int channel) { 
    if(abufPos < 11){
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
    //while (SENS.sar_slave_addr1.meas_status != 0);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    //while (SENS.sar_meas_start1.meas1_done_sar == 0);
   // adc_value = SENS.sar_meas_start1.meas1_data_sar;
    abuf[abufPos++] = SENS.sar_meas_start1.meas1_data_sar;
}
 /*   if(abuf[abufPos-1] > 893){
    digitalWrite(33, LOW);
    }
    else {digitalWrite(33, HIGH);
    }*/

//    if(abufPos > 10) {
//      abufPos = 0;
//        alarmFlag = 1;
//        startBit = 1;
//        timerAlarmDisable(adcTimer);
//        for(i=8; i>=1;i--){
//        Serial.print(!(abuf[i]/260));}
//        Serial.println();
//    }
      if(abufPos > 10) {
        abufPos = 0;
        alarmFlag = 1;
        startBit = 1;  
        timerAlarmDisable(adcTimer);
        temp = 0;
        for(i=8; i>=1;i--){
          temp = temp | (abuf[i]/thVoltage);
          //Serial.println(!temp);
          }   
        if(temp) {
          for(i=8; i>=1;i--){
             Serial.print(!(abuf[i]/thVoltage));
           // Serial.print(abuf[i]);
      }
      Serial.println();
      }
     
    }
    //Serial.println(abuf[abufPos-1]);
    //return adc_value;
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
// 
// if(abufPos > 9) {
//  abufPos = 0;
//  int i;
//  timerAlarmDisable(adcTimer);
// // for(i =0; i<11; i++) {
//   /* value = abuf[i]*1.1/4096;
//    Serial.println(abuf[i]);*/
//   /* if(abuf[i] > 894){
//    digitalWrite(33, HIGH);
//    delay(1);}
//    else {digitalWrite(33, LOW);
//    delay(1);}*/
//  //}
//  
//  alarmFlag = 1;}
}
