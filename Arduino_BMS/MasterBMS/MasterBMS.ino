#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LT_I2C.h"
#include "QuikEval_EEPROM.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6811.h"
#include <stdio.h>
#include "stdlib.h"

#include <SoftwareSerial.h>
/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1

#define DATALOG_DISABLED 0
#define limit_v 117.6
/**************** Local Function Declaration *******************/
uint8_t buff[30];
void measurement_loop(uint8_t datalog_en);
void print_cells(uint8_t datalog_en);
void check_error(int error);
//void STMsig(void);
/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
const uint8_t TOTAL_IC = 4;//!< Number of ICs in the daisy chain. # of slave BMS
const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED; //!< Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection
const uint8_t SEL_REG_A = REG_1; //!< Register Selection
const uint8_t SEL_REG_B = REG_2; //!< Register Selection

const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41500; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.15V)
const uint16_t UV_THRESHOLD = 25500; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(2.55V)

//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = ENABLED;  //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = ENABLED; //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED; //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = ENABLED; //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = ENABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = ENABLED; //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop

cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable

bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool GPIOBITS_A[5] = {true, true, true, true, true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
uint16_t UV = UV_THRESHOLD; //!< Under-voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD; //!< Over-voltage Comparison Voltage
bool DCCBITS_A[12] = {false, false, false, false, false, false, false, false, false, false, false, false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
//cell balancing에 이용될 것.
bool DCTOBITS[4] = {false, false, false, true}; //!< Discharge time value // Dcto 0,1,2,3 // Programed for 4 min
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */

int SDCout = 7;
int Buzzer = 8;
int BrakeSensor = A3;

int count = 0;
int BScnt = 0;
int BZcnt = 0;
int isr_cnt=0;

void setup()
{
  pinMode(SDCout, OUTPUT);
  pinMode(BrakeSensor, INPUT);
  pinMode(Buzzer, OUTPUT);

  uint8_t streg = 0; //state register
  int8_t error = 0; //pec error count. being -1 when error occurs
  uint32_t conv_time = 0; //convolsion time. dont touch
  int8_t s_pin_read = 0;
  Serial.begin(115200);


//  attachInterrupt(0,BuzzerTone_isr,HIGH);
  
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock
  LTC6811_init_cfg(TOTAL_IC, BMS_IC); //# of slave BMS, cell_asic stucture. Initialization.

  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    LTC6811_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
  } //0~3번 돌아가면서 ic에 선언된 전역변수로 세팅해주기

  LTC6811_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6811_init_reg_limits(TOTAL_IC, BMS_IC);

  wakeup_sleep(TOTAL_IC);
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    BMS_IC[current_ic].pwm.tx_data[0] = 0x88; // Duty cycle for S pin 2 and 1
    BMS_IC[current_ic].pwm.tx_data[1] = 0x88; // Duty cycle for S pin 4 and 3
    BMS_IC[current_ic].pwm.tx_data[2] = 0x88; // Duty cycle for S pin 6 and 5
    BMS_IC[current_ic].pwm.tx_data[3] = 0x88; // Duty cycle for S pin 8 and 7
    BMS_IC[current_ic].pwm.tx_data[4] = 0x88; // Duty cycle for S pin 10 and 9
    BMS_IC[current_ic].pwm.tx_data[5] = 0x88; // Duty cycle for S pin 12 and 11
    //data sheet table 51.
    LTC6811_wrcfg(TOTAL_IC, BMS_IC); //write configuration
  }
  LTC6811_wrpwm(TOTAL_IC, 0, BMS_IC);
  wakeup_idle(TOTAL_IC); // wake the ICs connected in daisy chain. idle mode: 프로세스 실행하지 않고 있는 상태

  wakeup_sleep(TOTAL_IC);
  /**************************************************************************************
     S pin control.
     1)Ensure that the pwm is set according to the requirement using the previous case.
     2)Choose the value depending on the required number of pulses on S pin.
     Refer to the data sheet.
  ***************************************************************************************/
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    BMS_IC[current_ic].sctrl.tx_data[0] = 0xFF; // No. of high pulses on S pin 2 and 1
    BMS_IC[current_ic].sctrl.tx_data[1] = 0xFF; // No. of high pulses on S pin 4 and 3
    BMS_IC[current_ic].sctrl.tx_data[2] = 0xFF; // No. of high pulses on S pin 6 and 5
    BMS_IC[current_ic].sctrl.tx_data[3] = 0xFF; // No. of high pulses on S pin 8 and 7
    BMS_IC[current_ic].sctrl.tx_data[4] = 0xFF; // No. of high pulses on S pin 10 and 9
    BMS_IC[current_ic].sctrl.tx_data[5] = 0xFF; // No. of high pulses on S pin 12 and 11
    // Active cell balance /w LT8584
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
  }
  LTC6811_wrsctrl(TOTAL_IC, streg, BMS_IC);
  // Start S Control pulsing
  wakeup_idle(TOTAL_IC);
  LTC6811_stsctrl(); //start sControl

  // Read S Control Register Group
  wakeup_idle(TOTAL_IC);
  error = LTC6811_rdsctrl(TOTAL_IC, streg, BMS_IC);
  check_error(error);
}

/*!*********************************************************************
  \brief Main loop
  @return void
***********************************************************************/
void loop()
{
  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS_IC);
  digitalWrite(SDCout, HIGH);
  measurement_loop(DATALOG_ENABLED);
}

void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  char input = 0;

  while (input != 'm')
  {
    BuzzerTone();
    for (uint8_t current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {
      if (BMS_IC[current_ic].stat.flags[0] == 0xAA) {
        digitalWrite(10, LOW);
        //Serial.println("good!");
      }
      else
      {
        digitalWrite(10, HIGH);
      }
    }
    if (Serial.available() > 0)
    {
      input = read_char();
    }

    if (MEASURE_CELL == ENABLED) //중요
    {
      LTC6811_set_cfgr_dis(TOTAL_IC, BMS_IC, DCCBITS_A);
      wakeup_idle(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      LTC6811_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
      check_error(error);
      print_cells(datalog_en); //data print
    }
    delay(MEASUREMENT_LOOP_TIME);
  }
}

/*!*********************************
  \brief Prints the main menu
  @return void
***********************************/


/********************************************************************************
  \brief Prints the configuration data that is going to be written to the LTC6811
  to the serial port.
  @return void
 ********************************************************************************/

void print_cells(uint8_t datalog_en)
{
  double total_v = 0;
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
//    Serial.print("[");                                                        // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 처리          
//    Serial.print(current_ic + 1);                                             // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 처리
//    Serial.print("]");                                                        // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 처리
//    Serial.print(":");                                                        // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 처리  
    if (datalog_en == 0) {

    }
    else {
      for (int i = 0; i < BMS_IC[current_ic].ic_reg.cell_channels; i++)
      {
        
        
        total_v += (double)(BMS_IC[current_ic].cells.c_codes[i] * 0.0001);
            
        if(BMS_IC[current_ic].cells.c_codes[i] > 10000 && BMS_IC[current_ic].cells.c_codes[i] < 65535)
        {
       //  
        // else{}
        bool StateOfCell = (BMS_IC[current_ic].cells.c_codes[i] < OV_THRESHOLD && BMS_IC[current_ic].cells.c_codes[i] > UV_THRESHOLD);  
        if (!StateOfCell) digitalWrite(SDCout, LOW);
        }
//        Serial.print("[");                                                     // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 처리 
//        Serial.print((double)(BMS_IC[current_ic].cells.c_codes[i] * 0.0001));  // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 처리 
//        Serial.print("]");                                                     // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 처리 
 
      }
    }
//    Serial.println();                                                           // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 처리 
//    Serial.println();                                                           // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 처리 
  }
//String SerialData="";                                                        // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 해제
//SerialData = String(total_v,2);                                              // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 해제
////Serial.println(117.56);                                                      // Tester
//Serial.println(SerialData);                                                  // 배터리 팩 검차 마무리 과정에서 수정가능하면 주석 해제
}

void check_error(int error)
{
  if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}
/*
void STMsig(void)
{
  Serial.println("SDC signal OUTPUT to STM");
  digitalWrite(SDCout, HIGH);
  
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    for (int i = 0; i < BMS_IC[current_ic].ic_reg.cell_channels; i++)
    {
      bool StateOfCell = (BMS_IC[current_ic].cells.c_codes[i] < OV_THRESHOLD && BMS_IC[current_ic].cells.c_codes[i] > UV_THRESHOLD);
      if (!StateOfCell)digitalWrite(SDCout, LOW);
      else digitalWrite(SDCout, HIGH);
    }
}
*/

void BuzzerTone(void)
{
  int BS = analogRead(BrakeSensor);
  
  if (BScnt > 0) {
    exit(0);
  }
  if (BS > 900 && BScnt == 0 && BZcnt == 0) {
    tone(Buzzer, 440, 500);
    delay(500);
    tone(Buzzer, 392, 500);
    delay(500);
    tone(Buzzer, 440, 500);
    delay(500);
    tone(Buzzer, 523.25, 500);
    delay(500);
    tone(Buzzer, 587, 500);
    delay(500);
    tone(Buzzer, 523, 500);
    delay(500);
    tone(Buzzer, 783, 500);
    delay(1000);
    delay(3000);
    count++;
    BZcnt++;
  }
  if (BS < 900 && count > 0) {
    BScnt++;
  }
//  Serial.print("BS"); Serial.println(BS);
}

//void BuzzerTone_isr(){
//    
//    if(isr_cnt<1){
//    tone(Buzzer, 440, 500);
//    delay(500);
//    tone(Buzzer, 392, 500);
//    delay(500);
//    tone(Buzzer, 440, 500);
//    delay(500);
//    tone(Buzzer, 523.25, 500);
//    delay(500);
//    tone(Buzzer, 587, 500);
//    delay(500);
//    tone(Buzzer, 523, 500);
//    delay(500);
//    tone(Buzzer, 783, 500);
//    delay(1000);
//    delay(3000);
//    isr_cnt++;
//    }
//
//}
