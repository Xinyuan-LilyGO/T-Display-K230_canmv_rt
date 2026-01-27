#include "LR2021.h"
#if !RADIOLIB_EXCLUDE_LR20XX
#include <unistd.h>
LR2021::LR2021(Module* mod) : LR20xx(mod) {
  chipType = RADIOLIB_LR20xx_DEVICE_LR2021;
}

int16_t LR2021::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, float tcxoVoltage) {
  // execute common part
  int16_t state = LR20xx::begin(bw, sf, cr, syncWord, preambleLength, tcxoVoltage);
  RADIOLIB_ASSERT(state);
 
  // configure publicly accessible settings
  state = setFrequency(freq);
  RADIOLIB_ASSERT(state);
  printf("qqqqqqqqqqqqqqqqqqqqqqqqqq\n");
  state=calibrate(0x6F);//
  RADIOLIB_ASSERT(state);
  usleep(80000);
  printf("calibrate after\n"); 
  if(freq<1600)//433MHZ 868MHZ 915MHZ 960MHZ
  {
    state=configLfClock(0x00);
    RADIOLIB_ASSERT(state);
    printf("configLfClock after\n");
    state=setPaConfig(0,0,6,7,16);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01
    RADIOLIB_ASSERT(state);
    printf("setPaConfig,state:%d\n",state);
    state=setPa(0);
    RADIOLIB_ASSERT(state);
    printf("setPa,state:%d\n",state);
    state=setRxPath(0,0);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01  rx_boost:0-7 5
    RADIOLIB_ASSERT(state);
    printf("setRxPath0 after\n");
  }
  else//2400MHZ
  {
    //state = setPaConfig(0x1, 0, 0x06, 0x07,0x10);
    printf("aaaaaaaaaaaaaaaaaaaaaaaaa\n");
    state=setPaConfig(1,0,6,7,16);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01  16
    RADIOLIB_ASSERT(state);
    printf("setPaConfig,state:%d\n",state);
    state=setPa(1);
    RADIOLIB_ASSERT(state);
    printf("setPa,state:%d\n",state);
    state=setRxPath(1,4);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01  rx_boost:0-7 5 0
    RADIOLIB_ASSERT(state);
    printf("setRxPath4 after\n");
    //usleep(155000);
  }  
  usleep(500000);
  state = setOutputPower(power);
  printf("setOutputPower after\n");
  return(state);
}

int16_t LR2021::beginGFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, float tcxoVoltage) {
  // execute common part
  int16_t state = LR20xx::beginGFSK(br, freqDev, rxBw, preambleLength, tcxoVoltage);
  RADIOLIB_ASSERT(state);

  // configure publicly accessible settings
  state = setFrequency(freq);
  RADIOLIB_ASSERT(state);

  state = setOutputPower(power);
  return(state);
}

int16_t LR2021::beginLRFHSS(float freq, uint8_t bw, uint8_t cr, int8_t power, float tcxoVoltage) {
  // execute common part
  int16_t state = LR20xx::beginLRFHSS(bw, cr, tcxoVoltage);
  RADIOLIB_ASSERT(state);

  // configure publicly accessible settings
  state = setFrequency(freq);
  RADIOLIB_ASSERT(state);

  state = setOutputPower(power);
  return(state);
}
int16_t LR2021::beginFLRC(float freq, uint16_t br_bw, uint8_t cr2, int8_t power2, uint16_t preambleLength2,float tcxoVoltage2, uint8_t dataShaping)
{
#if 0
 //float freq1=2400;//915
 float bw=1000;
 uint8_t sf=7;//10 
 uint8_t cr=7; //8
 uint8_t syncWord=0x12; //0x12
 int8_t power=12;//22
 uint16_t preambleLength=8;//12
 float tcxoVoltage=0;//1.6,2.7,0:normal 32m ,other:tcxo
 int16_t state = LR20xx::begin(bw, sf, cr, syncWord, preambleLength, tcxoVoltage);
  RADIOLIB_ASSERT(state);
#endif

#if 0
//OK
//float freq1=2400;//915
 float bw=500;
 uint8_t sf=7;//10 
 uint8_t cr=7; //8
 uint8_t syncWord=0x12; //0x12
 int8_t power=12;//22
 uint16_t preambleLength=8;//12
 float tcxoVoltage=0;//1.6,2.7,0:normal 32m ,other:tcxo
#endif

 int16_t state = LR20xx::beginFLRC(br_bw, cr2,preambleLength2,tcxoVoltage2, dataShaping);
 RADIOLIB_ASSERT(state);
  // configure publicly accessible settings
  state = setFrequency(freq);
  RADIOLIB_ASSERT(state);
  printf("setFrequency,state:%d,freq:%f\n",state,freq);
 
  state=calibrate(0x6F);
  RADIOLIB_ASSERT(state); 
  usleep(80000);
  if(freq<1600)//433MHZ 868MHZ 915MHZ 960MHZ
  {
    state=configLfClock(0x00);
    RADIOLIB_ASSERT(state);
    printf("configLfClock after\n");
    state=setPaConfig(0,0,6,7,16);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01
    RADIOLIB_ASSERT(state);
    printf("setPaConfig,state:%d\n",state);
    state=setPa(0);
    RADIOLIB_ASSERT(state);
    printf("setPa,state:%d\n",state);
    state=setRxPath(0,0);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01  rx_boost:0-7 5
    RADIOLIB_ASSERT(state);
    printf("setRxPath0 after\n");
  }
  else//2400MHZ
  {
    //state = setPaConfig(0x1, 0, 0x06, 0x07,0x10);  

    state=setPaConfig(1,0,6,7,16);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01  16
    RADIOLIB_ASSERT(state);
    printf("setPaConfig,state:%d\n",state);
    state=setPa(1);
    RADIOLIB_ASSERT(state);
    printf("setPa,state:%d\n",state);
    state=setRxPath(1,4);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01  rx_boost:0-7 5 0
    RADIOLIB_ASSERT(state);
    printf("setRxPath4 after\n");
    //usleep(155000);
    
  }
  usleep(500000);
  state = setOutputPower(power2);
  return(state);










}






int16_t LR2021::setFrequency(float freq) {
  return(this->setFrequency(freq, true));//true
}

int16_t LR2021::setFrequency(float freq, bool calibrate, float band) {
  

  // calibrate image rejection
  if(calibrate) {
    //int16_t state = LR20xx::calibImage(freq - band, freq + band);
    if(freq<1600)//433MHZ 868MHZ 915MHZ 960MHZ
    {
    int16_t state =calibrate_front_end(0,(uint32_t)(freq*1000000.0f),0,(uint32_t)(0*1000000.0f),0,(uint32_t)(0*1000000.0f));
    //LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01
    RADIOLIB_ASSERT(state);
    printf("lr2021,calibrate_front_end.PA_SEL_LF,state:%d\n",state);
    }
    else//2400MHZ
    {
    int16_t state =calibrate_front_end(1,(uint32_t)(freq*1000000.0f),0,(uint32_t)(0*1000000.0f),0,(uint32_t)(0*1000000.0f));
    //LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01
    RADIOLIB_ASSERT(state);
    printf("lr2021,calibrate_front_end.PA_SEL_HF,state:%d\n",state);
    }
  }
  //usleep(155000); //5000
  usleep(500000); 
  // set frequency
  return(LR20xx::setRfFrequency((uint32_t)(freq*1000000.0f)));
}

int16_t LR2021::setOutputPower(int8_t power) {
  // set output power
  int16_t state = setTxParams(power*2, RADIOLIB_LR20xx_PA_RAMP_2_US);// init_factory:RADIOLIB_LR20xx_PA_RAMP_2_US  32
  printf("setTxParams,state:%d,power:%d\n",state,power);
  return(state);
  
  
}

int16_t LR2021::setOutputPower(int8_t power, bool forceHighPower) {
  // check if power value is configurable
  int16_t state = this->checkOutputPower(power, NULL, forceHighPower);
  RADIOLIB_ASSERT(state);

  // determine whether to use HP or LP PA and check range accordingly
  bool useHp = forceHighPower || (power > 12);
  
  // TODO how and when to configure OCP?
  useHp=false;
  // update PA config - always use VBAT for high-power PA
  state = setPaConfig((uint8_t)useHp, 0, 0x06, 0x07,0x06);
  RADIOLIB_ASSERT(state);

  // set output power
  state = setTxParams(power*2, RADIOLIB_LR20xx_PA_RAMP_2_US);//RADIOLIB_LR20xx_PA_RAMP_32_US
  return(state);
}

int16_t LR2021::checkOutputPower(int8_t power, int8_t* clipped) {
  return(checkOutputPower(power, clipped, false));
}

int16_t LR2021::checkOutputPower(int8_t power, int8_t* clipped, bool forceHighPower) {
  if(forceHighPower || (power > 12)) {
    if(clipped) {
      *clipped = RADIOLIB_MAX(-9, RADIOLIB_MIN(22, power));
    }
    RADIOLIB_CHECK_RANGE(power, -9, 22, RADIOLIB_ERR_INVALID_OUTPUT_POWER);
  
  } else {
    if(clipped) {
      *clipped = RADIOLIB_MAX(-19, RADIOLIB_MIN(12, power));
    }
    RADIOLIB_CHECK_RANGE(power, -19, 12, RADIOLIB_ERR_INVALID_OUTPUT_POWER);
  
  }
  return(RADIOLIB_ERR_NONE);
}
#endif
