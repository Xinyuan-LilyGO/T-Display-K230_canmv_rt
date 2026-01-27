#include "LR20xx.h"

#include "../../utils/CRC.h"
#include "../../utils/Cryptography.h"

#include <string.h>
#include <math.h>
#include "lr20xx_pram.h"
#if !RADIOLIB_EXCLUDE_LR20XX

LR20xx::LR20xx(Module* mod) : PhysicalLayer(RADIOLIB_LR11X0_FREQUENCY_STEP_SIZE, RADIOLIB_LR11X0_MAX_PACKET_LENGTH) {
  this->mod = mod;
  this->XTAL = false;
}

int16_t LR20xx::begin(float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, uint16_t preambleLength, float tcxoVoltage) {
  // set module properties and perform initial setup
  int16_t state = this->modSetup(tcxoVoltage, RADIOLIB_LR20xx_PACKET_TYPE_LORA);
  RADIOLIB_ASSERT(state);
  this->ldrOptimize=0;//1
  // configure publicly accessible settings
  state = setBandwidth(bw);
  RADIOLIB_ASSERT(state);

  state = setSpreadingFactor(sf);
  RADIOLIB_ASSERT(state);

  state = setCodingRate(cr);
  RADIOLIB_ASSERT(state);

  //state = setSyncWord(syncWord);
  uint16_t lora_sync_word=((uint16_t)(syncWord<<8)|0x34);
  state = setLoRaSyncWord(lora_sync_word);
  RADIOLIB_ASSERT(state);

  state = setPreambleLength(preambleLength);
  RADIOLIB_ASSERT(state);

  // set publicly accessible settings that are not a part of begin method
  state = setCRC(2);
  RADIOLIB_ASSERT(state);

  state = invertIQ(true);//false
  RADIOLIB_ASSERT(state);

  state = setRegulatorLDO();
  RADIOLIB_ASSERT(state);
  printf("begin........\n");
  return(RADIOLIB_ERR_NONE);
}

int16_t LR20xx::beginGFSK(float br, float freqDev, float rxBw, uint16_t preambleLength, float tcxoVoltage) {
  // set module properties and perform initial setup
  int16_t state = this->modSetup(tcxoVoltage, RADIOLIB_LR20xx_PACKET_TYPE_FSK);
  RADIOLIB_ASSERT(state);
   printf("modSetup after\n");
   this->bitRate=400000;//50000   300000    400000  10000
   this->pulseShape=RADIOLIB_LR20xx_GFSK_SHAPING_GAUSSIAN_BT_0_5;//RADIOLIB_LR20xx_GFSK_SHAPING_NONE
   this->rxBandwidth=RADIOLIB_LR20xx_GFSK_RX_BW_1_111_000_HZ;//111000  RADIOLIB_LR20xx_GFSK_RX_BW_1_111_000_HZ  RADIOLIB_LR20xx_GFSK_RX_BW_888_000_HZ
   this->frequencyDev=300000;//25000  150000  200000  50000
   state=setModulationParamsGFSK(this->bitRate, this->pulseShape, this->rxBandwidth, this->frequencyDev);
   printf("setModulationParamsGFSK ,state:%d\n",state);
   RADIOLIB_ASSERT(state);
   printf("setModulationParamsGFSK after\n");
  int len=229;
  this->preambleLengthGFSK=32;
  this->preambleDetLength=RADIOLIB_LR20xx_GFSK_PREAMBLE_DETECTOR_DISABLED;
  //this->syncWordLength=
  this->addrComp=RADIOLIB_LR20xx_GFSK_ADDRESS_FILTERING_DISABLED;
  this->packetType=RADIOLIB_LR20xx_GFSK_HEADER_8BITS;
  this->crcTypeGFSK=RADIOLIB_LR20xx_GFSK_CRC_4_BYTES_INVERTED;//RADIOLIB_LR20xx_GFSK_CRC_1_BYTE_INVERTED
  this->whitening=RADIOLIB_LR20xx_GFSK_WHITENING_ON;//RADIOLIB_LR20xx_GFSK_WHITENING_OFF
  state = setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, 0, this->addrComp, this->packetType, len, this->crcTypeGFSK, this->whitening);
  RADIOLIB_ASSERT(state);
   uint64_t gfsk_sync_word =0x0123456789ABCDEF;
  state =setFSKSyncWord(gfsk_sync_word,1,40);//40
  RADIOLIB_ASSERT(state);
printf("setFSKSyncWord after\n");

  uint16_t whitening_seed=0x0123;
  state =setGfskWhitParams(whitening_seed);
  RADIOLIB_ASSERT(state);
  printf("setGfskWhitParams after\n");

   uint32_t initial=0x01234567;
   uint32_t polynomial=0x01234567;
   state = setGfskCrcParams(initial, polynomial); 
   RADIOLIB_ASSERT(state);

  //state=configFifoIrq(RADIOLIB_LR20xx_FIFO_FLAG_FULL,RADIOLIB_LR20xx_FIFO_FLAG_EMPTY,32,0,0,32);
  state=configFifoIrq(RADIOLIB_LR20xx_FIFO_FLAG_THRESHOLD_HIGH,RADIOLIB_LR20xx_FIFO_FLAG_THRESHOLD_LOW,250,200,0,0);
  RADIOLIB_ASSERT(state);
  printf("configFifoIrq after \n");
  state = setRegulatorLDO();
  RADIOLIB_ASSERT(state);

  return(RADIOLIB_ERR_NONE);
}

int16_t LR20xx::beginLRFHSS(uint8_t bw, uint8_t cr, float tcxoVoltage) {
  // set module properties and perform initial setup
  int16_t state = this->modSetup(tcxoVoltage, RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS );
  RADIOLIB_ASSERT(state);

  // configure publicly accessible settings
  state = setLrFhssConfig(bw, cr);
  RADIOLIB_ASSERT(state);

  state = setSyncWord(0x12AD101B);
  RADIOLIB_ASSERT(state);

  state = setRegulatorLDO();
  RADIOLIB_ASSERT(state);

  // set fixed configuration
  return(setModulationParamsLrFhss(RADIOLIB_LR11X0_LR_FHSS_BIT_RATE_RAW, RADIOLIB_LR11X0_LR_FHSS_SHAPING_GAUSSIAN_BT_1_0));
}

int16_t LR20xx::beginFLRC(uint16_t br, uint8_t cr,uint16_t preambleLength,float tcxoVoltage, uint8_t dataShaping) {
  // set module properties
  this->mod->init();
  this->mod->hal->pinMode(this->mod->getIrq(), this->mod->hal->GpioModeInput);
  this->mod->hal->pinMode(this->mod->getGpio(), this->mod->hal->GpioModeInput);
  this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_ADDR] = Module::BITS_32;
  this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_CMD] = Module::BITS_16;
  this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_16;
  this->mod->spiConfig.statusPos = 0;
  this->mod->spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_READ] = RADIOLIB_LR20xx_CMD_READ_REG_MEM;
  this->mod->spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_WRITE] = RADIOLIB_LR20xx_CMD_WRITE_REG_MEM;
  this->mod->spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_NOP] = RADIOLIB_LR20xx_CMD_NOP;
  this->mod->spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_STATUS] = RADIOLIB_LR20xx_CMD_GET_STATUS;
  this->mod->spiConfig.stream = true;
  this->mod->spiConfig.parseStatusCb = SPIparseStatus;
  this->mod->spiConfig.checkStatusCb = SPIcheckStatus;
  // try to find the LR11x0 chip - this will also reset the module at least once
  if(!LR20xx::findChip(this->chipType)) {
    RADIOLIB_DEBUG_BASIC_PRINTLN("No LR20xx found!");
    this->mod->term();
    return(RADIOLIB_ERR_CHIP_NOT_FOUND);
  }
  RADIOLIB_DEBUG_BASIC_PRINTLN("M\tLR20xx"); 
  
   // initialize FLRC modulation variables
  //this->bitRateKbps = br;
  this->bitRate = RADIOLIB_LR20xx_FLRC_BR_2_600_BW_2_6;//RADIOLIB_LR20xx_FLRC_BR_0_650_BW_0_6
  this->codingRateFLRC = RADIOLIB_LR20xx_FLRC_CR_1_1;//3_4
  this->shaping = RADIOLIB_LR20xx_FLRC_PULSE_SHAPE_BT_05;

  // initialize FLRC packet variables
  //this->preambleLengthGFSK = preambleLength;
  this->syncWordLen = 2;
  this->syncWordMatch = RADIOLIB_LR20xx_FLRC_RX_MATCH_SYNCWORD_1;
  this->crcFLRC = RADIOLIB_LR20xx_FLRC_CRC_OFF;//RADIOLIB_LR20xx_FLRC_CRC_2_BYTES
  
  this->setPacketType(RADIOLIB_LR20xx_PACKET_TYPE_FLRC);
  printf("setPacketType after\n");
  
  
  
  
  // set mode to standby
  //int16_t state;
  int16_t state = standby();
  printf("standby(), state:%d\n",state);
  RADIOLIB_ASSERT(state);
  // set TCXO control, if requested
  if(!this->XTAL && tcxoVoltage > 0.0) {
    state = setTCXO(tcxoVoltage,320000);//9156
    RADIOLIB_ASSERT(state);
  }

  state=clearErrors();
  RADIOLIB_ASSERT(state);
  printf("clearErrors after\n");
  state=patch_load_pram(0x801000,pram, pram_size);
  RADIOLIB_ASSERT(state);
  printf("patch_load_pram after\n");
   uint8_t output[6] = { 0 };
   state=patch_execute_spare(2, 0, 0, output, 6 );
   printf("patch_execute_spare state:%d\n",state);
  state=clearErrors();
  RADIOLIB_ASSERT(state);
  printf("clearErrors after\n");
  

   //state=setPaConfig(0,0,6,7,6);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01
   state=setDioFunction(9,1,3);
   RADIOLIB_ASSERT(state);
    printf("setDioFunction after\n");
    if(is_switch==1)
    {
    state=setDioFunction(5,2,2);
   RADIOLIB_ASSERT(state);
    printf("setDioFunction(5,2,2) after\n");
    state=setDioFunction(6,2,0);
   RADIOLIB_ASSERT(state);
    printf("setDioFunction(6,2,0) after\n");
    state=setDioRfSwitchCfg(5,0,1,0,1,1);
    state=setDioRfSwitchCfg(6,1,0,1,0,0);
    }
   // set Rx/Tx fallback mode to STDBY_RC
  state = this->setRxTxFallbackMode(RADIOLIB_LR20xx_FALLBACK_MODE_STBY_XOSC);//RADIOLIB_LR20xx_FALLBACK_MODE_STBY_RC
  RADIOLIB_ASSERT(state);
  // clear IRQ
  state = this->clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
  printf("clearIrq,state:%d\n",state);
  state |= this->setDioIrqParams(9, (RADIOLIB_LR20xx_IRQ_TX_DONE | RADIOLIB_LR20xx_IRQ_RX_DONE | RADIOLIB_LR20xx_IRQ_TIMEOUT |RADIOLIB_LR20xx_IRQ_LORA_HEADER_ERROR |RADIOLIB_LR20xx_IRQ_LEN_ERROR | RADIOLIB_LR20xx_IRQ_CRC_ERROR));//
  RADIOLIB_ASSERT(state);
  printf("setDioIrqParams:%08X\n",(RADIOLIB_LR20xx_IRQ_TX_DONE | RADIOLIB_LR20xx_IRQ_RX_DONE | RADIOLIB_LR20xx_IRQ_TIMEOUT |RADIOLIB_LR20xx_IRQ_LORA_HEADER_ERROR |RADIOLIB_LR20xx_IRQ_LEN_ERROR | RADIOLIB_LR20xx_IRQ_CRC_ERROR));
  printf("setDioIrqParams,state:%d\n",state);
  
  // if something failed, show the device errors
  #if RADIOLIB_DEBUG_BASIC
  if(state != RADIOLIB_ERR_NONE) {
    // unless mode is forced to standby, device errors will be 0
    standby();
    uint16_t errors = 0;
    getErrors(&errors);
    RADIOLIB_DEBUG_BASIC_PRINTLN("Calibration failed, device errors: 0x%X", errors);
  }
  #endif 
  // set modem
  state = this->setPacketType(RADIOLIB_LR20xx_PACKET_TYPE_FLRC);
  RADIOLIB_ASSERT(state);
  printf("setPacketType after\n");
  
  state=this->setModulationParamsFLRC(RADIOLIB_LR20xx_FLRC_BR_2_600_BW_2_6, RADIOLIB_LR20xx_FLRC_CR_1_1, RADIOLIB_LR20xx_FLRC_PULSE_SHAPE_BT_05);
  RADIOLIB_ASSERT(state);
  printf("setModulationParamsFLRC after\n");
  
 
  
  this->headerType=0;
  this->implicitLen=229;//64  8 230
  this->preambleLengthFLRC=RADIOLIB_LR20xx_FLRC_PREAMBLE_LEN_24_BITS;//16
  this->crcTypeFLRC=RADIOLIB_LR20xx_FLRC_CRC_2_BYTES;
   state = setPacketParamsFLRC(this->preambleLengthFLRC,RADIOLIB_LR20xx_FLRC_SYNCWORD_LENGTH_4_BYTES, RADIOLIB_LR20xx_FLRC_RX_MATCH_SYNCWORD_1,this->headerType, this->implicitLen, this->crcTypeFLRC);
   RADIOLIB_ASSERT(state);
  printf("setPacketParamsFLRC after \n");
  
   state=configFifoIrq(0x3F,0x3F,229,0,0,229);//RADIOLIB_LR20xx_FIFO_FLAG_THRESHOLD_HIGH
  RADIOLIB_ASSERT(state);
  printf("configFifoIrq after \n");
  
  
   state = setRegulatorLDO();
   RADIOLIB_ASSERT(state);
  return(state); 

/*
  // initialize FLRC modulation variables
  this->bitRateKbps = br;
  this->bitRate = RADIOLIB_SX128X_FLRC_BR_0_650_BW_0_6;
  this->codingRateFLRC = RADIOLIB_SX128X_FLRC_CR_3_4;
  this->shaping = RADIOLIB_SX128X_FLRC_BT_0_5;

  // initialize FLRC packet variables
  this->preambleLengthGFSK = preambleLength;
  this->syncWordLen = 2;
  this->syncWordMatch = RADIOLIB_SX128X_GFSK_FLRC_SYNC_WORD_1;
  this->crcGFSK = RADIOLIB_SX128X_GFSK_FLRC_CRC_2_BYTE;
  this->whitening = RADIOLIB_SX128X_GFSK_BLE_WHITENING_OFF;

  // reset the module and verify startup
  int16_t state = reset();
  RADIOLIB_ASSERT(state);

  // set mode to standby
  state = standby();
  RADIOLIB_ASSERT(state);

  // configure settings not accessible by API
  state = config(RADIOLIB_SX128X_PACKET_TYPE_FLRC);
  RADIOLIB_ASSERT(state);

  // configure publicly accessible settings
  state = setFrequency(freq);
  RADIOLIB_ASSERT(state);

  state = setBitRate(br);
  RADIOLIB_ASSERT(state);

  state = setCodingRate(cr);
  RADIOLIB_ASSERT(state);

  state = setOutputPower(pwr);
  RADIOLIB_ASSERT(state);

  state = setPreambleLength(preambleLength);
  RADIOLIB_ASSERT(state);

  state = setDataShaping(dataShaping);
  RADIOLIB_ASSERT(state);

  // set publicly accessible settings that are not a part of begin method
  uint8_t sync[] = { 0x2D, 0x01, 0x4B, 0x1D};
  state = setSyncWord(sync, 4);
  RADIOLIB_ASSERT(state);
*/
  return(state);
}

int16_t LR20xx::reset() {
  // run the reset sequence
  this->mod->hal->pinMode(this->mod->getRst(), this->mod->hal->GpioModeOutput);
  this->mod->hal->digitalWrite(this->mod->getRst(), this->mod->hal->GpioLevelLow);
  this->mod->hal->delay(10);
  this->mod->hal->digitalWrite(this->mod->getRst(), this->mod->hal->GpioLevelHigh);

  // the typical transition duration should be 273 ms
  this->mod->hal->delay(300);
  
  // wait for BUSY to go low
  RadioLibTime_t start = this->mod->hal->millis();
  while(this->mod->hal->digitalRead(this->mod->getGpio())) {
    this->mod->hal->yield();
    if(this->mod->hal->millis() - start >= 3000) {
      RADIOLIB_DEBUG_BASIC_PRINTLN("BUSY pin timeout after reset!");
      return(RADIOLIB_ERR_SPI_CMD_TIMEOUT);
    }
  }

  return(RADIOLIB_ERR_NONE);
}

int16_t LR20xx::transmit(uint8_t* data, size_t len, uint8_t addr) {
   // set mode to standby
  int16_t state = standby();
  RADIOLIB_ASSERT(state);

  // check packet length
  if(len > RADIOLIB_LR11X0_MAX_PACKET_LENGTH) {
    return(RADIOLIB_ERR_PACKET_TOO_LONG);
  }

  // get currently active modem
  uint8_t modem = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  state = getPacketType(&modem);
  RADIOLIB_ASSERT(state);
  RadioLibTime_t timeout = getTimeOnAir(len);
  if(modem == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    // calculate timeout (150% of expected time-on-air)
    timeout = (timeout * 3) / 2;

  } else if((modem == RADIOLIB_LR20xx_PACKET_TYPE_FSK) || (modem == RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS )) {
    // calculate timeout (500% of expected time-on-air)
    timeout = timeout * 5;

  } else {
    return(RADIOLIB_ERR_UNKNOWN);
  }

  RADIOLIB_DEBUG_BASIC_PRINTLN("Timeout in %lu us", timeout);

  // start transmission
  state = startTransmit(data, len, addr);
  RADIOLIB_ASSERT(state);

  // wait for packet transmission or timeout
  RadioLibTime_t start = this->mod->hal->micros();
  while(!this->mod->hal->digitalRead(this->mod->getIrq())) {
    this->mod->hal->yield();
    if(this->mod->hal->micros() - start > timeout) {
      finishTransmit();
      return(RADIOLIB_ERR_TX_TIMEOUT);
    }
  }
  RadioLibTime_t elapsed = this->mod->hal->micros() - start;

  // update data rate
  this->dataRateMeasured = (len*8.0)/((float)elapsed/1000000.0);

  return(finishTransmit());
}

int16_t LR20xx::receive(uint8_t* data, size_t len) {
  // set mode to standby
  int16_t state = standby();
  RADIOLIB_ASSERT(state);

  RadioLibTime_t timeout = 0;

  // get currently active modem
  uint8_t modem = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  state = getPacketType(&modem);
  RADIOLIB_ASSERT(state);
  if(modem == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    // calculate timeout (100 LoRa symbols, the default for SX127x series)
    float symbolLength = (float)(uint32_t(1) << this->spreadingFactor) / (float)this->bandwidthKhz;
    timeout = (RadioLibTime_t)(symbolLength * 100.0);
  
  } else if(modem == RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    // calculate timeout (500 % of expected time-one-air)
    size_t maxLen = len;
    if(len == 0) { 
      maxLen = 0xFF;
    }
    float brBps = ((float)(RADIOLIB_LR11X0_CRYSTAL_FREQ) * 1000000.0 * 32.0) / (float)this->bitRate;
    timeout = (RadioLibTime_t)(((maxLen * 8.0) / brBps) * 1000.0 * 5.0);
  
  } else if(modem == RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS ) {
    size_t maxLen = len;
    if(len == 0) { 
      maxLen = 0xFF;
    }
    timeout = (RadioLibTime_t)(((maxLen * 8.0) / (RADIOLIB_LR11X0_LR_FHSS_BIT_RATE)) * 1000.0 * 5.0);

  } else {
    return(RADIOLIB_ERR_UNKNOWN);
  
  }

  RADIOLIB_DEBUG_BASIC_PRINTLN("Timeout in %lu ms", timeout);

  // start reception
  uint32_t timeoutValue = (uint32_t)(((float)timeout * 1000.0) / 30.52);
  state = startReceive(timeoutValue);
  RADIOLIB_ASSERT(state);

  // wait for packet reception or timeout
  bool softTimeout = false;
  RadioLibTime_t start = this->mod->hal->millis();
  while(!this->mod->hal->digitalRead(this->mod->getIrq())) {
    this->mod->hal->yield();
    // safety check, the timeout should be done by the radio
    if(this->mod->hal->millis() - start > timeout) {
      softTimeout = true;
      break;
    }
  }

  // if it was a timeout, this will return an error code
  // TODO taken from SX126x, does this really work?
  state = standby();
  if((state != RADIOLIB_ERR_NONE) && (state != RADIOLIB_ERR_SPI_CMD_TIMEOUT)) {
    return(state);
  }

  // check whether this was a timeout or not
  if((getIrqStatus() & RADIOLIB_LR20xx_IRQ_TIMEOUT) || softTimeout) {
    standby();
    clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
    return(RADIOLIB_ERR_RX_TIMEOUT);
  }

  // read the received data
  return(readData(data, len));
}

int16_t LR20xx::transmitDirect(uint32_t frf) {
  // set RF switch (if present)
  //this->mod->setRfSwitchState(Module::MODE_TX);

  // user requested to start transmitting immediately (required for RTTY)
  //int16_t state = RADIOLIB_ERR_NONE;
  /*if(frf != 0) {
    state = setRfFrequency(frf);
  }
  RADIOLIB_ASSERT(state);*/
int16_t state;
state =setRfFrequency(frf);
RADIOLIB_ASSERT(state);


  // start transmitting
  return(setTxCw(2));//0x00:normal tx mode; 0x01: infinite premable; 0x02: continue wave ;0x03:PRBS9
}

int16_t LR20xx::receiveDirect() {
  // set RF switch (if present)
  this->mod->setRfSwitchState(Module::MODE_RX);

  // LR20xx is unable to output received data directly
  return(RADIOLIB_ERR_UNKNOWN);
}

int16_t LR20xx::scanChannel() {
  return(this->scanChannel(RADIOLIB_LR11X0_CAD_PARAM_DEFAULT, RADIOLIB_LR11X0_CAD_PARAM_DEFAULT, RADIOLIB_LR11X0_CAD_PARAM_DEFAULT));
}

int16_t LR20xx::scanChannel(uint8_t symbolNum, uint8_t detPeak, uint8_t detMin) {
  // set mode to CAD
  int state = startChannelScan(symbolNum, detPeak, detMin);
  RADIOLIB_ASSERT(state);

  // wait for channel activity detected or timeout
  while(!this->mod->hal->digitalRead(this->mod->getIrq())) {
    this->mod->hal->yield();
  }

  // check CAD result
  return(getChannelScanResult());
}

int16_t LR20xx::standby() {
  return(LR20xx::standby(RADIOLIB_LR20xx_STANDBY_RC));
}

int16_t LR20xx::standby(uint8_t mode, bool wakeup) {
   //this->mod->setRfSwitchPins(RADIOLIB_NC, RADIOLIB_NC); 
  // set RF switch (if present)
  //this->mod->setRfSwitchState(Module::MODE_IDLE);
  wakeup=false;
  if(wakeup) {
    // pull NSS low for a while to wake up
    this->mod->hal->digitalWrite(this->mod->getCs(), this->mod->hal->GpioLevelLow);
    this->mod->hal->delay(1);
    this->mod->hal->digitalWrite(this->mod->getCs(), this->mod->hal->GpioLevelHigh);
  }

  uint8_t buff[1] = { mode };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_STANDBY, true, buff, 1));
}

int16_t LR20xx::sleep() {
  return(LR20xx::sleep(true, 0));
}

int16_t LR20xx::sleep(bool retainConfig, uint32_t sleepTime) {
  // set RF switch (if present)
  this->mod->setRfSwitchState(Module::MODE_IDLE);

  uint8_t buff[] = { 
    (uint8_t)retainConfig,
    (uint8_t)((sleepTime >> 24) & 0xFF), (uint8_t)((sleepTime >> 16) & 0xFF),
    (uint8_t)((sleepTime >> 16) & 0xFF), (uint8_t)(sleepTime & 0xFF),
  };
  if(sleepTime) {
    buff[0] |= RADIOLIB_LR11X0_SLEEP_WAKEUP_ENABLED;
  }

  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_SLEEP, true, buff, sizeof(buff));

  // wait for the module to safely enter sleep mode
  this->mod->hal->delay(1);

  return(state);
}

void LR20xx::setIrqAction(void (*func)(void)) {
  this->mod->hal->attachInterrupt(this->mod->hal->pinToInterrupt(this->mod->getIrq()), func, this->mod->hal->GpioInterruptRising);
}

void LR20xx::clearIrqAction() {
  this->mod->hal->detachInterrupt(this->mod->hal->pinToInterrupt(this->mod->getIrq()));
}

void LR20xx::setPacketReceivedAction(void (*func)(void)) {
  this->setIrqAction(func);
}

void LR20xx::clearPacketReceivedAction() {
  this->clearIrqAction();
}

void LR20xx::setPacketSentAction(void (*func)(void)) {
  this->setIrqAction(func);
}

void LR20xx::clearPacketSentAction() {
  this->clearIrqAction();
}

int16_t LR20xx::startTransmit(uint8_t* data, size_t len, uint8_t addr) {
  // suppress unused variable warning
  (void)addr;
  // check packet length
  if(len > RADIOLIB_LR11X0_MAX_PACKET_LENGTH) {
    return(RADIOLIB_ERR_PACKET_TOO_LONG);
  }
  // maximum packet length is decreased by 1 when address filtering is active
  if((this->addrComp != RADIOLIB_LR20xx_GFSK_ADDRESS_FILTERING_DISABLED) && (len > RADIOLIB_LR11X0_MAX_PACKET_LENGTH - 1)) {
    return(RADIOLIB_ERR_PACKET_TOO_LONG);
  }
  // set packet Length
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t modem = RADIOLIB_LR20xx_PACKET_TYPE_NONE; 
  state = getPacketType(&modem);
  //modem=RADIOLIB_LR20xx_PACKET_TYPE_FLRC;
  RADIOLIB_ASSERT(state);
  if(modem == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
       //len=8;
     this->implicitLen=len;
    state = setPacketParamsLoRa(this->preambleLengthLoRa, this->headerType, len, this->crcTypeLoRa, this->invertIQEnabled);//len
    //printf("preambleLengthLoRa:%d,headerType:%d,implicitLen:%d,crcTypeLoRa:%d,invertIQEnabled:%d\n",this->preambleLengthLoRa, this->headerType, this->implicitLen, this->crcTypeLoRa, this->invertIQEnabled);
  //printf("setPacketParamsLoRa after \n");
  } else if(modem == RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
  /*this->preambleLengthGFSK=32;
  this->preambleDetLength=RADIOLIB_LR20xx_GFSK_PREAMBLE_DETECTOR_DISABLED;
  //this->syncWordLength=
  this->addrComp=RADIOLIB_LR20xx_GFSK_ADDRESS_FILTERING_DISABLED;
  this->packetType=RADIOLIB_LR20xx_GFSK_HEADER_8BITS;
  this->crcTypeGFSK=RADIOLIB_LR20xx_GFSK_CRC_1_BYTE_INVERTED;//RADIOLIB_LR20xx_GFSK_CRC_1_BYTE_INVERTED
  this->whitening=RADIOLIB_LR20xx_GFSK_WHITENING_OFF;
    state = setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, 0, this->addrComp, this->packetType, len, this->crcTypeGFSK, this->whitening);
    printf("setPacketParamsGFSK after,state:%d \n",state);*/
  } else if(modem == RADIOLIB_LR20xx_PACKET_TYPE_FLRC)
  {
   /*this->headerType=0;
  this->implicitLen=len;//64  8 230
  this->preambleLengthFLRC=RADIOLIB_LR20xx_FLRC_PREAMBLE_LEN_24_BITS;//16
  this->crcTypeFLRC=RADIOLIB_LR20xx_FLRC_CRC_2_BYTES;
   state = setPacketParamsFLRC(this->preambleLengthFLRC,RADIOLIB_LR20xx_FLRC_SYNCWORD_LENGTH_4_BYTES, RADIOLIB_LR20xx_FLRC_RX_MATCH_SYNCWORD_1,this->headerType, this->implicitLen, this->crcTypeFLRC);*/

 /* this->headerType=1;//0

  this->implicitLen=len;

  this->preambleLengthFLRC=RADIOLIB_LR20xx_FLRC_PREAMBLE_LEN_32_BITS;

  this->crcTypeFLRC=RADIOLIB_LR20xx_FLRC_CRC_OFF;

   state = setPacketParamsFLRC(this->preambleLengthFLRC,RADIOLIB_LR20xx_FLRC_SYNCWORD_LENGTH_2_BYTES, RADIOLIB_LR20xx_FLRC_RX_MATCH_SYNCWORD_1,this->headerType, len, this->crcTypeFLRC);

  printf("setPacketParamsFLRC after \n");*/

  }else if(modem != RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS ) {

    return(RADIOLIB_ERR_UNKNOWN);

  

  }
  //state = clearIrq(RADIOLIB_LR20xx_IRQ_ALL);

  //RADIOLIB_ASSERT(state);
  //printf("clearIrq after \n");
  
  
  // set DIO mapping
  /*state = setDioIrqParams(11, (RADIOLIB_LR20xx_IRQ_TX_DONE | RADIOLIB_LR20xx_IRQ_RX_DONE | RADIOLIB_LR20xx_IRQ_TIMEOUT |RADIOLIB_LR20xx_IRQ_LORA_HEADER_ERROR |RADIOLIB_LR20xx_IRQ_LEN_ERROR | RADIOLIB_LR20xx_IRQ_CRC_ERROR));//11
  RADIOLIB_ASSERT(state);*/
  //printf("setDioIrqParams after \n");
  /*if(modem == RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS ) {
    // in LR-FHSS mode, the packet is built by the device
    // TODO add configurable grid step and device offset

    state = lrFhssBuildFrame(this->lrFhssHdrCount, this->lrFhssCr, RADIOLIB_LR11X0_LR_FHSS_GRID_STEP_FCC, true, this->lrFhssBw, this->lrFhssHopSeq, 0, data, len);
    RADIOLIB_ASSERT(state);

  } else */
  {
     state=clearTxBuffer();
     RADIOLIB_ASSERT(state);
     //printf("clearTxBuffer after \n");
     
     //state=configFifoIrq(RADIOLIB_LR20xx_FIFO_FLAG_FULL,RADIOLIB_LR20xx_FIFO_FLAG_EMPTY,32,0,0,32);
     //RADIOLIB_ASSERT(state);
     //printf("configFifoIrq after \n");
    // write packet to buffer
    state = writeBuffer8(data, len);
    RADIOLIB_ASSERT(state);

  //printf("writeBuffer8 after \n");
  }
  //state = standby();
  //printf("standby(), state:%d\n",state);//add
  //printf("RADIOLIB_LR20xx_IRQ_ALL:%08X\n",RADIOLIB_LR20xx_IRQ_ALL);
  // clear interrupt flags
  //state = clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
  //RADIOLIB_ASSERT(state);
  // printf("clearIrq after \n");
   /*uint16_t errors = 0;
   getErrors(&errors);
   printf("setTx before, device errors: 0x%X\n", errors);*/
  // set RF switch (if present)
  //this->mod->setRfSwitchState(Module::MODE_TX);
  // start transmission
  state = setTx(RADIOLIB_LR20xx_TX_TIMEOUT_NONE);////test
  //state = setTx(640000);
  //state = setTxCw(0);
  //printf("setTx(),state:%d\n",state);
  RADIOLIB_ASSERT(state);
  printf("setTx after \n");
   
   

   /*uint8_t stat1;
   uint8_t stat2;
   uint32_t irq;
   state=getStatus(&stat1,&stat2,&irq);
   printf("getStatus,state:%d,stat1:%02X,stat2:%02X\n",state,stat1,stat2);*/
 
  // wait for BUSY to go low (= PA ramp up done)
  if(this->mod->getGpio()!=RADIOLIB_NC)
  {
  while(this->mod->hal->digitalRead(this->mod->getGpio())) {
    this->mod->hal->yield();
  }
  }

  return(state);
}
#if 0
int16_t LR20xx::startTransmit(uint8_t* data, size_t len, uint8_t addr) {
  // suppress unused variable warning
  (void)addr;
  // check packet length
  if(len > RADIOLIB_LR11X0_MAX_PACKET_LENGTH) {
    return(RADIOLIB_ERR_PACKET_TOO_LONG);
  }
  // maximum packet length is decreased by 1 when address filtering is active
  if((this->addrComp != RADIOLIB_LR11X0_GFSK_ADDR_FILTER_DISABLED) && (len > RADIOLIB_LR11X0_MAX_PACKET_LENGTH - 1)) {
    return(RADIOLIB_ERR_PACKET_TOO_LONG);
  }
  // set packet Length
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t modem = RADIOLIB_LR20xx_PACKET_TYPE_NONE; 
  state = getPacketType(&modem);
  RADIOLIB_ASSERT(state);
  if(modem == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
     this->implicitLen=len;
    state = setPacketParamsLoRa(this->preambleLengthLoRa, this->headerType, len, this->crcTypeLoRa, this->invertIQEnabled);//len
    printf("preambleLengthLoRa:%d,headerType:%d,implicitLen:%d,crcTypeLoRa:%d,invertIQEnabled:%d\n",this->preambleLengthLoRa, this->headerType, this->implicitLen, this->crcTypeLoRa, this->invertIQEnabled);
  printf("setPacketParamsLoRa after \n");
  } else if(modem == RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    state = setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, this->syncWordLength, this->addrComp, this->packetType, len, this->crcTypeGFSK, this->whitening);
  
  }  else if(modem == RADIOLIB_LR20xx_PACKET_TYPE_FLRC)
  {
  this->headerType=0;
  this->implicitLen=len;
  this->preambleLengthFLRC=RADIOLIB_LR20xx_FLRC_PREAMBLE_LEN_32_BITS;
  this->crcTypeFLRC=RADIOLIB_LR20xx_FLRC_CRC_OFF;
   state = setPacketParamsFLRC(this->preambleLengthFLRC,RADIOLIB_LR20xx_FLRC_SYNCWORD_LENGTH_2_BYTES, RADIOLIB_LR20xx_FLRC_RX_MATCH_SYNCWORD_1,this->headerType, len, this->crcTypeFLRC);
  printf("setPacketParamsFLRC after \n");
  }else if(modem != RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS ) {
    return(RADIOLIB_ERR_UNKNOWN);
  
  }
 
  RADIOLIB_ASSERT(state);
  state = clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
  RADIOLIB_ASSERT(state);
  printf("clearIrq after \n");
  
  
  // set DIO mapping
  state = setDioIrqParams(11, (RADIOLIB_LR20xx_IRQ_TX_DONE | RADIOLIB_LR20xx_IRQ_RX_DONE | RADIOLIB_LR20xx_IRQ_TIMEOUT |RADIOLIB_LR20xx_IRQ_LORA_HEADER_ERROR |RADIOLIB_LR20xx_IRQ_LEN_ERROR | RADIOLIB_LR20xx_IRQ_CRC_ERROR));//11
  RADIOLIB_ASSERT(state);
  printf("setDioIrqParams after \n");
  /*if(modem == RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS ) {
    // in LR-FHSS mode, the packet is built by the device
    // TODO add configurable grid step and device offset
    state = lrFhssBuildFrame(this->lrFhssHdrCount, this->lrFhssCr, RADIOLIB_LR11X0_LR_FHSS_GRID_STEP_FCC, true, this->lrFhssBw, this->lrFhssHopSeq, 0, data, len);
    RADIOLIB_ASSERT(state);

  } else */
  {
     state=clearTxBuffer();
     RADIOLIB_ASSERT(state);
     printf("clearTxBuffer after \n");
     
     state=configFifoIrq(RADIOLIB_LR20xx_FIFO_FLAG_FULL,RADIOLIB_LR20xx_FIFO_FLAG_EMPTY,32,0,0,32);
     RADIOLIB_ASSERT(state);
     printf("configFifoIrq after \n");
    // write packet to buffer
    state = writeBuffer8(data, len);
    RADIOLIB_ASSERT(state);
  printf("writeBuffer8 after \n");
  }
  //state = standby();
  //printf("standby(), state:%d\n",state);//add
  printf("RADIOLIB_LR20xx_IRQ_ALL:%08X\n",RADIOLIB_LR20xx_IRQ_ALL);
  // clear interrupt flags
  //state = clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
  //RADIOLIB_ASSERT(state);
  // printf("clearIrq after \n");
   uint16_t errors = 0;
   getErrors(&errors);
   printf("setTx before, device errors: 0x%X\n", errors);
  // set RF switch (if present)
  //this->mod->setRfSwitchState(Module::MODE_TX);
  // start transmission
  state = setTx(RADIOLIB_LR20xx_TX_TIMEOUT_NONE);
   //state = setTx(640000);
  //state = setTxCw(0);
  //RADIOLIB_ASSERT(state);
  printf("setTx(),state:%d\n",state);
   printf("setTx after \n");
   
   
   uint8_t stat1;
   uint8_t stat2;
   uint32_t irq;
   state=getStatus(&stat1,&stat2,&irq);
   printf("getStatus,state:%d,stat1:%02X,stat2:%02X\n",state,stat1,stat2);
 
  // wait for BUSY to go low (= PA ramp up done)
  if(this->mod->getGpio()!=RADIOLIB_NC)
  {
  while(this->mod->hal->digitalRead(this->mod->getGpio())) {
    this->mod->hal->yield();
  }
  }

  return(state);
}
#endif

int16_t LR20xx::finishTransmit() {
  // clear interrupt flags
  clearIrq(RADIOLIB_LR20xx_IRQ_ALL);

  // set mode to standby to disable transmitter/RF switch
  return(standby());
}

int16_t LR20xx::startReceive() {
  return(this->startReceive(RADIOLIB_LR20xx_RX_TIMEOUT_INF, RADIOLIB_LR20xx_IRQ_RX_DONE, 0, 0));//
}
int16_t LR20xx::startReceive(uint32_t timeout, uint32_t irqFlags, uint32_t irqMask, size_t len) {
  (void)irqMask;
  (void)len;
  
  // check active modem
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t modem = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  state = getPacketType(&modem);
  RADIOLIB_ASSERT(state);
  if((modem != RADIOLIB_LR20xx_PACKET_TYPE_LORA) && 
     (modem != RADIOLIB_LR20xx_PACKET_TYPE_FSK) &&
     (modem != RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS )&&(modem != RADIOLIB_LR20xx_PACKET_TYPE_FLRC)) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // set DIO mapping
  uint32_t irq = irqFlags;
  if(timeout != RADIOLIB_LR20xx_RX_TIMEOUT_INF) {
    irq |= RADIOLIB_LR20xx_IRQ_TIMEOUT;
  }
   state = clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
  RADIOLIB_ASSERT(state);
     state=clearRxBuffer();
     RADIOLIB_ASSERT(state);
     //printf("clearRxBuffer after \n");
        
  // set implicit mode and expected len if applicable
  if((this->headerType == RADIOLIB_LR20xx_LORA_HEADER_IMPLICIT) && (modem == RADIOLIB_LR20xx_PACKET_TYPE_LORA)) {
    state = setPacketParamsLoRa(this->preambleLengthLoRa, this->headerType, this->implicitLen, this->crcTypeLoRa, this->invertIQEnabled);
    RADIOLIB_ASSERT(state);

     printf("preambleLengthLoRa:%d,headerType:%d,implicitLen:%d,crcTypeLoRa:%d,invertIQEnabled:%d\n",this->preambleLengthLoRa, this->headerType, this->implicitLen, this->crcTypeLoRa, this->invertIQEnabled);
    printf("setPacketParamsLoRa,state:%d\n",state);
  } 

  
   if(modem == RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
  /*this->preambleLengthGFSK=32;
  this->preambleDetLength=RADIOLIB_LR20xx_GFSK_PREAMBLE_DETECTOR_DISABLED;
  //this->syncWordLength=
  this->addrComp=RADIOLIB_LR20xx_GFSK_ADDRESS_FILTERING_DISABLED;
  this->packetType=RADIOLIB_LR20xx_GFSK_HEADER_8BITS;
  this->crcTypeGFSK=RADIOLIB_LR20xx_GFSK_CRC_1_BYTE_INVERTED;//RADIOLIB_LR20xx_GFSK_CRC_1_BYTE_INVERTED
  this->whitening=RADIOLIB_LR20xx_GFSK_WHITENING_OFF;
    state = setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, 0, this->addrComp, this->packetType, 255, this->crcTypeGFSK, this->whitening);
    printf("setPacketParamsGFSK after,state:%d \n",state);*/
  }
  
  
  
  
   
  state = setRx(timeout);
  return(state);
}
#if 0 
//init
int16_t LR20xx::startReceive(uint32_t timeout, uint32_t irqFlags, uint32_t irqMask, size_t len) {
  (void)irqMask;
  (void)len;
  
  // check active modem
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t modem = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  state = getPacketType(&modem);
  RADIOLIB_ASSERT(state);
  if((modem != RADIOLIB_LR20xx_PACKET_TYPE_LORA) && 
     (modem != RADIOLIB_LR20xx_PACKET_TYPE_FSK) &&
     (modem != RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS )&&(modem != RADIOLIB_LR20xx_PACKET_TYPE_FLRC)) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // set DIO mapping
  uint32_t irq = irqFlags;
  if(timeout != RADIOLIB_LR20xx_RX_TIMEOUT_INF) {
    irq |= RADIOLIB_LR20xx_IRQ_TIMEOUT;
  }
   state = clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
  RADIOLIB_ASSERT(state);
  printf("clearIrq(),state:%d\n",state);
   
  state = setDioIrqParams(11, (RADIOLIB_LR20xx_IRQ_RX_DONE | RADIOLIB_LR20xx_IRQ_TX_DONE | RADIOLIB_LR20xx_IRQ_TIMEOUT |RADIOLIB_LR20xx_IRQ_LORA_HEADER_ERROR |RADIOLIB_LR20xx_IRQ_LEN_ERROR | RADIOLIB_LR20xx_IRQ_CRC_ERROR));
  RADIOLIB_ASSERT(state);
  printf("setDioIrqParams,state:%d\n",state);

     state=clearRxBuffer();
     RADIOLIB_ASSERT(state);
     printf("clearRxBuffer after \n");
     
     
  // set implicit mode and expected len if applicable
  if((this->headerType == RADIOLIB_LR20xx_LORA_HEADER_IMPLICIT) && (modem == RADIOLIB_LR20xx_PACKET_TYPE_LORA)) {
    state = setPacketParamsLoRa(this->preambleLengthLoRa, this->headerType, this->implicitLen, this->crcTypeLoRa, this->invertIQEnabled);
    RADIOLIB_ASSERT(state);
     printf("preambleLengthLoRa:%d,headerType:%d,implicitLen:%d,crcTypeLoRa:%d,invertIQEnabled:%d\n",this->preambleLengthLoRa, this->headerType, this->implicitLen, this->crcTypeLoRa, this->invertIQEnabled);
    printf("setPacketParamsLoRa,state:%d\n",state);
  }
  //if((this->headerType == RADIOLIB_LR20xx_LORA_HEADER_IMPLICIT) && (modem == RADIOLIB_LR20xx_PACKET_TYPE_FLRC))
  if((this->headerType == 0) && (modem == RADIOLIB_LR20xx_PACKET_TYPE_FLRC))
  {
  this->headerType=0;
  this->implicitLen=9;//64
  this->preambleLengthFLRC=RADIOLIB_LR20xx_FLRC_PREAMBLE_LEN_32_BITS;
  this->crcTypeFLRC=RADIOLIB_LR20xx_FLRC_CRC_OFF;
   state = setPacketParamsFLRC(this->preambleLengthFLRC,RADIOLIB_LR20xx_FLRC_SYNCWORD_LENGTH_2_BYTES, RADIOLIB_LR20xx_FLRC_RX_MATCH_SYNCWORD_1,this->headerType, this->implicitLen, this->crcTypeFLRC);
  printf("setPacketParamsFLRC after \n");
  }
   uint16_t errors = 0;
   getErrors(&errors);
   printf("setRx before, device errors: 0x%X\n", errors);
  // set mode to receive
  state = setRx(timeout);
  printf("setRx(),state:%d\n",state);    
    if(modem == RADIOLIB_LR20xx_PACKET_TYPE_FLRC)
   { uint16_t pkt_rx=0;
     uint16_t pkt_crc_err=0;
     uint16_t pkt_len_err=0;
     state=getRxStatsFLRC(&pkt_rx,&pkt_crc_err,&pkt_len_err);
     printf("getRxStatsFLRC,state:%d,pkt_rx:%d,pkt_crc_err:%d,pkt_len_err:%d\n",state,pkt_rx,pkt_crc_err,pkt_len_err);
 }
  return(state);
}
#endif
uint32_t LR20xx::getIrqStatus() {
  // there is no dedicated "get IRQ" command, the IRQ bits are sent after the status bytes
  uint8_t buff[6] = { 0 };
  this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_0;
  mod->SPItransferStream(NULL, 0, false, NULL, buff, sizeof(buff), true);
  this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_16;
  uint32_t irq = ((uint32_t)(buff[2]) << 24) | ((uint32_t)(buff[3]) << 16) | ((uint32_t)(buff[4]) << 8) | (uint32_t)buff[5];
  return(irq);
}
uint32_t LR20xx::get_clear_IrqStatus() {
  uint8_t buff[4] = { 0};
  int16_t state=this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_CLEAR_IRQ_STATUS, false, buff, sizeof(buff));
  //printf("RADIOLIB_LR20xx_CMD_GET_CLEAR_IRQ_STATUS,state:%d\n",state);
  uint32_t irq = ((uint32_t)(buff[0]) << 24) | ((uint32_t)(buff[1]) << 16) | ((uint32_t)(buff[2]) << 8) | (uint32_t)buff[3];
  return(irq);
}

uint16_t LR20xx::getFifoIrqFlags() {
  uint8_t buff[2] = { 0};
  int16_t state=this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_FIFO_IRQ_FLAGS, false, buff, sizeof(buff));
  uint16_t irq =((uint16_t)(buff[0]) << 8) | (uint16_t)buff[1];
  return(irq);
}

int16_t LR20xx::clearFifoIrqFlags(uint8_t rx_fifo_flags,uint8_t tx_fifo_flags) {
uint8_t buff[2] = {
    rx_fifo_flags,tx_fifo_flags
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_CLEAR_FIFO_IRQ_FLAGS, true, buff, sizeof(buff)));
}
uint16_t LR20xx::get_clear_FifoIrqFlags() {
  uint8_t buff[2] = { 0};
  int16_t state=this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_CLEAR_FIFO_IRQ_FLAGS, false, buff, sizeof(buff));
  uint16_t irq =((uint16_t)(buff[0]) << 8) | (uint16_t)buff[1];
  return(irq);
}
int16_t LR20xx::readData(uint8_t* data, size_t len) {
  // check active modem
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t offset = 0;
   size_t length=len;
  // read packet data
  state = readBuffer8(data, length, offset);
  RADIOLIB_ASSERT(state);
  //printf("readBuffer8 after\n");
  // clear the Rx buffer
  state = clearRxBuffer();
  RADIOLIB_ASSERT(state);
  return(state);
}

int16_t LR20xx::startChannelScan() {
  return(this->startChannelScan(RADIOLIB_LR11X0_CAD_PARAM_DEFAULT, RADIOLIB_LR11X0_CAD_PARAM_DEFAULT, RADIOLIB_LR11X0_CAD_PARAM_DEFAULT));
}

int16_t LR20xx::startChannelScan(uint8_t symbolNum, uint8_t detPeak, uint8_t detMin) {
  // check active modem
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t modem = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  state = getPacketType(&modem);
  RADIOLIB_ASSERT(state);
  if(modem != RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // set mode to standby
  state = standby();
  RADIOLIB_ASSERT(state);

  // set RF switch (if present)
  this->mod->setRfSwitchState(Module::MODE_RX);
/*#define IRQ_MASK ( LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_TX_DONE | LR20XX_SYSTEM_IRQ_TIMEOUT | \
      LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR | LR20XX_SYSTEM_IRQ_CRC_ERROR )
      */
  // set DIO pin mapping
  state = setDioIrqParams(11, (RADIOLIB_LR20xx_IRQ_RX_DONE | RADIOLIB_LR20xx_IRQ_TX_DONE | RADIOLIB_LR20xx_IRQ_TIMEOUT |RADIOLIB_LR20xx_IRQ_LORA_HEADER_ERROR |RADIOLIB_LR20xx_IRQ_LEN_ERROR | RADIOLIB_LR20xx_IRQ_CRC_ERROR));
  RADIOLIB_ASSERT(state);

  // clear interrupt flags
  state = clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
  RADIOLIB_ASSERT(state);

  // set mode to CAD
  return(startCad(symbolNum, detPeak, detMin));
}

int16_t LR20xx::getChannelScanResult() {
  // check active modem
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t modem = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  state = getPacketType(&modem);
  RADIOLIB_ASSERT(state);
  if(modem != RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // check CAD result
  uint32_t cadResult = getIrqStatus();
  if(cadResult & RADIOLIB_LR20xx_IRQ_CAD_DETECTED) {
    // detected some LoRa activity
    return(RADIOLIB_LORA_DETECTED);
  } else if(cadResult & RADIOLIB_LR20xx_IRQ_CAD_DONE) {
    // channel is free
    return(RADIOLIB_CHANNEL_FREE);
  }

  return(RADIOLIB_ERR_UNKNOWN);
}

int16_t LR20xx::setBandwidth(float bw) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // ensure byte conversion doesn't overflow
  RADIOLIB_CHECK_RANGE(bw, 0.0, 1100.0, RADIOLIB_ERR_INVALID_BANDWIDTH);//510

  // check allowed bandwidth values
  uint16_t bw_div2 = bw / 2 + 0.01;
  switch (bw_div2)  {
    case 31: // 62.5:
      this->bandwidth = RADIOLIB_LR20xx_LORA_BW_62_5;
      break;
    case 62: // 125.0:
      this->bandwidth = RADIOLIB_LR20xx_LORA_BW_125_0;
      break;
    case 125: // 250.0
      this->bandwidth = RADIOLIB_LR20xx_LORA_BW_250_0;
      break;
    case 250: // 500.0
      this->bandwidth = RADIOLIB_LR20xx_LORA_BW_500_0;
      break;
    case 500: // 1000.0
      this->bandwidth = RADIOLIB_LR20xx_LORA_BW_1000_0;
      break;
    default:
      return(RADIOLIB_ERR_INVALID_BANDWIDTH);
  }

  // update modulation parameters
  this->bandwidthKhz = bw;
  return(setModulationParamsLoRa(this->spreadingFactor, this->bandwidth, this->codingRate, this->ldrOptimize));
}

int16_t LR20xx::setSpreadingFactor(uint8_t sf, bool legacy) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  RADIOLIB_CHECK_RANGE(sf, 5, 12, RADIOLIB_ERR_INVALID_SPREADING_FACTOR);

  // TODO enable SF6 legacy mode
  if(legacy && (sf == 6)) {
    //this->mod->SPIsetRegValue(RADIOLIB_LR11X0_REG_SF6_SX127X_COMPAT, RADIOLIB_LR11X0_SF6_SX127X, 18, 18);
  }

  // update modulation parameters
  this->spreadingFactor = sf;
  return(setModulationParamsLoRa(this->spreadingFactor, this->bandwidth, this->codingRate, this->ldrOptimize));
}

int16_t LR20xx::setCodingRate(uint8_t cr, bool longInterleave) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  RADIOLIB_CHECK_RANGE(cr, 5, 8, RADIOLIB_ERR_INVALID_CODING_RATE);

  if(longInterleave) {
    switch(cr) {
      case 5:
      case 6:
        this->codingRate = cr;
        break;
      case 8: 
        this->codingRate = cr - 1;
        break;
      default:
        return(RADIOLIB_ERR_INVALID_CODING_RATE);
    }
  
  } else {
    this->codingRate = cr - 4;
  
  }
  // update modulation parameters
  return(setModulationParamsLoRa(this->spreadingFactor, this->bandwidth, this->codingRate, this->ldrOptimize));
}

int16_t LR20xx::setSyncWord(uint32_t syncWord) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    return(setLoRaSyncWord(syncWord & 0xFFFF));  
  } else if(type == RADIOLIB_LR20xx_PACKET_TYPE_FSK)
  {
    //return(setLoRaSyncWord(syncWord & 0xFFFF)); 
  }
  else if(type == RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS ) {
    return(lrFhssSetSyncWord(syncWord));
 
  }
  
  return(RADIOLIB_ERR_WRONG_MODEM);
}

int16_t LR20xx::setBitRate(float br) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
    //FSK
  if((type == RADIOLIB_LR20xx_PACKET_TYPE_FSK)) {
    /*if((uint16_t)br == 125) {
      this->bitRate = RADIOLIB_LR20xx_GFSK_BR_0_260_BW_0_3;
    } else if((uint16_t)br == 250) {
      this->bitRate = RADIOLIB_SX128X_BLE_GFSK_BR_0_250_BW_0_6;
    } else if((uint16_t)br == 400) {
      this->bitRate = RADIOLIB_SX128X_BLE_GFSK_BR_0_400_BW_1_2;
    } else if((uint16_t)br == 500) {
      this->bitRate = RADIOLIB_SX128X_BLE_GFSK_BR_0_500_BW_1_2;
    } else if((uint16_t)br == 800) {
      this->bitRate = RADIOLIB_SX128X_BLE_GFSK_BR_0_800_BW_2_4;
    } else if((uint16_t)br == 1000) {
      this->bitRate = RADIOLIB_SX128X_BLE_GFSK_BR_1_000_BW_2_4;
    } else if((uint16_t)br == 1600) {
      this->bitRate = RADIOLIB_SX128X_BLE_GFSK_BR_1_600_BW_2_4;
    } else if((uint16_t)br == 2000) {
      this->bitRate = RADIOLIB_SX128X_BLE_GFSK_BR_2_000_BW_2_4;
    } else {
      return(RADIOLIB_ERR_INVALID_BIT_RATE);
    }*/
    this->bitRateKbps = (uint16_t)br;
      // set bit rate value
  // TODO implement fractional bit rate configuration
  this->bitRate = br * 1000.0;
  return(setModulationParamsGFSK(this->bitRate, this->pulseShape, this->rxBandwidth, this->frequencyDev));
  } else if(type == RADIOLIB_LR20xx_PACKET_TYPE_FLRC) { // FLRC
  RADIOLIB_CHECK_RANGE(br, 260, 2600, RADIOLIB_ERR_INVALID_BIT_RATE);
    if((uint16_t)br == 260) {
      this->bitRate = RADIOLIB_LR20xx_FLRC_BR_0_260_BW_0_3;
    } else if((uint16_t)br == 325) {
      this->bitRate = RADIOLIB_LR20xx_FLRC_BR_0_325_BW_0_3;
    } else if((uint16_t)br == 520) {
      this->bitRate = RADIOLIB_LR20xx_FLRC_BR_0_520_BW_0_6;
    } else if((uint16_t)br == 650) {
      this->bitRate = RADIOLIB_LR20xx_FLRC_BR_0_650_BW_0_6;
    } else if((uint16_t)br == 1040) {
      this->bitRate = RADIOLIB_LR20xx_FLRC_BR_1_040_BW_1_2;
    } else if((uint16_t)br == 1300) {
      this->bitRate = RADIOLIB_LR20xx_FLRC_BR_1_300_BW_1_2;
    } else if((uint16_t)br == 2080) {
      this->bitRate = RADIOLIB_LR20xx_FLRC_BR_2_080_BW_2_6;
    }else if((uint16_t)br == 2600) {
      this->bitRate = RADIOLIB_LR20xx_FLRC_BR_2_600_BW_2_6;
    }else {
      return(RADIOLIB_ERR_INVALID_BIT_RATE);
    }
    // update modulation parameters
    this->bitRateKbps = (uint16_t)br;
    return(setModulationParamsFLRC(this->bitRate, this->codingRateFLRC, this->shaping));

  }

  return(RADIOLIB_ERR_WRONG_MODEM);
  
  
  
  
  
  
  
  
  
  
}

int16_t LR20xx::setFrequencyDeviation(float freqDev) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // set frequency deviation to lowest available setting (required for digimodes)
  float newFreqDev = freqDev;
  if(freqDev < 0.0) {
    newFreqDev = 0.6;
  }

  RADIOLIB_CHECK_RANGE(newFreqDev, 0.6, 200.0, RADIOLIB_ERR_INVALID_FREQUENCY_DEVIATION);
  this->frequencyDev = newFreqDev * 1000.0;
  return(setModulationParamsGFSK(this->bitRate, this->pulseShape, this->rxBandwidth, this->frequencyDev));
}

int16_t LR20xx::setRxBandwidth(float rxBw) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // check modulation parameters
  /*if(2 * this->frequencyDev + this->bitRate > rxBw * 1000.0) {
    return(RADIOLIB_ERR_INVALID_MODULATION_PARAMETERS);
  }*/

  // check allowed receiver bandwidth values
  

  
  
  if(fabs(rxBw - 3.5) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_3_500_HZ;
  }else if(fabs(rxBw - 4.2) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_4_200_HZ;
  }else if(fabs(rxBw - 4.3) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_4_300_HZ;
  }else if(fabs(rxBw - 4.5) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_4_500_HZ;
  }else if(fabs(rxBw - 4.8) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_4_800_HZ;
  }else if(fabs(rxBw - 5.2) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_5_200_HZ;
  }else if(fabs(rxBw - 5.6) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_5_600_HZ;
  }else if(fabs(rxBw - 5.8) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_5_800_HZ;
  }else if(fabs(rxBw - 6.0) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_6_000_HZ;
  }else if(fabs(rxBw - 6.9) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_6_900_HZ;
  }else if(fabs(rxBw - 7.4) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_7_400_HZ;
  }else if(fabs(rxBw - 111) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_111_000_HZ;
  }else if(fabs(rxBw - 222) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_222_000_HZ;
  }else if(fabs(rxBw - 333) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_333_000_HZ;
  }else if(fabs(rxBw - 444) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_444_000_HZ;
  }else if(fabs(rxBw - 555) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_555_000_HZ;
  }else if(fabs(rxBw - 666) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_666_000_HZ;
  }else if(fabs(rxBw - 888) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_888_000_HZ;
  }else if(fabs(rxBw - 1111) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_1_111_000_HZ;
  }else if(fabs(rxBw - 1333) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_1_333_000_HZ;
  }else if(fabs(rxBw - 2222) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_2_222_000_HZ;
  }else if(fabs(rxBw - 2666) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_2_666_000_HZ;
  }else if(fabs(rxBw - 2857) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_2_857_000_HZ;
  }else if(fabs(rxBw - 3076) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR20xx_GFSK_RX_BW_3_076_000_HZ;
  }
  
  
  
  
  
  
  
  
  
  /*else if(fabs(rxBw - 8.0) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_8_0;
  } else if(fabs(rxBw - 9.7) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_9_7;
  } else if(fabs(rxBw - 11.7) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_11_7;
  } else if(fabs(rxBw - 14.6) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_14_6;
  } else if(fabs(rxBw - 19.5) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_19_5;
  } else if(fabs(rxBw - 23.4) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_23_4;
  } else if(fabs(rxBw - 29.3) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_29_3;
  } else if(fabs(rxBw - 39.0) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_39_0;
  } else if(fabs(rxBw - 46.9) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_46_9;
  } else if(fabs(rxBw - 58.6) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_58_6;
  } else if(fabs(rxBw - 78.2) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_78_2;
  } else if(fabs(rxBw - 93.8) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_93_8;
  } else if(fabs(rxBw - 117.3) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_117_3;
  } else if(fabs(rxBw - 156.2) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_156_2;
  } else if(fabs(rxBw - 187.2) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_187_2;
  } else if(fabs(rxBw - 234.3) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_234_3;
  } else if(fabs(rxBw - 312.0) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_312_0;
  } else if(fabs(rxBw - 373.6) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_373_6;
  } else if(fabs(rxBw - 467.0) <= 0.001) {
    this->rxBandwidth = RADIOLIB_LR11X0_GFSK_RX_BW_467_0;
  } 
  */
  else {
    return(RADIOLIB_ERR_INVALID_RX_BANDWIDTH);
  }

  // update modulation parameters
  return(setModulationParamsGFSK(this->bitRate, this->pulseShape, this->rxBandwidth, this->frequencyDev));
}

int16_t LR20xx::setSyncWord(uint8_t* syncWord, size_t len) {
  if((!syncWord) || (!len) || (len > RADIOLIB_LR11X0_GFSK_SYNC_WORD_LEN)) {
    return(RADIOLIB_ERR_INVALID_SYNC_WORD);
  }

  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    // with length set to 1 and LoRa modem active, assume it is the LoRa sync word
    if(len > 1) {
      return(RADIOLIB_ERR_INVALID_SYNC_WORD);
    }
    return(setSyncWord(syncWord[0]));

  } else if(type != RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  
  }

  // update sync word length
  this->syncWordLength = len*8;
  state = setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, this->syncWordLength, this->addrComp, this->packetType, RADIOLIB_LR11X0_MAX_PACKET_LENGTH, this->crcTypeGFSK, this->whitening);
  RADIOLIB_ASSERT(state);

  // sync word is passed most-significant byte first
  uint8_t fullSyncWord[RADIOLIB_LR11X0_GFSK_SYNC_WORD_LEN] = { 0 };
  memcpy(fullSyncWord, syncWord, len);
  return(setGfskSyncWord(fullSyncWord));
}

int16_t LR20xx::setSyncBits(uint8_t *syncWord, uint8_t bitsLen) {
  if((!syncWord) || (!bitsLen) || (bitsLen > 8*RADIOLIB_LR11X0_GFSK_SYNC_WORD_LEN)) {
    return(RADIOLIB_ERR_INVALID_SYNC_WORD);
  }

  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  uint8_t bytesLen = bitsLen / 8;
  if ((bitsLen % 8) != 0) {
    bytesLen++;
  }

  return(setSyncWord(syncWord, bytesLen));
}

int16_t LR20xx::setNodeAddress(uint8_t nodeAddr) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // enable address filtering (node only)
  this->addrComp = RADIOLIB_LR11X0_GFSK_ADDR_FILTER_NODE;
  state = setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, this->syncWordLength, this->addrComp, this->packetType, RADIOLIB_LR11X0_MAX_PACKET_LENGTH, this->crcTypeGFSK, this->whitening);
  RADIOLIB_ASSERT(state);
  
  // set node address
  this->node = nodeAddr;
  return(setPacketAdrs(this->node, 0));
}

int16_t LR20xx::setBroadcastAddress(uint8_t broadAddr) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // enable address filtering (node and broadcast)
  this->addrComp = RADIOLIB_LR11X0_GFSK_ADDR_FILTER_NODE_BROADCAST;
  state = setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, this->syncWordLength, this->addrComp, this->packetType, RADIOLIB_LR11X0_MAX_PACKET_LENGTH, this->crcTypeGFSK, this->whitening);
  RADIOLIB_ASSERT(state);
  
  // set node and broadcast address
  return(setPacketAdrs(this->node, broadAddr));
}

int16_t LR20xx::disableAddressFiltering() {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // disable address filterin
  this->addrComp = RADIOLIB_LR11X0_GFSK_ADDR_FILTER_DISABLED;
  return(setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, this->syncWordLength, this->addrComp, this->packetType, RADIOLIB_LR11X0_MAX_PACKET_LENGTH, this->crcTypeGFSK, this->whitening));
}

int16_t LR20xx::setDataShaping(uint8_t sh) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // set data shaping
  /*switch(sh) {
    case RADIOLIB_SHAPING_NONE:
      this->pulseShape = RADIOLIB_LR11X0_GFSK_SHAPING_NONE;
      break;
    case RADIOLIB_SHAPING_0_3:
      this->pulseShape = RADIOLIB_LR11X0_GFSK_SHAPING_GAUSSIAN_BT_0_3;
      break;
    case RADIOLIB_SHAPING_0_5:
      this->pulseShape = RADIOLIB_LR11X0_GFSK_SHAPING_GAUSSIAN_BT_0_5;
      break;
    case RADIOLIB_SHAPING_0_7:
      this->pulseShape = RADIOLIB_LR11X0_GFSK_SHAPING_GAUSSIAN_BT_0_7;
      break;
    case RADIOLIB_SHAPING_1_0:
      this->pulseShape = RADIOLIB_LR11X0_GFSK_SHAPING_GAUSSIAN_BT_1_0;
      break;
    default:
      return(RADIOLIB_ERR_INVALID_DATA_SHAPING);
  }*/
    this->pulseShape=sh;
  // update modulation parameters
  return(setModulationParamsGFSK(this->bitRate, this->pulseShape, this->rxBandwidth, this->frequencyDev));
}

int16_t LR20xx::setEncoding(uint8_t encoding) {
  return(setWhitening(encoding));
}

int16_t LR20xx::fixedPacketLengthMode(uint8_t len) {
  return(setPacketMode(RADIOLIB_LR11X0_GFSK_PACKET_LENGTH_FIXED, len));
}

int16_t LR20xx::variablePacketLengthMode(uint8_t maxLen) {
  return(setPacketMode(RADIOLIB_LR11X0_GFSK_PACKET_LENGTH_VARIABLE, maxLen));
}

int16_t LR20xx::setWhitening(bool enabled, uint16_t initial) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  if(!enabled) {
    // disable whitening
    this->whitening = RADIOLIB_LR11X0_GFSK_WHITENING_DISABLED;

  } else {
    // enable whitening
    this->whitening = RADIOLIB_LR11X0_GFSK_WHITENING_ENABLED;

    // write initial whitening value
    state = setGfskWhitParams(initial);
    RADIOLIB_ASSERT(state);
  }

  return(setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, this->syncWordLength, this->addrComp, this->packetType, RADIOLIB_LR11X0_MAX_PACKET_LENGTH, this->crcTypeGFSK, this->whitening));
}

int16_t LR20xx::setDataRate(DataRate_t dr) {
  // select interpretation based on active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  
  if(type == RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    // set the bit rate
    state = this->setBitRate(dr.fsk.bitRate);
    RADIOLIB_ASSERT(state);

    // set the frequency deviation
    state = this->setFrequencyDeviation(dr.fsk.freqDev);

  } else if(type == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    // set the spreading factor
    state = this->setSpreadingFactor(dr.lora.spreadingFactor);
    RADIOLIB_ASSERT(state);

    // set the bandwidth
    state = this->setBandwidth(dr.lora.bandwidth);
    RADIOLIB_ASSERT(state);

    // set the coding rate
    state = this->setCodingRate(dr.lora.codingRate);
  
  } else if(type == RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS ) {

  
  }

  return(state);
}

int16_t LR20xx::checkDataRate(DataRate_t dr) {
  // select interpretation based on active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);

  if(type == RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    RADIOLIB_CHECK_RANGE(dr.fsk.bitRate, 0.6, 300.0, RADIOLIB_ERR_INVALID_BIT_RATE);
    RADIOLIB_CHECK_RANGE(dr.fsk.freqDev, 0.6, 200.0, RADIOLIB_ERR_INVALID_FREQUENCY_DEVIATION);
    return(RADIOLIB_ERR_NONE);

  } else if(type == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    RADIOLIB_CHECK_RANGE(dr.lora.spreadingFactor, 5, 12, RADIOLIB_ERR_INVALID_SPREADING_FACTOR);
    RADIOLIB_CHECK_RANGE(dr.lora.bandwidth, 0.0, 510.0, RADIOLIB_ERR_INVALID_BANDWIDTH);
    RADIOLIB_CHECK_RANGE(dr.lora.codingRate, 5, 8, RADIOLIB_ERR_INVALID_CODING_RATE);
    return(RADIOLIB_ERR_NONE);
  
  }

  return(RADIOLIB_ERR_UNKNOWN);
}

int16_t LR20xx::setPreambleLength(size_t preambleLength) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    this->preambleLengthLoRa = preambleLength;
    return(setPacketParamsLoRa(this->preambleLengthLoRa, this->headerType,  this->implicitLen, this->crcTypeLoRa, (uint8_t)this->invertIQEnabled));
  } else if(type == RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    this->preambleLengthGFSK = preambleLength;
    this->preambleDetLength = RADIOLIB_LR11X0_GFSK_PREAMBLE_DETECT_16_BITS;
    
    return(setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, this->syncWordLength, this->addrComp, this->packetType, RADIOLIB_LR11X0_MAX_PACKET_LENGTH, this->crcTypeGFSK, this->whitening));
  }

  return(RADIOLIB_ERR_WRONG_MODEM);
}

int16_t LR20xx::setTCXO(float voltage, uint32_t delay) {
  // check if TCXO is enabled at all
  if(this->XTAL) {
    return(RADIOLIB_ERR_INVALID_TCXO_VOLTAGE);
  }

  // set mode to standby
  standby();

  // check RADIOLIB_LR11X0_ERROR_STAT_HF_XOSC_START_ERR flag and clear it
  uint16_t errors = 0;
  int16_t state = getErrors(&errors);
  RADIOLIB_ASSERT(state);
  if(errors & RADIOLIB_LR11X0_ERROR_STAT_HF_XOSC_START_ERR) {
    clearErrors();
  }

  // check 0 V disable
  if(fabs(voltage - 0.0) <= 0.001) {
    setTcxoMode(0, 0);
    return(reset());
  }

  // check allowed voltage values
  uint8_t tune = 0;
  if(fabs(voltage - 1.6) <= 0.001) {
    tune = RADIOLIB_LR20xx_TCXO_VOLTAGE_1_6;
  } else if(fabs(voltage - 1.7) <= 0.001) {
    tune = RADIOLIB_LR20xx_TCXO_VOLTAGE_1_7;
  } else if(fabs(voltage - 1.8) <= 0.001) {
    tune = RADIOLIB_LR20xx_TCXO_VOLTAGE_1_8;
  } else if(fabs(voltage - 2.2) <= 0.001) {
    tune = RADIOLIB_LR20xx_TCXO_VOLTAGE_2_2;
  } else if(fabs(voltage - 2.4) <= 0.001) {
    tune = RADIOLIB_LR20xx_TCXO_VOLTAGE_2_4;
  } else if(fabs(voltage - 2.7) <= 0.001) {
    tune = RADIOLIB_LR20xx_TCXO_VOLTAGE_2_7;
  } else if(fabs(voltage - 3.0) <= 0.001) {
    tune = RADIOLIB_LR20xx_TCXO_VOLTAGE_3_0;
  } else if(fabs(voltage - 3.3) <= 0.001) {
    tune = RADIOLIB_LR20xx_TCXO_VOLTAGE_3_3;
  } else {
    return(RADIOLIB_ERR_INVALID_TCXO_VOLTAGE);
  }

  // calculate delay value
  uint32_t delayValue = (uint32_t)((float)delay / 30.52f);
  if(delayValue == 0) {
    delayValue = 1;
  }
 printf("start setTCXO mode\n");
  // enable TCXO control
  return(setTcxoMode(tune, delayValue));
}

int16_t LR20xx::setCRC(uint8_t len, uint32_t initial, uint32_t polynomial, bool inverted) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    // LoRa CRC doesn't allow to set CRC polynomial, initial value, or inversion
    this->crcTypeLoRa = len > 0 ? RADIOLIB_LR20xx_LORA_CRC_ENABLED : RADIOLIB_LR20xx_LORA_CRC_DISABLED;
    state = setPacketParamsLoRa(this->preambleLengthLoRa, this->headerType, this->implicitLen, this->crcTypeLoRa, (uint8_t)this->invertIQEnabled);
  
  } else if(type == RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    // update packet parameters
    switch(len) {
      case 0:
        this->crcTypeGFSK = RADIOLIB_LR11X0_GFSK_CRC_DISABLED;
        break;
      case 1:
        if(inverted) {
          this->crcTypeGFSK = RADIOLIB_LR11X0_GFSK_CRC_1_BYTE_INV;
        } else {
          this->crcTypeGFSK = RADIOLIB_LR11X0_GFSK_CRC_1_BYTE;
        }
        break;
      case 2:
        if(inverted) {
          this->crcTypeGFSK = RADIOLIB_LR11X0_GFSK_CRC_2_BYTE_INV;
        } else {
          this->crcTypeGFSK = RADIOLIB_LR11X0_GFSK_CRC_2_BYTE;
        }
        break;
      default:
        return(RADIOLIB_ERR_INVALID_CRC_CONFIGURATION);
    }

    state = setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, this->syncWordLength, this->addrComp, this->packetType, RADIOLIB_LR11X0_MAX_PACKET_LENGTH, this->crcTypeGFSK, this->whitening);
    RADIOLIB_ASSERT(state);

    state = setGfskCrcParams(initial, polynomial);
  
  }

  return(state);
}

int16_t LR20xx::invertIQ(bool enable) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  this->invertIQEnabled = enable;
  return(setPacketParamsLoRa(this->preambleLengthLoRa, this->headerType, this->implicitLen, this->crcTypeLoRa, (uint8_t)this->invertIQEnabled));
}

float LR20xx::getRSSI() {
  float val = 0;
  bool packet=true;
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  (void)getPacketType(&type);
  if(type == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
  
  	if(packet)
  	{
          (void)getPacketStatusLoRa(&val, NULL, NULL);
        }
        else
        {
        getRssiInst(&val);
        }

  } else if(type == RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    (void)getPacketStatusGFSK(NULL, &val, NULL, NULL);
  
  }else if(type == RADIOLIB_LR20xx_PACKET_TYPE_FLRC)
  { 
  (void)getPacketStatusFLRC(NULL,&val,NULL,NULL);
  }

  return(val);
}

float LR20xx::getSNR() {
  float val = 0;

  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  (void)getPacketType(&type);
  if(type == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    (void)getPacketStatusLoRa(NULL, &val, NULL);
  }

  return(val);
}

float LR20xx::getFrequencyError() {
  // TODO implement this
  return(0);
}

size_t LR20xx::getRxFifoLevel()
{
   uint16_t fifo_level = 0;
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_CONFIG_GET_RX_FIFO_LEVEL, false, buff, sizeof(buff));
  // pass the replies
    if( state == 0 )
  {
  fifo_level = ( ( ( uint16_t ) buff[0] ) << 8 ) + ( uint16_t ) buff[1];
   //printf("fifo_level:%d\n",fifo_level);
  }
  return ((size_t)fifo_level); 

}
size_t LR20xx::getTxFifoLevel()
{
 uint16_t fifo_level = 0;
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_CONFIG_GET_TX_FIFO_LEVEL, false, buff, sizeof(buff));
  // pass the replies
    if( state == 0 )
  {
  fifo_level = ( ( ( uint16_t ) buff[0] ) << 8 ) + ( uint16_t ) buff[1];
   //printf("fifo_level:%d\n",fifo_level);
  }
  return ((size_t)fifo_level); 

}

size_t LR20xx::getPacketLength(bool update) {
  return(this->getPacketLength(update, NULL));
}

size_t LR20xx::getPacketLength(bool update, uint8_t* offset) {
  (void)update;

  // in implicit mode, return the cached value
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  (void)getPacketType(&type);
  if((type == RADIOLIB_LR20xx_PACKET_TYPE_LORA) && (this->headerType == RADIOLIB_LR20xx_LORA_HEADER_IMPLICIT)) {
    return(this->implicitLen);
  }

  uint16_t pkt_len = 0;
  
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_RX_PACKET_LENGTH, false, buff, sizeof(buff));
  //printf("RADIOLIB_LR20xx_CMD_GET_RX_PACKET_LENGTH,state:%d,buff[0]:%02X,buff[1]:%02X\n",state,buff[0],buff[1]);
  // pass the replies
    if( state == 0 )
  {
  pkt_len = ( ( ( uint16_t ) buff[0] ) << 8 ) + ( uint16_t ) buff[1];
   //printf("pkt_len:%d\n",pkt_len);
  }
 // (void)getRxBufferStatus(&len, offset);
  return((size_t)pkt_len);
}

RadioLibTime_t LR20xx::getTimeOnAir(size_t len) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  (void)getPacketType(&type);
  if(type == RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    // calculate number of symbols
    float N_symbol = 0;
    if(this->codingRate <= RADIOLIB_LR20xx_LORA_CR_4_8_SHORT) {
      // legacy coding rate - nice and simple

      // get SF coefficients
      float coeff1 = 0;
      int16_t coeff2 = 0;
      int16_t coeff3 = 0;
      if(this->spreadingFactor < 7) {
        // SF5, SF6
        coeff1 = 6.25;
        coeff2 = 4*this->spreadingFactor;
        coeff3 = 4*this->spreadingFactor;
      } else if(this->spreadingFactor < 11) {
        // SF7. SF8, SF9, SF10
        coeff1 = 4.25;
        coeff2 = 4*this->spreadingFactor + 8;
        coeff3 = 4*this->spreadingFactor;
      } else {
        // SF11, SF12
        coeff1 = 4.25;
        coeff2 = 4*this->spreadingFactor + 8;
        coeff3 = 4*(this->spreadingFactor - 2);
      }

      // get CRC length
      int16_t N_bitCRC = 16;
      if(this->crcTypeLoRa == RADIOLIB_LR20xx_LORA_CRC_DISABLED) {
        N_bitCRC = 0;
      }

      // get header length
      int16_t N_symbolHeader = 20;
      if(this->headerType == RADIOLIB_LR20xx_LORA_HEADER_IMPLICIT) {
        N_symbolHeader = 0;
      }

      // calculate number of LoRa preamble symbols
      uint32_t N_symbolPreamble = (this->preambleLengthLoRa & 0x0F) * (uint32_t(1) << ((this->preambleLengthLoRa & 0xF0) >> 4));

      // calculate the number of symbols
      N_symbol = (float)N_symbolPreamble + coeff1 + 8.0 + ceil(RADIOLIB_MAX((int16_t)(8 * len + N_bitCRC - coeff2 + N_symbolHeader), (int16_t)0) / (float)coeff3) * (float)(this->codingRate + 4);

    } else {
      // long interleaving - abandon hope all ye who enter here
      /// \todo implement this mess - SX1280 datasheet v3.0 section 7.4.4.2

    }

    // get time-on-air in us
    return(((uint32_t(1) << this->spreadingFactor) / this->bandwidthKhz) * N_symbol * 1000.0);

  } else if(type == RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(((uint32_t)len * 8 * 1000000UL) / this->bitRate);
  
  } else if(type == RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS ) {
    // calculate the number of bits based on coding rate
    uint16_t N_bits;
    switch(this->lrFhssCr) {
      case RADIOLIB_LR11X0_LR_FHSS_CR_5_6:
        N_bits = ((len * 6) + 4) / 5; // this is from the official LR11xx driver, but why the extra +4?
        break;
      case RADIOLIB_LR11X0_LR_FHSS_CR_2_3:
        N_bits = (len * 3) / 2;
        break;
      case RADIOLIB_LR11X0_LR_FHSS_CR_1_2:
        N_bits = len * 2;
        break;
      case RADIOLIB_LR11X0_LR_FHSS_CR_1_3:
        N_bits = len * 3;
        break;
      default:
        return(RADIOLIB_ERR_INVALID_CODING_RATE);
    }

    // calculate number of bits when accounting for unaligned last block
    uint16_t N_payBits = (N_bits / RADIOLIB_LR11X0_LR_FHSS_FRAG_BITS) * RADIOLIB_LR11X0_LR_FHSS_BLOCK_BITS;
    uint16_t N_lastBlockBits = N_bits % RADIOLIB_LR11X0_LR_FHSS_FRAG_BITS;
    if(N_lastBlockBits) {
      N_payBits += N_lastBlockBits + 2;
    }

    // add header bits
    uint16_t N_totalBits = (RADIOLIB_LR11X0_LR_FHSS_HEADER_BITS * this->lrFhssHdrCount) + N_payBits;
    return(((uint32_t)N_totalBits * 8 * 1000000UL) / RADIOLIB_LR11X0_LR_FHSS_BIT_RATE);
  
  }

  return(0);
}

RadioLibTime_t LR20xx::calculateRxTimeout(RadioLibTime_t timeoutUs) {
  // the timeout value is given in units of 30.52 microseconds
  // the calling function should provide some extra width, as this number of units is truncated to integer
  RadioLibTime_t timeout = timeoutUs / 30.52;
  return(timeout);
}

int16_t LR20xx::irqRxDoneRxTimeout(uint32_t &irqFlags, uint32_t &irqMask) {
  irqFlags = (RADIOLIB_LR20xx_IRQ_RX_DONE | RADIOLIB_LR20xx_IRQ_TIMEOUT);  // flags that can appear in the IRQ register
  irqMask  = irqFlags; // on LR11x0, these are the same
  return(RADIOLIB_ERR_NONE);
}

bool LR20xx::isRxTimeout() {
  uint32_t irq = getIrqStatus();
  bool rxTimedOut = irq & RADIOLIB_LR20xx_IRQ_TIMEOUT;
  return(rxTimedOut);
}
  
uint8_t LR20xx::randomByte() {
  uint32_t num = 0;
  (void)getRandomNumber(&num);
  return((uint8_t)num);
}

int16_t LR20xx::implicitHeader(size_t len) {
  return(this->setHeaderType(RADIOLIB_LR20xx_LORA_HEADER_IMPLICIT, len));
}

int16_t LR20xx::explicitHeader() {
  return(this->setHeaderType(RADIOLIB_LR20xx_LORA_HEADER_EXPLICIT));
}

float LR20xx::getDataRate() const {
  return(this->dataRateMeasured);
}

int16_t LR20xx::setRegulatorLDO() {
  return(this->setRegMode(RADIOLIB_LR20xx_REG_MODE_SIMO_OFF));
}

int16_t LR20xx::setRegulatorDCDC() {
  return(this->setRegMode(RADIOLIB_LR20xx_REG_MODE_SIMO_TX_ONLY));
}

int16_t LR20xx::setRxBoostedGainMode(bool en) {
  uint8_t buff[1] = { (uint8_t)en };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_RX_BOOSTED, true, buff, sizeof(buff)));
}

void LR20xx::setRfSwitchTable(const uint32_t (&pins)[Module::RFSWITCH_MAX_PINS], const Module::RfSwitchMode_t table[]) {
  // find which pins are used
  uint8_t enable = 0;
  for(size_t i = 0; i < Module::RFSWITCH_MAX_PINS; i++) {
    if((pins[i] == RADIOLIB_NC) || (pins[i] > RADIOLIB_LR11X0_DIO10)) {
      continue;
    }
    enable |= 1UL << pins[i];
  }

  // now get the configuration
  uint8_t modes[7] = { 0 };
  for(size_t i = 0; i < 7; i++) {
    for(size_t j = 0; j < Module::RFSWITCH_MAX_PINS; j++) {
      modes[i] |= (table[i].values[j] > 0) ? (1UL << j) : 0;
    }
  }

  // set it
  //this->setDioAsRfSwitch(enable, modes[0], modes[1], modes[2], modes[3], modes[4], modes[5], modes[6]);
}

int16_t LR20xx::setLrFhssConfig(uint8_t bw, uint8_t cr, uint8_t hdrCount, uint16_t hopSeed) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_LRFHSS ) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // check and cache all parameters
  RADIOLIB_CHECK_RANGE((int8_t)cr, (int8_t)RADIOLIB_LR11X0_LR_FHSS_CR_5_6, (int8_t)RADIOLIB_LR11X0_LR_FHSS_CR_1_3, RADIOLIB_ERR_INVALID_CODING_RATE);
  this->lrFhssCr = cr;
  RADIOLIB_CHECK_RANGE((int8_t)bw, (int8_t)RADIOLIB_LR11X0_LR_FHSS_BW_39_06, (int8_t)RADIOLIB_LR11X0_LR_FHSS_BW_1574_2, RADIOLIB_ERR_INVALID_BANDWIDTH);
  this->lrFhssBw = bw;
  RADIOLIB_CHECK_RANGE(hdrCount, 1, 4, RADIOLIB_ERR_INVALID_BIT_RANGE);
  this->lrFhssHdrCount = hdrCount;
  RADIOLIB_CHECK_RANGE((int16_t)hopSeed, (int16_t)0x000, (int16_t)0x1FF, RADIOLIB_ERR_INVALID_DATA_SHAPING);
  this->lrFhssHopSeq = hopSeed;
  return(RADIOLIB_ERR_NONE);
}

int16_t LR20xx::startWifiScan(char wifiType, uint8_t mode, uint16_t chanMask, uint8_t numScans, uint16_t timeout) {
  uint8_t type;
  switch(wifiType) {
    case('b'):
      type = RADIOLIB_LR11X0_WIFI_SCAN_802_11_B;
      break;
    case('g'):
      type = RADIOLIB_LR11X0_WIFI_SCAN_802_11_G;
      break;
    case('n'):
      type = RADIOLIB_LR11X0_WIFI_SCAN_802_11_N;
      break;
    case('*'):
      type = RADIOLIB_LR11X0_WIFI_SCAN_ALL;
      break;
    default:
      return(RADIOLIB_ERR_INVALID_WIFI_TYPE);
  }

  // go to standby
  int16_t state = standby();
  RADIOLIB_ASSERT(state);

  // reset cumulative timings
  state = wifiResetCumulTimings();
  RADIOLIB_ASSERT(state);

  // set DIO mapping
  //state = setDioIrqParams(RADIOLIB_LR11X0_IRQ_WIFI_DONE, 0);//by lv
  //RADIOLIB_ASSERT(state);

  // start scan with the maximum number of results and abort on timeout
  this->wifiScanMode = mode;
  state = wifiScan(type, chanMask, this->wifiScanMode, RADIOLIB_LR11X0_WIFI_MAX_NUM_RESULTS, numScans, timeout, RADIOLIB_LR11X0_WIFI_ABORT_ON_TIMEOUT_ENABLED);
  return(state);
}

void LR20xx::setWiFiScanAction(void (*func)(void)) {
  this->setIrqAction(func);
}

void LR20xx::clearWiFiScanAction() {
  this->clearIrqAction();
}

int16_t LR20xx::getWifiScanResultsCount(uint8_t* count) {
  // clear IRQ first, as this is likely to be called right after scan has finished
  int16_t state = clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
  RADIOLIB_ASSERT(state);

  uint8_t buff[1] = { 0 };
  state = this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_GET_NB_RESULTS, false, buff, sizeof(buff));

  // pass the replies
  if(count) { *count = buff[0]; }

  return(state);
}

int16_t LR20xx::getWifiScanResult(LR11x0WifiResult_t* result, uint8_t index, bool brief) {
  if(!result) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }

  // read a single result
  uint8_t format = brief ? RADIOLIB_LR11X0_WIFI_RESULT_TYPE_BASIC : RADIOLIB_LR11X0_WIFI_RESULT_TYPE_COMPLETE;
  uint8_t raw[RADIOLIB_LR11X0_WIFI_RESULT_MAX_LEN] = { 0 };
  int16_t state = wifiReadResults(index, 1, format, raw);
  RADIOLIB_ASSERT(state);

  // parse the information
  switch(raw[0] & 0x03) {
    case(RADIOLIB_LR11X0_WIFI_SCAN_802_11_B):
      result->type = 'b';
      break;
    case(RADIOLIB_LR11X0_WIFI_SCAN_802_11_G):
      result->type = 'g';
      break;
    case(RADIOLIB_LR11X0_WIFI_SCAN_802_11_N):
      result->type = 'n';
      break;
  }
  result->dataRateId = (raw[0] & 0xFC) >> 2;
  result->channelFreq = 2407 + (raw[1] & 0x0F)*5;
  result->origin = (raw[1] & 0x30) >> 4;
  result->ap = (raw[1] & 0x40) != 0;
  result->rssi = (float)raw[2] / -2.0f;;
  memcpy(result->mac, &raw[3], RADIOLIB_LR11X0_WIFI_RESULT_MAC_LEN);

  if(!brief) {
    if(this->wifiScanMode == RADIOLIB_LR11X0_WIFI_ACQ_MODE_FULL_BEACON) {
      LR11x0WifiResultExtended_t* resultExtended = reinterpret_cast<LR11x0WifiResultExtended_t*>(result);
      resultExtended->rate = raw[3];
      resultExtended->service = (((uint16_t)raw[4] << 8) | ((uint16_t)raw[5]));
      resultExtended->length = (((uint16_t)raw[6] << 8) | ((uint16_t)raw[7]));
      resultExtended->frameType = raw[9] & 0x03;
      resultExtended->frameSubType = (raw[9] & 0x3C) >> 2;
      resultExtended->toDistributionSystem = (raw[9] & 0x40) != 0;
      resultExtended->fromDistributionSystem = (raw[9] & 0x80) != 0;
      memcpy(resultExtended->mac0, &raw[10], RADIOLIB_LR11X0_WIFI_RESULT_MAC_LEN);
      memcpy(resultExtended->mac, &raw[16], RADIOLIB_LR11X0_WIFI_RESULT_MAC_LEN);
      memcpy(resultExtended->mac2, &raw[22], RADIOLIB_LR11X0_WIFI_RESULT_MAC_LEN);
      resultExtended->timestamp = (((uint64_t)raw[28] << 56) | ((uint64_t)raw[29] << 48)) | 
                                  (((uint64_t)raw[30] << 40) | ((uint64_t)raw[31] << 32)) | 
                                  (((uint64_t)raw[32] << 24) | ((uint64_t)raw[33] << 16)) | 
                                  (((uint64_t)raw[34] << 8) | (uint64_t)raw[35]);
      resultExtended->periodBeacon = (((uint16_t)raw[36] << 8) | ((uint16_t)raw[37])) * 1024UL;
      resultExtended->seqCtrl = (((uint16_t)raw[38] << 8) | ((uint16_t)raw[39]));
      memcpy(resultExtended->ssid, &raw[40], RADIOLIB_LR11X0_WIFI_RESULT_SSID_LEN);
      resultExtended->currentChannel = raw[72];
      memcpy(resultExtended->countryCode, &raw[73], 2);
      resultExtended->countryCode[2] = '\0';
      resultExtended->ioReg = raw[75];
      resultExtended->fcsCheckOk = (raw[76] != 0);
      resultExtended->phiOffset = (((uint16_t)raw[77] << 8) | ((uint16_t)raw[78]));
      return(RADIOLIB_ERR_NONE);
    }

    LR11x0WifiResultFull_t* resultFull = reinterpret_cast<LR11x0WifiResultFull_t*>(result);
    resultFull->frameType = raw[3] & 0x03;
    resultFull->frameSubType = (raw[3] & 0x3C) >> 2;
    resultFull->toDistributionSystem = (raw[3] & 0x40) != 0;
    resultFull->fromDistributionSystem = (raw[3] & 0x80) != 0;
    memcpy(resultFull->mac, &raw[4], RADIOLIB_LR11X0_WIFI_RESULT_MAC_LEN);
    resultFull->phiOffset = (((uint16_t)raw[10] << 8) | ((uint16_t)raw[11]));
    resultFull->timestamp = (((uint64_t)raw[12] << 56) | ((uint64_t)raw[13] << 48)) | 
                            (((uint64_t)raw[14] << 40) | ((uint64_t)raw[15] << 32)) | 
                            (((uint64_t)raw[16] << 24) | ((uint64_t)raw[17] << 16)) | 
                            (((uint64_t)raw[18] << 8) | (uint64_t)raw[19]);
    resultFull->periodBeacon = (((uint16_t)raw[20] << 8) | ((uint16_t)raw[21])) * 1024UL;
  }

  return(RADIOLIB_ERR_NONE);
}

int16_t LR20xx::wifiScan(uint8_t wifiType, uint8_t* count, uint8_t mode, uint16_t chanMask, uint8_t numScans, uint16_t timeout) {
  if(!count) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }

  // start scan
  RADIOLIB_DEBUG_BASIC_PRINTLN("WiFi scan start");
  int16_t state = startWifiScan(wifiType, mode, chanMask, numScans, timeout);
  RADIOLIB_ASSERT(state);

  // wait for scan finished or timeout
  RadioLibTime_t softTimeout = 30UL * 1000UL;
  RadioLibTime_t start = this->mod->hal->millis();
  while(!this->mod->hal->digitalRead(this->mod->getIrq())) {
    this->mod->hal->yield();
    if(this->mod->hal->millis() - start > softTimeout) {
      RADIOLIB_DEBUG_BASIC_PRINTLN("Timeout waiting for IRQ");
      this->standby();
      return(RADIOLIB_ERR_RX_TIMEOUT);
    }
  }
  RADIOLIB_DEBUG_BASIC_PRINTLN("WiFi scan done in %lu ms", (long unsigned int)(this->mod->hal->millis() - start));

  // read number of results
  return(getWifiScanResultsCount(count));
}

int16_t LR20xx::getVersionInfo(LR11x0VersionInfo_t* info) {
  if(!info) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }

  int16_t state = this->getVersion(&info->hardware, &info->device, &info->fwMajor, &info->fwMinor);
  RADIOLIB_ASSERT(state);
  return state;
  // LR1121 does not have GNSS and WiFi scanning
 /* if(this->chipType == RADIOLIB_LR11X0_DEVICE_LR1121) {
    info->fwMajorWiFi = 0;
    info->fwMinorWiFi = 0;
    info->fwGNSS = 0;
    info->almanacGNSS = 0;
    return(RADIOLIB_ERR_NONE);
  }

  state = this->wifiReadVersion(&info->fwMajorWiFi, &info->fwMinorWiFi);
  RADIOLIB_ASSERT(state);
  return(this->gnssReadVersion(&info->fwGNSS, &info->almanacGNSS));*/
}

int16_t LR20xx::updateFirmware(const uint32_t* image, size_t size, bool nonvolatile) {
  if(!image) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }

  // put the device to bootloader mode
  int16_t state = this->reboot(true);
  RADIOLIB_ASSERT(state);
  this->mod->hal->delay(500);

  // check we're in bootloader
  uint8_t device = 0xFF;
  state = this->getVersion(NULL, &device, NULL, NULL);
  RADIOLIB_ASSERT(state);
  if(device != RADIOLIB_LR11X0_DEVICE_BOOT) {
    RADIOLIB_DEBUG_BASIC_PRINTLN("Failed to put device to bootloader mode, %02x != %02x", (unsigned int)device, (unsigned int)RADIOLIB_LR11X0_DEVICE_BOOT);
    return(RADIOLIB_ERR_CHIP_NOT_FOUND);
  }

  // erase the image
  state = this->bootEraseFlash();
  RADIOLIB_ASSERT(state);

  // wait for BUSY to go low
  RadioLibTime_t start = this->mod->hal->millis();
  while(this->mod->hal->digitalRead(this->mod->getGpio())) {
    this->mod->hal->yield();
    if(this->mod->hal->millis() - start >= 3000) {
      RADIOLIB_DEBUG_BASIC_PRINTLN("BUSY pin timeout after erase!");
      return(RADIOLIB_ERR_SPI_CMD_TIMEOUT);
    }
  }

  // upload the new image
  const size_t maxLen = 64;
  size_t rem = size % maxLen;
  size_t numWrites = (rem == 0) ? (size / maxLen) : ((size / maxLen) + 1);
  RADIOLIB_DEBUG_BASIC_PRINTLN("Writing image in %lu chunks, last chunk size is %lu words", (unsigned long)numWrites, (unsigned long)rem);
  for(size_t i = 0; i < numWrites; i ++) {
    uint32_t offset = i * maxLen;
    uint32_t len = (i == (numWrites - 1)) ? rem : maxLen;
    RADIOLIB_DEBUG_BASIC_PRINTLN("Writing chunk %d at offset %08lx (%u words)", (int)i, (unsigned long)offset, (unsigned int)len);
    this->bootWriteFlashEncrypted(offset*sizeof(uint32_t), (uint32_t*)&image[offset], len, nonvolatile);
  }

  // kick the device from bootloader
  state = this->reset();
  RADIOLIB_ASSERT(state);

  // verify we are no longer in bootloader
  state = this->getVersion(NULL, &device, NULL, NULL);
  RADIOLIB_ASSERT(state);
  if(device == RADIOLIB_LR11X0_DEVICE_BOOT) {
    RADIOLIB_DEBUG_BASIC_PRINTLN("Failed to kick device from bootloader mode, %02x == %02x", (unsigned int)device, (unsigned int)RADIOLIB_LR11X0_DEVICE_BOOT);
    return(RADIOLIB_ERR_CHIP_NOT_FOUND);
  }

  return(state);
}

int16_t LR20xx::patch_load_pram(const uint32_t address, const uint32_t* buffer,
                                        const uint32_t length )
{
  // The pram is written by blocks of 32 words
  const uint16_t n_32_word_blocks = length / 32;
  size_t rem = length % 32;
  size_t numWrites = (rem == 0) ? n_32_word_blocks: (n_32_word_blocks + 1);
  RADIOLIB_DEBUG_BASIC_PRINTLN("Writing image in %lu chunks, last chunk size is %lu words", (unsigned long)numWrites, (unsigned long)rem);
  for(size_t i = 0; i < numWrites; i ++) {
    uint32_t offset = i * 32;
    uint32_t len = (i == (numWrites - 1)) ? rem : 32;
    RADIOLIB_DEBUG_BASIC_PRINTLN("Writing chunk %d at offset %08lx (%u words)", (int)i, (unsigned long)offset, (unsigned int)len);
    //this->bootWriteFlashEncrypted(offset*sizeof(uint32_t), (uint32_t*)&image[offset], len, nonvolatile);
    //this->writeCommon(RADIOLIB_LR11X0_CMD_BOOT_WRITE_FLASH_ENCRYPTED, offset, ((uint32_t*)&)buffer[offset], len, nonvolatile);
    int16_t state=this->writeRegMem32(address+offset,(uint32_t*)&buffer[offset], len);
    RADIOLIB_ASSERT(state);
  }

return 0;

/*
    // The pram is written by blocks of 32 words
    const uint16_t n_32_word_blocks = length / 32;
    
    uint8_t cbuffer[5]={0};
    uint8_t cdata[256]={0};
       if( length > 64 )
    {
        return -1;
    }
    // Write all blocks of 32 words
    for( uint16_t index_32_word_block = 0; index_32_word_block < n_32_word_blocks; index_32_word_block++ )
    {
        const uint32_t*       local_buffer  = buffer + ( index_32_word_block * 32 );
        const uint32_t        local_address = address + ( index_32_word_block * 32 * 4 );
        
        
        
        cbuffer[0] = ( uint8_t ) ( RADIOLIB_LR20xx_CMD_WRITE_REG_MEM >> 8 );
        cbuffer[1] = ( uint8_t ) ( RADIOLIB_LR20xx_CMD_WRITE_REG_MEM >> 0 );
        cbuffer[2] = ( uint8_t ) ( local_address >> 16 );
        cbuffer[3] = ( uint8_t ) ( local_address >> 8 );
        cbuffer[4] = ( uint8_t ) ( local_address >> 0 ); 
        for( uint16_t index = 0; index < 32; index++ )
          {
             uint8_t* cdata_local = &cdata[index * sizeof( uint32_t )];

             cdata_local[0] = ( uint8_t ) ( local_buffer[index] >> 24 );
             cdata_local[1] = ( uint8_t ) ( local_buffer[index] >> 16 );
             cdata_local[2] = ( uint8_t ) ( local_buffer[index] >> 8 );
             cdata_local[3] = ( uint8_t ) ( local_buffer[index] >> 0 );
    }
        
        const lr20xx_status_t status        = lr20xx_regmem_write_regmem32( context, local_address, local_buffer, 32 );
        if( status != LR20XX_STATUS_OK )
        {
            return status;
        }
    }

    // Check if there are remaining words to write and if so, write it in a single call
    const uint16_t n_remaining_words = length - ( n_32_word_blocks * 32 );
    if( n_remaining_words > 0 )
    {
        const lr20xx_status_t status = lr20xx_regmem_write_regmem32(
            context, address + ( n_32_word_blocks * 32 * 4 ), buffer + ( n_32_word_blocks * 32 ), n_remaining_words );
        return status;
    }
    else
    {
        return LR20XX_STATUS_OK;
    }
*/
}
int16_t LR20xx::patch_execute_spare(uint8_t command_index,
                                            const uint8_t* command_arguments, uint16_t command_n_arguments,
                                            uint8_t* command_outputs, uint16_t command_n_outputs)
{

 if( command_outputs == NULL )
    {
        return patch_execute_spare_write(command_index, command_arguments, command_n_arguments );
    }
    else
    {
        return patch_execute_spare_read(command_index, command_arguments, command_n_arguments,
                                                command_outputs, command_n_outputs );
    }
}
int16_t LR20xx::patch_execute_spare_write(uint8_t command_index,
                                                  const uint8_t* command_arguments, uint16_t command_n_arguments )
{

uint8_t buff[1] = {command_index };
 return(this->SPIcommand(RADIOLIB_LR20xx_CMD_PATCH_EXECUTE_SPARE, true, buff, sizeof(buff)));
}

int16_t LR20xx::patch_execute_spare_read(uint8_t command_index,
                                                 const uint8_t* command_arguments, uint16_t command_n_arguments,
                                                 uint8_t* command_outputs, uint16_t command_n_outputs )
{
    // cbuffer will hold two bytes of opcode, one byte of command index, and all bytes of arguments (which is
    // command_n_arguments)
    const uint16_t cbuffer_length = 3 + command_n_arguments;

    // Check that the cbuffer length will be enough. If not do nothing and return error status.
    if( cbuffer_length > 64 )
    {
        return 3;
    }

    // Prepare cbuffer by setting the two bytes of opcode and one byte of command index
    /*uint8_t cbuffer[LR20XX_PATCH_MAX_SPARE_READ_CBUFFER_SIZE] = { ( uint8_t )( LR20XX_PATCH_EXECUTE_SPARE_OC >> 8 ),
                                                                  ( uint8_t ) LR20XX_PATCH_EXECUTE_SPARE_OC,
                                                                  command_index };

    // Append the argument bytes to cbuffer
    for( uint16_t argument_index = 0; argument_index < command_n_arguments; argument_index++ )
    {
        cbuffer[3 + argument_index] = command_arguments[argument_index];
    }*/

  uint8_t buff[1] = { command_index };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_PATCH_EXECUTE_SPARE, false, buff, sizeof(buff));
    //*(command_outputs[0]) = (uint8_t)buff[0];
    return state;
}


int16_t LR20xx::isGnssScanCapable() {
  // get the version
  LR11x0VersionInfo_t version;
  int16_t state = this->getVersionInfo(&version);
  RADIOLIB_ASSERT(state);

  // check the device firmware version is sufficient
  uint16_t versionFull = ((uint16_t)version.fwMajor << 8) | (uint16_t)version.fwMinor;
  if((version.device == RADIOLIB_LR20xx_DEVICE_LR2021) && (versionFull >= 0x0401)) {
    return(RADIOLIB_ERR_NONE);
  } else if((version.device == RADIOLIB_LR20xx_DEVICE_LR2021) && (versionFull >= 0x0201)) {
    return(RADIOLIB_ERR_NONE);
  }

  return(RADIOLIB_ERR_UNSUPPORTED);
}

int16_t LR20xx::gnssScan(uint16_t* resSize) {
  // go to standby
  int16_t state = standby();
  RADIOLIB_ASSERT(state);

  // set DIO mapping
  //state = setDioIrqParams(RADIOLIB_LR11X0_IRQ_GNSS_DONE, 0);//by lv
  //RADIOLIB_ASSERT(state);

  state = this->gnssSetConstellationToUse(0x03);
  RADIOLIB_ASSERT(state);

  // set scan mode
  state = this->gnssSetMode(0x03);
  RADIOLIB_ASSERT(state);

  // start scan with high effort
  RADIOLIB_DEBUG_BASIC_PRINTLN("GNSS scan start");
  state = this->gnssPerformScan(0x01, 0x3C, 8);
  RADIOLIB_ASSERT(state);

  // wait for scan finished or timeout
  RadioLibTime_t softTimeout = 300UL * 1000UL;
  RadioLibTime_t start = this->mod->hal->millis();
  while(!this->mod->hal->digitalRead(this->mod->getIrq())) {
    this->mod->hal->yield();
    if(this->mod->hal->millis() - start > softTimeout) {
      RADIOLIB_DEBUG_BASIC_PRINTLN("Timeout waiting for IRQ");
      this->standby();
      return(RADIOLIB_ERR_RX_TIMEOUT);
    }
  }

  RADIOLIB_DEBUG_BASIC_PRINTLN("GNSS scan done in %lu ms", (long unsigned int)(this->mod->hal->millis() - start));
  
  state = this->clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
  RADIOLIB_ASSERT(state);

  int8_t status = 0;
  uint8_t info = 0;
  state = this->gnssReadDemodStatus(&status, &info);
  RADIOLIB_ASSERT(state);
  RADIOLIB_DEBUG_BASIC_PRINTLN("Demod status %d, info %02x", (int)status, (unsigned int)info);

  uint8_t fwVersion = 0;
  uint32_t almanacCrc = 0;
  uint8_t errCode = 0;
  uint8_t almUpdMask = 0;
  uint8_t freqSpace = 0;
  state = this->gnssGetContextStatus(&fwVersion, &almanacCrc, &errCode, &almUpdMask, &freqSpace);
  RADIOLIB_DEBUG_BASIC_PRINTLN("Context status fwVersion %d, almanacCrc %lx, errCode %d, almUpdMask %d, freqSpace %d", 
    (int)fwVersion, (unsigned long)almanacCrc, (int)errCode, (int)almUpdMask, (int)freqSpace);
  RADIOLIB_ASSERT(state);

  uint8_t stat[53] = { 0 };
  state = this->gnssReadAlmanacStatus(stat);
  RADIOLIB_ASSERT(state);
  //Module::hexdump(NULL, stat, 53);

  return(this->gnssGetResultSize(resSize));
}

int16_t LR20xx::getGnssScanResult(uint16_t size) {
  // read the result
  uint8_t res[256] = { 0 };
  int16_t state = this->gnssReadResults(res, size);
  RADIOLIB_ASSERT(state);
  RADIOLIB_DEBUG_BASIC_PRINTLN("Result type: %02x", (int)res[0]);
  //Module::hexdump(NULL, res, size);

  return(state);
}

int16_t LR20xx::getGnssPosition(float* lat, float* lon, bool filtered) {
  uint8_t error = 0;
  uint8_t nbSvUsed = 0;
  uint16_t accuracy = 0;
  int16_t state;
  if(filtered) {
    state = this->gnssReadDopplerSolverRes(&error, &nbSvUsed, NULL, NULL, NULL, NULL, lat, lon, &accuracy, NULL);
  } else {
    state = this->gnssReadDopplerSolverRes(&error, &nbSvUsed, lat, lon, &accuracy, NULL, NULL, NULL, NULL, NULL);
  }
  RADIOLIB_ASSERT(state);
  RADIOLIB_DEBUG_BASIC_PRINTLN("Solver error %d, nbSvUsed %d, accuracy = %u", (int)error, (int)nbSvUsed, (unsigned int)accuracy);

  return(state);
}

int16_t LR20xx::modSetup(float tcxoVoltage, uint8_t modem) {
  this->mod->init();
  this->mod->hal->pinMode(this->mod->getIrq(), this->mod->hal->GpioModeInput);
  this->mod->hal->pinMode(this->mod->getGpio(), this->mod->hal->GpioModeInput);
  this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_ADDR] = Module::BITS_32;
  this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_CMD] = Module::BITS_16;
  this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_16;
  this->mod->spiConfig.statusPos = 0;
  this->mod->spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_READ] = RADIOLIB_LR20xx_CMD_READ_REG_MEM;
  this->mod->spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_WRITE] = RADIOLIB_LR20xx_CMD_WRITE_REG_MEM;
  this->mod->spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_NOP] = RADIOLIB_LR20xx_CMD_NOP;
  this->mod->spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_STATUS] = RADIOLIB_LR20xx_CMD_GET_STATUS;
  this->mod->spiConfig.stream = true;
  this->mod->spiConfig.parseStatusCb = SPIparseStatus;
  this->mod->spiConfig.checkStatusCb = SPIcheckStatus;

  // try to find the LR20xx chip - this will also reset the module at least once
  if(!LR20xx::findChip(this->chipType)) {
    RADIOLIB_DEBUG_BASIC_PRINTLN("No LR20xx found!");
    this->mod->term();
    return(RADIOLIB_ERR_CHIP_NOT_FOUND);
  }
  RADIOLIB_DEBUG_BASIC_PRINTLN("M\tLR20xx"); 
  // set mode to standby
  //int16_t state;
  int16_t state = standby();
  printf("standby(), state:%d\n",state);
  RADIOLIB_ASSERT(state);
  // set TCXO control, if requested
  if(!this->XTAL && tcxoVoltage > 0.0) {
    state = setTCXO(tcxoVoltage,320000);//9156
    RADIOLIB_ASSERT(state);
  }
  
  
  //state=configLfClock(0x00);
  //RADIOLIB_ASSERT(state);
 // printf("configLfClock after\n");
  state=clearErrors();
  RADIOLIB_ASSERT(state);
  printf("clearErrors after\n");
  state=patch_load_pram(0x801000,pram, pram_size);
  RADIOLIB_ASSERT(state);
  printf("patch_load_pram after\n");
   uint8_t output[6] = { 0 };
   state=patch_execute_spare(2, 0, 0, output, 6 );
   printf("patch_execute_spare state:%d\n",state);
  state=clearErrors();
  RADIOLIB_ASSERT(state);
  printf("clearErrors after\n");
  
   //state=calibrate(0x6F);//0x08  6F    
   //RADIOLIB_ASSERT(state); 
   /*state=setPaConfig(1,0,6,7,16);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01   6
   RADIOLIB_ASSERT(state);
   printf("setPaConfig after\n");
   state=setPa(1);
   RADIOLIB_ASSERT(state);
   state=setRxPath(1,4);//LR20XX_RADIO_COMMON_PA_SEL_LF:0x00,LR20XX_RADIO_COMMON_PA_SEL_HF:0x01  
   RADIOLIB_ASSERT(state);
   printf("setRxPath after\n");*/
   
   state=setDioFunction(9,1,3);//11
   RADIOLIB_ASSERT(state);
    printf("setDioFunction after\n");
    if(is_switch==1)
    {
    state=setDioFunction(5,2,2);
   RADIOLIB_ASSERT(state);
    printf("setDioFunction(5,2,2) after\n");
    state=setDioFunction(6,2,0);
   RADIOLIB_ASSERT(state);
    printf("setDioFunction(6,2,0) after\n"); 
    state=setDioRfSwitchCfg(5,0,1,0,1,1);
    state=setDioRfSwitchCfg(6,1,0,1,0,0);
    }
  // configure settings not accessible by API
  return(config(modem));
}

int16_t LR20xx::SPIparseStatus(uint8_t in) {
  if((in & 0b00001110) == RADIOLIB_LR20xx_STAT_1_CMD_PERR) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  } else if((in & 0b00001110) == RADIOLIB_LR20xx_STAT_1_CMD_FAIL) {
    return(RADIOLIB_ERR_SPI_CMD_FAILED);
  } else if((in == 0x00) || (in == 0xFF)) {
    return(RADIOLIB_ERR_CHIP_NOT_FOUND);
  }
  return(RADIOLIB_ERR_NONE);
}

int16_t LR20xx::SPIcheckStatus(Module* mod) {
  // the status check command doesn't return status in the same place as other read commands,
  // but only as the first byte (as with any other command), hence LR20xx::SPIcommand can't be used
  // it also seems to ignore the actual command, and just sending in bunch of NOPs will work 
  uint8_t buff[6] = { 0 };
  mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_0;
  int16_t state = mod->SPItransferStream(NULL, 0, false, NULL, buff, sizeof(buff), true);
  mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_16;
  RADIOLIB_ASSERT(state);
  return(LR20xx::SPIparseStatus(buff[0]));
}

int16_t LR20xx::SPIcommand(uint16_t cmd, bool write, uint8_t* data, size_t len, uint8_t* out, size_t outLen) {
  int16_t state = RADIOLIB_ERR_UNKNOWN;
  if(!write) {
    // the SPI interface of LR11x0 requires two separate transactions for reading
    // send the 16-bit command
    state = this->mod->SPIwriteStream(cmd, out, outLen, true, false);
    RADIOLIB_ASSERT(state);

    // read the result without command
    this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_CMD] = Module::BITS_0;
    state = this->mod->SPIreadStream(RADIOLIB_LR20xx_CMD_NOP, data, len, true, false);
    this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_CMD] = Module::BITS_16;

  } else {
    // write is just a single transaction
    state = this->mod->SPIwriteStream(cmd, data, len, true, true);
  
  }
  
  return(state);
}

bool LR20xx::findChip(uint8_t ver) {
  uint8_t i = 0;
  bool flagFound = false;
  while((i < 10) && !flagFound) {
    // reset the module
    reset();

    // read the version
    LR11x0VersionInfo_t info;
    
    int16_t state = getVersionInfo(&info);
    //printf("getVersionInfo state:%d,hardware:0x%02x\n",state,info.hardware);
    //printf("Found LR20xx: RADIOLIB_LR20xx_CMD_GET_VERSION = 0x%02x\n", info.device);
    printf("Base FW version: %d.%d\n", (int)info.fwMajor, (int)info.fwMinor);
  
    info.device=0x00;
    if((state == RADIOLIB_ERR_NONE) && (info.device == ver)) {
      RADIOLIB_DEBUG_BASIC_PRINTLN("Found LR20xx: RADIOLIB_LR20xx_CMD_GET_VERSION = 0x%02x", info.device);
      RADIOLIB_DEBUG_BASIC_PRINTLN("Base FW version: %d.%d", (int)info.fwMajor, (int)info.fwMinor);
      if(this->chipType != RADIOLIB_LR20xx_DEVICE_LR2021) {
        RADIOLIB_DEBUG_BASIC_PRINTLN("WiFi FW version: %d.%d", (int)info.fwMajorWiFi, (int)info.fwMinorWiFi);
        RADIOLIB_DEBUG_BASIC_PRINTLN("GNSS FW version: %d.%d", (int)info.fwGNSS, (int)info.almanacGNSS);
      }
      flagFound = true;
    } else {
      RADIOLIB_DEBUG_BASIC_PRINTLN("LR20xx not found! (%d of 10 tries) RADIOLIB_LR20xx_CMD_GET_VERSION = 0x%02x", i + 1, info.device);
      RADIOLIB_DEBUG_BASIC_PRINTLN("Expected: 0x%02x", ver);
      this->mod->hal->delay(10);
      i++;
    }
  }

  return(flagFound);
}

int16_t LR20xx::config(uint8_t modem) {
  int16_t state = RADIOLIB_ERR_UNKNOWN;
       
  // set Rx/Tx fallback mode to STDBY_RC
  state = this->setRxTxFallbackMode(RADIOLIB_LR20xx_FALLBACK_MODE_STBY_RC);
  RADIOLIB_ASSERT(state);

  // clear IRQ
  state = this->clearIrq(RADIOLIB_LR20xx_IRQ_ALL);
  printf("clearIrq,state:%d\n",state);
  state |= this->setDioIrqParams(9, (RADIOLIB_LR20xx_IRQ_TX_DONE | RADIOLIB_LR20xx_IRQ_RX_DONE | RADIOLIB_LR20xx_IRQ_TIMEOUT |RADIOLIB_LR20xx_IRQ_LORA_HEADER_ERROR |RADIOLIB_LR20xx_IRQ_LEN_ERROR | RADIOLIB_LR20xx_IRQ_CRC_ERROR));//
  RADIOLIB_ASSERT(state);
  printf("setDioIrqParams:%08X\n",(RADIOLIB_LR20xx_IRQ_TX_DONE | RADIOLIB_LR20xx_IRQ_RX_DONE | RADIOLIB_LR20xx_IRQ_TIMEOUT |RADIOLIB_LR20xx_IRQ_LORA_HEADER_ERROR |RADIOLIB_LR20xx_IRQ_LEN_ERROR | RADIOLIB_LR20xx_IRQ_CRC_ERROR));
  printf("setDioIrqParams,state:%d\n",state);
  
  
  
  
  //state=setRfFrequency((uint32_t)(868*1000000.0f));
 // printf("setRfFrequency,state:%d\n",state);
  // calibrate all blocks
  //(void)this->calibrate(RADIOLIB_LR20xx_CALIBRATE_ALL);
  //state=this->calibrate_front_end(0,(uint32_t)(868*1000000.0f),0,(uint32_t)(0*1000000.0f),0,(uint32_t)(0*1000000.0f));//first :0
 // printf("calibrate_front_end,state:%d\n",state);
  // wait for calibration completion
  this->mod->hal->delay(5);
  while(this->mod->hal->digitalRead(this->mod->getGpio())) {
    this->mod->hal->yield();
  }
  
  // if something failed, show the device errors
  #if RADIOLIB_DEBUG_BASIC
  if(state != RADIOLIB_ERR_NONE) {
    // unless mode is forced to standby, device errors will be 0
    standby();
    uint16_t errors = 0;
    getErrors(&errors);
    RADIOLIB_DEBUG_BASIC_PRINTLN("Calibration failed, device errors: 0x%X", errors);
  }
  #endif 
  // set modem
  state = this->setPacketType(modem);
  return(state);
}

int16_t LR20xx::setPacketMode(uint8_t mode, uint8_t len) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_FSK) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // set requested packet mode
  state = setPacketParamsGFSK(this->preambleLengthGFSK, this->preambleDetLength, this->syncWordLength, this->addrComp, mode, len, this->crcTypeGFSK, this->whitening);
  RADIOLIB_ASSERT(state);

  // update cached value
  this->packetType = mode;
  return(state);
}

int16_t LR20xx::startCad(uint8_t symbolNum, uint8_t detPeak, uint8_t detMin) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // select CAD parameters
  // TODO the magic numbers are based on Semtech examples, this is probably suboptimal
  uint8_t num = symbolNum;
  if(num == RADIOLIB_LR11X0_CAD_PARAM_DEFAULT) {
    num = 2;
  }
  
  const uint8_t detPeakValues[8] = { 48, 48, 50, 55, 55, 59, 61, 65 };
  uint8_t peak = detPeak;
  if(peak == RADIOLIB_LR11X0_CAD_PARAM_DEFAULT) {
    peak = detPeakValues[this->spreadingFactor - 5];
  }

  uint8_t min = detMin;
  if(min == RADIOLIB_LR11X0_CAD_PARAM_DEFAULT) {
    min = 10;
  }

  // set CAD parameters
  // TODO add configurable exit mode and timeout
  state = setCadParams(num, peak, min, RADIOLIB_LR11X0_CAD_EXIT_MODE_STBY_RC, 0);
  RADIOLIB_ASSERT(state);

  // start CAD
  return(setCad());
}

int16_t LR20xx::setHeaderType(uint8_t hdrType, size_t len) {
  // check active modem
  uint8_t type = RADIOLIB_LR20xx_PACKET_TYPE_NONE;
  int16_t state = getPacketType(&type);
  RADIOLIB_ASSERT(state);
  if(type != RADIOLIB_LR20xx_PACKET_TYPE_LORA) {
    return(RADIOLIB_ERR_WRONG_MODEM);
  }

  // set requested packet mode
  state = setPacketParamsLoRa(this->preambleLengthLoRa, hdrType, len, this->crcTypeLoRa, this->invertIQEnabled);
  RADIOLIB_ASSERT(state);

  // update cached value
  this->headerType = hdrType;
  this->implicitLen = len;

  return(state);
}

Module* LR20xx::getMod() {
  return(this->mod);
}

int16_t LR20xx::writeRegMem32(uint32_t addr, uint32_t* data, size_t len) {
  // check maximum size
  if(len > (RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN/sizeof(uint32_t))) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }
  return(this->writeCommon(RADIOLIB_LR20xx_CMD_WRITE_REG_MEM, addr, data, len, false));
}

int16_t LR20xx::readRegMem32(uint32_t addr, uint32_t* data, size_t len) {
  // check maximum size
  if(len >= (RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN/sizeof(uint32_t))) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }

  // the request contains the address and length
  uint8_t reqBuff[5] = {
    (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF),
    (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF),
    (uint8_t)len,
  };

  // build buffers - later we need to ensure endians are correct, 
  // so there is probably no way to do this without copying buffers and iterating
  #if RADIOLIB_STATIC_ONLY
    uint8_t rplBuff[RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN];
  #else
    uint8_t* rplBuff = new uint8_t[len*sizeof(uint32_t)];
  #endif

  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_READ_REG_MEM, false, rplBuff, len*sizeof(uint32_t), reqBuff, sizeof(reqBuff));

  // convert endians
  if(data && (state == RADIOLIB_ERR_NONE)) {
    for(size_t i = 0; i < len; i++) {
      data[i] = ((uint32_t)rplBuff[2 + i*sizeof(uint32_t)] << 24) | ((uint32_t)rplBuff[3 + i*sizeof(uint32_t)] << 16) | ((uint32_t)rplBuff[4 + i*sizeof(uint32_t)] << 8) | (uint32_t)rplBuff[5 + i*sizeof(uint32_t)];
    }
  }

  #if !RADIOLIB_STATIC_ONLY
    delete[] rplBuff;
  #endif
  
  return(state);
}

int16_t LR20xx::writeBuffer8(uint8_t* data, size_t len) {
  // check maximum size
  if(len > RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_WRITE_BUFFER, true, data, len));
}

int16_t LR20xx::readBuffer8(uint8_t* data, size_t len, size_t offset) {
  // check maximum size
  if(len > RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }
   uint16_t cmd_read=RADIOLIB_LR20xx_CMD_READ_BUFFER;
   uint8_t cmdBuf[2]={(uint8_t)((cmd_read>>8)&0xFF),(uint8_t)(cmd_read&0xFF)};
this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_0;
 int16_t state = this->mod->SPItransferStream(cmdBuf, sizeof(cmdBuf), false, NULL, data, len, true);
 this->mod->spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_16; 
  return(state);
}

int16_t LR20xx::clearRxBuffer(void) {
  return (this->SPIcommand(RADIOLIB_LR20xx_CMD_CLEAR_RX_BUFFER, true, NULL, 0));
}
int16_t LR20xx::clearTxBuffer(void) {
  return (this->SPIcommand(RADIOLIB_LR20xx_CMD_CLEAR_TX_BUFFER, true, NULL, 0));
}

int16_t LR20xx::writeRegMemMask32(uint32_t addr, uint32_t mask, uint32_t data) {
  uint8_t buff[12] = {
    (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF),
    (uint8_t)((mask >> 24) & 0xFF), (uint8_t)((mask >> 16) & 0xFF), (uint8_t)((mask >> 8) & 0xFF), (uint8_t)(mask & 0xFF),
    (uint8_t)((data >> 24) & 0xFF), (uint8_t)((data >> 16) & 0xFF), (uint8_t)((data >> 8) & 0xFF), (uint8_t)(data & 0xFF),
  };
  return (this->SPIcommand(RADIOLIB_LR20xx_CMD_WRITE_REG_MEM_MASK, true, buff, sizeof(buff)));
}

int16_t LR20xx::getStatus(uint8_t* stat1, uint8_t* stat2, uint32_t* irq) {
  uint8_t buff[6] = { 0 };

  // the status check command doesn't return status in the same place as other read commands
  // but only as the first byte (as with any other command), hence LR20xx::SPIcommand can't be used
  // it also seems to ignore the actual command, and just sending in bunch of NOPs will work 
  int16_t state = this->mod->SPItransferStream(NULL, 0, false, NULL, buff, sizeof(buff), true);
  
  
  //int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_STATUS, false, buff, sizeof(buff));
  
  // pass the replies
  if(stat1) { *stat1 = buff[0]; }
  if(stat2) { *stat2 = buff[1]; }
  if(irq)   { *irq = ((uint32_t)(buff[2]) << 24) | ((uint32_t)(buff[3]) << 16) | ((uint32_t)(buff[4]) << 8) | (uint32_t)buff[5]; }

  return (state);
}

int16_t LR20xx::getVersion(uint8_t* hw, uint8_t* device, uint8_t* major, uint8_t* minor) {
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_VERSION, false, buff, sizeof(buff));
  if(major)   { *major = buff[0]; }
  if(minor)   { *minor = buff[1]; }

  return(state);
}

int16_t LR20xx::getErrors(uint16_t* err) {
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_ERRORS, false, buff, sizeof(buff));

  // pass the replies
  if(err) { *err = ((uint16_t)(buff[0]) << 8) | (uint16_t)buff[1];  }

  return(state);
}

int16_t LR20xx::clearErrors(void) {
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_CLEAR_ERRORS, true, NULL, 0));
}

int16_t LR20xx::calibrate(uint8_t params) {
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_CALIBRATE, true, &params, 1));
}
int16_t LR20xx::calibrate_front_end(uint8_t rx_path_hf_lf_1,uint32_t freq1,uint8_t rx_path_hf_lf_2,uint32_t freq2,uint8_t rx_path_hf_lf_3,uint32_t freq3) {

uint16_t freq1_4mhz = ( freq1 + LR20XX_RADIO_COMMON_FRONT_END_CALIBRATION_STEP_IN_HZ - 1 ) /
                                   LR20XX_RADIO_COMMON_FRONT_END_CALIBRATION_STEP_IN_HZ;
uint16_t calib_freq1_4mhz= ((rx_path_hf_lf_1==1)? 0x8000 : 0x0000 ) | freq1_4mhz;                                  

uint16_t freq2_4mhz = ( freq2 + LR20XX_RADIO_COMMON_FRONT_END_CALIBRATION_STEP_IN_HZ - 1 ) /
                                   LR20XX_RADIO_COMMON_FRONT_END_CALIBRATION_STEP_IN_HZ;
uint16_t calib_freq2_4mhz= ((rx_path_hf_lf_2==1)? 0x8000 : 0x0000 ) | freq2_4mhz; 

uint16_t freq3_4mhz = ( freq3 + LR20XX_RADIO_COMMON_FRONT_END_CALIBRATION_STEP_IN_HZ - 1 ) /
                                   LR20XX_RADIO_COMMON_FRONT_END_CALIBRATION_STEP_IN_HZ;
uint16_t calib_freq3_4mhz= ((rx_path_hf_lf_3==1)? 0x8000 : 0x0000 ) | freq3_4mhz; 
printf("calib_freq1_4mhz:0x%04X,calib_freq2_4mhz:0x%04X,calib_freq3_4mhz:0x%04X\n",calib_freq1_4mhz,calib_freq2_4mhz,calib_freq3_4mhz);
calib_freq2_4mhz=0;
calib_freq3_4mhz=0;
uint8_t buff[6] = { (uint8_t)((calib_freq1_4mhz >> 8) & 0xFF), (uint8_t)(calib_freq1_4mhz & 0xFF),
			(uint8_t)((calib_freq2_4mhz >> 8) & 0xFF), (uint8_t)(calib_freq2_4mhz & 0xFF),
			(uint8_t)((calib_freq3_4mhz >> 8) & 0xFF), (uint8_t)(calib_freq3_4mhz & 0xFF)};
printf("calib_front_end:0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n",buff[0],buff[1],buff[2],buff[3],buff[4],buff[5]);			
 return(this->SPIcommand(RADIOLIB_LR20xx_CMD_CALIBRATE_FRONT_END, true, buff, sizeof(buff)));
}
int16_t LR20xx::setRegMode(uint8_t mode) {
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_REG_MODE, true, &mode, 1));
}

/*int16_t LR20xx::calibImage(float freq1, float freq2) {
  uint8_t buff[2] = {
    (uint8_t)floor((freq1 - 1.0f) / 4.0f),
    (uint8_t)ceil((freq2 + 1.0f) / 4.0f)
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_CALIB_IMAGE, true, buff, sizeof(buff)));
}*/
int16_t LR20xx::setDioFunction(uint8_t dio, uint8_t func,uint8_t pull_drive) {
  uint8_t buff[2] = { dio, (uint8_t)((func<<4)|(pull_drive&0xF))};
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_DIO_FUNC, true, buff, sizeof(buff)));
}
/*int16_t LR20xx::setDioAsRfSwitch(uint8_t en, uint8_t stbyCfg, uint8_t rxCfg, uint8_t txCfg, uint8_t txHpCfg, uint8_t txHfCfg, uint8_t gnssCfg, uint8_t wifiCfg) {
  uint8_t buff[8] = { en, stbyCfg, rxCfg, txCfg, txHpCfg, txHfCfg, gnssCfg, wifiCfg };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_DIO_AS_RF_SWITCH, true, buff, sizeof(buff)));
}*/
int16_t LR20xx::setDioRfSwitchCfg(uint8_t dio,uint8_t tx_hf, uint8_t rx_hf, uint8_t tx_lf, uint8_t rx_lf,uint8_t stbyCfg) {
  uint8_t buff[2] = {dio,(uint8_t)(((tx_hf&0x1)<<4)|((rx_hf&0x1)<<3)|((tx_lf&0x1)<<2)|((rx_lf&0x1)<<1)|(stbyCfg&0x1)) };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_DIO_RF_SWITCH_CFG, true, buff, sizeof(buff)));
}
int16_t LR20xx::setDioIrqParams(uint8_t dio, uint32_t irq) {
  uint8_t buff[5] = {
    dio,
    (uint8_t)((irq >> 24) & 0xFF), (uint8_t)((irq >> 16) & 0xFF), (uint8_t)((irq >> 8) & 0xFF), (uint8_t)(irq & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_DIO_IRQ_PARAMS, true, buff, sizeof(buff)));
}

int16_t LR20xx::configFifoIrq(uint8_t RxFifoEnable,uint8_t TxFifoEnable, uint16_t RxHighThreshold,uint16_t TxLowThreshold,uint16_t RxLowThreshold,uint16_t TxHighThreshold) {
  uint8_t buff[10] = {
    RxFifoEnable,TxFifoEnable,(uint8_t)((RxHighThreshold >> 8) & 0xFF), (uint8_t)(RxHighThreshold & 0xFF),
    (uint8_t)((TxLowThreshold >> 8) & 0xFF), (uint8_t)(TxLowThreshold & 0xFF),(uint8_t)((RxLowThreshold >> 8) & 0xFF), (uint8_t)(RxLowThreshold & 0xFF),
    (uint8_t)((TxHighThreshold >> 8) & 0xFF), (uint8_t)(TxHighThreshold & 0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_CONFIG_FIFO_IRQ, true, buff, sizeof(buff)));
}
int16_t LR20xx::clearIrq(uint32_t irq) {
  uint8_t buff[4] = {
    (uint8_t)((irq >> 24) & 0xFF), (uint8_t)((irq >> 16) & 0xFF), (uint8_t)((irq >> 8) & 0xFF), (uint8_t)(irq & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_CLEAR_IRQ, true, buff, sizeof(buff)));
}
int16_t LR20xx::configLfClock(uint8_t setup) {
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_CONFIG_LF_CLOCK, true, &setup, 1));
}

int16_t LR20xx::setTcxoMode(uint8_t tune, uint32_t delay) {
  uint8_t buff[5] = {
    tune, (uint8_t)((delay >> 24) & 0xFF),(uint8_t)((delay >> 16) & 0xFF), (uint8_t)((delay >> 8) & 0xFF), (uint8_t)(delay & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_TCXO_MODE, true, buff, sizeof(buff)));
}

int16_t LR20xx::reboot(bool stay) {
  uint8_t buff[1] = { (uint8_t)(stay*3) };
  return(this->mod->SPIwriteStream(RADIOLIB_LR11X0_CMD_REBOOT, buff, sizeof(buff), true, false));
}

int16_t LR20xx::getVbat(float* vbat) {
  uint8_t buff[1] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_VBAT, false, buff, sizeof(buff));

  // pass the replies
  if(vbat) { *vbat = (((float)buff[0]/51.0f) - 1.0f)*1.35f; }

  return(state);
}

int16_t LR20xx::getTemp(float* temp) {
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_TEMP, false, buff, sizeof(buff));

  // pass the replies
  if(temp) {
    uint16_t raw = ((uint16_t)(buff[0]) << 8) | (uint16_t)buff[1];
    *temp = 25.0f - (1000.0f/1.7f)*(((float)raw/2047.0f)*1350.0f - 0.7295f);
  }

  return(state);
}

int16_t LR20xx::setFs(void) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_FS, true, NULL, 0));
}

int16_t LR20xx::getRandomNumber(uint32_t* rnd) {
  uint8_t buff[4] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_RANDOM_NUMBER, false, buff, sizeof(buff));

  // pass the replies
  if(rnd) { *rnd = ((uint32_t)(buff[0]) << 24) | ((uint32_t)(buff[1]) << 16) | ((uint32_t)(buff[2]) << 8) | (uint32_t)buff[3];  }

  return(state);
}

int16_t LR20xx::eraseInfoPage(void) {
  // only page 1 can be erased
  uint8_t buff[1] = { RADIOLIB_LR11X0_INFO_PAGE };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_ERASE_INFO_PAGE, true, buff, sizeof(buff)));
}

int16_t LR20xx::writeInfoPage(uint16_t addr, const uint32_t* data, size_t len) {
  // check maximum size
  if(len > (RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN/sizeof(uint32_t))) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }

  // build buffers - later we need to ensure endians are correct, 
  // so there is probably no way to do this without copying buffers and iterating
  size_t buffLen = sizeof(uint8_t) + sizeof(uint16_t) + len*sizeof(uint32_t);
  #if RADIOLIB_STATIC_ONLY
    uint8_t dataBuff[sizeof(uint8_t) + sizeof(uint16_t) + RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
  #else
    uint8_t* dataBuff = new uint8_t[buffLen];
  #endif

  // set the address
  dataBuff[0] = RADIOLIB_LR11X0_INFO_PAGE;
  dataBuff[1] = (uint8_t)((addr >> 8) & 0xFF);
  dataBuff[2] = (uint8_t)(addr & 0xFF);

  // convert endians
  for(size_t i = 0; i < len; i++) {
    dataBuff[3 + i] = (uint8_t)((data[i] >> 24) & 0xFF);
    dataBuff[4 + i] = (uint8_t)((data[i] >> 16) & 0xFF);
    dataBuff[5 + i] = (uint8_t)((data[i] >> 8) & 0xFF);
    dataBuff[6 + i] = (uint8_t)(data[i] & 0xFF);
  }

  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_WRITE_INFO_PAGE, true, dataBuff, buffLen);
  #if !RADIOLIB_STATIC_ONLY
    delete[] dataBuff;
  #endif
  return(state);
}

int16_t LR20xx::readInfoPage(uint16_t addr, uint32_t* data, size_t len) {
  // check maximum size
  if(len > (RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN/sizeof(uint32_t))) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }

  // the request contains the address and length
  uint8_t reqBuff[4] = {
    RADIOLIB_LR11X0_INFO_PAGE,
    (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF),
    (uint8_t)len,
  };

  // build buffers - later we need to ensure endians are correct, 
  // so there is probably no way to do this without copying buffers and iterating
  #if RADIOLIB_STATIC_ONLY
    uint8_t rplBuff[RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
  #else
    uint8_t* rplBuff = new uint8_t[len*sizeof(uint32_t)];
  #endif

  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_READ_INFO_PAGE, false, rplBuff, len*sizeof(uint32_t), reqBuff, sizeof(reqBuff));

  // convert endians
  if(data && (state == RADIOLIB_ERR_NONE)) {
    for(size_t i = 0; i < len; i++) {
      data[i] = ((uint32_t)rplBuff[2 + i*sizeof(uint32_t)] << 24) | ((uint32_t)rplBuff[3 + i*sizeof(uint32_t)] << 16) | ((uint32_t)rplBuff[4 + i*sizeof(uint32_t)] << 8) | (uint32_t)rplBuff[5 + i*sizeof(uint32_t)];
    }
  }
  
  #if !RADIOLIB_STATIC_ONLY
    delete[] rplBuff;
  #endif
  
  return(state);
}

int16_t LR20xx::getChipEui(uint8_t* eui) {
  if(!eui) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_CHIP_EUI, false, eui, RADIOLIB_LR11X0_EUI_LEN));
}

int16_t LR20xx::getSemtechJoinEui(uint8_t* eui) {
  if(!eui) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_SEMTECH_JOIN_EUI, false, eui, RADIOLIB_LR11X0_EUI_LEN));
}

int16_t LR20xx::deriveRootKeysAndGetPin(uint8_t* pin) {
  if(!pin) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_DERIVE_ROOT_KEYS_AND_GET_PIN, false, pin, RADIOLIB_LR11X0_PIN_LEN));
}

int16_t LR20xx::enableSpiCrc(bool en) {
  // TODO implement this
  (void)en;
  // LR11X0 CRC is gen 0xA6 (0x65 but reflected), init 0xFF, input and result reflected
  /*RadioLibCRCInstance.size = 8;
  RadioLibCRCInstance.poly = 0xA6;
  RadioLibCRCInstance.init = 0xFF;
  RadioLibCRCInstance.out = 0x00;
  RadioLibCRCInstance.refIn = true;
  RadioLibCRCInstance.refOut = true;*/
  return(RADIOLIB_ERR_UNSUPPORTED);
}

int16_t LR20xx::driveDiosInSleepMode(bool en) {
  uint8_t buff[1] = { (uint8_t)en };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_DRIVE_DIOS_IN_SLEEP_MODE, true, buff, sizeof(buff)));
}

int16_t LR20xx::resetStats(void) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_RESET_STATS, true, NULL, 0));
}

int16_t LR20xx::getStats(uint16_t* nbPktReceived, uint16_t* nbPktCrcError, uint16_t* data1, uint16_t* data2) {
  uint8_t buff[8] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_STATS, false, buff, sizeof(buff));

  // pass the replies
  if(nbPktReceived) { *nbPktReceived = ((uint16_t)(buff[0]) << 8) | (uint16_t)buff[1]; }
  if(nbPktCrcError) { *nbPktCrcError = ((uint16_t)(buff[2]) << 8) | (uint16_t)buff[3]; }
  if(data1) { *data1 = ((uint16_t)(buff[4]) << 8) | (uint16_t)buff[5]; }
  if(data2) { *data2 = ((uint16_t)(buff[6]) << 8) | (uint16_t)buff[7]; }

  return(state);
}

int16_t LR20xx::getPacketType(uint8_t* type) {
  uint8_t buff[1] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_PACKET_TYPE, false, buff, sizeof(buff));

  // pass the replies
  if(type) { *type = buff[0]; }

  return(state);
}

int16_t LR20xx::getRxBufferStatus(uint8_t* len, uint8_t* startOffset) {
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_RX_BUFFER_STATUS, false, buff, sizeof(buff));

  // pass the replies
  if(len) { *len = buff[0]; }
  if(startOffset) { *startOffset = buff[1]; }

  return(state);
}

int16_t LR20xx::getRxStatsFLRC(uint16_t* pkt_rx, uint16_t* pkt_crc_err,uint16_t* pkt_len_err) {
  uint8_t buff[6] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_RX_STATS_FLRC, false, buff, sizeof(buff));
  // pass the replies
  if(pkt_rx) { *pkt_rx = ((buff[0]<<8)|(buff[1])); }
  if(pkt_crc_err) { *pkt_crc_err = ((buff[2]<<8)|(buff[3])); }
  if(pkt_len_err) { *pkt_len_err = ((buff[4]<<8)|(buff[5])); }
  return(state);
}


int16_t LR20xx::getPacketStatusLoRa(float* rssiPkt, float* snrPkt, float* signalRssiPkt) {
  uint8_t buff[6] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_PACKET_STATUS_LORA, false, buff, sizeof(buff));
  // pass the replies
  if(rssiPkt) { *rssiPkt = (float)((buff[3]<<1)+((buff[5]&0x02)>>1)) / -2.0f; }
  if(snrPkt) { *snrPkt = (float)buff[2] / 4.0f; }
  if(signalRssiPkt) { *signalRssiPkt = (float)((buff[4]<<1)+(buff[5]&0x01))/-2.0f; }
  /*if(rssiPkt) { *rssiPkt = (float)((buff[2]<<1)+((buff[4]&0x02)>>1)) / -2.0f; }
  if(snrPkt) { *snrPkt = (float)buff[1] / 4.0f; }
  if(signalRssiPkt) { *signalRssiPkt = (float)((buff[3]<<1)+(buff[4]&0x01))/-2.0f; }*/
  return(state);
}

int16_t LR20xx::getPacketStatusGFSK(float* rssiSync, float* rssiAvg, uint8_t* rxLen, uint8_t* stat) {
  uint8_t buff[4] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_PACKET_STATUS_FSK, false, buff, sizeof(buff));

  // pass the replies
  if(rssiSync) { *rssiSync = (float)buff[0] / -2.0f; }
  if(rssiAvg) { *rssiAvg = (float)buff[1] / -2.0f; }
  if(rxLen) { *rxLen = buff[2]; }
  if(stat) { *stat = buff[3]; }

  return(state);
}
int16_t LR20xx::getPacketStatusFLRC(float* rssiSync, float* rssiAvg,uint16_t* rxLen,uint8_t* sw_num) {
  uint8_t buff[5] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_PACKET_STATUS_FLRC, false, buff, sizeof(buff));
  // pass the replies
  if(rxLen) { *rxLen = (uint16_t)(buff[0]<<8)+(uint16_t)(buff[1]); }
  if(sw_num) { *sw_num = buff[4]>>4; }
  if(rssiAvg) { *rssiAvg = (float)((buff[2]<<1)+(buff[4]&0x01)) / -2.0f; }
  if(rssiSync) { *rssiSync = (float)((buff[3]<<1)+((buff[4]>>2)&0x01))/-2.0f; }
  return(state);
}




int16_t LR20xx::getRssiInst(float* rssi) {
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_GET_RSSI_INST, false, buff, sizeof(buff));
  // pass the replies
  if(rssi) { *rssi = (float)((buff[0]<<1)+(buff[1]>>7)) / -2.0f; }

  return(state);
}

int16_t LR20xx::setGfskSyncWord(uint8_t* sync) {
  if(!sync) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_GFSK_SYNC_WORD, true, sync, RADIOLIB_LR11X0_GFSK_SYNC_WORD_LEN));
}

int16_t LR20xx::setLoRaPublicNetwork(bool pub) {
  uint8_t buff[1] = { (uint8_t)pub };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_LORA_PUBLIC_NETWORK, true, buff, sizeof(buff)));
}

int16_t LR20xx::setRx(uint32_t timeout) {
  uint8_t buff[3] = {
    (uint8_t)((timeout >> 16) & 0xFF), (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_RX, true, buff, sizeof(buff)));
}

int16_t LR20xx::setTx(uint32_t timeout) {
  uint8_t buff[3] = {
    (uint8_t)((timeout >> 16) & 0xFF), (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_TX, true, buff, sizeof(buff)));
}

int16_t LR20xx::setRfFrequency(uint32_t rfFreq) {
  uint8_t buff[4] = {
    (uint8_t)((rfFreq >> 24) & 0xFF), (uint8_t)((rfFreq >> 16) & 0xFF),
    (uint8_t)((rfFreq >> 8) & 0xFF), (uint8_t)(rfFreq & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_RF_FREQUENCY, true, buff, sizeof(buff)));
}

int16_t LR20xx::autoTxRx(uint32_t delay, uint8_t intMode, uint32_t timeout) {
  uint8_t buff[7] = {
    (uint8_t)((delay >> 16) & 0xFF), (uint8_t)((delay >> 8) & 0xFF), (uint8_t)(delay & 0xFF), intMode,
    (uint8_t)((timeout >> 16) & 0xFF), (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_AUTO_TX_RX, true, buff, sizeof(buff)));
}

int16_t LR20xx::setCadParams(uint8_t symNum, uint8_t detPeak, uint8_t detMin, uint8_t cadExitMode, uint32_t timeout) {
  uint8_t buff[7] = {
    symNum, detPeak, detMin, cadExitMode,
    (uint8_t)((timeout >> 16) & 0xFF), (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_CAD_PARAMS, true, buff, sizeof(buff)));
}

int16_t LR20xx::setPacketType(uint8_t type) {
  uint8_t buff[1] = { type };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_PACKET_TYPE, true, buff, sizeof(buff)));
}

int16_t LR20xx::setModulationParamsLoRa(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro) {
  uint8_t buff[2];
  buff[0]=((sf<<4)|(bw&0xF));
  buff[1]=((cr<<4)|(ldro&0x3));
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_MODULATION_PARAMS_LORA, true, buff, sizeof(buff)));
}

int16_t LR20xx::setModulationParamsGFSK(uint32_t br, uint8_t sh, uint8_t rxBw, uint32_t freqDev) {
  uint8_t buff[10] = { 
    (uint8_t)((br >> 24) & 0xFF), (uint8_t)((br >> 16) & 0xFF),
    (uint8_t)((br >> 8) & 0xFF), (uint8_t)(br & 0xFF), sh, rxBw,
     (uint8_t)((freqDev >> 16) & 0xFF),
    (uint8_t)((freqDev >> 8) & 0xFF), (uint8_t)(freqDev & 0xFF)
  };
  printf("br:%d,sh:%d,rxbw:%d,freqDev:%d\n",br,sh,rxBw,freqDev);
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_MODULATION_PARAMS_FSK, true, buff, sizeof(buff)));
}

int16_t LR20xx::setModulationParamsLrFhss(uint32_t br, uint8_t sh) {
  uint8_t buff[5] = { 
    (uint8_t)((br >> 24) & 0xFF), (uint8_t)((br >> 16) & 0xFF),
    (uint8_t)((br >> 8) & 0xFF), (uint8_t)(br & 0xFF), sh
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_MODULATION_PARAMS_BLE, true, buff, sizeof(buff)));
}

int16_t LR20xx::setModulationParamsSigfox(uint32_t br, uint8_t sh) {
  // same as for LR-FHSS
  return(this->setModulationParamsLrFhss(br, sh));
}

int16_t LR20xx::setModulationParamsFLRC(uint8_t BitRateBw, uint8_t cr, uint8_t pluseShape) {
  uint8_t buff[2];
  buff[0]=BitRateBw;
  buff[1]=((cr<<4)|(pluseShape&0x0F));
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_MODULATION_PARAMS_FLRC, true, buff, sizeof(buff)));
}







int16_t LR20xx::setPacketParamsLoRa(uint16_t preambleLen, uint8_t hdrType, uint8_t payloadLen, uint8_t crcType, uint8_t invertIQ) {
  uint8_t buff[4] = { 
    (uint8_t)((preambleLen >> 8) & 0xFF), (uint8_t)(preambleLen & 0xFF),
    payloadLen, (uint8_t)((hdrType<<2)|(crcType<<1)|(invertIQ<<0))
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_PACKET_PARAMS_LORA, true, buff, sizeof(buff)));
}

int16_t LR20xx::setPacketParamsGFSK(uint16_t preambleLen, uint8_t preambleDetectorLen, uint8_t payloadLen_unit, uint8_t addrCmp, uint8_t packType, uint16_t payloadLen, uint8_t crcType, uint8_t whiten) {
  uint8_t buff[7] = { 
    (uint8_t)((preambleLen >> 8) & 0xFF), (uint8_t)(preambleLen & 0xFF),
    preambleDetectorLen,(uint8_t)(((payloadLen_unit&0x3)<<4)|(((addrCmp&0x3)<<2)|(packType&0x3))), (uint8_t)((payloadLen>>8)&0xFF),(uint8_t)(payloadLen&0xFF), (uint8_t)(((crcType&0xF)<<4)|(whiten&0xF))
  };
  printf("setPacketParamsGFSK,buff[0]:%02X,buff[1]:%02X,buff[2]:%02X,buff[3]:%02X,buff[4]:%02X,buff[5]:%02X,buff[6]:%02X\n",buff[0],buff[1],buff[2],buff[3],buff[4],buff[5],buff[6]);
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_PACKET_PARAMS_FSK, true, buff, sizeof(buff)));
}

int16_t LR20xx::setPacketParamsSigfox(uint8_t payloadLen, uint16_t rampUpDelay, uint16_t rampDownDelay, uint16_t bitNum) {
  uint8_t buff[7] = { 
    payloadLen, (uint8_t)((rampUpDelay >> 8) & 0xFF), (uint8_t)(rampUpDelay & 0xFF),
    (uint8_t)((rampDownDelay >> 8) & 0xFF), (uint8_t)(rampDownDelay & 0xFF),
    (uint8_t)((bitNum >> 8) & 0xFF), (uint8_t)(bitNum & 0xFF),
  };
  //return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_PACKET_PARAMS, true, buff, sizeof(buff)));
  return 0;
}

int16_t LR20xx::setPacketParamsFLRC(uint8_t preambleLen,uint8_t sync_word_len,uint8_t match_sync_word,uint8_t hdrType, uint16_t payloadLen, uint8_t crcType) {
  uint8_t buff[4] = {
    (uint8_t)(((preambleLen&0xF)<<2)|(sync_word_len & 0x3))  , (uint8_t)(((match_sync_word&0x3)<<6)|((match_sync_word&0x7)<<3)|((hdrType&0x1)<<2)|(crcType&0x3)),
    (uint8_t)((payloadLen>>8)&0xFF), (uint8_t)(payloadLen&0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_PACKET_PARAMS_FLRC, true, buff, sizeof(buff)));
}

int16_t LR20xx::setTxParams(int8_t pwr, uint8_t ramp) {
  uint8_t buff[2] = { (uint8_t)pwr, ramp };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_TX_PARAMS, true, buff, sizeof(buff)));
}
int16_t LR20xx::setRxPath(uint8_t rx_path, uint8_t boost_level) {
  uint8_t buff[2] = { rx_path, boost_level };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_RX_PATH, true, buff, sizeof(buff)));
}

int16_t LR20xx::setPacketAdrs(uint8_t node, uint8_t broadcast) {
  uint8_t buff[2] = { node, broadcast };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_PACKET_ADRS, true, buff, sizeof(buff)));
}

int16_t LR20xx::setRxTxFallbackMode(uint8_t mode) {
  uint8_t buff[1] = { mode };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_RX_TX_FALLBACK_MODE, true, buff, sizeof(buff)));
}

int16_t LR20xx::setRxDutyCycle(uint32_t rxPeriod, uint32_t sleepPeriod, uint8_t mode) {
  uint8_t buff[7] = {
    (uint8_t)((rxPeriod >> 16) & 0xFF), (uint8_t)((rxPeriod >> 8) & 0xFF), (uint8_t)(rxPeriod & 0xFF),
    (uint8_t)((sleepPeriod >> 16) & 0xFF), (uint8_t)((sleepPeriod >> 8) & 0xFF), (uint8_t)(sleepPeriod & 0xFF),
    mode
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_RX_DUTY_CYCLE, true, buff, sizeof(buff)));
}

int16_t LR20xx::setPaConfig(uint8_t paSel, uint8_t paLfMode, uint8_t paLfDutyCycle, uint8_t paLfSlices,uint8_t paHfDutyCycle) {
  uint8_t buff[3] = { (uint8_t)((paSel<<7)|(paLfMode&0x3)), (uint8_t)((paLfDutyCycle<<4)|(paLfSlices&0xF)), (uint8_t)(paHfDutyCycle&0x1F)};        
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_PA_CONFIG, true, buff, sizeof(buff)));
}

int16_t LR20xx::setPa(uint8_t pa_sel) {
  uint8_t buff[1] = { pa_sel };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SEL_PA, true, buff, sizeof(buff)));

}


int16_t LR20xx::stopTimeoutOnPreamble(bool stop) {
  uint8_t buff[1] = { (uint8_t)stop };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_STOP_TIMEOUT_ON_PREAMBLE, true, buff, sizeof(buff)));
}

int16_t LR20xx::setCad(void) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_CAD, true, NULL, 0));
}

int16_t LR20xx::setTxCw(uint8_t mode) {
  uint8_t buff[1] = { mode };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_TX_CW, true, buff, sizeof(buff)));
}

int16_t LR20xx::setTxInfinitePreamble(void) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_TX_INFINITE_PREAMBLE, true, NULL, 0));
}

int16_t LR20xx::setLoRaSynchTimeout(uint8_t symbolNum) {
  uint8_t buff[1] = { symbolNum };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_LORA_SYNCH_TIMEOUT, true, buff, sizeof(buff)));
}

int16_t LR20xx::setRangingAddr(uint32_t addr, uint8_t checkLen) {
  uint8_t buff[5] = {
    (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF),
    (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF), checkLen
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_RANGING_ADDR, true, buff, sizeof(buff)));
}

int16_t LR20xx::setRangingReqAddr(uint32_t addr) {
  uint8_t buff[4] = {
    (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF),
    (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_RANGING_REQ_ADDR, true, buff, sizeof(buff)));
}

int16_t LR20xx::getRangingResult(uint8_t type, float* res) {
  uint8_t reqBuff[1] = { type };
  uint8_t rplBuff[4] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_RANGING_RESULT, false, rplBuff, sizeof(rplBuff), reqBuff, sizeof(reqBuff));
  RADIOLIB_ASSERT(state);

  if(res) { 
    if(type == RADIOLIB_LR11X0_RANGING_RESULT_DISTANCE) {
      uint32_t raw = ((uint32_t)(rplBuff[0]) << 24) | ((uint32_t)(rplBuff[1]) << 16) | ((uint32_t)(rplBuff[2]) << 8) | (uint32_t)rplBuff[3];
      *res = ((float)(raw*3e8))/((float)(4096*this->bandwidthKhz*1000));
    } else {
      *res = (float)rplBuff[3]/2.0f;
    }
  }

  return(state);
}

int16_t LR20xx::setRangingTxRxDelay(uint32_t delay) {
  uint8_t buff[4] = {
    (uint8_t)((delay >> 24) & 0xFF), (uint8_t)((delay >> 16) & 0xFF),
    (uint8_t)((delay >> 8) & 0xFF), (uint8_t)(delay & 0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_RANGING_TX_RX_DELAY, true, buff, sizeof(buff)));
}

int16_t LR20xx::setGfskCrcParams(uint32_t init, uint32_t poly) {
  uint8_t buff[8] = {
   (uint8_t)((poly >> 24) & 0xFF), (uint8_t)((poly >> 16) & 0xFF),
    (uint8_t)((poly >> 8) & 0xFF), (uint8_t)(poly & 0xFF),
    (uint8_t)((init >> 24) & 0xFF), (uint8_t)((init >> 16) & 0xFF),
    (uint8_t)((init >> 8) & 0xFF), (uint8_t)(init & 0xFF)   
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_GFSK_CRC_PARAMS, true, buff, sizeof(buff)));
  
}

int16_t LR20xx::setGfskWhitParams(uint16_t seed) {
  uint8_t buff[2] = {
    (uint8_t)((seed >> 8) & 0xFF), (uint8_t)(seed & 0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_GFSK_WHIT_PARAMS, true, buff, sizeof(buff)));
}

int16_t LR20xx::setRangingParameter(uint8_t symbolNum) {
  // the first byte is reserved
  uint8_t buff[2] = { 0x00, symbolNum };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_RANGING_PARAMETER, true, buff, sizeof(buff)));
}

int16_t LR20xx::setRssiCalibration(const int8_t* tune, int16_t gainOffset) {
  uint8_t buff[11] = {
    (uint8_t)((tune[0] & 0x0F) | (uint8_t)(tune[1] & 0x0F) << 4),
    (uint8_t)((tune[2] & 0x0F) | (uint8_t)(tune[3] & 0x0F) << 4),
    (uint8_t)((tune[4] & 0x0F) | (uint8_t)(tune[5] & 0x0F) << 4),
    (uint8_t)((tune[6] & 0x0F) | (uint8_t)(tune[7] & 0x0F) << 4),
    (uint8_t)((tune[8] & 0x0F) | (uint8_t)(tune[9] & 0x0F) << 4),
    (uint8_t)((tune[10] & 0x0F) | (uint8_t)(tune[11] & 0x0F) << 4),
    (uint8_t)((tune[12] & 0x0F) | (uint8_t)(tune[13] & 0x0F) << 4),
    (uint8_t)((tune[14] & 0x0F) | (uint8_t)(tune[15] & 0x0F) << 4),
    (uint8_t)((tune[16] & 0x0F) | (uint8_t)(tune[17] & 0x0F) << 4),
    (uint8_t)(((uint16_t)gainOffset >> 8) & 0xFF), (uint8_t)(gainOffset & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_SET_RSSI_CALIBRATION, true, buff, sizeof(buff)));
}

int16_t LR20xx::setLoRaSyncWord(uint16_t sync) {
  //uint8_t buff[2];
  //buff[0]=((sync>>8)&0xff);
  //buff[1]=((sync>>0)&0xff);
  uint8_t buff[1];
  buff[0]=((sync>>8)&0xff);
  //buff[1]=((sync>>0)&0xff);
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_LORA_SYNC_WORD, true, buff, sizeof(buff)));
}
int16_t LR20xx::setFSKSyncWord(uint64_t sync,uint8_t bit_order,uint8_t nb_bits) {
  uint8_t buff[9] = {
       (uint8_t)((sync >> 56) & 0xFF), (uint8_t)((sync >> 48) & 0xFF),
       (uint8_t)((sync >> 40) & 0xFF), (uint8_t)((sync >> 32) & 0xFF),
       (uint8_t)((sync >> 24) & 0xFF), (uint8_t)((sync >> 16) & 0xFF),
       (uint8_t)((sync >> 8) & 0xFF), (uint8_t)(sync & 0xFF),(uint8_t)((bit_order&0x1)<<7|(nb_bits&0x7F))
  };
  printf("setFSKSyncWord,buff[0]:%02X,buff[8]:%02X\n",buff[0],buff[8]);
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_FSK_SYNC_WORD, true, buff, sizeof(buff)));
}

int16_t LR20xx::setFLRCSyncWord(uint8_t sync_num,uint32_t sync) {
  uint8_t buff[5] = {
  sync_num,
     (uint8_t)((sync >> 24) & 0xFF), (uint8_t)((sync >> 16) & 0xFF),
    (uint8_t)((sync >> 8) & 0xFF), (uint8_t)(sync & 0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR20xx_CMD_SET_FLRC_SYNC_WORD, true, buff, sizeof(buff)));
}




int16_t LR20xx::lrFhssBuildFrame(uint8_t hdrCount, uint8_t cr, uint8_t grid, bool hop, uint8_t bw, uint16_t hopSeq, int8_t devOffset, uint8_t* payload, size_t len) {
  // check maximum size
  const uint8_t maxLen[4][4] = {
    { 189, 178, 167, 155, },
    { 151, 142, 133, 123, },
    { 112, 105,  99,  92, },
    {  74,  69,  65,  60, },
  };
  if((cr > RADIOLIB_LR11X0_LR_FHSS_CR_1_3) || ((hdrCount - 1) > (int)sizeof(maxLen[0])) || (len > maxLen[cr][hdrCount - 1])) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }

  // build buffers
  size_t buffLen = 9 + len;
  #if RADIOLIB_STATIC_ONLY
    uint8_t dataBuff[9 + 190];
  #else
    uint8_t* dataBuff = new uint8_t[buffLen];
  #endif

  // set properties of the packet
  dataBuff[0] = hdrCount;
  dataBuff[1] = cr;
  dataBuff[2] = RADIOLIB_LR11X0_LR_FHSS_MOD_TYPE_GMSK;
  dataBuff[3] = grid;
  dataBuff[4] = (uint8_t)hop;
  dataBuff[5] = bw;
  dataBuff[6] = (uint8_t)((hopSeq >> 8) & 0x01);
  dataBuff[7] = (uint8_t)(hopSeq & 0xFF);
  dataBuff[8] = devOffset;
  memcpy(&dataBuff[9], payload, len);

  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_LR_FHSS_BUILD_FRAME, true, dataBuff, buffLen);
  #if !RADIOLIB_STATIC_ONLY
    delete[] dataBuff;
  #endif
  return(state);
}

int16_t LR20xx::lrFhssSetSyncWord(uint32_t sync) {
  uint8_t buff[4] = {
    (uint8_t)((sync >> 24) & 0xFF), (uint8_t)((sync >> 16) & 0xFF),
    (uint8_t)((sync >> 8) & 0xFF), (uint8_t)(sync & 0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_LR_FHSS_SET_SYNC_WORD, true, buff, sizeof(buff)));
}

int16_t LR20xx::configBleBeacon(uint8_t chan, uint8_t* payload, size_t len) {
  return(this->bleBeaconCommon(RADIOLIB_LR11X0_CMD_CONFIG_BLE_BEACON, chan, payload, len));
}

int16_t LR20xx::getLoRaRxHeaderInfos(uint8_t* info) {
  uint8_t buff[1] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GET_LORA_RX_HEADER_INFOS, false, buff, sizeof(buff));

  // pass the replies
  if(info) { *info = buff[0]; }

  return(state);
}

int16_t LR20xx::bleBeaconSend(uint8_t chan, uint8_t* payload, size_t len) {
  return(this->bleBeaconCommon(RADIOLIB_LR11X0_CMD_BLE_BEACON_SEND, chan, payload, len));
}

int16_t LR20xx::bleBeaconCommon(uint16_t cmd, uint8_t chan, uint8_t* payload, size_t len) {
  // check maximum size
  // TODO what is the actual maximum?
  if(len > RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }

  // build buffers
  #if RADIOLIB_STATIC_ONLY
    uint8_t dataBuff[sizeof(uint8_t) + RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
  #else
    uint8_t* dataBuff = new uint8_t[sizeof(uint8_t) + len];
  #endif

  // set the channel
  dataBuff[0] = chan;
  memcpy(&dataBuff[1], payload, len);

  int16_t state = this->SPIcommand(cmd, true, dataBuff, sizeof(uint8_t) + len);
  #if !RADIOLIB_STATIC_ONLY
    delete[] dataBuff;
  #endif
  return(state);
}

int16_t LR20xx::wifiScan(uint8_t type, uint16_t mask, uint8_t acqMode, uint8_t nbMaxRes, uint8_t nbScanPerChan, uint16_t timeout, uint8_t abortOnTimeout) {
  uint8_t buff[9] = {
    type, (uint8_t)((mask >> 8) & 0xFF), (uint8_t)(mask & 0xFF),
    acqMode, nbMaxRes, nbScanPerChan,
    (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF),
    abortOnTimeout
  };

  // call the SPI write stream directly to skip waiting for BUSY - it will be set to high once the scan starts
  return(this->mod->SPIwriteStream(RADIOLIB_LR11X0_CMD_WIFI_SCAN, buff, sizeof(buff), false, false));
}

int16_t LR20xx::wifiScanTimeLimit(uint8_t type, uint16_t mask, uint8_t acqMode, uint8_t nbMaxRes, uint16_t timePerChan, uint16_t timeout) {
  uint8_t buff[9] = {
    type, (uint8_t)((mask >> 8) & 0xFF), (uint8_t)(mask & 0xFF),
    acqMode, nbMaxRes,
    (uint8_t)((timePerChan >> 8) & 0xFF), (uint8_t)(timePerChan & 0xFF),
    (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_SCAN_TIME_LIMIT, true, buff, sizeof(buff)));
}

int16_t LR20xx::wifiCountryCode(uint16_t mask, uint8_t nbMaxRes, uint8_t nbScanPerChan, uint16_t timeout, uint8_t abortOnTimeout) {
  uint8_t buff[7] = {
    (uint8_t)((mask >> 8) & 0xFF), (uint8_t)(mask & 0xFF),
    nbMaxRes, nbScanPerChan,
    (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF),
    abortOnTimeout
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_COUNTRY_CODE, true, buff, sizeof(buff)));
}

int16_t LR20xx::wifiCountryCodeTimeLimit(uint16_t mask, uint8_t nbMaxRes, uint16_t timePerChan, uint16_t timeout) {
  uint8_t buff[7] = {
    (uint8_t)((mask >> 8) & 0xFF), (uint8_t)(mask & 0xFF),
    nbMaxRes,
    (uint8_t)((timePerChan >> 8) & 0xFF), (uint8_t)(timePerChan & 0xFF),
    (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_COUNTRY_CODE_TIME_LIMIT, true, buff, sizeof(buff)));
}

int16_t LR20xx::wifiReadResults(uint8_t index, uint8_t nbResults, uint8_t format, uint8_t* results) {
  uint8_t buff[3] = { index, nbResults, format };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_READ_RESULTS, false, results, RADIOLIB_LR11X0_WIFI_RESULT_MAX_LEN, buff, sizeof(buff)));
}

int16_t LR20xx::wifiResetCumulTimings(void) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_RESET_CUMUL_TIMINGS, true, NULL, 0));
}

int16_t LR20xx::wifiReadCumulTimings(uint32_t* detection, uint32_t* capture, uint32_t* demodulation) {
  uint8_t buff[16] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_READ_CUMUL_TIMINGS, false, buff, sizeof(buff));

  // pass the replies
  if(detection) { *detection = ((uint32_t)(buff[4]) << 24) | ((uint32_t)(buff[5]) << 16) | ((uint32_t)(buff[6]) << 8) | (uint32_t)buff[7]; }
  if(capture) { *capture = ((uint32_t)(buff[8]) << 24) | ((uint32_t)(buff[9]) << 16) | ((uint32_t)(buff[10]) << 8) | (uint32_t)buff[11]; }
  if(demodulation) { *demodulation = ((uint32_t)(buff[12]) << 24) | ((uint32_t)(buff[13]) << 16) | ((uint32_t)(buff[14]) << 8) | (uint32_t)buff[15]; }

  return(state);
}

int16_t LR20xx::wifiGetNbCountryCodeResults(uint8_t* nbResults) {
  uint8_t buff[1] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_GET_NB_COUNTRY_CODE_RESULTS, false, buff, sizeof(buff));

  // pass the replies
  if(nbResults) { *nbResults = buff[0]; }

  return(state);
}

int16_t LR20xx::wifiReadCountryCodeResults(uint8_t index, uint8_t nbResults, uint8_t* results) {
  uint8_t reqBuff[2] = { index, nbResults };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_READ_COUNTRY_CODE_RESULTS, false, results, nbResults, reqBuff, sizeof(reqBuff)));
}

int16_t LR20xx::wifiCfgTimestampAPphone(uint32_t timestamp) {
  uint8_t buff[4] = {
    (uint8_t)((timestamp >> 24) & 0xFF), (uint8_t)((timestamp >> 16) & 0xFF),
    (uint8_t)((timestamp >> 8) & 0xFF), (uint8_t)(timestamp & 0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_COUNTRY_CODE_TIME_LIMIT, true, buff, sizeof(buff)));
}

int16_t LR20xx::wifiReadVersion(uint8_t* major, uint8_t* minor) {
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_WIFI_READ_VERSION, false, buff, sizeof(buff));

  // pass the replies
  if(major) { *major = buff[0]; }
  if(minor) { *minor = buff[1]; }

  return(state);
}

int16_t LR20xx::gnssSetConstellationToUse(uint8_t mask) {
  uint8_t buff[1] = { mask };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_SET_CONSTELLATION_TO_USE, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssReadConstellationToUse(uint8_t* mask) {
  uint8_t buff[1] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_CONSTELLATION_TO_USE, false, buff, sizeof(buff));

  // pass the replies
  if(mask) { *mask = buff[0]; }

  return(state);
}

int16_t LR20xx::gnssSetAlmanacUpdate(uint8_t mask) {
  uint8_t buff[1] = { mask };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_SET_ALMANAC_UPDATE, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssReadAlmanacUpdate(uint8_t* mask) {
  uint8_t buff[1] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_ALMANAC_UPDATE, false, buff, sizeof(buff));

  // pass the replies
  if(mask) { *mask = buff[0]; }

  return(state);
}

int16_t LR20xx::gnssReadVersion(uint8_t* fw, uint8_t* almanac) {
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_VERSION, false, buff, sizeof(buff));

  // pass the replies
  if(fw) { *fw = buff[0]; }
  if(almanac) { *almanac = buff[1]; }

  return(state);
}

int16_t LR20xx::gnssReadSupportedConstellations(uint8_t* mask) {
  uint8_t buff[1] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_SUPPORTED_CONSTELLATIONS, false, buff, sizeof(buff));

  // pass the replies
  if(mask) { *mask = buff[0]; }

  return(state);
}

int16_t LR20xx::gnssSetMode(uint8_t mode) {
  uint8_t buff[1] = { mode };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_SET_MODE, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssAutonomous(uint32_t gpsTime, uint8_t resMask, uint8_t nbSvMask) {
  uint8_t buff[7] = {
    (uint8_t)((gpsTime >> 24) & 0xFF), (uint8_t)((gpsTime >> 16) & 0xFF),
    (uint8_t)((gpsTime >> 8) & 0xFF), (uint8_t)(gpsTime & 0xFF),
    RADIOLIB_LR11X0_GNSS_AUTO_EFFORT_MODE, resMask, nbSvMask
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_AUTONOMOUS, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssAssisted(uint32_t gpsTime, uint8_t effort, uint8_t resMask, uint8_t nbSvMask) {
  uint8_t buff[7] = {
    (uint8_t)((gpsTime >> 24) & 0xFF), (uint8_t)((gpsTime >> 16) & 0xFF),
    (uint8_t)((gpsTime >> 8) & 0xFF), (uint8_t)(gpsTime & 0xFF),
    effort, resMask, nbSvMask
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_ASSISTED, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssSetAssistancePosition(float lat, float lon) {
  uint16_t latRaw = (lat*2048.0f)/90.0f + 0.5f;
  uint16_t lonRaw = (lon*2048.0f)/180.0f + 0.5f;
  uint8_t buff[4] = {
    (uint8_t)((latRaw >> 8) & 0xFF), (uint8_t)(latRaw & 0xFF),
    (uint8_t)((lonRaw >> 8) & 0xFF), (uint8_t)(lonRaw & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_SET_ASSISTANCE_POSITION, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssReadAssistancePosition(float* lat, float* lon) {
  uint8_t buff[4] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_ASSISTANCE_POSITION, false, buff, sizeof(buff));

  // pass the replies
  if(lat) {
    uint16_t latRaw = ((uint16_t)(buff[0]) << 8) | (uint16_t)(buff[1]);
    *lat = ((float)latRaw*90.0f)/2048.0f;
  }
  if(lon) {
    uint16_t lonRaw = ((uint16_t)(buff[2]) << 8) | (uint16_t)(buff[3]);
    *lon = ((float)lonRaw*180.0f)/2048.0f;
  }

  return(state);
}

int16_t LR20xx::gnssPushSolverMsg(uint8_t* payload, size_t len) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_PUSH_SOLVER_MSG, true, payload, len));
}

int16_t LR20xx::gnssPushDmMsg(uint8_t* payload, size_t len) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_PUSH_DM_MSG, true, payload, len));
}

int16_t LR20xx::gnssGetContextStatus(uint8_t* fwVersion, uint32_t* almanacCrc, uint8_t* errCode, uint8_t* almUpdMask, uint8_t* freqSpace) {
  // send the command - datasheet here shows extra bytes being sent in the request
  // but doing that fails so treat it like any other read command
  uint8_t buff[9] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_GET_CONTEXT_STATUS, false, buff, sizeof(buff));

  // pass the replies
  if(fwVersion) { *fwVersion = buff[2]; }
  if(almanacCrc) { *almanacCrc = ((uint32_t)(buff[3]) << 24) | ((uint32_t)(buff[4]) << 16) | ((uint32_t)(buff[5]) << 8) | (uint32_t)buff[6]; }
  if(errCode) { *errCode = (buff[7] & 0xF0) >> 4; }
  if(almUpdMask) { *almUpdMask = (buff[7] & 0x0E) >> 1; }
  if(freqSpace) { *freqSpace = ((buff[7] & 0x01) << 1) | ((buff[8] & 0x80) >> 7); }

  return(state);
}

int16_t LR20xx::gnssGetNbSvDetected(uint8_t* nbSv) {
  uint8_t buff[1] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_GET_NB_SV_DETECTED, false, buff, sizeof(buff));

  // pass the replies
  if(nbSv) { *nbSv = buff[0]; }

  return(state);
}

int16_t LR20xx::gnssGetSvDetected(uint8_t* svId, uint8_t* snr, uint16_t* doppler, size_t nbSv) {
  // TODO this is arbitrary - is there an actual maximum?
  if(nbSv > RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN/sizeof(uint32_t)) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }

  // build buffers
  size_t buffLen = nbSv*sizeof(uint32_t);
  #if RADIOLIB_STATIC_ONLY
    uint8_t dataBuff[RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
  #else
    uint8_t* dataBuff = new uint8_t[buffLen];
  #endif

  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_GET_SV_DETECTED, false, dataBuff, buffLen);
  if(state == RADIOLIB_ERR_NONE) {
    for(size_t i = 0; i < nbSv; i++) {
      if(svId) { svId[i] = dataBuff[4*i]; }
      if(snr) { snr[i] = dataBuff[4*i + 1]; }
      if(doppler) { doppler[i] = ((uint16_t)(dataBuff[4*i + 2]) << 8) | (uint16_t)dataBuff[4*i + 3]; }
    }
  }

  #if !RADIOLIB_STATIC_ONLY
    delete[] dataBuff;
  #endif
  return(state);
}

int16_t LR20xx::gnssGetConsumption(uint32_t* cpu, uint32_t* radio) {
  uint8_t buff[8] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_GET_CONSUMPTION, false, buff, sizeof(buff));

  // pass the replies
  if(cpu) { *cpu = ((uint32_t)(buff[0]) << 24) | ((uint32_t)(buff[1]) << 16) | ((uint32_t)(buff[2]) << 8) | (uint32_t)buff[3]; }
  if(radio) { *radio = ((uint32_t)(buff[4]) << 24) | ((uint32_t)(buff[5]) << 16) | ((uint32_t)(buff[6]) << 8) | (uint32_t)buff[7]; }

  return(state);
}

int16_t LR20xx::gnssGetResultSize(uint16_t* size) {
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_GET_RESULT_SIZE, false, buff, sizeof(buff));

  // pass the replies
  if(size) { *size = ((uint16_t)(buff[0]) << 8) | (uint16_t)buff[1]; }
  
  return(state);
}

int16_t LR20xx::gnssReadResults(uint8_t* result, uint16_t size) {
  if(!result) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_RESULTS, false, result, size));
}

int16_t LR20xx::gnssAlmanacFullUpdateHeader(uint16_t date, uint32_t globalCrc) {
  uint8_t buff[RADIOLIB_LR11X0_GNSS_ALMANAC_BLOCK_SIZE] = {
    RADIOLIB_LR11X0_GNSS_ALMANAC_HEADER_ID,
    (uint8_t)((date >> 8) & 0xFF), (uint8_t)(date & 0xFF),
    (uint8_t)((globalCrc >> 24) & 0xFF), (uint8_t)((globalCrc >> 16) & 0xFF), 
    (uint8_t)((globalCrc >> 8) & 0xFF), (uint8_t)(globalCrc & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_ALMANAC_FULL_UPDATE, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssAlmanacFullUpdateSV(uint8_t svn, uint8_t* svnAlmanac) {
  uint8_t buff[RADIOLIB_LR11X0_GNSS_ALMANAC_BLOCK_SIZE] = { svn };
  memcpy(&buff[1], svnAlmanac, RADIOLIB_LR11X0_GNSS_ALMANAC_BLOCK_SIZE - 1);
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_ALMANAC_FULL_UPDATE, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssGetSvVisible(uint32_t time, float lat, float lon, uint8_t constellation, uint8_t* nbSv) {
  uint16_t latRaw = (lat*2048.0f)/90.0f + 0.5f;
  uint16_t lonRaw = (lon*2048.0f)/180.0f + 0.5f;
  uint8_t reqBuff[9] = { 
    (uint8_t)((time >> 24) & 0xFF), (uint8_t)((time >> 16) & 0xFF),
    (uint8_t)((time >> 8) & 0xFF), (uint8_t)(time & 0xFF),
    (uint8_t)((latRaw >> 8) & 0xFF), (uint8_t)(latRaw & 0xFF),
    (uint8_t)((lonRaw >> 8) & 0xFF), (uint8_t)(lonRaw & 0xFF),
    constellation,
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_GET_SV_VISIBLE, false, nbSv, 1, reqBuff, sizeof(reqBuff)));
}

int16_t LR20xx::gnssPerformScan(uint8_t effort, uint8_t resMask, uint8_t nbSvMax) {
  uint8_t buff[3] = { effort, resMask, nbSvMax };
  // call the SPI write stream directly to skip waiting for BUSY - it will be set to high once the scan starts
  return(this->mod->SPIwriteStream(RADIOLIB_LR11X0_CMD_GNSS_SCAN, buff, sizeof(buff), false, false));
}

int16_t LR20xx::gnssReadLastScanModeLaunched(uint8_t* lastScanMode) {
  uint8_t buff[1] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_LAST_SCAN_MODE_LAUNCHED, false, buff, sizeof(buff));

  // pass the replies
  if(lastScanMode) { *lastScanMode = buff[0]; }
  
  return(state);
}

int16_t LR20xx::gnssFetchTime(uint8_t effort, uint8_t opt) {
  uint8_t buff[2] = { effort, opt };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_FETCH_TIME, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssReadTime(uint8_t* err, uint32_t* time, uint32_t* nbUs, uint32_t* timeAccuracy) {
  uint8_t buff[12] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_TIME, false, buff, sizeof(buff));

  // pass the replies
  if(err) { *err = buff[0]; }
  if(time) { *time = ((uint32_t)(buff[1]) << 24) | ((uint32_t)(buff[2]) << 16) | ((uint32_t)(buff[3]) << 8) | (uint32_t)buff[4]; }
  if(nbUs) { *nbUs = ((uint32_t)(buff[5]) << 16) | ((uint32_t)(buff[6]) << 8) | (uint32_t)buff[7]; }
  if(timeAccuracy) { *timeAccuracy = ((uint32_t)(buff[8]) << 24) | ((uint32_t)(buff[9]) << 16) | ((uint32_t)(buff[10]) << 8) | (uint32_t)buff[11]; }
  
  return(state);
}

int16_t LR20xx::gnssResetTime(void) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_RESET_TIME, true, NULL, 0));
}

int16_t LR20xx::gnssResetPosition(void) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_RESET_POSITION, true, NULL, 0));
}

int16_t LR20xx::gnssReadDemodStatus(int8_t* status, uint8_t* info) {
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_DEMOD_STATUS, false, buff, sizeof(buff));

  // pass the replies
  if(status) { *status = (int8_t)buff[0]; }
  if(info) { *info = buff[1]; }
  
  return(state);
}

int16_t LR20xx::gnssReadCumulTiming(uint32_t* timing, uint8_t* constDemod) {
  uint8_t rplBuff[125] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR20xx_CMD_READ_REG_MEM, false, rplBuff, 125);
  RADIOLIB_ASSERT(state);

  // convert endians
  if(timing) {
    for(size_t i = 0; i < 31; i++) {
      timing[i] = ((uint32_t)rplBuff[i*sizeof(uint32_t)] << 24) | ((uint32_t)rplBuff[1 + i*sizeof(uint32_t)] << 16) | ((uint32_t)rplBuff[2 + i*sizeof(uint32_t)] << 8) | (uint32_t)rplBuff[3 + i*sizeof(uint32_t)];
    }
  }

  if(constDemod) { *constDemod = rplBuff[124]; }
  
  return(state);
}

int16_t LR20xx::gnssSetTime(uint32_t time, uint16_t accuracy) {
  uint8_t buff[6] = {
    (uint8_t)((time >> 24) & 0xFF), (uint8_t)((time >> 16) & 0xFF),
    (uint8_t)((time >> 8) & 0xFF), (uint8_t)(time & 0xFF),
    (uint8_t)((accuracy >> 8) & 0xFF), (uint8_t)(accuracy & 0xFF),
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_SET_TIME, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssReadDopplerSolverRes(uint8_t* error, uint8_t* nbSvUsed, float* lat, float* lon, uint16_t* accuracy, uint16_t* xtal, float* latFilt, float* lonFilt, uint16_t* accuracyFilt, uint16_t* xtalFilt) {
  uint8_t buff[18] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_DOPPLER_SOLVER_RES, false, buff, sizeof(buff));

  // pass the replies
  if(error) { *error = buff[0]; }
  if(nbSvUsed) { *nbSvUsed = buff[1]; }
  if(lat) {
    uint16_t latRaw = ((uint16_t)(buff[2]) << 8) | (uint16_t)buff[3];
    *lat = ((float)latRaw * 90.0f)/2048.0f;
  }
  if(lon) {
    uint16_t lonRaw = ((uint16_t)(buff[4]) << 8) | (uint16_t)buff[5];
    *lon = ((float)lonRaw * 180.0f)/2048.0f;
  }
  if(accuracy) { *accuracy = ((uint16_t)(buff[6]) << 8) | (uint16_t)buff[7]; }
  if(xtal) { *xtal = ((uint16_t)(buff[8]) << 8) | (uint16_t)buff[9]; }
  if(latFilt) {
    uint16_t latRaw = ((uint16_t)(buff[10]) << 8) | (uint16_t)buff[11];
    *latFilt = ((float)latRaw * 90.0f)/2048.0f;
  }
  if(lonFilt) {
    uint16_t lonRaw = ((uint16_t)(buff[12]) << 8) | (uint16_t)buff[13];
    *lonFilt = ((float)lonRaw * 180.0f)/2048.0f;
  }
  if(accuracyFilt) { *accuracyFilt = ((uint16_t)(buff[14]) << 8) | (uint16_t)buff[15]; }
  if(xtalFilt) { *xtalFilt = ((uint16_t)(buff[16]) << 8) | (uint16_t)buff[17]; }
  
  return(state);
}

int16_t LR20xx::gnssReadDelayResetAP(uint32_t* delay) {
  uint8_t buff[3] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_DELAY_RESET_AP, false, buff, sizeof(buff));

  if(delay) { *delay = ((uint32_t)(buff[0]) << 16) | ((uint32_t)(buff[1]) << 8) | (uint32_t)buff[2]; }
  
  return(state);
}

int16_t LR20xx::gnssAlmanacUpdateFromSat(uint8_t effort, uint8_t bitMask) {
  uint8_t buff[2] = { effort, bitMask };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_ALMANAC_UPDATE_FROM_SAT, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssReadAlmanacStatus(uint8_t* status) {
  // TODO parse the reply into some structure
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_ALMANAC_STATUS, false, status, 53));
}

int16_t LR20xx::gnssConfigAlmanacUpdatePeriod(uint8_t bitMask, uint8_t svType, uint16_t period) {
  uint8_t buff[4] = { bitMask, svType, (uint8_t)((period >> 8) & 0xFF), (uint8_t)(period & 0xFF) };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_CONFIG_ALMANAC_UPDATE_PERIOD, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssReadAlmanacUpdatePeriod(uint8_t bitMask, uint8_t svType, uint16_t* period) {
  uint8_t reqBuff[2] = { bitMask, svType };
  uint8_t rplBuff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_ALMANAC_UPDATE_PERIOD, false, rplBuff, sizeof(rplBuff), reqBuff, sizeof(reqBuff));
  RADIOLIB_ASSERT(state);

  if(period) { *period = ((uint16_t)(rplBuff[0]) << 8) | (uint16_t)rplBuff[1]; }

  return(state);
}

int16_t LR20xx::gnssConfigDelayResetAP(uint32_t delay) {
  uint8_t buff[3] = { (uint8_t)((delay >> 16) & 0xFF), (uint8_t)((delay >> 8) & 0xFF), (uint8_t)(delay & 0xFF) };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_CONFIG_DELAY_RESET_AP, true, buff, sizeof(buff)));
}

int16_t LR20xx::gnssGetSvWarmStart(uint8_t bitMask, uint8_t* sv, uint8_t nbVisSat) {
  uint8_t reqBuff[1] = { bitMask };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_GET_SV_WARM_START, false, sv, nbVisSat, reqBuff, sizeof(reqBuff)));
}

int16_t LR20xx::gnssReadWNRollover(uint8_t* status, uint8_t* rollover) {
  uint8_t buff[2] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_WN_ROLLOVER, false, buff, sizeof(buff));

  if(status) { *status = buff[0]; }
  if(rollover) { *rollover = buff[1]; }
  
  return(state);
}

int16_t LR20xx::gnssReadWarmStartStatus(uint8_t bitMask, uint8_t* nbVisSat, uint32_t* timeElapsed) {
  uint8_t reqBuff[1] = { bitMask };
  uint8_t rplBuff[5] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_WARM_START_STATUS, false, rplBuff, sizeof(rplBuff), reqBuff, sizeof(reqBuff));
  RADIOLIB_ASSERT(state);

  if(nbVisSat) { *nbVisSat = rplBuff[0]; }
  if(timeElapsed) { *timeElapsed = ((uint32_t)(rplBuff[1]) << 24) | ((uint32_t)(rplBuff[2]) << 16) | ((uint32_t)(rplBuff[3]) << 8) | (uint32_t)rplBuff[4]; }

  return(state);
}

int16_t LR20xx::gnssWriteBitMaskSatActivated(uint8_t bitMask, uint32_t* bitMaskActivated0, uint32_t* bitMaskActivated1) {
  uint8_t reqBuff[1] = { bitMask };
  uint8_t rplBuff[8] = { 0 };
  size_t rplLen = (bitMask & 0x01) ? 8 : 4; // GPS only has the first bit mask
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_GNSS_READ_WARM_START_STATUS, false, rplBuff, rplLen, reqBuff, sizeof(reqBuff));
  RADIOLIB_ASSERT(state);

  if(bitMaskActivated0) { *bitMaskActivated0 = ((uint32_t)(rplBuff[0]) << 24) | ((uint32_t)(rplBuff[1]) << 16) | ((uint32_t)(rplBuff[2]) << 8) | (uint32_t)rplBuff[3]; }
  if(bitMaskActivated1) { *bitMaskActivated1 = ((uint32_t)(rplBuff[4]) << 24) | ((uint32_t)(rplBuff[5]) << 16) | ((uint32_t)(rplBuff[6]) << 8) | (uint32_t)rplBuff[7]; }

  return(state);
}

int16_t LR20xx::cryptoSetKey(uint8_t keyId, uint8_t* key) {
  if(!key) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }
  uint8_t buff[1 + RADIOLIB_AES128_KEY_SIZE] = { 0 };
  buff[0] = keyId;
  memcpy(&buff[1], key, RADIOLIB_AES128_KEY_SIZE);
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_CRYPTO_SET_KEY, false, buff, sizeof(buff)));
}

int16_t LR20xx::cryptoDeriveKey(uint8_t srcKeyId, uint8_t dstKeyId, uint8_t* key) {
  if(!key) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }
  uint8_t buff[2 + RADIOLIB_AES128_KEY_SIZE] = { 0 };
  buff[0] = srcKeyId;
  buff[1] = dstKeyId;
  memcpy(&buff[2], key, RADIOLIB_AES128_KEY_SIZE);
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_CRYPTO_DERIVE_KEY, false, buff, sizeof(buff)));
}

int16_t LR20xx::cryptoProcessJoinAccept(uint8_t decKeyId, uint8_t verKeyId, uint8_t lwVer, uint8_t* header, uint8_t* dataIn, size_t len, uint8_t* dataOut) {
  // calculate buffer sizes
  size_t headerLen = 1;
  if(lwVer) {
    headerLen += 11; // LoRaWAN 1.1 header is 11 bytes longer than 1.0
  }
  size_t reqLen = 3*sizeof(uint8_t) + headerLen + len;
  size_t rplLen = sizeof(uint8_t) + len;

  // build buffers
  #if RADIOLIB_STATIC_ONLY
    uint8_t reqBuff[RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
    uint8_t rplBuff[RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
  #else
    uint8_t* reqBuff = new uint8_t[reqLen];
    uint8_t* rplBuff = new uint8_t[rplLen];
  #endif
  
  // set the request fields
  reqBuff[0] = decKeyId;
  reqBuff[1] = verKeyId;
  reqBuff[2] = lwVer;
  memcpy(&reqBuff[3], header, headerLen);
  memcpy(&reqBuff[3 + headerLen], dataIn, len);

  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_CRYPTO_PROCESS_JOIN_ACCEPT, false, rplBuff, rplLen, reqBuff, reqLen);
  #if !RADIOLIB_STATIC_ONLY
    delete[] reqBuff;
  #endif
  if(state != RADIOLIB_ERR_NONE) {
    #if !RADIOLIB_STATIC_ONLY
      delete[] rplBuff;
    #endif
    return(state);
  }

  // check the crypto engine state
  if(rplBuff[0] != RADIOLIB_LR11X0_CRYPTO_STATUS_SUCCESS) {
    RADIOLIB_DEBUG_BASIC_PRINTLN("Crypto Engine error: %02x", rplBuff[0]);
    return(RADIOLIB_ERR_SPI_CMD_FAILED);
  }

  // pass the data
  memcpy(dataOut, &rplBuff[1], len);
  return(state);
}

int16_t LR20xx::cryptoComputeAesCmac(uint8_t keyId, uint8_t* data, size_t len, uint32_t* mic) {
  size_t reqLen = sizeof(uint8_t) + len;
  #if RADIOLIB_STATIC_ONLY
    uint8_t reqBuff[sizeof(uint8_t) + RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
  #else
    uint8_t* reqBuff = new uint8_t[reqLen];
  #endif
  uint8_t rplBuff[5] = { 0 };
  
  reqBuff[0] = keyId;
  memcpy(&reqBuff[1], data, len);

  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_CRYPTO_COMPUTE_AES_CMAC, false, rplBuff, sizeof(rplBuff), reqBuff, reqLen);
  #if !RADIOLIB_STATIC_ONLY
    delete[] reqBuff;
  #endif

  // check the crypto engine state
  if(rplBuff[0] != RADIOLIB_LR11X0_CRYPTO_STATUS_SUCCESS) {
    RADIOLIB_DEBUG_BASIC_PRINTLN("Crypto Engine error: %02x", rplBuff[0]);
    return(RADIOLIB_ERR_SPI_CMD_FAILED);
  }

  if(mic) { *mic = ((uint32_t)(rplBuff[1]) << 24) |  ((uint32_t)(rplBuff[2]) << 16) | ((uint32_t)(rplBuff[3]) << 8) | (uint32_t)rplBuff[4]; }
  return(state);
}

int16_t LR20xx::cryptoVerifyAesCmac(uint8_t keyId, uint32_t micExp, uint8_t* data, size_t len, bool* result) {
   size_t reqLen = sizeof(uint8_t) + sizeof(uint32_t) + len;
  #if RADIOLIB_STATIC_ONLY
    uint8_t reqBuff[sizeof(uint8_t) + sizeof(uint32_t) + RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
  #else
    uint8_t* reqBuff = new uint8_t[reqLen];
  #endif
  uint8_t rplBuff[1] = { 0 };
  
  reqBuff[0] = keyId;
  reqBuff[1] = (uint8_t)((micExp >> 24) & 0xFF);
  reqBuff[2] = (uint8_t)((micExp >> 16) & 0xFF);
  reqBuff[3] = (uint8_t)((micExp >> 8) & 0xFF);
  reqBuff[4] = (uint8_t)(micExp & 0xFF);
  memcpy(&reqBuff[5], data, len);

  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_CRYPTO_VERIFY_AES_CMAC, false, rplBuff, sizeof(rplBuff), reqBuff, reqLen);
  #if !RADIOLIB_STATIC_ONLY
    delete[] reqBuff;
  #endif

  // check the crypto engine state
  if(rplBuff[0] != RADIOLIB_LR11X0_CRYPTO_STATUS_SUCCESS) {
    RADIOLIB_DEBUG_BASIC_PRINTLN("Crypto Engine error: %02x", rplBuff[0]);
    return(RADIOLIB_ERR_SPI_CMD_FAILED);
  }

  if(result) { *result = (rplBuff[0] == RADIOLIB_LR11X0_CRYPTO_STATUS_SUCCESS); }
  return(state);
}

int16_t LR20xx::cryptoAesEncrypt01(uint8_t keyId, uint8_t* dataIn, size_t len, uint8_t* dataOut) {
  return(this->cryptoCommon(RADIOLIB_LR11X0_CMD_CRYPTO_AES_ENCRYPT_01, keyId, dataIn, len, dataOut));
}

int16_t LR20xx::cryptoAesEncrypt(uint8_t keyId, uint8_t* dataIn, size_t len, uint8_t* dataOut) {
  return(this->cryptoCommon(RADIOLIB_LR11X0_CMD_CRYPTO_AES_ENCRYPT, keyId, dataIn, len, dataOut));
}

int16_t LR20xx::cryptoAesDecrypt(uint8_t keyId, uint8_t* dataIn, size_t len, uint8_t* dataOut) {
  return(this->cryptoCommon(RADIOLIB_LR11X0_CMD_CRYPTO_AES_DECRYPT, keyId, dataIn, len, dataOut));
}

int16_t LR20xx::cryptoStoreToFlash(void) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_CRYPTO_STORE_TO_FLASH, true, NULL, 0));
}

int16_t LR20xx::cryptoRestoreFromFlash(void) {
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_CRYPTO_RESTORE_FROM_FLASH, true, NULL, 0));
}

int16_t LR20xx::cryptoSetParam(uint8_t id, uint32_t value) {
  uint8_t buff[5] = {
    id,
    (uint8_t)((value >> 24) & 0xFF), (uint8_t)((value >> 16) & 0xFF),
    (uint8_t)((value >> 8) & 0xFF), (uint8_t)(value & 0xFF)
  };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_CRYPTO_SET_PARAM, true, buff, sizeof(buff)));
}

int16_t LR20xx::cryptoGetParam(uint8_t id, uint32_t* value) {
  uint8_t reqBuff[1] = { id };
  uint8_t rplBuff[4] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_CRYPTO_GET_PARAM, false, rplBuff, sizeof(rplBuff), reqBuff, sizeof(reqBuff));
  RADIOLIB_ASSERT(state);
  if(value) { *value = ((uint32_t)(rplBuff[0]) << 24) | ((uint32_t)(rplBuff[1]) << 16) | ((uint32_t)(rplBuff[2]) << 8) | (uint32_t)rplBuff[3]; }
  return(state);
}

int16_t LR20xx::cryptoCheckEncryptedFirmwareImage(uint32_t offset, uint32_t* data, size_t len, bool nonvolatile) {
  // check maximum size
  if(len > (RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN/sizeof(uint32_t))) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }
  return(this->writeCommon(RADIOLIB_LR11X0_CMD_CRYPTO_CHECK_ENCRYPTED_FIRMWARE_IMAGE, offset, data, len, nonvolatile));
}

int16_t LR20xx::cryptoCheckEncryptedFirmwareImageResult(bool* result) {
  uint8_t buff[1] = { 0 };
  int16_t state = this->SPIcommand(RADIOLIB_LR11X0_CMD_CRYPTO_CHECK_ENCRYPTED_FIRMWARE_IMAGE_RESULT, false, buff, sizeof(buff));

  // pass the replies
  if(result) { *result = (bool)buff[0]; }
  
  return(state);
}

int16_t LR20xx::bootEraseFlash(void) {
  // erasing flash takes about 2.5 seconds, temporarily tset SPI timeout to 3 seconds
  RadioLibTime_t timeout = this->mod->spiConfig.timeout;
  this->mod->spiConfig.timeout = 3000;
  int16_t state = this->mod->SPIwriteStream(RADIOLIB_LR11X0_CMD_BOOT_ERASE_FLASH, NULL, 0, false, false);
  this->mod->spiConfig.timeout = timeout;
  return(state);
}

int16_t LR20xx::bootWriteFlashEncrypted(uint32_t offset, uint32_t* data, size_t len, bool nonvolatile) {
  // check maximum size
  if(len > (RADIOLIB_LR20xx_SPI_MAX_READ_WRITE_LEN/sizeof(uint32_t))) {
    return(RADIOLIB_ERR_SPI_CMD_INVALID);
  }
  return(this->writeCommon(RADIOLIB_LR11X0_CMD_BOOT_WRITE_FLASH_ENCRYPTED, offset, data, len, nonvolatile));
}

int16_t LR20xx::bootReboot(bool stay) {
  uint8_t buff[1] = { (uint8_t)stay };
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_BOOT_REBOOT, true, buff, sizeof(buff)));
}

int16_t LR20xx::bootGetPin(uint8_t* pin) {
  if(!pin) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_BOOT_GET_PIN, false, pin, RADIOLIB_LR11X0_PIN_LEN));
}

int16_t LR20xx::bootGetChipEui(uint8_t* eui) {
  if(!eui) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_BOOT_GET_CHIP_EUI, false, eui, RADIOLIB_LR11X0_EUI_LEN));
}

int16_t LR20xx::bootGetJoinEui(uint8_t* eui) {
  if(!eui) {
    return(RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED);
  }
  return(this->SPIcommand(RADIOLIB_LR11X0_CMD_BOOT_GET_JOIN_EUI, false, eui, RADIOLIB_LR11X0_EUI_LEN));
}

int16_t LR20xx::writeCommon(uint16_t cmd, uint32_t addrOffset, const uint32_t* data, size_t len, bool nonvolatile) {
  // build buffers - later we need to ensure endians are correct, 
  // so there is probably no way to do this without copying buffers and iterating
  size_t buffLen = sizeof(uint32_t) + len*sizeof(uint32_t);
  #if RADIOLIB_STATIC_ONLY
    uint8_t dataBuff[sizeof(uint32_t) + RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
  #else
    uint8_t* dataBuff = new uint8_t[buffLen];
  #endif

  // set the address or offset
  dataBuff[0] = (uint8_t)((addrOffset >> 16) & 0xFF);
  dataBuff[1] = (uint8_t)((addrOffset >> 8) & 0xFF);
  dataBuff[2] = (uint8_t)(addrOffset & 0xFF);

  // convert endians
  for(size_t i = 0; i < len; i++) {
    uint32_t bin = 0;
    if(nonvolatile) {
      bin = RADIOLIB_NONVOLATILE_READ_DWORD(data + i);
    } else {
      bin = data[i];
    }
    dataBuff[3 + i*sizeof(uint32_t)] = (uint8_t)((bin >> 24) & 0xFF);
    dataBuff[4 + i*sizeof(uint32_t)] = (uint8_t)((bin >> 16) & 0xFF);
    dataBuff[5 + i*sizeof(uint32_t)] = (uint8_t)((bin >> 8) & 0xFF);
    dataBuff[6 + i*sizeof(uint32_t)] = (uint8_t)(bin & 0xFF);
  }

  int16_t state = this->mod->SPIwriteStream(cmd, dataBuff, buffLen, true, false);
  #if !RADIOLIB_STATIC_ONLY
    delete[] dataBuff;
  #endif
  return(state);
}

int16_t LR20xx::cryptoCommon(uint16_t cmd, uint8_t keyId, uint8_t* dataIn, size_t len, uint8_t* dataOut) {
  // build buffers
  #if RADIOLIB_STATIC_ONLY
    uint8_t reqBuff[RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
    uint8_t rplBuff[RADIOLIB_LR11X0_SPI_MAX_READ_WRITE_LEN];
  #else
    uint8_t* reqBuff = new uint8_t[sizeof(uint8_t) + len];
    uint8_t* rplBuff = new uint8_t[sizeof(uint8_t) + len];
  #endif
  
  // set the request fields
  reqBuff[0] = keyId;
  memcpy(&reqBuff[1], dataIn, len);

  int16_t state = this->SPIcommand(cmd, false, rplBuff, sizeof(uint8_t) + len, reqBuff, sizeof(uint8_t) + len);
  #if !RADIOLIB_STATIC_ONLY
    delete[] reqBuff;
  #endif
  if(state != RADIOLIB_ERR_NONE) {
    #if !RADIOLIB_STATIC_ONLY
      delete[] rplBuff;
    #endif
    return(state);
  }

  // check the crypto engine state
  if(rplBuff[0] != RADIOLIB_LR11X0_CRYPTO_STATUS_SUCCESS) {
    RADIOLIB_DEBUG_BASIC_PRINTLN("Crypto Engine error: %02x", rplBuff[0]);
    return(RADIOLIB_ERR_SPI_CMD_FAILED);
  }

  // pass the data
  memcpy(dataOut, &rplBuff[1], len);
  return(state);
}

#endif
