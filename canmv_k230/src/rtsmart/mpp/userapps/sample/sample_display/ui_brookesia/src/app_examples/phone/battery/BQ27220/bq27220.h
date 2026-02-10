#pragma once
#ifndef BQ27220_H
#define BQ27220_H
#ifdef __cplusplus
extern "C" {
#endif
typedef union ControlStatus{
    struct __reg4
    {
        // Low byte, Low bit first
        uint8_t BATT_ID : 3; /**< Battery Identification */
        bool SNOOZE     : 1; /**< SNOOZE mode is enabled */
        bool BCA        : 1; /**< fuel gauge board calibration routine is active */
        bool CCA        : 1; /**< Coulomb Counter Calibration routine is active */
        uint8_t RSVD0   : 2; /**< Reserved */
        // High byte, Low bit first
        uint8_t RSVD1; /**< Reserved */
    } reg;
    uint16_t full;
} BQ27220ControlStatus;

typedef union BatteryStatus {
    struct __reg3
    {
        // Low byte, Low bit first
        uint16_t DSG        : 1; /**< The device is in DISCHARGE */
        uint16_t SYSDWN     : 1; /**< System down bit indicating the system should shut down */
        uint16_t TDA        : 1; /**< Terminate Discharge Alarm */
        uint16_t BATTPRES   : 1; /**< Battery Present detected */
        uint16_t AUTH_GD    : 1; /**< Detect inserted battery */
        uint16_t OCVGD      : 1; /**< Good OCV measurement taken */
        uint16_t TCA        : 1; /**< Terminate Charge Alarm */
        uint16_t RSVD       : 1; /**< Reserved */
        // High byte, Low bit first
        uint16_t CHGING     : 1; /**< Charge inhibit */
        uint16_t FC         : 1; /**< Full-charged is detected */
        uint16_t OTD        : 1; /**< Overtemperature in discharge condition is detected */
        uint16_t OTC        : 1; /**< Overtemperature in charge condition is detected */
        uint16_t SLEEP      : 1; /**< Device is operating in SLEEP mode when set */
        uint16_t OCVFALL    : 1; /**< Status bit indicating that the OCV reading failed due to current */
        uint16_t OCVCOMP    : 1; /**< An OCV measurement update is complete */
        uint16_t FD         : 1; /**< Full-discharge is detected */
    } reg;
    uint16_t full;
}BQ27220BatteryStatus;

typedef enum {
    Bq27220OperationStatusSecSealed = 0b11,
    Bq27220OperationStatusSecUnsealed = 0b10,
    Bq27220OperationStatusSecFull = 0b01,
} Bq27220OperationStatusSec;

typedef union OperationStatus{
    struct __reg1
    {
        // Low byte, Low bit first
        bool CALMD      : 1; /**< Calibration mode enabled */
        uint8_t SEC     : 2; /**< Current security access */
        bool EDV2       : 1; /**< EDV2 threshold exceeded */
        bool VDQ        : 1; /**< Indicates if Current discharge cycle is NOT qualified or qualified for an FCC updated */
        bool INITCOMP   : 1; /**< gauge initialization is complete */
        bool SMTH       : 1; /**< RemainingCapacity is scaled by smooth engine */
        bool BTPINT     : 1; /**< BTP threshold has been crossed */
        // High byte, Low bit first
        uint8_t RSVD1   : 2; /**< Reserved */
        bool CFGUPDATE  : 1; /**< Gauge is in CONFIG UPDATE mode */
        uint8_t RSVD0   : 5; /**< Reserved */
    } reg;
    uint16_t full;
} BQ27220OperationStatus;

typedef union GaugingStatus{
    struct __reg2
    {
        // Low byte, Low bit first
        bool FD       : 1; /**< Full Discharge */
        bool FC       : 1; /**< Full Charge */
        bool TD       : 1; /**< Terminate Discharge */
        bool TC       : 1; /**< Terminate Charge */
        bool RSVD0    : 1; /**< Reserved */
        bool EDV      : 1; /**< Cell voltage is above or below EDV0 threshold */
        bool DSG      : 1; /**< DISCHARGE or RELAXATION */
        bool CF       : 1; /**< Battery conditioning is needed */
        // High byte, Low bit first
        uint8_t RSVD1 : 2; /**< Reserved */
        bool FCCX     : 1; /**< fcc1hz clock going into CC: 0 = 1 Hz, 1 = 16 Hz*/
        uint8_t RSVD2 : 2; /**< Reserved */
        bool EDV1     : 1; /**< Cell voltage is above or below EDV1 threshold */
        bool EDV2     : 1; /**< Cell voltage is above or below EDV2 threshold */
        bool VDQ      : 1; /**< Charge cycle FCC update qualification */
    } reg;
    uint16_t full;
} BQ27220GaugingStatus;

    void bq27220_init();
 // get
    uint16_t bq27220_getDeviceNumber(void);  // sub-commands
    uint16_t bq27220_getVoltage(void);
    int16_t bq27220_getCurrent(void);
    void bq27220_getControlStatus(BQ27220ControlStatus *ctrl_sta);
    void bq27220_getBatteryStatus(BQ27220BatteryStatus *batt_sta);
    void bq27220_getOperationStatus(BQ27220OperationStatus *oper_sta);
    void bq27220_getGaugingStatus(BQ27220GaugingStatus *gauging_sta);
    float bq27220_getTemperature(void);
    uint16_t bq27220_getFullChargeCapacity(void);
    uint16_t bq27220_getDesignCapacity(void);
    uint16_t bq27220_getRemainingCapacity(void);
    uint16_t bq27220_getStateOfCharge(void);
    uint16_t bq27220_getStateOfHealth(void);
    uint16_t bq27220_getChargeVoltageMax(void);


#ifdef __cplusplus
}
#endif

#endif
