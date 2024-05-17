#include "Arduino.h"
#include "tdk_ussm_defs.h"
class TDK_USSM
{
    public:
    TDK_USSM(int ioPin);                       //Constructor for single sensor with single pin operated in IO mode. 
    TDK_USSM(int txPin,int rxPin);             //Constructor for single sensor mode (tx pin , rx pin). 
    TDK_USSM(const int ioPins[],int n, int txLow=LOW, int txHigh=HIGH, int rxPinMode=INPUT_PULLUP, int rxLowUs=TBIT0_CMP);   //Constructor for multiple sensors full config
    TDK_USSM(const int txPin[],const int rxPin[],int n, const int anaPin[]=NULL, int txLow=LOW, int txHigh=HIGH, int rxPinMode=INPUT_PULLUP, int rxLowUs=TBIT0_CMP);   //Constructor for multiple sensors full config
   ~TDK_USSM();                               //Destructor

    //-- Init function
    void Init(const int txPin[],const int rxPin[], int n, const int anaPin[]=NULL, int txLow=LOW, int txHigh=HIGH, int rxPinMode=INPUT, int rxLowUs=TBIT0_CMP); //for constructor or on Runtime
    void InitDrive(int txLow=LOW, int txHigh=HIGH, int rxPinMode=INPUT, int rxLowUs=TBIT0_CMP); //for constructor or on Runtime

    //-- HAL IO Functions
    void inline HalTxInit(int val, int mask=WAVEGEN_BUS_MASK) ;
    void inline HalTxWrite(int val, int mask=WAVEGEN_BUS_MASK) ;
    void inline HalTxBusWrite(const int txPin[], int val, int nbit) ;
    void (*pHalTxBusWrite)(const int txPin[], int val, int nbit);
    void inline HalRxInit(int mask=WAVEGEN_BUS_MASK) ; 
    int  inline HalRxRead(int mask=WAVEGEN_BUS_MASK, int defaultVal=1) ;
    int  inline HalRxAnaRead(int n, int rightShift=0) ;
    int  inline HalRxAnaRead(uint8_t *buf, int mask=WAVEGEN_BUS_MASK, int rightShift=0) ;
    int         HalDmaStart_CallBack(int mode, int devices, uint8_t *anaBuf=NULL, int anaBufSize=0, int n_sample=0, int sample_time_us=0) ;
    int         HalDmaStop_CallBack(void) ;
    void        HalSetTimerFreqHz(uint32_t val);


    //-- Basic Time of Flight Measurement
    long GetTimeOfFlight()      const; // returns time-of-flight in us of sensor 0
    long GetTimeOfFlight(int n) const; // returns time-of-flight in us of sensor n
    long GetTimeOfFlight(int txPin, int rxPin) const; //returns time-of-flight in us of sensor connected to txPin / rxPin

    //-- Basic Distance Measurement
    long GetDistanceMm()        const; // returns distance in mm of sensor 0
    long GetDistanceMm(int n)   const; // returns distance in mm of sensor n
    long GetDistanceMm(int txPin, int rxPin)   const; // returns distance in mm of sensor n
    
    long GetDistanceCm()        const; // returns distance in cm of sensor 0
    long GetDistanceCm(int n)   const; // returns distance in cm of sensor n
    long GetDistanceCm(int txPin, int rxPin)   const; // returns distance in mm of sensor n
    
    long GetDistanceInch()      const; // returns distance in inch of sensor 0
    long GetDistanceInch(int n) const; // returns distance in inch of sensor n
    long GetDistanceInch(int txPin, int rxPin) const; // returns distance in inch of sensor n


    //-- Wave Management Functions
    long WaveEncode(const int cmdId, eWaveStruct_t *rxWave, int rxWaveSize, int deviceMask=1, int masterMask=WAVEGEN_BUS_MASK);
    long WaveSendReceive(int deviceMask, int masterMask=WAVEGEN_BUS_MASK, int cmdId=CMD_SEND_RECEIVE_A, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, int skipTx=0, int timeDelayUs=-1);
    long WaveGenCapture(eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, int txrxWaveAppend=1, const eWaveGenCmd *inCmdWave=NULL, uint8_t *anaBuf=NULL, int anaBufize=0, int n_sample=0, int sample_time_us=0, uint8_t *txBits=NULL, int nTxBits=0, int devices=0x1, int masters=0x1);
    long WaveDecodeData(int n, const int cmdId=CMD_SEND_RECEIVE_A, const int statusCfg=2, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, eSensorData *rxData=NULL, int rxByteSize=TDK_USSM_DATA_SIZE, int maxEchos=TDK_USSM_DEFAULT_MAX_ECHOS, uint32_t dmaMask=0ul);
    long WaveDecodeBytes(int n, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, int stopPos=0, eSensorData *rxData=NULL, int rxByteSize=TDK_USSM_DATA_SIZE);


    //-- Advanced ToF measurement    
    long CalPulse(int deviceMask) ;

    long SendA(int deviceMask) ;
    long SendB(int deviceMask) ;
    long SendC(int deviceMask) ;

    long ReceiveA(int deviceMask) ;
    long ReceiveB(int deviceMask) ;
    long ReceiveC(int deviceMask) ;

    long SendReceiveA(int deviceMask, int masterMask) ;
    long SendReceiveB(int deviceMask, int masterMask) ;
    long SendReceiveC(int deviceMask, int masterMask) ;

    long EnvelopSendA(int deviceMask, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=10) ;
    long EnvelopReceiveA(int deviceMask, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=10) ;
    long EnvelopSendReceiveA(int deviceMask, int masterMask, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=10) ;

    long JtagSendA(int deviceMask, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=20)  ;
    long JtagReceiveA(int deviceMask, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=20)  ;
    long JtagSendReceiveA(int deviceMask, int masterMask, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0,  uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=20)  ;
    long JtagEnvelopThreshold(int deviceMask, int masterMask, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=20)  ;
    long JtagThreshold(int deviceMask, eWaveStruct_t *rxWave=NULL, int rxWaveSize=0, uint8_t *pBuf=NULL, int bufSize=0, int nSample=512, int sampleTimeUs=20)  ;



    //-- Conversion Functions ToF to Distance
    long MicrosecondsToMillimeters(long microseconds) const; // Time of Flight to Millimeters 
    long MicrosecondsToInches(long microseconds) const; // Time of Flight to Inches 
    long MicrosecondsToCentimeters(long microseconds) const; // Time of Flight to Centimeters 


    //-- Registers Read/Write
    long RegisterReadWrite (const int txPin, const int rxPin, const int cmdId, uint8_t *rxBytes, int nRxBits,uint8_t *txBytes=NULL, int nTxBits=0);   
    long ReadRegister   (const int cmdId, uint8_t *rxBytes, int nRxBits, const int n=0, eSensorRegisters *sensorRegisters=NULL) ; // returns total number of read bits of sensor n
    long WriteRegister  (const int cmdId, uint8_t *txBytes, int nTxBits, const int n=0, eSensorRegisters *sensorRegisters=NULL) ; // returns total number of written bits to sensor n
    int  SaveRegisterData(eSensorRegisters *pRegs, const int cmdId, uint8_t *pBytes) ;

    long RegisterFieldSetGet (const int cmdId, int RegisterId, int fieldId, int n, uint8_t *getValue=NULL, uint32_t setValue=0) ;

    long RegisterFieldSetGet (int RegisterId, int fieldId, int n, uint32_t fieldValue=0) ;   
    long GetFieldValue(int RegisterId, int fieldId, uint8_t *regBytes, int n=0) ;
    long SetFieldValue(int RegisterId, int fieldId, uint8_t *regBytes, uint32_t value, int n=0) ;

    long CalibWrite     (uint8_t *txBytes,const int n=0) ;
    long CalibRead      (uint8_t *rxBytes,const int n=0) ;

    long MeasWrite      (uint8_t *txBytes,const int n=0) ;
    long MeasRead       (uint8_t *rxBytes,const int n=0) ;

    long ThresAWrite    (uint8_t *txBytes,const int n=0) ;
    long ThresARead     (uint8_t *rxBytes,const int n=0) ;

    long ThresBWrite    (uint8_t *txBytes,const int n=0) ;
    long ThresBRead     (uint8_t *rxBytes,const int n=0) ;  
    
    long ReadId         (uint8_t *rxBytes,const int n=0) ;      
    long ReadStatus     (uint8_t *rxBytes,const int n=0) ;    
    long ReadTemp       (uint8_t *rxBytes,const int n=0) ;  
    long ReadNfdStatus  (uint8_t *rxBytes,const int n=0) ;  
    long ReadEeprom     (uint8_t *rxBytes,const int n=0) ; 

    long Standby        (const int n=0) ;
    long Wakeup         (const int n=0) ; 
    long EepromProg     (const int progPin, const int n=0, const int progON=HIGH, const int prgOFF=LOW) ; 
     

    //-- Helper Functions
    eIOState  GetIoState(eWaveStruct_t *pwave, uint32_t pos, uint8_t bit_id) const;
    int       GetBit(uint8_t *buf, uint32_t bit_id) const;
    int       SetBit(uint8_t *buf, uint32_t bit_id, int val) const;
    uint32_t  GetBitsU32(uint8_t *src, uint32_t src_lsb, uint32_t nbit) const;
    uint32_t  SetBitsU32(uint8_t *dest, uint32_t dest_lsb, uint32_t nbit, uint32_t val32) const;
    int       GetBitRange(uint8_t *dest, uint32_t dest_lsb, uint8_t *src, uint32_t src_lsb, uint32_t nbit) const;
    uint32_t  GetBitsNbyte(uint32_t nbits) const;
    int       GetEvenParity(uint8_t *pBuf, uint32_t lsb, uint32_t msb) const;
 
    bool      GetValOfRealVal(const eLUTStruct *pLut, uint32_t *pVal, float real_val) const;
    bool      GetRealValOfVal(const eLUTStruct *pLut, float *pRealVal, uint32_t val, char *pDesc) const;

    bool      IsSendReceiveCommand(const int cmd_id) const;
    bool      IsSendReceiveA(const int cmd_id) const;
    bool      IsValidCommand(int cmd_id) const;

    inline int DeviceToMask(int n); // Easy conversion from single device to Device Mask

    //-- Public Runtime Variables
    eSimpleCmdStruct    cmd;
    eWaveStruct_t       waveBuf[TDK_USSM_WAVE_BUF_SIZE];
    const int           waveBufSize = TDK_USSM_WAVE_BUF_SIZE;
    eSensorData         dataBuf;
    int                 _rx_buf_ptr;
    int                 _tx_buf_ptr;
    
    int                 TimeAdjustUs; 
 
    /************************
     * Private Section
     * */
    private:
    
    //-- Sensors Pins and count
    int *txPins;                    // txPin list
    int *rxPins;                    // rxPin list
    int *anaPins;                   // anaPins list
    int nSensors;                   // Number of sensors
    
    //-- Signalling Mode
    int  IO_DRIVE_LOW;
    int  IO_RELEASE;
    int  RX_TBIT_LOW_US;
    int  IO_IN_MODE;
    
 private:
    //-- HAL Variables
    uint32_t            _hal_timer_freq_hz;
};
