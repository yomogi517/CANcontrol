#ifndef CANcontrol_H
#define CANcontrol_H

#include <cstdint>
#include <mbed.h>

class CANReception : public RawCAN {
    public:
        CANReception(char *fpData, unsigned char DataLength, PinName RD, PinName TD, int Hz);
        CANReception(char *fpData, PinName RD, PinName TD, int Hz);
        CANReception(PinName RD, PinName TD, int Hz);
        int setDataPointer(char *fpData);
        int printOriginalData();
        int printConvertedData();

    private:
        CANMessage msg;
        unsigned int RXID;
        char RXdata[8];
        char *fpMainData = NULL;
        unsigned char len = 8;
        CANType type = CANData;
        CANFormat format = CANStandard;

        void CANInterruptFunc();
};

class CANcontrolMD : public RawCAN {
    public:
        enum AMT102_ResolutionType{
            _2048 = 0,
            _1024 = 1,
            _1000 = 2,
            _800 = 3,
            _512 = 4,
            _500 = 5,
            _400 = 6,
            _384 = 7,
            _256 = 8,
            _250 = 9,
            _200 = 10,
            _192 = 11,
            _125 = 12,
            _100 = 13,
            _96 = 14,
            _48 = 15
        };

        enum OPAMP_PGAgain{
            _4 = 0,
            _8 = 1
        };

        CANcontrolMD(unsigned int MyID, PinName RD, PinName TD, int Hz);
        CANcontrolMD(PinName RD, PinName TD, int Hz);
        int setconfig(unsigned int MyID, uint8_t RadixPeriod_ms, uint8_t IndexPeriod, AMT102_ResolutionType Resolution, OPAMP_PGAgain PGAgain);
        int sendDutyLAP(char Duty);
        int sendDutySM(bool direction, uint8_t Duty);
        int sendRPM(bool direction, uint8_t RadixRPM, uint8_t IndexRPM);
        int sendAngularVelocity(bool direction, uint8_t RadixAngularVelocity, uint8_t IndexAngularVelocity);
        int sendCurrent(bool direction, uint8_t RadixCurrent, uint8_t IndexCurrent);
        
    private:
        CANMessage msg;
        unsigned int TXID;
        unsigned char TXdata[8];
        unsigned char len = 8;
        CANType type = CANData;
        CANFormat format = CANStandard;

};

#endif