#include "CANcontrol.h"
#include <cstdint>
#include <mbed.h>

// 受信割り込み関連Class CANReception
CANReception::CANReception(char *fpData, unsigned char DataLength, PinName RD, PinName TD, int Hz) : RawCAN(TD, RD, Hz) {
    fpMainData = fpData;
    len = DataLength;
    RawCAN::attach(Callback<void ()> (this, &CANReception::CANInterruptFunc), RawCAN::RxIrq);
}

CANReception::CANReception(char *fpData, PinName RD, PinName TD, int Hz) : RawCAN(TD, RD, Hz) {
    fpMainData = fpData;
    RawCAN::attach(Callback<void ()> (this, &CANReception::CANInterruptFunc), RawCAN::RxIrq);
}

CANReception::CANReception(PinName RD, PinName TD, int Hz) : RawCAN(TD, RD, Hz) {
    RawCAN::attach(Callback<void ()> (this, &CANReception::CANInterruptFunc), RawCAN::RxIrq);
}

int CANReception::setDataPointer(char *fpData){
    fpMainData = fpData;
    return 0;
}

int CANReception::printOriginalData(){
    printf("ID:0x%3x Data: 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x\n", RXID, RXdata[0], RXdata[1], RXdata[2], RXdata[3], RXdata[4], RXdata[5], RXdata[6], RXdata[7]);
    return 0;
}

int CANReception::printConvertedData(){
    if(RXID>=0x700 || RXdata[0]==0x90){ // MDのデータを受信したとき
        uint8_t duty = RXdata[1];
        float RPM = RXdata[2]*std::powf(10.0f, RXdata[3]);
        float Current = RXdata[4]*std::powf(10.0f, RXdata[5]);
        printf("MDID:0x%3x duty:%3d RPM:%8.2f Current:%2.1f\n", RXID, duty, RPM, Current);
    }else{
        printf("ID:0x%3x Data: 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x\n", RXID, RXdata[0], RXdata[1], RXdata[2], RXdata[3], RXdata[4], RXdata[5], RXdata[6], RXdata[7]);
    }
    return 0;
}

void CANReception::CANInterruptFunc(){
    if(RawCAN::read(msg)){
        RXID = msg.id;
        for(int i=0; i<8; i++){
            if(fpMainData != NULL){
                *(fpMainData+i) = msg.data[i];
            }
            RXdata[i] = msg.data[i];
        }
    }
}

CANcontrolMD::CANcontrolMD(unsigned int MyID, PinName TD, PinName RD, int Hz) : RawCAN(RD, TD, Hz) {
    TXID = MyID;
}

CANcontrolMD::CANcontrolMD(PinName TD, PinName RD, int Hz) : RawCAN(RD, TD, Hz){}

int CANcontrolMD::setconfig(unsigned int MyID, uint8_t RadixPeriod_ms, uint8_t IndexPeriod, AMT102_ResolutionType Resolution, OPAMP_PGAgain PGAgain){
    TXID = MyID;
    TXdata[0] = 0x00;
    TXdata[1] = RadixPeriod_ms;
    TXdata[2] = IndexPeriod;
    TXdata[3] = Resolution;
    TXdata[4] = PGAgain;
    TXdata[5] = 0;
    TXdata[6] = 0;
    TXdata[7] = 0;

    write(CANMessage(TXID,TXdata,8,CANData,CANStandard));   // 通信に失敗したときの処理を書くべき？

    return 0;
}

int CANcontrolMD::sendDutyLAP(char Duty){
    TXdata[0] = 0x10;
    TXdata[1] = 0;
    TXdata[2] = 0;
    TXdata[3] = 0;
    TXdata[4] = 0;
    TXdata[5] = 0;
    TXdata[6] = 0;
    TXdata[7] = Duty;

    write(CANMessage(TXID,TXdata,8,CANData,CANStandard));

    return 0;
}

int CANcontrolMD::sendDutySM(bool direction, uint8_t Duty){
    TXdata[0] = 0x10;
    TXdata[1] = 1;
    TXdata[2] = 0;
    TXdata[3] = 0;
    TXdata[4] = 0;
    TXdata[5] = 0;
    TXdata[6] = direction;
    TXdata[7] = Duty;

    write(CANMessage(TXID,TXdata,8,CANData,CANStandard));

    return 0;
}

int CANcontrolMD::sendRPM(bool direction, uint8_t RadixRPM, uint8_t IndexRPM){
    TXdata[0] = 0x11;
    TXdata[1] = direction;
    TXdata[2] = RadixRPM;
    TXdata[3] = IndexRPM;
    TXdata[4] = 0;
    TXdata[5] = 0;
    TXdata[6] = 0;
    TXdata[7] = 0;

    write(CANMessage(TXID,TXdata,8,CANData,CANStandard));

    return 0;
}

int CANcontrolMD::sendAngularVelocity(bool direction, uint8_t RadixAngularVelocity, uint8_t IndexAngularVelocity){
    TXdata[0] = 0x12;
    TXdata[1] = direction;
    TXdata[2] = RadixAngularVelocity;
    TXdata[3] = IndexAngularVelocity;
    TXdata[4] = 0;
    TXdata[5] = 0;
    TXdata[6] = 0;
    TXdata[7] = 0;

    write(CANMessage(TXID,TXdata,8,CANData,CANStandard));

    return 0;
}

int CANcontrolMD::sendCurrent(bool direction, uint8_t RadixCurrent, uint8_t IndexCurrent){
    TXdata[0] = 0x11;
    TXdata[1] = direction;
    TXdata[2] = RadixCurrent;
    TXdata[3] = IndexCurrent;
    TXdata[4] = 0;
    TXdata[5] = 0;
    TXdata[6] = 0;
    TXdata[7] = 0;

    write(CANMessage(TXID,TXdata,8,CANData,CANStandard));

    return 0;
}