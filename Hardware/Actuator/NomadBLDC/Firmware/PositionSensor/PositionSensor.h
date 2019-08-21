#ifndef POSITIONSENSOR_H
#define POSITIONSENSOR_H
class PositionSensor {
public:
    virtual void Sample(float dt) = 0;
    virtual float GetMechPosition() {return 0.0f;}
    virtual float GetMechPositionFixed() {return 0.0f;}
    virtual float GetElecPosition() {return 0.0f;}
    virtual float GetMechVelocity() {return 0.0f;}
    virtual float GetElecVelocity() {return 0.0f;}
    virtual void ZeroPosition(void) = 0;
    virtual int GetRawPosition(void) = 0;
    virtual void SetElecOffset(float offset) = 0;
    virtual int GetCPR(void) = 0;
    virtual  void WriteLUT(int new_lut[128]) = 0;
};
  
  
class PositionSensorEncoder: public PositionSensor {
public:
    PositionSensorEncoder(int CPR, float offset, int ppairs);
    virtual void Sample(float dt);
    virtual float GetMechPosition();
    virtual float GetElecPosition();
    virtual float GetMechVelocity();
    virtual float GetElecVelocity();
    virtual void ZeroPosition(void);
    virtual void SetElecOffset(float offset);
    virtual int GetRawPosition(void);
    virtual int GetCPR(void);
    virtual  void WriteLUT(int new_lut[128]); 
private:
    InterruptIn *ZPulse;
    DigitalIn *ZSense;
    //DigitalOut *ZTest;
    virtual void ZeroEncoderCount(void);
    virtual void ZeroEncoderCountDown(void);
    int _CPR, flag, rotations, _ppairs, raw;
    //int state;
    float _offset, MechPosition, MechOffset, dir, test_pos, oldVel, out_old, velVec[40];
    int offset_lut[128];
};

class PositionSensorAM5147: public PositionSensor{
public:
    PositionSensorAM5147(int CPR, float offset, int ppairs);
    virtual void Sample(float dt);
    virtual float GetMechPosition();
    virtual float GetMechPositionFixed();
    virtual float GetElecPosition();
    virtual float GetMechVelocity();
    virtual float GetElecVelocity();
    virtual int GetRawPosition();
    virtual void ZeroPosition();
    virtual void SetElecOffset(float offset);
    virtual void SetMechOffset(float offset);
    virtual int GetCPR(void);
    virtual void WriteLUT(int new_lut[128]);
private:
    float position, ElecPosition, ElecOffset, MechPosition, MechOffset, modPosition, oldModPosition, oldVel, velVec[40], MechVelocity, ElecVelocity, ElecVelocityFilt;
    int raw, _CPR, rotations, old_counts, _ppairs;
    SPI *spi;
    DigitalOut *cs;
    int readAngleCmd;
    int offset_lut[128];

};

#endif