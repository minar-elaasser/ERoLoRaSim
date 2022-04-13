
#ifndef __LORA_HOST_H_
#define __LORA_HOST_H_


#include <omnetpp.h>
#include "Server.h"
#include "loraPacket_m.h"

using namespace omnetpp;

namespace aloha {

/**
 * LoRa host; see NED file for more info.
 */
class Host : public cSimpleModule
{
private:
    // parameters

    simtime_t radioDelay;
    double txRate;
    cPar *pkLenBits;
    cPar *iaTime;
    double frameLenBits;
    simtime_t slotTime;
    bool isSlotted;
    int bufferedPkts; //assuming infinite queue length at host
    bool isBufferingEvents;
    bool addFrameHeader;
    bool dutyCycleActive;

    // position on the canvas, unit is m
    double minR;
    cPar *rndR;
    cPar *rndTheta;

    double x, y;
    double r, theta;
    //SNR
    cPar *shadowingParamater;
    double SNR;
    double pathlossExp;


    //LoRa configuration parameters
    int hostType; //0=conventionalLoRaHost 1=ICSLoRaHost
    int  SF;
    int nbOfChannels;
    int BW;

    //header parameters
    int CRC;
    int IH ;
    double CR ;
    int DE ;
    int Nprog;
    simtime_t TOA;

    //double exhaustiveBitPatternMatrix[3][6];

    double dutyCycle[5] = {0.01,0.01,0.001,0.1,0.01}; //duty cycle per sub-band
    double installedChannels[16][3] = {{1,868.1},{1,868.3},{1,868.5},{0,867.1},{0,867.3},{0,867.5},{0,867.7},{0,867.9}};
    int nbOfBusyChannels;

    // state variables, event pointers etc
    cModule *server;
    cMessage *endEvent;
    enum { IDLE = 0, TRANSMIT = 1, WAITINGFORCH = 2 } state;
    int pkCounter;

    simtime_t nextTransmissionTime;

    // speed of light in m/s
    const double propagationSpeed = 299792458.0;

    // animation parameters
    //    const double ringMaxRadius = 2000; // in m
    //    const double circlesMaxRadius = 1000; // in m
    double idleAnimationSpeed;
    double transmissionEdgeAnimationSpeed;
    double midtransmissionAnimationSpeed;

    // figures and animation state
    cPacket *lastPacket = nullptr; // a copy of the last sent message, needed for animation
    mutable cRingFigure *transmissionRing = nullptr; // shows the last packet
    mutable std::vector<cOvalFigure *> transmissionCircles; // ripples inside the packet ring

public:
    Host();
    virtual ~Host();



protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    simtime_t getNextTransmissionTime();
    int selectAvailableChannel(simtime_t TOA);

    double getMinEndTOff();
    //void freeInstalledChannelsBeforeTime(simtime_t);

    void printInstalledChannelsTable();




};

}; //namespace

#endif

