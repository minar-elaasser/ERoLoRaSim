
#ifndef __EROLORASIMDESKTOP_INTERFERER_H_
#define __EROLORASIMDESKTOP_INTERFERER_H_

#include <omnetpp.h>
#include "Server.h"
#include "InterferencePacket_m.h"

using namespace omnetpp;

namespace aloha {

/**
 * Interference Node - See NED file for more info
 */
class Interferer : public cSimpleModule
{
private:
    cModule *server;
    // position on the canvas, unit is m
    double minR;
    double netR;
    cPar *rndR;
    cPar *rndTheta;
    int sfNetR;

    double r, theta;
    double x, y;

    simtime_t radioDelay;
    const double propagationSpeed = 299792458.0;

    cMessage *endEvent;

//    enum { IDLE = 0, TRANSMIT = 1} state;

    //double installedLayers[4][4] = {{0.8,10,100,5},{0.15,10,20,5},{0.04,10,10,100},{0.01,10,1000,30}};
    // double installedLayers[4][4] = {{100,10,100,5},{500,10,20,5},{600,10,10,100},{50,10,1000,30}};
    //int txLayerIndex;
    simtime_t nextTransmissionTime;

    long interPkCounter;


    double meanArrivalRate;
    double txPower;
    double B;
    double txLength;
    int scalingFactor;

    double arrivalRate; //calculated
    simtime_t iaTime;

    int Gant;
    double K;
    cPar *interShadowingParamater;
    double pathlossExp;
    int BW;


public:
    Interferer();
    virtual ~Interferer();

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual void refreshDisplay() const override;

    simtime_t setNextTransmission();



};

} //namespace

#endif
