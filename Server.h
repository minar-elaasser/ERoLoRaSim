#ifndef __LORA_SERVER_H_
#define __LORA_SERVER_H_

#include <omnetpp.h>
#include <cmath>
#include <vector>

#include "loraPacket_m.h"
#include "InterferencePacket_m.h"

//#include "MatlabEngine.hpp"
//#include "MatlabDataArray.hpp"

using namespace omnetpp;

namespace aloha{

/**
 * LoRa gateway and server; see NED file for more info.
 * TODO: documentation
 */


class Server : public cSimpleModule
{
  private:
    // state variables, event pointers

    cMessage *endRxEvent;
    simtime_t endReceptionTime;
    simtime_t recvStartTime;
    int chFreqIndex;
    int chSFIndex;

    double receiveCounter;
    double countOfThreePacketsInSystem;
    double countOfTwoPacketsInSystem;
    double countOfOnePacketsInSystem;
    long receiveInterCounter;

    long rcvCounter[7]={0};
    long currentCollisionNumFrames[7]={0};
    long noOfCollidedPackets[7]={0};
    simtime_t timeInEachState[10] ={0};
    simtime_t lastAddRemoveRxEvent=0;

    long successPackets[7]={0};
    double successRate[7]= {0};
    double successProb[7]={0};

    cLongHistogram SIR1Stats;
    cOutVector SIR1Vector;


    struct receptionEvents
    {
        int chFreqIndex; //par(0)
        int chSFIndex; //par(1)
        simtime_t endReceptionTime ;
        double SNR; //par(2)
        bool collidedFlag; //par(3) 0= false(Not Corrupted) 1=Corrupted
        int pktType;//par(4) 0=conventionalLoRa, 1=ICSLoRa, 100=Inter-system Interference
    };

    std::vector<receptionEvents> buffRxEvents, filteredBuffRxEvents;

    enum { IDLE = 0, TRANSMISSION = 1, COLLISION = 2 };


    int Ptx_ed;
    int Gant;
    double K;
    double NF;
    double Pnoise;
    double pathlossExp;
    int BW;
    int BER;
    cPar *shadowingParameter;


    double netR;
    double numHosts;
    double minR;


    double* pTotalAreaSFTier;
    double* pRelativeDensity;

    double assignmentEstimateServiceRadius[6]={0,0,0,0,0,0};

    long* pNoOfHostsInEachSFTier;
    int nbOfChannels;

    double installedServerChannels[8][7] = {{868.1},{868.3},{868.5},{867.1},{867.3},{867.5},{867.7},{867.9}};

    double generatedSelectedCRPerSF[3][6] = {{0,0,0,0,0,0},{0,0,0,0,0,0},{1,1,1,1,1,1}}; //equivalant to all EDs using 4/5

    float snrOfThreePacketsInSystem[3204][3] = {0};
    int j = 0;

//    double requiredSNR[6] = {-6,-9,-12,-15,-17.5,-20}; // provided by LoRa Specifications 1.1
    double coverageSNR[6] = {-6.35, -9.15, -12, -14.8, -17.65, -20};//gamma coverage: dr.tallal's values for BER <=10^-5


    double CRValues[3] = {1,0.9,0.8};

    double usedServiceRadius[6] = {0,0,0,0,0,5000};
  public:
    Server();
    virtual ~Server();

    int getSF(double distance);
    double pEstimateServiceRadius[6]={0,0,0,0,0,0};
    double pSFPercentageInEachTier[6][6]={0,0,0,0,0,0};
    double requiredSNR[6] = {3.65,0.85,-2,-4.8,-7.65,-10.55};// //gamma M = gamma coverage +10 dB : statisfies 90% coverage probability
    double getCR(int SF);

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual void refreshDisplay() const override;

    void estimateServiceRadius();


    double *totalAreaSFTier(double estimateServiceRadius[]);
    double *relativeDensity(double totalAreaSFTier[]);

    void addRxEvent(int chFreqIndex, int chSFIndex, simtime_t endReceptionTime, double SNR, bool collidedFlag, int pktType);
    void removeRxEvent(std::vector<receptionEvents> &buffRxEvents, simtime_t endReceptionTime);
    void printInstalledServerChannelsTable();
    void printRxEventsBuffer();
    void printFilteredRxEventsBuffer();
    void printRcvCounter();
    void printNoOfCollidedPackets();
    void printNoOfSuccessPackets();
    void printSuccessRate();
    void printSuccessProb();
    void printUsedCRPattern();
    void printUsedServiceRadius();
    void printEstimatedServiceRadius();
    void printAssignmentEstimateServiceRadius();

    void printSnrOfThreePacketsInSystem();

    double getReceiverSensitivty(int SF);

    double getSNRThreshold(double BER, int SF, double SIR);
    double getICSSNRThreshold(double BER, int SF, double SIR);

//    void algorithm();
//    long *noOfHostsInEachSFTier();
//    int serverGetSF(double distance);




};

}; //namespace

#endif

