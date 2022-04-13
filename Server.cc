#include "Server.h"
#include <algorithm>
namespace aloha {

Define_Module(Server);

Server::Server()
{
    endRxEvent = nullptr;
}

Server::~Server()
{
    cancelAndDelete(endRxEvent);
    //delete pEstimateServiceRadius;
    delete pTotalAreaSFTier;
    delete pRelativeDensity;
}

void Server::initialize()
{
    gate("in")->setDeliverOnReceptionStart(true);

    //initializing endRxEvent
    endRxEvent = new cMessage("end-reception");
    endRxEvent->addPar("chFreqIndex");
    endRxEvent->addPar("chSFIndex");
    endRxEvent->addPar("SNR");
    endRxEvent->addPar("collidedFlag");
    endRxEvent->addPar("pktType");


    //displaying server in the GUI
    getDisplayString().setTagArg("p", 0, par("x").doubleValue());
    getDisplayString().setTagArg("p", 1, par("y").doubleValue());

    //initializing parameters from NED files
    netR = par("netR");
    numHosts = par("numHosts");
    minR = par("minR");
    EV << "totalNumHosts: "  << numHosts <<endl;
    nbOfChannels = par("nbOfChannels");

    Ptx_ed = par("Ptx_ed");
    Gant= par("Gant");
    K= par("K");
    NF= par("NF");
    BW = par("BW");
    pathlossExp = par("pathlossExp");
    BER = par("BER");
    shadowingParameter = &par("shadowingParameter");

    Pnoise =  -117;//-120.01;// Noise Power (dB) Pnoise = 174 - (10*std::log10(BW)) - NF

    //initializing statistical parameters
    receiveCounter = 0;
    countOfThreePacketsInSystem = 0;
    countOfTwoPacketsInSystem = 0;
    countOfOnePacketsInSystem = 0;
    receiveInterCounter = 0;
    WATCH(receiveInterCounter);
    WATCH(countOfThreePacketsInSystem);
    WATCH(countOfTwoPacketsInSystem);
    WATCH(countOfOnePacketsInSystem);

    estimateServiceRadius();
    pTotalAreaSFTier = totalAreaSFTier(pEstimateServiceRadius);
    pRelativeDensity = relativeDensity(pTotalAreaSFTier);

    //    pNoOfHostsInEachSFTier =noOfHostsInEachSFTier();
    //    noOfHostsInEachSFTier();

    //you want these lines to be drawn once only at initialize phase!
    for(int i = 5; i>=0; i--)
    {
        char const *pchar = ("r"+ std::to_string(i)).c_str();
        getDisplayString().setTagArg(pchar, 0, pEstimateServiceRadius[i]);
    }

    //    getDisplayString().setTagArg("r4", 0, minR);
    //    printEstimatedServiceRadius();

}

void Server::handleMessage(cMessage *msg)
{

    loraPacket *loraPkt = dynamic_cast<loraPacket *>(msg);
    InterferencePacket *interPkt = dynamic_cast<InterferencePacket *>(msg);

    //    findSFPercentageInEachTier();
    if (msg->isSelfMessage()) { //multiple endRxEvents received from different streams!! (each endRxEvent has different parameters)
        int msgRxChFreqIndex = msg->par(0).doubleValue()  ;
        int msgRxChSFIndex = msg->par(1).doubleValue();
        simtime_t msgRxChEndRxTime = msg->getTimestamp();
        //simtime_t msgRxChEndRxTime = msg->par(2).doubleValue();
        //EV << "SELF MESSAGE : " <<endl;

        if (msgRxChFreqIndex != 100) {//not an interference signal endEvent
            //EV<< "reception finished on channel: " << msgRxChFreqIndex<<"-" <<msgRxChSFIndex << "@sim time: " << simTime()<< "same as stamped time?:) " << msg->getTimestamp()<< "\n";
            installedServerChannels[msgRxChFreqIndex][msgRxChSFIndex]--;
            if (currentCollisionNumFrames[msgRxChSFIndex] >0)
                currentCollisionNumFrames[msgRxChSFIndex] --;
            if (installedServerChannels[msgRxChFreqIndex][msgRxChSFIndex] < 0)
                throw cRuntimeError ("Error using installed server channels - Simulation Error!");
            printInstalledServerChannelsTable();
        }
        else { //its an interference signal endEvent
          //EV << "msg->par(2): disaplying the linear power of interference packet:: " << msg->par(2).doubleValue() << endl;
            //EV<< "Pnoise_dB: " << Pnoise << endl;
            //            //update the Pnoise linearly then in dB
            double Pnoise_Lin = (pow(10,(Pnoise/10))); //noise linear
            //EV<< "Pnoise_Lin: " << Pnoise_Lin << endl;
            double Pinternoise_Lin = Pnoise_Lin - msg->par(2).doubleValue(); //removing interference noise from total Pnoise linearly
            //EV<< "Pinternoise_Lin: " << Pinternoise_Lin << endl;
            double Pinternoise_dB = 10*std::log10(Pinternoise_Lin); //
            //EV<< "Pinternoise_dB: " << Pinternoise_dB << endl;
            Pnoise = Pinternoise_dB;
            //EV<< "Pnoise " << Pnoise<< endl;
        }

        //EV<< "HERE WE ARE CHECKING THE BUFFER SIZE- buffRxEvents.size()=" << buffRxEvents.size() << endl;
        //EV<< "                                    - timeInEachState[buffRxEvents.size()]=" << timeInEachState[buffRxEvents.size()] << endl;
        //EV<< "                                    - simTime()=" << simTime() << endl;
        //EV<< "                                    - lastAddRemoveRxEvent=" << lastAddRemoveRxEvent << endl;
        //EV<< "                                    - timeInEachState[buffRxEvents.size()] + (simTime() - lastAddRemoveRxEvent)=" << timeInEachState[buffRxEvents.size()] + (simTime() - lastAddRemoveRxEvent)<< endl;
        timeInEachState[buffRxEvents.size()] = timeInEachState[buffRxEvents.size()] + (simTime() - lastAddRemoveRxEvent);
        lastAddRemoveRxEvent = simTime();
        for (int i = 0; i <10; i++)
        {
            //EV<< "timeInEachState[" <<i <<"]:" <<  timeInEachState[i] << endl;;
        }
        //EV<< "Deleting the entry from the event table:)" <<endl;
        removeRxEvent(buffRxEvents, msgRxChEndRxTime);

        printRxEventsBuffer();

        if (!buffRxEvents.empty())
        {
            cancelEvent(endRxEvent);
            endRxEvent->par(0) = (buffRxEvents).front().chFreqIndex;
            endRxEvent->par(1) = (buffRxEvents).front().chSFIndex;
            endRxEvent->setTimestamp((buffRxEvents).front().endReceptionTime);
            endRxEvent->par(2) = (buffRxEvents).front().SNR;
            endRxEvent->par(3) = (buffRxEvents).front().collidedFlag;
            //EV<< "Prior scheduled endRxEvents in buff, rescheduling the next one @ t= " << (buffRxEvents).front().endReceptionTime <<endl;
            scheduleAt((buffRxEvents).front().endReceptionTime, endRxEvent);
        }

        //        if (currentCollisionNumFrames[msgRxChSFIndex] >0)
        //        currentCollisionNumFrames[msgRxChSFIndex] --;
        //currentCollisionNumFrames[msgRxChSFIndex] =0;
        //EV<< "currentCollisionNumOfFrames: " <<endl;
        for (int i =0; i<=6; i++)
        {
            //EV<<  currentCollisionNumFrames[i]<< "|";
        }
        //EV <<endl;
        //EV<< "receiveCounter:" << receiveCounter << endl;
        printNoOfCollidedPackets();
        printInstalledServerChannelsTable();
    }
    else{
        if (loraPkt){

            ASSERT(loraPkt->isReceptionStart());

            //channel on which this packet is received on: Ch. represented by a pair of used attributes (SF, ch. freq.)
            int pktSF = loraPkt->getSF();
            double pktChFreq = loraPkt->getChFreq();
            double pktSNR = loraPkt->getSNR();
            int pktType = loraPkt->getPktType();

            //EV<< "pktSNR: " << pktSNR <<endl;
            //EV<<  loraPkt->getName()<< "arrived using channel (SF:"  << pktSF<< "-chFreq:" << pktChFreq<< ")"<< "with SNR: " <<pktSNR <<  "on t= "<<loraPkt->getArrivalTime()  <<endl;
            //EV<< "Packet Type:" << loraPkt->getPktType() <<endl;
            //find this channel index on installed server channels - which are the attributes to endRxEvent msg to end that stream!
            bool findChCheck = false;
            int i = 0;
            while (findChCheck == false && i< nbOfChannels)
            {
                if (installedServerChannels[i][0] == pktChFreq)
                {
                    findChCheck = true;
                    chFreqIndex = i;
                    chSFIndex = pktSF - 6;
                }
                i++;
            }

            bool pktCollidedFlag =false;
            //EV<< "pktSNR  " <<  pktSNR <<endl;
            //EV<< "coverageSNR[chSFIndex-1] " << coverageSNR[chSFIndex-1]<<endl;

            //if (pktSNR < requiredSNR[chSFIndex -1])
            // this packet can't be correctly demodulated because its signal strength < coverageSNR
            if (pktSNR < coverageSNR[chSFIndex -1])
            {
                //EV<< "MARK IT HEREEEEEE " << endl;
                //EV<< "pktSNR < coverageSNR[chSFIndex -1] " <<endl;
                pktCollidedFlag = true;
                currentCollisionNumFrames[chSFIndex]++;
                noOfCollidedPackets[chSFIndex]++;
            }

            if (findChCheck == false)
                throw cRuntimeError("Channel Frequency not installed on this server");


            endReceptionTime = simTime() + loraPkt->getDuration();


            if (lastAddRemoveRxEvent !=0)
            {
                //EV<< "HERE WE ARE CHECKING THE BUFFER SIZE- buffRxEvents.size()=" << buffRxEvents.size() << endl;
                //EV<< "                                    - timeInEachState[buffRxEvents.size()]=" << timeInEachState[buffRxEvents.size()] << endl;
                //EV<< "                                    - simTime()=" << simTime() << endl;
                //EV<< "                                    - lastAddRemoveRxEvent=" << lastAddRemoveRxEvent << endl;
                //EV<< "                                    - timeInEachState[buffRxEvents.size()] + (simTime() - lastAddRemoveRxEvent)=" << timeInEachState[buffRxEvents.size()] + (simTime() - lastAddRemoveRxEvent)<< endl;

                timeInEachState[buffRxEvents.size()] = timeInEachState[buffRxEvents.size()] + (simTime() - lastAddRemoveRxEvent);
            }
            //EV<< "Adding recevingEvent in Buffer " << endl;
            addRxEvent(chFreqIndex, chSFIndex, endReceptionTime, pktSNR, pktCollidedFlag, pktType); //sorted according to minimum endReceptionTime of each event
            lastAddRemoveRxEvent = simTime();

            for (int i = 0; i <10; i++)
            {
                //EV<< "timeInEachState[" <<i <<"]:" <<  timeInEachState[i] << endl;;
            }

            receiveCounter++;
            rcvCounter[chSFIndex]++;
            //EV<< "receiveCounter: " << receiveCounter << endl;
            //EV<< "rcvCounter[chSFIndex]: " << rcvCounter[chSFIndex] << endl;
            //EV<< "noOfCollidedPackets[chSFIndex]: " << noOfCollidedPackets[chSFIndex] << endl;
            //EV<< "installedServerChannels[chFreqIndex][chSFIndex] " << installedServerChannels[chFreqIndex][chSFIndex]  << endl;
            printInstalledServerChannelsTable();
            printRxEventsBuffer();

            recvStartTime = simTime();

            if (installedServerChannels[chFreqIndex][chSFIndex] ==0) {
                    countOfOnePacketsInSystem++;

                //EV<< "Start receiving on channel:" <<chFreqIndex <<"-"<<chSFIndex <<"\n";
            }
            else if (installedServerChannels[chFreqIndex][chSFIndex]  > 0){ //==1){//==1) { //> 0) {//lower bound mode thing! //==1
                //Intra-system interference (from same loRa SF network)
//                throw cRuntimeError( "CHECK THIS EVENT");
                EV<< "Another frame arrived while receiving -- overlapping in time on this channel (" <<pktChFreq<< "/" << pktSF<< ")\n";
                int checkFreqIndex = chFreqIndex;
                int checkSFIndex = chSFIndex;
                int checkPktType = pktType;
                double SIR;
                double SIR2;
                double SNRThreshold;
                double SNRThreshold2;

                //EV<<"Extracting all entries with chFreqIndex: " << chFreqIndex << " and chSFIndex " << chSFIndex <<endl;

                //impact of incoming packet on all other packets!
                //deleting buffer and resizing to zero to free space
                filteredBuffRxEvents.clear();
                filteredBuffRxEvents.shrink_to_fit();
//                std::copy_if(buffRxEvents.begin(), buffRxEvents.end(), back_inserter(filteredBuffRxEvents),[checkFreqIndex, checkSFIndex, pktSNR](receptionEvents tmp) { return tmp.chFreqIndex ==checkFreqIndex && tmp.chSFIndex == checkSFIndex && tmp.collidedFlag == false && tmp.SNR != pktSNR   ; });
                std::copy_if(buffRxEvents.begin(), buffRxEvents.end(), back_inserter(filteredBuffRxEvents),[checkFreqIndex, checkSFIndex, pktSNR](receptionEvents tmp) { return tmp.chFreqIndex ==checkFreqIndex && tmp.chSFIndex == checkSFIndex && tmp.SNR != pktSNR   ; });
                printFilteredRxEventsBuffer();

                if (installedServerChannels[chFreqIndex][chSFIndex]  ==1)
                {
                    countOfTwoPacketsInSystem++;
//                                        throw cRuntimeError( "CHECK THIS EVENT");

                }

                if (installedServerChannels[chFreqIndex][chSFIndex]  ==2)
                {
//                    snrOfThreePacketsInSystem[int(countOfThreePacketsInSystem)][0] = pktSNR;
                    countOfThreePacketsInSystem++;
//                    EV << "countOfSimThreePacketsInBuffer= " << countOfThreePacketsInSystem << endl;
//                    EV << "SNR1:" << pktSNR <<endl;
//                    throw cRuntimeError( "CHECK THIS EVENT");
                }

                int j = 1;

                /////// SIR Theoratical ////////////
                double totalSNR= 0;
                EV<< "pktSNR" << pktSNR <<endl;
//                long pktSNRdB = 10*log(pktSNR);
//                EV<< "pktSNRdB" << pktSNRdB <<endl;
                double SIR2New = 0;
                double SIR1New = 0;

                for (auto it : filteredBuffRxEvents)
                {
                    totalSNR = totalSNR + it.SNR;
                }
                EV << "totalSNR"<< totalSNR << endl;

                SIR2New = pktSNR-totalSNR;

//                double SIRTEST = pow(10, (SIRdB/10));
                EV << "SIR2New"<< SIR2New <<endl;

                ///////////////////////////////////////////

                for (auto it : filteredBuffRxEvents)
                {
                    //EV << "Printing filteredBuffRxEvent entry:" <<endl;
                    //EV << it.chFreqIndex << " | " << it.chSFIndex << "| " << it.endReceptionTime << "| " << it.SNR << "| " <<it.collidedFlag << endl;
                    //EV<< "Effect of Incoming packet on others existing packets in buffer->>>" << endl;

                    SIR1New = it.SNR - totalSNR;
                    EV << "SIR1New"<< SIR1New <<endl;


                    SIR = it.SNR - pktSNR ;
                    SIR2 = pktSNR - it.SNR;

                    EV << "SIR" << SIR << endl;
                    EV << "SIR2" << SIR2 << endl;
                    if (installedServerChannels[chFreqIndex][chSFIndex]  ==2)
                    {
//                        snrOfThreePacketsInSystem[int(countOfThreePacketsInSystem-1)][j] = it.SNR;
//                                        EV << "SNR2thenSNR3:" << it.SNR <<endl;
                                        j++;
                    }
                    //EV << "SIR" << SIR<<endl;
                    //EV << "SIR2" << SIR2<<endl;
                    //EV << "it.pktType" << it.pktType <<endl;
                    //EV << "checkPktType" << checkPktType <<endl;
                    if (it.pktType == checkPktType) //impact of LoRaHost on LoRaHost and ICSHost on ICSHost (same pktType)
                        {
                        //EV << "SAME TYPE" << endl;
                        //EV<< "CHECKING ON SNR THRESHOLD FUNCTION" <<endl;
                        SNRThreshold = getSNRThreshold(BER, pktSF, SIR);
                        SNRThreshold2 = getSNRThreshold(BER, pktSF, SIR2);
                        }
                    else
                    {
                        //EV << "DIFFERNT TYPE" << endl;
                        SNRThreshold = getICSSNRThreshold(BER, pktSF, SIR);
                        SNRThreshold2 = getICSSNRThreshold(BER, pktSF, SIR2);
                    }

//                    EV << "HARDCODING SNRTHRESHOLDS FOR STRICT VALUES LIKE IN OTHER REFERENCES!" << endl;
//                    SNRThreshold = 1;
//                    SNRThreshold2 = 1;

                    //EV << "SNRTHRESHOLD" << SNRThreshold <<endl;
                    //EV << "SNRTHRESHOLD2" << SNRThreshold2 << endl;
                    //EV << "SIR" << SIR << endl;
                    //EV << "SNRThreshold" << SNRThreshold <<endl;
                    //EV << "it.SNR < SNRThreshold" << (it.SNR < SNRThreshold) << " && " << "it.collidedFlag == false"<< (it.collidedFlag == false) << endl;
                    //EV << "it.SNR < SNRThreshold" <<it.SNR << "<" << SNRThreshold << (it.SNR < SNRThreshold) << " && " << "it.collidedFlag == false"<< (it.collidedFlag == false) << endl;

                    if (it.SNR < SNRThreshold && it.collidedFlag == false)
                    {
                        //EV << "1ST IF" <<endl;
                        //EV << "it.SNR < SNRThreshold :: COLLIDED!!" <<endl;
                        //EV << "Entry Marked as Collided>> "  << endl;
                        it.collidedFlag = true; // change in filteredBufferedRxEvents
                        (*(std::find_if(buffRxEvents.begin(), buffRxEvents.end(), [&](receptionEvents const & tmp){return tmp.SNR == it.SNR;}))).collidedFlag = true; //changed in buffRxEvents
                        printRxEventsBuffer();
                        currentCollisionNumFrames[chSFIndex]++;
                        noOfCollidedPackets[chSFIndex]++;
                    }
                    //EV<< pktSNR << "?" << SNRThreshold2 <<" " <<(pktSNR < SNRThreshold2) <<"&&" << (pktCollidedFlag == false) << endl;
                    if (pktSNR < SNRThreshold2 && pktCollidedFlag == false)
                    {
                        //EV<< "2ND IF" <<endl;
                        //EV<< "it.SNR > SNRThreshold :: COLLIDED!!" <<endl;
                        //EV<< "Entry Marked as Collided>> "  << endl;
                        //EV<< "it.collidedFlag" << it.collidedFlag << endl;
                        it.collidedFlag = true; // change in filteredBufferedRxEvents
                        (*(std::find_if(buffRxEvents.begin(), buffRxEvents.end(), [&](receptionEvents const & tmp){return tmp.SNR == it.SNR;}))).collidedFlag = true; //changed in buffRxEvents
                        printRxEventsBuffer();
                        currentCollisionNumFrames[chSFIndex]++;
                        noOfCollidedPackets[chSFIndex]++;
                    }


                }


                //EV<< "rcvCounter[chSFIndex]" << rcvCounter[chSFIndex]<< endl;
                //EV<< "noOfCollidedPackets[chSFIndex] " << noOfCollidedPackets[chSFIndex] << endl;


                if (noOfCollidedPackets[chSFIndex] > rcvCounter[chSFIndex])
                    throw cRuntimeError( "no. of collided packets greater than total received.. Check this event - Simulation Error");

                printNoOfCollidedPackets();

                // update network graphics
                if (hasGUI()) {
                    char buf[32];
                    sprintf(buf, "Collision!");
                    //sprintf(buf, "Collision! %frames-%d-%d", installedServerChannels[chFreqIndex][chSFIndex] , pktChFreq, pktSF);
                    //sprintf(buf, "Collision! (%ld frames)", currentCollisionNumFrames);
                    bubble(buf);
                    // getParentModule()->getCanvas()->holdSimulationFor(par("animationHoldTimeOnCollision"));
                }
//            }
            ////////////LOWER BOUND MODE CONT'D ///////////////////////////////////////////////
//                        else if (installedServerChannels[chFreqIndex][chSFIndex] >=2)
//                        {
//                            EV << "2 interfering signals or more on same channel" << endl;
//                            int checkFreqIndex = chFreqIndex;
//                            int checkSFIndex = chSFIndex;
//
//                            ////////////////////////////
//                            double SIR;
//                            double SIR2;
//
////                            double SIR;
////                            double SNRThreshold;
//
////                            EV << "HARDCODING SNRTHRESHOLDS FOR STRICT VALUES LIKE IN OTHER REFERENCES!" << endl;
////                            SNRThreshold = 1;
//
//                            EV <<"Extracting all entries with chFreqIndex: " << chFreqIndex << " and chSFIndex " << chSFIndex <<endl;
//
//                            //deleting buffer and resizing to zero to free space
//                            filteredBuffRxEvents.clear();
//                            filteredBuffRxEvents.shrink_to_fit();
//                            //this iterator includes all entries with same SF & chFreq and not marked as collided (N.B.: this includes the incoming packet as well)
//                            EV << "pkt SNR: " << pktSNR << endl;
//                            std::copy_if(buffRxEvents.begin(), buffRxEvents.end(), back_inserter(filteredBuffRxEvents),[checkFreqIndex, checkSFIndex, pktSNR](receptionEvents tmp) { return tmp.chFreqIndex ==checkFreqIndex && tmp.chSFIndex == checkSFIndex && tmp.collidedFlag == false && tmp.SNR != pktSNR   ; });
//                            //std::copy_if(buffRxEvents.begin(), buffRxEvents.end(), back_inserter(filteredBuffRxEvents),[checkFreqIndex, checkSFIndex, pktSNR](receptionEvents tmp) { return tmp.chFreqIndex ==checkFreqIndex && tmp.chSFIndex == checkSFIndex && tmp.collidedFlag == false ; });
//
//                            printFilteredRxEventsBuffer();////////
//                            for (auto it : filteredBuffRxEvents)
//                            {
//                                it.collidedFlag = true; // this command doesn't overwrite
//                                //updated!
//                                (*(std::find_if(buffRxEvents.begin(), buffRxEvents.end(), [&](receptionEvents const & tmp){return tmp.SNR == it.SNR;}))).collidedFlag = true; //changed in buffRxEvents
//                                currentCollisionNumFrames[chSFIndex]++;
//                                noOfCollidedPackets[chSFIndex]++;
//
//                                SIR = it.SNR - pktSNR ;
//                                SIR2 = pktSNR - it.SNR;
//                                SIR1Vector.record(SIR);
//                                SIR1Stats.collect(SIR);
//                            }
//                            printFilteredRxEventsBuffer();
//                            printRxEventsBuffer();
//
                     }
            ///////////////////////////////////////////////////
            installedServerChannels[chFreqIndex][chSFIndex]++;

            printInstalledServerChannelsTable();
            delete loraPkt;
        }
        else //if(interPkt) //Inter-System Interference Modeling [IEEE802.15 LPWAN PHY Interference Model]
        {
            //EV<<  interPkt->getName()<< " Interference packet with BW::"  << interPkt->getBW() <<  "and Power: " <<interPkt->getPower() <<  "@ t: "<<interPkt->getArrivalTime()  <<endl;
            //EV<< "interPkt->getDuration(): " << interPkt->getDuration() << endl;
            //EV <<"interPkt->getBW(): " << interPkt->getBW() <<endl;
            //EV << "interPkt->getPower()" << interPkt->getPower() <<endl; //int power in dB
            //EV<< "Incoming Interference Signal Power_dB:" <<   interPkt->getPower() << endl;
            double Power_Lin = pow(10,((interPkt->getPower())/10)); //convert to Lin

            //EV<< "Incoming Interference Signal Power_Lin: " << Power_Lin << endl;

            //EV<< "Pnoise_dB: " << Pnoise <<endl;
            float Pnoise_Lin = pow(10,(Pnoise /10)); //noise linear
            //EV<< "Pnoise_Lin: " << Pnoise_Lin << endl;
            //            EV << "Pnoise_Lin/2: " << Pnoise_Lin/2 << endl;
            //            EV << "Pnoise_Lin/2_dB" << 10*std::log10(Pnoise_Lin/2) << endl;

            float Pinternoise_Lin = Power_Lin + Pnoise_Lin; //adding background noise and interference noise linearly
            //EV<< "Pinternoise_Lin: " << Pinternoise_Lin << endl;
            float Pinternoise_dB = 10*std::log10(Pinternoise_Lin); //
            //EV<< "Pinternoise_dB: " << Pinternoise_dB << endl;
            Pnoise = Pinternoise_dB;


            endReceptionTime = simTime() + interPkt->getDuration();
            //EV<< "endReceptionTime" <<endReceptionTime << endl;
            //EV<< "Adding recevingEvent in Buffer " << endl;
            //addRxEvent(100, 100, endReceptionTime, interPkt->getPower(), 1 ); //sorted according to minimum endReceptionTime of each event
            addRxEvent(100, 100, endReceptionTime, Power_Lin, 1 ,100);
            printRxEventsBuffer();
            receiveInterCounter++;
            //EV<< "receiveInterCounter: " << receiveInterCounter << endl;

            //counting collision due to inter-system interference packets
            //loop on all SF fields with current frames and increase the counts

            //            installedServerChannels[chFreqIndex][chSFIndex]
            //            for(int i =1; i<=7;i++)
            //            {
            //                if (currentCollisionNumFrames[i]>0)
            //                {
            //                    currentCollisionNumFrames[i]++;
            //                    noOfCollidedPackets[i]++;
            //                }
            //
            //            }
            //


            //
            printRxEventsBuffer();
            for (auto &it : buffRxEvents)
            {
                //EV<< "check the for loop!" << endl;
                //EV<< "checking: it.chFreqIndex" << it.chFreqIndex << endl;
                //EV<< "checking: it.collidedFlag" << it.collidedFlag << endl;

                if (it.chFreqIndex !=100 && it.collidedFlag !=true)
                {
                    //EV<< "INSIDE THE IF CONDITION:" << endl;
                  //EV << "checking: it.chFreqIndex" << it.chFreqIndex << endl;
                    //EV<< "checking: it.collidedFlag" << it.collidedFlag << endl;

                    it.collidedFlag = true;
                    (*(std::find_if(buffRxEvents.begin(), buffRxEvents.end(), [&](receptionEvents const & tmp){return tmp.SNR == it.SNR;}))).collidedFlag = true;
                    noOfCollidedPackets[it.chSFIndex]++;

                    //EV<< "END IF CONDITION!!" << endl;
                }
            }
            printRxEventsBuffer();


            //ELGHLATA HENA!
            //            for (auto it : buffRxEvents)
            //            {
            //                if (it.chFreqIndex !=100 && it.collidedFlag !=true)
            //                {
            //                it.collidedFlag = true;
            ////                currentCollisionNumFrames[1]++;
            ////                noOfCollidedPackets[1]++;
            ////                currentCollisionNumFrames[it.chSFIndex]++;
            ////                noOfCollidedPackets[it.chSFIndex]++;
            //                }
            //            }

            delete interPkt;
        }
        //EV<< "Scheduling next endRxEvent:" <<endl;
        if (!buffRxEvents.empty() && endReceptionTime !=0) {
            cancelEvent(endRxEvent);
            endRxEvent->par(0) = (buffRxEvents).front().chFreqIndex;
            endRxEvent->par(1) = (buffRxEvents).front().chSFIndex;
            endRxEvent->setTimestamp((buffRxEvents).front().endReceptionTime);
            endRxEvent->par(2) = (buffRxEvents).front().SNR;
            endRxEvent->par(3) = (buffRxEvents).front().collidedFlag;
            scheduleAt((buffRxEvents).front().endReceptionTime, endRxEvent);
        }
    }
}

void Server::refreshDisplay() const
{
    //    //you want these lines to be drawn once only at initialize phase!
    //    for(int i = 5; i>=0; i--)
    //    {
    //        //displaying estimated service radii in GUI
    //        //std::string s = "r"+ std::to_string(i);
    //        //char const *pchar = s.c_str();
    //        char const *pchar = ("r"+ std::to_string(i)).c_str();
    //        EV << " printing keda eh??:)  " << pEstimateServiceRadius[i] <<endl;
    //        getDisplayString().setTagArg(pchar, 0, pEstimateServiceRadius[i]);
    //    }

    //    if (installedServerChannels[chFreqIndex][chSFIndex] >0){
    //    //if (!channelBusy) {
    //        getDisplayString().setTagArg("i2", 0, "status/off");
    //        getDisplayString().setTagArg("t", 0, "");
    //    }
    //    else if (currentCollisionNumFrames == 0) {
    //        getDisplayString().setTagArg("i2", 0, "status/yellow");
    //        getDisplayString().setTagArg("t", 0, "RECEIVE");
    //        getDisplayString().setTagArg("t", 2, "#808000");
    //    }
    //    else {
    //        getDisplayString().setTagArg("i2", 0, "status/red");
    //        getDisplayString().setTagArg("t", 0, "COLLISION");
    //        getDisplayString().setTagArg("t", 2, "#800000");
    //    }
}

void Server::finish()
{
    double totalSuccessRate = 0;
    double totalSuccessProb = 1;
    double totalSuccessPackets = 0;
    EV<< "check 2:" <<  endl;
    EV<< "receiveInterCounter" << receiveInterCounter << endl;

    EV<< "totalReceiveCounter: " << receiveCounter <<endl;
    EV<< "totalNumOfHosts: " << numHosts <<endl;
    EV<< "totalReceivedIntereferenceCounter: " << receiveInterCounter <<endl;

    for (int i =1; i <=6; i++)
    {
        if(rcvCounter[i]>0)
        {
            successPackets[i] = rcvCounter[i] - noOfCollidedPackets[i];
            successRate[i] =successPackets[i]/simTime() ;
            successProb[i] = double(successPackets[i])/double(rcvCounter[i]);
        }

        char buf[32];
        sprintf(buf, "SuccessRate: %i",i+6);
        recordScalar(buf, successRate[i]);

        char buf5[32];
        sprintf(buf5, "SuccessProb: %i",i+6);
        recordScalar(buf5, successProb[i]);

        char buf1[32];
        sprintf(buf1, "rcvCounter: %i",i+6);
        recordScalar(buf1, rcvCounter[i]);

        char buf2[32];
        sprintf(buf2, "noOfCollidedPackets: %i",i+6);
        recordScalar(buf2, noOfCollidedPackets[i]);

        char buf3[32];
        sprintf(buf3, "noOfSuccessPackets: %i",i+6);
        recordScalar(buf3, successPackets[i]);

        //            char buf4[32];
        //            sprintf(buf4, "noOfHostsInSFTier: %i",i+6);
        //            recordScalar(buf4, pNoOfHostsInEachSFTier[i-1]);

        totalSuccessPackets = totalSuccessPackets + successPackets[i];
        totalSuccessRate = totalSuccessRate + successRate[i];
        if(successProb[i]!=0) totalSuccessProb = totalSuccessProb * successProb[i];
        if (receiveCounter == 0) totalSuccessProb  = 0;

    }

    timeInEachState[buffRxEvents.size()] = timeInEachState[buffRxEvents.size()] + (simTime() - lastAddRemoveRxEvent);

    for (int i =0; i <10; i++)
    {
        char buf10[32];
        sprintf(buf10, "timeInState: %i",i);
        recordScalar(buf10, timeInEachState[i]);

    }

    recordScalar("totalNumHosts", numHosts);
    recordScalar("totalReceivedCounter",  receiveCounter);
    recordScalar("totalReceivedInterferenceCounter", receiveInterCounter);
    recordScalar("lastAddRemoveRxEvent",lastAddRemoveRxEvent);
    recordScalar("countOfThreePacketsInSystem",countOfThreePacketsInSystem);
    recordScalar("countOfTwoPacketsInSystem",countOfTwoPacketsInSystem);
    recordScalar("countOfOnePacketInSystem",countOfOnePacketsInSystem);

    printRcvCounter();
    printNoOfCollidedPackets();
    printNoOfSuccessPackets();
    printSuccessRate();
    printSuccessProb();
    successRate[0]=totalSuccessRate;
    successProb[0] = totalSuccessProb;
    recordScalar("totalSuccessRate", totalSuccessRate);
    recordScalar("totalSuccessProb", totalSuccessProb);
    EV << "totalSuccessRate: " << totalSuccessRate <<endl;
    EV << "totalSuccessProb: " << totalSuccessProb <<endl;
    EV << "countOfThreePacketsInSystem: " << countOfThreePacketsInSystem <<endl;
    EV << "countOfTwoPacketsInSystem: " << countOfTwoPacketsInSystem <<endl;
    EV << "countOfOnePacketInSystem: " << countOfOnePacketsInSystem <<endl;

    EV << "duration: " << simTime() << endl;
    recordScalar("duration", simTime());
    EV<< "lastAddRemoveRxEvent"<< lastAddRemoveRxEvent<< endl;


//    printSnrOfThreePacketsInSystem();

}

void Server::addRxEvent(int chFreqIndex, int chSFIndex, simtime_t endReceptionTime, double SNR, bool collidedFlag, int pktType)
{// this function creates a new endRxEvent and adds it in the buffer in a sorted fashion according to minimum
    //end reception time to be rescheduled next and free it's channel
    receptionEvents newEvent;
    newEvent.chFreqIndex = chFreqIndex;
    newEvent.chSFIndex = chSFIndex;
    newEvent.endReceptionTime = endReceptionTime;
    newEvent.SNR=SNR;
    newEvent.collidedFlag = collidedFlag; //default set to false
    newEvent.pktType = pktType;


    buffRxEvents.push_back(newEvent);
    if(!buffRxEvents.empty())
        sort(buffRxEvents.begin(),buffRxEvents.end(), [](const receptionEvents &a, const receptionEvents &b) {return (a.endReceptionTime < b.endReceptionTime);});
}

void Server::removeRxEvent(std::vector<receptionEvents> &buffRxEvents, simtime_t endReceptionTime)
{
    buffRxEvents.erase(
            std::remove_if(buffRxEvents.begin(), buffRxEvents.end(), [&](receptionEvents const & tmp){
        return tmp.endReceptionTime == endReceptionTime;

    }),
    buffRxEvents.end());
}

void Server::printRxEventsBuffer()
{
    EV<< "print rxEventBuffer called!" << endl;
    if(!buffRxEvents.empty())
    {
        EV<< "rxEventBuffer:" <<endl;
        for (unsigned int count = 0; count < buffRxEvents.size(); count++)
        {
          EV <<  buffRxEvents[count].chFreqIndex << " | " << buffRxEvents[count].chSFIndex << "| " << buffRxEvents[count].endReceptionTime << "| " << buffRxEvents[count].SNR << "| " << buffRxEvents[count].collidedFlag << "| " << buffRxEvents[count].pktType << endl;
        }
    }
}

void Server::printFilteredRxEventsBuffer()
{
    if(!filteredBuffRxEvents.empty())
    {
        EV<< "filteredEventBuffer:" <<endl;
        for (unsigned int count = 0; count < filteredBuffRxEvents.size(); count++)
        {
            EV <<  filteredBuffRxEvents[count].chFreqIndex << " | " << filteredBuffRxEvents[count].chSFIndex << "| " << filteredBuffRxEvents[count].endReceptionTime << "| " << filteredBuffRxEvents[count].SNR << "| " << filteredBuffRxEvents[count].collidedFlag << endl;
        }
    }
}

void Server::printRcvCounter()
{
    double checkTotalReceivedMessages=0;
    //EV<< "rcvCounter" <<endl;
    for (int i = 1; i <=6; i++)
    {
        //EV<<  rcvCounter[i] << "|" ;
        checkTotalReceivedMessages = checkTotalReceivedMessages + rcvCounter[i];
    }
    //EV<< "checkTotalReceivedMessages" << checkTotalReceivedMessages << endl;
    //EV<< endl;
}

void Server::printNoOfCollidedPackets()
{
    //EV<< "noOfCollidedPackets: " <<endl;
    for (int i = 1; i <=6; i++)
    {
        //EV<<  noOfCollidedPackets[i] << "|" ;
    }
    //EV<< endl;
}

void Server::printNoOfSuccessPackets()
{
    //EV<< "noOfSucessPackets: " <<endl;
    for (int i = 1; i <=6; i++)
    {
        //EV<<  successPackets[i] << "|" ;
    }
    //EV<< endl;
}

void Server::printSuccessRate()
{
    //EV<< "sucessRate: " <<endl;
    for (int i = 1; i <=6; i++)
    {
        //EV<<  successRate[i] << "|" ;
    }
    //EV<< endl;

}

void Server::printSuccessProb()
{
    //EV<< "sucessProb: " <<endl;
    for (int i = 1; i <=6; i++)
    {
        //EV<<  successProb[i] << "|" ;
    }
    //EV<< endl;

}

void Server::printInstalledServerChannelsTable()
{
    for (int i=0; i<nbOfChannels;i++)
    {
        for(int j=0; j<7;j++)
        {
            EV<< "| " << installedServerChannels[i][j] ;
        }
        EV<< endl;
    }
}

void Server::printSnrOfThreePacketsInSystem()
{
    for (int i=0; i<3204;i++)
    {
        for(int j=0; j<3;j++)
        {
            EV<< "| " << snrOfThreePacketsInSystem[i][j] ;
        }
        EV<< endl;
    }
}




void Server::printUsedCRPattern()
{
    //EV<< "Printing used CR pattern: " <<endl;
    for (int i =0;i<3;i++)
    {
        for (int j=0;j<6;j++)
        {
            //EV<< generatedSelectedCRPerSF[i][j] << "|" ;
        }
        //EV<< endl;
    }
}

void Server::printUsedServiceRadius()
{
    //EV<< "Printing Service Radius used"<<endl;
    for (int i =0; i<6 ;i++)
    {
        //EV<< "usedDerviceRadius[" <<i << "]: " << usedServiceRadius[i] <<endl;
    }
}

void Server::printEstimatedServiceRadius()
{
    //EV<< "Printing EstimateServiceRadius "<<endl;
    for (int i =0; i<6 ;i++)
    {
        //EV<< "pEstimateServiceRadius[" <<i << "]: " << pEstimateServiceRadius[i] <<endl;
    }
}

void Server::printAssignmentEstimateServiceRadius()
{
    //EV<< "Printing assignmentEstimateServiceRadius "<<endl;
    for (int i =0; i<6 ;i++)
    {
        //EV<< "assignmentEstimateServiceRadius[" <<i << "]: " << assignmentEstimateServiceRadius[i] <<endl;
    }
}

double Server::getReceiverSensitivty(int SF)
{
    double sensitivityCR[3][6] ={{-124.18 ,-126.95,-129.77,-132.58,-135.39,-138.25}, {-125.35,-128.11,-130.84,-133.58,-136.38,-139.19},{-125.69,-128.41,-131.14,-133.88,-136.68,-139.49} };
    double sensitivity125[6] = {0,0,0,0,0,0};

    for (int i =0; i<3;i++) // multiply each element in sensitivityCR * generatedSelectedCRPerSF - sum the column and place in sensitivity125 to be used!
    {
        for (int j =0; j<6;j++)
        {
            if (std::abs(sensitivityCR[i][j] * generatedSelectedCRPerSF[i][j]) >= 0.0005)
                sensitivity125[j] = sensitivity125[j] + (sensitivityCR[i][j] * generatedSelectedCRPerSF[i][j]);
        }
    }

    return sensitivity125[SF-7];
}

void Server::estimateServiceRadius()
{
    double sensitivityCR[3][6] ={{-124.18 ,-126.95,-129.77,-132.58,-135.39,-138.25}, {-125.35,-128.11,-130.84,-133.58,-136.38,-139.19},{-125.69,-128.41,-131.14,-133.88,-136.68,-139.49} };
    double sensitivity125[6] = {0,0,0,0,0,0};

    for (int i =0; i<3;i++) // multiply each element in sensitivityCR * generatedSelectedCRPerSF - sum the column and place in sensitivity125 to be used!
    {
        for (int j =0; j<6;j++)
        {
            if (std::abs(sensitivityCR[i][j] * generatedSelectedCRPerSF[i][j]) >= 0.0005)
                sensitivity125[j] = sensitivity125[j] + (sensitivityCR[i][j] * generatedSelectedCRPerSF[i][j]);
        }
    }

    for(int i =0; i<6; i++)
    {

        //        double rxSens = sensitivity125[i];

        //Path-loss model used

        //Oukamara-Hata Model TODO: list parameters used in hata model, make it customizable!:)
        //pEstimateServiceRadius[i] = round(pow(10,(((17.5-rxSens)-129.7478188)/37.19660225))*1000);
        //EV << "Okamara_r ["<< i <<"]: "<< pEstimateServiceRadius[i] <<endl;

        //Log-distance pathloss model

        //Pnoise = -117;// Noise Power (dB) Pnoise=174 - (10*std::log10(BW)) - NF ; //gives a positive number


        pEstimateServiceRadius[i] = (pow(10,((Ptx_ed+Gant-K-Pnoise-requiredSNR[i])/(10*pathlossExp))))*1000;
        //EV<< "Log-dist_SFServiceR ["<< i <<"]: "<< pEstimateServiceRadius[i] <<endl;
    }

}

double * Server::totalAreaSFTier(double estimateServiceRadius[])
{
    int i = 5;
    double *totalAreaSFTier = new double[6];
    double *pTotalAreaSFTier;
    while (i>0)
    {
        totalAreaSFTier[i] = 3.141593*((pEstimateServiceRadius[i]*pEstimateServiceRadius[i]) - (pEstimateServiceRadius[i-1]*pEstimateServiceRadius[i-1]));
        i--;
    }
    totalAreaSFTier[0] = 3.141593*(pEstimateServiceRadius[0]*pEstimateServiceRadius[0]);

    //    for (int i =0;i<6;i++)
    //    {
    //        EV << "totalArea["<< i <<"]" << totalAreaSFTier[i] <<endl;
    //    }
    return pTotalAreaSFTier = totalAreaSFTier;
}

double* Server::relativeDensity(double pTotalAreaSFTier[])
{
    double *relativeDensity = new double[6];


    //    double totalAreaCoveredByBS = 0;
    //
    //    for(int i=5; i>=0;i--)
    //    {
    //                totalAreaCoveredByBS+= pTotalAreaSFTier[i];
    //    }

    //SF Tier Area/ Total Area =)
    double totalAreaCoveredByBS = 3.14159*netR*netR; //this is equivalant to total areas for each SF tier area

    for(int i=5;i>=0;i--)
    {
        relativeDensity[i] = pTotalAreaSFTier[i]/ totalAreaCoveredByBS;
        //        EV << "RelativeDensity of " << i << " ="<< relativeDensity[i] <<endl;
    }


    pRelativeDensity = relativeDensity;
    return pRelativeDensity;

}

int Server:: getSF(double distance)
{
    //this is a public function accessed by each host to retrieve their equivalent SF from the gateway.
    //N.B.: SF retrieved based on calculated distance away from Base-station
    //and assuming that each tier includes ONLY hosts of their perspective SF (TODO: better documentation)
    int indexEstimateServiceRadius = 5; // nodes will always be assigned SF12 if they fall out of network radius
    //int indexEstimateServiceRadius = 100; // nodes falling out of net. radius will be signaled an error!
    double usedDistance;
    for (int i = 0; i<=5; i++)
    {
        if ((usedServiceRadius[i] < pEstimateServiceRadius[i]) && usedServiceRadius[i]!=0)
            usedDistance = usedServiceRadius[i];
        else
            usedDistance = pEstimateServiceRadius[i];

        if (distance <= usedDistance)
            //if (distance <= usedServiceRadius[i])
            //if (distance <= pEstimateServiceRadius[i])
        {
            indexEstimateServiceRadius = i;
            break;
        }
    }
    int SF = indexEstimateServiceRadius + 7;
    return SF;
}

double Server:: getCR(int SF)
{
    //this is a public function accessed by each host to retrieve their selected CR from the gateway.
    //N.B.: CR retrieved based on a bitPatternGenerator
    //(TODO: better documentation)
    int indexCR = 0;

    for (int i = 0; i<3;i++)
    {
        if (generatedSelectedCRPerSF[i][SF-7] ==1)
            indexCR = i;
    }

    double CR = CRValues[indexCR];
    return CR;
}

double Server::getSNRThreshold(double BER, int SF, double SIR)
{


    //EV<< "HERE CHECKING ON SNR THRESHOLDS THING" << endl;
    double refSF = 0;
    double gap = -2.847;

    double gapF;

    //initial coefficiants?=)
    //double coeffTableSF7[3][10] =   {{-0.2119,0.2679,0.8361,-0.8176,-1.2734,0.9299,0.0242,1.5556,-2.1061,-7.1756}, {-0.2314,0.3094,0.9613,-1.0693,-1.6174,1.5221,0.5181,1.0527,-2.9913,-5.3418}, {-0.2218,0.3045,0.8860,-1.0564,-1.4016,1.5967,0.2352,0.8243,-3.1737,-3.8934}};
    //double coeffTableSF9[3][10] =   {{-0.1725,0.2409,0.6178,-0.7413,-0.7603,0.8537,-0.5630,1.5291,-1.5084,-13.0995},{-0.2391,0.2753,1.0359,-0.8847,-1.8053,1.1163,0.6909,1.4036,-2.7574,-11.4611},{-0.2302,0.3136,0.9529,-1.1200,-1.5829,1.6873,0.4902,0.8361,-3.1041,-10.0261}};
    //  double mu[1][2] = {{6.2396,3.5535}}; //mean(x) and std(x) : initially=) to scale and center fitted curve

    //    coeff. analytical i drived for exact fitted curves
    //coeff of SF7
//    double coeffTableSF0[3][10] = {{-0.5184 ,0.6758,2.0408,-2.2020,-2.9094 ,2.5496 ,0.5666 ,1.1726,-2.3864 ,-6.8903}, {-0.6045,0.7510,2.5952,-2.5672,-4.2348,3.2179,1.9044,0.7020,-3.4545,-5.0021},    {-0.6076,0.7600,2.5364,-2.6175,-3.8867,3.3836,1.3768,0.4515,-3.5828,-3.5134}};
//  if (SF==8)
//      double coeffTableSF0[3][10] = {{-0.4956,0.5935,1.9658,-1.7758,-2.8061,1.7370,0.4273,1.6773,-2.0519,-9.9128},      {-0.7000,0.7466,3.1768,-2.5342,-5.3779, 3.0773,2.7063,0.8682,-3.4399,-8.0858},   {-0.1422, 0.3228, 0.1769,-0.5956,-0.1263,0.5739,-0.6754,1.6593,-3.2511,-6.6613}};
//  else if (SF==9)
//      double coeffTableSF0[3][10] = {{-0.4489,0.6475,1.6930,-2.1453,-2.2476,2.4400,0.0631,1.2856,-1.8786,-12.9879},     {-0.5624,0.7328,2.3464,-2.5052,-3.7359,3.0637,1.6379,0.9306,-3.2219,-11.2834},   {-0.5123,0.7561,2.0168,-2.6595,-3.0296,3.5310,1.0563,0.3809,-3.4778,-9.8021}};
//  else if (SF==10)
//      double coeffTableSF0[3][10] = {{-0.3615,0.5620,1.2719,-1.8276,-1.5462,2.1247,-0.4481,1.3143,-1.5796,-15.9138},    {-0.5396,0.6974,2.2665,-2.3757,-3.6533,2.8990,1.6049, 1.0201, -3.0931, -14.2835},{-0.5188, 0.7472,2.0868,-2.6366,-3.2215,3.4950,1.2532,0.4397,-3.4626,-12.8354}};
//  else if (SF==11)
//      double coeffTableSF0[3][10] = {{-0.3376,0.5540,1.1451,-1.8226,-1.2702,2.1481,-0.7409,1.2825,-1.3455,-18.8227},    {-0.4772,0.6300,1.9512,-2.0694,-3.0912, 2.4257,1.1722,1.3027,-2.8683,-17.2957},  {-0.5375,0.7598,2.2068,-2.7099,-3.4680,3.5806,1.4580,0.4727,-3.4511,-15.8661}};
//  else if (SF==12)
//      double coeffTableSF0[3][10] = {{-0.3906,0.6334,1.4078,-2.2231,-1.6465,2.7755,-0.6445,0.9336,-1.1526,-21.7085},     {-0.4573,0.5899,1.8518,-1.8480,-2.9335,2.0196,1.0691,1.5686,-2.7324,-20.2995},  {-0.5379,0.7378,2.2475,-2.6166,-3.6297,3.4337,1.6567,0.5922,-3.4547, -18.8903}};

    double coeffTableSF7[3][10] = {{-0.4956,0.5935,1.9658,-1.7758,-2.8061,1.7370,0.4273,1.6773,-2.0519,-9.9128}, {-0.7000,0.7466,3.1768,-2.5342,-5.3779, 3.0773,2.7063,0.8682,-3.4399,-8.0858}, {-0.5180,0.7365,2.0133,-2.5305,-2.9152,3.3254,0.7664,0.4482,-3.5231,-3.6179}};
    double coeffTableSF0[3][10] =   {{0,0,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}, {-0.5180,0.7365,2.0133,-2.5305,-2.9152,3.3254,0.7664,0.4482,-3.5231,-3.6179}};
    double coeffTableSF9[3][10] =   {{0,0,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}, {-0.5134,0.7621,2.0126,-2.7024,-2.9870,3.6274,0.9795,0.3143,-3.4574,-9.7487}};

    double mu[1][2] = {{6.0100,3.6632}}; //mean(x) and std(x) : exact values=) to scale and center fitted curve

    double (*usedCoeffTable)[10] = 0; // or = &a[0];

    if (refSF == 7)
        usedCoeffTable = coeffTableSF7;
    else if (refSF == 9)
        usedCoeffTable = coeffTableSF9;
    else if (refSF == 0)
            usedCoeffTable = coeffTableSF0;

    //EV<< "refSF=" <<refSF <<endl;
    int    BERIndex = (BER - 3);
    double tempMinar = usedCoeffTable[BERIndex][0] * 1 ;
    //EV<< "usedCoeffTable[BERIndex][0]=" << tempMinar <<endl;
    double x = (SIR-mu[0][0])/mu[0][1]; // center and scale you x input !
    double y = usedCoeffTable[BERIndex][0]*pow(x,9) + usedCoeffTable[BERIndex][1]*pow(x,8) + usedCoeffTable[BERIndex][2]*pow(x,7) + usedCoeffTable[BERIndex][3]*pow(x,6) + usedCoeffTable[BERIndex][4]*pow(x,5) + usedCoeffTable[BERIndex][5]*pow(x,4) + usedCoeffTable[BERIndex][6]*pow(x,3) +usedCoeffTable[BERIndex][7]*pow(x,2) +usedCoeffTable[BERIndex][8]*pow(x,1) +usedCoeffTable[BERIndex][9];
    double SNR;//
    if (SF >= refSF)
        gapF= (gap*(SF-refSF))*1;
    else
        gapF= (gap*(refSF-SF))*-1;
    gapF = 0; //temp
    SNR = y + gapF;
    //EV<< "SNR retrieved from here: " << SNR << endl;
    return SNR;
}

double Server::getICSSNRThreshold(double BER, int SF, double SIR)
{
    double refSF = 0; //use 0 if you want excat curve values!!
//    double gap = -2.847;
    double gap = -2.8626; //using SF=9, no center and scaling for new ICS values

    double gapF;

    //initial values from phoebe
    //double coeffTableSF7[3][10] =   {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}, {0.1575,0.03855,-1.018,0.05159,1.837,0.06197,-1.757,0.7421,-0.4001,-5.915}};
    //double mu[1][2] = {{5.5,4.183}}; //mean(x) and std(x) : to scale and center fitted curve

    //coefficients ANALYTICAL
    //double coeffTableSF7[3][10] = {0};
    //double coeffTableSF9[3][10] = {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0},{0.0033,-0.0135,0.0004,0.0555,-0.0717,0.0418,-0.0555,0.0724,-0.0530,-11.8999}};
    ////coefficients ANALYTICAL-SF7
    //double coeffTableSF0[3][10] =   {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}, {-0.0098,0.0068,0.0647,-0.0596,-0.1288,0.2092,-0.1477,0.1588,-0.1514,-6.1933}};
    //double mu[1][2] = {{6,3.6799}};//mean(x) and std(x) : NEW CURVES ANALYTICAL to scale and center fitted curve

    //coefficients SIMULATION
    double coeffTableSF7[3][10] =   {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}, {-0.0684,0.0270,0.3356,-0.0978,-0.5983,0.2370,0.1742,0.2547,-0.4923,-6.0208}};
    double coeffTableSF9[3][10] =   {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0},{0.0540,-0.0466,-0.2393,0.2036,0.2832,-0.1641,-0.2588,0.2636,-0.1727,-11.9012}};
//    ////coefficients SIMULATION-SF7
//    double coeffTableSF0[3][10] =   {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}, {-0.0684,0.0270,0.3356,-0.0978,-0.5983,0.2370,0.1742,0.2547,-0.4923,-6.0208}};

    ////coefficients SIMULATION-SF7 SIC

     double coeffTableSF0[3][10] =   {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}, {-0.4460,0.2317 ,   2.2414 ,  -0.7309   ,-4.5433   , 2.0478 ,   2.3803 ,  -0.9531   , 4.1060 ,  -1.2535}};

//    double mu[1][2] = {{6,3.894}}; //mean(x) and std(x) : NEW CURVES SIMULATION to scale and center fitted curve

     //SIC
     double mu[1][2] = {{5.0333,4.4178}}; //mean(x) and std(x) : NEW CURVES SIMULATION to scale and center fitted curve


    double (*usedCoeffTable)[10] = 0; // or = &a[0];

    if (refSF == 7)
        usedCoeffTable = coeffTableSF7;
    else if (refSF == 9)
        usedCoeffTable = coeffTableSF9;
    else if (refSF == 0)
        usedCoeffTable = coeffTableSF0;

    int    BERIndex = (BER - 3);

    double x = (SIR-mu[0][0])/mu[0][1]; // center and scale you x input !
    double y = usedCoeffTable[BERIndex][0]*pow(x,9) + usedCoeffTable[BERIndex][1]*pow(x,8) + usedCoeffTable[BERIndex][2]*pow(x,7) + usedCoeffTable[BERIndex][3]*pow(x,6) + usedCoeffTable[BERIndex][4]*pow(x,5) + usedCoeffTable[BERIndex][5]*pow(x,4) + usedCoeffTable[BERIndex][6]*pow(x,3) +usedCoeffTable[BERIndex][7]*pow(x,2) +usedCoeffTable[BERIndex][8]*pow(x,1) +usedCoeffTable[BERIndex][9];
    double SNR;//
    if (SF >= refSF)
        gapF= (gap*(SF-refSF))*1;
    else
        gapF= (gap*(refSF-SF))*-1;

    //temporary let gap=0 w khalas:)
    gapF = 0;

    SNR = y + gapF;
    return SNR;
}

//long * Server::noOfHostsInEachSFTier()
//{   //this function returns the number of hosts allocated in each SF tier
//
//
//    long *nbOfHostsInSFTier = new long[6] {0,0,0,0,0,0};
//    int nbOfConnectedHosts = 0;
//    bool reachedLastHost = false;
//    while (reachedLastHost == false)
//    {
//        char buf[20];
//        sprintf(buf, "host[%d]",  nbOfConnectedHosts);
//        //EV << "buf" << buf <<endl;
//        cModule *host = getModuleByPath(buf);
//
//        if (host!=nullptr)
//        {
//            ////Method1: retrieve the SF from the host itself!
//            //int numHosts = host->getVectorSize();
//            //Host *cHost = dynamic_cast<Host*>(host);
//            // int SF = cHost->SF; // SF will need to be a public variable in the host module and server module need to #include "host.h"
//
//            //Method2: calculates distance between host and server and get the SF equivalent to it!
//            nbOfConnectedHosts++; //counting all hosts connected to the server
////            double hostX = host->par("x").doubleValue();
////            double hostY = host->par("y").doubleValue();
//            double hostR = host->par("r").doubleValue();
//            double dist = hostR; //std::sqrt((hostX-0) * (hostX-0) + (hostY-0) * (hostY-0));
//            //EV << "dist" << dist <<endl;
//            //EV << "SF" << getSF(dist) <<endl;
//            nbOfHostsInSFTier[getSF(dist)-7] ++;
//            //EV << "nbOfHostsInSFTier[getSF(dist)-7]" << nbOfHostsInSFTier[getSF(dist)-7] <<endl;
//        }
//        else
//        {
//            reachedLastHost = true;
//        }
//
//    }
//    pNoOfHostsInEachSFTier = nbOfHostsInSFTier;
//    return pNoOfHostsInEachSFTier;
//
//}

//void Server::algorithm()
//{
//
//    double optimalG = nbOfChannels*0.5 ;
//    double lamda =  (double) 1/60; //(double) 1/86400;// <--app2 // app1--> // (double) 1/600 ; //(double) 1/5; // //our lamda is packet/sec// thats bits/sec-->264/600 ; //hardcoded to app 1[20B every 600 sec] with 13B headers
//    double rowLamdaT;
//    double CR;
//    int CRIndex;
//    double txTime;
//
//    EV << "your constant will be nbOfChannels * 0.5--> " << optimalG <<endl;
//    for (int i =0; i<6 ;i++)
//    {
//        int SF= i+7 ;
//        CR = getCR(SF);
//        CRIndex = std::distance(CRValues, std::find(CRValues, CRValues+3, CR));
//        txTime =112 / ((SF) * (BW/std::pow(2,(SF)))* CR);
//        rowLamdaT = pNoOfHostsInEachSFTier[i]*lamda* txTime;
//
//
//        if (rowLamdaT != optimalG )
//        {
//            while ((rowLamdaT < optimalG) && (CRIndex !=2) && (SF!=12))
//            {
//                EV << "here you need to increase cell radius by switching to higher CR" <<endl;
//
//                generatedSelectedCRPerSF[CRIndex][i] = 0;
//                generatedSelectedCRPerSF[CRIndex+1][i] =1;
//
//                CR = getCR(SF);
//                CRIndex = std::distance(CRValues, std::find(CRValues, CRValues+3, CR));
//                txTime =112 / ((SF) * (BW/std::pow(2,(SF)))* CR);
//
//                //updating radius, area and thus noOfHostsInEachSFTier & relativeDensity for each SF Tier
//                double rxSens = getReceiverSensitivty(SF);
//                pEstimateServiceRadius[i] =round(pow(10,(((17.5-rxSens)-129.7478188)/37.19660225))*1000);
//                pNoOfHostsInEachSFTier = noOfHostsInEachSFTier();
//
//                rowLamdaT = pNoOfHostsInEachSFTier[i]*lamda* txTime;
//
//            }
//
//            CRForEachSF[i] = CR;
//            rowLamdaTForEachSF[i]=rowLamdaT;
//            usedServiceRadius[i] = pEstimateServiceRadius[i];
//
//            if (rowLamdaT > optimalG && (SF!=12))
//            {
//                assignmentEstimateServiceRadius[i] = std::sqrt(((0.5*netR*netR*nbOfChannels)/(numHosts*lamda*txTime)) + (usedServiceRadius[i-1]*usedServiceRadius[i-1]));
//                usedServiceRadius[i] = assignmentEstimateServiceRadius[i];
//            }
//        }
//    }
//}

}; //namespace


