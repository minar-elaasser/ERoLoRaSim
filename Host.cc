//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2015 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#include <algorithm>

#include "Host.h"

namespace aloha {

Define_Module(Host);

Host::Host()
{
    endEvent = nullptr;
}

Host::~Host()
{
    delete lastPacket;
    cancelAndDelete(endEvent);
}

void Host::initialize()
{

    server = getModuleByPath("server");

    if (!server)
        throw cRuntimeError("server not found");

    pkLenBits =&par("pkLenBits") ;
    iaTime = &par("iaTime");

    slotTime = par("slotTime");
    isSlotted = slotTime > 0;
    bufferedPkts = 0;
    isBufferingEvents = par("isBufferingEvents");
    addFrameHeader = par("addFrameHeader");
    EV << "pkLenBits" << pkLenBits->longValue() <<endl;

    //used previously to abstract representation of header bits
    if (addFrameHeader == true)
        frameLenBits = pkLenBits->longValue() +(13*8);
    else
        frameLenBits = pkLenBits->longValue();

    dutyCycleActive = par("dutyCycleActive");

    endEvent = new cMessage("endState");
    state = IDLE;
    pkCounter = 0;
    WATCH((int&)state);
    WATCH(pkCounter);

//    r = par("r").doubleValue();
//    theta = par("theta").doubleValue();

    minR = par("minR");
    rndR = &par("rndR");
    rndTheta = &par("rndTheta");

    EV << "minR: " << minR <<endl;
    EV << "rndR: " << rndR->doubleValue() <<endl;
    EV << "rndTheta: " << rndTheta->doubleValue() <<endl;

    Server *cServer = dynamic_cast<Server*>(server);
    SF = par("SF");
    EV << "SF: " << SF <<endl;
    EV << "pEstimateService[7]" << cServer->pEstimateServiceRadius[SF-7] <<endl;
   hostType = par("hostType");
   EV << "hostType" << hostType <<endl;

    double netR = cServer->pEstimateServiceRadius[SF-7];
    r = (sqrt((rndR->doubleValue()*(1-((minR/netR)*(minR/netR))))+ (((minR/netR)*(minR/netR))))) * netR; //1040.5; ////1040.5; //
    theta =  2*3.14*rndTheta->doubleValue();
    x = r*cos(theta);   // the x coordinate of the host
    y = r*sin(theta);   // the y coordinate of the host

    EV << "r: " << r << endl;
    EV << "theta: " << theta <<endl;
    EV <<  "x: " << x <<endl;
    EV << "y: " <<y <<endl;

    shadowingParamater = &par("shadowingParamater");
    pathlossExp = par("pathlossExp");
    SNR = 0;

    EV << "Host radius: " << r <<endl;
//    x = par("x").doubleValue();
//    y = par("y").doubleValue();
//    EV << "Host x: " << x <<endl;
//    EV << "Host y: " << y <<endl;
//    EV << "Shadowing Parameter: " << shadowingParamater << endl;

    double serverX = server->par("x").doubleValue();
    double serverY = server->par("y").doubleValue();



    idleAnimationSpeed = par("idleAnimationSpeed");
    transmissionEdgeAnimationSpeed = par("transmissionEdgeAnimationSpeed");
    midtransmissionAnimationSpeed = par("midTransmissionAnimationSpeed");

    //double dist = r ; //std::sqrt((x-serverX) * (x-serverX) + (y-serverY) * (y-serverY));
    radioDelay = r / propagationSpeed;



    //Display parameters to visualize different apps running on each end-device!
    getDisplayString().setTagArg("p", 0, x);
    getDisplayString().setTagArg("p", 1, y);

    if (pkLenBits->longValue() >8)
        getDisplayString().setTagArg("i", 1, "red");
    else
        getDisplayString().setTagArg("i", 1, "blue");

    if (hostType ==1)
    {
        getDisplayString().parse("i=block/socket,,0;is=vs");
        getDisplayString().setTagArg("p", 0, x);
        getDisplayString().setTagArg("p", 1, y);
    }

    //Scheduling 1st transmission

    //fn of host id
    //simtime_t firstTx = uniform(0,getId());

    //fn. of the app iaTime
    //(rand()%(iaTime->doubleValue())

    //getTxTime directly
    simtime_t firstTx = getNextTransmissionTime();

    scheduleAt( firstTx, endEvent);

    nbOfChannels = par("nbOfChannels");
    BW = par("BW");

//    Server *cServer = dynamic_cast<Server*>(server);

    //ED behavior for SF Selection:
    //currently fixed for each subset of defined SF networks =)
    //SF = cServer->getSF(dist); //assigned by the GW  //7; //12;//8; //9; //7;
    //SF = intuniform(7,cServer->getSF(dist)); //locally set by the ED: random SF

    if (SF > 12 )
    {
        EV << "Out of Network Radius!" <<endl; //TODO: check what to do with nodes out of range!
        cRuntimeError ("Not Supported In This Simulation - nodes outside coverage area!");
    }


    //ED behavior for CR Selection:
    //CR = cServer->getCR(SF); // assigned by the GW
    CR = par("CR"); // conventional: set locally

    CRC  = par("CRC");
    IH  = par("IH");
    DE = par("DE");

    if (DE == 2 && SF > 10) //auto configuration of low data rate optimization use
        DE = 1;
    else
        DE = 0;
    Nprog = par("Nprog");
    int PL = frameLenBits/double(8); // in Bytes

    //    txRate = SF * (BW/std::pow(2,SF))* CR;
    //    EV << "txRate" << txRate <<endl;
//    EV << "BW" << BW <<endl;
//    EV <<  "PL" << PL << endl;
//    EV << "SF" << SF << endl;
//    EV << "CRC: " << CRC << endl;
//    EV << "IH: " << IH << endl;
//    EV << "CR: " << CR << endl;
//    EV << "DE: " << DE << endl;
//    EV << "Nprog: " << Nprog << endl;

    simtime_t Tsymbol =  1/(BW/std::pow(2,SF));
    simtime_t Tpreamble = (4.25 + Nprog)*Tsymbol ;

    double Npayload = 8 + std::fmax(ceil(double(8*PL - 4*SF + 28 + 16*CRC - 20*IH)/ double(4*(SF-(2*DE))))*(CR + 4),0);
    EV << "Npayload" << Npayload << endl;
    EV << "Npreamble" << Nprog + 4.25 << endl;
    simtime_t Tpayload = Npayload*Tsymbol;
    TOA =  (Tpayload + Tpreamble);

//    EV << "Npayload: " << Npayload << endl;
//    EV << "Tsymbol: " << Tsymbol << endl;
//    EV << "Tpreamble: " << Tpreamble << endl;
//    EV << "Tpayload: " << Tpayload << endl;
//
//    EV << "TOA: " << TOA << endl;
    EV << "TOA: " << TOA << endl;
    nbOfBusyChannels = 0;

    //animation parameters
    if (pkLenBits->doubleValue() > 8)
        getParentModule()->getDisplayString().setTagArg("i", 1, "yellow");
    else
        getParentModule()->getDisplayString().setTagArg("i", 1, "red");
}

void Host::handleMessage(cMessage *msg)
{
    ASSERT(msg == endEvent);

    //animation and display
    //getParentModule()->getCanvas()->setAnimationSpeed(transmissionEdgeAnimationSpeed, this);

    if (state == IDLE) {
        EV <<  "END OF IDLE STATE" <<endl;
        printInstalledChannelsTable();

        // generate packet and schedule timer when it ends
        char pkname[40];
        sprintf(pkname, "pk-%d-#%d", getId(), pkCounter++);

        state = TRANSMIT;

        loraPacket *pk = new loraPacket(pkname);
        pk->setBitLength(frameLenBits);
        pk->setSF(SF);
        pk->setDuration(TOA);
        pk->setPktType(hostType); //setting packet type = host type =)


        EV << "currentSimTime" << simTime() <<endl;
        int channel = selectAvailableChannel(TOA); //Available
        pk->setChFreq(installedChannels[channel][1]);

        EV << "generating packet " << pkname << " -SF: " << pk->getSF() << " -ch. Freq.: " << pk->getChFreq() <<endl;
        EV << "pkt Time-On-Air: " << TOA <<endl;

        Server *cServer = dynamic_cast<Server*>(server);
        SNR = cServer->requiredSNR[SF-7] - (10*pathlossExp*std::log10(r/cServer->pEstimateServiceRadius[SF-7]))- shadowingParamater->doubleValue();
        pk->setSNR(SNR);
        sendDirect(pk, radioDelay, TOA, server->gate("in"));

        //check if any channels will be free during transmission and update installed channels
        //lw fee aii endToff < simTime()+duration
        double minEndTOff = getMinEndTOff();
//        double minEndTOff = installedChannels[0][2];
//        for (int i =0; i < nbOfChannels; i++)
//        {
//            if (installedChannels[i][2] < minEndTOff)
//                minEndTOff = installedChannels[i][2];
//        }

        EV << "Time for packet to finish transmission::::" << simTime()+TOA <<endl;
        simtime_t tmp = minEndTOff;
       //if ((minEndTOff <= (simTime()+duration).dbl()) && (minEndTOff !=0) )
        if ((tmp <= (simTime()+TOA)) && (tmp !=0))
        {

            for(int i=0;i < nbOfChannels ; i++)
            {
                if (installedChannels[i][2] == minEndTOff)
                {
                //EV << "will see loop: " << installedChannels[i][2] <<endl;
                //EV << "minEndTOff" << minEndTOff <<endl;
                installedChannels[i][2] = 0 ;
                nbOfBusyChannels--;
                }
            }
            printInstalledChannelsTable();
        }


        if (bufferedPkts > 0 ) bufferedPkts--;

        scheduleAt(simTime()+TOA, endEvent);

        // let visualization code know about the new packet
        if (transmissionRing != nullptr) {
            delete lastPacket;
            lastPacket = pk->dup();
        }

        printInstalledChannelsTable();
        EV <<  "//IDLE STATE" <<endl;
    }
    else if (state == TRANSMIT) {
        // endEvent indicates end of transmission
        //check if all channels are busy -> go to state (WAITINGFORCH), if not -> go to state (IDLE)
        EV <<  "END OF TRANSMIT STATE" <<endl;

        printInstalledChannelsTable();

        nextTransmissionTime = getNextTransmissionTime();

        EV << "next time to schedule a packet?:) " << nextTransmissionTime <<endl;

        if (nbOfBusyChannels == nbOfChannels) //no av. channels
        {
            EV << "nbOfBusyChannels == nbOfChannels --> NO. AV. Ch --> waiting for ch. free" <<endl;
            state = WAITINGFORCH;
            //emit(stateSignal, state);

            //all channels are busy (in their Toff)
            // find minimum endToff: time for a sub-band to be free
            double minEndTOff = getMinEndTOff();
//            double minEndTOff = installedChannels[0][2];
//            for (int i =0; i < nbOfChannels; i++)
//            {
//                if (installedChannels[i][2] < minEndTOff)
//                    minEndTOff = installedChannels[i][2];
//            }


            //check if any packets will be generated while host will be busy waiting for a channel!
            if (nextTransmissionTime < minEndTOff) // < walla <=
            {//make sure to send directly asa a ch is free
                bufferedPkts++ ;
                //EV << "bufferedPkts" << bufferedPkts <<endl;
            }
            scheduleAt(minEndTOff, endEvent);

            printInstalledChannelsTable();
            EV <<  "//TRANSMIT STATE - when all channels are busy" <<endl;
        }
        else if (nbOfBusyChannels < nbOfChannels)
        {//there are some free channels to use and so you can transmit!
            EV << "nbOfBusyChannels < nbOfChannels --> YES AV. CH --> you are going to be idle" <<endl;
            state = IDLE;
            scheduleAt(nextTransmissionTime, endEvent);

            printInstalledChannelsTable();
            EV <<  "//TRANSMIT STATE - when some channels are still free" <<endl;
        }
    }
    else if (state == WAITINGFORCH) {
        //endEvent indicates end of waiting time of a channel
        EV << "END OF WAITINGFORCH STATE" << endl;
        state = IDLE;

        //update data structure of installed channels by freeing their endToff!! and decrementing nbOfBusyChannels
        for(int i=0;i < nbOfChannels ; i++)
        {
            //if (std::to_string(installedChannels[i][2]) <= std::to_string(simTime().dbl()))//both converted to strings because sometime converting to double loses precision and yields wrong results
            //if (std::to_string(installedChannels[i][2]) == std::to_string(simTime().dbl()) || installedChannels[i][2] < simTime().dbl())
            simtime_t tmp = installedChannels[i][2];

            //if (installedChannels[i][2] <= simTime().dbl())
            if (tmp <= simTime())
            {
//                EV << " installedChannels[i][2] - gowa el if" << std::to_string(installedChannels[i][2]) <<endl;
                installedChannels[i][2] = 0 ;
                nbOfBusyChannels--;
            }
        }
        //schedule sending after checking if there are any buffered pkts
         //scheduleAt(getNextTransmissionTime(), endEvent);//TODODONE: check if u should remove that!
        if (bufferedPkts > 0 && isBufferingEvents ==true)
        {
            scheduleAt(simTime(), endEvent);
//            EV << "bufferedPkts" << bufferedPkts <<endl;
        }
        else{
            scheduleAt(getNextTransmissionTime(), endEvent);
        }

        printInstalledChannelsTable();
        EV <<  "//WAITINGFORCH STATE" <<endl;
    }
    else {
        throw cRuntimeError("invalid state");
    }
}

simtime_t Host::getNextTransmissionTime()
{

    simtime_t t = simTime() + iaTime->doubleValue();
    EV << "iaTime" << iaTime->doubleValue() <<endl;

    if (!isSlotted)
        return t;
    else
        // align time of next transmission to a slot boundary
        return slotTime * ceil(t/slotTime);
}

int Host::selectAvailableChannel(simtime_t TOA)
{   //this function selects available channel and update installed channels data structure
    //this function takes packet duration (TOA) as input and returns a valid channel index number or 1000 if no channels are available
    //Method2.0: keeps track of nbofBusyChannels, if all are idle returns busy
    //if there is any free channel available.. randomly keep picking one out of all installed (free and busy) channels
    //when the free channel found: - Get channel sub-band and duty cycle of this sub-band.
    //                             - Calculate Toff and expected time for this sub-band Toff to end.
    //                             - Indicate each channel in corresponding sub-band is busy till the same endToff calculated and incrementing the count of nbOfBusyChannels
    //EV <<  "Selecting channel function: " <<endl;
    //printInstalledChannelsTable();
    //EV << "nbOfChannels:" << nbOfChannels <<endl;
    //EV << "nbOfBusyChannels:" << nbOfBusyChannels <<endl;

    int channel;
    if (nbOfBusyChannels <nbOfChannels) //make sure there is a free channel i.e.: wont loop here forever!
    {
        bool chFound = false;
        while (chFound ==false)
        {
            channel = uniform(0,nbOfChannels); //randomly pick any channel
            //EV << "Picked channel:" << channel <<endl;
            //EV << "endToff of it:" <<installedChannels[channel][2] <<endl;
            //EV << "current simtime: " << simTime().dbl() <<endl;
            if (installedChannels[channel][2] < simTime().dbl() ) //check if its free comparing current time endTOff
            {
                chFound = true;
                int chSubband = installedChannels[channel][0]; //channel sub-band
                //EV << "Duty cycle of subband of channel picked: " << dutyCycle[chSubband] <<endl; //duty cycle of this sub-band

                if (dutyCycleActive == false)
                {
                    //  deactivating the effect of Toff
                    simtime_t endToff = simTime()+TOA;

                    if ( installedChannels[channel][2] > 0) nbOfBusyChannels--; //if its the same subband of a previously used channel (a previously used channel in case of Toff deactivated) -> free it up
                    installedChannels[channel][2] =endToff.dbl();
                    nbOfBusyChannels++;
                }
                else if (dutyCycleActive == true)
                {
                    simtime_t endToff = simTime() + ((TOA / dutyCycle[chSubband])  - TOA); //expected time for the Toff to end

                    for(int i=0;i < nbOfChannels ; i++) //apply this expected time for all channels of similar sub-band
                    {

                        if (installedChannels[i][0] == chSubband)
                        {
                            if ( installedChannels[i][2] > 0) nbOfBusyChannels--; //if its the same subband of a previously used channel -> free it up
                            installedChannels[i][2] = endToff.dbl() ;
                            //if (installedChannels[i][2] > 0)//update installed channels data structure &by recalculating nb of busy channels
                            nbOfBusyChannels++;
                        }
                    }
                }
            }
        }

        //EV <<"Found channel :) of number:  " << channel<< endl;
    }
    else
    { //all channels are busy (in their Toff)
        channel = 1000; //u r supposd to throw an error here!!
        throw cRuntimeError("invalid channel selection");
        //EV << "All channels are in their Toff!!" <<endl;
    }

    //EV <<  "Selecting channel function: " <<endl;
    //printInstalledChannelsTable();
    //EV << "nbOfChannels:" << nbOfChannels <<endl;
    //EV << "nbOfBusyChannels:" << nbOfBusyChannels <<endl;

    return channel;
}

void Host::refreshDisplay() const
{
//    cCanvas *canvas = getParentModule()->getCanvas();
//    const int numCircles = 20;
//    const double circleLineWidth = 10;
//
//    // create figures on our first invocation
//    if (!transmissionRing) {
//        auto color = cFigure::GOOD_DARK_COLORS[getId() % cFigure::NUM_GOOD_DARK_COLORS];
//
//        transmissionRing = new cRingFigure(("Host" + std::to_string(getIndex()) + "Ring").c_str());
//        transmissionRing->setOutlined(false);
//        transmissionRing->setFillColor(color);
//        transmissionRing->setFillOpacity(0.25);
//        transmissionRing->setFilled(true);
//        transmissionRing->setVisible(false);
//        //transmissionRing->setZIndex(-1);
//        canvas->addFigure(transmissionRing);
//
//        for (int i = 0; i < numCircles; ++i) {
//            auto circle = new cOvalFigure(("Host" + std::to_string(getIndex()) + "Circle" + std::to_string(i)).c_str());
//            circle->setFilled(false);
//            circle->setLineColor(color);
//            circle->setLineOpacity(0.75);
//            circle->setLineWidth(circleLineWidth);
//            circle->setZoomLineWidth(true);
//            circle->setVisible(false);
//            //circle->setZIndex(-0.5);
//            transmissionCircles.push_back(circle);
//            canvas->addFigure(circle);
//        }
//    }
//
//    if (lastPacket) {
//        // update transmission ring and circles
//        //if (transmissionRing->getAssociatedObject() != lastPacket) {
//            //transmissionRing->setAssociatedObject(lastPacket);
//            f//or (auto c : transmissionCircles)
//                //c->setAssociatedObject(lastPacket);
//        //}
//
//        simtime_t now = simTime();
//        simtime_t frontTravelTime = now - lastPacket->getSendingTime();
//        simtime_t backTravelTime = now - (lastPacket->getSendingTime() + lastPacket->getDuration());
//
//        // conversion from time to distance in m using speed
//        double frontRadius = frontTravelTime.dbl() * propagationSpeed;
//        //double frontRadius = std::min(ringMaxRadius, frontTravelTime.dbl() * propagationSpeed);
//        double backRadius = backTravelTime.dbl() * propagationSpeed;
//        double circleRadiusIncrement = circlesMaxRadius / numCircles;
//
//        // update transmission ring geometry and visibility/opacity
//        double opacity = 1.0;
//        if (backRadius > ringMaxRadius) {
//            transmissionRing->setVisible(false);
//            transmissionRing->setAssociatedObject(nullptr);
//        }
//        else {
//            transmissionRing->setVisible(true);
//            transmissionRing->setBounds(cFigure::Rectangle(x - frontRadius, y - frontRadius, 2*frontRadius, 2*frontRadius));
//            transmissionRing->setInnerRadius(std::max(0.0, std::min(ringMaxRadius, backRadius)));
//            if (backRadius > 0)
//                opacity = std::max(0.0, 1.0 - backRadius / circlesMaxRadius);
//        }
//
//        transmissionRing->setLineOpacity(opacity);
//        transmissionRing->setFillOpacity(opacity/5);
//
//        // update transmission circles geometry and visibility/opacity
//        double radius0 = std::fmod(frontTravelTime.dbl() * propagationSpeed, circleRadiusIncrement);
//        for (int i = 0; i < (int)transmissionCircles.size(); ++i) {
//            double circleRadius = std::min(ringMaxRadius, radius0 + i * circleRadiusIncrement);
//            if (circleRadius < frontRadius - circleRadiusIncrement/2 && circleRadius > backRadius + circleLineWidth/2) {
//                transmissionCircles[i]->setVisible(true);
//                transmissionCircles[i]->setBounds(cFigure::Rectangle(x - circleRadius, y - circleRadius, 2*circleRadius, 2*circleRadius));
//                transmissionCircles[i]->setLineOpacity(std::max(0.0, 0.2 - 0.2 * (circleRadius / circlesMaxRadius)));
//            }
//            else
//                transmissionCircles[i]->setVisible(false);
//        }
//
//        // compute animation speed
//        double animSpeed = idleAnimationSpeed;
//        if ((frontRadius >= 0 && frontRadius < circlesMaxRadius) || (backRadius >= 0 && backRadius < circlesMaxRadius))
//            animSpeed = transmissionEdgeAnimationSpeed;
//        if (frontRadius > circlesMaxRadius && backRadius < 0)
//            animSpeed = midtransmissionAnimationSpeed;
//        canvas->setAnimationSpeed(animSpeed, this);
//    }
//    else {
//        // hide transmission rings, update animation speed
//        if (transmissionRing->getAssociatedObject() != nullptr) {
//            transmissionRing->setVisible(false);
//            transmissionRing->setAssociatedObject(nullptr);
//
//            for (auto c : transmissionCircles) {
//                c->setVisible(false);
//                c->setAssociatedObject(nullptr);
//            }
//            canvas->setAnimationSpeed(idleAnimationSpeed, this);
//        }
//    }
//
//    // update host appearance (color and text)
//    getDisplayString().setTagArg("t", 2, "#808000");
//    if (state == IDLE) {
//        getDisplayString().setTagArg("i", 1, "");
//        getDisplayString().setTagArg("t", 0, "");
//    }
//    else if (state == TRANSMIT) {
//        getDisplayString().setTagArg("i", 1, "yellow");
//        getDisplayString().setTagArg("t", 0, "TRANSMIT");
//    }
}

double Host::getMinEndTOff()
{
    double minEndTOff = installedChannels[0][2];
    for (int i =0; i < nbOfChannels; i++)
    {
        if (installedChannels[i][2] < minEndTOff)
            minEndTOff = installedChannels[i][2];
    }
    return minEndTOff;
}

void Host::printInstalledChannelsTable()
{
    for (int i=0; i<nbOfChannels;i++)
    {
        for(int j=0; j<3;j++)
        {
            EV << "| " << installedChannels[i][j] ;
        }
        EV << endl;
    }
}

}; //namespace
