
#include "Interferer.h"

namespace aloha {

Define_Module(Interferer);

Interferer::Interferer()
{
    endEvent = nullptr;
}

Interferer::~Interferer()
{

    cancelAndDelete(endEvent);
}

void Interferer::initialize()
{
    server = getModuleByPath("server");

    minR = par("minR");
    rndR = &par("rndR");
    rndTheta = &par("rndTheta");
    sfNetR = par("sfNetR");

    endEvent = new cMessage("endEvent");
//    endEvent->addPar("status"); //par(0)

    interPkCounter = 0;
    interShadowingParamater = &par("interShadowingParamater");

    pathlossExp = par("pathlossExp");

    EV << "minR: " << minR <<endl;
    EV << "rndR: " << rndR->doubleValue() <<endl;
    EV << "rndTheta: " << rndTheta->doubleValue() <<endl;

    //set network radius for which interferer will be randomly located
    Server *cServer = dynamic_cast<Server*>(server);
    EV << "sfNetR" << sfNetR <<endl;
    netR = cServer->pEstimateServiceRadius[sfNetR-7]; //interferer can randomly pick a place anywhere on all of coverage equivalant to SF12!!
    EV << "netR: " <<netR << endl;
    EV << "netArea: " << ((3.141593 * ((netR*netR)-(minR *minR)))/1000000) << endl;

    meanArrivalRate = par("meanArrivalRate");
    txPower = par("txPower"); //-120.01;//
    B = par("B");
    txLength = par("txLength");//71.9 ;//
    scalingFactor = par("scalingFactor");

    Gant = par("Gant"); // Antenna Gain (dB)
    K = par("K"); //Intercept of Pathloss model (dB)
    BW = par("BW");

    //c++ error precision of netR-minR >> setting it to zero manually
    arrivalRate = scalingFactor * meanArrivalRate *((3.141593 * ((netR*netR)-(0 *0)))/1000000)  * (double(BW)/1000000);
//    arrivalRate = scalingFactor * meanArrivalRate *((2 * 3.141593 * netR)/1000000)  * (double(BW)/1000000);
    EV << "meanArrivalRate" << meanArrivalRate << endl;
    EV << "txPower" << txPower << endl;
    EV << "B" << B << endl;
    EV << "txLength" << txLength << endl;
    EV << "BW" << BW <<endl;
    EV << "arrivalRate" << arrivalRate <<endl;

    simtime_t firstTx = setNextTransmission(); //sets next interferer location and iaTime for transmission

    scheduleAt(firstTx, endEvent);
    getDisplayString().setTagArg("p", 0, x);
    getDisplayString().setTagArg("p", 1, y);
}

void Interferer::handleMessage(cMessage *msg)
{
    ASSERT(msg == endEvent);
    EV << " Preparing This Inter-System Interference Packet =) " <<endl;
    simtime_t duration = txLength / 1000 ;

    char interPkname[40];
    sprintf(interPkname, "inter_pk-%d-#%d", getId(), interPkCounter++);
    InterferencePacket *interPk = new InterferencePacket(interPkname);
    interPk->setDuration(duration); //sec
    interPk->setBW(B); //kHz

    Server *cServer = dynamic_cast<Server*>(server);
    //how is it actucally calculated->> DR. TALLAL!!!
    //double Power = cServer->requiredSNR[sfNetR-7] - (10*pathlossExp*std::log10(r/cServer->pEstimateServiceRadius[sfNetR-7]))- interShadowingParamater->doubleValue();
//    double Power_dB = -120.01;//txPower + Gant - K - (10*pathlossExp*std::log10(r)) - interShadowingParamater->doubleValue() ;
    double Power_dB = txPower + Gant - K - (10*pathlossExp*std::log10(r)) - interShadowingParamater->doubleValue() ;
    EV << "txPower: " << txPower << endl;
    EV << "Gant: " << Gant << endl;
    EV << "K: " << K << endl;
    EV <<"pathlossExp: " << pathlossExp << endl;
    EV << "r: " << r << endl;
    EV << "shadowingParameter: " << interShadowingParamater->doubleValue() << endl;
    EV << "Power_dB: " << Power_dB << endl;
    interPk->setPower(Power_dB); //dBm


    EV << ">>>>>>Transmitting it >>>>>" << endl;
    sendDirect(interPk, radioDelay, duration, server->gate("in"));

    EV << "Set and Schedule next interferer signal:: " << endl;
    nextTransmissionTime = setNextTransmission();
    scheduleAt(nextTransmissionTime, endEvent);
}

void Interferer::finish()
{
    EV << "interPkCounter:" << interPkCounter << endl;
    EV << "Check1:" << endl;
    EV << "arrivalRate" << arrivalRate << endl;
    EV << "expectedNoOfActiveInterferers: " << simTime().dbl() * arrivalRate << endl;
}

void Interferer::refreshDisplay() const{
        getDisplayString().setTagArg("p", 0, x);
        getDisplayString().setTagArg("p", 1, y);
        getDisplayString().setTagArg("i2", 0, "status/yellow");
}

simtime_t Interferer::setNextTransmission()
{
    //generate random radius away from GW and calculate corresponding x, y and radioDelay
    r = (sqrt((rndR->doubleValue()*(1-((0/netR)*(0/netR))))+ (((0/netR)*(0/netR))))) * netR; //800;//
//    r = (sqrt((rndR->doubleValue()*(1-((minR/netR)*(minR/netR))))+ (((minR/netR)*(minR/netR))))) * netR; //800;//
    theta =  2*3.14*rndTheta->doubleValue();
    x = r*cos(theta);   // the x coordinate of the interferer
    y = r*sin(theta);   // the y coordinate of the interferer
    radioDelay = r / propagationSpeed;

    //randomly retrieve next layer to transmit
    //txLayerIndex = intuniform(0, (sizeof(installedLayers)/sizeof(installedLayers[0]))-1); // 0 -3 (no of installed layers=)
    //calculate arrivalRate and iaTime;
//    double arrivalRate = (installedLayers[0][0]+installedLayers[1][0]+installedLayers[2][0]+installedLayers[3][0]) * ((3.141593 * ((netR*netR)-(minR *minR)))/1000000)  * (installedLayers[txLayerIndex][2] /1000);
//    double arrivalRate0 = (installedLayers[0][0]) * ((3.141593 * ((netR*netR)-(minR *minR)))/1000000)  * (installedLayers[0][2] /1000);
//    double arrivalRate1 = (installedLayers[1][0]) * ((3.141593 * ((netR*netR)-(minR *minR)))/1000000)  * (installedLayers[1][2] /1000);
//    double arrivalRate2 = (installedLayers[2][0]) * ((3.141593 * ((netR*netR)-(minR *minR)))/1000000)  * (installedLayers[2][2] /1000);
//    double arrivalRate3 = (installedLayers[3][0]) * ((3.141593 * ((netR*netR)-(minR *minR)))/1000000)  * (installedLayers[3][2] /1000);
        //    double arrivalRate = (installedLayers[txLayerIndex][0]) * ((3.141593 * ((netR*netR)-(minR *minR)))/1000000)  * (installedLayers[txLayerIndex][2] /1000);
//    double totalArrivalRate =0;
//    for (int i = 0; i < 3;i++)
//    {
//        totalArrivalRate = totalArrivalRate + ((installedLayers[i][0]) * ((3.141593 * ((netR*netR)-(minR *minR)))/1000000)  * (installedLayers[i][2] /1000));
//    }

//    double arrivalRate0 = meanArrivalRate * ((3.141593 * ((netR*netR)-(minR *minR)))/1000000)  * (BW /1000);
//    simtime_t iaTime = exponential(1/ arrivalRate0);

    //return next time to generate an interference packet
//    iaTime = exponential(600); //CHECK2 !!
    iaTime = exponential(1/ arrivalRate);
    simtime_t t = simTime() + iaTime;

    EV << "r" << r<< endl;
    EV << "x" << x<< endl;
    EV << "y" << y<< endl;
    EV << "radioDelay" << radioDelay << endl;


    EV << "arrivalRate" << arrivalRate << endl;
    EV << "iaTime" << iaTime <<endl;
    EV  << "next interferer transmission @: " << t << endl;

    return t;
}

} //namespace
