
simple Interferer
{
    parameters:
        double minR @unit(m);
        volatile double rndR;
        volatile double rndTheta;
        int sfNetR = default(12);

        double meanArrivalRate;
        double txPower = default(10);
        double B;
        double txLength;
        int scalingFactor = default(1); //0:none, 1:Low; 10:Medium; 50:Dense
        
        //Log-Distance Path-Loss Model Parameters
        int Gant = default(2); // Antenna Gain (dB)
        double K = default(128.95) ; //Intercept of Pathloss model (dB)
        double NF = default(6); //Noise Figure (dB)
        double pathlossExp;
        volatile double interShadowingParamater;
//        volatile double shadowingParameter = default(0); //shadowingParameter
        int BW @unit(Hz) = default(125000 Hz);
        

        @display("i=block/control");
}
