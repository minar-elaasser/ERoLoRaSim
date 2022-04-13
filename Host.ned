simple Host
{
    parameters:
        volatile double pkLenBits @unit(b);   // packet length in bits
        volatile double iaTime @unit(s);   // packet interarrival time
        double slotTime @unit(s);          // zero means no slots (pure Aloha)

        bool isBufferingEvents = default(false);
        bool addFrameHeader = default(false);
        bool dutyCycleActive = default(true);
        int ackMode = default(0); //UnConfirmed mode =0, confirmed mode =1

        double minR @unit(m);
        double rndR;
        double rndTheta;

        volatile double shadowingParamater;
        double pathlossExp;

        int nbOfChannels = default(3);
        int BW @unit(Hz) = default(125000 Hz);
        int SF;
        int hostType= default(0); //0=conventionalLoRaHost 1=ICSLoRaHost

        //double CR = default(0.8);

        //frame adjustement
        int CRC = default(1); // Cyclic Redundency Check: 1=enable, 0=disable
        int IH = default(1); // Implicit Header: 1=enable, 0=disable
        double CR = default(1); // CR = {1,2,3,4}
        int DE = default(0); //low data rate optimization: 2= auto (enabled for SF11&12 only), 1=enable, 0=disable
        int Nprog = default(8);

        //animation and display parameters 		
        double idleAnimationSpeed;         // used when there is no packet being transmitted
        double transmissionEdgeAnimationSpeed; // used when the propagation of a first or last bit is visible
        double midTransmissionAnimationSpeed; // used during transmission
        bool controlAnimationSpeed = default(true);

        @display("i=block/circle,,0;is=vs");


}

