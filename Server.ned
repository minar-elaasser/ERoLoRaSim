 
//
// The central computer in the ALOHAnet network.
//
simple Server
{
    parameters:
        @display("i=device/antennatower_l");
        
        //@signal[receiveBegin](type="long");  // increases with each new frame arriving to the server and drops to 0 if the channel becomes finally idle
        //@signal[receive](type="long");  // for successful receptions (non-collisions): 1 at the start of the reception, 0 at the end of the reception
        //@signal[collision](type="long"); // the number of collided frames at the beginning of the collision period
        //@signal[collisionLength](type="simtime_t");  // the length of the last collision period at the end of the collision period
        //@signal[channelState](type="long");

        double x @unit(m); // the x coordinate of the server
        double y @unit(m); // the y coordinate of the server
        
        double netR @unit(m);
        double numHosts;
        double minR @unit(m);
        int Ptx_ed = default(14) ; //Transmission Power (dBm)
               
        //Log-Distance Path-Loss Model Parameters
        int Gant = default(2); // Antenna Gain (dB)
        double K = default(128.95) ; //Intercept of Pathloss model (dB)
        double NF = default(6); //Noise Figure (dB)
        double Pnoise = default(-117);// Noise Power (dB) Pnoise=174 - (10*std::log10(BW)) - NF ; //gives a positive number
        double pathlossExp;
        volatile double shadowingParameter = default(0); //shadowingParameter
        int BW @unit(Hz) = default(125000 Hz);
        
        //Intereference Model
        int BER = default(5); //BER={3,4,5}   3->(10^-3),4->(10^-3),5->(10^-3)
		
		int nbOfChannels = default(8);
		
        double animationHoldTimeOnCollision @unit(s) = default(0s); // in animation time

        //@statistic[serverChannelState](source="channelState";title="Channel state";enum="IDLE=0,TRANSMISSION=1,COLLISION=2";record=vector);
//        @statistic[receiveBegin](source="receiveBegin"; record=vector?; interpolationmode=sample-hold; title="receive begin");
//        @statistic[channelUtilization](source="timeavg(receive)"; record=last; interpolationmode=linear; title="channel utilization");
//        @statistic[collisionMultiplicity](source=collision; record=vector?,histogram; title="collision multiplicity");
//        @statistic[collisionLength](record=vector?,histogram,mean,sum,max; title="collision length");
//        @statistic[receivedFrames](source="sum(receive)"; record=last; title="received frames");
        
//        //@statistic[allReceivedFrames](source="sum(receiveBegin)"; record=last; title="successful received frames");        
//        //@statistic[successReceivedFrames](source="sum(receive)"; record=last; title="successful received frames");
//        @statistic[collidedFrames](source="sum(collision)"; record=last; title="collided frames");
    gates:
        input in @directIn;
}

