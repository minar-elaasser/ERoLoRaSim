
network LoRa
{
    parameters:
        //double numOfHostperKm2;
        //        int numHosts;
        int numHosts7; // number of hosts
        int numHosts8;
        int numHosts9;
        int numHosts10;
        int numHosts11;
        int numHosts12;
        double slotTime @unit(ms);  // zero means no slots (pure Aloha)
        bool dutyCycleActive = default(true);

        double netR @unit(m);
        double netD @unit(m) = default(netR *2);

        double minR @unit(m) = default(500m);
        volatile double rndR = uniform(0,1);
        volatile double rndTheta = uniform(0,1);
        //        volatile double shadowingParamater; //use that in case: SNR function of host location!

        @display("bgb=0,0,yellow,yellow,t");

    submodules:
        server: Server {
            netR = netR;
            numHosts = numHosts7 + numHosts8 + numHosts9 + numHosts10 + numHosts11 + numHosts12;
            minR = minR;
            @display("p=231,340");
        }

        hostSF7[numHosts7]: Host {
            dutyCycleActive = dutyCycleActive;
            slotTime = slotTime;

            minR = minR;
            rndR = rndR;
            rndTheta = rndTheta;
            SF = 7;
            @display("p=76,26");
        }

        hostSF8[numHosts8]: Host {
            dutyCycleActive = dutyCycleActive;
            slotTime = slotTime;

            minR = minR;
            rndR = rndR;
            rndTheta = rndTheta;
            SF = 8;
            @display("p=76,71");
        }

        hostSF9[numHosts9]: Host {
            dutyCycleActive = dutyCycleActive;
            slotTime = slotTime;
            minR = minR;
            rndR = rndR;
            rndTheta = rndTheta;
            SF = 9;
            @display("p=76,113");
        }

        hostSF10[numHosts10]: Host {
            dutyCycleActive = dutyCycleActive;
            slotTime = slotTime;
            minR = minR;
            rndR = rndR;
            rndTheta = rndTheta;
            SF = 10;
            @display("p=76,159");
        }

        hostSF11[numHosts11]: Host {
            dutyCycleActive = dutyCycleActive;
            slotTime = slotTime;
            minR = minR;
            rndR = rndR;
            rndTheta = rndTheta;
            SF = 11;
            @display("p=76,204");
        }

        hostSF12[numHosts12]: Host {
            dutyCycleActive = dutyCycleActive;
            slotTime = slotTime;
            minR = minR;
            rndR = rndR;
            rndTheta = rndTheta;
            SF = 12;
            @display("p=76,245");
        }

//        interferer1: Interferer {
//                minR = minR;
//                rndR = rndR;
//                rndTheta = rndTheta;
//                meanArrivalRate = 0.8;
//                B = 100;
//                txLength = 5; //msec
//                
//                @display("p=208,185;is=s;i=,yellow");
//        }
//        //
//        interferer2: Interferer {
//                minR = minR;
//                rndR = rndR;
//                rndTheta = rndTheta;
//                meanArrivalRate = 0.15;
//                B = 20;
//                txLength = 5;
//                @display("p=283,185;is=s;i=,green");
//        }
//        
//        interferer3: Interferer {
//                minR = minR;
//                rndR = rndR;
//                rndTheta = rndTheta;
//                meanArrivalRate = 0.04;
//                B = 10;
//                txLength = 100;
//                @display("p=274,123;is=s");
//        }
//        
//        interferer4: Interferer {
//                minR = minR;
//                rndR = rndR;
//                rndTheta = rndTheta;
//                meanArrivalRate = 0.01;
//                B = 1000;
//                txLength = 30;
//                @display("p=208,117;is=s;i=,#800000");
//        }

}



