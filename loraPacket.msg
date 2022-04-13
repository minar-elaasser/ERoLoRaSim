

namespace aloha;

packet loraPacket
{
    int SF;
    //int BW; check later if we need that?
    double chFreq; //channel frequency!(la2en howa msh shart same index at each host and server:)
    double SNR;
    int pktType;
}
