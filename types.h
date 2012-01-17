//This is a part of the source code I coded for a xbee and stepper-motor
//driven camera mount needed for a school play. More info on the current 
//status of the project can be found at its thingiverse page here: 
//http://www.thingiverse.com/thing:16092
//or at my code's Github repo here: 
//https://github.com/xtremd/Parametric-Openscad-Tripod-Mount
//and here:
//https://github.com/xtremd/XbeeRemoteCameraRig

//This code is released under the CC-BY-SA license. Read more about it here:
//http://creativecommons.org/licenses/by-sa/3.0/us/

//Go be awesome!

//-XtremD
//January 17th, 2012

#ifndef TYPES_H
#define TYPES_H

//The return status codes go like this for the packets.... 
//This enum must be in the header for compiler reasons.
//0 is no packet was recieved Is the device unplugged?
//1 is TX ACK
//2 is TX FAILED
//20 is RX
//30 is AT ACK
//31 is AT ACK WITH OUTPUT
//32 is AT FAIL
enum returnPacketStates {
  COMM_FAIL,
  PACKET_NOT_FINISHED,
  TX_ACK,
  TX_FAIL,
  RX_PACKET,
  AT_ACK,
  AT_ACK_DATA,
  AT_FAIL
};

#endif


