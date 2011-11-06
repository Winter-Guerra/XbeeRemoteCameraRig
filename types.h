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
  TX_ACK,
  TX_FAIL,
  RX_PACKET,
  AT_ACK,
  AT_ACK_DATA,
  AT_FAIL
};

#endif

