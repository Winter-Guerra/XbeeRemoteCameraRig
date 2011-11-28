#ifndef SETTINGS_H
#define SETTINGS_H

#define IS_CONTROLLER 1 //Is this the camera controller? Or the reciever?
#define DEBUG_MODE IS_CONTROLLER && 1 //Do we want debug output on the serial line?


#define AT_COMMAND_DELAY 10 //In milliseconds
#define TX_COMMAND_DELAY 20 //In milliseconds. (Ballparked by assuming )
#define PACKET_TIMEOUT 1000

#define PAYLOAD_LENGTH 20

#endif
