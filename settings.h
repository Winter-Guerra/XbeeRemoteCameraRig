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

#ifndef SETTINGS_H
#define SETTINGS_H

#define IS_CONTROLLER 0 //Is this the camera controller? Or the reciever?
#define DEBUG_MODE IS_CONTROLLER && 1 //Do we want debug output on the serial line?


#define AT_COMMAND_DELAY 10 //In milliseconds
#define TX_COMMAND_DELAY 20 //In milliseconds. (Ballparked by assuming )
#define PACKET_TIMEOUT 1000

#define PAYLOAD_LENGTH 20

#endif
