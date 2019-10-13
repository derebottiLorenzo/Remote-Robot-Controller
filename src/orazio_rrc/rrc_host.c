#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>
#include <linux/joystick.h>
#include <libwebsockets.h>
#include <signal.h>
#include "rrc_ws.h"
#include "orazio_client.h"
#include "orazio_print_packet.h"
#include "orazio_client_test_getkey.h"

#define NUM_JOINTS 2

typedef enum{
  System = 0,
  Joints = 1,
  Drive = 2,
  Ranges =3,
  None = 4,
  Start = -1,
  Stop = -2,
}Mode;

Mode mode = Start;
static struct OrazioClient *client = 0;

const char *banner[]={
  "remote_robot_controller",
  "generaized host for remote_robot_controller",
  "usage:"
  "$> rrc_host <parameters>",
  "starts a host that accept connections from localhost:9000",
  "parameters: ",
  "-serial-dev <string>: the serial device (default /dev/ttyACM0)",
  "-cam        <string>: the camera which streams(default /dev/video0)",
  0
};

void printBanner(){
  const char *const *line = banner;
  while (*line){
    printf("%s\n", *line);
    line++;
  }
}

static DifferentialDriveControlPacket drive_control = {
  .header.type = DIFFERENTIAL_DRIVE_CONTROL_PACKET_ID,
  .header.size = sizeof(DifferentialDriveControlPacket),
  .header.seq = 1,
  .translational_velocity = 0,
  .rotational_velocity = 0
};

void stopRobot(void){
  drive_control.header.seq = 0;
  drive_control.rotational_velocity = 0;
  drive_control.translational_velocity = 0;
  OrazioClient_sendPacket(client, (PacketHeader *)&drive_control, 0);
}

void* keyThread(void* arg){
  setConioTerminalMode();
  while(mode!=Stop) {
    KeyCode key_code=getKey();
    switch (key_code){
    case KeyEsc:
      mode=Stop;
      break;
    case KeyArrowUp:
      drive_control.translational_velocity+=0.1;
      drive_control.header.seq=1;
      break;
    case KeyArrowDown:
      drive_control.translational_velocity-=0.1;
      drive_control.header.seq=1;
      break;
    case KeyArrowRight:
      drive_control.rotational_velocity-=0.1;
      drive_control.header.seq=1;
      break;
    case KeyArrowLeft:
      drive_control.rotational_velocity+=0.1;
      drive_control.header.seq=1;
      break;
    case KeyS:
      mode=System;
      break;
    case KeyR:
      mode=Ranges;
      break;
    case KeyJ:
      mode=Joints;
      break;
    case KeyD:
      mode=Drive;
      break;
    case KeyX:
      mode=None;
      break;
    default:
      stopRobot();
    }
  }
  resetTerminalMode();
  return 0;
};

char* default_serial_device = "/dev/ttyACM0";
char* default_cam = "/dev/video0";

int main(int argc, char** argv){
  int c = 1;
  char* serial_device = default_serial_device;
  char* cam = default_cam;
  while(c < argc-1){
    if(!strcmp(argv[c], "-serial-dev")){
      c++;
      serial_device = argv[c];
    }
    else if(!strcmp(argv[c], "-cam")){
      c++;
      cam = argv[c];
    }
    else if(!strcmp(argv[c], "-help")){
      printBanner();
      return 0;
    }
  }

  printf("running with parameters\n");
  printf(" serial device: %s\n", serial_device);
  printf(" camera: %s\n", cam);

  SystemStatusPacket system_status={
    .header.type=SYSTEM_STATUS_PACKET_ID,
    .header.size=sizeof(SystemStatusPacket)
  };

  SystemParamPacket system_params={
    .header.type=SYSTEM_PARAM_PACKET_ID,
    .header.size=sizeof(SystemParamPacket)
  };

  DifferentialDriveStatusPacket drive_status={
    .header.type=DIFFERENTIAL_DRIVE_STATUS_PACKET_ID,
    .header.size=sizeof(DifferentialDriveStatusPacket)
  };

  JointStatusPacket joint_status[NUM_JOINTS];

  // 1. initialize orazio client
  client=OrazioClient_init(serial_device, 115200);
  if(!client) {
    printf("cannot open client on device\n");
    return -1;
  }

  // 2. sync the serial protocol
  printf("Syncing");
  for(int i=0; i<50; ++i){
    OrazioClient_sync(client,1);
    printf(".");
    fflush(stdout);
  }
  printf(" Done\n");

  // 3. read the configuration
  if(OrazioClient_readConfiguration(client,100)!=Success) return -1;

  // 4. get how many motor are on robot
  //    and initialize the index of each joint
  //    the client will read the index from the destination
  //    packet to know which joint is queried
  int num_joints=OrazioClient_numJoints(client);
  for(int i=0; i<num_joints; ++i){
    joint_status[i].header.header.type=JOINT_STATUS_PACKET_ID;
    joint_status[i].header.index=i;
  }  
  int retries=10;

  // 5. at the beginning we disable sending all packets
  //    ask the clients for system params
  OrazioClient_get(client, (PacketHeader*)&system_params);
  //    set the paramater to change
  system_params.periodic_packet_mask=0;
  //    send the packet immediately and try "retries" time
  OrazioClient_sendPacket(client, (PacketHeader*)&system_params, retries);
  //    get the parameters refreshed
  OrazioClient_get(client, (PacketHeader*) &system_params);

  // 6. server thread to read joyinput
  pthread_t key_thread;
  pthread_create(&key_thread, 0, keyThread, 0);

  struct OrazioWSContext* ctx = OrazioWebsocketServer_start(client, 9000, NULL, 115200, cam, &drive_control);
  if(!ctx){
    fprintf(stderr,"error on creating server thread\n");
    exit(EXIT_FAILURE);
  }
    
  Mode previous_mode=mode;
  /* 7. main loop:
     -- send in output all controls
     -- sync
     -- get variables/status from orazioClient through Orazio_get*/

  while(mode!=Stop){
    if(drive_control.header.seq){
      int result = OrazioClient_sendPacket(client, (PacketHeader*)&drive_control, 0);
      if(result) printf("error\n");
      drive_control.header.seq=0;
    }

    if(previous_mode!=mode){
      switch(mode){
      case System:
	system_params.periodic_packet_mask=PSystemStatusFlag;
	break;
      case Joints:
	system_params.periodic_packet_mask=PJointStatusFlag;
	break;  
      case Ranges:
	system_params.periodic_packet_mask=PSonarStatusFlag;
	break;
      case Drive:
	system_params.periodic_packet_mask=PDriveStatusFlag;
	break;
      case None:
	system_params.periodic_packet_mask=0;
	break;
      default:;
      }
      OrazioClient_sendPacket(client, (PacketHeader*)&system_params, retries);

      OrazioClient_get(client, (PacketHeader*)&system_params);
      previous_mode = mode;
    }

    OrazioClient_sync(client,1);

    char output_buf[1024];
    int pos=0;
    switch(mode){
    case System:
      OrazioClient_get(client, (PacketHeader*)&system_status);
      Orazio_printPacket(output_buf,(PacketHeader*)&system_status);
      printf("\r\033[2K%s",output_buf);
      break;
    case Joints:
      for(int i=0; i<num_joints; ++i){
	OrazioClient_get(client, (PacketHeader*)&joint_status[i]);
	pos+=Orazio_printPacket(output_buf+pos,(PacketHeader*)&joint_status[i]);
	printf("\r\033[2k%s",output_buf);
      }    
      break;
    case Drive:
      OrazioClient_get(client, (PacketHeader*)&drive_status);
      Orazio_printPacket(output_buf,(PacketHeader*)&drive_status);
      printf("\r\033[2k%s",output_buf);
      break;
    default:;
    }
  }
  void *retval;
  pthread_join(key_thread,&retval);
  printf("Terminating\n");
  OrazioWebsocketServer_stop(ctx);
  printf("Stopping Robot");
  stopRobot();
  for (int i=0; i<10;++i){
    printf(".");
    fflush(stdout);
    OrazioClient_sync(client,10);
  }
  printf("Done\n");
  OrazioClient_destroy(client);
}
