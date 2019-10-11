/*  This program sends joystick input to host on MARRtino
    through webSocketConnection and it reads webcam image from
    host.
*/

#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <libwebsockets.h>
#include <opencv2/highgui/highgui_c.h>

#include "capture_camera_mod.h"
#include "orazio_client_test_getkey.h"

#define _XOPEN_SOURCE
#define WIDTH 320
#define HEIGHT 240

static int interrupted;

const char* window="window";
char* filename = "buf";

char* default_joy_dev = "/dev/input/js0";
char* address = "localhost";

/* JoyPacket structure stores the params (axis, value, type) sent by joystick
   lock is the semaphore which is set to read/write the structure */
typedef struct JoyPacket{
  int axis;
  int value;
  int type;
}JoyPacket;

JoyPacket* ic;
unsigned char* out;

/* per_vhost_data__minimal stores the params of connection  */
struct per_vhost_data__minimal {
  struct lws_context *context;
  struct lws_vhost *vhost;
  const struct lws_protocols *protocol;
  struct lws_client_connect_info i;
  struct lws *client_wsi;

  char finished;
  char established;
};

struct msg {
  void *payload; /* is malloc'd */
  size_t len;
};

const char *banner[]={
  "remote_robot_controller",
  "generaized client for remote_robot_controller",
  "usage:"
  "$> rrc_client <parameters>",
  "starts a client that connects to ADDRESS:9000",
  "parameters: ",
  "-input-dev <string>: the joystick (default /dev/input/js0)",
  "-address   <string>: address of rrc_host (default 'localhost')",
  0
};

void printBanner(){
  const char*const* line=banner;
  while (*line) {
    printf("%s\n",*line);
    line++;
  }
}

/* show_frame function takes frame and its len, using openCV show a window 
   containing the frame given. The CVWaitkey is set at 20 */
void show_frame(unsigned char* frame, size_t len){
  FILE* f=fopen(filename, "w");
  if (!f)
    return;
  int size=len;
  fwrite(frame, size, 1, f);
  fclose(f);
  CvMat* mat_src=cvLoadImageM(filename, CV_LOAD_IMAGE_COLOR);
  cvShowImage("window",mat_src);
  cvWaitKey(20);
}

static int connect_client(struct per_vhost_data__minimal* vhd, const char* address, const char* protocol){
  vhd->i.context=vhd->context;
  vhd->i.port=9000;
  vhd->i.address=address;
  vhd->i.path="/client";
  vhd->i.host=vhd->i.address;
  vhd->i.origin=vhd->i.address;
  vhd->i.ssl_connection=0;

  vhd->i.protocol=protocol;
  vhd->i.pwsi=&vhd->client_wsi;

  return!lws_client_connect_via_info(&(vhd->i));
}

/* callback function that read the current state of joystick and send it through websocket */
static int callback_send_comm(struct lws *wsi,
			      enum lws_callback_reasons reason, void *user,
			      void *in, size_t len){

  struct per_vhost_data__minimal *vhd=(struct per_vhost_data__minimal*) lws_protocol_vh_priv_get(lws_get_vhost(wsi),lws_get_protocol(wsi));
  struct msg pmsg;

  switch (reason) {

  /* --- protocol lifecycle callbacks --- */
  case LWS_CALLBACK_PROTOCOL_INIT:

    vhd=lws_protocol_vh_priv_zalloc(lws_get_vhost(wsi),
				    lws_get_protocol(wsi),
				    sizeof(struct per_vhost_data__minimal));

    vhd->context=lws_get_context(wsi);
    vhd->protocol=lws_get_protocol(wsi);
    vhd->vhost=lws_get_vhost(wsi);

    if (connect_client(vhd, address, "exec_commands")){
      lws_timed_callback_vh_protocol(vhd->vhost, vhd->protocol, LWS_CALLBACK_USER, 1);
    }
    printf("[cmd_service] Protocol initialized\n");
    break;

  case LWS_CALLBACK_PROTOCOL_DESTROY:
    vhd->finished=1;
    break;

  case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
    lwsl_err("CLIENT_CONNECTION_ERROR: %s\n", in ? (char *)in : "(null)");
    vhd->client_wsi=NULL;
    lws_timed_callback_vh_protocol(vhd->vhost, vhd->protocol, LWS_CALLBACK_USER, 1);
    break;

  /* --- client callbacks --- */
  case LWS_CALLBACK_CLIENT_ESTABLISHED:
    printf("[ws_service] Connect with server success.\n");
    vhd->established=1;
    lws_callback_on_writable(vhd->client_wsi);
    break;

  case LWS_CALLBACK_CLIENT_WRITEABLE:
    if(wsi == NULL)
      return -1;
    int n;
    int len = sizeof(JoyPacket);
    pmsg.payload = malloc(LWS_PRE+len);
    /* setup the buffer*/
    memcpy (pmsg.payload+LWS_PRE, ic, sizeof(JoyPacket));
    /* write out*/
    n=lws_write(vhd->client_wsi, pmsg.payload+LWS_PRE, sizeof(JoyPacket), LWS_WRITE_BINARY);
    if(n < len){
      lwsl_err("ERROR %d writing to ws socket\n", n);
      return -1;
    }
    free(pmsg.payload);
    lws_callback_on_writable(vhd->client_wsi);
    break;

  case LWS_CALLBACK_CLOSED:
    printf("[cmd_service] LWS_CALLBACK_CLOSED\n");
    vhd->client_wsi=NULL;
    vhd->established=0;
    lws_timed_callback_vh_protocol(vhd->vhost, vhd->protocol,LWS_CALLBACK_USER, 1);
    break;

  default:
    break;
  }

  return 0;
}

/* callback function that is triggered when a frame is coming from websocket.
   Through the function show_frame the image on host's camera is shown */
static int callback_rcv_cam(struct lws *wsi,
			    enum lws_callback_reasons reason, void *user,
			    void *in, size_t len){
  struct per_vhost_data__minimal *vhd=(struct per_vhost_data__minimal*) lws_protocol_vh_priv_get(lws_get_vhost(wsi),lws_get_protocol(wsi));
  unsigned char *frame;
  switch (reason) {

    /* --- protocol lifecycle callbacks --- */
  case LWS_CALLBACK_PROTOCOL_INIT:
    vhd=lws_protocol_vh_priv_zalloc(lws_get_vhost(wsi),
				    lws_get_protocol(wsi),
				    sizeof(struct per_vhost_data__minimal));

    vhd->context=lws_get_context(wsi);
    vhd->protocol=lws_get_protocol(wsi);
    vhd->vhost=lws_get_vhost(wsi);

    if (connect_client(vhd, address, "cam_protocol")){
      lws_timed_callback_vh_protocol(vhd->vhost, vhd->protocol, LWS_CALLBACK_USER, 1);
    }
    printf("[cam_service] Protocol initialized\n");

    break;

  case LWS_CALLBACK_PROTOCOL_DESTROY:
    vhd->finished=1;
    break;

  case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
    lwsl_err("CLIENT_CONNECTION_ERROR: %s\n", in ? (char *)in : "(null)");
    vhd->client_wsi=NULL;
    lws_timed_callback_vh_protocol(vhd->vhost, vhd->protocol, LWS_CALLBACK_USER, 1);
    break;

    /* --- client callbacks --- */
  case LWS_CALLBACK_CLIENT_ESTABLISHED:
    printf("[cam_service] Connect with server success.\n");
    vhd->established=1;
    break;

  case LWS_CALLBACK_CLIENT_RECEIVE:
    frame = (unsigned char*) in;
    show_frame(frame, len);
    break;

  case LWS_CALLBACK_CLOSED:
    printf("[cam_service] LWS_CALLBACK_CLOSED\n");
    vhd->client_wsi = NULL;
    vhd->established = 0;
    lws_timed_callback_vh_protocol(vhd->vhost, vhd->protocol,LWS_CALLBACK_USER, 1);
    break;

  default:
    break;
  }

  return 0;
}

/* thread's function that reads every 1 ms the input from joypad */
void* joyThread(void* args_){

  char* dev=(char*) args_;
  struct js_event e;
  
  ic->axis=0;
  ic->value=0;
  ic->type=0;

  int fd=open(dev, O_RDONLY|O_NONBLOCK);

  while(fd < 0){
    printf("[joy_thread] no dev_input found on %s\nRetrying in 2 seconds...\n(press CTRL-C to abort)", dev);
    sleep(2);
  }

  printf("[joy_thread] dev_input opened\n");

  while(!interrupted){
    if(read(fd, &e, sizeof(e)) > 0){
      fflush(stdout);
      ic->axis=e.number;
      ic->value=e.value;
      ic->type=e.type;
    }
  }
  close(fd);
  printf("[joy_thread] dev_input closed\n");
  return 0;
}

static void sigint_handler(int sig)
{
  interrupted = 1;
}

int main(int argc, char** argv){
  pthread_t joy_thread;
  char* dev = default_joy_dev;
  int c = 1;
  signal(SIGINT, sigint_handler);
  while(c < argc){
    if(!strcmp(argv[c], "-address")){
      c++;
      address = argv[c];
    }
    else if(!strcmp(argv[c], "-input-dev")){
      c++;
      dev = argv[c];
    }
    else if(!strcmp(argv[c], "-help")){
      printBanner();
      return 0;
    }
  }

  printf("running with parameters\n");
  printf(" address: %s\n", address);
  printf(" input_device: %s\n", dev);

  ic = malloc(sizeof(JoyPacket));
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_create(&joy_thread, &attr, joyThread, dev);

  struct lws_context_creation_info info;
  struct lws_context *context;

  struct lws_protocols protocols[]={
    {
      "exec_commands",
      callback_send_comm,
      0,
      0,
    },
    {
      "cam_protocol",
      callback_rcv_cam,
      0,
      0,
      
    },
    { NULL, NULL, 0, 0 }
  };

  memset(&info, 0, sizeof info);
  info.port=CONTEXT_PORT_NO_LISTEN;
  info.protocols=protocols;

  context=lws_create_context(&info);
  printf("[Main] context created\n");

  int n = 0;
  if (context==NULL){
    printf("[Main] context is NULL\n");
    return -1;
  }

  while(n >=0 && !interrupted){
    n = lws_service(context, 10);
  }

  lws_context_destroy(context);
  void* arg;
  pthread_join(joy_thread, &arg);
  free(ic);
  printf("[Main] Terminated\n");
  return 0;
}
