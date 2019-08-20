/*  This program sends joystick input to host on MARRtinos
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

#define _XOPEN_SOURCE
#define WIDTH 640
#define HEIGHT 480

static int interrupted;

unsigned char* frame = NULL;
int n_frag = 0;

const char* window="window";
char* filename = "buf";

char* default_joy_dev = "/dev/input/js0";
char* address = "localhost";

typedef struct JoyPacket{
    int axis;
    int value;
    int type;
    pthread_mutex_t* lock;
}JoyPacket;

struct js_event e;
JoyPacket* ic;

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

void show_frame(unsigned char* frame){
    FILE* f=fopen(filename, "w");
    if (!f)
	    return;
    int size = WIDTH*HEIGHT;
    fprintf(f, "P5\n%d %d\n255\n", WIDTH, HEIGHT);
    fwrite(frame, size, 1, f);
    fclose(f);
    CvMat* mat_src=cvLoadImageM(filename, CV_LOAD_IMAGE_GRAYSCALE);
    cvShowImage("window",mat_src);
    cvWaitKey(42);
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

static int ws_service_callback(
                         struct lws *wsi,
                         enum lws_callback_reasons reason, void *user,
                         void *in, size_t len){
    
    struct per_vhost_data__minimal* vhd = (struct per_vhost_data__minimal*) lws_protocol_vh_priv_get(lws_get_vhost(wsi),lws_get_protocol(wsi)); 
    JoyPacket* ic_;

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
            printf("[ws_service] Protocol initialized\n");		    
            break;

        case LWS_CALLBACK_PROTOCOL_DESTROY:
            vhd->finished = 1;
            break;      

        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            lwsl_err("CLIENT_CONNECTION_ERROR: %s\n", in ? (char *)in : "(null)");
		    vhd->client_wsi = NULL;
		    lws_timed_callback_vh_protocol(vhd->vhost, vhd->protocol, LWS_CALLBACK_USER, 1);
		    break;

        /* --- client callbacks --- */
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            printf("[ws_service] Connect with server success.\n");
            vhd->established = 1;
            lws_callback_on_writable(wsi);
            break;
        
        case LWS_CALLBACK_CLIENT_WRITEABLE:
            ic_ = malloc(sizeof(JoyPacket));
            pthread_mutex_lock(ic->lock);
            ic_->axis = ic->axis;
            ic_->value = ic->value;
            ic_->type = ic->type;
            pthread_mutex_unlock(ic->lock);
            if(wsi == NULL)
                return -1;
            int n;            
            unsigned char* out = NULL;
            int size = sizeof(JoyPacket);
            out = (unsigned char*) malloc(sizeof(unsigned char)*(LWS_SEND_BUFFER_PRE_PADDING + size + LWS_SEND_BUFFER_POST_PADDING));
            //* setup the buffer*/
            memcpy (out + LWS_SEND_BUFFER_PRE_PADDING, ic_, size);
            //* write out*/
            n = lws_write(wsi, out + LWS_SEND_BUFFER_PRE_PADDING, size, LWS_WRITE_TEXT);
            if(n < len){
                lwsl_err("ERROR %d writing to ws socket\n", n);
			    return -1;
            }
            free(out);
            lws_callback_on_writable(vhd->client_wsi);
            break;

        case LWS_CALLBACK_CLOSED:
            printf("[ws_service] LWS_CALLBACK_CLOSED\n");
            vhd->client_wsi = NULL;
		    vhd->established = 0;
		    lws_timed_callback_vh_protocol(vhd->vhost, vhd->protocol,LWS_CALLBACK_USER, 1);
		    break;

        default:
            break;
    }

    return 0;
}

static int cam_callback(
                         struct lws *wsi,
                         enum lws_callback_reasons reason, void *user,
                         void *in, size_t len){
    struct msg amsg;
    struct per_vhost_data__minimal* vhd=(struct per_vhost_data__minimal*) lws_protocol_vh_priv_get(lws_get_vhost(wsi),lws_get_protocol(wsi));
    
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
            vhd->finished = 1;
            break; 

        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            lwsl_err("CLIENT_CONNECTION_ERROR: %s\n", in ? (char *)in : "(null)");
		    vhd->client_wsi = NULL;
		    lws_timed_callback_vh_protocol(vhd->vhost, vhd->protocol, LWS_CALLBACK_USER, 1);
            break;
        
        /* --- client callbacks --- */
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            printf("[cam_service] Connect with server success.\n");
            vhd->established=1;
            break;

        case LWS_CALLBACK_CLIENT_RECEIVE:
            if(lws_is_first_fragment(wsi)){
                frame = (unsigned char*) malloc(sizeof(unsigned char)*WIDTH*HEIGHT);
                amsg.len = len;
                amsg.payload = malloc(LWS_PRE + len);
                if(!amsg.payload){
                    lwsl_user("OOM: dropping\n");
                    break;
                }
                memcpy((unsigned char*)amsg.payload + LWS_PRE, in, len);
                memcpy(frame+n_frag, amsg.payload, len);
                n_frag += len;
            }
            else if(lws_is_final_fragment(wsi)){
                amsg.len = len;
                amsg.payload = malloc(LWS_PRE + len);
                if(!amsg.payload){
                    lwsl_user("OOM: dropping\n");
                    break;
                }
                memcpy((unsigned char*)amsg.payload + LWS_PRE, in, len);                
                memcpy(frame+n_frag, amsg.payload, len);
                n_frag = 0;
                show_frame(frame);   
                printf("frame_showed\n");
            }
            else{
                amsg.len = len;
                amsg.payload = malloc(LWS_PRE + len);
                if(!amsg.payload){
                    lwsl_user("OOM: dropping\n");
                    break;
                }
                memcpy((unsigned char*)amsg.payload + LWS_PRE, in, len);                
                memcpy(frame+n_frag, amsg.payload, len);
                n_frag += amsg.len;
            }
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

// Function to read the joystick
void* joyThread(void* args_){
    
    char* dev=(char*) args_;

    ic = (JoyPacket*) malloc(sizeof(JoyPacket));
    ic->axis=0;
    ic->value=0;
    ic->type=0;
    ic->lock=malloc(sizeof(pthread_mutex_t));
    if (pthread_mutex_init(ic->lock, NULL) != 0)
    {
        printf("[joy_thread] mutex init failed\n");
        exit(1);
    }
    
    int fd=open(dev, O_RDONLY|O_NONBLOCK);

    while(fd < 0){
        printf("[joy_thread] no dev_input found on %s\nRetrying in 2 seconds...\n(press CTRL-C to abort)", dev);
        sleep(2);
    }

    printf("[joy_thread] dev_input opened\n");

    while(!interrupted){
        if(read(fd, &e, sizeof(e)) > 0){
            fflush(stdout);
            pthread_mutex_lock(ic->lock);
            ic->axis=e.number;
            ic->value=e.value;
            ic->type=e.type;
            pthread_mutex_unlock(ic->lock);
        }
        usleep(1000);
    }
    close(fd);
    pthread_mutex_destroy(ic->lock);
    free(ic);
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
    
    pthread_create(&joy_thread, 0, joyThread, dev);

    struct lws_context_creation_info info;
	struct lws_context *context;
    
    struct lws_protocols protocols[]={
        {
            "exec_commands",
            ws_service_callback,
            0,
            0,
        },
        {
            "cam_protocol",
            cam_callback,
            0,
            0,
        },
        { NULL, NULL, 0, 0 }
    }; 

    memset(&info, 0, sizeof info);
    info.port=CONTEXT_PORT_NO_LISTEN;
    info.protocols=protocols;

    context=lws_create_context(&info);
    printf("[Main] context created.\n");

    int n = 0;
    if (context==NULL){
        printf("[Main] context is NULL.\n");
        return -1;
    }
    
    while(n >=0 && !interrupted){
        n = lws_service(context, 1);
    }

    lws_context_destroy(context);
    void* arg;
    pthread_join(joy_thread, &arg);
    lwsl_user("Completed\n");
    return 0;
}