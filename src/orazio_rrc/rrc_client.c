/*  This program sends joystick input to host on MARRtinos
    through webSocketConnection and it reads webcam image from 
    host.
*/

#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <libwebsockets.h>

#define _XOPEN_SOURCE

typedef struct JoyPacket{
    int axis;
    int value;
    int type;
}JoyPacket;

struct js_event e;
JoyPacket* ic;

typedef struct SessionData{
    int fd;
}SessionData;

typedef struct ClientArgs{
    struct lws_context* context;
    struct lws* wsi;
}ClientArgs;

static int destroy_flag=0;
static int connection_flag=0;
static int writeable_flag=0;

static int websocket_write_back(struct lws *wsi_in, JoyPacket* ic_){
    if(&ic_==NULL || wsi_in==NULL)
        return -1;

    int n;
    int len=sizeof(JoyPacket);
    char* out=NULL;

    out = (char *)malloc(sizeof(char)*(LWS_SEND_BUFFER_PRE_PADDING + len + LWS_SEND_BUFFER_POST_PADDING));
    //* setup the buffer*/
    memcpy (out + LWS_SEND_BUFFER_PRE_PADDING, ic_, len );
    //* write out*/
    n = lws_write(wsi_in, out + LWS_SEND_BUFFER_PRE_PADDING, len, LWS_WRITE_TEXT);

    printf("[websocket_write_back] %d, %d, %d\n", ic->axis, ic->value, ic->type);
    usleep(10000);
    free(out);
    
    return n;
}

static int ws_service_callback(
                         struct lws *wsi,
                         enum lws_callback_reasons reason, void *user,
                         void *in, size_t len)
{

    switch (reason) {

        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            printf("[ws_service] Connect with server success.\n");
            connection_flag = 1;
            break;

        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            printf("[ws_service] Connect with server error.\n");
            destroy_flag = 1;
            connection_flag = 0;
            break;

        case LWS_CALLBACK_CLOSED:
            printf("[ws_service] LWS_CALLBACK_CLOSED\n");
            destroy_flag = 1;
            connection_flag = 0;
            break;

        case LWS_CALLBACK_CLIENT_RECEIVE:
            printf("[ws_service] Client recived:%s\n", (char *)in);

            if (writeable_flag)
                destroy_flag = 1;

            break;
        case LWS_CALLBACK_CLIENT_WRITEABLE :
            printf("[ws_service] On writeable is called.\n");
            websocket_write_back(wsi, ic);
            writeable_flag = 1;
            break;

        default:
            break;
    }

    return 0;
}

static void* _inputfn(void* args_){

    ClientArgs* cli_args = args_;

    printf("[input_routine] This is input_routine.\n");

    //* waiting for connection with server done.*/
    while(!connection_flag)
        usleep(1000*20);

    //*Send greeting to server*/
    printf("[input_routine] Server is ready. send the first message to server.\n"); 
    websocket_write_back(cli_args->wsi, ic);

    //involked wriable
    printf("[input_routine] call on writable.\n"); 
    while(1){  
        lws_callback_on_writable(cli_args->wsi);
    }
}

// Function to read the joystick
void* joyThread(void* args_){
    char* dev=(char*) args_;
    int fd=open(dev, O_RDONLY|O_NONBLOCK);

    if(fd < 0){
        printf("no joy found on 5s\n", dev);
        return 0;
    }

    printf("joy opened\n");

    while(!connection_flag)
        usleep(1000*20);

    while(1){
        if(read(fd, &e, sizeof(e)) > 0){
            fflush(stdout);
            ic->axis=e.number;
            ic->value=e.value;
            ic->type=e.type;
            printf("axis: %d, value: %d, type: %d\n",ic->axis,ic->value,ic->type);
        }
        usleep(100);
    }
    close(fd);
    return 0;
}

int main(int argc, char** argv){
    pthread_t joy_thread;
    char* dev="/dev/input/js0";
    char* address="192.168.1.30";
    int port=9000;
    
    pthread_create(&joy_thread, 0, joyThread, dev);
    ic = (JoyPacket*) malloc(sizeof(JoyPacket));
    ic->axis=0;
    ic->value=0;
    ic->type=0;
    
    struct lws_context *context=NULL;
    struct lws_context_creation_info info;
    struct lws *wsi=NULL;
    struct lws_protocols protocol;
    struct lws_client_connect_info ccinfo={0};    

    memset(&info, 0, sizeof info);
    info.port=CONTEXT_PORT_NO_LISTEN;
    info.iface=NULL;
    info.protocols=&protocol;
    info.ssl_cert_filepath=NULL;
    info.ssl_private_key_filepath=NULL;
    info.extensions=lws_get_internal_extensions();
    info.gid=-1;
    info.uid=-1;
    info.options=0;

    protocol.name="input_command";
    protocol.callback=&ws_service_callback;
    protocol.per_session_data_size=sizeof(SessionData);
    protocol.rx_buffer_size=0;
    protocol.id=0;
    protocol.user=NULL;

    context=lws_create_context(&info);
    printf("[Main] context created.\n");

    if (context==NULL) {
        printf("[Main] context is NULL.\n");
        return -1;
    }

    ccinfo.context=context;
	ccinfo.address=address;
	ccinfo.port=port;
	ccinfo.path="/";
	ccinfo.host=lws_canonical_hostname( context );
	ccinfo.origin="origin";
    wsi=lws_client_connect_via_info(&ccinfo);
    if(!wsi){
        printf("Error on connection\n");
        return -1;
    }
    printf("[Main] wsi create success.\n");

    ClientArgs cl_args;
    cl_args.wsi = wsi;
    cl_args.context = context;

    pthread_t pid;
    pthread_create(&pid, NULL, _inputfn, &cl_args);
    pthread_detach(pid);

    while(!destroy_flag)
    {
        lws_service(context, 10);
    }

    lws_context_destroy(context);
    void* arg;
    pthread_join(joy_thread, &arg);
    return 0;
}