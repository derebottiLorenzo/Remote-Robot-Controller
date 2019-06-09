#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <libwebsockets.h>
#include "orazio_client.h"

#define PATH_SIZE 1024
#define BUF_SIZE 10240
#define NUM_VARIABLES 100
#define MAX_CONNECTIONS 1024
#define TV_AXIS 1
#define RV_AXIS 3
#define MAX_TV 1
#define MAX_RV 1
#define BOOST_BUTTON 4
#define HALT_BUTTON 5

typedef struct lws* WebSocketConnectionPtr;

typedef struct joy_packet{
    int axis;
    int value;
    int type;
}joy_packet;

typedef struct {
    char client_response[BUF_SIZE];
    char* client_response_begin;
    int client_response_length;
    WebSocketConnectionPtr connection;
    pthread_t thread;
    volatile int run;
    int rate;
    int port;
    struct OrazioClient* client;
} OrazioWSContext;

static OrazioWSContext* ws_ctx=0;
static DifferentialDriveControlPacket* drive_control;
float gain = 1.0;

static void execute_command(joy_packet* ic_){
    float tv=0;
    float rv=0;
    float tvscale=MAX_TV/32767.0;
    float rvscale=MAX_RV/32767.0;
    joy_packet* ic=ic_;
    int axis=ic->axis;
    int value=ic->value;
    int type=ic->type;
    if (axis==TV_AXIS && type==2){
        tv=-value*tvscale;
        printf("axis=%d, value=%d, type=%d, tv=%lf\n", axis, value, type, tv);
    }
    if (axis==RV_AXIS && type==2){
        rv =-value*rvscale;
        printf("axis=%d, value=%d, type=%d, rv=%lf\n", axis, value, type, rv);
    }
    if(axis==HALT_BUTTON && type==1){
        tv=0;
        rv=0;
    }
    else if (axis==BOOST_BUTTON && type==1){
        if(value) gain=2.0;
        else      gain=1.0;
    }    
    drive_control->rotational_velocity=rv*gain;
    drive_control->translational_velocity=tv*gain;
    drive_control->header.seq=1;
}

static int callback_http(struct lws *wsi,
                         enum lws_callback_reasons reason, void *user,
                         void *in, size_t len){
    OrazioWSContext* ctx=ws_ctx;    
    joy_packet* ic=(joy_packet*) in;
    switch(reason){
    case LWS_CALLBACK_SERVER_NEW_CLIENT_INSTANTIATED:
        printf("connectionj established\n");
        ctx->connection=wsi;
        break;
    case LWS_CALLBACK_RECEIVE:
        execute_command(ic);
        break;
    default:
        break;
    }
    
    return 0;
}

void* _websocketFn(void* args){
    OrazioWSContext* ctx=(OrazioWSContext*) args;
    ws_ctx=ctx;
    int port=ctx->port;
    const char* interface=NULL;

    struct lws_context_creation_info info;

    memset(&info, 0, sizeof(info));
    info.port=port;
    info.iface=interface;

    struct lws_protocols protocols[]={
        {
            .name="http-only",   // name
            .callback=callback_http, // callback
            .per_session_data_size=0,
            .rx_buffer_size=0,
            .user=ctx
        },
        {
            .name=NULL,
            .callback=NULL,
            .per_session_data_size=0,   /* End of list */
            .rx_buffer_size=0,
            .user=ctx
        }
    }; 

    info.protocols=protocols;
    info.ssl_cert_filepath=NULL;
    info.ssl_private_key_filepath=NULL;
    info.gid=-1;

    struct lws_context* context=lws_create_context(&info);
    if(context==NULL){
        fprintf(stderr, "libwebsocket init failed, no server active\n");
        return 0;
    }
    ctx->run=1;
    while(ctx->run){
        lws_service(context, 10);
    }
    lws_context_destroy(context);
    return 0;
}

OrazioWSContext* OrazioWebsocketServer_start(struct OrazioClient* client,
                                             int port,
                                             char* resource_path,
                                             int rate,
                                             DifferentialDriveControlPacket* drive_control_){

    OrazioWSContext* context=(OrazioWSContext*)malloc(sizeof(OrazioWSContext));
    context->port=port;
    context->client=client;
    context->client_response_begin=context->client_response+LWS_PRE;
    context->client_response_length=0;
    context->rate=rate;
    printf("context initialized\n");
    drive_control=drive_control_;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_create(&context->thread, &attr, _websocketFn, context);
    pthread_detach(&context->thread);
    return context;
}

void OrazioWebsocketServer_stop(OrazioWSContext* context){
    context->run=0;
    void* retval;
    pthread_join(context->thread, &retval);
}