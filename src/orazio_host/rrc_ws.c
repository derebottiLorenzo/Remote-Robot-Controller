#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <libwebsockets.h>
#include "orazio_client.h"
#include "capture_camera_mod.h"

#define MAX_CONNECTIONS 1024
#define TV_AXIS 1
#define RV_AXIS 3
#define MAX_TV 1
#define MAX_RV 1
#define BOOST_BUTTON 4
#define HALT_BUTTON 5

#define WIDTH 320
#define HEIGHT 240

typedef struct JoyPacket{
  int axis;
  int value;
  int type;
}JoyPacket;

/* one of these created for each message */

struct msg {
  void *payload; /* is malloc'd */
  size_t len;
};

/* one of these is created for each client connecting to us */

struct per_session_data__minimal {
  struct per_session_data__minimal *pss_list;
  struct lws *wsi;
  uint32_t tail;
};

/* one of these is created for each vhost our protocol is used with */

struct per_vhost_data__minimal {
  struct lws_context *context;
  struct lws_vhost *vhost;
  const struct lws_protocols *protocol;

  struct per_session_data__minimal *pss_list; /* linked-list of live pss*/
  pthread_t pthread_spam;

  pthread_mutex_t lock_ring;
  struct lws_ring *ring;

  char finished;
};

typedef struct{
  struct per_vhost_data__minimal* connections[MAX_CONNECTIONS];
  pthread_t thread;
  volatile int run;
  int rate;
  int port;
  char* cam;
  DifferentialDriveControlPacket* drive_control;
  camera_t* camera;
  struct OrazioClient *client;
} OrazioWSContext;

static OrazioWSContext* ws_ctx = 0;
float gain = 1.0;

int findConnection(OrazioWSContext *ctx, struct per_vhost_data__minimal* conn){
  for (int i = 0; i < MAX_CONNECTIONS; ++i)
    if (ctx->connections[i] == conn)
      return i;
  return -1;
}

int getFreeConnectionIdx(OrazioWSContext *ctx){
  for (int i = 0; i < MAX_CONNECTIONS; ++i)
    if (!ctx->connections[i]){
      return i;
    }
  return -1;
}

int freeConnection(OrazioWSContext *ctx, struct per_vhost_data__minimal* conn){
  int idx = findConnection(ctx, conn);
  if (idx < 0)
    return -1;
  ctx->connections[idx] = 0;
  return 0;
}

void initConnections(OrazioWSContext* ctx){
  memset(ctx->connections, 0, sizeof(struct per_vhost_data__minimal*) * MAX_CONNECTIONS);
}

static void
__minimal_destroy_message(void *_msg)
{
  struct msg *msg = _msg;
  
  free(msg->payload);
  msg->payload = NULL;
  msg->len = 0;
}

void* thread_spam(void* args){
  struct per_vhost_data__minimal *vhd = (struct per_vhost_data__minimal *) args;
  OrazioWSContext* ctx = ws_ctx;
  struct msg amsg;
  int n;
  if(!ctx->camera)
    exit(1);
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  while(ctx->run){
    if(!vhd->pss_list)
      goto wait;
    if(camera_frame(ctx->camera, timeout) > 0){
      unsigned char *rgb = yuyv2rgb(ctx->camera->head.start, ctx->camera->width, ctx->camera->height);
      FILE *out = fopen("result.jpg", "w+");
      jpeg(out, rgb, ctx->camera->width, ctx->camera->height, 15);
      size_t size;
      fseek(out, 0, SEEK_END);
      size = ftell(out);
      fseek(out, 0, SEEK_SET);
      
      if(size <= 4096){
	int len = LWS_PRE+size;
	amsg.payload = malloc(len);
	if(!amsg.payload){
	  lwsl_user("[Thread_spam] Malloc error\n");
	  goto wait_unlock;
	}
	int err = fread(amsg.payload+LWS_PRE, 1, size, out);
	if (err != size){
	  printf("[Thread_Spam] Error: There was Error %d reading the file\n", err);
	  exit(1);
	}
	pthread_mutex_lock(&vhd->lock_ring);
	n = (int)lws_ring_get_count_free_elements(vhd->ring);
	if(!n) {
	  lwsl_user("[Thread_spam] Ring is full\n");
	  goto wait_unlock;
	}
	amsg.len = size+LWS_PRE;
	n = lws_ring_insert(vhd->ring, &amsg, 1);
	if(n!=1){
	  __minimal_destroy_message(&amsg);
	  lwsl_user("[Thread_spam] Cannot add elem to ring\n");
	}
	else {
	  lws_cancel_service(vhd->context);
	}
      }
      free(rgb);
      fclose(out);
    }
  wait_unlock:
    pthread_mutex_unlock(&vhd->lock_ring);
  wait:
    usleep(100);
  }

  lwsl_notice("[Thread_spam] %p exiting\n", (void *)pthread_self());
  pthread_exit(NULL);
  return NULL;
}

static int callback_rcv_comm(struct lws *wsi,
                         enum lws_callback_reasons reason, void *user,
                         void *in, size_t len){

  struct per_session_data__minimal *pss = (struct per_session_data__minimal *)user;
  struct per_vhost_data__minimal *vhd = (struct per_vhost_data__minimal *) lws_protocol_vh_priv_get(lws_get_vhost(wsi), lws_get_protocol(wsi));
  OrazioWSContext* ctx = ws_ctx;
  JoyPacket* ic;
  int idx = 0;
    
  switch (reason){
        
    /* --- protocol lifecycle callbacks --- */
  case LWS_CALLBACK_PROTOCOL_INIT:
    vhd = lws_protocol_vh_priv_zalloc(lws_get_vhost(wsi), lws_get_protocol(wsi), sizeof(struct per_vhost_data__minimal));
    vhd->context = lws_get_context(wsi);
    vhd->protocol = lws_get_protocol(wsi);
    vhd->vhost = lws_get_vhost(wsi);
    printf("[motion_protocol] Protocol initialized\n");
    break;
        
  case LWS_CALLBACK_ESTABLISHED:
    lws_ll_fwd_insert(pss, pss_list, vhd->pss_list);
    pss->wsi = wsi;
    idx = getFreeConnectionIdx(ctx);
    if(idx >= 0)
      ctx->connections[idx] = vhd;
    printf("[motion_protocol] Connection established\n");           
    break;
        
  case LWS_CALLBACK_CLOSED:
    lws_ll_fwd_remove(struct per_session_data__minimal, pss_list, pss, vhd->pss_list);
    freeConnection(ctx, vhd);
    break;  
        
  case LWS_CALLBACK_RECEIVE:
    ic = (JoyPacket*) in;
    float tv = 0;
    float rv = 0;
    float tvscale = MAX_TV / 32767.0;
    float rvscale = MAX_RV / 32767.0;
    int axis = ic->axis;
    int value = ic->value;
    int type = ic->type;
    if (axis==TV_AXIS && type==2){
      tv = -value*tvscale;
    }
    if (axis==RV_AXIS && type==2){
      rv = -value*rvscale;
    }
    if(axis==HALT_BUTTON && type==1){
      tv = 0;
      rv = 0;
    }
    else if (axis==BOOST_BUTTON && type==1){
      if(value)
	gain = 2.0;
      else
	gain = 1.0;
    }
    ws_ctx->drive_control->header.seq = 1;
    ws_ctx->drive_control->translational_velocity = tv;
    ws_ctx->drive_control->rotational_velocity = rv;
    break;       
        
  default:
    break;
  }

  return 0;
}

int num_frame = 0;

static int callback_send_cam(struct lws *wsi,
                         enum lws_callback_reasons reason, void *user,
                         void *in, size_t len){
    
  struct per_session_data__minimal *pss = (struct per_session_data__minimal*) user;
  struct per_vhost_data__minimal *vhd = (struct per_vhost_data__minimal*) lws_protocol_vh_priv_get(lws_get_vhost(wsi),lws_get_protocol(wsi));
  const struct msg *pmsg;
  OrazioWSContext* ctx = ws_ctx;
  void *retval;
  int m, idx = 0;

  switch(reason){
        
    /* --- protocol lifecycle callbacks --- */
  case LWS_CALLBACK_PROTOCOL_INIT:
    vhd = lws_protocol_vh_priv_zalloc(lws_get_vhost(wsi), lws_get_protocol(wsi), 
				      sizeof(struct per_vhost_data__minimal));
    if(!vhd)
      return 1;

    pthread_mutex_init(&vhd->lock_ring, NULL);

    vhd->context = lws_get_context(wsi);
    vhd->protocol = lws_get_protocol(wsi);
    vhd->vhost = lws_get_vhost(wsi);
    
    vhd->ring = lws_ring_create(sizeof(struct msg), 8, __minimal_destroy_message);
    if (!vhd->ring) {
      lwsl_err("[Cam_service] %s: failed to create ring\n", __func__);
      return 1;
    }
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    if (pthread_create(&vhd->pthread_spam, &attr, thread_spam, vhd)) {
      lwsl_err("[Cam_service] thread creation failed\n");
      goto init_fail;
    }
    
    printf("[Cam_service] Protocol initialized\n");
    break;

  case LWS_CALLBACK_PROTOCOL_DESTROY:
init_fail:
    vhd->finished = 1;
    pthread_join(vhd->pthread_spam, &retval);

    if (vhd->ring)
      lws_ring_destroy(vhd->ring);

    pthread_mutex_destroy(&vhd->lock_ring);
    break;    

  case LWS_CALLBACK_ESTABLISHED:
    lws_ll_fwd_insert(pss, pss_list, vhd->pss_list);
    idx = getFreeConnectionIdx(ctx);
    if(idx >= 0)
      ctx->connections[idx] = vhd;
    pss->tail = lws_ring_get_oldest_tail(vhd->ring);
    pss->wsi = wsi;
    printf("[Cam_service] Connection established\n");
    break;
        
  case LWS_CALLBACK_CLOSED:
    lws_ll_fwd_remove(struct per_session_data__minimal, pss_list, pss, vhd->pss_list);
    freeConnection(ctx, vhd);
    break;
        
  case LWS_CALLBACK_SERVER_WRITEABLE:
    /*take one frame from queue and send on libwebsockets*/
    pthread_mutex_lock(&vhd->lock_ring); /* --------- ring lock { */

    pmsg = lws_ring_get_element(vhd->ring, &pss->tail);
    if (!pmsg) {
      pthread_mutex_unlock(&vhd->lock_ring); /* } ring lock ------- */
      break;
    }

    /* notice we allowed for LWS_PRE in the payload already */
    m = lws_write(wsi, ((unsigned char *)pmsg->payload) + LWS_PRE,
		  pmsg->len, LWS_WRITE_BINARY);
    if (m < (int)pmsg->len) {
      pthread_mutex_unlock(&vhd->lock_ring); /* } ring lock ------- */
      lwsl_err("[Cam_service] ERROR %d writing to ws socket\n", m);
      return -1;
    }

    lws_ring_consume_and_update_oldest_tail(
	    vhd->ring,	/* lws_ring object */
	    struct per_session_data__minimal, /* type of objects with tails */
	    &pss->tail,	/* tail of guy doing the consuming */
	    1,		/* number of payload objects being consumed */
	    vhd->pss_list,	/* head of list of objects with tails */
	    tail,		/* member name of tail in objects with tails */
	    pss_list	/* member name of next object in objects with tails */
    );

    /* more to do? */
    if (lws_ring_get_element(vhd->ring, &pss->tail))
      /* come back as soon as we can write more */
      lws_callback_on_writable(pss->wsi);

    pthread_mutex_unlock(&vhd->lock_ring); /* } ring lock ------- */
    
    break;

  case LWS_CALLBACK_RECEIVE:
    break;

  case LWS_CALLBACK_EVENT_WAIT_CANCELLED:
    if (!vhd)
      break;
    /*
     * When the "spam" threads add a message to the ringbuffer,
     * they create this event in the lws service thread context
     * using lws_cancel_service().
     *
     * We respond by scheduling a writable callback for all
     * connected clients.
     */
    lws_start_foreach_llp(struct per_session_data__minimal **,
			  ppss, vhd->pss_list) {
      lws_callback_on_writable((*ppss)->wsi);
    } lws_end_foreach_llp(ppss, pss_list);
    break;
        
  default:
    break;
  }

  return 0;
}

void* _websocketFn(void* args){
    
  OrazioWSContext* ctx = (OrazioWSContext*) args;
  ws_ctx = ctx;
  struct lws_context_creation_info info;
  struct lws_context *context;
  int n = 0;
  int port = ctx->port;
  int rate = ctx->rate;

  memset(&info, 0, sizeof info); /* otherwise uninitialized garbage */
  info.port = port;
  info.mounts = NULL;

  struct lws_protocols protocols[] = {
    {
      .name="http-only",   // name
      .callback=lws_callback_http_dummy, // callback
      .per_session_data_size=0,
      .rx_buffer_size=0,
      .user=NULL
    },
    {
      .name="exec_commands",
      .callback=callback_rcv_comm,
      .per_session_data_size=sizeof(struct per_session_data__minimal),
      .rx_buffer_size=128,
      .id=0,
      .user=NULL,
      .tx_packet_size=0
    },
    {
      .name="cam_protocol",
      .callback=callback_send_cam,
      .per_session_data_size=sizeof(struct per_session_data__minimal),
      .rx_buffer_size=4096,
      .id=0,
      .user=NULL, 
      .tx_packet_size=0
    },
    {
      .name=NULL,
      .callback=NULL,
      .per_session_data_size=0,   
      .rx_buffer_size=0,
      .user=NULL
    }
  };
    
  info.protocols = protocols;
  info.vhost_name = "localhost";
  info.ws_ping_pong_interval = 10;

  context = lws_create_context(&info);
  if(!context)
    exit(1);
    
  ctx->run = 1;    
  while (ctx->run && n >= 0){
    n = lws_service(context, 10);
    usleep(1000000/rate);
  }
    
  lws_context_destroy(context);
  return 0;
}

OrazioWSContext* OrazioWebsocketServer_start(struct OrazioClient* client,
                                             int port,
                                             char* resource_path,
                                             int rate,
                                             char* cam,
                                             DifferentialDriveControlPacket* _drive_control ){
  OrazioWSContext* context = (OrazioWSContext*) malloc(sizeof(OrazioWSContext));
  context->port = port;
  context->client = client;
  context->rate = rate;
  initConnections(context);
  context->cam = cam;
  context->camera = camera_initialize(context->cam, WIDTH, HEIGHT);
  context->drive_control = _drive_control;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_create(&context->thread, &attr, _websocketFn, context);
  return context;
}

void OrazioWebsocketServer_stop(OrazioWSContext* context){
  context->run = 0;
  void* retval;
  pthread_join(context->thread, &retval);
  camera_finish(context->camera);
  camera_close(context->camera);
  free(context);
}
