#pragma once
#include "orazio_client.h"

struct OrazioWSContext;
struct joy_packet;

// starts a websocket server bind to the shell
struct OrazioWSContext* OrazioWebsocketServer_start(struct OrazioClient* client,
                                             int port,
                                             char* resource_path,
                                             int rate,
                                             char* cam,
                                             DifferentialDriveControlPacket* _drive_control);

// stops a websocket server bind to the shell
void OrazioWebsocketServer_stop(struct OrazioWSContext* context);