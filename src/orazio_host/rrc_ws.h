#pragma once
#include "orazio_client.h"

struct OrazioWSContext;
struct joy_packet;

// starts a websocket server biund to the shell
struct OrazioWSContext* OrazioWebsocketServer_start(struct OrazioClient* client,
                                                    int port,
                                                    char* resource_path,
                                                    int rate,
                                                    DifferentialDriveControlPacket* drive_control_);

// stops a websocket server biund to the shell
void OrazioWebsocketServer_stop(struct OrazioWSContext* context);