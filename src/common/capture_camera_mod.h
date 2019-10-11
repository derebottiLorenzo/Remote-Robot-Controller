#pragma once
#include <stdint.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <asm/types.h>
#include <linux/videodev2.h>

typedef struct buffer_t{
	uint8_t* start;
	size_t length;
} buffer_t;

typedef struct camera_t{
	int fd;
	uint32_t width;
	uint32_t height;
	buffer_t head;        // buffer for the current image

	size_t buffer_count;
	buffer_t* buffers;    // image buffers four nimage buffers
} camera_t;



camera_t* camera_initialize(char* dev, int width, int height);
int camera_frame(camera_t* camera, struct timeval timeout);
void camera_finish(camera_t *camera);
void camera_close(camera_t *camera);
uint8_t* yuyv2rgb(uint8_t* yuyv, uint32_t width, uint32_t height);
void jpeg(FILE* dest, uint8_t* rgb, uint32_t width, uint32_t height, int quality);