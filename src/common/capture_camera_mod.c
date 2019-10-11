//Most of the code taken from github
//https://gist.github.com/bellbind/6813905
//copyright: him

/*
 * capturing from UVC cam
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <opencv2/highgui/highgui_c.h>
#include <jpeglib.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include "capture_camera_mod.h"

#define FALSE 0
#define TRUE 1

void quit(const char *msg){
  fprintf(stderr, "[%s] %d: %s\n", msg, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

int xioctl(int fd, int request, void *arg){
  for (int i = 0; i < 100; i++){
    int r = ioctl(fd, request, arg);
    if (r != -1 || errno != EINTR)
      return r;
  }
  return -1;
}

/*
  Opens the camera device and stores the requested image size in the camera struct
*/

camera_t *camera_open(const char *device, uint32_t width, uint32_t height){
  int fd = open(device, O_RDWR | O_NONBLOCK, 0);
  if (fd == -1)
    quit("open");
  camera_t *camera = malloc(sizeof(camera_t));
  camera->fd = fd;
  camera->width = width;
  camera->height = height;
  camera->buffer_count = 0;
  camera->buffers = NULL;
  camera->head.length = 0;
  camera->head.start = NULL;
  printf("device opened\n");
  return camera;
}

/*
  1. queries the capability of he camera
  2. checks if device supports cropping
  3. allocates memory buffers for dma operation
  4. sets up mmap with the requested buffers
*/
void camera_init(camera_t *camera){
  struct v4l2_capability cap;
  if (xioctl(camera->fd, VIDIOC_QUERYCAP, &cap) == -1)
    quit("VIDIOC_QUERYCAP");
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    quit("no capture");
  if (!(cap.capabilities & V4L2_CAP_STREAMING))
    quit("no streaming");
  printf("camera supports capture and streaming\n");

  struct v4l2_cropcap cropcap;
  memset(&cropcap, 0, sizeof cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(camera->fd, VIDIOC_CROPCAP, &cropcap) == 0){
    struct v4l2_crop crop;
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect;
    if (xioctl(camera->fd, VIDIOC_S_CROP, &crop) == -1)
      {
	// cropping not supported
      }
  }
  printf("camera supports cropping\n");

  struct v4l2_format format;
  memset(&format, 0, sizeof format);
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = camera->width;
  format.fmt.pix.height = camera->height;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  format.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(camera->fd, VIDIOC_S_FMT, &format) == -1)
    quit("VIDIOC_S_FMT");
  printf("set format to %d x %d\n", camera->width, camera->height);

  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof req);
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(camera->fd, VIDIOC_REQBUFS, &req) == -1)
    quit("VIDIOC_REQBUFS");
  camera->buffer_count = req.count;
  camera->buffers = calloc(req.count, sizeof(buffer_t));
  printf("allocated %d buffers\n", req.count);

  //here we do a mmap for each individual buffer
  size_t buf_max = 0;
  for (size_t i = 0; i < camera->buffer_count; i++){
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buf) == -1)
      quit("VIDIOC_QUERYBUF");
    if (buf.length > buf_max)
      buf_max = buf.length;
    camera->buffers[i].length = buf.length;
    camera->buffers[i].start =
      mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
	   camera->fd, buf.m.offset);
    if (camera->buffers[i].start == MAP_FAILED)
      quit("mmap");
    printf("mmapping buffer[%d]\n", (int)i);
  }
  camera->head.start = malloc(buf_max);
}

// starts the streaming (one single xioctl)
void camera_start(camera_t *camera){
  for (size_t i = 0; i < camera->buffer_count; i++){
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1)
      quit("VIDIOC_QBUF"); // query for a buffer
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(camera->fd, VIDIOC_STREAMON, &type) == -1)
    quit("VIDIOC_STREAMON");
}

// stops the streaming
void camera_stop(camera_t *camera){
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(camera->fd, VIDIOC_STREAMOFF, &type) == -1)
    quit("VIDIOC_STREAMOFF");
}

// unmaps the buffers
void camera_finish(camera_t *camera){
  for (size_t i = 0; i < camera->buffer_count; i++){
    munmap(camera->buffers[i].start, camera->buffers[i].length);
  }
  free(camera->buffers);
  camera->buffer_count = 0;
  camera->buffers = NULL;
  free(camera->head.start);
  camera->head.length = 0;
  camera->head.start = NULL;
}

// closes the device
void camera_close(camera_t *camera){
  if (close(camera->fd) == -1)
    quit("close");
  free(camera);
}

// captures a frame from the current buffer
int camera_capture(camera_t *camera){
  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(camera->fd, VIDIOC_DQBUF, &buf) == -1)
    return FALSE; // buffer exchange with the driver - full
  memcpy(camera->head.start, camera->buffers[buf.index].start, buf.bytesused);
  camera->head.length = buf.bytesused;
  if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1)
    return FALSE; // buffer exchange with the driver - empty
  return TRUE;
}

int camera_frame(camera_t *camera, struct timeval timeout){
  // waits fror a new frame, when camera ready
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(camera->fd, &fds);
  int r = select(camera->fd + 1, &fds, 0, 0, &timeout);
  if (r == -1)
    quit("select");
  if (r == 0)
    return FALSE;
  return camera_capture(camera);
}

camera_t *camera_initialize(char* dev, int width, int height){
  camera_t *camera = camera_open(dev, width, height);
  camera_init(camera);
  camera_start(camera);

  return camera;
}

void savePGM(camera_t *camera, char *filename){
  FILE *f = fopen(filename, "w");
  if (!f)
    return;
  fprintf(f, "P5\n%d %d\n255\n", camera->width, camera->height);
  int size = camera->width * camera->height;
  char buffer[size];
  char *dest = buffer;
  char *src = camera->head.start;
  for (int r = 0; r < camera->height; ++r){
    for (int c = 0; c < camera->width; ++c, ++dest, src += 2){
      *dest = *src;
    }
  }
  fwrite(buffer, size, 1, f);
  fclose(f);
}

void jpeg(FILE* dest, uint8_t* rgb, uint32_t width, uint32_t height, int quality){
  JSAMPARRAY image;
  image = calloc(height, sizeof (JSAMPROW));
  for (size_t i = 0; i < height; i++) {
    image[i] = calloc(width * 3, sizeof (JSAMPLE));
    for (size_t j = 0; j < width; j++) {
      image[i][j * 3 + 0] = rgb[(i * width + j) * 3 + 0];
      image[i][j * 3 + 1] = rgb[(i * width + j) * 3 + 1];
      image[i][j * 3 + 2] = rgb[(i * width + j) * 3 + 2];
    }
  }
  
  struct jpeg_compress_struct compress;
  struct jpeg_error_mgr error;
  compress.err = jpeg_std_error(&error);
  jpeg_create_compress(&compress);
  jpeg_stdio_dest(&compress, dest);
  
  compress.image_width = width;
  compress.image_height = height;
  compress.input_components = 3;
  compress.in_color_space = JCS_RGB;
  jpeg_set_defaults(&compress);
  jpeg_set_quality(&compress, quality, TRUE);
  jpeg_start_compress(&compress, TRUE);
  jpeg_write_scanlines(&compress, image, height);
  jpeg_finish_compress(&compress);
  jpeg_destroy_compress(&compress);

  for (size_t i = 0; i < height; i++) {
    free(image[i]);
  }
  free(image);
}

int minmax(int min, int v, int max){
  return (v < min) ? min : (max < v) ? max : v;
}

uint8_t* yuyv2rgb(uint8_t* yuyv, uint32_t width, uint32_t height){
  uint8_t* rgb = calloc(width * height * 3, sizeof (uint8_t));
  for (size_t i = 0; i < height; i++) {
    for (size_t j = 0; j < width; j += 2) {
      size_t index = i * width + j;
      int y0 = yuyv[index * 2 + 0] << 8;
      int u = yuyv[index * 2 + 1] - 128;
      int y1 = yuyv[index * 2 + 2] << 8;
      int v = yuyv[index * 2 + 3] - 128;
      rgb[index * 3 + 0] = minmax(0, (y0 + 359 * v) >> 8, 255);
      rgb[index * 3 + 1] = minmax(0, (y0 + 88 * v - 183 * u) >> 8, 255);
      rgb[index * 3 + 2] = minmax(0, (y0 + 454 * u) >> 8, 255);
      rgb[index * 3 + 3] = minmax(0, (y1 + 359 * v) >> 8, 255);
      rgb[index * 3 + 4] = minmax(0, (y1 + 88 * v - 183 * u) >> 8, 255);
      rgb[index * 3 + 5] = minmax(0, (y1 + 454 * u) >> 8, 255);
    }
  }
  return rgb;
}
