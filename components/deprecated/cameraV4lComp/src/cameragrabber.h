//--------------------------------------------------------------------------
// Clase CameraSimple I
//--------------------------------------------------------------------------

#ifndef CAMERAGRABBER_H
#define CAMERAGRABBER_H

#include <stdio.h>                                                           
#include <stdlib.h>                                                          
#include <unistd.h>                                                          
#include <sys/types.h>                                                       
#include <sys/stat.h>                                                        
#include <sys/file.h>                                                        
#include <string.h>                                                          
#include <linux/videodev2.h>                                                 
#include <sys/ioctl.h>                                                       
#include <sys/mman.h>                                                        
#include <errno.h>                                                           
#include <fcntl.h>                                                           
#include <time.h>                                                            
#include <sys/time.h>                                                        
#include <signal.h>       

using namespace std;                                                                                                    
                                                                             
/* Fixed point arithmetic */                                                 
#define FIXED Sint32                                                         
#define FIXED_BITS 16                                                        
#define TO_FIXED(X) (((Sint32)(X))<<(FIXED_BITS))                            
#define FROM_FIXED(X) (((Sint32)(X))>>(FIXED_BITS))                          
#define NB_BUFFER 4                                                          
#define DHT_SIZE 432
#define HEADERFRAME1 0xaf

#ifndef V4L2_CID_BACKLIGHT_COMPENSATION                                      
#define V4L2_CID_BACKLIGHT_COMPENSATION (V4L2_CID_PRIVATE_BASE+0)            
#define V4L2_CID_POWER_LINE_FREQUENCY   (V4L2_CID_PRIVATE_BASE+1)            
#define V4L2_CID_SHARPNESS              (V4L2_CID_PRIVATE_BASE+2)            
#define V4L2_CID_HUE_AUTO               (V4L2_CID_PRIVATE_BASE+3)            
#define V4L2_CID_FOCUS_AUTO             (V4L2_CID_PRIVATE_BASE+4)            
#define V4L2_CID_FOCUS_ABSOLUTE         (V4L2_CID_PRIVATE_BASE+5)            
#define V4L2_CID_FOCUS_RELATIVE         (V4L2_CID_PRIVATE_BASE+6)            
#define V4L2_CID_EXPOSURE_AUTO          (V4L2_CID_PRIVATE_BASE+10)           
#define V4L2_CID_EXPOSURE_ABSOLUTE              (V4L2_CID_PRIVATE_BASE+11)   
#define V4L2_CID_POWER_LINE_FREQUENCY_DISABLED 0                             
#define V4L2_CID_POWER_LINE_FREQUENCY_50HZ 1                                 
#define V4L2_CID_POWER_LINE_FREQUENCY_60HZ 2                                 
#endif 

#define MAX_CAMERAS 10

struct vdIn {                                                                
  int fd;                                                                  
  char *videodevice;                                                       
  char *status;                                                            
  struct v4l2_capability cap;                                              
  struct v4l2_format fmt;                                                  
  struct v4l2_buffer buf;                                                  
  struct v4l2_requestbuffers rb;                                           
  void *mem[NB_BUFFER];                                                    
  unsigned char *framebuffer;       
  int isstreaming;                                                         
  int grabmethod;                                                          
  int width;                                                               
  int height;                                                              
  int fps;                                                                 
  int formatIn;                                                            
  int formatOut;                                                           
  int framesizeIn;                                                         
};

class CameraGrabber
{
	public:
	CameraGrabber();
	virtual ~CameraGrabber();
	void grab();
	bool init();                                                 
	
	static void *runWrap(void *arg)
	{
		((CameraGrabber*) arg)->run();
	}

	private:                                                             
	vdIn vd[MAX_CAMERAS];                                        
	void uvcGrab();  
	int isv4l2Control(struct vdIn *vd, int control, struct v4l2_queryctrl *queryctrl);
	int enum_frame_intervals(int dev, __u32 pixfmt, __u32 width, __u32 height);
	int enum_frame_sizes(int dev, __u32 pixfmt);

};
//----------------------------------------------------------------------------
#endif
