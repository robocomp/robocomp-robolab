#include "cameragrabber.h"

CameraGrabber::CameraGrabber(RoboCompCameraSimplePub::CameraSimplePubPrx camera_proxy_) : camera_proxy(camera_proxy_)
{
    bool flag =init();
};

CameraGrabber::~CameraGrabber()
{
    for (int i=0; i<MAX_CAMERAS; ++i)
    {                                    
      if ( vd[i].isstreaming )
      {                                     
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;              
        ioctl ( vd[i].fd, VIDIOC_STREAMOFF, &type );         
        vd[i].isstreaming = 0;                               
      }                                                            
      
      free(vd[i].framebuffer); 
      vd[i].framebuffer = NULL;                                    
      free(vd[i].videodevice);                                     
      free(vd[i].status);                                          
      vd[i].videodevice = NULL;                                    
      vd[i].status = NULL;                                         
    } 
  };

//--------------------------------------------------------------------------
// Otros metodos
//--------------------------------------------------------------------------
                          
int CameraGrabber::isv4l2Control(struct vdIn *vd, int control, struct v4l2_queryctrl *queryctrl)
{                                                                                
  int err =0;                                                              
  queryctrl->id = control;                                                 
  err = ioctl(vd->fd, VIDIOC_QUERYCTRL, queryctrl);                        
  if (err < 0)                                                             
  {                                                                        
     perror("ioctl querycontrol error");                              
  }                                                                        
  else if (queryctrl->flags & V4L2_CTRL_FLAG_DISABLED)                     
  {                                                                        
    printf("control %s disabled\n", (char *) queryctrl->name);       
  }                                                                        
  else if (queryctrl->flags & V4L2_CTRL_TYPE_BOOLEAN)                      
  {                                                                        
    return 1;                                                        
  } 
  else if (queryctrl->type & V4L2_CTRL_TYPE_INTEGER)                       
  {                                                                        
    return 0;                                                        
  }                                                                        
  else {                                                                   
    printf("contol %s unsupported\n", (char *) queryctrl->name);     
    return 0;                                                        
  }                                                                        
  return -1;
}


int CameraGrabber::enum_frame_intervals(int dev, __u32 pixfmt, __u32 width, __u32 height)       
{                                                                                
  int ret;                                                                 
  struct v4l2_frmivalenum fival;

  memset(&fival, 0, sizeof(fival));                                        
  fival.index = 0;                                                         
  fival.pixel_format = pixfmt;                                             
  fival.width = width;                                                     
  fival.height = height;                                                   
  printf("\t\t\t Time interval between frame: ");                               
  while ((ret = ioctl(dev, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0) {    
    if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
    {
      printf("%u/%u, ",                                
                       fival.discrete.numerator, fival.discrete.denominator);
    }
    else if (fival.type == V4L2_FRMIVAL_TYPE_CONTINUOUS)
    {                              
      printf("{min { %u/%u } .. max { %u/%u } }, ",                         
                       fival.stepwise.min.numerator, fival.stepwise.min.numerator,
                       fival.stepwise.max.denominator, fival.stepwise.max.denominator);
      break;                                                                          
    }
    else if (fival.type == V4L2_FRMIVAL_TYPE_STEPWISE)
    {                                          
      printf("{min { %u/%u } .. max { %u/%u } / "                                     
                       "stepsize { %u/%u } }, ",                                       
                       fival.stepwise.min.numerator, fival.stepwise.min.denominator,   
                       fival.stepwise.max.numerator, fival.stepwise.max.denominator,   
                       fival.stepwise.step.numerator, fival.stepwise.step.denominator);
      break;                                                                          
    }
    
    fival.index++;
    }
    if (ret != 0 && errno != EINVAL) {                                                                      
      printf("ERROR enumerating frame sizes: %d\n", errno);                                           
      return errno;                                                                                   
    }                                                                                                       
   
   return 0; 
}    

int CameraGrabber::enum_frame_sizes(int dev, __u32 pixfmt)
{
	int ret;
	struct v4l2_frmsizeenum fsize;

	memset(&fsize, 0, sizeof(fsize));
	fsize.index = 0;
	fsize.pixel_format = pixfmt;
	while ((ret = ioctl(dev, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0) {
		if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
			printf("\t\t { discrete: width = %u, height = %u }\n",
					fsize.discrete.width, fsize.discrete.height);
			ret = enum_frame_intervals(dev, pixfmt,
					fsize.discrete.width, fsize.discrete.height);
			if (ret != 0)
				printf("  Unable to enumerate frame sizes.\n");
		} else if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {
			printf("\t\t { continuous: min { width = %u, height = %u } .. "
					"max { width = %u, height = %u } }\n",
					fsize.stepwise.min_width, fsize.stepwise.min_height,
					fsize.stepwise.max_width, fsize.stepwise.max_height);
			printf("  Refusing to enumerate frame intervals.\n");
			break;
		} else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
			printf("\t\t { stepwise: min { width = %u, height = %u } .. "
					"max { width = %u, height = %u } / "
					"stepsize { width = %u, height = %u } }\n",
					fsize.stepwise.min_width, fsize.stepwise.min_height,
					fsize.stepwise.max_width, fsize.stepwise.max_height,
					fsize.stepwise.step_width, fsize.stepwise.step_height);
			printf("  Refusing to enumerate frame intervals.\n");
			break;
		}
		fsize.index++;
	}
	if (ret != 0 && errno != EINVAL) {
		printf("ERROR enumerating frame sizes: %d\n", errno);
		return errno;
	}
	printf("\n");
	return 0;
}

bool CameraGrabber::init(std::list<std::string> cameras)
{
  int grabmethod = 1; // mmap                                                                             
  int format;                                                                                             
  format = V4L2_PIX_FMT_YUYV;                                                                             
                                                                                                                                                                                                                                                        
  int numCameras;
  if (cameras.size() < MAX_CAMERAS )
	numCameras = cameras.size();
  else
	  qFatal("Aborting. Number of cameras greater than max allowed");
  
  //Open devices
  for (int i=0; i<numCameras; ++i)                                                                        
  {                                                                                                       
    memset(&vd[i], 0, sizeof(struct vdIn));
    vd[i].videodevice = (char *)calloc(1+1, sizeof(char));                                          
    vd[i].status = (char *)calloc(1, 100*sizeof(char));                                             
    //Esto deber..a hacerse con una expresi..n regular                                              
    snprintf(vd[i].videodevice, 12, "%s", "/dev/video0");                                           
    printf ( "CameraGrabber::init() \n\t Camera info: current video device %s \n", vd[i].videodevice );           
    vd[i].width = 320;                                                                              
    vd[i].height = 240;                                                                             
    vd[i].formatIn = format;                                                                        
    vd[i].grabmethod = grabmethod;                                                                  
    vd[i].fps = 30;
	printf ( "\t Camera info: setting formtat to V4L2_PIX_FMT_YUYV, size to %d,%d and fps to %d\n",  vd[i].width,  vd[i].height,  vd[i].fps );       
    if ((vd[i].fd = open(vd[i].videodevice, O_RDWR)) == -1 )                                        
             printf ( "CameraGrabber::init() fatal error: Opening V4L interface" ); 

    memset(&vd[i].cap, 0, sizeof(struct v4l2_capability));
    
    if (ioctl(vd[i].fd, VIDIOC_QUERYCAP, &vd[i].cap) < 0)                                            
                           printf ( "CameraGrabber::init() fatal error: Unable to query device %s ", vd[i].videodevice );                                                                                                                                                    
    if ((vd[i].cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)==0)                                            
                           printf ( "CameraGrabber::init() fatal error: video capture not supported %s ", vd[i].videodevice );                                                                                                                                                                                         
    if (vd[i].grabmethod)                                                                                    
    {                                                                                                        
      if (!(vd[i].cap.capabilities & V4L2_CAP_STREAMING))                                              
        printf ( "CameraGrabber::init() fatal error: %s does not support streaming", vd[i].videodevice );
    }                                                                                                        
    else                                                                                                     
    {                                                                                                        
      if (!(vd[i].cap.capabilities & V4L2_CAP_READWRITE) )                                             
        printf ( "CameraGrabber::init() fatal error: %s does not support read i/o ", vd[i].videodevice );
    } 

    struct v4l2_fmtdesc fmt;                                                                                
    memset(&fmt, 0, sizeof(fmt));                                                                           
    fmt.index = 0;                                                                                          
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 

    while (ioctl(vd[i].fd, VIDIOC_ENUM_FMT, &fmt) == 0)
    {
		fmt.index++;
        printf("\t { pixelformat = '%c%c%c%c', description = '%s' }\n",fmt.pixelformat & 0xFF, (fmt.pixelformat >> 8) & 0xFF, 	(fmt.pixelformat >> 16) & 0xFF, (fmt.pixelformat >> 24) & 0xFF,	fmt.description);
		if (enum_frame_sizes(vd[i].fd, fmt.pixelformat) != 0)
			printf("  Unable to enumerate frame sizes.\n");
    }
    // set format in
    memset (&vd[i].fmt, 0, sizeof(struct v4l2_format));
    vd[i].fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vd[i].fmt.fmt.pix.width = vd[i].width;
    vd[i].fmt.fmt.pix.height = vd[i].height;
    vd[i].fmt.fmt.pix.pixelformat = vd[i].formatIn;
    vd[i].fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (ioctl(vd[i].fd, VIDIOC_S_FMT, &vd[i].fmt) < 0)
	printf ( "CameraGrabber::init() fatal error: Unable to set format through VIDIOC_S_FMT" );
    if ( (vd[i].fmt.fmt.pix.width!=(uint)vd[i].width) || (vd[i].fmt.fmt.pix.height!=(uint)vd[i].height) )
	printf ( "CameraGrabber::init() fatal error: Size %dx%d is not available. Suggested %dx%d", vd[i].width, vd[i].height, vd[i].fmt.fmt.pix.width, vd[i].fmt.fmt.pix.height);

		/* set framerate */
//		struct v4l2_streamparm* setfps;
//		setfps=(struct v4l2_streamparm *) calloc(1, sizeof(struct v4l2_streamparm));
//		memset(setfps, 0, sizeof(struct v4l2_streamparm));
//		setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//		setfps->parm.capture.timeperframe.numerator=1;
//		setfps->parm.capture.timeperframe.denominator=30;
//		if( ioctl(vd[i].fd, VIDIOC_S_PARM, setfps) < 0 )
//				printf ( "CameraGrabber::init() fatal error: Unable to set frame rate through VIDIOC_S_PARM" );

    // request buffers
    memset(&vd[i].rb, 0, sizeof(struct v4l2_requestbuffers));
    vd[i].rb.count = NB_BUFFER;
    vd[i].rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vd[i].rb.memory = V4L2_MEMORY_MMAP;
    if (ioctl(vd[i].fd, VIDIOC_REQBUFS, &vd[i].rb) < 0)
	printf ( "CameraGrabber::init() fatal error: Unable to allocate buffers through VIDIOC_REQBUFS" );

    // map the buffers
    for (int ii=0; ii<NB_BUFFER; ii++)
    {
		memset(&vd[i].buf, 0, sizeof(struct v4l2_buffer));
		vd[i].buf.index = ii;
		vd[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vd[i].buf.memory = V4L2_MEMORY_MMAP;
		if (ioctl(vd[i].fd, VIDIOC_QUERYBUF, &vd[i].buf ) < 0)
			printf ( "CameraGrabber::init() fatal error: Unable to query buffer through VIDIOC_QUERYBUF. Error number %d ", errno );
		vd[i].mem[ii] = mmap(0, vd[i].buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, vd[i].fd, vd[i].buf.m.offset );
		if (vd[i].mem[ii] == MAP_FAILED)
			printf ( "CameraGrabber::init() fatal error: Unable to map the buffer through mmap. Error number: %d ",errno );
	}
     // Queue the buffers.
     for (int ii = 0; ii < NB_BUFFER; ++ii)
     {
       memset(&vd[i].buf, 0, sizeof(struct v4l2_buffer));
       vd[i].buf.index = ii;
       vd[i].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
       vd[i].buf.memory = V4L2_MEMORY_MMAP;
       if (ioctl(vd[i].fd, VIDIOC_QBUF, &vd[i].buf) < 0)
         printf ( "CameraGrabber::init() fatal error: Unable to queue buffers through VIDIOC_QBUF. Error number: %d",errno );
     }

     // alloc a temp buffer to reconstruct the pict
     vd[i].framesizeIn = ( vd[i].width * vd[i].height << 1 );
     switch ( vd[i].formatIn )
     {
		case V4L2_PIX_FMT_MJPEG:
			printf("MJPEG not supported");
			// 			if ((vd[i].tmpbuffer=(unsigned char *)calloc(1, (size_t)vd[i].framesizeIn)) == 0)
			// 				printf ( "CameraGrabber::init() fatal error: not enough memory to allocate vd[i].tmpbuffer" );
			// 			vd[i].framebuffer = (unsigned char *)calloc(1, (size_t)vd[i].width*(vd[i].height + 8)*2);
		break;
		case V4L2_PIX_FMT_YUYV:
 			vd[i].framebuffer = (unsigned char *)calloc(1, ( size_t ) vd[i].framesizeIn);
			break;
		case V4L2_PIX_FMT_GREY:
			vd[i].framebuffer = (unsigned char *)calloc(1, ( size_t ) vd[i].framesizeIn);
			printf("V4LCapture::init() V4L2 error: Format not recognized!!\n");
			break;
		default:
			printf("V4LCapture::init() V4L2 error: Format not recognized!!\n");
			return false;
      }

      if (vd[i].framebuffer == 0)
		printf ( "CameraGrabber::init() fatal error: not enough memory to allocate vd[i].framebuffer" );

	int control;
	struct v4l2_control control_s;
	struct v4l2_queryctrl queryctrl;
	int err;

	// Set the power line frequency value
	control = V4L2_CID_POWER_LINE_FREQUENCY;
	control_s.id = control;
	if (isv4l2Control(&vd[i], control, &queryctrl) < 0) return false;
	  control_s.value =  V4L2_CID_POWER_LINE_FREQUENCY_60HZ;
	if ((err = ioctl(vd[i].fd, VIDIOC_S_CTRL, &control_s)) < 0)
	  printf("CameraGrabber::init() V4L2 error: ioctl set_light_frequency_filter error\n");
		
	// Set the saturation value
	control = V4L2_CID_SATURATION;
	control_s.id = control;
	if (isv4l2Control(&vd[i], control, &queryctrl) < 0) return false;
	
	// Set the auto exposure off for max speed
	control = V4L2_CID_EXPOSURE_AUTO;
	control_s.id = control;
//		if (isv4l2Control(&vd[i], control, &queryctrl) < 0) return false;
//		control_s.value = 8;
// 		if ((err = ioctl(vd[i].fd, VIDIOC_S_CTRL, &control_s)) < 0)
// 			printf("V4LCapture::init() V4L2 error: ioctl set_exposureOff error\n");
// 		printf("CameraComp: AutoExposure: %d\n", 1);
//
// 	// Set the auto exposure off for max speed
 //		control = V4L2_CID_EXPOSURE_ABSOLUTE;
 //		control_s.id = control;
 //		if (isv4l2Control(&vd[i], control, &queryctrl) < 0) return false;
 //		control_s.value = 200;
 //		if ((err = ioctl(vd[i].fd, VIDIOC_S_CTRL, &control_s)) < 0)
 //			printf("V4LCapture::init() V4L2 error: ioctl set_exposureOff error\n");
 //		printf("CameraComp: AutoExposure: %d\n", 900);


  }
  return true;
}

void CameraGrabber::uvcGrab(  )
{
    if (vd[0].isstreaming == false)
    {
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(vd[0].fd, VIDIOC_STREAMON, &type) < 0)
	  printf ( "CameraGrabber fatal error: Unable to start capture using VIDIOC_STREAMON: Error number: %d", errno );
	else
	  vd[0].isstreaming = 1;
    }

    memset(&vd[0].buf, 0, sizeof(struct v4l2_buffer));
    vd[0].buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vd[0].buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(vd[0].fd, VIDIOC_DQBUF, &vd[0].buf) < 0)
      printf("CameraGrabber fatal error: Unable to dequeue buffer using VIDIOC_DQBUF: Error number: %d", errno);

	if (vd[0].buf.bytesused > (uint)vd[0].framesizeIn)
		memcpy(vd[0].framebuffer, vd[0].mem[vd[0].buf.index], (size_t) vd[0].framesizeIn);
	else
		memcpy(vd[0].framebuffer, vd[0].mem[vd[0].buf.index], (size_t) vd[0].buf.bytesused);

	if (ioctl(vd[0].fd, VIDIOC_QBUF, &vd[0].buf) < 0)
      printf ( "CameraGrabber fatal error: Unable to requeue buffer using VIDIOC_QBUF: Error number: %d", errno );
}

///* Run thread method

void CameraGrabber::run()
{
  int i = 0;
  RoboCompCameraSimplePub::TImg img;
  img.resize( vd[0].buf.length );
  while (true)
  {
 	uvcGrab();
	memcpy(&img[0], vd[0].framebuffer, vd[0].buf.length );
// 	try
// 	{ camera_proxy->putYUVImage( img );}
// 	catch(const Ice::Exception &ex)
// 	{ std::cout << ex << std::endl; }
    i++;
    if(i%10 == 0)
		printf("publishing %d\n",i);
  }
}

