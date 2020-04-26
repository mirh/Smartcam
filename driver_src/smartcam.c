/*
 * SmartCam Video Capture driver - This code emulates a real video device with v4l2 api
 *
 * Copyright (C) 2008 Ionut Dediu <deionut@yahoo.com>
 *
 * Licensed under the GNU General Public License Version 2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 /*
  * Authors:
  * Ionut Dediu <deionut@yahoo.com> : main author
  * Tomas Janousek <tomi@nomi.cz> : implement YUYV pixel format, fix poll and nonblock
  */

#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <linux/version.h>
//#include <linux/videodev2.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-common.h>

#define SMARTCAM_MODULE_NAME "smartcam"
#define SMARTCAM_MAJOR_VERSION 0
#define SMARTCAM_MINOR_VERSION 1
#define SMARTCAM_RELEASE 0
#define SMARTCAM_VERSION KERNEL_VERSION(SMARTCAM_MAJOR_VERSION, SMARTCAM_MINOR_VERSION, SMARTCAM_RELEASE)

#define SMARTCAM_FRAME_WIDTH	320
#define SMARTCAM_FRAME_HEIGHT	240
#define SMARTCAM_YUYV_FRAME_SIZE	SMARTCAM_FRAME_WIDTH * SMARTCAM_FRAME_HEIGHT * 2
#define SMARTCAM_RGB_FRAME_SIZE	SMARTCAM_FRAME_WIDTH * SMARTCAM_FRAME_HEIGHT * 3
#define SMARTCAM_BUFFER_SIZE	((SMARTCAM_RGB_FRAME_SIZE + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1))
#define MAX_STREAMING_BUFFERS	7
#define SMARTCAM_NFORMATS 2

//#define SMARTCAM_DEBUG
#undef SCAM_MSG				/* undef it, just in case */
#ifdef SMARTCAM_DEBUG
#     define SCAM_MSG(fmt, args...) printk(KERN_ALERT "smartcam:" fmt, ## args)
# else
#     define SCAM_MSG(fmt, args...)	/* not debugging: nothing */
#endif

/* ------------------------------------------------------------------
    Basic structures
   ------------------------------------------------------------------*/

struct smartcam_dev {
    struct v4l2_device  v4l2_dev;
    struct video_device vdev;
    struct mutex        mutex;
};


static struct v4l2_pix_format formats[] = {
{
    .width = SMARTCAM_FRAME_WIDTH,
    .height = SMARTCAM_FRAME_HEIGHT,
    .pixelformat = V4L2_PIX_FMT_YUYV,
    .field = V4L2_FIELD_NONE,
    .bytesperline = SMARTCAM_YUYV_FRAME_SIZE / SMARTCAM_FRAME_HEIGHT,
    .sizeimage = SMARTCAM_YUYV_FRAME_SIZE,
    .colorspace = V4L2_COLORSPACE_SMPTE170M,
    .priv = 0,
}, {
    .width = SMARTCAM_FRAME_WIDTH,
    .height = SMARTCAM_FRAME_HEIGHT,
    .pixelformat = V4L2_PIX_FMT_RGB24,
    .field = V4L2_FIELD_NONE,
    .bytesperline = SMARTCAM_RGB_FRAME_SIZE / SMARTCAM_FRAME_HEIGHT,
    .sizeimage = SMARTCAM_RGB_FRAME_SIZE,
    .colorspace = V4L2_COLORSPACE_SRGB,
    .priv = 0,
} };

static const char fmtdesc[2][5] = { "YUYV", "RGB3" };

static DECLARE_WAIT_QUEUE_HEAD(wq);

static char* frame_data = NULL;
static __u32 frame_sequence = 0;
static __u32 last_read_frame = 0;
static __u32 format = 0;
static struct timeval frame_timestamp;

static inline void v4l2l_get_timestamp(struct timeval *tv) {
	/* ktime_get_ts is considered deprecated, so use ktime_get_ts64 if possible */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
	struct timespec ts;
	ktime_get_ts(&ts);
#else
	struct timespec64 ts;
	ktime_get_ts64(&ts);
#endif

	tv->tv_sec = (time_t)ts.tv_sec;
	tv->tv_usec = (suseconds_t)(ts.tv_nsec / NSEC_PER_USEC);
}

/* ------------------------------------------------------------------
    IOCTL vidioc handling
   ------------------------------------------------------------------*/
static int vidioc_querycap(struct file *file, void  *priv, struct v4l2_capability *cap)
{
    struct smartcam_dev *dev = video_drvdata(file);
    strcpy(cap->driver, "smartcam");
    strcpy(cap->card, "smartcam");
    snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", dev->v4l2_dev.name);
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
    cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
#endif
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
    return 0;
}

static int vidioc_enum_fmt_cap(struct file *file, void  *priv, struct v4l2_fmtdesc *f)
{
    SCAM_MSG("(%s) %s called, index=%d\n", current->comm, __FUNCTION__, f->index);
    if(f->index >= SMARTCAM_NFORMATS)
        return -EINVAL;
    strlcpy(f->description, fmtdesc[f->index], sizeof(f->description));
    f->pixelformat = formats[f->index].pixelformat;

    return 0;
}

static int vidioc_g_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    f->fmt.pix = formats[format];

    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
    return 0;
}

static int vidioc_try_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    int i;

    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);

    for (i = 0; i < SMARTCAM_NFORMATS; i++) {
        if (f->fmt.pix.pixelformat == formats[i].pixelformat) {
            f->fmt.pix = formats[format];
            return 0;
        }
    }

    return -EINVAL;
}

static int vidioc_s_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    int i;

    SCAM_MSG("%s called\n", __FUNCTION__);

    for (i = 0; i < SMARTCAM_NFORMATS; i++) {
        if ((f->fmt.pix.width == formats[i].width) &&
            (f->fmt.pix.height == formats[i].height) &&
            (f->fmt.pix.pixelformat == formats[i].pixelformat)) {
            format = i;
            f->fmt.pix = formats[format];
            return 0;
        }
    }

        SCAM_MSG("smartcam (%s): vidioc_s_fmt_cap called\n\t\twidth=%d; height=%d; \
        pixelformat=%s\n\t\tfield=%d; bytesperline=%d; sizeimage=%d; colspace = %d; return EINVAL\n",
        f->fmt.pix.width, f->fmt.pix.height, (char*)&f->fmt.pix.pixelformat,
        f->fmt.pix.field, f->fmt.pix.bytesperline, f->fmt.pix.sizeimage, f->fmt.pix.colorspace);

    return -EINVAL;
}

/************************ STREAMING IO / MMAP ***************************/

static int smartcam_mmap(struct file *file, struct vm_area_struct *vma)
{
        int ret;
        long length = vma->vm_end - vma->vm_start;
        unsigned long start = vma->vm_start;
        char *vmalloc_area_ptr = frame_data;
        unsigned long pfn;

    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);

        if (length > SMARTCAM_BUFFER_SIZE)
                return -EIO;

        /* loop over all pages, map each page individually */
        while (length > 0)
    {
                pfn = vmalloc_to_pfn (vmalloc_area_ptr);
                ret = remap_pfn_range(vma, start, pfn, PAGE_SIZE, PAGE_SHARED);
        if(ret < 0)
        {
                        return ret;
                }
                start += PAGE_SIZE;
                vmalloc_area_ptr += PAGE_SIZE;
                length -= PAGE_SIZE;
        }

        return 0;
}

static int vidioc_reqbufs(struct file *file, void *priv, struct v4l2_requestbuffers *reqbuf)
{
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);

    if(reqbuf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
                SCAM_MSG("Bad memory type for v4l2_requestbuffers\n");
        return -EINVAL;
    }
    if(reqbuf->memory != V4L2_MEMORY_MMAP)
    {
        return -EINVAL;
    }
    if(reqbuf->count < 1)
        reqbuf->count = 1;
    if(reqbuf->count > MAX_STREAMING_BUFFERS)
        reqbuf->count = MAX_STREAMING_BUFFERS;
    return 0;
}

static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *vidbuf)
{
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);

    if(vidbuf->index < 0 || vidbuf->index >= MAX_STREAMING_BUFFERS)
    {
        SCAM_MSG("vidioc_querybuf called - invalid buf index\n");
        return -EINVAL;
    }
    if(vidbuf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        SCAM_MSG("vidioc_querybuf called - invalid buf type\n");
        return -EINVAL;
    }
        vidbuf->memory = V4L2_MEMORY_MMAP;

    vidbuf->memory = V4L2_MEMORY_MMAP;
    vidbuf->length = SMARTCAM_BUFFER_SIZE;
    vidbuf->bytesused = formats[format].sizeimage;
    vidbuf->flags = V4L2_BUF_FLAG_MAPPED;
    vidbuf->m.offset = 2 * vidbuf->index * vidbuf->length;
    vidbuf->reserved = 0;
    return 0;
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *vidbuf)
{
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);

    if(vidbuf->index < 0 || vidbuf->index >= MAX_STREAMING_BUFFERS)
    {
        return -EINVAL;
    }
    if(vidbuf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        return -EINVAL;
    }
    if(vidbuf->memory != V4L2_MEMORY_MMAP)
    {
        return -EINVAL;
    }
    vidbuf->length = SMARTCAM_BUFFER_SIZE;
    vidbuf->bytesused = formats[format].sizeimage;
    vidbuf->flags = V4L2_BUF_FLAG_MAPPED;
    return 0;
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *vidbuf)
{
    if(file->f_flags & O_NONBLOCK)
        SCAM_MSG("(%s) %s called (non-blocking)\n", current->comm, __FUNCTION__);
    else
        SCAM_MSG("(%s) %s called (blocking)\n", current->comm, __FUNCTION__);

    if(vidbuf->index < 0 || vidbuf->index >= MAX_STREAMING_BUFFERS)
    {
        return -EINVAL;
    }
    if(vidbuf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        return -EINVAL;
    }
    if(vidbuf->memory != V4L2_MEMORY_MMAP)
    {
        return -EINVAL;
    }

    if(!(file->f_flags & O_NONBLOCK))
     // interruptible_sleep_on_timeout(&wq, HZ); /* wait max 1 second */
        msleep_interruptible(1000);

    vidbuf->length = SMARTCAM_BUFFER_SIZE;
    vidbuf->bytesused = formats[format].sizeimage;
    vidbuf->flags = V4L2_BUF_FLAG_MAPPED;
    vidbuf->timestamp = frame_timestamp;
    vidbuf->sequence = frame_sequence;
    last_read_frame = frame_sequence;
    return 0;
}

static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
    return 0;
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
    return 0;
}

static int vidioc_s_std (struct file *file, void *priv, v4l2_std_id i)
{
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
    return 0;
}

static int vidioc_g_std (struct file *file, void *priv, v4l2_std_id *i)
{
        SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
        *i = V4L2_STD_NTSC_M;
        return 0;
}

/* only one input */
static int vidioc_enum_input(struct file *file, void *priv, struct v4l2_input *inp)
{
    if(inp->index != 0)
    {
        SCAM_MSG("(%s) %s called - return EINVAL\n", current->comm, __FUNCTION__);
        return -EINVAL;
    }
    else
    {
        SCAM_MSG("(%s) %s called - return 0\n", current->comm, __FUNCTION__);
    }
    inp->type = V4L2_INPUT_TYPE_CAMERA;
    inp->std = V4L2_STD_NTSC_M;
    strcpy(inp->name, "smartcam input");

    return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
    *i = 0;
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
    return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
    SCAM_MSG("(%s) %s called, input = %d\n", current->comm, __FUNCTION__, i);
    if(i > 0)
        return -EINVAL;

    return 0;
}

/* --- controls ---------------------------------------------- */
static int vidioc_queryctrl(struct file *file, void *priv, struct v4l2_queryctrl *qc)
{
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
    return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl)
{
    SCAM_MSG("(%s) %s called - return EINVAL\n", current->comm, __FUNCTION__);
    return -EINVAL;
}

static int vidioc_s_ctrl(struct file *file, void *priv,	struct v4l2_control *ctrl)
{
    SCAM_MSG("(%s) %s called - return EINVAL\n", current->comm, __FUNCTION__);
    return -EINVAL;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
static int vidioc_selection(struct file *file, void *priv, struct v4l2_selection *selection)
{
	if (selection->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (selection->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		selection->r.top = 0;
		selection->r.left = 0;
		selection->r.width = SMARTCAM_FRAME_WIDTH;
		selection->r.height = SMARTCAM_FRAME_HEIGHT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#else
static int vidioc_cropcap(struct file *file, void *priv, struct v4l2_cropcap *cropcap)
{
    struct v4l2_rect defrect;

    SCAM_MSG("(%s) %s called - return 0\n", current->comm, __FUNCTION__);

    defrect.left = defrect.top = 0;
    defrect.width = SMARTCAM_FRAME_WIDTH;
    defrect.height = SMARTCAM_FRAME_HEIGHT;

    cropcap->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cropcap->bounds = cropcap->defrect = defrect;
    return 0;

}

static int vidioc_g_crop(struct file *file, void *priv, struct v4l2_crop *crop)
{
    SCAM_MSG("%s called - return EINVAL\n", __FUNCTION__);
    return -EINVAL;
}

static int vidioc_s_crop(struct file *file, void *priv, const struct v4l2_crop *crop)
{
    SCAM_MSG("(%s) %s called - return EINVAL\n", current->comm, __FUNCTION__);
    //return -EINVAL;
    return 0;
}
#endif

static int vidioc_g_parm(struct file *file, void *priv, struct v4l2_streamparm *streamparm)
{
    SCAM_MSG("(%s) %s called - return 0\n", current->comm, __FUNCTION__);

    memset(streamparm, 0, sizeof(*streamparm));
    streamparm->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    streamparm->parm.capture.capturemode = 0;
    streamparm->parm.capture.timeperframe.numerator = 1;
    streamparm->parm.capture.timeperframe.denominator = 10;
    streamparm->parm.capture.extendedmode = 0;
    streamparm->parm.capture.readbuffers = 3;

    return 0;
}

static int vidioc_s_parm(struct file *file, void *priv, struct v4l2_streamparm *streamparm)
{
    if(streamparm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        SCAM_MSG("(%s) %s called; numerator=%d, denominator=%d - return EINVAL\n", current->comm, __FUNCTION__,
             streamparm->parm.capture.timeperframe.numerator, streamparm->parm.capture.timeperframe.denominator);
        return -EINVAL;
    }
    SCAM_MSG("(%s) %s called; numerator=%d, denominator=%d, readbuffers=%d - return 0\n", current->comm, __FUNCTION__,
         streamparm->parm.capture.timeperframe.numerator,
         streamparm->parm.capture.timeperframe.denominator,
         streamparm->parm.capture.readbuffers);

    return 0;
}

/* ------------------------------------------------------------------
    File operations for the device
   ------------------------------------------------------------------*/

static int smartcam_open(struct file *file)
{
        SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
    return 0;
}

static ssize_t smartcam_read(struct file *file, char __user *data, size_t count, loff_t *f_pos)
{
        SCAM_MSG("(%s) %s called (count=%d, f_pos = %d)\n", current->comm, __FUNCTION__, (int) count, (int) *f_pos);

    if(*f_pos >= formats[format].sizeimage)
        return 0;

    if (!(file->f_flags & O_NONBLOCK))
     // interruptible_sleep_on_timeout(&wq, HZ/10); /* wait max 1 second */
        msleep_interruptible(100);
    last_read_frame = frame_sequence;

    if(*f_pos + count > formats[format].sizeimage)
        count = formats[format].sizeimage - *f_pos;
    if(copy_to_user(data, frame_data + *f_pos, count))
    {
        return -EFAULT;
    }
    return 0;
}

static int Clamp (int x)
{
        int r = x;      /* round to nearest */

        if (r < 0)         return 0;
        else if (r > 255)  return 255;
        else               return r;
}

static void rgb_to_yuyv(void)
{
    unsigned char *rp = frame_data, *wp = frame_data;
    for (; rp < (unsigned char *)(frame_data + SMARTCAM_RGB_FRAME_SIZE);
            rp += 6, wp += 4) {
        unsigned char r1 = rp[0], g1 = rp[1], b1 = rp[2];
        unsigned char r2 = rp[3], g2 = rp[4], b2 = rp[5];

        unsigned char y1 = Clamp((299 * r1 + 587 * g1 + 114 * b1) / 1000);
        unsigned char u1 = Clamp((-169 * r1 - 331 * g1 + 500 * b1) / 1000 + 128);
        unsigned char v1 = Clamp((500 * r1 - 419 * g1 - 81 * b1) / 1000 + 128);
        unsigned char y2 = Clamp((299 * r2 + 587 * g2 + 114 * b2) / 1000);
        /*unsigned char u2 = Clamp((-169 * r2 - 331 * g2 + 500 * b2) / 1000 + 128);
        unsigned char v2 = Clamp((500 * r2 - 419 * g2 - 81 * b2) / 1000 + 128);*/

        wp[0] = y1;
        wp[1] = u1;
        wp[2] = y2;
        wp[3] = v1;
    }
}

static ssize_t smartcam_write(struct file *file, const char __user *data, size_t count, loff_t *f_pos)
{
    SCAM_MSG("(%s) %s called (count=%d, f_pos = %d)\n", current->comm, __FUNCTION__, (int) count, (int) *f_pos);

    if (count >= SMARTCAM_RGB_FRAME_SIZE)
        count = SMARTCAM_RGB_FRAME_SIZE;

    if(copy_from_user(frame_data, data, count))
    {
        return -EFAULT;
    }
    ++ frame_sequence;

    if (formats[format].pixelformat == V4L2_PIX_FMT_YUYV)
        rgb_to_yuyv();

    v4l2l_get_timestamp(&frame_timestamp);
    wake_up_interruptible_all(&wq);
    return count;
}

static unsigned int smartcam_poll(struct file *file, struct poll_table_struct *wait)
{
    int mask = (POLLOUT | POLLWRNORM);	/* writable */
    if (last_read_frame != frame_sequence)
        mask |= (POLLIN | POLLRDNORM);	/* readable */

    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);

    poll_wait(file, &wq, wait);

    return mask;
}

static int smartcam_release(struct file *file)
{
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
    return 0;
}

static const struct v4l2_file_operations smartcam_fops = {
    .owner          = THIS_MODULE,
    .open           = smartcam_open,
    .release        = smartcam_release,
    .read           = smartcam_read,
    .write          = smartcam_write,
    .poll           = smartcam_poll,
    .unlocked_ioctl = video_ioctl2, /* V4L2 ioctl handler */
    .mmap           = smartcam_mmap,
};

static const struct v4l2_ioctl_ops smartcam_ioctl_ops = {
    .vidioc_querycap      = vidioc_querycap,
    .vidioc_enum_fmt_vid_cap  = vidioc_enum_fmt_cap,
    .vidioc_g_fmt_vid_cap     = vidioc_g_fmt_cap,
    .vidioc_try_fmt_vid_cap   = vidioc_try_fmt_cap,
    .vidioc_s_fmt_vid_cap     = vidioc_s_fmt_cap,
    .vidioc_reqbufs       = vidioc_reqbufs,
    .vidioc_querybuf      = vidioc_querybuf,
    .vidioc_qbuf          = vidioc_qbuf,
    .vidioc_dqbuf         = vidioc_dqbuf,
    .vidioc_s_std         = vidioc_s_std,
    .vidioc_g_std	      = vidioc_g_std,
    .vidioc_enum_input    = vidioc_enum_input,
    .vidioc_g_input       = vidioc_g_input,
    .vidioc_s_input       = vidioc_s_input,
    .vidioc_queryctrl     = vidioc_queryctrl,
    .vidioc_g_ctrl        = vidioc_g_ctrl,
    .vidioc_s_ctrl        = vidioc_s_ctrl,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
    .vidioc_g_selection   = vidioc_selection,
#else
    .vidioc_cropcap	      = vidioc_cropcap,
    .vidioc_g_crop	      = vidioc_g_crop,
    .vidioc_s_crop	      = vidioc_s_crop,
#endif
    .vidioc_g_parm	      = vidioc_g_parm,
    .vidioc_s_parm	      = vidioc_s_parm,
    .vidioc_streamon      = vidioc_streamon,
    .vidioc_streamoff     = vidioc_streamoff,
};

static struct video_device smartcam_vid = {
    .name		= "smartcam",
    .vfl_type               = VFL_TYPE_GRABBER,
    .fops           = &smartcam_fops,
    .minor		= -1,
    .release	= video_device_release_empty,
    .tvnorms        = V4L2_STD_NTSC_M,
    .ioctl_ops	= &smartcam_ioctl_ops,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
    .device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING,
#endif
};

/* -----------------------------------------------------------------
    Initialization and module stuff
   ------------------------------------------------------------------*/

static int __init smartcam_init(void)
{
    struct smartcam_dev *dev;
    struct video_device *vfd;
    int ret = 0;
    
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;
    
    mutex_init(&dev->mutex);
    
    frame_data =  (char*) vmalloc(SMARTCAM_BUFFER_SIZE);
    if(!frame_data)
    {
        return -ENOMEM;
    }
    frame_sequence = last_read_frame = 0;
//  ret = video_register_device(&smartcam_vid, VFL_TYPE_GRABBER, -1);
    snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name),
            "%s", SMARTCAM_MODULE_NAME);
    ret = v4l2_device_register(NULL, &dev->v4l2_dev);
    if (ret)
        goto free_dev;
    vfd = &dev->vdev;
    *vfd = smartcam_vid;
    vfd->v4l2_dev = &dev->v4l2_dev;
    vfd->lock = &dev->mutex;
    video_set_drvdata(vfd, dev);
    ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);

    if (ret < 0)
        goto unreg_dev;
    SCAM_MSG("(%s) load status: %d\n", current->comm, ret);
    return 0;
unreg_dev:
    v4l2_device_unregister(&dev->v4l2_dev);
free_dev:
    kfree(dev);
    return ret;
}

static void __exit smartcam_exit(void)
{
    struct smartcam_dev *dev;
    SCAM_MSG("(%s) %s called\n", current->comm, __FUNCTION__);
    frame_sequence = 0;
    vfree(frame_data);
    video_unregister_device(&smartcam_vid);
    v4l2_device_unregister(&dev->v4l2_dev);
    kfree(dev);
}

module_init(smartcam_init);
module_exit(smartcam_exit);

MODULE_DESCRIPTION("Smartphone Webcam");
MODULE_AUTHOR("Ionut Dediu");
MODULE_LICENSE("Dual BSD/GPL");
