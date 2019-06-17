/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   common.h
 * Author: Nabil Dehaini
 *
 * Created on March 22, 2019, 5:06 PM
 * contains all the necessary variables that are shared between the capture object 
 * and the mux object.h
 * There is just a header and there is no common.cpp
 */

#ifndef COMMON_H
#define COMMON_H

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>  // cv::Canny()
#include"opencv2/opencv.hpp"
#include <thread>
#include<condition_variable>
#include <alsa/asoundlib.h>
#ifdef __cplusplus
extern "C" {
#endif
    
#include <iostream>
#include<time.h>
#include <SDL2/SDL.h>
#include<chrono>

#include<sys/socket.h>
#include <netinet/in.h>
#include<poll.h>  
    
//these headers need to be inside an extern "C" caption    
/*              extern "C" headers              */
#include <libavutil/common.h>
#include <libavutil/frame.h>
#include <libavutil/samplefmt.h>
#include <libavutil/pixdesc.h>
    
#include <libavformat/avformat.h>
#include <libavutil/timestamp.h>
#include <libavutil/avassert.h>
#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>    
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>    
#include<libswresample/swresample.h>
/*              extern "C" headers end          */
    
#define STREAM_PIX_FMT    AV_PIX_FMT_YUV420P /* default pix_fmt */
#define SCALE_FLAGS SWS_BICUBIC

#define SAMPLE_FREQUENCY 44100
#define BIT_RATE 60000    

#define MAX_SECONDS 1

#define USEC_UNIT 1000000
    
#define AV_LOG_FILE_NAME "av_mux_log"
#define AV_LOG_EXT ".txt"
#define AV_LOG_FILE AV_LOG_FILE_NAME AV_LOG_EXT    
    
#undef av_err2str
#define av_err2str(errnum) av_make_error_string((char*)__builtin_alloca(AV_ERROR_MAX_STRING_SIZE),AV_ERROR_MAX_STRING_SIZE, errnum)
#undef av_ts2str
#define av_ts2str(ts) av_ts_make_string((char*)__builtin_alloca(AV_TS_MAX_STRING_SIZE), ts)
#undef av_ts2timestr
#define av_ts2timestr(ts, tb) av_ts_make_time_string((char*)__builtin_alloca(AV_TS_MAX_STRING_SIZE), ts, tb)
    
    
using namespace std;
using namespace cv;

//#define FRAME_RATE 20
//#define TOT_BUFF_SIZE FRAME_RATE*2
#define DEFAULT_FILE_NAME "/tmp/media/mux_output.mp4"

struct audio_stream {
    uint8_t *buff;
    size_t buffsize;//this is the total bufsize created.  It is 2* buffsize_max.  We regularly have to read data greater then buffsize.  that when we stop
    size_t buffsize_max;//approximately the total buff size that we read. This value is less than buff_size
    size_t write_idx;//The amount we actually write every sample iteration
    size_t read_idx;//The amount that we actually read every sample iteration 
    size_t buff_tot;//total data that is actually ready before we pass the structure to th muxer.  we make sure that we only read data to this amount. 
    mutex mtx;
    condition_variable cv;
};

struct video_stream {
    int buffsize;
    int buffsize_max;
    Mat *frames;
    int read_idx;
    int write_idx;
    int frames_to_write;
    long int frames_captured;
    mutex mtx;
    condition_variable cv;
    vector<uint8_t> *imgbuf;
};

//muxing struct
typedef struct OutputStream {
    AVStream *st;
    AVCodecContext *enc;
    /* pts of the next frame that will be generated */
    int64_t next_pts;
    int samples_count;
    AVFrame *frame;
    AVFrame *tmp_frame;
    struct SwsContext *sws_ctx;
    struct SwrContext *swr_ctx;
} OutputStream;


//using myclock = std::chrono::steady_clock;
using myclock = std::chrono::high_resolution_clock;


inline static double time_duration(myclock::time_point first_tp, myclock::time_point last_tp) {
    myclock::duration d = last_tp - first_tp;
    double seconds = (double) d.count() * myclock::period::num / myclock::period::den;
    return seconds;
}

static double get_time(myclock::time_point &current_tp)
{
    myclock::time_point tp = myclock::now();
    double dur = time_duration(current_tp, tp);
    current_tp = tp;
    return dur;    
}

class av_common {
public:
    av_common(){}
    int frame_width; // = 640;
    int frame_height; // = 480;
    int frame_rate;
    int video_dev_id;
    int audio_dev_id;
    bool audio_ready, video_ready;
    bool mux_audio, mux_video;
    string filename;
    audio_stream audio1, audio2;
    video_stream video1, video2;
    double tot_vid_count, tot_audio_count;
    double vid_time, aud_time;
    
    myclock::time_point audio_time_point;
    double audio_duration;

    
private:
}; 
 
#ifdef __cplusplus
}
#endif




#endif /* COMMON_H */

