/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ffmpeg_transcode.h
 * Author: Nabil Dehaini
 *
 * Created on February 18, 2019, 2:44 PM
 * This here is where we mux audio and video.  Each channel is acquired separately 
 * in cam_capture and a thread is created to MUX the channels here.  The code is
 * acquired from the transcode example in ffmpeg examples and customised to work
 * with open_cv video capture and SDL2 audio capture.  
 */

#ifndef FFMPEG_TRANSCODE_H
#define FFMPEG_TRANSCODE_H

#include "common.h"
#include "logger.h"

using namespace std;

class ffmpeg_mux {
public:
    ffmpeg_mux(){};
    int init();
    int finish();
    virtual ~ffmpeg_mux(){};
    
    av_common *common;
    int mux();
    int process_mux();
    bool keepcapturing;
    video_stream *video_reader1;
    audio_stream *audio_reader1;
    uint8_t *sound_buff;
    bool mux_audio, mux_video;
private:

    int reset_pts;
    uint8_t *frame_buf;
    size_t buffsize;
    OutputStream video_st, audio_st;
    AVOutputFormat *fmt;
    AVFormatContext *oc;
    AVCodec *audio_codec, *video_codec;
    int have_video, have_audio, encode_video, encode_audio;
    AVDictionary *opt;
    void wait_signal();
    int allocate_buffer(OutputStream *ost);
    int deallocate_buffer();
    void close_stream(AVFormatContext *oc, OutputStream *ost);
    int write_video_frame_cv(AVFormatContext *oc, OutputStream *ost, cv::Mat *opencv_frame);    
    AVFrame *get_video_frame_cv(OutputStream *ost, cv::Mat *opencv_frame);
    AVFrame *alloc_picture(enum AVPixelFormat pix_fmt, int width, int height);
    int write_audio_frame_ex(AVFormatContext *oc, OutputStream *ost, AVFrame *avframe);
    void open_video(AVFormatContext *oc, AVCodec *codec, OutputStream *ost, AVDictionary *opt_arg);
    AVFrame *get_audio_frame_ex(OutputStream *ost, audio_stream *audio);
    void open_audio(AVFormatContext *oc, AVCodec *codec, OutputStream *ost, AVDictionary *opt_arg);
    AVFrame *alloc_audio_frame(enum AVSampleFormat sample_fmt,
            uint64_t channel_layout,
            int sample_rate, int nb_samples);
    void add_stream(OutputStream *ost, AVFormatContext *oc,
            AVCodec **codec,
            enum AVCodecID codec_id);
    int write_frame(AVFormatContext *fmt_ctx, const AVRational *time_base, AVStream *st, AVPacket *pkt);
    void do_mux(audio_stream *audio_reader, video_stream *video_reader);    
/////////////////This is the mother of all things here.////////////////////////      
    int mux_ex(audio_stream *audio_reader, video_stream *video_reader);
};

#endif /* FFMPEG_TRANSCODE_H */

