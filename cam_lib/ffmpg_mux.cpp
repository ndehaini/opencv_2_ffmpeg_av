/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ffmpeg_mux.cpp
 * Author: Nabil Dehaini
 * This file is taken straight out of transcode example and customised for the purpose of real time muxing .  
 * 
 * 
 */



#include <iostream>
#include <condition_variable>

#include "ffmpg_mux.h"
#include "logger.h"
#include "cam_exception.h"



static void log_packet(const AVFormatContext *fmt_ctx, const AVPacket *pkt)
{
    AVRational *time_base = &fmt_ctx->streams[pkt->stream_index]->time_base;
    printf("pts:%s pts_time:%s dts:%s dts_time:%s duration:%s duration_time:%s stream_index:%d\n",    
           av_ts2str(pkt->pts), av_ts2timestr(pkt->pts, time_base),
           av_ts2str(pkt->dts), av_ts2timestr(pkt->dts, time_base),
           av_ts2str(pkt->duration), av_ts2timestr(pkt->duration, time_base),
           pkt->stream_index);
}
int ffmpeg_mux::write_frame(AVFormatContext *fmt_ctx, const AVRational *time_base, AVStream *st, AVPacket *pkt)
{
    /* rescale output packet timestamp values from codec to stream timebase */
    av_packet_rescale_ts(pkt, *time_base, st->time_base);
    pkt->stream_index = st->index;
    /* Write the compressed frame to the media file. */
//    log_packet(fmt_ctx, pkt);
    return av_interleaved_write_frame(fmt_ctx, pkt);
}

/* Add an output stream. */
void ffmpeg_mux::add_stream(OutputStream *ost, AVFormatContext *oc,
                       AVCodec **codec,
                       enum AVCodecID codec_id)
{
    AVCodecContext *c;
    int i;
    /* find the encoder */
    *codec = avcodec_find_encoder(codec_id);
    if (!(*codec)) {
        throw cam_exception(string("Could not find encoder for ")+ avcodec_get_name(codec_id));
//        fprintf(stderr, "Could not find encoder for '%s'\n",
//                avcodec_get_name(codec_id));
//        exit(1);
    }
    ost->st = avformat_new_stream(oc, NULL);
    if (!ost->st) {
        throw cam_exception("Could not allocate stream");
//        fprintf(stderr, "Could not allocate stream\n");
//        exit(1);
    }
    ost->st->id = oc->nb_streams-1;
    c = avcodec_alloc_context3(*codec);
    if (!c) {
        throw cam_exception("Could not alloc an encoding context");
//        fprintf(stderr, "Could not alloc an encoding context\n");
//        exit(1);
    }
    ost->enc = c;
    switch ((*codec)->type) {
    case AVMEDIA_TYPE_AUDIO:
//        c->sample_fmt = AV_SAMPLE_FMT_FLTP;//AV_SAMPLE_FMT_FLTP;//AV_SAMPLE_FMT_S32P;
        c->sample_fmt  = (*codec)->sample_fmts ?
            (*codec)->sample_fmts[0] : AV_SAMPLE_FMT_FLTP;
        c->bit_rate    = 64000;
        c->sample_rate = 44100;
        if ((*codec)->supported_samplerates) {
            c->sample_rate = (*codec)->supported_samplerates[0];
            for (i = 0; (*codec)->supported_samplerates[i]; i++) {
                if ((*codec)->supported_samplerates[i] == 44100)
                    c->sample_rate = 44100;
            }
        }
        c->channel_layout = AV_CH_LAYOUT_MONO;//AV_CH_LAYOUT_MONO;//AV_CH_LAYOUT_STEREO;
        if ((*codec)->channel_layouts) {
            c->channel_layout = (*codec)->channel_layouts[0];
            for (i = 0; (*codec)->channel_layouts[i]; i++) {
                if ((*codec)->channel_layouts[i] == AV_CH_LAYOUT_STEREO)
                    c->channel_layout = AV_CH_LAYOUT_STEREO;
            }
        }
        c->channels        = av_get_channel_layout_nb_channels(c->channel_layout);

        ost->st->time_base = (AVRational){ 1, c->sample_rate};
        //ost->st->time_base = (AVRational){ 1, 176400};
        c->time_base = ost->st->time_base;//I just added this to conform with what is being set for the video
        break;
    case AVMEDIA_TYPE_VIDEO:
        c->codec_id = codec_id;
        c->bit_rate = 400000;
        /* Resolution must be a multiple of two. */
        c->width    = common->frame_width;
        c->height   = common->frame_height;
        /* timebase: This is the fundamental unit of time (in seconds) in terms
         * of which frame timestamps are represented. For fixed-fps content,
         * timebase should be 1/framerate and timestamp increments should be
         * identical to 1. */
        ost->st->time_base = (AVRational){ 1, common->frame_rate };
        c->time_base       = ost->st->time_base;
        c->gop_size      = 12; /* emit one intra frame every twelve frames at most */
        c->pix_fmt       = STREAM_PIX_FMT;
        if (c->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
            /* just for testing, we also add B-frames */
            c->max_b_frames = 2;
        }
        if (c->codec_id == AV_CODEC_ID_MPEG1VIDEO) {
            /* Needed to avoid using macroblocks in which some coeffs overflow.
             * This does not happen with normal video, it just happens here as
             * the motion of the chroma plane does not match the luma plane. */
            c->mb_decision = 2;
        }
    break;
    default:
        break;
    }
    /* Some formats want stream headers to be separate. */
    if (oc->oformat->flags & AVFMT_GLOBALHEADER)
        c->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
}
/**************************************************************/
/* audio output */
AVFrame *ffmpeg_mux::alloc_audio_frame(enum AVSampleFormat sample_fmt,
                                  uint64_t channel_layout,
                                  int sample_rate, int nb_samples)
{
    AVFrame *frame = av_frame_alloc();
    int ret;
    if (!frame) {
        throw cam_exception("Error allocating an audio frame");
//        fprintf(stderr, "Error allocating an audio frame\n");
//        exit(1);
    }
    frame->format = sample_fmt;
    frame->channel_layout = channel_layout;
    frame->sample_rate = sample_rate;
    frame->nb_samples = nb_samples;
    if (nb_samples) {
        ret = av_frame_get_buffer(frame, 0);
        if (ret < 0) {
        throw cam_exception("Error allocating an audio buffer");
//            fprintf(stderr, "Error allocating an audio buffer\n");
//            exit(1);
        }
    }
    return frame;
}

void ffmpeg_mux::open_audio(AVFormatContext *oc, AVCodec *codec, OutputStream *ost, AVDictionary *opt_arg)
{
    AVCodecContext *c;
    int nb_samples;
    int ret;
    AVDictionary *opt = NULL;
    c = ost->enc;
    /* open it */
    av_dict_copy(&opt, opt_arg, 0);
    ret = avcodec_open2(c, codec, &opt);
    av_dict_free(&opt);
    if (ret < 0) {
        throw cam_exception(string("Could not open audio codec: ")+av_err2str(ret));
//        fprintf(stderr, "Could not open audio codec: %s\n", av_err2str(ret));
//        exit(1);
    }

    if (c->codec->capabilities & AV_CODEC_CAP_VARIABLE_FRAME_SIZE)
        nb_samples = 10000;
    else
        nb_samples = c->frame_size;
    ost->frame     = alloc_audio_frame(c->sample_fmt, c->channel_layout,
                                       c->sample_rate, nb_samples);
    ost->tmp_frame = alloc_audio_frame(AV_SAMPLE_FMT_S32P, c->channel_layout,
                                       c->sample_rate, nb_samples);
//    ost->tmp_frame = alloc_audio_frame(AV_SAMPLE_FMT_S16, c->channel_layout,
//                                       c->sample_rate, nb_samples);
    /* copy the stream parameters to the muxer */
    ret = avcodec_parameters_from_context(ost->st->codecpar, c);
    if (ret < 0) {
        throw cam_exception("Could not copy the stream parameters");
//        fprintf(stderr, "Could not copy the stream parameters\n");
//        exit(1);
    }
    /* create resampler context */
        ost->swr_ctx = swr_alloc();
        if (!ost->swr_ctx) {
            throw cam_exception("Could not allocate resampler context\n");
//            fprintf(stderr, "Could not allocate resampler context\n");
//            exit(1);
        }
        /* set options */
        av_opt_set_int       (ost->swr_ctx, "in_channel_count",   c->channels,       0);
        av_opt_set_int       (ost->swr_ctx, "in_sample_rate",     c->sample_rate,    0);
        av_opt_set_sample_fmt(ost->swr_ctx, "in_sample_fmt",      AV_SAMPLE_FMT_S32P, 0);
//        av_opt_set_sample_fmt(ost->swr_ctx, "in_sample_fmt",      AV_SAMPLE_FMT_S16, 0);
        av_opt_set_int       (ost->swr_ctx, "out_channel_count",  c->channels,       0);
        av_opt_set_int       (ost->swr_ctx, "out_sample_rate",    c->sample_rate,    0);
        av_opt_set_sample_fmt(ost->swr_ctx, "out_sample_fmt",     c->sample_fmt,     0);
        /* initialize the resampling context */
        if ((ret = swr_init(ost->swr_ctx)) < 0) {
            throw cam_exception("Failed to initialize the resampling context");
//            fprintf(stderr, "Failed to initialize the resampling context\n");
//            exit(1);
        }
}

AVFrame* ffmpeg_mux::get_audio_frame_ex(OutputStream* ost, audio_stream *audio) 
{

    AVFrame *frame = ost->tmp_frame;
    //either commit to reading the full buffer or whatever remains
    size_t bytes_to_write = ((audio->read_idx+buffsize)<=audio->buff_tot)?buffsize:(audio->buff_tot-audio->read_idx);
    //may need to zero the buffer here
    memcpy(frame_buf, &audio->buff[audio->read_idx], bytes_to_write);
//    memcpy(frame_buf, &audio->read_ptr[audio->read_idx], buffsize);
    audio->read_idx+=bytes_to_write;
    frame->data[0] = frame_buf;
    frame->pts = ost->next_pts;
    ost->next_pts += frame->nb_samples;//buffsize;//frame->nb_samples*(bytes_to_write/buffsize);
    if(bytes_to_write!=buffsize)
        cout<<"bytes_to_write does not equal buffsize. bytes_to_write: "<<bytes_to_write<<" buffsize:"<<buffsize<<endl;
    return frame;
}


/*
 * encode one audio frame and send it to the muxer
 * return 1 when encoding is finished, 0 otherwise
 */

int ffmpeg_mux::write_audio_frame_ex(AVFormatContext* oc, OutputStream* ost, AVFrame *avframe)
{
    AVCodecContext *c;
    AVPacket pkt = { 0 }; // data and size must be 0;
    AVFrame *frame;
    int ret;
    int got_packet;
    int dst_nb_samples;
    av_init_packet(&pkt);
    c = ost->enc;
    frame = avframe;
    if (frame) {
        /* convert samples from native format to destination codec format, using the resampler */
            /* compute destination number of samples */
            dst_nb_samples = av_rescale_rnd(swr_get_delay(ost->swr_ctx, c->sample_rate) + frame->nb_samples,
                                            c->sample_rate, c->sample_rate, AV_ROUND_UP);
            av_assert0(dst_nb_samples == frame->nb_samples);
        /* when we pass a frame to the encoder, it may keep a reference to it
         * internally;
         * make sure we do not overwrite it here
         */
        ret = av_frame_make_writable(ost->frame);
        if (ret < 0)
            exit(1);
        /* convert to destination format */
        ret = swr_convert(ost->swr_ctx,
                          ost->frame->data, dst_nb_samples,
                          (const uint8_t **)frame->data, frame->nb_samples);
        if (ret < 0) {
            throw cam_exception("Error while converting");
//            fprintf(stderr, "Error while converting\n");
//            exit(1);
        }
        frame = ost->frame;
        frame->pts = av_rescale_q(ost->samples_count, (AVRational){1, c->sample_rate}, c->time_base);
        ost->samples_count += dst_nb_samples;
    }
    ret = avcodec_encode_audio2(c, &pkt, frame, &got_packet);
    if (ret < 0) {
        throw cam_exception(string("Error encoding audio frame:")+ av_err2str(ret));
//        fprintf(stderr, "Error encoding audio frame: %s\n", av_err2str(ret));
//        exit(1);
    }
    if (got_packet) {
        ret = write_frame(oc, &c->time_base, ost->st, &pkt);
        if (ret < 0) {
            throw cam_exception(string("Error while writing audio frame: ")+av_err2str(ret));
//            fprintf(stderr, "Error while writing audio frame: %s\n",
//                    av_err2str(ret));
//            exit(1);
        }
    }
    av_free_packet(&pkt);
    return (frame || got_packet) ? 0 : 1;    
}

/**************************************************************/
/* video output */
AVFrame *ffmpeg_mux::alloc_picture(enum AVPixelFormat pix_fmt, int width, int height)
{
    AVFrame *picture;
    int ret;
    picture = av_frame_alloc();
    if (!picture)
        return NULL;
    picture->format = pix_fmt;
    picture->width  = width;
    picture->height = height;
    /* allocate the buffers for the frame data */
    ret = av_frame_get_buffer(picture, 32);
    if (ret < 0) {
        throw cam_exception("Could not allocate frame data");
//        fprintf(stderr, "Could not allocate frame data.\n");
//        exit(1);
    }
    return picture;
}
 void ffmpeg_mux::open_video(AVFormatContext *oc, AVCodec *codec, OutputStream *ost, AVDictionary *opt_arg)
{
    int ret;
    AVCodecContext *c = ost->enc;
    AVDictionary *opt = NULL;
    av_dict_copy(&opt, opt_arg, 0);
    /* open the codec */
    ret = avcodec_open2(c, codec, &opt);
    av_dict_free(&opt);
    if (ret < 0) {
        throw cam_exception(string("Could not open video codec")+av_err2str(ret));
//        fprintf(stderr, "Could not open video codec: %s\n", av_err2str(ret));
//        exit(1);
    }
    /* allocate and init a re-usable frame */
    ost->frame = alloc_picture(c->pix_fmt, c->width, c->height);
    if (!ost->frame) {
        throw cam_exception("Could not allocate video frame");
//        fprintf(stderr, "Could not allocate video frame\n");
//        exit(1);
    }
    /* If the output format is not YUV420P, then a temporary YUV420P
     * picture is needed too. It is then converted to the required
     * output format. */
    ost->tmp_frame = NULL;
    if (c->pix_fmt != AV_PIX_FMT_YUV420P) {
        ost->tmp_frame = alloc_picture(AV_PIX_FMT_YUV420P, c->width, c->height);
        if (!ost->tmp_frame) {
            throw cam_exception("Could not allocate temporary picture");
//            fprintf(stderr, "Could not allocate temporary picture\n");
//            exit(1);
        }
    }
    /* copy the stream parameters to the muxer */
    ret = avcodec_parameters_from_context(ost->st->codecpar, c);
    if (ret < 0) {
        throw cam_exception("Could not copy the stream parameters");
//        fprintf(stderr, "Could not copy the stream parameters\n");
//        exit(1);
    }
}

AVFrame *ffmpeg_mux::get_video_frame_cv(OutputStream* ost, cv::Mat* opencv_frame)
{
    AVCodecContext *c = ost->enc;

    /* when we pass a frame to the encoder, it may keep a reference to it
     * internally; make sure we do not overwrite it here */
    if (av_frame_make_writable(ost->frame) < 0)
        exit(1);
                      
 
//            nullptr, dst_width, dst_height, AV_PIX_FMT_BGR24,
//            dst_width, dst_height, vstrm->codec->pix_fmt, SWS_BICUBIC, nullptr, nullptr, nullptr);                     
  //  if (c->pix_fmt != AV_PIX_FMT_YUV420P) {
        /* as we only generate a YUV420P picture, we must convert it
         * to the codec pixel format if needed */
        if (!ost->sws_ctx) {
            ost->sws_ctx = sws_getContext(c->width, c->height,
                                          AV_PIX_FMT_BGR24,//AV_PIX_FMT_YUV420P,
                                          c->width, c->height,
                                          c->pix_fmt,
                                          SCALE_FLAGS, NULL, NULL, NULL);
            if (!ost->sws_ctx) {
                throw cam_exception("Could not initialize the conversion context");
//                fprintf(stderr,
//                        "Could not initialize the conversion context\n");
//                exit(1);
            }
        }
        //fill_yuv_image(ost->tmp_frame, ost->next_pts, c->width, c->height);
        
        //get image
        //cv::Mat img;
        const int stride[] = {static_cast<int> (opencv_frame->step[0])};
        sws_scale(ost->sws_ctx, &opencv_frame->data, stride, 0, opencv_frame->rows, ost->frame->data, ost->frame->linesize);

    ost->frame->pts = ost->next_pts;
    ost->next_pts++;
    return ost->frame;    
}

int ffmpeg_mux::write_video_frame_cv(AVFormatContext* oc, OutputStream* ost, cv::Mat* opencv_frame)
{
    int ret;
    AVCodecContext *c;
    AVFrame *frame;
    int got_packet = 0;
    AVPacket pkt = { 0 };
    c = ost->enc;
    frame = get_video_frame_cv(ost, opencv_frame);
    av_init_packet(&pkt);
    /* encode the image */
    ret = avcodec_encode_video2(c, &pkt, frame, &got_packet);
    if (ret < 0) {
        throw cam_exception(string("Error encoding video frame: ")+av_err2str(ret));
//        fprintf(stderr, "Error encoding video frame: %s\n", av_err2str(ret));
//        exit(1);
    }
    if (got_packet) {
        ret = write_frame(oc, &c->time_base, ost->st, &pkt);
    } else {
        ret = 0;
    }
    if (ret < 0) {
        throw cam_exception(string("Error while writing video frame: ")+av_err2str(ret));
//        fprintf(stderr, "Error while writing video frame: %s\n", av_err2str(ret));
//        exit(1);
    }
    av_free_packet(&pkt);
    return (frame || got_packet) ? 0 : 1;    
}

void ffmpeg_mux::close_stream(AVFormatContext *oc, OutputStream *ost)
{
    avcodec_free_context(&ost->enc);
    av_frame_free(&ost->frame);
    av_frame_free(&ost->tmp_frame);
    sws_freeContext(ost->sws_ctx);
    swr_free(&ost->swr_ctx);
}

/// calculate the required audio buffer size for the audio frame and assign it to the frame and 
/// tmp frame in the audio outputstream.  
/// \param ost
/// \return not used.
int ffmpeg_mux::allocate_buffer(OutputStream *ost)
{
    buffsize = av_samples_get_buffer_size(NULL,ost->enc->channels,ost->enc->frame_size,ost->enc->sample_fmt,1);
    LOG_EX(AV_LOG_FILE, string("MUX buffsize = "+std::to_string(buffsize)).c_str(), "INFO ");
    LOG_EX(AV_LOG_FILE, string("Encoder channels: "+std::to_string(ost->enc->channels)).c_str(), "INFO ");
    LOG_EX(AV_LOG_FILE, string("Encoder frame size: "+std::to_string(ost->enc->frame_size)).c_str(), "INFO ");
    LOG_EX(AV_LOG_FILE, string("frame.nb_samples: "+std::to_string(ost->frame->nb_samples)).c_str(), "INFO ");
    
    frame_buf = (uint8_t *)av_malloc(buffsize);
    avcodec_fill_audio_frame(ost->tmp_frame, ost->enc->channels, AV_SAMPLE_FMT_S16,(const uint8_t*)frame_buf, buffsize, 1);
    avcodec_fill_audio_frame(ost->frame, ost->enc->channels, ost->enc->sample_fmt,(const uint8_t*)frame_buf, buffsize, 1);
    
    return 0;
}

int ffmpeg_mux::deallocate_buffer(){
    av_free(frame_buf);    
    return 0;
}


int ffmpeg_mux::init()
{
    
    int ret;
    video_st = { 0 };
    audio_st = { 0 };
    have_video = 0; 
    have_audio = 0;
    encode_video = 0;
    encode_audio = 0;
    reset_pts = 0;
    opt = NULL;
    avformat_alloc_output_context2(&oc, NULL, NULL, common->filename.data());
    if (!oc) {
        LOG_EX(AV_LOG_FILE, "Could not deduce output format from file extension: using MPEG", "WARN ");
        avformat_alloc_output_context2(&oc, NULL, "mpeg", common->filename.data());
    }
    if (!oc)
        return 1;
    
    
    fmt = oc->oformat;
    /* Add the audio and video streams using the default format codecs
     * and initialize the codecs. */
    if (fmt->video_codec != AV_CODEC_ID_NONE) {
        add_stream(&video_st, oc, &video_codec, fmt->video_codec);
        have_video = 1;
        encode_video = 1;
    }
 //   fmt->audio_codec = AV_CODEC_ID_MP3;
    if (fmt->audio_codec != AV_CODEC_ID_NONE) {
        add_stream(&audio_st, oc, &audio_codec, fmt->audio_codec);
        have_audio = 1;
        encode_audio = 1;
    }
    /* Now that all the parameters are set, we can open the audio and
     * video codecs and allocate the necessary encode buffers. */
    if (have_video)
        open_video(oc, video_codec, &video_st, opt);
    if (have_audio)
        open_audio(oc, audio_codec, &audio_st, opt);
    av_dump_format(oc, 0, common->filename.data(), 1);
    /* open the output file, if needed */
    if (!(fmt->flags & AVFMT_NOFILE)) {
        ret = avio_open(&oc->pb, common->filename.data(), AVIO_FLAG_WRITE);
        if (ret < 0) {
            throw cam_exception("Could not open"+common->filename+" "+av_err2str(ret));
//            fprintf(stderr, "Could not open '%s': %s\n", common->filename.data(),
//                    av_err2str(ret));
//            return 1;
        }
    }
    /* Write the stream header, if any. */
    ret = avformat_write_header(oc, &opt);
    if (ret < 0) {
        throw cam_exception(string("Error occurred when opening output file")+av_err2str(ret));
//        fprintf(stderr, "Error occurred when opening output file: %s\n",
//                av_err2str(ret));
//        return 1;
    }
    
    allocate_buffer(&audio_st);    
    return 0;
}

int ffmpeg_mux::finish()
{
    av_write_trailer(oc);
    /* Close each codec. */
    if (have_video)
        close_stream(oc, &video_st);
    if (have_audio)
        close_stream(oc, &audio_st);
    if (!(fmt->flags & AVFMT_NOFILE))
        /* Close the output file. */
        avio_closep(&oc->pb);
    /* free the stream */
    avformat_free_context(oc);
    deallocate_buffer();
    return 0;        
}


void ffmpeg_mux::wait_signal()
{

    auto check_buffs = [this](){return (!common->mux_audio||common->audio_ready)&&(!common->mux_video||common->video_ready);};

    unique_lock<mutex> lk(video_reader1->mtx, adopt_lock);
    cout<<"MUX Waiting for signal from av threads"<<endl;
    video_reader1->cv.wait(lk, check_buffs);
    cout<<"MUX signalled by av threads to start MUXing"<<endl;
    lk.unlock();
}


//Not used any more.  mux_ex is the real function
int ffmpeg_mux::mux()
{
//    init();
    
    while (keepcapturing) {
        //wait for signal
        wait_signal();
        common->audio_ready = false;
        common->video_ready = false;
        //lock mutexes for that buffer
//        if (keepcapturing) {
            myclock::time_point start_tp = myclock::now();
            cout<<"MUX locking mutexes"<<endl;
            unique_lock<mutex> lck_a(audio_reader1->mtx);
            unique_lock<mutex> lck_v(video_reader1->mtx);

//            counter++;
//            if(counter==7){
//                counter=0;
//                audio_st.next_pts = video_st.next_pts;
//            }
            //we have to make this more sophisticated later.
            encode_audio = common->mux_audio;
            encode_video = common->mux_video;
            while (encode_video || encode_audio) {
                /* select the stream to encode */
                if (encode_video &&
                        (!encode_audio || av_compare_ts(video_st.next_pts, video_st.enc->time_base,
                        audio_st.next_pts, audio_st.enc->time_base) <= 0)) {
//                    cout<<"^^^^^^^^^^^^^^^video MUX^^^^^^^^^^^^^^^"<<endl;
                    encode_video= !write_video_frame_cv(oc, &video_st, &video_reader1->frames[video_reader1->read_idx]);
//                    cout<<"encoding video"<<endl;
                    video_reader1->read_idx++;
//                    if(vs->read_idx>=MAX_FRAMES)
                    if(video_reader1->read_idx>=video_reader1->frames_to_write)
                    {
                       // cout<<"MUX Video buffer spent"<<endl;
                        encode_video = false;
                    }
                } else {
                    
//                    cout<<"^^^^^^^^^^^^^^^MUX Audio^^^^^^^^^^^^^^^"<<endl;
                    AVFrame *f = get_audio_frame_ex(&audio_st, audio_reader1);
//                    encode_audio = !write_audio_frame(oc, &audio_st);
                    encode_audio = !write_audio_frame_ex(oc, &audio_st, f);
//                    cout<<"encoding audio"<<endl;

//                    if(as->read_idx>=as->buffsize_max)
                    if(audio_reader1->read_idx>=audio_reader1->buff_tot)
                    {
                        encode_audio = false;
               //         cout<<"MUX audio buffer spent. read_idx= "<<audio_reader->read_idx<<endl;
                    }
                }
            }
            //need to assert
            cout<<"MUX changing pointers and resetting"<<endl;
            lck_a.unlock();
            lck_v.unlock();

            video_reader1->read_idx = 0;            
            audio_reader1->read_idx = 0;            
            encode_video = 1;
            encode_audio = 1;
            double dur = time_duration(start_tp, myclock::now());
            cout<<"MUX duration = "<<dur<<endl;
            if(common->mux_video)
                video_reader1=(video_reader1==&common->video1)?&common->video2:&common->video1;
            if(common->mux_audio)
                audio_reader1 = (audio_reader1==&common->audio1)?&common->audio2:&common->audio1;
//        }
        //unlock mutexes and wait for signal
    }
    cout<<"MUX exited loop"<<endl;
    
    return 0;

//    finish();
    /* Write the trailer, if any. The trailer must be written before you
     * close the CodecContexts open when you wrote the header; otherwise
     * av_write_trailer() may try to use memory that was freed on
     * av_codec_close(). */
}


int ffmpeg_mux::process_mux(){
    thread t = std::thread(&ffmpeg_mux::do_mux,this, audio_reader1, video_reader1);
    t.detach();//try not detaching this when you get a chance.  "DEFINITELY DON'T JOIN"
    return 0;
}

void ffmpeg_mux::do_mux(audio_stream* audio_reader, video_stream* video_reader) {
    try{
    mux_ex(audio_reader, video_reader);
    }
    catch(cam_exception &e){
        LOG_EX(AV_LOG_FILE, e.what(), "ERROR");
        exit(1);
    }
}

int ffmpeg_mux::mux_ex(audio_stream *au_reader, video_stream *vid_reader) {
    
    common->audio_ready = false;
    common->video_ready = false;

    myclock::time_point start_tp = myclock::now();
    cout << "MUX locking mutexes" << endl;
    unique_lock<mutex> lck_a(au_reader->mtx);
    unique_lock<mutex> lck_v(vid_reader->mtx);

    encode_audio = common->mux_audio;
    encode_video = common->mux_video;
    reset_pts++;
    while (encode_video || encode_audio) {
        /* select the stream to encode */
        if (encode_video &&
                (!encode_audio || av_compare_ts(video_st.next_pts, video_st.enc->time_base,
                audio_st.next_pts, audio_st.enc->time_base) <= 0)) {
            //                    cout<<"^^^^^^^^^^^^^^^video MUX^^^^^^^^^^^^^^^"<<endl;
            encode_video = !write_video_frame_cv(oc, &video_st, &vid_reader->frames[vid_reader->read_idx]);
            //                    cout<<"encoding video"<<endl;
            vid_reader->read_idx++;
            //                    if(vs->read_idx>=MAX_FRAMES)
            if (vid_reader->read_idx >= vid_reader->frames_to_write) {
                // cout<<"MUX Video buffer spent"<<endl;
                encode_video = false;
            }
        } else {

            //                    cout<<"^^^^^^^^^^^^^^^MUX Audio^^^^^^^^^^^^^^^"<<endl;
            AVFrame *f = get_audio_frame_ex(&audio_st, au_reader);
            //                    encode_audio = !write_audio_frame(oc, &audio_st);
            encode_audio = !write_audio_frame_ex(oc, &audio_st, f);
            //                    cout<<"encoding audio"<<endl;

            //                    if(as->read_idx>=as->buffsize_max)
            if (au_reader->read_idx >= au_reader->buff_tot) {
                encode_audio = false;
                //         cout<<"MUX audio buffer spent. read_idx= "<<audio_reader->read_idx<<endl;
            }
        }
    }
    //need to assert
    cout << "MUX finished" << endl;
    common->tot_vid_count += vid_reader->frames_to_write;
    common->tot_audio_count += au_reader->buff_tot;
    lck_a.unlock();
    lck_v.unlock();

    vid_reader->read_idx = 0;
    au_reader->read_idx = 0;

    double dur = time_duration(start_tp, myclock::now());
    cout << "MUX duration = " << dur << endl;
    cout << "audio samples count: " << audio_st.samples_count << " audio stream next_pts " << audio_st.next_pts << endl;
    cout << "video samples count: " << video_st.samples_count << "  stream next_pts " << video_st.next_pts << endl;
    cout << "video pts total:" << video_st.next_pts / common->frame_rate << " audio pts total:" << audio_st.next_pts / 176400 << endl;
    cout << "video pts calculated:" << common->tot_vid_count / common->frame_rate <<
            " audio pts calculated:" << common->tot_audio_count / 176400 << endl;
    return 0;
}