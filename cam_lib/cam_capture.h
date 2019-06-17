/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   cam_capture.h
 * Author: Nabil Dehaini
 *
 * Created on February 1, 2019, 2:04 PM
 * This here is where we capture video from the a camera device with open_cv
 * and audio from a microphone device with SDL2.  When the video and audio buffers 
 * are full we execute a detached thread to mux the streams.  Meanwhile a second set of 
 * buffers is recording the video and audio.
 */

#ifndef CAM_CAPTURE_H
#define CAM_CAPTURE_H


#include "ffmpg_mux.h"
#include "common.h"//has all the necessary includes
#include "logger.h"
#include "configs.h"

using namespace cv;
using namespace std;


class cam_capture {
public:
    cam_capture();
    void say_hi();
    int init_all();
    int close_all();
    int capture_to_send(int port_no);
    int start_capturing();
    int stop_capturing();
    int stop_sending();

private:

    //audio stuff
    snd_pcm_t *capture_handle;

    bool is_capturing;
    audio_stream *audio_ptr;
    video_stream *video_ptr;

    unique_lock<mutex> vid_lock;
    unique_lock<mutex> aud_lock;
    int recording_device, playback_device;
    SDL_AudioSpec desiredAudioRecSpec, receivedAudioRecSpec, desiredAudioPlaySpec, receivedAudioPlaySpec;

    Mat frame, socket_frame;
    std::vector<uint8_t> imgbuf;

    VideoCapture capture;
    VideoWriter writer;
    char user_key;
    thread cam_thread;
    thread audio_thread;
    thread mux_thread;
    thread sock_thread;
    thread  v, a;
    mutex cam_mutex;

    mutex user_mutex;
    condition_variable cv; 
    bool capture_video, capture_audio;
    bool send_tcp;

    int server_socket, no_connections;
    int new_socks[5];
    ofstream raw_audio;
    int bytes_per_second;
    int bytes_to_read;
    double tot_audio_duration;

    int init_camera();
    int init_audio_SDL();

    int start_mux();

    void set_av_common_default();
    int set_video_writer(std::string filename);
    int stop_capture_to_file();
    int init_audio_stream();
    int init_video_stream();
    int fin_video();
    void do_capture();
    void capture_loop();
    void socket_main(int val);
    int send_frame(int &sock_fd, std::vector<uchar> encoded);
    int prepare_frame(std::vector<uchar> &encoded);
    int fin_audio_SDL();

    int check_sock(struct pollfd &pfd);
    int check_con();
    static void rec_callback(void *userdata, Uint8 * stream, int len);

    inline bool set_lock(unique_lock<mutex> &lk, mutex &m);
    inline void add_audio(uint8_t *buff, int len);
    inline void add_frame(Mat &f);
    inline void swap_vid_pointers();
    inline void swap_aud_pointers();
    inline bool check_audio_finished();
    inline bool vid_check_buff_end();
    inline double calc_sleep_time(myclock::time_point start_tp, double max_sleep_time, double &time_to_sleep_offset);
    inline int compare_av_time(double &sleep_modifier);
    int add_text(Mat frame, const char * text);
    
    /*AV Stream vars*/
    myclock::time_point vid_buff_time, aud_buff_time;
    
    ffmpeg_mux fmx;
    av_common common;
};
#endif /* CAM_CAPTURE_H */
