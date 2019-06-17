/* 
 * File:   cam_capture.cpp
 * Author: Nabil Dehaini
 *
 * Created on February 1, 2019, 2:04 PM
 */


#include <iostream>
#include <unistd.h>
#include"cam_capture.h"
#include "cam_exception.h"

using std::cout; using std::cerr; using std::endl;


cam_capture::cam_capture() 
{
}


void cam_capture::say_hi(){
    std::cout<<"Hello World from cam_capture_lib"<<std::endl;
}

int cam_capture::init_all() {
    
    capture_video = false;
    capture_audio = false;
    is_capturing = false;
    send_tcp = false;
    no_connections = 0;
    
    common.audio1.buff = NULL;
    common.audio2.buff = NULL;

    common.video1.frames=NULL;
    common.video1.imgbuf=NULL;
    common.video2.frames=NULL;
    common.video2.imgbuf=NULL;
    
    memset(new_socks, 0, sizeof (new_socks));
    server_socket = 0;
    set_av_common_default();

    init_camera();
    init_audio_SDL();
    return 0;
}

int cam_capture::close_all() {
    
    LOG_EX(AV_LOG_FILE, "closing everything", "INFO ");
    fin_audio_SDL();   
    fin_video();
    capture.release();
    return 0;
}

int cam_capture::fin_audio_SDL() {
    delete[] common.audio1.buff;
    delete[] common.audio2.buff;
    SDL_CloseAudioDevice(recording_device);
    SDL_Quit();
    //raw_audio.close();
    return 0;
}


int cam_capture::init_camera()
{
    capture = VideoCapture(common.video_dev_id);
    if (!capture.isOpened())
    {
        const string error = "cannot init capture device " + std::to_string(common.video_dev_id);
        throw cam_exception(error, true);
    }
    
    capture.set(CAP_PROP_FRAME_WIDTH, common.frame_width);
    capture.set(CAP_PROP_FRAME_HEIGHT, common.frame_height);
    
//    capture.set(CAP_PROP_FPS, MAX_FRAMES);
    
    if(capture.set(CAP_PROP_FPS, 30)==0)
        LOG_EX(AV_LOG_FILE, "FPS set to 30", "INFO ");

//    double fps = capture.get(CAP_PROP_FPS);
//    cout<<"capture fps="<<fps<<endl;
    imgbuf = std::vector<uint8_t>(common.frame_height * common.frame_width * 3 + 16);
    frame = cv::Mat(common.frame_height, common.frame_width, CV_8UC3, imgbuf.data(), common.frame_width * 3);
    
//    common.video_timebase = AVRational{1, common.frame_rate};

    init_video_stream();
    return 0;
    
}


int cam_capture::init_video_stream() 
{
    auto init_stream = [this](video_stream & video) {
        video.read_idx = 0;
        video.write_idx = 0;
        video.frames_captured = 0;
        video.frames_to_write = 0;

        video.buffsize = common.frame_rate*(MAX_SECONDS+1);
        video.buffsize_max = common.frame_rate*MAX_SECONDS;
        video.imgbuf = new vector<uint8_t>[video.buffsize];
        video.frames = new Mat[video.buffsize];
        for (int i = 0; i < video.buffsize; i++) {
            video.imgbuf[i] = std::vector<uint8_t>(common.frame_height * common.frame_width * 3 + 16);
            video.frames[i] = cv::Mat(common.frame_height, common.frame_width, CV_8UC3, video.imgbuf[i].data(), common.frame_width * 3);
        }
    };
    
    init_stream(common.video1);
    init_stream(common.video2);
    video_ptr = &common.video1;
    return 0;
}

int cam_capture::fin_video()
{
    delete[] common.video1.frames;
    delete[] common.video1.imgbuf;
    delete[] common.video2.frames;
    delete[] common.video2.imgbuf;
    return 0;
}

int cam_capture::init_audio_stream() 
{
    auto init_stream = [this](audio_stream & audio) {
        audio.buffsize = (MAX_SECONDS + 1) * bytes_per_second;
        audio.buffsize_max = MAX_SECONDS*bytes_per_second;
        audio.buff = new uint8_t[audio.buffsize];
        audio.write_idx = 0;
        audio.read_idx = 0;

    };

    init_stream(common.audio1);
    init_stream(common.audio2);
    audio_ptr = &common.audio1;
    tot_audio_duration=0;
    return 0;
}

int cam_capture::init_audio_SDL()
{
//    auto set_audio_spec_S16 = [this](SDL_AudioSpec &s, SDL_AudioCallback c) {
//        SDL_zero(s);
//        s.freq = 44100;
//        s.format = AUDIO_S16;//AUDIO_F32;
//        s.channels = 2; //mono or stereo
//        s.samples = 4096;
//        s.callback = c;
//        s.userdata = this;
//    };
//    auto set_audio_spec_F32 = [](SDL_AudioSpec &s, SDL_AudioCallback c) {
//        SDL_zero(s);
//        s.freq = 44100;
//        s.format = AUDIO_F32;
//        s.channels = 2; //mono or stereo
//        s.samples = 4096;
//        s.callback = c;
//    };  
    
    auto set_audio_spec_S32 = [this](SDL_AudioSpec &s, SDL_AudioCallback c) {
        SDL_zero(s);
        s.freq = SAMPLE_FREQUENCY;//44100;
        s.format = AUDIO_S32;
        s.channels = 1; //mono or stereo
        s.samples = 4096;//2048;//4096;//8192;//8192;//4096;
        s.callback = c;
        s.userdata = this;
    };    
    
  
    
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        string er = string("SDL could not initialize! SDL Error: ",SDL_GetError());
        throw cam_exception(er);
//        return 1;
    }
    
    int no_recording_devices = SDL_GetNumAudioDevices(SDL_TRUE);
    if (no_recording_devices <= 0) {
        throw cam_exception("no recording devices found on system");
//        return 2;
    }

    if (common.audio_dev_id >= no_recording_devices)
        throw cam_exception("Audio device does not exist");
        //return 3;
    
    set_audio_spec_S32(desiredAudioRecSpec, rec_callback);

    recording_device = SDL_OpenAudioDevice(SDL_GetAudioDeviceName(common.audio_dev_id, SDL_TRUE), SDL_TRUE, &desiredAudioRecSpec, &receivedAudioRecSpec, SDL_AUDIO_ALLOW_FORMAT_CHANGE);
    if (recording_device == 0) {
        string er = string("Failed to open recording device! SDL_Error: ",SDL_GetError());
        throw cam_exception(er);
//        return 4;
    }

    int bytes_per_sample = receivedAudioRecSpec.channels * (SDL_AUDIO_BITSIZE(receivedAudioRecSpec.format) / 8);
    bytes_per_second = bytes_per_sample * receivedAudioRecSpec.freq;
    bytes_to_read = receivedAudioRecSpec.samples*bytes_per_sample;

    LOG_EX(AV_LOG_FILE,string("bytes per sample: "+ 
            std::to_string(bytes_per_sample)).c_str(),"INFO ");
    LOG_EX(AV_LOG_FILE, 
            string("bytes per second: "+ std::to_string(bytes_per_second)).c_str(), "INFO ");
    LOG_EX(AV_LOG_FILE, 
            string("bytes to read: "+ std::to_string(bytes_to_read)).c_str(), "INFO ");
    LOG_EX(AV_LOG_FILE,"Audio Initialized", "INFO " );

    init_audio_stream();
//    raw_audio.open("/tmp/media/raw_audio.raw")
    return 0;
    
}

void cam_capture::swap_aud_pointers() {
    audio_ptr->buff_tot = audio_ptr->write_idx;
    audio_ptr->write_idx = 0;
    audio_ptr->cv.notify_one();
    if (!common.mux_video)//If doing audio only
    {
        vid_lock = unique_lock<mutex>(video_ptr->mtx);
        video_ptr->cv.notify_one();
        vid_lock.unlock();

    }
    aud_lock.unlock();
    cout << "Audio Buff Write complete.\nbuff size= " << audio_ptr->buffsize_max << "\ntotal bytes written = " << audio_ptr->buff_tot <<endl;

    fmx.audio_reader1 = audio_ptr;
    audio_ptr = (audio_ptr == &common.audio1) ? &common.audio2 : &common.audio1;

}

void cam_capture::add_audio(uint8_t *buff, int len)
{
    static int first_time = 1;
    if(first_time)
    {
        first_time = 0;
        unique_lock<mutex> lk(cam_mutex);
        cv.notify_one();
        lk.unlock();
        return;
    }
    
    static bool islocked = false;
    if(islocked==false)
    {
        islocked = true;
        if(!set_lock(aud_lock, audio_ptr->mtx))
            throw cam_exception("Could not lock the audio buffer to write. This shouldn't happen.");
    }     
//    this->raw_audio.write((const char*)buff, len);
    memcpy(&audio_ptr->buff[audio_ptr->write_idx], buff, len);
    audio_ptr->write_idx+=len;

    if((audio_ptr->write_idx>=audio_ptr->buffsize_max)||(!capture_audio))
    {        
        common.aud_time+=audio_ptr->write_idx;
        common.audio_ready = true;
        swap_aud_pointers();
 
        double dur = get_time(aud_buff_time);
        cout<<"\nTotal time to fill audio buffer = " << dur << " seconds" << "\nTotal Audio Duration=" << tot_audio_duration << endl;
        tot_audio_duration+=dur;
        islocked = false;

        if(!capture_audio)
            SDL_PauseAudioDevice(recording_device, SDL_TRUE);
    }
}

void cam_capture::rec_callback(void* userdata, Uint8* stream, int len)
{
    cam_capture* p = (cam_capture*)userdata;
    
 //   cout<<"got data.  Length: "<<len<<endl;
    p->add_audio(stream, len);
}

bool cam_capture::vid_check_buff_end()
{
    bool res = false;
    if(video_ptr->write_idx==video_ptr->buffsize_max)
    {
        swap_vid_pointers();

        common.audio_ready=(!capture_audio)?true:common.audio_ready;
        myclock::time_point tp = myclock::now();
        double dur  = time_duration(vid_buff_time, tp);
        cout<<"Total time to fill video buffer is "<<dur<<" seconds"<<endl;
        vid_buff_time = tp;      
    }
    return res;        
}


void cam_capture::swap_vid_pointers()
{
    video_ptr->frames_to_write = video_ptr->write_idx;
    cout<<"<<<<<<<<<<<<Video Buff Capture complete with "<<video_ptr->frames_to_write<<" frames"<<endl;
    video_ptr->write_idx=0;
    common.video_ready = true;

    video_ptr->cv.notify_one();//make sure that audio.buff_ready and video.buff_ready are set to true by now.  
    vid_lock.unlock();
    
    fmx.video_reader1 = video_ptr;
    video_ptr = (video_ptr==&common.video1)?&common.video2:&common.video1;
    //fmx.process_mux();
}
bool cam_capture::check_audio_finished()
{
    bool res = false;

    if(common.audio_ready){
        swap_vid_pointers();
        res = true;
        double dur  = get_time(vid_buff_time);
        cout<<"Total time to fill video buffer is "<<dur<<" seconds"<<endl;
    }
    return res;
}


void cam_capture::add_frame(Mat &f) {
    //check that haven't blown the buffer
    if(video_ptr->write_idx>video_ptr->buffsize)
    {
        throw cam_exception("Error video buffer completely overrun. Possible that audio function hasn't returned");
    }
    
    f.copyTo(video_ptr->frames[video_ptr->write_idx]);
    common.vid_time++;
    video_ptr->write_idx++;
}


bool cam_capture::set_lock(unique_lock<mutex> &lk, mutex &m)
{
    lk = unique_lock<mutex>(m, defer_lock);
    bool res = lk.try_lock();
    return res;
}


double cam_capture::calc_sleep_time(myclock::time_point start_tp, double max_sleep_time, double &time_to_sleep_offset){
        myclock::time_point next_t = myclock::now();
        double frame_duration = time_duration(start_tp, next_t)*USEC_UNIT;
        double sleep_time = max_sleep_time - frame_duration; 
        if(sleep_time<0)
        {
            //put a counter and count how many times this happens then raise an exception
            cout<<"frame duration time spillover. sleep time: "<<sleep_time<<" Frame duration: "<<
                    frame_duration<<" Max time to sleep: "<<max_sleep_time<<endl;
            sleep_time = 1;
        }
        //count how many times the offset is greater than sleep time. If more than 10 then raise an exception     
        if(std::abs(time_to_sleep_offset)>sleep_time)
            cout<<"why is sleep time less than time_to_sleep offset?  sleep_time:"<<sleep_time<<"time_to_sleep_offset"<<time_to_sleep_offset<<endl;
        sleep_time = (sleep_time+time_to_sleep_offset)<0?0:sleep_time+time_to_sleep_offset;

        return sleep_time;
    
}
/*compares the pts of audio and video.  This keeps then 2 streams synchronized.  
 returns 0 if are equal; 1 vid greater; -1 aud greater  
 */
int cam_capture::compare_av_time(double &sleep_offset) {
    static double err_margin = 1/(double)common.frame_rate;
    double aud_t = common.aud_time/bytes_per_second;
    double vid_t = common.vid_time/common.frame_rate;

    sleep_offset = vid_t-aud_t;
    int res = (sleep_offset>err_margin)?1:(sleep_offset<-err_margin)?-1:0;
    if(res>0)
        cout<<"<<<<<<<<<<<video ahead of audio by:"<<sleep_offset<<"err margin"<<err_margin<<endl;
    else if(res<0) 
        cout<<">>>>>>>>>>>>audio ahead:"<<sleep_offset<<endl;
    else{
        sleep_offset = 0;
        cout<<"audio and video in line"<<endl;
    }
    sleep_offset*=(USEC_UNIT/common.frame_rate);
    
    return res;
}

void cam_capture::do_capture() {
    try{
        capture_loop();
    }
    catch(cam_exception &e){
        LOG_EX(AV_LOG_FILE, e.what(), "ERROR");
        exit(1);
    }

}

void cam_capture::capture_loop()
{
    double fps_max_sleep_time,sleep_time, time_to_sleep_offset;
    myclock::time_point start_tp, next_t;
    bool end_period = false;

    vid_buff_time = myclock::now();

    fps_max_sleep_time = USEC_UNIT/common.frame_rate;
    
    if(common.mux_audio)
    {
        capture_audio = true;
        aud_buff_time = myclock::now();
        SDL_PauseAudioDevice(recording_device, SDL_FALSE);
        unique_lock<mutex> lk(cam_mutex);
        cv.wait(lk);
        lk.unlock();
    }
    
    
    set_lock(vid_lock, video_ptr->mtx);
    time_to_sleep_offset=0;
    while(capture_video)
    {
        start_tp = myclock::now();
        
        capture>>frame;
        if(frame.empty())
        {
            continue;
        }
        
        video_ptr->frames_captured++;
        add_frame(frame);

        if(common.mux_audio)
            end_period= check_audio_finished();
        else
            end_period= vid_check_buff_end();
        
        if(end_period)
        {
            int r=compare_av_time(time_to_sleep_offset);

            end_period=false;
            fmx.process_mux();
            //we should possibly end every thing around here.  When we want to stop the application
            assert(video_ptr != fmx.video_reader1);
            if(!set_lock(vid_lock, video_ptr->mtx))
                throw cam_exception("Could not lock video mutex.  Possible problem with mux thread hogging the lock");
        }
        //check if debugging
        imshow("Camera Capture", frame);
        waitKey(1);

        sleep_time = calc_sleep_time(start_tp, fps_max_sleep_time, time_to_sleep_offset);
        usleep(sleep_time);
    }
    usleep(100000);
    if (common.mux_audio) {
        capture_audio = false;
//        SDL_PauseAudioDevice(recording_device, SDL_TRUE);
        //wait for the audio to come back and then mux and finish up.  
        //this is not very neat and should be fixed eventually.  
        if (video_ptr->write_idx > 0) {//If we have video and audio frames to write
            while (!common.audio_ready) usleep(10000);//wait for the audio to come back
            cout<<"---------------finished capturing--------------"<<endl;
            cout<<"No of frames captured:"<<video_ptr->write_idx<<endl;
            cout<<"Audio buff size:"<<fmx.audio_reader1->buff_tot<<endl;
            cout<<"-----------------------------------------------"<<endl;
            swap_vid_pointers();//get the video read pointers ready.  Audio pointers are setup by the audio callback function
            fmx.process_mux();//mux video and audio
        }
    }
    
    cout<<"no of frames Captured = "<<common.video1.frames_captured+common.video2.frames_captured<<endl;
}

int cam_capture::start_mux()
{
    fmx.keepcapturing = true;
    fmx.audio_reader1 = audio_ptr;
    fmx.video_reader1 = video_ptr;
    fmx.common = &common;
    fmx.init();
    return 0;
}


int cam_capture::start_capturing()
{
    if(is_capturing)
        return 0 ;
    
    bool o = capture.isOpened(); 
    
    if(!capture.isOpened())//if capture is not openned then 
        throw cam_exception("capture is not opened");

    common.audio_ready = false;
    common.video_ready = false;
    common.tot_audio_count =0;
    common.tot_vid_count=0;

    is_capturing = true;
    start_mux();

    if(common.mux_video){
        capture_video = true;
        cam_thread = std::thread(&cam_capture::do_capture, this);
    }

    return 0;
}

int cam_capture::stop_capturing()
{
    if(!is_capturing)
        return 0;
    
    LOG_EX(AV_LOG_FILE, "Stopping capture", "INFO ");
    if(send_tcp)
        stop_sending();

  
    capture_video = false;
    capture_audio = false;
    fmx.keepcapturing = false;

    common.audio_ready = true;
    common.video_ready = true;
    
    usleep(USEC_UNIT);

    fmx.finish(); 
    LOG_EX(AV_LOG_FILE, "Muxing stopped", "INFO ");

    cam_thread.join();
    is_capturing = false;
    return 0;
}
int cam_capture::set_video_writer(std::string filename)
{
    /*to do
     check if filename exists here*/
    
    int fourcc = VideoWriter::fourcc('m','p','4','v');
    writer = VideoWriter(filename, fourcc,common.frame_rate, Size(common.frame_width, common.frame_height),true);
    int res = writer.isOpened()?0:1; 
    return res;
}

int cam_capture::capture_to_send(int port_no) {
    if(server_socket>0)
        return 0;
    //setup socket
    if((server_socket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP))==-1)
    {
        throw cam_exception("Error creating socket in method 'capture_to_send'");
    }
    

    sockaddr_in localAddr;
    memset(&localAddr, 0, sizeof (localAddr));
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port = htons(port_no);//htons(localPort);

    if (bind(server_socket, (sockaddr *) & localAddr, sizeof (sockaddr_in)) < 0) {
        throw cam_exception("Error binding socket to port "+std::to_string(port_no));
    }    
    if(listen(server_socket, 5)<0)
    {
        throw cam_exception("Error on socket listen");
    }
    send_tcp = true;
    
    sock_thread = std::thread(&cam_capture::socket_main, this, 0);
    
    return 0;
    
}

int cam_capture::stop_sending()
{
    send_tcp = false;
    //shutdown(server_socket, 2);
    if(server_socket>0)
        close(server_socket);
    sock_thread.join();
    server_socket = 0;
    return 0;
}

int cam_capture::check_sock(struct pollfd &pfd)
{
    if(poll(&pfd, 1,0)>0)
    {
        char buff;
        if(recv(pfd.fd, &buff, sizeof(buff),MSG_PEEK | MSG_DONTWAIT)==0)
            return 1;
    }
    return 0;
}
int cam_capture::check_con()
{
    struct pollfd pfd;
    struct sockaddr client;
    pfd.fd = server_socket;
    pfd.events = POLLIN;
    if(poll(&pfd,1,0)==POLLIN)
    {
        
//        int s = accept(server_socket, &client, (socklen_t*)sizeof(sockaddr));
        int s = accept(server_socket, NULL, 0);
        if(s==-1)
            throw cam_exception("error accepting new socket connection");
        new_socks[no_connections]=s;
        no_connections++;
    }
    return 0;
}

void rejig(int sock[5])
{
    int i=0, j=0;
    auto nextsock=[&]()//lambda
    {
        for(j;j<5;j++)
            if(sock[j]!=0)
            {
                sock[i]=sock[j];
                sock[j]=0;
                break;
            }
        return j;
    };
    
    for(int i=0;i<5;i++)
    {
        j=i+1;
        if(sock[i]==0)
            nextsock();
    }
}


void cam_capture::socket_main(int val)
{
    int i;
  
    std::vector<uchar> encoded;
    int j=0;
    while(send_tcp)
    {
        j++;
        if(j==10)
        {
            check_con();
            j=0;
        }
        //do_accept();
        if(no_connections==0)
        {
            usleep(1000);
            continue;
        }
        
        std::unique_lock<std::mutex> lck(cam_mutex);
        cv.wait(lck);
        //std::cout<<"socket is now ready to take frame to use."<<endl;
        //frame>>socket_frame;
        frame.copyTo(socket_frame);
        cv.notify_one();
        lck.unlock();
        if(prepare_frame(encoded)==0)
        {
            for(i=0;i<5;i++)
                if(new_socks[i]>0)
                    if (send_frame(new_socks[i], encoded)!=0)
                            std::cout<<"send frame error"<<endl;
        }
        rejig(new_socks);    
    }
    for(i=0;i<5;i++)
        if(new_socks[i]>0)close(new_socks[i]);
    memset(new_socks, 0, sizeof(new_socks));
    
 }

int cam_capture::send_frame(int &sock_fd, std::vector<uchar> encoded)
{
#define PACK_SIZE 4096

    struct pollfd pfd;
    pfd.fd = sock_fd;
    pfd.events = 0; 
    
    int i;
    int buf_info[2] = {0};
    buf_info[0] = encoded.size();
    buf_info[1] = buf_info[0] / PACK_SIZE + 1; //No of packets to send.  

    //lambda check if the data is sent otherwise close the socket;
    auto check_sent = [&](int data_sent){
        if(data_sent<=0){
            ::close(sock_fd); sock_fd=0;no_connections--;/*rejig(new_socks);*/};
            return (data_sent>0)?true:false;};

    auto check_and_send = [&](void* data,size_t buf_size ) {
        
        if(check_sock(pfd)==1)
        {
            close(sock_fd);
            sock_fd = 0;
            return (ssize_t)-1;
        }
        return send(sock_fd, data, buf_size, (int)0);
    };
    
//    if(!check_sent(send(sock_fd, buf_info, sizeof (buf_info), 0)))
//        return 2;
        
    if(check_and_send(buf_info, (size_t)sizeof(buf_info))==-1)
        return 2;
    
    for (i = 0; i < buf_info[1] - 2; i++)
    {
        if(check_and_send(&encoded[i * PACK_SIZE], PACK_SIZE)==-1)
            return 2;
    }
        
    int final_pack = buf_info[0]-(i * PACK_SIZE); //buf_info[0]=encoded.size()
    if(check_and_send(&encoded[i * PACK_SIZE], final_pack)==-1)
        return 2;

    cout << "\nframe_size =" << buf_info[0] << " Total data sent: " << ((i * PACK_SIZE) + final_pack) << endl;
    return 0;
    
}

int cam_capture::prepare_frame(std::vector<uchar> &encoded)
{
    int jpegqual = 80; // Compression Parameter
    //Mat frame, send;
    //cap >> frame;
    if (socket_frame.size().width == 0) 
        return 1; //simple integrity check; skip erroneous data...
    std::vector <int> compression_params;
    //resize(frame, send, Size(FRAME_WIDTH, FRAME_HEIGHT), 0, 0, INTER_LINEAR);
    
    compression_params.push_back(IMWRITE_JPEG_QUALITY);
    compression_params.push_back(jpegqual);

    imencode(".jpg", socket_frame, encoded, compression_params);
  //  imshow("send", send);
  //  waitKey(FRAME_INTERVAL);
    return 0;
}

int cam_capture::stop_capture_to_file()
{
//    writer.release();
    writer.~VideoWriter();
    return 0;
}

int cam_capture::add_text(Mat frame, const char * text)
{
//    const char * txt = "OpenCV forever!";
    Size textsize = getTextSize(text, FONT_HERSHEY_COMPLEX, 0.5, 0.5, 0);
    Point org(frame.size().width -textsize.width-5, frame.size().height - textsize.height-10);
    putText(frame, text, org, FONT_HERSHEY_COMPLEX, 0.5,
        Scalar(255,255,255), 0.5, LINE_AA);    
    
    return 0;
}

void cam_capture::set_av_common_default(){
    configs conf;
    ifstream cfg_file;
    string val;
    bool res=false;
    cfg_file.open("av_settings.cfg");
    if (cfg_file.is_open()) {
        while(std::getline(cfg_file, val)){
            conf.add_config(val);
        }
    }
    
    conf.get_config("frame_height", common.frame_height, 480);
    conf.get_config("frame_width", common.frame_width, 640);
    conf.get_config("frame_rate", common.frame_rate, 15);
    conf.get_config("video_dev_id",common.video_dev_id,0);
    conf.get_config("audio_dev_id",common.audio_dev_id,1);
    conf.get_config("av_file_name",common.filename,string(DEFAULT_FILE_NAME));
    
    common.mux_audio = true;
    common.mux_video = true;
}

