#include <iostream>
#include<unistd.h>
#include"cam_capture.h"
#include "cam_exception.h"
#include "common.h"
#include "logger.h"
//using namespace cv;
using std::cout;
using std::cerr;
using std::endl;

int main(int, char**) {
    cout << "Opencv and audio to ffmpeg" << endl;
    rename_log(AV_LOG_FILE_NAME, AV_LOG_EXT);
    LOG_EX(AV_LOG_FILE, "AV Cam capture exe", "INFO ");


    bool do_capture = false;
    
    cam_capture c;
    try {
        c.init_all();
        char i = 0;
        while (i != 'q') {
            std::cin>>i;
            switch (i) {
                case 's':

  //                  c.capture_to_send(8080);
                    break;
                case 'c':
                    do_capture = !do_capture;
                    break;

                default:
                    break;

            }

            if (do_capture)
                c.start_capturing();
            else
                c.stop_capturing();
        }
        
        c.stop_capturing();
        c.close_all();
        
    } catch (cam_exception &e) {
        LOG_EX(AV_LOG_FILE, (char*) e.what(), "ERROR");
        c.stop_capturing();
        c.close_all();
        exit(1);
    }
    exit(0);
}
