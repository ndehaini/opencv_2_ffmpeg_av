# open_cv_2_ffmpeg_av
capture opencv and SDL2 audio and mux in ffmpeg api


opencv_2_ffmpeg This project captures opencv video from a cam, audio from an attached device and muxes into mp4 file using ffmpeg api.

The learning knowledge and some code were acquired from the ffmpeg tutroials as well as other sources, I will give due credit to any sources in time.

Before using the code, please ensure that opencv, ffmpeg and SDL2 libs are installed.
Pull source Ensure the av_settings.cfg is correctly configured and is pointing to the correct audio and cam devices as well as the directories where mp4 file is going to be.


Start cam_exe
type c to start capturing.  You will get a display of the camera view. 
type q to quit

known issues:  
Type c to start capturing but if you type c again the capturing stops as it is supposed to.  If you start capturing again, the display hangs, so you can't restart the capturing.  I will fix this, but for now just type q to quit capturing.  
