/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   logger.h
 * Author: nd
 *
 * Created on February 12, 2018, 8:53 AM
 */

#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
    
//#define FILE_NAME "/var/log/shutdown_daemon.log"
    
#define NEW_FILE(fname){FILE *fp = fopen(fname, "w+");fclose(fp);}    
#define LOG_EX(fname,msg,stat){char b[1024]={0}; get_log(msg, stat, b);log_activity(fname, b);}
#define PRNT(...){char _bf[1024] = {0}; snprintf(_bf, sizeof(_bf)-1, __VA_ARGS__);printf("%s\n", _bf);}

void log_activity(const char * fname, const char * c);
void get_log(const char * msg, const char * stat, char * bf);
void rename_log(const char* filename_no_ext,const char* ext);
    

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */

