/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


#include "logger.h"


void log_activity(const char * fname, const char * c)
{
    FILE *fp;
    fp = fopen(fname, "a+");
    fprintf(fp,"%s\n",c);
    fclose(fp);    
}
void get_log(const char * msg, const char * stat, char * bf)
{
    char tbuff[20]={0};
    time_t now = time(NULL);
    struct tm * to = localtime(&now);

    
    strftime(tbuff, sizeof (tbuff), "%Y-%m-%d %H:%M:%S", to);
    char result[1024] = {0};
    sprintf(bf,"%s [%s] %s", tbuff,stat, msg);   
}

void rename_log(const char* filename_no_ext, const char* ext)
{
    char old[256] = {0};
    sprintf(old, "%s%s", filename_no_ext,ext);
    if( (access(old, 0 )) == -1 )
        return;
    
    char tbuff[20]={0};
    time_t now = time(NULL);
    struct tm * to = localtime(&now);

    
    strftime(tbuff, sizeof (tbuff), "%Y%m%d_%H%M%S", to);
    char new[256] = {0};
    sprintf(new,"%s_%s%s",filename_no_ext ,tbuff, ext);   

    printf("old %s, new %s\n",old, new);
    rename(old, new);
}


