/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   configs.h
 * Author: Nabil Dehaini
 *
 * Created on June 6, 2019, 8:49 AM
 * This here is only used to load a configuration file av_settings.cfg. The config
 * file is a name=value pair and holds all the video and audio configuration. 
 */

#ifndef CONFIGS_H
#define CONFIGS_H

#include <string>


using namespace std;
struct configs{
public:
    std::map<string, string> config_list;
    configs() {
    };

    void from_str(string s, int &val) {
        val = std::stoi(s);
    };
    void from_str(string s, string &val) {
        val = s;
    };
    
    void add_config(string str){
        string name, val;
        if(get_name_value(str, "=", name, val)!=string::npos)
            config_list.insert(std::make_pair(name, val));
    }
    
    template<typename T>
    void get_config(string cfg_name, T &val, T defaut_val){
        string s = config_list[cfg_name];
        if(s!="")
            from_str(s, val);
        else val=defaut_val;
    };

    int get_name_value(string str, string token, string &name, string &value) {
        int i = str.find(token);
        if (i != str.npos) {
            name = str.substr(0, i);
            value = str.substr(i + 1, str.length() - 1);
        }
        return i;
    };
    
    template<typename OUT>
    void get_val(string in, OUT& ret, OUT(*F)(string)){ret=F(in);};
};


#endif /* CONFIGS_H */

