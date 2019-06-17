/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   cam_exception.cpp
 * Author: Nabil Dehaini
 * 
 * Created on February 7, 2019, 6:07 PM
 */

#include "cam_exception.h"
#include<string.h>

cam_exception::cam_exception() {
}

cam_exception::cam_exception(const cam_exception& orig) {
}

cam_exception::~cam_exception() {
}

cam_exception::cam_exception(const string& _msg, bool incl_sys_error) throw() : msg(_msg) {
    if(incl_sys_error) {
    msg.append(":[sys_error]");
    msg.append(strerror(errno));
  }
}

const char *cam_exception::what() const throw()
{
    return msg.c_str();
}