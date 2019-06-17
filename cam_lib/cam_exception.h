/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   cam_exception.h
 * Author: Nabil Dehaini
 *
 * Created on February 7, 2019, 6:07 PM
 * A simple exception class for handling exceptions.  
 */

#ifndef CAM_EXCEPTION_H
#define CAM_EXCEPTION_H
#include<exception>
#include<string>

using namespace std;
class cam_exception: exception {
public:                     
    cam_exception();
    cam_exception (const string &_msg, bool incl_sys_error=false)throw();
    cam_exception(const cam_exception& orig);
    virtual ~cam_exception();
    virtual const char* what() const throw();
private:
    std::string msg;

};

#endif /* CAM_EXCEPTION_H */

