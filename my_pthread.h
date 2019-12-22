#ifndef MY_PTHREAD_H
#define MY_PTHREAD_H

#include <iostream>

void* process(void* ptr);
void* display(void* ptr);
void* read_frame(void* ptr);
void* pthread_parallel(void* ptr);
void* pthread_parallel(std::string video_path);
void display_save();

#endif // MY_PTHREAD_H
