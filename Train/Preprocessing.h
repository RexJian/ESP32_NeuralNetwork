#pragma once
#ifndef _PREPROCESSING_H_
#define _PREPROCESSING_H_

char* read_txt(char* read_pth);
int get_data_len(char* text);
void get_3axis_data(float** datas, int data_len, char* text);
//void get_substr(char* dest, char* src, int start, int cnt);
float** downsample(float** datas, int length, int target_length);
float get_median(float* nums, int len);
void insert_zeros(float** datas, int idx);
float** upsample2(float** datas, int length, int target_length);
int change_data_format(float** axis_data, float* train_data, int cnt, int target_length);
void normalize_data(float* datas, int length);
void get_full_log_path(char*, char*, int, char*);

#endif 



