#pragma  warning (disable:4996) 
#include < stdio.h>
#include <string.h>
#include <stdlib.h>
#include<math.h>
char* read_txt(char* read_pth)
{
	FILE* fptr = fopen(read_pth, "r");
	if (fptr == NULL)
	{
		printf("%s cannot read", read_pth);
		exit(1);
	}
	fseek(fptr, 0, SEEK_END);
	int text_len = ftell(fptr);
	rewind(fptr);
	char* text = malloc(sizeof(char) * text_len);
	fread(text, 1, text_len, fptr);
	fclose(fptr);
	return text;
}

int get_data_len(char* text)
{
	int count = 0, idx = 0;
	while (*(text + idx))
	{
		if (*(text + idx) == 'y')
			count++;
		idx++;
	}
	return count;
}

void get_substr(char* dest, char* src, int start, int cnt)
{
	strncpy(dest, src + start, cnt);
	dest[cnt] = 0;
}

void get_3axis_data(float** datas, int data_len, char* text)
{
	for (int col_idx = 0; col_idx < 3; col_idx++)
		datas[col_idx] = (float*)malloc(sizeof(float) * data_len);

	int cnt = 0, cnt_buf = 0, axis = 0;
	int idx1 = 0, idx2 = 0;
	while (cnt < data_len && *(text + idx1) != NULL)
	{
		char substr[6];
		if (*(text + idx1) == ':')
		{
			if (cnt_buf >= 2)
			{
				idx2 = idx1 + 5;
				char end_char = *(text + idx2);
				if (end_char < 48 || end_char>57)
				{
					get_substr(substr, text, idx1 + 1, 4);
					idx1 += 6;
				}
				else
				{
					get_substr(substr, text, idx1 + 1, 5);
					idx1 += 7;
				}
				//printf("%f\n", atof(substr));
				if (axis == 0)
				{
					datas[0][cnt] = atof(substr);
					axis++;
				}
				else if (axis == 1)
				{
					datas[1][cnt] = atof(substr);
					axis++;
				}
				else
				{
					datas[2][cnt] = atof(substr);
					axis = 0;
					cnt++;
				}

			}
			else
				idx1++;
			cnt_buf++;
		}
		else
			idx1++;
	}

}

float** downsample(float** datas, int length, int target_length)
{
	float** resample_datas = (float**)malloc(sizeof(float*) * 3);
	resample_datas[0] = (float*)malloc(sizeof(float) * target_length);
	resample_datas[1] = (float*)malloc(sizeof(float) * target_length);
	resample_datas[2] = (float*)malloc(sizeof(float) * target_length);
	float x_median = get_median(datas[0], length);
	float y_median = get_median(datas[1], length);
	float z_median = get_median(datas[2], length);
	float* diff_median = (float*)malloc(sizeof(float) * length);
	float* diff_median_buf = (float*)malloc(sizeof(float) * length);
	for (int i = 0; i < length; i++)
		diff_median[i] = fabs(datas[0][i] - x_median) + fabs(datas[1][i] - y_median) + fabs(datas[2][i] - z_median);

	//bubble sort
	float tmp = 0;
	for (int i = 0; i < length; i++)
		diff_median_buf[i] = diff_median[i];


	for (int i = length - 1; i > 0; i--)
	{
		for (int j = 0; j <= i - 1; j++)
		{
			if (diff_median_buf[j] > diff_median_buf[j + 1])
			{
				tmp = diff_median_buf[j];
				diff_median_buf[j] = diff_median_buf[j + 1];
				diff_median_buf[j + 1] = tmp;
			}
		}
	}


	for (int i = length - 1; i > length - (length - target_length) - 1; i--)
	{
		for (int j = 0; j < length; j++)
		{
			if (diff_median[j] == diff_median_buf[i])
			{
				diff_median[j] = 9999999;
				break;
			}
		}
	}
	//for (int i = 0; i < len; i++)
	//{
	//	printf("%.2f  ", diff_median[i]);
	//}
	//add data to resample data
	int j = 0;
	for (int i = 0; i < length; i++)
	{
		if (diff_median[i] != 9999999)
		{
			resample_datas[0][j] = datas[0][i];
			resample_datas[1][j] = datas[1][i];
			resample_datas[2][j] = datas[2][i];
			j++;
		}
		if (j >= target_length)
			break;
	}
	//normalize_data(resample_datas[0], target_length);
	//normalize_data(resample_datas[1], target_length);
	//normalize_data(resample_datas[2], target_length);
	//for (int j = 0; j < 23; j++)
	//	printf("x:%.2f  y:%.2f  z:%.2f\n", resample_datas[0][j], resample_datas[1][j], resample_datas[2][j]);
	free(datas);

	return resample_datas;
}
float get_median(float* nums, int len)
{
	float median;
	float* nums_buf = (float*)malloc(sizeof(float) * len);
	for (int i = 0; i < len; i++)
		nums_buf[i] = nums[i];
	float tmp;
	for (int i = 0; i < len; i++)
	{
		for (int j = 0; j < len - i - j; j++)
		{
			if (nums_buf[j] > nums_buf[j + 1])
			{
				tmp = nums_buf[j];
				nums_buf[j] = nums_buf[j + 1];
				nums_buf[j + 1] = tmp;
			}
		}
	}
	if (len % 2 == 0)
		median = (nums_buf[(len / 2) - 1] + nums_buf[(len / 2)]) / 2;
	else
		median = nums_buf[len / 2];

	//for (int i = 0; i < len; i++)
	//	printf("%.2f ", nums_buf[i]);
	return median;
}
void insert_zeros(float** datas, int idx)
{
	datas[0][idx] = 0;
	datas[1][idx] = 0;
	datas[2][idx] = 0;
}
float** upsample2(float** datas, int length, int target_length)
{
	float factor = (float)(length - 1) / (target_length - 1);
	float** resample_datas = (float**)malloc(sizeof(float*) * 3);
	resample_datas[0] = (float*)malloc(sizeof(float) * target_length);
	resample_datas[1] = (float*)malloc(sizeof(float) * target_length);
	resample_datas[2] = (float*)malloc(sizeof(float) * target_length);

	for (int i = 0; i < target_length; i++)
	{
		float index = i * factor;
		int lower_index = (int)index;
		int upper_index = lower_index + 1;
		float weight = index - lower_index;
		if (upper_index >= length)
		{
			resample_datas[0][i] = datas[0][lower_index];
			resample_datas[1][i] = datas[1][lower_index];
			resample_datas[2][i] = datas[2][lower_index];
		}
		else
		{
			resample_datas[0][i] = (1 - weight) * datas[0][lower_index] + weight * datas[0][upper_index];
			resample_datas[1][i] = (1 - weight) * datas[1][lower_index] + weight * datas[1][upper_index];
			resample_datas[2][i] = (1 - weight) * datas[2][lower_index] + weight * datas[2][upper_index];
		}
	}
	//normalize_data(resample_datas[0], target_length);
	//normalize_data(resample_datas[1], target_length);
	//normalize_data(resample_datas[2], target_length);

	free(datas[0]);
	free(datas[1]);
	free(datas[2]);

	return resample_datas;
}

int change_data_format(float** axis_data, float* train_data, int cnt, int target_length)
{
	for (int length_cnt = 0; length_cnt < target_length; length_cnt++)
	{
		for (int axis_cnt = 0; axis_cnt < 3; axis_cnt++)
		{
			train_data[cnt] = axis_data[axis_cnt][length_cnt];
			cnt++;
		}
	}
	return cnt;
}

void normalize_data(float* datas, int length)
{
	float min = datas[0];
	float max = datas[0];
	for (int i = 0; i < length; i++)
	{
		if (datas[i] < min)
			min = datas[i];
		if (datas[i] > max)
			max = datas[i];
	}
	for (int i = 0; i < length; i++)
		datas[i] = (datas[i] - min) / (max - min);
}

void get_full_log_path(char* folder, char* num, int cnt, char* full_log_pth)
{
	char num_cnt[3];
	memset(num_cnt, 0, sizeof(num_cnt));
	itoa(cnt, num_cnt, 10);
	strcat(full_log_pth, folder);
	strcat(full_log_pth, num);
	strcat(full_log_pth, num_cnt);
	strcat(full_log_pth, ".log");
}