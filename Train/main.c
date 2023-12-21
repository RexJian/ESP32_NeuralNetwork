#pragma  warning (disable:4996) 
#include < stdio.h>
#include <string.h>
#include <stdlib.h>
#include<math.h>
#include "ANN_NeuralNetwork.h"
#include "Preprocessing.h"


#define NUM_HIDDEN_NODES 12
#define NUM_OUTPUTS 14


float init_weights2() { return (float)rand() / (float)RAND_MAX; }
//float init_weights() {
//	float a = (float)rand() / (float)RAND_MAX;
//	int b = (int)rand();
//	if (b % 2 == 0)
//		return -a;
//	else
//		return a;
//}

int main()
{

	char* yagiyang_datapth = "E:/Course/MCU/ESP/data/yagiyang2/";
	char num_cnt[3];
	int len = 0;
	int epoch = 500;
	float** axis_datas = (float**)malloc(sizeof(float*) * 3);
	char* paths[] = { "zero","one","two","three","four","five","six","seven","eight","nine" ,"add","sub","mul","div" };
	int target_len = 30;
	int batch_size = 1;
	int batch_size_cnt = 0;
	float* train_data = (float*)malloc(sizeof(float) * batch_size * target_len * 3);
	int train_cnt = 0;

	//set model parameter 


	float* hidden_layer_bias = malloc(sizeof(float) * NUM_HIDDEN_NODES);
	for (int i = 0; i < NUM_HIDDEN_NODES; i++)
		hidden_layer_bias[i] = init_weights2();

	float* output_layer_bias = malloc(sizeof(float) * NUM_OUTPUTS);
	for (int i = 0; i < NUM_OUTPUTS; i++)
		output_layer_bias[i] = init_weights2();

	float** hidden_weights = malloc(sizeof(float*) * NUM_HIDDEN_NODES);
	for (int i = 0; i < NUM_HIDDEN_NODES; i++)
		hidden_weights[i] = malloc(sizeof(float) * target_len * 3);
	for (int i = 0; i < NUM_HIDDEN_NODES; i++)
		for (int j = 0; j < target_len * 3; j++)
			hidden_weights[i][j] = 0.1;

	float** output_weights = malloc(sizeof(float*) * NUM_OUTPUTS);
	for (int i = 0; i < NUM_OUTPUTS; i++)
		output_weights[i] = malloc(sizeof(float) * NUM_HIDDEN_NODES);
	for (int i = 0; i < NUM_OUTPUTS; i++)
		for (int j = 0; j < NUM_HIDDEN_NODES; j++)
			output_weights[i][j] = init_weights2();

	float lr = 1;
	/////////////////////////////

	for (int epoch_cnt = 0; epoch_cnt < epoch; epoch_cnt++)
	{
		printf("This is epoch %d\n", epoch_cnt);
		for (int cnt = 1; cnt < 11; cnt++)
		{
			for (int num = 0; num < NUM_OUTPUTS; num++)
			{

				batch_size_cnt++;
				char full_log_pth[100] = { 0 };
				get_full_log_path(yagiyang_datapth, paths[num], cnt, full_log_pth);
				char* text = read_txt(full_log_pth);
				int data_len = get_data_len(text);
				get_3axis_data(axis_datas, data_len, text);
				if (data_len < target_len)
					axis_datas = upsample2(axis_datas, data_len, target_len);

				else if (data_len > target_len)
					axis_datas = downsample(axis_datas, data_len, target_len);

				/*normalize_data(axis_datas[0], target_len);
				normalize_data(axis_datas[1], target_len);
				normalize_data(axis_datas[2], target_len);*/


				/*for (int j = 0; j <23; j++)
					printf("x:%.2f  y:%.2f  z:%.2f\n", axis_datas[0][j], axis_datas[1][j], axis_datas[2][j]);*/


				if (cnt != 10)
				{
					train_cnt = change_data_format(axis_datas, train_data, train_cnt, target_len);

					if (train_cnt == batch_size * target_len * 3)
					{
						ann_train(
							train_cnt,
							NUM_OUTPUTS,
							NUM_HIDDEN_NODES,
							train_data,
							hidden_layer_bias,
							output_layer_bias,
							hidden_weights,
							output_weights,
							num,
							lr);
						train_cnt = 0;
						batch_size_cnt = 0;
					}
				}
			}
		}
	}
	batch_size_cnt = 0;
	printf("\n");
	printf("\n");

	//Test
	printf("Test Start!!\n");
	for (int num = 0; num < NUM_OUTPUTS; num++)
	{

		batch_size_cnt++;
		char full_log_pth[100] = { 0 };
		get_full_log_path(yagiyang_datapth, paths[num], 10, full_log_pth);
		char* text = read_txt(full_log_pth);
		int data_len = get_data_len(text);
		get_3axis_data(axis_datas, data_len, text);
		if (data_len < target_len)
			axis_datas = upsample2(axis_datas, data_len, target_len);

		else if (data_len > target_len)
			axis_datas = downsample(axis_datas, data_len, target_len);

		//normalize_data(axis_datas[0], target_len);
		//normalize_data(axis_datas[1], target_len);
		//normalize_data(axis_datas[2], target_len);


		/*for (int j = 0; j <23; j++)
			printf("x:%.2f  y:%.2f  z:%.2f\n", axis_datas[0][j], axis_datas[1][j], axis_datas[2][j]);*/

		train_cnt = change_data_format(axis_datas, train_data, train_cnt, target_len);

		if (train_cnt == batch_size * target_len * 3)
		{
			ann_test(
				train_cnt,
				NUM_OUTPUTS,
				NUM_HIDDEN_NODES,
				train_data,
				hidden_layer_bias,
				output_layer_bias,
				hidden_weights,
				output_weights,
				num,
				lr);
			train_cnt = 0;
			batch_size_cnt = 0;
		}

	}

	printf("\n\n\n");
	printf("hidden weights\n");
	for (int i = 0; i < NUM_HIDDEN_NODES; i++)
	{
		for (int j = 0; j < target_len * 3; j++)
			printf("%.2f ,", hidden_weights[i][j]);
		printf("\n");
	}
	printf("\n\n\n");
	printf("hidden layer bias\n");
	for (int i = 0; i < NUM_HIDDEN_NODES; i++)
		printf("%.2f ,", hidden_layer_bias[i]);

	printf("\n\n\n");
	printf("output_weights\n");
	for (int i = 0; i < NUM_OUTPUTS; i++)
	{
		for (int j = 0; j < 12; j++)
			printf("%.2f ,", output_weights[i][j]);
		printf("\n");
	}

	printf("\n\n\n");
	printf("output layer bias\n");
	for (int i = 0; i < NUM_OUTPUTS; i++)
		printf("%.2f ,", output_layer_bias[i]);
	return 0;
}
