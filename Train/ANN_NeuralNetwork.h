#pragma once
#ifndef _ANN_NEURALNETWORK_H_
#define _ANN_NEURALNETWORK_H_
void ann_train(
	int input_length,
	int output_length,
	int num_hiddennodes,
	float* train_input,
	float* hidden_layer_bias,
	float* output_layer_bias,
	float** hidden_weights,
	float** output_weights,
	int train_output,
	float lr);
void ann_test(
	int input_length,
	int output_length,
	int num_hiddennodes,
	float* train_input,
	float* hidden_layer_bias,
	float* output_layer_bias,
	float** hidden_weights,
	float** output_weights,
	int train_output,
	float lr);
#endif // !_TRAIN_H_



