#include <stdlib.h>
#include <stdio.h>
#include <math.h>
int* one_hot_output2(int output)
{
	int* ont_hot_result = malloc(sizeof(int) * 14);
	for (int i = 0; i < 14; i++)
	{
		if (i == output)
			ont_hot_result[i] = 1;
		else
			ont_hot_result[i] = 0;
	}
	return ont_hot_result;

}
float sigmoid3(float x) { return (float)1 / (1 + exp(-x)); }

int get_result(float* outputs, int length)
{
	int pred_ans = 0;
	float max = -999999;
	for (int i = 0; i < length; i++)
	{
		if (outputs[i] > max)
		{
			max = outputs[i];
			pred_ans = i;
		}
	}
	return pred_ans;
}

int one_hot_inverse(int* outputs, int length)
{

	for (int i = 0; i < length; i++)
	{
		if (outputs[i] == 1)
		{
			return i;
		}
	}
	return 999;
}

float update_output_weights(float ground_truth, float out_layer_val, float hidden_layer_val, float output_weigth, float lr)
{
	float error, new_weight;
	error = -(ground_truth - out_layer_val) * out_layer_val * (1 - out_layer_val) * hidden_layer_val;
	new_weight = output_weigth - (lr * error);
	return new_weight;
}

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
	float lr)
{
	int* one_hot_result = one_hot_output2(train_output);
	//float lr = 1;

	float* hidden_layer = malloc(sizeof(float) * num_hiddennodes);
	for (int i = 0; i < num_hiddennodes; i++)
		hidden_layer[i] = 0;

	float* output_layer = malloc(sizeof(float) * output_length);
	for (int i = 0; i < output_length; i++)
		output_layer[i] = 0;

	float** hidden_old_weight = malloc(sizeof(float*) * num_hiddennodes);
	for (int i = 0; i < num_hiddennodes; i++)
		hidden_old_weight[i] = malloc(sizeof(float) * input_length);

	float** output_old_weight = malloc(sizeof(float*) * output_length);
	for (int i = 0; i < output_length; i++)
		output_old_weight[i] = malloc(sizeof(float) * num_hiddennodes);

	float* hidden_bias_old_weights = malloc(sizeof(float) * num_hiddennodes);
	for (int i = 0; i < num_hiddennodes; i++)
		hidden_bias_old_weights[i] = 0.03;

	float* output_bias_old_weights = malloc(sizeof(float) * output_length);
	for (int i = 0; i < output_length; i++)
		output_bias_old_weights[i] = 0.03;

	float* delta = malloc(sizeof(float) * output_length);
	for (int i = 0; i < output_length; i++)
		delta[i] = 0;

	float* eh = malloc(sizeof(float) * num_hiddennodes);
	for (int i = 0; i < num_hiddennodes; i++)
		eh[i] = 0;

	float* loss_split = malloc(sizeof(float) * output_length);
	for (int i = 0; i < output_length; i++)
		loss_split[i] = 0;

	//for (int i = 0; i < input_length; i++)
	//	printf("train_input: %.2f\n", train_input[i]);

	for (int i = 0; i < input_length; i++)
	{
		for (int h = 0; h < num_hiddennodes; h++)
		{
			hidden_old_weight[h][i] = hidden_weights[h][i];
			hidden_layer[h] += train_input[i] * hidden_weights[h][i];
			//printf("HiddenLayer%d: %.5f\n",h, hidden_layer[h]);
		}
	}


	for (int h = 0; h < num_hiddennodes; h++)
		printf("Hidden_Layer: %.8f\n", hidden_layer[h]);


	for (int h = 0; h < num_hiddennodes; h++)
	{
		hidden_layer[h] += hidden_layer_bias[h];
		hidden_bias_old_weights[h] = hidden_layer_bias[h];
		hidden_layer[h] = sigmoid3(hidden_layer[h]);
	}

	for (int o = 0; o < output_length; o++)
	{
		for (int h = 0; h < num_hiddennodes; h++)
		{
			output_layer[o] += hidden_layer[h] * output_weights[o][h];
		}
		output_layer[o] += output_layer_bias[o];
		output_bias_old_weights[o] = output_layer_bias[o];
		output_layer[o] = sigmoid3(output_layer[o]);
	}
	int pred = 0;
	pred = get_result(output_layer, output_length);

	for (int o = 0; o < output_length; o++)
		printf("%.5f ", output_layer[o]);
	int ground_truth = one_hot_inverse(one_hot_result, output_length);
	printf("\nPred: %d   GroundTruth: %d  \n", pred, ground_truth);

	//Back propagation
	for (int o = 0; o < output_length; o++)
	{
		for (int h = 0; h < num_hiddennodes; h++)
		{
			output_old_weight[o][h] = output_weights[o][h];
			output_weights[o][h] = update_output_weights(one_hot_result[o], output_layer[o], hidden_layer[h], output_weights[o][h], lr);
		}
		output_layer_bias[o] = update_output_weights(one_hot_result[o], output_layer[o], 1, output_layer_bias[o], lr);
	}

	float eb = 0;
	for (int o = 0; o < output_length; o++)
		delta[o] = -(one_hot_result[o] - output_layer[o]) * output_layer[o] * (1 - output_layer[o]);

	for (int h = 0; h < num_hiddennodes; h++)
	{
		for (int o = 0; o < output_length; o++)
			eh[h] = eh[h] + delta[o] * output_old_weight[o][h];
	}

	for (int o = 0; o < output_length; o++)
		eb += delta[o] * output_bias_old_weights[o];

	for (int h = 0; h < num_hiddennodes; h++)
	{
		for (int i = 0; i < input_length; i++)
		{
			hidden_weights[h][i] = hidden_old_weight[h][i] - lr * (eh[h] * hidden_layer[h] * (1 - hidden_layer[h]) * train_input[i]);
		}
		hidden_layer_bias[h] = hidden_bias_old_weights[h] - lr * (eb * hidden_layer[h] * (1 - hidden_layer[h]) * 1);
	}
	float loss = 0;
	for (int o = 0; o < output_length; o++)
	{
		loss_split[o] = loss_split[0] + ((one_hot_result[o] - output_layer[o]) * (one_hot_result[o] - output_layer[o]));
		loss += loss_split[o];
	}

	loss = loss / output_length;

	printf("MSE Loss : %.4f\n", loss);
	free(one_hot_result);
	free(hidden_layer);
	free(output_layer);
	free(hidden_old_weight);
	free(output_old_weight);
	free(hidden_bias_old_weights);
	free(output_bias_old_weights);
	free(delta);
	free(eh);
	free(loss_split);
}
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
	float lr)
{
	int* one_hot_result = one_hot_output2(train_output);
	//float lr = 1;

	float* hidden_layer = malloc(sizeof(float) * num_hiddennodes);
	for (int i = 0; i < num_hiddennodes; i++)
		hidden_layer[i] = 0;

	float* output_layer = malloc(sizeof(float) * output_length);
	for (int i = 0; i < output_length; i++)
		output_layer[i] = 0;


	for (int i = 0; i < input_length; i++)
	{
		for (int h = 0; h < num_hiddennodes; h++)
		{
			hidden_layer[h] += train_input[i] * hidden_weights[h][i];
			//printf("Hidden_weights: %.2f\n", hidden_weights[h][i]);
		}
	}

	for (int h = 0; h < num_hiddennodes; h++)
	{
		hidden_layer[h] += hidden_layer_bias[h];
		hidden_layer[h] = sigmoid3(hidden_layer[h]);
	}

	for (int o = 0; o < output_length; o++)
	{
		for (int h = 0; h < num_hiddennodes; h++)
		{
			output_layer[o] += hidden_layer[h] * output_weights[o][h];
		}
		output_layer[o] += output_layer_bias[o];
		output_layer[o] = sigmoid3(output_layer[o]);
	}
	int pred = 0;
	pred = get_result(output_layer, output_length);

	for (int o = 0; o < output_length; o++)
		printf("%.5f ", output_layer[o]);
	int ground_truth = one_hot_inverse(one_hot_result, output_length);
	printf("\nPred: %d   GroundTruth: %d  \n", pred, ground_truth);
	free(one_hot_result);
	free(hidden_layer);
	free(output_layer);
}