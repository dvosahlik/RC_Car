// 
// 
// 

#include "Transfer_function.h"
#include "Types.h"

typedef struct {
	z_function* fce;
	double* old_inputs;
	double* old_outputs;
	} tf_data_t;
	


/************************************************************************/
/* function the returns the result of a*b which are polynomials                                                                     */
/************************************************************************/
char multiply_polynom(double*a, int size_a, double*b, int size_b, double* res, int* size_res)
{
	*size_res = size_a*size_b;
	for (int i = 0; i < *size_res;i++)
	{
		res[i] = 0;
	}
	for (int i = 1; i <= size_a;i++)
	{
		for (int j = 1; j <= size_b;j++)
		{
			res[i*j - 1] += a[i - 1]*b[j - 1];
		}
	}
}

/************************************************************************/
/* not used. Intended to use for conversion from the continuous transfer function to the discrete one.                                                                  */
/************************************************************************/
char do_the_polynomial_coeffs(double *original, int original_size, double*next, double* multiply_with, int multiply_with_size, double** result, int* result_sizes, 
double Ts)
{
	double constant[2];
	for (int i =0;i < original_size;i++)
	{
		double* on_i = (double*)calloc(sizeof(double), multiply_with_size);
		on_i[0] = -1;
		on_i[1] = 1;
		constant[0] = -1;
		constant[1] = 1;
		int size_next = 2;
		for (int j = 1; j < i;j++)
		{
			multiply_polynom(constant, 2, on_i, size_next, next, &size_next);
			memcpy(on_i, next, size_next*sizeof(double));
		}
		if (i == 0)
		{
			on_i[0] = 1;
			on_i[1] = 0;
			size_next = 1;
		}
		multiply_polynom(on_i, size_next, multiply_with, multiply_with_size, next, &size_next);
		memcpy((result[i]), next, sizeof(double)*size_next);
		result_sizes[i] = size_next;
		for (int j = 0; j < size_next;j++)
		{
			(result[i])[j] = (result[i])[j]*original[i]*2/(double)Ts;
		}
		free(on_i);
	}
}

/************************************************************************/
/* not used. Intended to use for conversion from the continuous world to the discrete one.                                                                     */
/************************************************************************/
char convert_to_discrete_tf(s_function* fce, z_function* disc, double Ts)
{
	double* multiply_with = (double*)calloc(sizeof(double), fce->denominator_size*fce->nominator_size);
	double* next = (double*)malloc(sizeof(double)*fce->denominator_size*fce->nominator_size);
	int size_next = 2;
	multiply_with[0] = 1;
	multiply_with[1] = 1;
	double constant[2];
	constant[0] = 1;
	constant[1] = 1;
	for (int i = 2; i < fce->denominator_size;i++)
	{		
		multiply_polynom(constant, 2, multiply_with, size_next, next, &size_next);
		memcpy(multiply_with, next, size_next*sizeof(double));
	}
	double **nominator_new = (double**)malloc(sizeof(double*)*(fce->nominator_size));
	for(int i = 0; i < fce->nominator_size;i++)
	{
		(nominator_new[i]) = (double *)calloc(sizeof(double), (fce->nominator_size)*(fce->denominator_size));
	}
	int *nominator_new_sizes = (int*)calloc(sizeof(int), fce->nominator_size);
	do_the_polynomial_coeffs(fce->nominator, fce->nominator_size, next, multiply_with, fce->denominator_size, nominator_new, nominator_new_sizes, Ts);
	for (int i = 0; i < fce->nominator_size;i++)
	{
		for (int j = 0; j < fce->nominator_size;j++)
		{
			if (nominator_new_sizes[j] > i)
			{
				disc->nominator[i] += nominator_new[j][i];
			}
		}	 
	}
	
	double **denominator_new = (double**)malloc(sizeof(double*)*(fce->denominator_size));
	for(int i = 0; i < fce->denominator_size;i++)
	{
		(denominator_new[i]) = (double *)calloc(sizeof(double), (fce->denominator_size)*(fce->denominator_size));
	}
	int *denominator_new_sizes = (int*)calloc(sizeof(int), fce->denominator_size);
	do_the_polynomial_coeffs(fce->denominator, fce->denominator_size, next, multiply_with, fce->denominator_size, denominator_new, denominator_new_sizes, Ts);
	for (int i = 0; i < fce->denominator_size;i++)
	{
		for (int j = 0; j < fce->denominator_size;j++)
		{
			if (denominator_new_sizes[j] > i)
			{
				disc->denominator[i] += denominator_new[j][i];
			}
		}
	}
	
	
	free(multiply_with);
	free(next);
	for(int i = 0; i < fce->nominator_size;i++)
	{
		free(nominator_new[i]);
	}
	free(nominator_new);
	free(nominator_new_sizes);
	for(int i = 0; i < fce->denominator_size;i++)
	{
		free(denominator_new[i]);
	}
	free(denominator_new);
	free(denominator_new_sizes);
}

/************************************************************************/
/* shifts the data in a buffer *data. Throws out the last value and shifts all the others by one.                                                                     */
/************************************************************************/
char shift_data(double* data, int size)
{
	double tmp = data[0];
	for (int i = 1; i < size;i++)
	{
		double tmp2 = data[i];
		data[i] = tmp;
		tmp = tmp2;
		
	}
}

/************************************************************************/
/* returns the output of the discrete transfer function.                                                                     */
/************************************************************************/
double tf_output(double input, signal_block* block)
{
	tf_data_t* data = (tf_data_t*)(block->pointer_on_data);
	double y = 0;
	for (int i = 1; i < data->fce->denominator_size;i++)
	{
		y -= data->fce->denominator[i]*data->old_outputs[i - 1];
	}
	shift_data(data->old_inputs, data->fce->nominator_size);
	data->old_inputs[0] = input;
	for (int i = 0; i < data->fce->nominator_size;i++)
	{
		y += data->fce->nominator[i]*data->old_inputs[i];
	}
	y = y /(data->fce->denominator[0]);
	shift_data(data->old_outputs, data->fce->denominator_size);
	data->old_outputs[0] = y;
	return y;
}

/************************************************************************/
/* Constructor of the z-function. Returns the specified discrete transfer function as a signal block.                                                                     */
/************************************************************************/
char Transfer_function(z_function* fce, signal_block* block)
{
	//convert_to_discrete_tf(fce, discrete, Ts);
	//for(int i = discrete->nominator_size; i > 0;i--)
	//{
		//if (((discrete->nominator)[i]) != 0)
		//break;
		//discrete->nominator_size--;
	//}
	//
	//for(int i = discrete->denominator_size; i > 0;i--)
	//{
		//if (((discrete->denominator)[i]) != 0)
		//break;
		//discrete->denominator_size--;
	//}
	//double* den = (double *)calloc(sizeof(double), discrete->denominator_size);
	//double* nom = (double *)calloc(sizeof(double), discrete->nominator_size);
	//for (int i = 0; i < discrete->nominator_size;i++)
	//{
		//nom[discrete->denominator_size - i] = discrete->nominator[i];
	//}
	//for (int i = 0; i < discrete->denominator_size;i++)
	//{
		//den[discrete->denominator_size - i] = discrete->denominator[i];
	//}
	//free(discrete->denominator);
	//free(discrete->nominator);
	//discrete->nominator = nom;
	//discrete->denominator = den;
	//discrete->nominator_size = discrete->denominator_size;
	tf_data_t* data = (tf_data_t*) malloc(sizeof(tf_data_t));
	data->fce = fce;
	data->old_inputs = (double*)calloc(sizeof(double), fce->nominator_size);
	data->old_outputs = (double*)calloc(sizeof(double), fce->denominator_size);
	block->serial_signal_output = tf_output;
	block->pointer_on_data = data;
}


