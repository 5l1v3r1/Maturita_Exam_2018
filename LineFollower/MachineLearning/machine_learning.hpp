#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#define Ni                      (0.1f)
#define Alpha                   (0.50f)
#define error_max               (0.00001f)
#define iteration_MAX           (1000000)

#define tanh_derivation(y)      (1.0f - (y * y))
#define P(num_W, num_M)         (int)((num_W / (1 - error_max)) * logf(num_M / (1 - error_max)))

class MachineLearning
{
    public:
        //  Variables
        uint num_of_layers, last_layer_index, *num_of_neurons, num_of_patterns;
        float ***weights, ***delta_weights_old, **sigma, **output;

        //  Functions
        MachineLearning(uint _num_of_layers, uint *_num_of_neurons, uint _num_of_patterns);
        void initNet(int seed);
        void getOutput(float *_inputs);
        float getError(float *_o);
        void AdaptationWeights(float *_inputs);
        void clearOutputs(void);
        ~MachineLearning();        
};