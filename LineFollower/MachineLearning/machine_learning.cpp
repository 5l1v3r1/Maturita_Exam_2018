#include "machine_learning.hpp"

MachineLearning::MachineLearning(uint _num_of_layers, uint *_num_of_neurons, uint _num_of_patterns)
{   
    uint num_all_W, num_all_M;

    // Copy attributes
    num_of_layers = _num_of_layers;
    last_layer_index = _num_of_layers - 1;
    num_of_neurons = _num_of_neurons;
    num_of_patterns = _num_of_patterns;

    // add recurrent inputs
    num_of_neurons[0] += num_of_neurons[last_layer_index];

    //  init weights, sigma, input, output for layers
    weights = new float**[_num_of_layers];
    delta_weights_old = new float**[_num_of_layers];
    output = new float*[_num_of_layers];
    sigma = new float*[_num_of_layers];

    //  init weights for input layer ==> (s = 0)
    num_all_W = 0;
    num_all_M = _num_of_neurons[0];
    weights[0] = new float*[_num_of_neurons[0]];
    delta_weights_old[0] = new float*[_num_of_neurons[0]];
    output[0] = new float[_num_of_neurons[0]];
    sigma[0] = new float[_num_of_neurons[0]];
    for (uint m = 0; m < _num_of_neurons[0]; m++)
    {
        weights[0][m] = new float[1];
        delta_weights_old[0][m] = new float[1];
        num_all_W += 1;
    }

    //  init weights for other layers ==> (0 < s <= S_LAST)
    for (uint s = 1; s < _num_of_layers; s++)
    {
        num_all_M += _num_of_neurons[s];
        weights[s] = new float*[_num_of_neurons[s]];
        delta_weights_old[s] = new float*[_num_of_neurons[s]];
        output[s] = new float[_num_of_neurons[s]];        
        sigma[s] = new float[_num_of_neurons[s]];
        for (uint m = 0; m < _num_of_neurons[s]; m++)
        {
            weights[s][m] = new float[_num_of_neurons[s - 1]];
            delta_weights_old[s][m] = new float[_num_of_neurons[s - 1]];
            num_all_W += _num_of_neurons[s - 1];
        }
    }

    std::cout << "num_all_M = " << num_all_M << "\n";
    std::cout << "num_all_W = " << num_all_W << "\n";
    std::cout << "P = " << P(num_all_W, num_all_M) << "\n";
}

void MachineLearning::initNet(int seed)
{
    uint num_of_inputs;
    float k;

    // init rand()
    srand(seed);

    //  set psudo-random value for init NN
    for (uint s = 0; s < num_of_layers; s++)
    {
        for (uint m = 0; m < num_of_neurons[s]; m++)
        {
            if (s == 0) num_of_inputs = 1;
            else        num_of_inputs = num_of_neurons[s - 1];

            k = 2.4f / num_of_inputs;
            for (uint n = 0; n < num_of_inputs; n++)
            {
                weights[s][m][n] = (((float)rand() / RAND_MAX) * (k + k)) - k;
                delta_weights_old[s][m][n] = 0.0f;
            }
        }
    }
}

void MachineLearning::getOutput(float *_inputs)
{    
    //  active input neurons ==> (s = 0)
    for (uint m = 0; m < num_of_neurons[0] - num_of_neurons[last_layer_index]; m++)
    {
        //  calc output based on the input (t)
        output[0][m] = tanhf(weights[0][m][0] * _inputs[m]);
        //std::cout << "input[" << _t << "][" << m << "]= " << inputs[_t][m] << "   ";
    }
    //std::cout << "\n";
    for (uint m = num_of_neurons[0] - num_of_neurons[last_layer_index], i = 0; m < num_of_neurons[0]; m++, i++)
    {
        //  calc output based on the input (t)
        output[0][m] = tanhf(weights[0][m][0] * output[last_layer_index][i]);
        //std::cout << "input[" << _t << "][" << m << "]= " << inputs[_t][m] << "   ";
    }

    //  active other neurons ==> (0 < s <= S_LAST)
    for (uint s = 1; s < num_of_layers; s++)
    {
        for (uint m = 0, n1 = 0; m < num_of_neurons[s]; m++)
        {
            output[s][m] = 0.0f;

            //  calc output based on the previous layer from (t)
            for (n1 = 0; n1 < num_of_neurons[s - 1]; n1++)
            {
                output[s][m] += weights[s][m][n1] * output[s - 1][n1];
            }

            output[s][m] = tanhf(output[s][m]);
        }
    }
}

void MachineLearning::clearOutputs(void)
{
    for (uint i = 0; i < num_of_neurons[last_layer_index]; i++)
    {
        output[last_layer_index][i] = 0.0f;
    }    
}

float MachineLearning::getError(float *_o)
{
    float _J = 0.0f;

    // calc local net error and sigma
    for (uint m = 0; m < num_of_neurons[last_layer_index]; m++)
    {
        sigma[last_layer_index][m] = _o[m] - output[last_layer_index][m];
        _J += sigma[last_layer_index][m] * sigma[last_layer_index][m];
        sigma[last_layer_index][m] *= tanh_derivation(output[last_layer_index][m]);
    }
    _J *= 0.5f;
    
    //  calc sigma for other layer ==> (0 <= s < S_LAST)
    for (uint s = last_layer_index - 1; s >= 0; s--)
    {
        for (uint m = 0; m < num_of_neurons[s]; m++)
        {
            float sum = 0.0f;
            for (uint x = 0; x < num_of_neurons[s + 1]; x++)
            {
                sum += sigma[s + 1][x] * weights[s + 1][x][m];
            }
            sigma[s][m] = tanh_derivation(output[s][m]) * sum;
        }
    }

    return _J;
}

void MachineLearning::AdaptationWeights(float *_inputs)
{
    //  change weights of input neurons ==> (s = 0)
    for (uint m = 0; m < num_of_neurons[0] - num_of_neurons[last_layer_index]; m++)
    {
        //  calc delta weights based on the inputs from all times
        delta_weights_old[0][m][0] = (Ni * sigma[0][m] * _inputs[m]) + (Alpha * delta_weights_old[0][m][0]);
        weights[0][m][0] += delta_weights_old[0][m][0];
    }
    for (uint m = num_of_neurons[0] - num_of_neurons[last_layer_index], i = 0; m < num_of_neurons[0]; m++, i++)
    {
        //  calc delta weights based on the inputs from all times
        delta_weights_old[0][m][0] = (Ni * sigma[0][m] * output[last_layer_index][i]) + (Alpha * delta_weights_old[0][m][0]);
        weights[0][m][0] += delta_weights_old[0][m][0];
    }

    //  change weights of hidden neurons ==> (0 < s < S_LAST)
    for (uint s = 1; s < num_of_layers; s++)
    {
        for (uint m = 0, n1 = 0; m < num_of_neurons[s]; m++)
        {
            //  calc delta weights based on the previous layer from (t)
            for (n1 = 0; n1 < num_of_neurons[s - 1]; n1++)
            {
                delta_weights_old[s][m][n1] = (Ni * sigma[s][m] * output[s - 1][n1]) + (Alpha * delta_weights_old[s][m][n1]);
                weights[s][m][n1] += delta_weights_old[s][m][n1];
            }
        }
    }
}

MachineLearning::~MachineLearning()
{
}