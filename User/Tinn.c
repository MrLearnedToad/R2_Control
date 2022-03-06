#include "Tinn.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include "string.h"
#include "main.h"

// Computes error.
static float err(const float a, const float b)
{
    return 0.5f * (a - b) * (a - b);
}

// Returns partial derivative of error function.
static float pderr(const float a, const float b)
{
    return a - b;
}

// Computes total error of target to output.
static float toterr(const float* const tg, const float* const o, const int size)
{
    float sum = 0.0f;
    for(int i = 0; i < size; i++)
        sum += err(tg[i], o[i]);
    return sum;
}

// Activation function.
static float act(const float a)
{
    return 1.0f / (1.0f + expf(-a));
}

// Returns partial derivative of activation function.
static float pdact(const float a)
{
    return a * (1.0f - a);
}

// Returns floating point random from 0.0 - 1.0.
static float frand()
{
    return rand() / (float) RAND_MAX;
}

// Performs back propagation.
static void bprop(const Tinn t, const float* const in, const float* const ro, const float* const tg, float rate)
{
    for(int i = 0; i < t.nhid; i++)
    {
        float sum = 0.0f;
        // Calculate total error change with respect to output.
        for(int j = 0; j < t.nops; j++)
        {
            const float a = pderr(ro[j], tg[j]);
            const float b = pdact(ro[j]);
            sum += a * b * t.x[j * t.nhid + i];
            // Correct weights in hidden to output layer.
            t.x[j * t.nhid + i] -= rate * a * t.h[i];
        }
        // Correct weights in input to hidden layer.
        for(int j = 0; j < t.nips; j++)
            t.w[i * t.nips + j] -= rate * sum * pdact(t.h[i]) * in[j];
    }
}

// Performs forward propagation.
static void fprop(const Tinn t, const float* const in)
{
    // Calculate hidden layer neuron values.
    for(int i = 0; i < t.nhid; i++)
    {
        float sum = 0.0f;
        for(int j = 0; j < t.nips; j++)
            sum += in[j] * t.w[i * t.nips + j];
        t.h[i] = act(sum + t.b[0]);
    }
    // Calculate output layer neuron values.
    for(int i = 0; i < t.nops; i++)
    {
        float sum = 0.0f;
        for(int j = 0; j < t.nhid; j++)
            sum += t.h[j] * t.x[i * t.nhid + j];
        t.o[i] = sum + t.b[1];
    }
}

// Randomizes tinn weights and biases.
static void wbrand(const Tinn t)
{
    for(int i = 0; i < t.nw; i++) t.w[i] = frand() - 0.5f;
    for(int i = 0; i < t.nb; i++) t.b[i] = frand() - 0.5f;
}

// Returns an output prediction given an input.
float* xtpredict(const Tinn t, const float* const in)
{
    fprop(t, in);
    return t.o;
}

// Trains a tinn with an input and target output with a learning rate. Returns target to output error.
float xttrain(const Tinn t, const float* const in, const float* const ro, const float* const tg, float rate)
{    
    bprop(t, in, ro, tg, rate);
//    fprop(t, in);
    return toterr(tg, t.o, t.nops);
}

// Constructs a tinn with number of inputs, number of hidden neurons, and number of outputs
Tinn xtbuild(const int nips, const int nhid, const int nops)
{
    Tinn t;
    // Tinn only supports one hidden layer so there are two biases.
    t.nb = 2;
    t.nw = nhid * (nips + nops);
    t.w = (float*) calloc(t.nw, sizeof(*t.w));
    t.x = t.w + nhid * nips;
    t.b = (float*) calloc(t.nb, sizeof(*t.b));
    t.h = (float*) calloc(nhid, sizeof(*t.h));
    t.o = (float*) calloc(nops, sizeof(*t.o));
    t.nips = nips;
    t.nhid = nhid;
    t.nops = nops;
    wbrand(t);
    return t;
}

// Saves a tinn.
void xtsave(const Tinn t,UART_HandleTypeDef *uart)
{
    uint8_t transmit_buffer[20];
    transmit_buffer[0]=0x02;
    transmit_buffer[17]=1;
    float temp;
    // Save header.
    temp=t.nips;
    memcpy(transmit_buffer+1,&temp,4);
    temp=t.nhid;
    memcpy(transmit_buffer+5,&temp,4);
    temp=t.nops;
    memcpy(transmit_buffer+9,&temp,4);
    HAL_UART_Transmit(uart,transmit_buffer,18,10);
    for(int i=1;i<17;i++)
    {
        transmit_buffer[i]=0;
    }
    // Save biases and weights.
    for(int i = 0; i < t.nb; i++)
    {
        memcpy(transmit_buffer+1,&t.b[i],4);
        HAL_UART_Transmit(uart,transmit_buffer,18,10);
    }
    for(int i = 0; i < t.nw; i++)
    {
        memcpy(transmit_buffer+1,&t.w[i],4);
        HAL_UART_Transmit(uart,transmit_buffer,18,10);
    }
    return;
}

// Loads a tinn.
Tinn xtload(int nips,int nhid,int nops,float *weights,float *biases)
{
    // Build a new tinn.
    const Tinn t = xtbuild(nips, nhid, nops);
    // Load bias and weights.
    for(int i = 0; i < t.nb; i++) t.b[i]=biases[i];
    for(int i = 0; i < t.nw; i++) t.w[i]=weights[i];
    return t;
}

// Frees object from heap.
void xtfree(const Tinn t)
{
    free(t.w);
    free(t.b);
    free(t.h);
    free(t.o);
}
