#pragma once
#include "main.h"

/*简单神经网络控制算法 1.0
修改自https://github.com/glouw/tinn项目
修改内容：将监督学习修改为无监督的增强学习
          修改了输出层激活函数，使其兼容控制类用途
          删除了用于Windows的输入输出系统，增加了串口输出
          编写使用说明
*/


typedef struct
{
    // All the weights.
    float* w;
    // Hidden to output layer weights.
    float* x;
    // Biases.
    float* b;
    // Hidden layer.
    float* h;
    // Output layer.
    float* o;
    // Number of biases - always two - Tinn only supports a single hidden layer.
    int nb;
    // Number of weights.
    int nw;
    // Number of inputs.
    int nips;
    // Number of hidden neurons.
    int nhid;
    // Number of outputs.
    int nops;
}
Tinn;

float* xtpredict(Tinn, const float* in);//正向运行神经网络

float xttrain(Tinn, const float* in, const float* const ro, const float* tg, float rate);//根据实际输出的数据进行学习

Tinn xtbuild(int nips, int nhid, int nops);//创建新的神经网络

void xtsave(Tinn,UART_HandleTypeDef *uart);//将神经网络参数用串口打出

Tinn xtload(int nips,int nhid,int nops,float *weights,float *biases);//加载已有的神经网络

void xtfree(Tinn);//删除神经网络

