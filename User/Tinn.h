#pragma once
#include "main.h"

/*������������㷨 1.0
�޸���https://github.com/glouw/tinn��Ŀ
�޸����ݣ����ලѧϰ�޸�Ϊ�޼ල����ǿѧϰ
          �޸�������㼤�����ʹ����ݿ�������;
          ɾ��������Windows���������ϵͳ�������˴������
          ��дʹ��˵��
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

float* xtpredict(Tinn, const float* in);//��������������

float xttrain(Tinn, const float* in, const float* const ro, const float* tg, float rate);//����ʵ����������ݽ���ѧϰ

Tinn xtbuild(int nips, int nhid, int nops);//�����µ�������

void xtsave(Tinn,UART_HandleTypeDef *uart);//������������ô��ڴ��

Tinn xtload(int nips,int nhid,int nops,float *weights,float *biases);//�������е�������

void xtfree(Tinn);//ɾ��������

