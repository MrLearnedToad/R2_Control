#include"math.h"
#include"Ann.h"
#include"stdlib.h"
#include "main.h"

uint8_t NNlearn=1;

double ANN_pid_run(ANN_PID_handle *handle,double target,double current_value)
{
    static float last_speed;
    for(int i=3;i>0;i--)
    {
        handle->error[i]=handle->error[i-1];
    }
    if(fabs(last_speed-current_value)>handle->pid_handle.Dead_Zone)
        handle->error[0]=(last_speed-current_value)/normalize_factor;
    else
        handle->error[0]=0;
    if(NNlearn==1)
        NN_bcak_prop(&handle->network,handle->error,ANN_learning_rate);//神经网络学习
    handle->input[0]=handle->error[0];
    handle->input[1]=handle->error[1];
    handle->input[2]=handle->error[2];
    if(NNlearn==1)
    {
        NN_fprop(&handle->network,handle->input);//神经网络正向运行
        handle->pid_handle.KP=handle->network.outputout[0]*multiply_factor_P;
        handle->pid_handle.KI=handle->network.outputout[1]*multiply_factor_I;
        handle->pid_handle.KD=handle->network.outputout[2]*multiply_factor_D;
    }
    last_speed=target;
    return Pid_Run(&handle->pid_handle,target,current_value)*multiply_factor_PID_out;
}

ANN_PID_handle* ANN_pid_init(ANN_PID_handle *handle)//在最开始首先执行这个函数
{
    NN_create_network(3,4,&handle->network);
    handle->input[0]=0;
    handle->input[1]=0;
    handle->input[2]=0;
    handle->pid_handle.I_Limit=10000;
    handle->pid_handle.Dead_Zone=1;
    handle->pid_handle.I_MAX=10000;
    handle->pid_handle.PID_MAX=100000;
    NN_fprop(&handle->network,handle->input);
    handle->pid_handle.KP=handle->network.outputout[0];
    handle->pid_handle.KI=handle->network.outputout[1];
    handle->pid_handle.KD=handle->network.outputout[2];
    return handle;
}

matrix* create_matrix(const int row,const int col)
{
    matrix* ret=(matrix*)malloc(sizeof(matrix));
    ret->col=col;
    ret->row=row;
    return ret;
}

void set_matrix(matrix *tar,int row,int col,double data)
{
    tar->mat[(tar->col+1)*row+col]=data;
    return;
}

double read_matrix(const matrix *tar,int row,int col)
{
    return tar->mat[(tar->col+1)*row+col];
}

matrix *copy_matrix(const matrix *tar)
{
    matrix *ret;
    ret=create_matrix(tar->row,tar->col);
    for (int i = 0; i < tar->row; i++)
    {
        for (int j = 0; j < tar->col; j++)
        {
            set_matrix(ret,i,j,read_matrix(tar,i,j));
        }
    }
    return ret;
}

void delete_matrix(matrix *tar)
{
    free(tar->mat);
    free(tar);
    return;
}

void rand_matrix(matrix *tar)
{
    for (int i = 0; i < tar->row; i++)
    {
        for (int j = 0; j < tar->col; j++)
        {
            set_matrix(tar,i,j,0.3);
        }
    }
    return;
}

double pd_tanh(double num)
{
    return (1-pow(tanh(num),2));
}

void NN_bcak_prop(NN_handler *network,double *output_error,double rate)
{
    double gradient,pd_pidout_nnout[3];

    pd_pidout_nnout[0]=output_error[1]-output_error[2];
    pd_pidout_nnout[1]=output_error[1];
    pd_pidout_nnout[2]=output_error[1]-2*output_error[2]+output_error[3];
    /*修正隐藏层到输出层系数*/
    for (int i = 0; i < network->hidden_num+1; i++)
    {
        for (int j = 0; j < 3; j++)
        {
                        
            gradient=-(-output_error[0]*1*pd_pidout_nnout[j]*pd_tanh(network->hiddensum[j])*network->hiddenout[i]);
            set_matrix(&network->hidden_2_output,i,j,read_matrix(&network->hidden_2_output,i,j)+gradient*rate);
        }
    }
    /*修正输入层到隐藏层系数*/
    for (int i = 0; i < network->input_num+1; i++)
    {
        for (int j = 0; j < network->hidden_num; j++)
        {
            gradient=-(-output_error[0]*1
            *(pd_pidout_nnout[0]*pd_tanh(network->hiddensum[0])*read_matrix(&network->hidden_2_output,j,0)
             +pd_pidout_nnout[1]*pd_tanh(network->hiddensum[1])*read_matrix(&network->hidden_2_output,j,1)
             +pd_pidout_nnout[2]*pd_tanh(network->hiddensum[2])*read_matrix(&network->hidden_2_output,j,2)
            )*0.5*pd_tanh(network->inputsum[j])*network->inputout[i]);
            set_matrix(&network->input_2_hidden,i,j,read_matrix(&network->hidden_2_output,i,j)+gradient*rate);
        }
    }
    return;
}


void NN_fprop(NN_handler *network,double *input)
{
    for (int i = 0; i < network->input_num; i++)
    {
        network->inputout[i]=input[i];
    }

    for (int i = 0; i < network->hidden_num; i++)
    {
        network->inputsum[i]=0;
        for (int j = 0; j < network->input_num+1; j++)
        {
            network->inputsum[i]+=read_matrix(&network->input_2_hidden,j,i)*network->inputout[j];
        }
        network->hiddenout[i]=tanh(network->inputsum[i]);
    }

    for (int i = 0; i < 3; i++)
    {
        network->hiddensum[i]=0;
        for (int j = 0; j < network->hidden_num+1; j++)
        {
            network->hiddensum[i]+=read_matrix(&network->hidden_2_output,j,i)*network->hiddenout[j];
        }
        network->outputout[i]=0.5f*(1+tanh(network->hiddensum[i]));
    }
    return;
}

NN_handler* NN_create_network(int input_num,int hidden_num,NN_handler *network)
{
    network->input_2_hidden.col=hidden_num;
    network->input_2_hidden.row=input_num+1;
    network->hidden_2_output.row=hidden_num+1;
    network->hidden_2_output.col=3;

    network->hidden_num=hidden_num;

    network->input_num=input_num;

    network->hiddenout[hidden_num]=1;
    network->inputout[input_num]=1;

    rand_matrix(&(network->hidden_2_output));
    rand_matrix(&(network->input_2_hidden));
    
    return network;
}

float Pid_Run(PID_T *Pid, float Target, float Feedback)
{
    Pid->Error_Last = Pid->Error;
    Pid->Error = Target - Feedback;

    Pid->P_OUT = Pid->KP * Pid->Error;
    if (ABS(Pid->Error) <= Pid->I_Limit)
    {
        Pid->I_OUT += Pid->KI * Pid->Error;
    }
    else
    {
        Pid->I_OUT = 0;
    }

    Pid->D_OUT = Pid->KD * (Pid->Error - Pid->Error_Last);

    Pid->I_OUT = CLAMP(Pid->I_OUT, -Pid->I_MAX, Pid->I_MAX);
    Pid->PID_OUT = Pid->P_OUT + Pid->I_OUT + Pid->D_OUT;
    Pid->PID_OUT = CLAMP(Pid->PID_OUT, -Pid->PID_MAX, Pid->PID_MAX);
    if (ABS(Pid->PID_OUT) < Pid->Dead_Zone)
    {
        Pid->PID_OUT = 0;
    }
    return Pid->PID_OUT;
}
