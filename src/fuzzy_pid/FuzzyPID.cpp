/*
 * @Author: zhangL2auto
 * @Date: 2024-02-02 10:49:00
 * @LastEditors: 2auto deutschlei47@126.com
 * @LastEditTime: 2024-02-03 14:10:21
 */
#include "FuzzyPID.h"

FuzzyPID::FuzzyPID()
{
    kp = 0;
    ki = 0;
    kd = 0;
    fuzzy_output = 0;
    area_delta_kd = 0;
    area_delta_ki = 0;
    area_delta_kp = 0;
    area_fuzzy_output = 0;
    erro_sum = 0;
}
FuzzyPID::~FuzzyPID(){}

// 计算e de的隶属度函数
void FuzzyPID::get_grad_membership(float erro, float erro_c)
{
    if (erro > e_membership_values[0] && erro < e_membership_values[6])
        {
            for (int i = 0; i < num_area - 2; i++)
            {
                if  (erro >= e_membership_values[i] && erro <= e_membership_values[i + 1])
                {
                    e_grad[0] = -(erro - e_membership_values[i + 1]) / (e_membership_values[i + 1] - e_membership_values[i]);
                    e_grad[1] = 1 - e_grad[0];
                    e_grad_index[0] = i;
                    e_grad_index[1] = i + 1;
                    break;
                }
            }

        }
    else
    {
        if (erro <= e_membership_values[0])
        {
            e_grad[0] = 1;
            e_grad[1] = 0;
            e_grad_index[0] = 0;
            e_grad_index[1] = -1;
        }
        else if (erro >= e_membership_values[6])
        {
            e_grad[0] = 1;
            e_grad[1] = 0;
            e_grad_index[0] = 6;
            e_grad_index[1] = -1;            
        }
    }
    if (erro_c > ec_membership_values[0] && erro_c < ec_membership_values[6])
        {
            for (int i = 0; i < num_area - 2; i++)
            {
                if  (erro_c >= ec_membership_values[i] && erro_c <= ec_membership_values[i + 1])
                {
                    ec_grad[0] = -(erro_c - ec_membership_values[i + 1]) / (ec_membership_values[i + 1] - ec_membership_values[i]);
                    ec_grad[1] = 1 - ec_grad[0];
                    ec_grad_index[0] = i;
                    ec_grad_index[1] = i + 1;
                    break;
                }
            }

        }
    else
    {
        if (erro_c <= ec_membership_values[0])
        {
            ec_grad[0] = 1;
            ec_grad[1] = 0;
            ec_grad_index[0] = 0;
            ec_grad_index[1] = -1;
        }
        else if (erro_c >= ec_membership_values[6])
        {
            ec_grad[0] = 1;
            ec_grad[1] = 0;
            ec_grad_index[0] = 6;
            ec_grad_index[1] = -1;            
        }
    }
    
}

// 获得输出增量kp, ki, kd的隶属度
void FuzzyPID::get_sum_grad()
{
    for (int i = 0; i <= num_area - 1; i++)
    {
        kp_grad_sum[i] = 0;
        ki_grad_sum[i] = 0;
        kd_grad_sum[i] = 0;
    }
    for (int i = 0; i < 2; i++)
    {
        if (e_grad_index[i] == -1)
        {
            continue;
        }
        for (int j = 0; j < 2; j++)
        {
            if (ec_grad_index[j] != -1)
            {
                int index_kp = Kp_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
                int index_ki = Ki_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
                int index_kd = Kd_rule_list[e_grad_index[i]][ec_grad_index[j]] + 3;
                kp_grad_sum[index_kp] = kp_grad_sum[index_kp] + (e_grad[i] * ec_grad[j]);
                ki_grad_sum[index_ki] = ki_grad_sum[index_ki] + (e_grad[i] * ec_grad[j]);
                kd_grad_sum[index_kd] = kd_grad_sum[index_kd] + (e_grad[i] * ec_grad[j]);
            }
            else
            {
                continue;
            }
        }
    }
}

// 计算输出增量kp, ki, kd对应的论域值，初步的解模糊
void FuzzyPID::get_out()
{
    for (int i = 0; i < num_area - 1; i++)
    {
        area_delta_kp += kp_membership_values[i] * kp_grad_sum[i];
        area_delta_ki += ki_membership_values[i] * ki_grad_sum[i];
        area_delta_kd += kd_membership_values[i] * kd_grad_sum[i];
    }
}
// 区间映射
float FuzzyPID::quantization(float maximum, float minimum, float x)
{
    return 6.0 * (x - minimum)/(maximum - minimum) - 3;
}
// 反区间映射

float FuzzyPID::reverse_quantization(float maximum, float minimum, float qvalues)
{
    return (qvalues + 3) * (maximum - minimum) / 6.0 + minimum;
}

// 模糊PID控制函数
float FuzzyPID::fuzzy_pid_controller(float e_max, float e_min, float ec_max, 
        float ec_min, float kp_max, float kp_min, float erro, float erro_c,
        float ki_max, float ki_min, float kd_max, float kd_min,  float erro_pre,
        float erro_ppre)
{
    erro_sum += erro;
    area_erro = quantization(e_max, e_min, erro);
    area_erro_c = quantization(ec_max, ec_min, erro_c);
    get_grad_membership(area_erro, area_erro_c);
    get_sum_grad();
    get_out();
    delta_kp = reverse_quantization(kp_max, kp_min, area_delta_kp);
    delta_ki = reverse_quantization(ki_max, ki_min, area_delta_ki);
    delta_kd = reverse_quantization(kd_max, kd_min, area_delta_kd);
    area_delta_kp = 0, area_delta_ki = 0, area_delta_kd = 0;
    kp += delta_kp;
    ki += delta_ki;
    kd += delta_kd;
    std::cout << "Kp: " << kp << std::endl;
    kp = (kp < 0) ? 0 : kp;
    ki = (ki < 0) ? 0 : ki;
    kd = (kd < 0) ? 0 : kd;
    delta_kp = 0, delta_ki = 0, delta_kd = 0;
    float pid_output = kp * (erro - erro_pre) + ki * erro + kd * (erro - 2 * erro_pre + erro_ppre);
    return pid_output; 
}