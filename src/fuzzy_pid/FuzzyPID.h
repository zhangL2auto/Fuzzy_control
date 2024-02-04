#ifndef FuzzyPID_H
#define FuzzyPID_H

class FuzzyPID
{
public:
    FuzzyPID();
    ~FuzzyPID();
    void get_grad_membership(float erro, float erro_c);
    float quantization(float maximum, float minimum, float x);
    float reverse_quantization(float maximum, float minimum, float qvalues);
    void get_sum_grad();
    void get_out();
    float fuzzy_pid_controller(float e_max, float e_min, float ec_max, 
            float ec_min, float kp_max, float kp_min, float erro, float erro_c,
            float ki_max, float ki_min, float kd_max, float kd_min,  float erro_pre,
            float erro_ppre);
    const int num_area = 8;

    float e_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3};
    float ec_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3};
    float kp_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3};
    float ki_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3};
    float kd_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3};
    float fuzzy_output_membership_values[7] = {-3, -2, -1, 0, 1, 2, 3};

    float kp;
    float ki;
    float kd;
    float area_delta_kp;
    float area_delta_ki;
    float area_delta_kd;
    float area_fuzzy_output;
    float delta_kp;
    float delta_ki;
    float delta_kd;
    float fuzzy_output;
    float area_erro;
    float area_erro_c;
    float erro_sum;
    float e_grad[2];
    float ec_grad[2];
    int e_grad_index[2];  //输入e隶属度在规则表的索引
    int ec_grad_index[2];
    float grad_sum[7] = {0, 0, 0, 0, 0, 0, 0};
    float kp_grad_sum[7] = {0, 0, 0, 0, 0, 0, 0};
    float ki_grad_sum[7] = {0, 0, 0, 0, 0, 0, 0};
    float kd_grad_sum[7] = {0, 0, 0, 0, 0, 0, 0};

    // 推理规则表
    // 论域隶属值
    int NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3; 
    // kp规则表
    int  Kp_rule_list[7][7] = { {PB,PB,PM,PM,PS,ZO,ZO},     
                                {PB,PB,PM,PS,PS,ZO,NS},
                                {PM,PM,PM,PS,ZO,NS,NS},
                                {PM,PM,PS,ZO,NS,NM,NM},
                                {PS,PS,ZO,NS,NS,NM,NM},
                                {PS,ZO,NS,NM,NM,NM,NB},
                                {ZO,ZO,NM,NM,NM,NB,NB} };
    // ki规则表
    int  Ki_rule_list[7][7] = { {NB,NB,NM,NM,NS,ZO,ZO},     
                                {NB,NB,NM,NS,NS,ZO,ZO},
                                {NB,NM,NS,NS,ZO,PS,PS},
                                {NM,NM,NS,ZO,PS,PM,PM},
                                {NM,NS,ZO,PS,PS,PM,PB},
                                {ZO,ZO,PS,PS,PM,PB,PB},
                                {ZO,ZO,PS,PM,PM,PB,PB} };
    // kd规则表
    int  Kd_rule_list[7][7] = { {PS,NS,NB,NB,NB,NM,PS},
                                {PS,NS,NB,NM,NM,NS,ZO},
                                {ZO,NS,NM,NM,NS,NS,ZO},
                                {ZO,NS,NS,NS,NS,NS,ZO},
                                {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
                                {PB,NS,PS,PS,PS,PS,PB},
                                {PB,PM,PM,PM,PS,PS,PB} };

    int  Fuzzy_rule_list[7][7] = { {PB,PB,PB,PB,PM,ZO,ZO},  
                                   {PB,PB,PB,PM,PM,ZO,ZO},
                                   {PB,PM,PM,PS,ZO,NS,NM},
                                   {PM,PM,PS,ZO,NS,NM,NM},
                                   {PS,PS,ZO,NM,NM,NM,NB},
                                   {ZO,ZO,ZO,NM,NB,NB,NB},
                                   {ZO,NS,NB,NB,NB,NB,NB}};

};
#endif