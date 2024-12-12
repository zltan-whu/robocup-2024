#ifndef PID_H
#define PID_H

class PID
{
private:
    double Kp;  // 比例系数
    double Ki;  // 积分系数
    double Kd;  // 微分系数
    
    double previousError;  // 上一次的误差
    double integral;       // 积分

public:
    
    PID() : previousError(0.0), integral(0.0) {}
    ~PID(){};

    // 设置PID参数
    void setParameters(double Kp, double Ki, double Kd)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
    }

    // 计算PID修正值
    double calculate(double error)
    {
        double derivative = error - previousError;  // 微分
        integral += error;                            // 积分

        if(integral >= 3)
        {
            integral = 3;
        }
        else if (integral <= -3)
        {
            integral = -3;
        }
        
        
        double output = Kp * error + Ki * integral + Kd * derivative;
        
        previousError = error;
        //std::cout<<"previousErr:"<<previousError<<std::endl;


        return output;
    }

    // 重置PID（例如，当重新开始控制时）
    void reset()
    {
        previousError = 0.0;
        integral = 0.0;
    }
};

#endif // PID_H
