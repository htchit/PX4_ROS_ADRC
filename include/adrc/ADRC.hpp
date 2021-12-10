#include <stdint.h>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#define ABS(X)  (((X)>0)?(X):-(X))

typedef struct ADRC_Parameter
{
    float r;
    float h;
    float N;
    float omega;
    float b0;
    float k_p;
    float k_d;
};

class ADRC_CONTROLLER
{
    public:
        void Init_ADRC(ADRC_Parameter param)
        {
            this->controller_param.r=param.r;
            this->controller_param.h=param.h;
            this->controller_param.N=param.N;
            this->controller_param.omega=param.beta_omega;
            this->controller_param.b0=param.b0;
            this->controller_param.k_p=param.k_p;
            this->controller_param.k_d=param.k_d;
            this->beta_01 = 3*controller_param.omega;
            this->beta_02 = 3*controller_param.omega*controller_param.omega;
            this->beta_03 = controller_param.omega*controller_param.omega;
        }

        float ADRC_run(float exp,float fb)
        {
            this->ADRC_Control(exp,fb);
            return this->u;
        }

    private:

        ADRC_Parameter controller_param;

         /*****安排过度过程*******/
        float x1;    //跟踪微分期状态量
        float x2;    //跟踪微分期状态量微分项
        float h0;
        float fh;//最速微分加速度跟踪量

        /*****扩张状态观测器*******/
        /******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
        float z1;
        float z2;
        float z3; //根据控制对象输入与输出，提取的扰动信息
        float e;  //系统状态误差
        float y;  //系统输出量
        float fe1;
        float fe2;

        float beta_01;
        float beta_02;
        float beta_03;

        /**********系统状态误差反馈率*********/
        float e0; //状态误差积分项
        float e1; //状态偏差
        float e2; //状态量微分项
        float u0; //非线性组合系统输出
        float u;  //带扰动补偿后的输出

        /*********第二种组合形式*********/
        /*
        float alpha1; //u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
        float alpha2; //0<alpha1<1<alpha2
        */
        
        /*********第三种组合形式*********/
        float h1;    //u0=-fhan(e1,e2,r,h1);

        int Sign_ADRC(float Input)
        {
            int output = 0;
            if (Input>1E-6) output = 1;
            else if (Input<-1E-6) output = -1;
            else output = 0;
            return output;
        }

        int Fsg_ADRC(float x, float d)
        {
            int output = 0;
            output = (Sign_ADRC(x + d) - Sign_ADRC(x - d)) / 2;
            return output;
        }

        //TD
        void Fhan_ADRC(float expect_ADRC)//安排ADRC过度过程
        {
            float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0;
            float x1_delta = 0;//ADRC状态跟踪误差项

            x1_delta = this->x1 - expect_ADRC;//用x1-v(k)替代x1得到离散更新公式

            this->h0 = this->controller_param.N*controller_param.h;//用h0替代h，解决最速跟踪微分器速度超调问题

            d =controller_param.r*this->h0*this->h0;//d=rh^2;

            a0 = this->h0*this->x2;//a0=h*x2

            y = x1_delta + a0;//y=x1+a0

            a1 = sqrt(d*(d + 8 * ABS(y)));//a1=sqrt(d*(d+8*ABS(y))])

            a2 = a0 + Sign_ADRC(y)*(a1 - d) / 2;//a2=a0+sign(y)*(a1-d)/2;

            a = (a0 + y)*Fsg_ADRC(y, d) + a2*(1 - Fsg_ADRC(y, d));

            this->fh = -controller_param.r * (a / d) * Fsg_ADRC(a, d)-controller_param.r*Sign_ADRC(a)*(1 - Fsg_ADRC(a, d));//得到最速微分加速度跟踪量

            this->x1 +=controller_param.h*this->x2;//跟新最速跟踪状态量x1

            this->x2 +=controller_param.h*this->fh;//跟新最速跟踪状态量微分x2
        }

        //原点附近有连线性段的连续幂次函数
        float Fal_ADRC(float e, float alpha, float zeta)
        {
            int s = 0;
            float fal_output = 0;
            s = (Sign_ADRC(e + zeta) - Sign_ADRC(e - zeta)) / 2;
            fal_output = e*s / (powf(zeta, 1 - alpha)) + powf(ABS(e), alpha)*Sign_ADRC(e)*(1 - s);
            return fal_output;
        }

        //ESO
        void ESO_ADRC()
        {
            this->e = this->z1 - this->y;//状态误差

            this->fe1 = Fal_ADRC(this->e, 0.5, controller_param.h);//非线性函数，提取跟踪状态与当前状态误差
            this->fe2 = Fal_ADRC(this->e, 0.25, controller_param.h);

            /*************扩展状态量更新**********/
            this->z1 = controller_param.h * (this->z2 - this->beta_01 * this->e);
            this->z2 = controller_param.h * (this->z3 - this->beta_02 * this->fe + controller_param.b0 * this->u);
            this->z3 +=controller_param.h * (-this->beta_03 * this->fe2);
        }

        float Constrain_Float(float amt, float low, float high) 
        {
            if (amt>high)
            {
                return high;
            }
            else if (amt<low)
            {
                return low;
            }
            else
            {
                return amt;
            }
            
        }
        
        /*
        void Nolinear_Conbination_ADRC(void)
        {
            float temp_e2 = 0;
            temp_e2 = Constrain_Float(this->e2, -20, 20);
            this->u0 =controller_param.k_p * Fal_ADRC(this->e1 ,controller_param.alpha1 ,controller_param.zeta) +controller_param.k_d * Fal_ADRC(temp_e2,controller_param.alpha2,controller_param.zeta);
        }
        */

        /*-- ADRC 控制的完整流程 --*/
        float ADRC_Control( float expect_ADRC, float feedback_ADRC)
        {

            Fhan_ADRC(expect_ADRC);//TD

            this->y = feedback_ADRC;

            ESO_ADRC();//eso

            this->e1 = this->x1 - this->z1;//状态偏差项
            this->e2 = this->x2 - this->z2;//状态微分项，

            this->u0 =controller_param.k_1*this->e1+controller_param.k_2*this->e2;
            //Nolinear_Conbination_ADRC();

            this->u=this->u0-this->z3/controller_param.b0;

            this->u = Constrain_Float(this->u, -10, 10);

            return this->u;
        }
};