#include <iostream>
#include <deque>
#include <vector>
#include <cmath>

#include <Eigen/Dense>

namespace numeric
{
    template <typename T>
    class derivative
    {
        public:
            derivative(double timestep, T init):dt(timestep), old(init){}
            T forward_differences(T val, T val_old)
            {
                return (val-val_old)/dt;
            }

            T central_differences(T val_old, T val_new)
            {
                return (val_new - val_old)/(2*dt);
            }

            T operator () (T val)
            {
                if(x.size() < 3)
                {
                    x.push_back(val);
                    xd = forward_differences(val, old);
                    old = val;
                }
                else
                {
                    x.push_back(val);
                    x.pop_front();
                    xd = central_differences(x[0], x[2]);
                }
                return xd;
            }
        private:
            std::deque<T> x;
            T old;
            T xd;
            double dt;
    };

    template <typename T>
    class integral
    {
        public:
            integral(double timestep, T init) : dt(timestep), i(init), val_old(0) { }
            T operator () (T val)
            {
                i += dt/2 * (val + val_old);
                val_old = val;
                return i;
            }

        private:
            T i;
            T val_old;
            double dt;
    };
}

namespace filter
{

    template <typename T>
    class butter
    {
        public:
            // Default constructor necessary to store functor in map
            /* butter()
            {
                std::cerr << "Empty Filter was Constructed! Undefined behavior might occur!" << std::endl;
            } */
            butter(std::vector<double> a, std::vector<double> b, T zero):a(a),b(b), zero(zero)
            {
                inBuf = std::vector<T>(a.size(), zero);
                outBuf = std::vector<T>(b.size(), zero);
            };
            T operator () (T val)
            {
                inBuf.insert(inBuf.begin(), val);
                outBuf.insert(outBuf.begin(), zero);
                inBuf.pop_back();
                outBuf.pop_back();

                T tmp = zero;
                for(int k = 0; k < inBuf.size(); k++)
                {
                    tmp += b[k] * inBuf[k] - a[k] * outBuf[k];
                }
                outBuf.front() = tmp;

                return tmp;
            }
        private:
            T zero;
            std::vector<double> a;
            std::vector<double> b;
            std::vector<T> inBuf;
            std::vector<T> outBuf;
    };


    template <typename T>
    class spline
    {
        public:
            spline(T start, T end): start(start), end(end)
            {
                buf = std::vector<T>{start, start, end, end};
                a0 = - buf[0] + buf[1] - buf[2] + buf[3];
                a1 =   buf[0] - buf[1] - a0;
                a2 = - buf[0] + buf[2];
                a3 =   buf[1];

            }
            T operator () (double mu)
            {
                return pow(mu,3)*a0 + pow(mu,2)*a1 + mu*a2 + a3;
            }
        private:
            T start;
            T end;
            std::vector<T> buf;
            T a0;
            T a1;
            T a2;
            T a3;
    };

    class KalmanFilter
    {
        public:
            KalmanFilter(Eigen::MatrixXd A,
                         Eigen::MatrixXd B,
                         Eigen::MatrixXd C,
                         Eigen::MatrixXd Q,
                         Eigen::MatrixXd R)
            {
                this->A = A;
                this->B = B;
                this->C = C;
                this->Q = Q;
                this->R = R;

                this->I = Eigen::MatrixXd::Identity(A.rows(), A.cols());
                this->P = this->I;
                this->x_hat = Eigen::VectorXd::Zero(A.cols());
            }

            Eigen::VectorXd operator () (Eigen::VectorXd y, double u)
            {
                predict(u);
                update(y);
                return x_hat;
            }

        private:
            void predict(double u)
            {
                x_hat = A * x_hat + B * u;
                P = A * P * A.transpose() + Q;
            }

            void update(Eigen::VectorXd y)
            {
                auto K = (P * C.transpose()) * ((C * P * C.transpose()) + R).inverse(); 
                x_hat = x_hat + K * (y - (C * x_hat));
                P = (I - (K * C)) * P;
            }

            Eigen::VectorXd x_hat;
            Eigen::MatrixXd P;
            Eigen::MatrixXd A;
            Eigen::MatrixXd B;
            Eigen::MatrixXd C;
            Eigen::MatrixXd Q;
            Eigen::MatrixXd R;
            Eigen::MatrixXd I;
    };
}

