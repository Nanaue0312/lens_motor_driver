#ifndef __KALMAN_FLITER_HPP__
#define __KALMAN_FLITER_HPP__

#include <type_traits>

namespace utools::fliter
{
    /// @brief 1维卡尔曼滤波
    /// @tparam _InputType 输出值类型
    template <typename _InputType, typename = typename std::enable_if<std::is_floating_point<_InputType>::value>::type>
    // template <typename _InputType, typename = typename std::enable_if_t<std::is_integral_v<T>>>
    class Kalman1D
    {
    protected:
        _InputType _x;        // 状态
        _InputType _A{1};     // 先验状态估计 x(n)=A*x(n-1)+u(n),u(n)~N(0,q)
        _InputType _H{1};     // z(n)=H*x(n)+w(n),w(n)~N(0,r)
        _InputType _q{10e-6}; // 过程噪声协方差(预测),10e-6
        _InputType _r{10e-5}; // 测量噪声协方差,10e-5
        _InputType _p;        // 估计误差协方差
        _InputType _gain;

    public:
        /// @brief  请根据自己的要求更改 A,H,q,r
        /// @param init_x 初始x状态值
        /// @param init_p 初始估计误差协方差
        Kalman1D(const _InputType init_x, const _InputType init_p) : _x(init_x), _p(init_p)
        {
        }

        inline const _InputType &calc(const _InputType z_measure)
        {
            // 预测
            _x = _A * _x;
            _p = _A * _A * _p + _q; /* p(n|n-1)=A^2*p(n-1|n-1)+q */
            // 测量
            _gain = _p * _H / (_p * _H * _H + _r);
            _x = _x + _gain * (z_measure - _H * _x);
            _p = (1 - _gain * _H) * _p;
            return _x;
        }
    };

    /// @brief 2维卡尔曼滤波
    /// @tparam _InputType 输出值类型
    template <typename _InputType, typename = typename std::enable_if<std::is_floating_point<_InputType>::value>::type>
    // template <typename _InputType, typename = typename std::enable_if_t<std::is_integral_v<T>>>
    class Kalman2D
    {
    protected:
        _InputType _x[2];                      // 状态:[0]-角度[1]-角度差, 2x1
        _InputType _A[2][2]{{1, 0.1}, {0, 1}}; // X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2
        _InputType _H[2]{1, 0};                // Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2
        _InputType _q[2]{10e-7, 10e-7};        // 过程噪声协方差(预测),2x1 [q0,0; 0,q1]
        _InputType _r{10e-7};                  // 测量噪声协方差
        _InputType _p[2][2];                   // 估计误差协方差,2x2 [p0 p1; p2 p3]
        _InputType _gain[2];                   // 2x1

    public:
        /// @brief  请根据自己的要求更改 A,H,q,r
        /// @param init_x 初始x状态值
        /// @param init_p 初始估计误差协方差
        Kalman2D(const _InputType *init_x, const _InputType **init_p)
        {
            std::copy(init_x, init_x + sizeof(_InputType) * 1, _x);
            std::copy(init_p, init_p + sizeof(_InputType) * 2 * 2, _p);
        }

        inline const _InputType &calc(const _InputType z_measure)
        {
            /* 预测 */
            _x[0] = _A[0][0] * _x[0] + _A[0][1] * _x[1];
            _x[1] = _A[1][0] * _x[0] + _A[1][1] * _x[1];
            /* p(n|n-1)=A^2*p(n-1|n-1)+q */
            _p[0][0] = _A[0][0] * _p[0][0] + _A[0][1] * _p[1][0] + _q[0];
            _p[0][1] = _A[0][0] * _p[0][1] + _A[1][1] * _p[1][1];
            _p[1][0] = _A[1][0] * _p[0][0] + _A[0][1] * _p[1][0];
            _p[1][1] = _A[1][0] * _p[0][1] + _A[1][1] * _p[1][1] + _q[1];

            /* 测量 */
            /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T 意思是转置. */
            auto temp0 = _p[0][0] * _H[0] + _p[0][1] * _H[1];
            auto temp1 = _p[1][0] * _H[0] + _p[1][1] * _H[1];
            auto temp = _r + _H[0] * temp0 + _H[1] * temp1;
            _gain[0] = temp0 / temp;
            _gain[1] = temp1 / temp;
            /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
            temp = _H[0] * _x[0] + _H[1] * _x[1];
            _x[0] = _x[0] + _gain[0] * (z_measure - temp);
            _x[1] = _x[1] + _gain[1] * (z_measure - temp);

            /* 更新 p: p(n|n) = [I - gain * H] * p(n|n-1) */
            _p[0][0] = (1 - _gain[0] * _H[0]) * _p[0][0];
            _p[0][1] = (1 - _gain[0] * _H[1]) * _p[0][1];
            _p[1][0] = (1 - _gain[1] * _H[0]) * _p[1][0];
            _p[1][1] = (1 - _gain[1] * _H[1]) * _p[1][1];

            return _x[0];
        }
    };
}
#endif
