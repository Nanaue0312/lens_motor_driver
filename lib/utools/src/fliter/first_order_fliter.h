/// 一阶低通滤波器的离散形式通常使用如下的递推公式：
/// y[n]=α⋅x[n]+(1−α)⋅y[n−1]
/// 其中，y[n]是当前时刻的输出，x[n]是当前时刻的输入，y[n−1]是上一时刻的输出，α是滤波器的系数，取值范围为0到1。

#ifndef __UTOOLS_FIRST_ORDER_FLITER_H__
#define __UTOOLS_FIRST_ORDER_FLITER_H__

namespace utools::fliter
{
    template <typename _InputType>
    class FirstOrder
    {
    private:
        float __alpha{1};
        float __alpha_inv{0};
        _InputType __y{0};

    public:
        FirstOrder() = default;

        FirstOrder(float alpha, _InputType y = 0)
            : __alpha(alpha), __y(y)
        {
            __alpha_inv = 1 - __alpha;
        }

        virtual ~FirstOrder() = default;

        /// @brief 重新对历史值进行初始化操作
        /// @param y y值
        void reinit(_InputType y, float alpha = -1)
        {
            __y = y;
            if (alpha != -1)
            {
                set_alpha(alpha);
            }
        }

        /// @brief 重新设置滤波器系数alpha
        /// @param alpha alpha值
        void set_alpha(float alpha)
        {
            __alpha = alpha;
            __alpha_inv = 1 - __alpha;
        }

        /// @brief 获取当前滤波器系数alpha
        /// @return 当前滤波器系数alpha
        float get_alpha() const
        {
            return __alpha;
        }

        /// @brief 根据当前的值计算返回结果
        /// @tparam _OutputType 输出数据类型
        /// @param x 当前值
        /// @return 滤波后的值
        template <typename _OutputType>
        _OutputType calc(const _InputType &x)
        {
            __y = __alpha * x + __alpha_inv * __y;
            return static_cast<_OutputType>(__y);
        }

        /// @brief 根据当前的值计算返回结果
        /// @tparam _OutputType 输出数据类型
        /// @return 返回上一次滤波后的值
        template <typename _OutputType>
        _OutputType calc()
        {
            return static_cast<_OutputType>(__y);
        }
    };

} // namespace utools::fliter

#endif // __UTOOLS_FIRST_ORDER_FLITER_H__