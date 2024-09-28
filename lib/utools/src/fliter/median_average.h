/*
A、名称：中位值平均滤波法（又称防脉冲干扰平均滤波法）
B、方法：
    采一组队列去掉最大值和最小值后取平均值，
    相当于“中位值滤波法”+“算术平均滤波法”。
    连续采样N个数据，去掉一个最大值和一个最小值，
    然后计算N-2个数据的算术平均值。
    N值的选取：3-14。
C、优点：
    融合了“中位值滤波法”+“算术平均滤波法”两种滤波法的优点。
    对于偶然出现的脉冲性干扰，可消除由其所引起的采样值偏差。
    对周期干扰有良好的抑制作用。
    平滑度高，适于高频振荡的系统。
D、缺点：
    计算速度较慢，和算术平均滤波法一样。
    比较浪费RAM。
*/
#ifndef __MEDIAN_AVERAGE_FLITER_HPP__
#define __MEDIAN_AVERAGE_FLITER_HPP__

#include <array>
#include <numeric>

namespace utools::fliter
{
    /// @brief 中位值平均滤波法
    /// @tparam _InputType 输出值的数据类型
    /// @tparam _BufSize 缓存的大小
    template <class _InputType, unsigned short _BufSize>
    class MedianAverage
    {
    protected:
        std::array<_InputType, static_cast<size_t>(_BufSize)> _buf; // 数据缓存
        unsigned short _buf_pos{0};                                 // 当前操作数据的位置
        _InputType _init_val{0};                                    // 默认的初始值
        _InputType _prev_sum{0};                                    // 记录上一次的计算结果

        inline const bool _can_calc() const
        {
            return _buf_pos == _BufSize - 1;
        }

    public:
        MedianAverage(_InputType init_val = 0)
        {
            reinit(init_val);
        }

        /// @brief 写入数据
        /// @param val 数据值
        inline const bool push(_InputType val)
        {
            if (!_can_calc())
                this->_buf.at(++this->_buf_pos % _BufSize) = val;
            return _can_calc();
        }

        /// @brief 读取当前的滤波结果
        /// @tparam _Tr 指定返回值的类型
        /// @param reinitable 是否需要重新初始化
        /// @return 滤波结果
        template <class _ReturnType>
        inline const _ReturnType calc(bool reinitable = true)
        {
            if (_can_calc())
            {
                std::sort(_buf.begin(), _buf.end(), std::less<_InputType>());
                _prev_sum = std::accumulate(_buf.begin() + 1, _buf.end() - 1, 0); // 存在超出范围的风险
                if (reinitable)
                    reinit(_init_val); // 重新初始化
            }
            return static_cast<_ReturnType>(static_cast<_ReturnType>(_prev_sum) / (_BufSize - 2));
        }

        /// @brief 重新初始化缓存
        /// @param init_val 初始化值
        void reinit(_InputType init_val = 0)
        {
            this->_buf.fill(init_val); // 初始化数组
            this->_buf_pos = 0;
        }
    };
}
#endif
