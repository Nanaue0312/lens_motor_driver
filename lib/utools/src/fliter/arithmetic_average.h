/*
A、名称：算术平均滤波法
B、方法：
    连续取N个采样值进行算术平均运算：
    N值较大时：信号平滑度较高，但灵敏度较低；
    N值较小时：信号平滑度较低，但灵敏度较高；
    N值的选取：一般流量，N=12；压力：N=4。
C、优点：
    适用于对一般具有随机干扰的信号进行滤波；
    这种信号的特点是有一个平均值，信号在某一数值范围附近上下波动。
D、缺点：
    对于测量速度较慢或要求数据计算速度较快的实时控制不适用；
    比较浪费RAM。
*/

#ifndef __ARITHMETIC_AVERAGE_FLITER_HPP__
#define __ARITHMETIC_AVERAGE_FLITER_HPP__

#include <array>
#include <numeric>

namespace utools::fliter
{

    /// @brief 递推平均滤波法
    /// @tparam _InputType 输出值的数据类型
    /// @tparam _BufSize 缓存的大小
    template <class _InputType, unsigned short _BufSize>
    class ArithmeticAverage
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
        ArithmeticAverage(_InputType init_val = 0) : _init_val(init_val)
        {
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
                _prev_sum = std::accumulate(this->_buf.begin(), this->_buf.end(), 0); // 存在超出范围的风险
                if (reinitable)
                    reinit(_init_val); // 重新初始化
            }
            return static_cast<_ReturnType>(static_cast<_ReturnType>(_prev_sum) / _BufSize);
        }

        /// @brief 写入数据
        /// @param val 数据值
        /// @return 如果已经是最后一个数据了，返回true，反之返回false，当为true时，可以去取出数据
        inline const bool push(_InputType val)
        {
            if (!_can_calc())
                this->_buf.at(++this->_buf_pos % _BufSize) = val;
            return _can_calc();
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