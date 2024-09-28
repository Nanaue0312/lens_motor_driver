
/*
A、名称：递推平均滤波法（又称滑动平均滤波法）
B、方法：
    把连续取得的N个采样值看成一个队列，队列的长度固定为N，
    每次采样到一个新数据放入队尾，并扔掉原来队首的一次数据（先进先出原则），
    把队列中的N个数据进行算术平均运算，获得新的滤波结果。
    N值的选取：流量，N=12；压力，N=4；液面，N=4-12；温度，N=1-4。
C、优点：
    对周期性干扰有良好的抑制作用，平滑度高；
    适用于高频振荡的系统。
D、缺点：
    灵敏度低，对偶然出现的脉冲性干扰的抑制作用较差；
    不易消除由于脉冲干扰所引起的采样值偏差；
    不适用于脉冲干扰比较严重的场合；
    比较浪费RAM。
*/
#ifndef __MOVING_AVERAGE_FLITER_HPP__
#define __MOVING_AVERAGE_FLITER_HPP__

#include <array>
#include <numeric>

namespace utools::fliter
{
    /// @brief 递推平均滤波法
    /// @tparam _InputType 输出值的数据类型
    /// @tparam _BufSize 缓存的大小
    template <class _InputType, unsigned short _BufSize>
    class MovingAverage
    {
    protected:
        std::array<_InputType, static_cast<size_t>(_BufSize)> _buf; // 数据缓存
        unsigned short _buf_pos{0};                                 // 当前操作数据的位置

    public:
        MovingAverage(_InputType init_val = 0)
        {
            this->reinit(init_val);
        }

        /// @brief 写入数据
        /// @param val 数据值
        inline void push(_InputType val)
        {
            this->_buf.at(++this->_buf_pos % _BufSize) = val;
        }

        /// @brief 读取当前的滤波结果
        /// @tparam _Tr 指定返回值的类型
        /// @return 滤波结果
        template <class _Tre>
        inline const _Tre calc()
        {
            return static_cast<_Tre>(static_cast<_Tre>(std::accumulate(this->_buf.begin(), this->_buf.end(), 0)) / _BufSize);
        }

        /// @brief 读取当前的滤波结果
        /// @return 滤波结果
        inline const _InputType calc()
        {
            return this->calc<_InputType>();
        }

        /// @brief 增加新的数据，并返回计算结果
        /// @tparam _Tre 返回值类型
        /// @param val 输入值
        /// @return 滤波结果
        template <class _Tre>
        inline const _Tre calc(_InputType val)
        {
            this->push(val);
            return this->calc<_Tre>();
        }

        /// @brief 增加新的数据，并返回计算结果，计算结果类型与输出数据一样
        /// @param val 输入值
        /// @return 滤波结果
        const _InputType calc(_InputType val)
        {
            return this->calc<_InputType>(val);
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

#endif // __MOVING_AVERAGE_FLITER__
