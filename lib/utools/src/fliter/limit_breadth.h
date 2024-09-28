
/*
A、名称：限幅滤波法（又称程序判断滤波法）
B、方法：
    根据经验判断，确定两次采样允许的最大偏差值（设为A），
    每次检测到新值时判断：
    如果本次值与上次值之差<=A，则本次值有效，
    如果本次值与上次值之差>A，则本次值无效，放弃本次值，用上次值代替本次值。
C、优点：
    能有效克服因偶然因素引起的脉冲干扰。
D、缺点：
    无法抑制那种周期性的干扰。
    平滑度差。
*/
#ifndef __LIMIT_BREADTH_FLITER_HPP__
#define __LIMIT_BREADTH_FLITER_HPP__
namespace utools::fliter
{
    template <class _InputType>
    class LimitBreadth
    {
    protected:
        _InputType _upper_limit_val{0}; // 判断上限
        _InputType _lower_limit_val{0}; // 判断下限
        _InputType _prev_input_val{0};  // 前序输入值
    public:
        /// @brief 只有初始化的构造
        /// @param init_val 初始化值
        LimitBreadth(_InputType init_val = 0) : _prev_input_val(init_val)
        {
        }

        /// @brief 设置范围的构造
        /// @param lower_limit_val 上限值
        /// @param upper_limit_val 下限值
        /// @param init_val 初始化值
        LimitBreadth(_InputType lower_limit_val, _InputType upper_limit_val, _InputType init_val = 0) : _upper_limit_val(upper_limit_val), _lower_limit_val(lower_limit_val), _prev_input_val(init_val)
        {
        }

        /// @brief 设置上下限范围
        /// @param lower_limit_val 上限值
        /// @param upper_limit_val 下限值
        void set_limit_range(_InputType lower_limit_val, _InputType upper_limit_val)
        {
            _upper_limit_val = upper_limit_val;
            _lower_limit_val = lower_limit_val;
        }

        /// @brief 重新初始化缓存
        /// @param init_val 初始化值
        void reinit(_InputType init_val)
        {
            _prev_input_val = init_val;
        }

        /// @brief 进行滤波输出
        /// @param val 输入值
        /// @return 判断后输出相应的值
        const _InputType &calc(_InputType val)
        {
            if ((val <= _upper_limit_val) && (val >= _lower_limit_val))
            {
                _prev_input_val = val;
            }
            return _prev_input_val;
        }
    };
}
#endif
