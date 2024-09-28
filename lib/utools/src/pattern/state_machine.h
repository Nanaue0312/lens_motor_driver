#ifndef __UTOOLS_PATTERN_STATE_MACHINE_H__
#define __UTOOLS_PATTERN_STATE_MACHINE_H__

#include <functional>
#include <map>
#include <string>

namespace utools::pattern
{
    /// @brief 状态机的设计
    /// @tparam State 状态
    /// @tparam Event 事件
    /// @note 状态: 定义状态的类型。
    /// @note 事件: 定义状态机接收的事件。
    /// @note 状态表: 定义状态转换。
    /// @note 状态机类: 实现状态机的逻辑。
    template <typename State, typename Event>
    class StateMachine
    {
    public:
        using TransitionFunction = std::function<void()>;

        /// @brief 添加状态转换
        /// @param from 起始状态
        /// @param event 触发事件
        /// @param to 目标状态
        /// @param func 转换函数
        /// @return StateMachine& 自身引用
        StateMachine &add_transition(State from, Event event, State to, TransitionFunction func = nullptr)
        {
            __transitions[{from, event}] = {to, func};
        }

        /// @brief 设置初始状态
        /// @param state 初始状态
        /// @return StateMachine& 自身引用
        StateMachine &set_initial_state(State state)
        {
            __curr_state = state;
        }

        /// @brief 处理事件
        /// @param event 事件
        /// @return bool 是否成功处理事件
        bool handle_event(Event event)
        {
            auto it = __transitions.find({__curr_state, event});
            if (it != __transitions.end())
            {
                if (it->second.second)
                {
                    it->second.second(); // 执行状态转换的函数（如果有）
                }
                __curr_state = it->second.first; // 更新当前状态
                return true;
            }
            return false;
        }

        /// @brief 获取当前状态
        /// @return 当前状态
        State get_curr_state() const
        {
            return __curr_state;
        }

    private:
        State __curr_state; // 当前状态

        struct transition_t
        {
            State first;               // 目标状态
            TransitionFunction second; // 状态转换函数
        };

        std::map<std::pair<State, Event>, transition_t> __transitions; // 状态转换表
    };
}

#endif // __UTOOLS_PATTERN_STATE_MACHINE_H__
