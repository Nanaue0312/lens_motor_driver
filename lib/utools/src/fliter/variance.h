/// @brief 计算方差
#ifndef __VARIANCE_HPP__
#define __VARIANCE_HPP__

#include <vector>
#include <numeric>
#include <type_traits>

namespace utools::fliter
{
    template <typename _ValType,
              typename = std::enable_if<std::is_floating_point<_ValType>::value>>
    class Variance
    {
    private:
        std::vector<_ValType> __data;

    public:
        inline std::vector<_ValType> &data() const
        {
            return __data;
        }

        void push(const _ValType x, const _ValType y)
        {
            __data.push_back(std::make_pair<_ValType, _ValType>(x, y));
        }

        const _ValType calc() const
        {
            auto sum{std::accumulate(__data.begin(), __data.end(), 0)};
            float ave{sum / __data.size()};
            _ValType com{0};
            for (auto itm : __data)
            {
                com += pow(itm - ave, 2);
            }
            return com / __data.size();
        }

        void reint()
        {
            __data.clear();
        }
    };
}
#endif
