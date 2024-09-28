#ifndef __RING_BUFFER_HPP__
#define __RING_BUFFER_HPP__

#include <memory>
#include <cstring>
#include <vector>

namespace utools::mem
{
    template <typename _DataType>
    class RingBuffer
    {
    public:
        /// @brief 一段数据和内容，用于按照开始和结束符号查找的返回值
        using span_data_t = struct span_data_t
        {
            int head{-1}; // 没有找到头
            int len{-1};  // 没有打到结尾
        };

        typedef _DataType value_type;
        typedef typename std::vector<value_type>::size_type size_type;
        typedef _DataType &reference;
        typedef const _DataType &const_reference;
        typedef _DataType *pointer;

    private:
        value_type *__buf{nullptr}; // 缓存区
        size_type __capacity{2};    // 缓存容量
        volatile size_type __tail{0};
        volatile size_type __head{0};

        size_type __size_end_pos{1}; // 数据总长度-1，__size-1

        /// @brief 读取当前缓存指针所在位置
        /// @param pos 需要读取的值
        /// @return 在缓存中的位置
        inline const size_type __mod(size_type pos) const
        {
            return (pos & __size_end_pos);
        }

        inline const size_type __head_pos() const
        {
            return __mod(__head);
        }

        inline const size_type __tail_pos() const
        {
            return __mod(__tail);
        }

        /// @brief 计算为2的平方数
        /// @param num 需要判读的值
        /// @return 返回是否成功
        inline const int __is_power_of_two(size_type num) const
        {
            if (num < 2)
            {
                return 0;
            }
            return (num & (num - 1)) == 0;
        }

        /// @brief 寻找最接近参数的2的平方数
        /// @param num 需要处理的值
        /// @return 2的平方数
        inline size_type __roundup_power_of_two(size_type num)
        {
            if (num == 0)
            {
                return 2;
            }
            int i = 0;
            for (; num != 0; ++i)
            {
                num >>= 1;
            }
            return 1U << i;
        }

    public:
        /// @brief 读取当前的缓存区域容量
        /// @return 数据最大长度，如果需要得到相关的字节数量，需要再乘以数据类型对应的字节长度
        inline const size_type capacity() const
        {
            return __capacity;
        }

        /// @brief 判断缓存是否为空
        /// @return 是否为空
        inline const bool is_empty() const
        {
            return __head == __tail;
        }

        /// @brief 判断缓存是否已经满了
        /// @return 是否已满
        inline const bool is_full() const
        {
            return __capacity == (__tail - __head);
        }

        /// @brief 查询当前的数据长度
        /// @return 数据长度
        inline const size_type len() const
        {
            return __tail - __head;
        }

        /// @brief 查询当前的数据长度
        /// @return 数据长度
        inline const size_type size() const
        {
            return __tail - __head;
        }

        /// @brief 查询可以使用的缓存长度
        /// @return 可用缓存长度
        inline const size_type remain() const
        {
            return __capacity - __tail + __head;
        }

        /// @brief 向缓存的尾部增加数据
        /// @param data 数据源地址
        /// @param sz 数据长度
        /// @return 是否成功
        const bool push_back(std::vector<value_type> &data, const size_type sz)
        {
            if (sz > remain())
            {
                return false;
            }

            size_type i = std::min(sz, __capacity - __tail_pos());

            std::copy(data.begin(), data.begin() + i, __buf + __head_pos());
            std::copy(data.begin() + i, data.begin() + sz, __buf + __head_pos() + i);

            __tail += sz;

            return true;
        }

        /// @brief 从head开始弹出指定长度的数据，会删除被读取出来的数据
        /// @param data 数据接收地址
        /// @param sz 需要弹出的数据长度
        /// @return 返回实际弹出的数据长度，如果为0，则表示没有数据可弹出
        const size_type pop_head(value_type *data, size_type sz)
        {
            if (is_empty())
            {
                return 0;
            }
            if (sz > len())
            {
                sz = len();
            }

            size_type i = std::min(sz, __capacity - __head_pos());
            memcpy(data, __buf + __head_pos(), i);
            memcpy(data + i, __buf, sz - i);

            __head += sz;
            return sz;
        }

        /// @brief 从head开始弹出指定长度的数据，会删除被读取出来的数据
        /// @param vec_data 接收数据的vector，会自动扩容
        /// @param sz 需要弹出的数据长度
        /// @return 返回实际弹出的数据长度，如果为0，则表示没有数据可弹出
        const size_type pop_head(std::vector<value_type> &vec_data, size_type sz)
        {
            if (is_empty())
            {
                return 0;
            }
            if (sz > len())
            {
                sz = len();
            }

            size_type i = std::min(sz, __capacity - __head_pos());

            std::copy(__buf + __head_pos(), __buf + __head_pos() + i, vec_data.end());
            std::copy(__buf, __buf + sz - i, vec_data.end());

            __head += sz;
            return sz;
        }

        /// @brief 从head开始弹出指定长度的数据，不会删除被读取出来的数据
        /// @param sz 需要弹出的数据长度
        /// @return 返回实际弹出的数据长度，如果为0，则表示没有数据可弹出
        std::vector<value_type> pop_head(size_type sz)
        {
            if (is_empty())
            {
                return std::vector<value_type>();
            }
            if (sz > len())
            {
                sz = len();
            }

            std::vector<value_type> vec_data;

            size_type i = std::min(sz, __capacity - __head_pos());

            std::copy(__buf + __head_pos(), __buf + __head_pos() + i, vec_data.end());
            std::copy(__buf, __buf + sz - i, vec_data.end());

            __head += sz;
            return vec_data;
        }

        /// @brief 从head开始读取1个数据，不会删除数据
        /// @param pos 需要读取数据的起始位置
        /// @return 返回读取的数据，如果为空，则表示没有数据可读取
        value_type read(size_type pos)
        {
            return __buf[__mod(pos + __head)];
        }

        /// @brief 从head开始读取sz个数据，不会删除数据
        /// @param start_pos 需要读取数据的起始位置
        /// @param sz 需要读取的数据个数
        /// @return 返回读取的数据，如果为空，则表示没有数据可读取
        std::vector<value_type> read(size_type start_pos, size_type sz)
        {
            if (!sz || is_empty())
            {
                return std::vector<value_type>();
            }
            if (sz > len() - start_pos)
            {
                sz = len() - start_pos;
            }

            std::vector<value_type> vec_data;
            vec_data.reserve(sz);

            auto tmp_head_pos = __mod(__head + start_pos);
            size_type i = std::min(sz, __capacity - tmp_head_pos);
            std::copy(__buf + tmp_head_pos, __buf + tmp_head_pos + i, vec_data.end());
            std::copy(__buf, __buf + sz - i, vec_data.end());

            return vec_data;
        }

        /// @brief 删除指定的数据
        /// @param sz 数据长度，从head开始计算
        /// @return 删除数据的长度
        const size_type drain(size_type sz)
        {
            if (sz > len())
            {
                sz = len();
            }
            __head += sz;
            return sz;
        }

        /// @brief 查找指定的元素所在的位置
        /// @param sep 查找数据地址
        /// @param seplen 需要查找数据长度
        /// @param offset 开始位置的偏移量
        /// @return 开始的位置，-1表示没有查找到
        inline const int search(const value_type *sep, const size_type seplen, size_type offset = 0) const
        {
            if (!sep || len() < seplen + offset)
            {
                return -1;
            }

            size_type sep_pos{0};
            for (size_type curr_head{__head + offset}, temp_head;
                 curr_head < __tail;
                 ++curr_head)
            {
                temp_head = curr_head;

                for (sep_pos = 0; sep_pos < seplen && temp_head < __tail; ++sep_pos)
                {
                    if (__buf[__mod(temp_head)] != sep[sep_pos])
                        break;
                    ++temp_head;
                }

                if (sep_pos == seplen)
                {
                    return temp_head - __head - seplen; // 转换成从0开始的引用
                }
            }
            return -1;
        }

        /// @brief 查找指定的元素所在的位置，单个数据时
        /// @param sep 查找数据地址
        /// @param offset 开始位置的偏移量
        /// @return 开始的位置，-1表示没有查找到
        inline const int search(const value_type sep, size_type offset = 0) const
        {
            if (len() < 1 + offset)
            {
                return -1;
            }
            for (size_type curr_head{__head + offset};
                 curr_head < __tail;
                 ++curr_head)
            {
                if (__buf[__mod(curr_head)] == sep)
                {
                    return curr_head - __head; // 转换成从0开始的引用
                }
            }
            return -1;
        }

        /// @brief 查找指定的元素所在的位置
        /// @param sep 查找数据列表
        /// @param offset 开始位置的偏移量
        /// @return 开始的位置，-1表示没有查找到
        inline const int search(std::initializer_list<value_type> &&sep, size_type offset = 0) const
        {
            auto seplen = sep.size();
            if (seplen == 0 || len() < seplen + offset)
            {
                return -1;
            }

            size_type sep_pos{0};
            for (size_type curr_head{__head + offset}, temp_head;
                 curr_head < __tail;
                 ++curr_head)
            {
                temp_head = curr_head;

                for (sep_pos = 0; sep_pos < seplen && temp_head < __tail; ++sep_pos)
                {
                    if (__buf[__mod(temp_head)] != *(sep.begin() + sep_pos))
                        break;
                    ++temp_head;
                }

                if (sep_pos == seplen)
                {
                    return temp_head - __head - seplen; // 转换成从0开始的引用
                }
            }
            return -1;
        }

        /// @brief 按照指定的开头和结尾寻找数据，单个数据开关或结尾
        /// @param begin_sep 数据开始标记
        /// @param end_sep 数据结束标记
        /// @param offset 开始位置的偏移量
        /// @return [start_header_index, length]，其中长度包含了end_sep的长度
        const span_data_t span(std::initializer_list<value_type> &&begin_sep,
                               std::initializer_list<value_type> &&end_sep,
                               size_type offset = 0) const
        {
            span_data_t result;
            int beging_seplen{static_cast<int>(begin_sep.size())};
            int end_seplen{static_cast<int>(end_sep.size())};
            result.head = search(std::forward<std::initializer_list<value_type>>(begin_sep), offset);
            if (result.head == -1)
            {
                return result;
            }
            int end_head{search(std::forward<std::initializer_list<value_type>>(end_sep), result.head + beging_seplen + offset)};
            if (end_head != -1)
            {
                result.len = end_head - result.head + end_seplen;
            }
            return result;
        }

        /// @brief 按照指定的开头和结尾寻找数据
        /// @param begin_sep 数据开始标记
        /// @param begin_seplen 开始标记长度
        /// @param end_sep 数据结束标记
        /// @param end_seplen 结束标记长度
        /// @param offset 开始位置的偏移量
        /// @return [start_header_index, length]，其中长度包含了end_sep的长度
        const span_data_t span(const value_type *begin_sep, const size_type begin_seplen,
                               const value_type *end_sep, const size_type end_seplen,
                               size_type offset = 0) const
        {
            span_data_t result;
            result.head = search(begin_sep, begin_seplen, offset);
            if (result.head == -1)
            {
                return result;
            }
            int end_head{search(end_sep, end_seplen, result.head + begin_seplen)};
            if (end_head != -1)
            {
                result.len = end_head - result.head + end_seplen;
            }
            return result;
        }

        /// @brief 按照指定的开头和结尾寻找数据，单个数据开关或结尾
        /// @param begin_sep 数据开始标记
        /// @param end_sep 数据结束标记
        /// @param offset 开始位置的偏移量
        /// @return [start_header_index, length]，其中长度包含了end_sep的长度
        const span_data_t span(const value_type begin_sep, const value_type end_sep, size_type offset = 0) const
        {
            span_data_t result;
            result.head = search(begin_sep, offset);
            if (result.head == -1)
            {
                return result;
            }
            auto end_head = search(end_sep, result.head + 1);
            if (end_head != -1)
            {
                result.len = end_head - result.head + 1;
            }
            return result;
        }

        /// @brief 读取数据，但不会将数据删除
        /// @param dest 目标地址
        /// @param sz 数据长度
        /// @return 读取的数据长度
        const size_type read(value_type *dest, size_type sz)
        {
            if (is_empty() || !sz)
            {
                return 0;
            }
            if (sz > len())
            {
                sz = len();
            }

            size_type i = std::min(sz, __capacity - __head_pos());
            memcpy(dest, __buf + __head_pos(), i);
            memcpy(dest + i, __buf, sz - i);
            return sz;
        }

        /// @brief 读取数据，但不会将数据删除
        /// @param dest 目标地址
        /// @param start_pos 数据索引开始的位置
        /// @param sz 数据长度
        /// @return 读取的数据长度
        const size_type read(value_type *dest, size_type start_pos, size_type sz)
        {
            if (!dest || !sz || is_empty())
            {
                return 0;
            }
            if (sz > len() - start_pos)
            {
                sz = len() - start_pos;
            }

            auto tmp_head_pos = __mod(__head + start_pos);
            size_type i = std::min(sz, __capacity - tmp_head_pos);
            memcpy(dest, __buf + tmp_head_pos, i);
            memcpy(dest + i, __buf, sz - i);
            return sz;
        }

        /// @brief 通过[]来访问元素
        ///        注意：此功能不进行越界处理，需要使用者自行判断
        /// @param pos 读取数组位置
        /// @return 返回指定地址的值的引用
        template <typename _IdxType, typename = std::enable_if<
                                         std::is_integral<_IdxType>::value ||
                                         std::is_unsigned<_IdxType>::value>>
        inline value_type &operator[](const _IdxType pos)
        {
            return __buf[__mod(pos + __head)];
        }

        RingBuffer() = delete;
        RingBuffer(const RingBuffer &) = delete;
        RingBuffer &operator=(const RingBuffer &) = delete;

        RingBuffer(RingBuffer &&other)
            : __buf(other.__buf), __capacity(other.__capacity),
              __tail(other.__tail), __head(other.__head), __size_end_pos(other.__size_end_pos)
        {
        }

        RingBuffer &operator=(RingBuffer &&other)
        {
            if (__buf)
            {
                delete[] __buf;
            }
            __buf = other.__buf;
            __capacity = other.__capacity;
            __tail = other.__tail;
            __head = other.__head;
            __size_end_pos = other.__size_end_pos;
            // 删除原对象的值
            other.__buf = nullptr;
            other.__capacity = 0;
            other.__tail = 0;
            other.__head = 0;
            other.__size_end_pos = 0;
            return *this;
        }

        RingBuffer(size_type capacity) : __capacity(capacity)
        {
            if (!__is_power_of_two(__capacity))
            {
                __capacity = __roundup_power_of_two(__capacity);
            }
            __buf = new value_type[__capacity];
            __size_end_pos = __capacity - 1;
        }

        ~RingBuffer()
        {
            if (__buf)
            {
                delete[] __buf;
            }
        }
    };
}

#endif // __RING_BUFFER_HPP__;
