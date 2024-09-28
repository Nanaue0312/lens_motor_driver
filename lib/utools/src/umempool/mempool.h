/// @brief 内存池功能

#ifndef __MEMPOOL_HPP__
#define __MEMPOOL_HPP__

#if UTOOLS_MEMPOLL_EANBLE == 1

#include <cstring>
#include <cstdint>
#include <vector>

namespace utools::pool
{
    /// @brief 内存池
    /// @tparam _ChunkSizeLimit 最小的chunk空间，每次申请时的最小单元，默认是4k
    /// @tparam _Align 数据对齐方式，默认是16
    /// @tparam _MaxSize 单块内存最大的值
    /// @tparam _DelegLargerSize 是否托管大内存
    template <uint32_t _ChunkSizeLimit = 4096, uint32_t _Align = 16, uint32_t _MaxSize = 512, bool _DelegLargerSize = true>
    class Mempool
    {
    public:
        /// @brief 申请内存空间
        /// @param size 内存大小
        /// @return 返回申请到的空间，如果失败返回nullptr
        void *allocate(const uint32_t size)
        {
            // 1、如果申请的空间大可申请空间，调用系统函数申请
            if (_DelegLargerSize)
            {
                if (_MaxSize < size && _DelegLargerSize)
                {
                    return operator new(size, std::nothrow);
                }
                return nullptr;
            }

            // 2、判断是否还有可用的空间
            volatile block_element_pointer_t &block{__free_blocks[__freelist_index(size)]}; // 找到对应的可用空间
            if (block != nullptr)
            {
                void *dptr{reinterpret_cast<void *>(block)};
                block = block->next; // 将指针指向一下个数据
                return dptr;
            }
            // 3、如果需要重新申请，在此申请
            return __refill(__round_up(size)); // 重新分配空间
        }

        /// @brief 根据类型申请空间，并进行初始化
        /// @tparam _T 对象类型
        /// @tparam ...Args 参数列类型
        /// @param ...args 参数列
        /// @return 创建好的对象，失败返回nullptr
        template <class _T, class... Args>
        inline _T *allocate(Args &&...args)
        {
            void *result{allocate(sizeof(_T))};
            if (nullptr != result)
            {
                new (result) _T(std::forward<Args>(args)...);
                return reinterpret_cast<_T *>(result);
            }
            return nullptr;
        }

        /// @brief 申请内存空间并清零缓存
        /// @param size 内存大小
        /// @return 返回申请到的空间，如果失败返回nullptr
        void *callocate(const uint32_t size)
        {
            void *result{allocate(size)};
            if (result)
            {
                memset(result, 0, size);
            }
            return result;
        }

        /// @brief 回收已经分配的内存空间，回收前并不进行检测，只根据大小进行
        /// @param ptr 空间的指针
        /// @param size 申请时指针的大小
        inline void deallocate(void *ptr, uint32_t size)
        {
            // 1、如果申请的空间大可申请空间，调用系统函数申请
            if (_MaxSize < size && _DelegLargerSize)
            {
                operator delete(ptr);
                return;
            }
            // 2、从缓存中删除数据
            volatile block_element_pointer_t &block{__free_blocks[__freelist_index(size)]};
            reinterpret_cast<block_element_pointer_t>(ptr)->next = block;
            block = reinterpret_cast<block_element_pointer_t>(ptr);
        }

        /// @brief 回收已经分配的内存空间，回收前并不进行检测，根据数据类型
        /// @tparam _T 回收的数据类型
        /// @param obj 需要回收的对象，注意：此操作不会检查内存的合法性
        template <typename _T>
        inline void deallocate(_T *obj)
        {
            obj->~_T(); // 调用析构函数
            deallocate(obj, sizeof(_T));
        }

        /// @brief 重新申请一个空间，并且将原空间的数据复制到新的空间中
        /// @param ptr 原空间大小
        /// @param size 新申请的空间大小
        /// @return 新申请的空间指针
        template <typename _T>
        _T *reallocate(_T *ptr, const uint32_t old_size, const uint32_t new_size)
        {
            if (nullptr == ptr && new_size > 0)
            {
                return static_cast<_T *>(allocate(new_size));
            }
            if (nullptr != ptr && 0 == new_size)
            {
                deallocate(ptr, old_size);
                return nullptr;
            }
            // 如果新申请的空间不大小当前块的空间，可以直接返回当前的块，只更新空间块的大小即可
            if (_DelegLargerSize)
            {
                if (_MaxSize >= old_size && new_size <= __round_up(old_size))
                {
                    return ptr;
                }
            }
            else
            {
                if (new_size <= __round_up(old_size))
                {
                    return ptr;
                }
            }
            _T *tmp_dat{reinterpret_cast<_T *>(allocate(new_size))};
            if (tmp_dat)
            {
                if (nullptr != ptr)
                {
                    memcpy(tmp_dat, ptr, old_size <= new_size ? old_size : new_size);
                }
                // 删除老的对象，等价与deallocate功能
                deallocate(ptr, old_size);
                // 返回新的地址
                return tmp_dat;
            }
            return nullptr; // 分配失败
        }

        Mempool(const Mempool &) noexcept = delete;

        Mempool(Mempool &&_mempool) noexcept
        {
            *this = _mempool;
        }

        Mempool() noexcept = default;

        virtual ~Mempool() noexcept
        {
            for (auto chunk : __chunks)
            {
                operator delete(chunk); // 删除所有的块
            }
        }

        Mempool &operator=(const Mempool &) = delete;
        Mempool &operator=(Mempool &&_mempool)
        {
            if (this != &_mempool)
            {
                std::copy(&_mempool.__free_blocks[0], &_mempool.__free_blocks[_MaxSize / _Align - 1], __free_blocks);
                std::swap(__chunks, _mempool.__chunks);
                std::swap(__start_free, _mempool.__start_free);
                std::swap(__end_free, _mempool.__start_free);
            }
            return *this;
        }

    private:
        /// @brief block元素的名称
        union block_element_t
        {
            block_element_t *next{nullptr};
        };
        typedef block_element_t *block_element_pointer_t;

        std::vector<uint8_t *> __chunks;                                   // 所有已经申请的空间
        volatile block_element_pointer_t __free_blocks[_MaxSize / _Align]; // 数据块，freelist
        uint8_t *__start_free{nullptr};                                    // 可用空间的开始
        uint8_t *__end_free{nullptr};                                      // 可用空间的结束

        /// @brief 上向对齐，根据_Align值对其
        /// @param bytes 需要的空间
        /// @return 返回对齐后要申请的空间
        inline const uint32_t __round_up(const uint32_t bytes) const
        {
            return (((bytes) + _Align - 1) & ~(_Align - 1));
        }

        /// @brief 根据需要的数据大小，生成freelist的索引
        /// @param bytes 需要的空间
        /// @return 返回索引
        inline const uint32_t __freelist_index(const uint32_t bytes) const
        {
            return (((bytes) + _Align - 1) / _Align - 1);
        }

        /// @brief 加载新的block
        /// @param size 需要的空间，已经被对其后的数据
        /// @return 返回数据头，nullptr表示失败
        inline volatile block_element_pointer_t __refill(const uint32_t size)
        {
        REFILL_TRY_AGAIN:
            // 1、查看现在是不是还有可以使用的空间，如果直接返回一块空间
            uint32_t free_size{static_cast<uint32_t>(__end_free - __start_free)}; // 将ptrdiff_t转换为uint32_t，用于表示可用空间
            if (free_size >= size)
            {
                // a) 直接跳过第一个数据，因为后面要返回
                uint8_t *result{__start_free};
                __start_free += size;
                // b) 将free_blocks的指针指向下一个数据
                __free_blocks[__freelist_index(size)] = reinterpret_cast<block_element_pointer_t>(__start_free); // 将数据放到freelist中
                decltype(__start_free) next_free{__start_free + size};
                // c) 循环将新的数据加入到free_blocks中，最多一次增加16个数据
                uint32_t block_count{0};
                while (__end_free > __start_free && block_count < 16)
                {
                    reinterpret_cast<block_element_pointer_t>(__start_free)->next = reinterpret_cast<block_element_pointer_t>(next_free);
                    __start_free = next_free;
                    next_free += size;
                    ++block_count;
                }
                // 将最后一数据所指向的位置设备为空
                reinterpret_cast<block_element_pointer_t>(__start_free)->next = nullptr;
                return reinterpret_cast<block_element_pointer_t>(result);
            }
            // 2、将可用空间中的碎片回收，只有当内存空间大8时才需要回收
            else if (free_size >= 8)
            {
                volatile block_element_pointer_t &block{__free_blocks[__freelist_index(free_size)]}; // 找到对应的可用空间
                reinterpret_cast<block_element_pointer_t>(__start_free)->next = block;
                block = reinterpret_cast<block_element_pointer_t>(__start_free);
            }
            // 3、没有空间，并且已经回收碎片，需要新建申请一个空间
            if (__allocate_chunk(size, 32))
            {
                // return __refill(size); // 申请到了新的空间
                goto REFILL_TRY_AGAIN;
            }
            return nullptr;
        }

        /// @brief 申请大内存块，用于后面的使用
        /// @param block_size 需要申请的block数据大小
        /// @param block_count 需要申请的block数据个数
        /// @return 返回数据头，nullptr表示失败
        inline uint8_t *__allocate_chunk(const uint32_t block_size, const uint32_t block_count)
        {
            // 1、判断需要新申请的空间大小
            uint32_t chunk_size{(block_size * block_count) < 1}; // 申请需要的两倍空间
            if (chunk_size < _ChunkSizeLimit)
            {
                chunk_size = _ChunkSizeLimit; // 如果申请的空间小于4k，自动升级到4k
            }
            __start_free = reinterpret_cast<uint8_t *>(operator new(chunk_size)); // 重新获取新的内存块
            if (nullptr == __start_free)
            {
                return __allocate_handler(block_size); // 此时只需要申请一个可用空间
            }
            __end_free = __start_free + chunk_size;
            __chunks.push_back(__start_free); // 保存内存页所有的位置，析构时删除使用
            return __start_free;
        }

        /// @brief 处理没有可用缓存的情况
        /// @param size 需要的空间，会尝试从已经分配的空间里面找到可用的空间
        /// @return 返回数据头，nullptr表示失败
        inline uint8_t *__allocate_handler(const uint32_t size)
        {
            for (size_t i = _MaxSize / _Align - 1; i > __freelist_index(size); --i)
            {
                volatile block_element_pointer_t &block{__free_blocks[i]};
                if (block != nullptr)
                {
                    __start_free = reinterpret_cast<uint8_t *>(block);
                    __end_free = __start_free + _Align * (i + 1);
                    block = block->next; // 将指针指向一下个数据
                    return __start_free; // 返回结果
                }
            }
            return nullptr;
        }
    };
}
#endif // UTOOLS_MEMPOLL_EANBLE
#endif // __MEMPOOL_HPP__