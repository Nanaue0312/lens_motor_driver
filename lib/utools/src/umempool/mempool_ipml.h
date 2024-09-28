/// @brief 内存管理模块

#ifndef __MEMPOOL_IMPL_H__
#define __MEMPOOL_IMPL_H__

#if UTOOLS_MEMPOLL_EANBLE == 1

#include "mempool.h"

namespace utools::pool
{
#ifndef UTOOLS_MEMPOLL_EANBLE

#define mp_malloc(size) malloc(size)
#define mp_calloc(nmemb, size) calloc(nmemb, size)
#define mp_free(ptr, ...) free(ptr)
#define mp_realloc(ptr, size) realloc(ptr, size)

#else

#ifndef UTOOLS_MEMPOLL_CHUNK_SIZE
#define UTOOLS_MEMPOLL_CHUNK_SIZE 4096 // 默认内存块的大小
#endif

    extern ::utools::pool::Mempool<UTOOLS_MEMPOLL_CHUNK_SIZE> global_mempool;

    template <typename T, typename... Args>
    constexpr void *mp_malloc(Args &&...args)
    {
        return (global_mempool.allocate<T>(std::forward<Args>(args)...));
    }

    template <typename _SizeType>
    void *mp_malloc(const _SizeType sz) { return (global_mempool.allocate(sz)); }

#define mp_calloc(sz) (global_mempool.callocate(sz))

    template <typename _T, typename... Args>
    constexpr void mp_free(_T *ptr, Args &&...args)
    {
        return (global_mempool.deallocate(ptr, std::forward<Args>(args)...));
    }

#define mp_realloc(ptr, old_sz, new_sz) (global_mempool.reallocate(ptr, old_sz, new_sz))

#endif
}

#endif //  UTOOLS_MEMPOLL_EANBLE

#endif // __MEMPOOL_IMPL_H__
