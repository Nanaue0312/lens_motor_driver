#ifndef __UTIOOLS_H__
#define __UTIOOLS_H__

// 编码相关的函数
#include "coded/hexstr.h" // 将二进制数据转为十六进制字符串
#include "coded/base64.h"
#include "coded/md5.h"
#include "coded/sha1.h"
#include "coded/uuid.h"   // 生成UUID
#include "coded/endian.h" // 处理大小端功能
#include "coded/crc.h"
#include "coded/bit_packer.h"
#include "coded/byte_packer.h"
namespace utcode = utools::code; // 编码相关的函数

// 字符串相关的辅助函数
#include "ufmt/f_string.h"
#include "ufmt/str_conv.h"
namespace utfmt = utools::fmt; // 字符串相关的辅助函数

// 假数据生成器，包括随机数生成器
#include "fake_data.h"
namespace utfake = utools::fake_data; // 假数据生成器

// 时间相关的功能函数
#include "utime/time_utils.h"
#include "utime/time_invoker.h"
#include "utime/timer.h"
#include "utime/std_time_conv.h"
#include "utime/stable_interval_invoker.h"
namespace utime = utools::time; // 时间相关的功能函数

// 日志相关定义
#include "logger/logger.h"
using utlog = utools::logger; // 日志相关定义

// 内存池相关定义，默认不使用内存池
#include "umempool/mempool.h"
#include "umempool/mempool_ipml.h"
#include "uthread_pool/thread_pool.h"
namespace utpool = utools::pool; // 内存池相关定义

// 环形缓存和环形对列功能
#include "ring_buffer/ring_buffer.h"
#include "ring_buffer/ring_queue.h"
#include "any_store/any_type_map.h" // 任意类型存储
namespace utmem = utools::mem;      // 环形缓存和环形对列功能

// 线程协作相关的功能函数
#include "collab/collab.h"
#include "collab/collab_task.h"
#include "collab/sync_queue.h"
#include "collab/sync_partner.h"
#include "collab/spin_wait.h"
#include "collab/wait_notify.h"
#include "collab/func_wrapper.h"
#include "collab/flow_task.h"
#include "collab/sync_partner_freertos.h"
#include "collab/dispatcher.h"
#include "collab/utcondition_variable.h"
#include "collab/utmutex.h"
namespace utcollab = utools::collab; // 线程协作相关的功能函数

// 常用计算工具函数
#include "umath/umath.h"
namespace utmath = utools::math; // 常用计算工具函数

// 常用滤器算法
#include "fliter/arithmetic_average.h"
#include "fliter/kalman.h"
#include "fliter/least_square_method.h"
#include "fliter/median_average.h"
#include "fliter/median_fliter.h"
#include "fliter/moving_average.h"
#include "fliter/variance.h"
#include "fliter/first_order_fliter.h"
namespace utfliter = utools::fliter; // 常用数字滤器算法

// 关于功能，版本管理等
#include "about_mang/about_mang.h"
namespace utabout = utools::about; // 关于功能，版本管理等

// 设计模式相关
#include "pattern/singleton.h"         // 单例模式
#include "pattern/state_machine.h"     // 状态机模式
#include "pattern/observe.h"           // 观察者模式
namespace utpattern = utools::pattern; // 设计模式相关

#endif // __UTIOOLS_H__