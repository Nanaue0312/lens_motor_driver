#include "logger.h"
#include <stdio.h>

namespace utools
{
    std::function<void(const char *)> logger::__print_fun{[](const char *c_str)
                                                          { printf(c_str); }};
    std::array<bool, 7> logger::__log_levels{false, true, true, true, true, true, true};
    std::string logger::__end_str{"\n"};
    std::string logger::__space_str{" "};

#if UTOOLS_LOGGER_FILTER_ENABLE == 1 // 开启过滤功能后可用
    std::vector<std::string> logger::__filter_keywords{};
    std::mutex logger::__filter_mutex;
#endif
}