#include "mempool_ipml.h"

#ifdef UTOOLS_MEMPOLL_EANBLE
::utools::pool::Mempool<UTOOLS_MEMPOLL_CHUNK_SIZE> global_mempool;
#endif // UTOOLS_MEMPOLL_EANBLE