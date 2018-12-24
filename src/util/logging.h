#ifndef QUADMESH_SRC_UTIL_LOGGING_H_
#define QUADMESH_SRC_UTIL_LOGGING_H_

#include <memory>
#include <string_view>

#include <spdlog/spdlog.h>

std::shared_ptr<spdlog::logger> init_logger(const char* logger_id = nullptr);

#endif // QUADMESH_SRC_UTIL_LOGGING_H_