#include "logging.h"

#include <spdlog/sinks/stdout_sinks.h>

using namespace std;

std::shared_ptr<spdlog::logger> init_logger(const char* logger_id)
{
    static constexpr auto default_id = "logger";
    if (logger_id == nullptr) {
        logger_id = default_id;
    }

    auto result = spdlog::get(logger_id);
    if (result == nullptr) {
        result = spdlog::stdout_logger_mt(logger_id);
        result->flush_on(spdlog::level::err);
    }

    return result;
}