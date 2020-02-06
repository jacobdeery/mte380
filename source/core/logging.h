#include <stdexcept>

#include <boost/log/trivial.hpp>

#define LOG_INFO(msg) BOOST_LOG_TRIVIAL(info) << msg
#define LOG_WARN(msg) BOOST_LOG_TRIVIAL(warning) << msg

#define LOG_ERROR(msg)               \
    BOOST_LOG_TRIVIAL(error) << msg; \
    throw std::runtime_error(msg);

#define CHECK(cond, msg)                                \
    if (!(cond)) {                                      \
        LOG_ERROR(std::string("CHECK failed: ") + msg); \
    }

namespace mte {

void InitializeLogging(const std::string& process_name);

}  // namespace mte
