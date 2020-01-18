#include <iostream>
#include <locale>

#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>

#include "logging.h"

namespace logging = boost::log;

// TODO(jacob): fix this hardcoded path.
const boost::filesystem::path log_dir{"~/logs"};

namespace mte {

void InitializeLogging(const std::string& process_name) {
    const auto log_formatter = [process_name](logging::record_view const& rec,
                                              logging::formatting_ostream& stream) {
        auto date_time_formatter =
            stream << logging::expressions::format_date_time<boost::posix_time::ptime>(
                "TimeStamp", "[%Y-%m-%d %H:%M:%S.%f]");
        date_time_formatter(rec, stream);
        stream << " [" << process_name << "]: [" << rec[logging::trivial::severity] << "] "
               << rec[logging::expressions::smessage];
    };
    
    const auto logging_timestamp = boost::posix_time::second_clock::local_time();
    const auto facet = new boost::posix_time::time_facet();
    facet->format("%Y%m%d_%H%M%S");

    std::stringstream stream;
    stream.imbue(std::locale(std::cout.getloc(), facet));
    stream << logging_timestamp;

    logging::add_file_log(
        logging::keywords::file_name = (log_dir / stream.str() / (process_name + ".log")).string(),
        logging::keywords::format = log_formatter);

    logging::add_console_log(std::cout, logging::keywords::format = log_formatter);

    logging::add_common_attributes();
}

}  // namespace mte
