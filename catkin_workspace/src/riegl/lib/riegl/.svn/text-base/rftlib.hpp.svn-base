// $Id: rftlib.hpp 222 2009-06-19 07:43:03Z rs $

#ifndef RFTLIB_HPP
#define RFTLIB_HPP

#include <riegl/rdtp.hpp>

#include <string>
#include <vector>
#include <stdexcept>

#if defined(__MINGW32__) || defined(__GNUC__)
#include <stdint.h>
namespace rftlib {
    using ::uint64_t;
}
#   endif

#if defined(_MSC_VER)
namespace rftlib {
    typedef unsigned __int64  uint64_t;
}
#endif

namespace rftlib {

    struct dir_entry {
        bool is_dir;
        uint64_t size;
        std::string time;
        std::string path;
    };

    class rft_client_session {
        public:

            rft_client_session();
            virtual ~rft_client_session();

            void
            open(
                const std::string& authority
            );

            void
            close();

            std::vector<std::string>
            glob(
                std::string pattern
            );

            std::vector<dir_entry>
            stat(
                std::string pattern
            );

            bool
            rm(
                std::string path
                , bool recursive = false
            );

            bool
            mkdir(
                std::string path
            );

            std::string
            cp_remote_to_local(
                uint64_t& copied_bytes
                , std::string remote
                , std::string local =""
            );

            std::string
            cp_local_to_remote(
                uint64_t& copied_bytes
                , std::string local
                , std::string remote = ""
            );

        private:
            rdtp_client_session rdtp;
            std::string remote_uri;
    };

} // namespace rftlib
#endif //RFTLIB_HPP
