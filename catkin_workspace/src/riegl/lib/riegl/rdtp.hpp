// $Id: rdtp.hpp 207 2009-06-16 10:23:52Z rs $

#ifndef RDTP_HPP
#define RDTP_HPP

#include <string>
#include <cstddef>
#include <map>
#include <memory>

namespace rftlib {

    std::string
    uri_encode(
        const std::string& s
    );

    std::string
    uri_decode(
        const std::string& s
    );

    typedef std::map<std::string, std::string> options;

    class rdtp_client_session {
        public:

            rdtp_client_session();
            virtual ~rdtp_client_session();

            void open(const std::string& authority);
            void close();

            void request(const std::string& cmd, const options& opt);
            std::string status(options& opt);

            std::size_t readsome(char* buffer, std::size_t count);
            void write(const char* buffer, std::size_t count);

        private:

            class impl;
            friend class impl;
            std::auto_ptr<impl> pimpl;
    };
}

#endif //RDTP_HPP
