// $Id: rddpconn.hpp 251 2009-07-09 09:28:13Z rs $

#ifndef RDDPCONN_HPP
#define RDDPCONN_HPP

#include <riegl/connection.hpp>

#include <string>

namespace scanlib {
//*****************************************************************************
//  rddp_rconnection
//*****************************************************************************
class RIVLIB_API rddp_rconnection
    : public basic_rconnection
{
public:
    rddp_rconnection(
        const std::string& rddp_uri
        , const std::string& continuation = std::string()
    );

    ~rddp_rconnection();

    void open();
    void close();
    void cancel();
    void request_shutdown();

    size_type readsome(
        void* buf
        , size_type count
    );

private:
    class impl;
    impl* pimpl;
};

} // namespace scanlib


#endif // RDDPCONN_HPP
