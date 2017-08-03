// $Id: connection.hpp 418 2010-03-11 12:56:50Z rs $

#ifndef CONNECTION_HPP
#define CONNECTION_HPP

//!\file connection.hpp
//! The abstract base classes for read and write connection protocols.

#include <riegl/config.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <ctime>
#include <iostream>

#ifndef RIVLIB_TR1_DIR
#   if defined(_MSC_VER)
#       include <memory>
#   else
#       include <tr1/memory>
#   endif
#else
#   include <tr1/memory>
#endif

//class RTL_API std::runtime_error;
//template class RIVLIB_API std::tr1::function<void (void)>;

namespace scanlib {

//!\brief exception, thrown when cancelled
class RIVLIB_API cancelled
    : public std::runtime_error
{
public:
    //! \param[in] message A description of the cancellation request.
    explicit cancelled(const std::string& message)
        : std::runtime_error(message)
    {}
    virtual ~cancelled() throw()
    {}
};

//!\brief base class for connections
//!\details Basic connection class for read and write protocols.
class RIVLIB_API basic_connection
{
public:
    typedef std::size_t size_type;
    typedef uint64_t    pos_type;

    //! The open and close may be implemented by a derived object
    //! where the constructor needs to block to establish
    //! or shut down the connection.
    virtual void
    open(
    ) {
    }
    virtual void
    close(
    ) {
    }

    //! Cause a blocked read or write to throw an exception of
    //! type "cancelled".
    virtual void
    cancel(
    ) {
    } //!<cancel outstanding operation

    virtual ~basic_connection() {}

    bool
    eoi(
    ) const {
        return is_eoi;
    }

    operator void*(
    ) {
        return is_eoi?0:this;
    }

public:
    std::string id; //!< An id describing the connection, e.g. the file name from the remote side.

protected:
    bool is_eoi;

    //! The default constructor is protected
    //! to enforce derivation from this class.
    //!\post id is initialized to a default value of "UNNAMED_<year><month><day>_<hour><minute><second>"
    basic_connection(
    )   : is_eoi(false)
    {
        // set a default id
        char t[8+13+1];
        time_t now = time(0);
        strftime(t, 8+13+1, "UNNAMED_%y%m%d_%H%M%S", gmtime(&now));
        id = t;
    }

private:
    // Prohibit copying of connection object.
    basic_connection(const basic_connection&);
    const basic_connection& operator=(const basic_connection&);
};


//!\brief abstract protocol class for read connections.
//!\details This is the abstract base class for read connections
class RIVLIB_API basic_rconnection
    : virtual public basic_connection
{
public:

    //! virtual constructor
    //!\param uri connection uri
    //!\param continuation resume information for aborted transfers
    //!\return connection class matching the protocol specified in uri
    static std::tr1::shared_ptr<basic_rconnection>
    create(
        const std::string& uri
        , const std::string& continuation = std::string()
    )
    {
        std::auto_ptr<basic_rconnection> pa =
            create_impl(uri, continuation);
        std::tr1::shared_ptr<basic_rconnection> p(pa);
        return p;
    }

    //! Read at maximum count bytes into user provided buffer.
    //! Function may block if no bytes available.
    //!\throw scanlib::cancelled
    //!\param buf user provided buffer
    //!\param count size of buffer in bytes
    //!\return number of actually read bytes or zero if sending
    //!        side has finished sending.
    virtual size_type
    readsome(
        void* buf
        , size_type count
    ) = 0;

    //! Read count bytes into user provided buffer.
    //! Function will block if no bytes available.
    //!\throw scanlib::cancelled
    //!\param buf user provided buffer
    //!\param count size of buffer in bytes
    //!\return a reference to the read object.
    basic_rconnection&
    read(
        void* buf
        , size_type count
    );

    //!\return return the number of bytes read during last succesful read
    size_type
    gcount(
    ) const {
        return read_count;
    }

    //! INTERNAL ONLY
    virtual pos_type
    tellg(
    ) const {
        return read_pos;
    }

    //! Get context to continue an aborted transfer.
    //! The returned string can be feed into create to resume a transfer.
    //!\return the continuation string
    virtual std::string continuation() const
    { return std::string(); }

    //! Signal a request to stop sending to the remote peer.
    //! After requesting a shutdown, the application is expected
    //! to read data until end of file is detected.
    virtual void request_shutdown() {}

protected:
    basic_rconnection(
    )   : read_count(0)
        , read_pos (0) {
    }

    size_type read_count;
    pos_type  read_pos;

private:
    //! INTERNAL ONLY
    static std::auto_ptr<basic_rconnection>
    create_impl(
        const std::string& uri
        , const std::string& continuation = std::string()
    );

};

//!\brief INTERNAL ONLY
class RIVLIB_API basic_wconnection
    : virtual public basic_connection
{
public:
    // virtual constructor
    static std::tr1::shared_ptr<basic_wconnection>
    create(
        const std::string& uri
    )
    {
        std::auto_ptr<basic_wconnection> pa =
            create_impl(uri);
        std::tr1::shared_ptr<basic_wconnection> p(pa);
        return p;
    }

    // Write at maximum count bytes from user provided buffer.
    // Return number of actually written bytes.
    // Function may block if it cannot send at least one byte.
    virtual size_type write(
        const void* buf
        , size_type count
    ) = 0;

    // Signal sending has finished, i.e. no more calls to write
    // will follow.
    virtual void shutdown() {}

    // Callback function that can be called by an implementation
    // of connection to signal the user a request to initiate
    // a graceful shutdown.
    // Function must not block and may be invoked from another
    // thread than read or write.
    //std::tr1::function<void (void)> on_shutdown_request;

protected:
    basic_wconnection() {};

private:
    static std::auto_ptr<basic_wconnection>
    create_impl(
        const std::string& uri
    );
};

//!\brief INTERNAL ONLY
class RIVLIB_API basic_rwconnection
    : public basic_rconnection
    , public basic_wconnection
{
    // virtual constructor
    static std::tr1::shared_ptr<basic_rwconnection>
    create(
        const std::string& uri
    )
    {
        std::auto_ptr<basic_rwconnection> pa =
            create_impl(uri);
        std::tr1::shared_ptr<basic_rwconnection> p(pa);
        return p;
    }

protected:
    basic_rwconnection() {};

private:
    static std::auto_ptr<basic_rwconnection>
    create_impl(
        const std::string& uri
    );
};

} // namespace scanlib



#endif // CONNECTION_HPP
