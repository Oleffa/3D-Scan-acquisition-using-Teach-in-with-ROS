// $Id: rxpstream.hpp 454 2010-06-13 13:47:06Z rs $

#ifndef RXPSTREAM_HPP
#define RXPSTREAM_HPP

#include <riegl/config.hpp>
#include <riegl/ridataspec.hpp>
#include <riegl/rxpmarker.hpp>
#include <riegl/connection.hpp>
#include <riegl/fileconn.hpp>
#include <riegl/rddpconn.hpp>
#include <riegl/rdtpconn.hpp>

#ifndef RIVLIB_TR1_DIR
#   if defined(_MSC_VER)
#       include <memory>
#   else
#       include <tr1/memory>
#   endif
#else
#   include <tr1/memory>
#endif

#include <string>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <sstream>

#include <iostream> //DEBUG:

namespace scanlib {

template<class B>
struct basic_rxp_packet
{
    basic_rxp_packet() : id_main(0), id_sub(0), begin(0), end(0) {}
    unsigned id_main;
    unsigned id_sub;
    const B* begin;
    const B* end;
    std::vector<B> buffer;
};

template<class B>
class basic_rxp_istream
{
    std::tr1::shared_ptr<basic_rconnection> rc;
    std::tr1::shared_ptr<decoder_rxpmarker> dec;
    buffer                                  buf;
    lookup_table                            lookup;
    const B*                                begin;
    const B*                                end;
    package_id                              id;

    void init(const char* uri) {
        std::string s(uri);
        if (s.find(':') == std::string::npos)
            s = std::string("file:")+s;
        rc = basic_rconnection::create(s);
        rc->open();
        dec = std::tr1::shared_ptr<decoder_rxpmarker>(
            new decoder_rxpmarker(rc)
        );
    }

public:

    basic_rxp_istream(const char* uri) {
        init(uri);
    }

    basic_rxp_istream(const std::string& uri) {
        init(uri.c_str());
    }

    ~basic_rxp_istream() {
        rc->close();
    }

    operator void*() {
        if (dec->eoi())
            return 0;
        else
            return this;
    }

    bool good() const {
        if (dec->eoi())
            return false;
        else
            return true;
    }

    basic_rxp_istream& operator>>(package_id& x) {
        dec->get(buf);
        if (!dec->eoi()) {
            basic_package<B*> pkg(buf.begin(), buf.end(), lookup);
            begin = pkg.begin();
            end = pkg.end();
            x = id = package_id(pkg.id.main, pkg.id.sub);
            if (pkg.id.main == 1 && pkg.id.sub == 0) {
                header<const B*> arg(pkg.begin(), pkg.end(), false);
                lookup.load(arg.id_lookup);
            }
        }
        return *this;
    }

    template<class P>
    basic_rxp_istream& operator>>(P& p) {
        if (!dec->eoi()) {
            if (P::id_main == id.main && P::id_sub == id.sub) {
                typename P::template rebind<const B*>::type r(begin, end, false);
                p = r;
            }
            else
                throw(std::runtime_error("basic_rxp_istream: id mismatch"));
        }
        return *this;
    }

    basic_rxp_istream& operator>>(basic_rxp_packet<B>& p) {
        if (!dec->eoi()) {
            p.id_main = id.main;
            p.id_sub = id.sub;
            p.begin = begin;
            p.end = end;
        }
        return *this;
    }

};

template<class B>
class basic_rxp_ostream
{
    std::tr1::shared_ptr<basic_wconnection> wc;
    std::tr1::shared_ptr<encoder_rxpmarker> enc;
    buffer                                  buf;
    lookup_table                            lookup;
    package_id                              id;

    void init(const char* uri) {
        std::string s(uri);
        if (s.find(':') == std::string::npos)
            s = std::string("file:")+s;
        wc = basic_wconnection::create(s);
        wc->open();
        enc = std::tr1::shared_ptr<encoder_rxpmarker>(
            new encoder_rxpmarker(wc)
        );
        enc->put(buf);
    }

public:

    basic_rxp_ostream(const char* uri) {
        init(uri);
    }

    basic_rxp_ostream(const std::string& uri) {
        init(uri.c_str());
    }

    ~basic_rxp_ostream() {
        wc->close();
    }

    operator void*() {
        return this; //todo: Is the stream always ok?
    }

    bool good() const {
        return true; //todo: Is the stream always ok?
    }

    basic_rxp_ostream& operator<<(const package_id& x) {
        id = x;
        return *this;
    }

    template<class P>
    basic_rxp_ostream& operator<<(const P& src) {
        if (P::id_main == id.main && P::id_sub == id.sub) {
            buffer_package<typename P::template rebind<B*>::type > dst(
                lookup, buf
            );
            dst = src;
            dst.resize();
            enc->put(buf);
        }
        else
            throw(std::runtime_error("basic_rxp_ostream: id mismatch"));
        return *this;
    }

    template<class other_it>
    basic_rxp_ostream& operator<<(const header<other_it>& src) {
        if (1 == id.main && 0 == id.sub) {
            buffer_package<header<B*> > dst(buf);
            dst = src;
            lookup_table lu;
            lu.load(dst.id_lookup);
            lookup = lu;
            dst.resize();
            enc->put(buf);
        }
        else
            throw(std::runtime_error("basic_rxp_ostream: id mismatch"));
        return *this;
    }

    basic_rxp_ostream& operator<<(const basic_rxp_packet<B>& p) {
        if  (
                (p.id_main == id.main && p.id_sub == id.sub)
                || (id.main == 0 && id.sub == 0) // handle opaque unknowns
            ) {
            if (1 == p.id_main && 0 == p.id_sub) {
                header<const B*> h(p.begin, p.end, false);
                *this << h;
            }
            else {
                write_package<B*> wp(
                    p.id_main, p.id_sub, lookup, buf.begin(), buf.end()
                );
                std::copy(p.begin, p.end, wp.begin());
                buf.resize((wp.begin()-buf.begin())+(p.end-p.begin));
                enc->put(buf);
            }
        }
        else {
            std::stringstream msg;
            msg << "basic_rxp_ostream: id mismatch: "
                << "expected " << id.main << "." << id.sub << " "
                << "got " << p.id_main << "." << p.id_sub;
            throw(std::runtime_error(msg.str()));
        }
        return *this;
    }
};

typedef basic_rxp_packet<uint32_t>   rxp_packet;
typedef basic_rxp_istream<uint32_t>  rxp_istream;
typedef basic_rxp_ostream<uint32_t>  rxp_ostream;

} // namespace scanlib


namespace std {

template<class charT, class traits, class B>
basic_istream<charT, traits>&
operator>>(basic_istream<charT, traits>& in, scanlib::basic_rxp_packet<B>& p)
{
    if (in.iword(scanlib::stream_unknown_idx)) {
        p.id_main = p.id_sub = 0;
    } else {
        p.id_main = in.iword(scanlib::stream_id_main_idx);
        p.id_sub = in.iword(scanlib::stream_id_sub_idx);
    }
    p.begin = p.end = 0;
    if (scanlib::ridataspec_read(p.buffer, p.id_main, p.id_sub, in)) {
        if (p.buffer.size()) {
            p.begin = &(p.buffer[0]);
            p.end = &p.buffer[0]+p.buffer.size();
        }
        if (in.iword(scanlib::stream_unknown_idx)) {
            p.id_main = in.iword(scanlib::stream_id_main_idx);
            p.id_sub = in.iword(scanlib::stream_id_sub_idx);
        }
    }
    else {
        p.buffer.clear();
        in.setstate(ios::failbit);
    }
    return in;
}

template<class charT, class traits, class B>
basic_ostream<charT, traits>&
operator<<(basic_ostream<charT, traits>& out, scanlib::basic_rxp_packet<B>& p)
{
    //std::cout << out.iword(scanlib::stream_id_main_idx) << ", " << out.iword(scanlib::stream_id_sub_idx) << std::endl;
#if 0
        if (
            p.id_main != out.iword(scanlib::stream_id_main_idx) ||
            p.id_sub !== out.iword(scanlib::stream_id_sub_idx)
        ) {
            //todo: give a warning?
        }
#endif
    scanlib::ridataspec_write(p.begin, p.end, p.id_main, p.id_sub, out);

    return out;
}



} // namespace std

#endif // RXPSTREAM_HPP
