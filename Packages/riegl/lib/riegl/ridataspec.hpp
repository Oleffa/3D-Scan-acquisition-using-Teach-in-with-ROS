// This file was auto-generated from:
// /home/rs/projects/rivlib/scanlib/src/ridataspec.psp
// with pkgspec (x86_linux_gcc43) 2.11.467

#ifndef RIDATASPEC_HPP
#define RIDATASPEC_HPP

//! The version of ridataspec
//
//! The version macro evaluates to "2,20,497" for this version of the library.
//! The three fields are: major,minor and build numbers.
//! A change in major number typically will indicate changes in
//! the API or kind of packages.
#define RIDATASPEC_VERSION "2.20.497"

#ifndef DOXYGEN
#ifndef __cplusplus
#error This file cannot be used in C mode.
#endif /* __cplusplus */
#endif /* DOXYGEN */

#if defined(__cplusplus) || defined(DOXYGEN)
#include <riegl/detail/package.hpp>

#include <bitset>
#include <map>
#include <string>

namespace scanlib {

//-----------------------------------------------------------------------------
//!\brief class selector
//!\details A selector is a bitset, indexed by the package id.
//! The package id to use for indexing is one of the enum values 
//! defined in struct package_id. A couple of predefined
//! sets are available. Selectors can be understood as sets whose elements
//! are package (id's). Selector can be combined using set operations.
//! Please look for bitset in your documentation of the C++ standard library
//! to find out more.
//! Please note, that while it is possible to create arbitrary
//! selector combinations, care should be taken to always include the mandatory
//! header packet.
//! Selectors are used with the basic_packets class.
typedef std::bitset<121> selector_type;
//!\brief select all packets
//!\details <para>This selector selects all documented packets.</para>
extern const selector_type select_all;
//!\brief select no packets
//!\details <para>This selector selects no packets.</para>
extern const selector_type select_none;
//!\brief select packets of protocol class
//!\details <para>This selector selects the following documented packets:</para>
//! <para>frame_start_dn, frame_start_up, frame_stop, header, header_ext, line_start_dn, line_start_up, line_stop, meas_start, meas_stop, void_data</para>
extern const selector_type select_protocol;
//!\brief select packets of attribute class
//!\details <para>This selector selects the following documented packets:</para>
//! <para>atmosphere, atmosphere_1, beam_geometry, device_mounting, extents, monitoring_info, scan_rect_fov, scanner_pose, units, units_1</para>
extern const selector_type select_attribute;
//!\brief select packets of instrument class
//!\details <para>This selector selects the following documented packets:</para>
//! <para>beam_geometry, device_mounting, extents, units, units_1</para>
extern const selector_type select_instrument;
//! INTERNAL ONLY
extern const selector_type select_debug;
//!\brief select packets of data class
//!\details <para>This selector selects the following documented packets:</para>
//! <para>counter_sync, pps_sync, pps_sync_ext</para>
extern const selector_type select_data;
//!\brief select packets of scan class
//!\details <para>This selector selects the following documented packets:</para>
//! <para>atmosphere, atmosphere_1, counter_sync, frame_start_dn, frame_start_up, frame_stop, line_start_dn, line_start_up, line_stop, meas_start, meas_stop, monitoring_info, pps_sync, pps_sync_ext, scan_rect_fov, scanner_pose</para>
extern const selector_type select_scan;
//! INTERNAL ONLY
extern const selector_type select_deprecated;
//! INTERNAL ONLY
extern const selector_type select_internal;
//! INTERNAL ONLY
extern const selector_type select_packed;
//!\brief select packets of notify class
//!\details <para>This selector selects the following documented packets:</para>
//! <para>unsolicited_message, unsolicited_message_1</para>
extern const selector_type select_notify;
//!\brief select packets of status class
//!\details <para>This selector selects the following documented packets:</para>
//! <para>hk_gps, hk_gps_ts, hk_gps_ts_status, hk_gps_ts_status_dop, hk_incl, hk_incl_4axes, hk_rtc</para>
extern const selector_type select_status;
//!\brief select packets of legacy class
//!\details <para>This selector selects the following documented packets:</para>
//! <para>hk_rtc</para>
extern const selector_type select_legacy;

//! Get class selector by string name
//
//! Returns a selector by giving a string representaion of its name.
//! If the name is not a valid class name, the select_none selector will
//! be returned.
//!\param name any valid class name string
selector_type get_class_selector(const char* name);
//! Get class selector by string name
//
//! Returns a selector by giving a string representaion of its name.
//! If the name is not a valid class name, the select_none selector will
//! be returned.
//!\param name any valid class name string
selector_type get_class_selector(const std::string& name);
//-----------------------------------------------------------------------------
//! enumerated package identifiers
//
//! Packets can be refered by string name, by major and minor numbers,
//! and by an C++ enumeration. This class ties together these
//! various identifiers. While the string names and major, minor numbers
//! are unique outside the RiVLib context, the enumerations are only
//! valid in the context of a particular library version. Consequently
//! if you want to externally store a packet id, either string name or
//! major and minor numbers should be used. The enumerated values on the
//! other hand consist of a contiguous range and therefore can be used
//! as an index into an array. Please note however, that the documentation does not
//! list the entire set of values, the internal packets are omitted.
//! This means, that you cannot infer the numerical value of the enumeration
//! from its listed position in the documentation.
//! The enumerations are used as the index into the selector bitset
//! that can be set for packet dispatching.
struct package_id
{
    enum type {
        unknown = 0
        , atmosphere = 2
        , atmosphere_1 = 3
        , beam_geometry = 5
        , counter_sync = 10
        , device_mounting = 17
        , extents = 19
        , frame_start_dn = 25
        , frame_start_up = 26
        , frame_stop = 27
        , header = 29
        , header_ext = 30
        , hk_gps = 39
        , hk_gps_ts = 40
        , hk_gps_ts_status = 41
        , hk_gps_ts_status_dop = 42
        , hk_incl = 43
        , hk_incl_4axes = 44
        , hk_rtc = 57
        , line_start_dn = 78
        , line_start_up = 79
        , line_stop = 80
        , meas_start = 82
        , meas_stop = 83
        , monitoring_info = 84
        , pps_sync = 91
        , pps_sync_ext = 92
        , scan_rect_fov = 104
        , scanner_pose = 105
        , units = 113
        , units_1 = 114
        , unsolicited_message = 115
        , unsolicited_message_1 = 116
        , void_data = 117
        #ifndef DOXYGEN
        , alert = 1
        , avg_fine_ref_dg = 4
        , calib_waveform = 6
        , calib_waveform_L2 = 7
        , channel_combination_table = 8
        , context_end = 9
        , crc32_check = 11
        , crc32_header = 12
        , datagram_separator = 13
        , debug_hw_dg = 14
        , debug_sw_dg = 15
        , device_geometry = 16
        , echo = 18
        , firmware = 20
        , firmware_1 = 21
        , firmware_2 = 22
        , firmware_3 = 23
        , frame_start = 24
        , generic_end = 28
        , hk_bat = 31
        , hk_bat_1 = 32
        , hk_bat_2 = 33
        , hk_cam = 34
        , hk_ctr = 35
        , hk_ctr_1 = 36
        , hk_extended_external = 37
        , hk_extended_internal = 38
        , hk_pwr = 45
        , hk_pwr_1 = 46
        , hk_rad = 47
        , hk_rng = 48
        , hk_rng_1 = 49
        , hk_rng_2 = 50
        , hk_rng_3 = 51
        , hk_rng_4 = 52
        , hk_rng_5 = 53
        , hk_rng_6 = 54
        , hk_rng_7 = 55
        , hk_rng_8 = 56
        , hk_rtc_sys = 58
        , hk_scn = 59
        , hk_scn_1 = 60
        , hk_time = 61
        , ht_dbg_data = 62
        , inclination = 63
        , inclination_4axes = 64
        , inclination_device = 65
        , inclination_device_4axes = 66
        , inclination_device_4axes_offset = 67
        , laser_echo = 68
        , laser_echo_qual = 69
        , laser_echo_sw = 70
        , laser_shot = 71
        , laser_shot_1angle = 72
        , laser_shot_2angles = 73
        , laser_shot_2angles_rad = 74
        , laser_shot_3angles = 75
        , laser_shot_6angles = 76
        , line_start = 77
        , magnetic_field = 81
        , packed_frame_echo = 85
        , packed_frame_laser_shot_2angles = 86
        , packed_frame_laser_shot_2angles_rad = 87
        , packed_key_echo = 88
        , packed_key_laser_shot_2angles = 89
        , packed_key_laser_shot_2angles_rad = 90
        , range_calc = 93
        , range_finder_debug_acq = 94
        , range_finder_debug_calc = 95
        , range_finder_debug_laser = 96
        , range_finder_debug_rcv = 97
        , range_finder_program = 98
        , range_finder_settings = 99
        , rel_refl_table = 100
        , sbl_dg_data = 101
        , sbl_dg_data_compressed = 102
        , sbl_dg_header = 103
        , slt_dg = 106
        , slt_dg_1 = 107
        , slt_dg_2 = 108
        , slt_dg_3 = 109
        , tgt_dg = 110
        , ublox_lea5t_rxm = 111
        , ublox_lea5t_rxm_sfrb = 112
        , wfm_dg_hp = 118
        , wfm_dg_lp = 119
        , wfm_dg_shp = 120
        #endif //DOXYGEN
    };

    //! constructor from type
    //
    //! Construct a packet_id type enumeration.
    //!\param type_id package type identifier 
    package_id(package_id::type type_id) {
        *this = name_from_type[type_id].second;
    }

    //! constructor from id's
    //
    //! Construct a packet_id from externally unique main and sub id's
    //! Will map to unknown resp. main=0, sub=0 if requesting undefined packet.
    //!\param main_ main packet id
    //!\param sub_ sub packet id
    package_id(unsigned main_, unsigned sub_)
        : main(main_), sub(sub_)
    {}

    //! constructor from string name
    //
    //! Construct a packet_id from externally unique packet name
    //! Will map to unknown resp. main=0, sub=0 if requesting undefined packet.
    //!\param name string name of packet
    package_id(const char* name) {
        std::map<std::string, package_id>::const_iterator it;
        it = id_from_name.find(name);
        if (it != id_from_name.end()) {
            main = it->second.main;
            sub = it->second.sub;
        }
        else {
            main = 0;
            sub = 0;
        }
    }

    //! constructor from packet reference
    //
    //! Construct a packet_id from a packet instance
    //!\param p packet instance
    template<class P>
    explicit package_id(const P& p)
        : main(P::id_main), sub(P::id_sub)
    {}

    //! default constructor
    //
    //! Construct a id of invalid type
    package_id() {
        main = 0;
        sub = 0;
    }

    //! constructor from string name
    //
    //! Construct a packet_id from externally unique packet name
    //! Will map to unknown resp. main=0, sub=0 if requesting undefined packet.
    //!\param name string name of packet
    package_id(const std::string& name) {
        std::map<std::string, package_id>::const_iterator it;
        it = id_from_name.find(name);
        if (it != id_from_name.end()) {
            main = it->second.main;
            sub = it->second.sub;
        }
        else {
            main = 0;
            sub = 0;
        }
    }

    //! convert to enumeration
    //
    //! This operator converts the external id's into enumeration format.
    operator type() const;

    //! get string representation
    //
    //! This function converts the external id's into its string name.
    std::string string() const;
    unsigned main; //!< external main id
    unsigned sub;  //!< external sub id

private:
    static const std::pair<std::string, package_id> name_from_type[];
    static const std::map<std::string, package_id> id_from_name;
};

std::ostream& operator<<(std::ostream& s, const package_id& id); //!< INTERNAL ONLY
std::istream& operator>>(std::istream& s, package_id& id);//!< INTERNAL ONLY
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
template<class it>
struct package
    : public basic_package<it>
{
    package_id::type type;

    package(it begin, it end, const lookup_table& lu)
        : basic_package<it>(begin, end, lu)
    {
        lookup_table::id& id(basic_package<it>::id);
        type = package_id(id.main, id.sub);
    }
};
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct alert
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef alert<rebind_it> type;
    };

    typedef it iterator_type;

    alert(it begin, it end, bool dirty=false)
        : unit(begin)
        , number(begin)
        , subnumber(begin)
        , type(begin)
        , timestamp(begin)
        , message(begin)
        , flags(begin)
        , info(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 29, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 696};
    it begin() const { return unit.begin(); }
    it end() const { return info.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t unit;//!<  error signaling device unit 
    #else
    field<uint8_t, sc_uint8, 0, it> unit;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t number;//!<  error number 
    #else
    field<uint16_t, sc_uint16, 8, it> number;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t subnumber;//!<  error sub number 
    #else
    field<uint16_t, sc_uint16, 24, it> subnumber;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t type;//!<  error type (WARNING=1, NORMAL=2, NORMAL_UI=4, FATAL=8) 
    #else
    field<uint16_t, sc_uint16, 40, it> type;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint64_t timestamp;//!<  error timestamp, since the Epoch (00:00:00 UTC, January 1, 1970) [s] 
    #else
    field<uint64_t, sc_uint64, 56, it> timestamp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char message[64];//!<  error message 
    #else
    array<char, 64, sc_char, 120, it> message;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t flags;//!<  bitmapped flags (LASER_OFF=1, MEASUREMENT_STOP=2, MEASUREMENT_ABORT=4, SHUTDOWN=8) 
    #else
    field<uint32_t, sc_uint32, 632, it> flags;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float info;//!<  additional undefine error information 
    #else
    field<float, sc_float32, 664, it> info;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    alert& operator=(const alert<ito>& o) {
        unit = o.unit;
        number = o.number;
        subnumber = o.subnumber;
        type = o.type;
        timestamp = o.timestamp;
        for(unsigned n=0; n<64; ++n) message[n] = o.message[n];
        flags = o.flags;
        info = o.info;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct alert<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef alert<rebind_it> type;
    };

    enum { id_main = 29, id_sub = 0};

    uint8_t          unit;
    uint16_t         number;
    uint16_t         subnumber;
    uint16_t         type;
    uint64_t         timestamp;
    char             message[64];
    uint32_t         flags;
    float            info;

    alert() {}
    template<class it>
    alert(const alert<it>& o) {
        unit = o.unit;
        number = o.number;
        subnumber = o.subnumber;
        type = o.type;
        timestamp = o.timestamp;
        for(unsigned n=0; n<64; ++n) message[n] = o.message[n];
        flags = o.flags;
        info = o.info;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const alert<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.unit;
        s << ", " << x.number;
        s << ", " << x.subnumber;
        s << ", " << x.type;
        s << ", " << x.timestamp;
        s << ", "; write_array(s, 64, x.message);
        s << ", " << x.flags;
        s << ", " << x.info;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, alert<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.unit;
        s >> ',' >> x.number;
        s >> ',' >> x.subnumber;
        s >> ',' >> x.type;
        s >> ',' >> x.timestamp;
        s >> ','; read_array(s, 64, x.message);
        s >> ',' >> x.flags;
        s >> ',' >> x.info;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! environmental information

//! <para>This package belongs to the predefined selectors:</para>
//! <para>attribute, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct atmosphere
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef atmosphere<rebind_it> type;
    };

    typedef it iterator_type;

    atmosphere(it begin, it end, bool dirty=false)
        : temperature(begin)
        , pressure(begin)
        , rel_humidity(begin)
        , rng_scale(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 27, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 128};
    it begin() const { return temperature.begin(); }
    it end() const { return rng_scale.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float temperature;//!<  user specified average temperature along measurement path [C] 
    #else
    field<float, sc_float32, 0, it> temperature;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float pressure;//!<  user specified average pressure along measurment path [mbar] 
    #else
    field<float, sc_float32, 32, it> pressure;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float rel_humidity;//!<  user specified relative humidity along measurement path [%] 
    #else
    field<float, sc_float32, 64, it> rel_humidity;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float rng_scale;//!<  INTERNAL ONLY conversion factor from range in sample intervals into range in echo packages 
    #else
    field<float, sc_float32, 96, it> rng_scale;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    atmosphere& operator=(const atmosphere<ito>& o) {
        temperature = o.temperature;
        pressure = o.pressure;
        rel_humidity = o.rel_humidity;
        rng_scale = o.rng_scale;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct atmosphere<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef atmosphere<rebind_it> type;
    };

    enum { id_main = 27, id_sub = 0};

    float            temperature;
    float            pressure;
    float            rel_humidity;
    float            rng_scale;

    atmosphere() {}
    template<class it>
    atmosphere(const atmosphere<it>& o) {
        temperature = o.temperature;
        pressure = o.pressure;
        rel_humidity = o.rel_humidity;
        rng_scale = o.rng_scale;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const atmosphere<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.temperature;
        s << ", " << x.pressure;
        s << ", " << x.rel_humidity;
        s << ", " << x.rng_scale;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, atmosphere<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.temperature;
        s >> ',' >> x.pressure;
        s >> ',' >> x.rel_humidity;
        s >> ',' >> x.rng_scale;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! extended environmental information

//! <para>This package belongs to the predefined selectors:</para>
//! <para>attribute, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct atmosphere_1
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef atmosphere_1<rebind_it> type;
    };

    typedef it iterator_type;

    atmosphere_1(it begin, it end, bool dirty=false)
        : temperature(begin)
        , pressure(begin)
        , rel_humidity(begin)
        , rng_scale(begin)
        , pressure_sl(begin)
        , amsl(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 27, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 192};
    it begin() const { return temperature.begin(); }
    it end() const { return amsl.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float temperature;//!<  user specified average temperature along measurement path [C] 
    #else
    field<float, sc_float32, 0, it> temperature;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float pressure;//!<  user specified average pressure along measurment path [mbar] 
    #else
    field<float, sc_float32, 32, it> pressure;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float rel_humidity;//!<  user specified relative humidity along measurement path [%] 
    #else
    field<float, sc_float32, 64, it> rel_humidity;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float rng_scale;//!<  INTERNAL ONLY conversion factor from range in sample intervals into range in echo packages 
    #else
    field<float, sc_float32, 96, it> rng_scale;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float pressure_sl;//!<  user specified atmospheric pressure at sea level [mbar] 
    #else
    field<float, sc_float32, 128, it> pressure_sl;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float amsl;//!<  user specified height above mean sea level (AMSL) [m] 
    #else
    field<float, sc_float32, 160, it> amsl;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    atmosphere_1& operator=(const atmosphere_1<ito>& o) {
        temperature = o.temperature;
        pressure = o.pressure;
        rel_humidity = o.rel_humidity;
        rng_scale = o.rng_scale;
        pressure_sl = o.pressure_sl;
        amsl = o.amsl;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct atmosphere_1<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef atmosphere_1<rebind_it> type;
    };

    enum { id_main = 27, id_sub = 1};

    float            temperature;
    float            pressure;
    float            rel_humidity;
    float            rng_scale;
    float            pressure_sl;
    float            amsl;

    atmosphere_1() {}
    template<class it>
    atmosphere_1(const atmosphere_1<it>& o) {
        temperature = o.temperature;
        pressure = o.pressure;
        rel_humidity = o.rel_humidity;
        rng_scale = o.rng_scale;
        pressure_sl = o.pressure_sl;
        amsl = o.amsl;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const atmosphere_1<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.temperature;
        s << ", " << x.pressure;
        s << ", " << x.rel_humidity;
        s << ", " << x.rng_scale;
        s << ", " << x.pressure_sl;
        s << ", " << x.amsl;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, atmosphere_1<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.temperature;
        s >> ',' >> x.pressure;
        s >> ',' >> x.rel_humidity;
        s >> ',' >> x.rng_scale;
        s >> ',' >> x.pressure_sl;
        s >> ',' >> x.amsl;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct avg_fine_ref_dg
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef avg_fine_ref_dg<rebind_it> type;
    };

    typedef it iterator_type;

    avg_fine_ref_dg(it begin, it end, bool dirty=false)
        : packet_nr(begin)
        , slice_nr(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 50006, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 1056};
    it begin() const { return packet_nr.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t packet_nr;
    #else
    field<uint8_t, sc_uint8, 0, it> packet_nr;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t slice_nr;
    #else
    field<uint8_t, sc_uint8, 8, it> slice_nr;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 32 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sample(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint32_t sample;
        #else
        field<uint32_t, sc_uint32, 0, it> sample;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[32];
    #else
    sequence<avg_fine_ref_dg, 32, 32, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    avg_fine_ref_dg& operator=(const avg_fine_ref_dg<ito>& o) {
        packet_nr = o.packet_nr;
        slice_nr = o.slice_nr;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct avg_fine_ref_dg<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef avg_fine_ref_dg<rebind_it> type;
    };

    enum { id_main = 50006, id_sub = 0};

    uint8_t          packet_nr;
    uint8_t          slice_nr;
    std::size_t data_size;
    enum { data_max_size = 32 };
    struct sequence_definition {
        uint32_t         sample;
    } data[32];

    avg_fine_ref_dg() {}
    template<class it>
    avg_fine_ref_dg(const avg_fine_ref_dg<it>& o) {
        packet_nr = o.packet_nr;
        slice_nr = o.slice_nr;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const avg_fine_ref_dg<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.packet_nr;
        s << ", " << x.slice_nr;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].sample
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, avg_fine_ref_dg<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.packet_nr;
        s >> ',' >> x.slice_nr;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].sample)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! laser beam description

//! <para>This package belongs to the predefined selectors:</para>
//! <para>attribute, instrument</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct beam_geometry
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef beam_geometry<rebind_it> type;
    };

    typedef it iterator_type;

    beam_geometry(it begin, it end, bool dirty=false)
        : beam_exit_diameter(begin)
        , beam_divergence(begin)
        , beam_focus(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 3, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 96};
    it begin() const { return beam_exit_diameter.begin(); }
    it end() const { return beam_focus.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float beam_exit_diameter;//!<  beam width at exit aperture [m] 
    #else
    field<float, sc_float32, 0, it> beam_exit_diameter;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float beam_divergence;//!<  beam divergence in far field [rad] 
    #else
    field<float, sc_float32, 32, it> beam_divergence;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float beam_focus;//!<  INTERNAL ONLY distance of beam waist from origin [m] 
    #else
    field<float, sc_float32, 64, it> beam_focus;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    beam_geometry& operator=(const beam_geometry<ito>& o) {
        beam_exit_diameter = o.beam_exit_diameter;
        beam_divergence = o.beam_divergence;
        beam_focus = o.beam_focus;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct beam_geometry<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef beam_geometry<rebind_it> type;
    };

    enum { id_main = 3, id_sub = 0};

    float            beam_exit_diameter;
    float            beam_divergence;
    float            beam_focus;

    beam_geometry() {}
    template<class it>
    beam_geometry(const beam_geometry<it>& o) {
        beam_exit_diameter = o.beam_exit_diameter;
        beam_divergence = o.beam_divergence;
        beam_focus = o.beam_focus;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const beam_geometry<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.beam_exit_diameter;
        s << ", " << x.beam_divergence;
        s << ", " << x.beam_focus;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, beam_geometry<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.beam_exit_diameter;
        s >> ',' >> x.beam_divergence;
        s >> ',' >> x.beam_focus;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct calib_waveform
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef calib_waveform<rebind_it> type;
    };

    typedef it iterator_type;

    calib_waveform(it begin, it end, bool dirty=false)
        : endianess(begin)
        , n_shp_samples(begin)
        , R(begin)
        , B(begin)
        , K(begin)
        , P(begin)
        , N(begin)
        , sum_shp_samples(begin)
        , db_start(begin)
        , refpuls_position(begin)
        , avg_npulse(begin)
        , reference0(begin)
        , reference1(begin)
        , reference2(begin)
        , reference3(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 50100, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 131520};
    it begin() const { return endianess.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t endianess;
    #else
    field<uint16_t, sc_uint16, 0, it> endianess;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t n_shp_samples;
    #else
    field<uint8_t, sc_uint8, 16, it> n_shp_samples;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t R;
    #else
    field<uint32_t, sc_uint32, 32, it> R;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t B;
    #else
    field<uint32_t, sc_uint32, 64, it> B;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t K;
    #else
    field<uint32_t, sc_uint32, 96, it> K;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t P;
    #else
    field<uint32_t, sc_uint32, 128, it> P;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t N;
    #else
    field<uint32_t, sc_uint32, 160, it> N;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t sum_shp_samples;
    #else
    field<uint32_t, sc_uint32, 192, it> sum_shp_samples;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t db_start;
    #else
    field<uint32_t, sc_uint32, 224, it> db_start;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t refpuls_position;
    #else
    field<uint32_t, sc_uint32, 256, it> refpuls_position;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t avg_npulse;
    #else
    field<uint32_t, sc_uint32, 288, it> avg_npulse;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t reference0;
    #else
    field<uint32_t, sc_uint32, 320, it> reference0;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t reference1;
    #else
    field<uint32_t, sc_uint32, 352, it> reference1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t reference2;
    #else
    field<uint32_t, sc_uint32, 384, it> reference2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t reference3;
    #else
    field<uint32_t, sc_uint32, 416, it> reference3;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 4096 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sums(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint32_t sums;
        #else
        field<uint32_t, sc_uint32, 0, it> sums;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[4096];
    #else
    sequence<calib_waveform, 32, 448, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    calib_waveform& operator=(const calib_waveform<ito>& o) {
        endianess = o.endianess;
        n_shp_samples = o.n_shp_samples;
        R = o.R;
        B = o.B;
        K = o.K;
        P = o.P;
        N = o.N;
        sum_shp_samples = o.sum_shp_samples;
        db_start = o.db_start;
        refpuls_position = o.refpuls_position;
        avg_npulse = o.avg_npulse;
        reference0 = o.reference0;
        reference1 = o.reference1;
        reference2 = o.reference2;
        reference3 = o.reference3;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].sums = o.data[n].sums;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct calib_waveform<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef calib_waveform<rebind_it> type;
    };

    enum { id_main = 50100, id_sub = 0};

    uint16_t         endianess;
    uint8_t          n_shp_samples;
    uint32_t         R;
    uint32_t         B;
    uint32_t         K;
    uint32_t         P;
    uint32_t         N;
    uint32_t         sum_shp_samples;
    uint32_t         db_start;
    uint32_t         refpuls_position;
    uint32_t         avg_npulse;
    uint32_t         reference0;
    uint32_t         reference1;
    uint32_t         reference2;
    uint32_t         reference3;
    std::size_t data_size;
    enum { data_max_size = 4096 };
    struct sequence_definition {
        uint32_t         sums;
    } data[4096];

    calib_waveform() {}
    template<class it>
    calib_waveform(const calib_waveform<it>& o) {
        endianess = o.endianess;
        n_shp_samples = o.n_shp_samples;
        R = o.R;
        B = o.B;
        K = o.K;
        P = o.P;
        N = o.N;
        sum_shp_samples = o.sum_shp_samples;
        db_start = o.db_start;
        refpuls_position = o.refpuls_position;
        avg_npulse = o.avg_npulse;
        reference0 = o.reference0;
        reference1 = o.reference1;
        reference2 = o.reference2;
        reference3 = o.reference3;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].sums = o.data[n].sums;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const calib_waveform<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.endianess;
        s << ", " << x.n_shp_samples;
        s << ", " << x.R;
        s << ", " << x.B;
        s << ", " << x.K;
        s << ", " << x.P;
        s << ", " << x.N;
        s << ", " << x.sum_shp_samples;
        s << ", " << x.db_start;
        s << ", " << x.refpuls_position;
        s << ", " << x.avg_npulse;
        s << ", " << x.reference0;
        s << ", " << x.reference1;
        s << ", " << x.reference2;
        s << ", " << x.reference3;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].sums
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, calib_waveform<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.endianess;
        s >> ',' >> x.n_shp_samples;
        s >> ',' >> x.R;
        s >> ',' >> x.B;
        s >> ',' >> x.K;
        s >> ',' >> x.P;
        s >> ',' >> x.N;
        s >> ',' >> x.sum_shp_samples;
        s >> ',' >> x.db_start;
        s >> ',' >> x.refpuls_position;
        s >> ',' >> x.avg_npulse;
        s >> ',' >> x.reference0;
        s >> ',' >> x.reference1;
        s >> ',' >> x.reference2;
        s >> ',' >> x.reference3;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].sums)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct calib_waveform_L2
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef calib_waveform_L2<rebind_it> type;
    };

    typedef it iterator_type;

    calib_waveform_L2(it begin, it end, bool dirty=false)
        : endianess(begin)
        , n_shp_samples(begin)
        , R(begin)
        , B(begin)
        , K(begin)
        , P(begin)
        , N(begin)
        , sum_shp_samples(begin)
        , db_start(begin)
        , refpuls_position(begin)
        , avg_npulse(begin)
        , reference0(begin)
        , reference1(begin)
        , reference2(begin)
        , reference3(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 50103, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 393664};
    it begin() const { return endianess.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t endianess;
    #else
    field<uint16_t, sc_uint16, 0, it> endianess;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t n_shp_samples;
    #else
    field<uint8_t, sc_uint8, 16, it> n_shp_samples;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t R;
    #else
    field<uint32_t, sc_uint32, 32, it> R;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t B;
    #else
    field<uint32_t, sc_uint32, 64, it> B;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t K;
    #else
    field<uint32_t, sc_uint32, 96, it> K;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t P;
    #else
    field<uint32_t, sc_uint32, 128, it> P;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t N;
    #else
    field<uint32_t, sc_uint32, 160, it> N;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t sum_shp_samples;
    #else
    field<uint32_t, sc_uint32, 192, it> sum_shp_samples;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t db_start;
    #else
    field<uint32_t, sc_uint32, 224, it> db_start;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t refpuls_position;
    #else
    field<uint32_t, sc_uint32, 256, it> refpuls_position;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t avg_npulse;
    #else
    field<uint32_t, sc_uint32, 288, it> avg_npulse;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t reference0;
    #else
    field<uint32_t, sc_uint32, 320, it> reference0;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t reference1;
    #else
    field<uint32_t, sc_uint32, 352, it> reference1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t reference2;
    #else
    field<uint32_t, sc_uint32, 384, it> reference2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t reference3;
    #else
    field<uint32_t, sc_uint32, 416, it> reference3;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 12288 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sums(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint32_t sums;
        #else
        field<uint32_t, sc_uint32, 0, it> sums;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[12288];
    #else
    sequence<calib_waveform_L2, 32, 448, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    calib_waveform_L2& operator=(const calib_waveform_L2<ito>& o) {
        endianess = o.endianess;
        n_shp_samples = o.n_shp_samples;
        R = o.R;
        B = o.B;
        K = o.K;
        P = o.P;
        N = o.N;
        sum_shp_samples = o.sum_shp_samples;
        db_start = o.db_start;
        refpuls_position = o.refpuls_position;
        avg_npulse = o.avg_npulse;
        reference0 = o.reference0;
        reference1 = o.reference1;
        reference2 = o.reference2;
        reference3 = o.reference3;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].sums = o.data[n].sums;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct calib_waveform_L2<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef calib_waveform_L2<rebind_it> type;
    };

    enum { id_main = 50103, id_sub = 0};

    uint16_t         endianess;
    uint8_t          n_shp_samples;
    uint32_t         R;
    uint32_t         B;
    uint32_t         K;
    uint32_t         P;
    uint32_t         N;
    uint32_t         sum_shp_samples;
    uint32_t         db_start;
    uint32_t         refpuls_position;
    uint32_t         avg_npulse;
    uint32_t         reference0;
    uint32_t         reference1;
    uint32_t         reference2;
    uint32_t         reference3;
    std::size_t data_size;
    enum { data_max_size = 12288 };
    struct sequence_definition {
        uint32_t         sums;
    } data[12288];

    calib_waveform_L2() {}
    template<class it>
    calib_waveform_L2(const calib_waveform_L2<it>& o) {
        endianess = o.endianess;
        n_shp_samples = o.n_shp_samples;
        R = o.R;
        B = o.B;
        K = o.K;
        P = o.P;
        N = o.N;
        sum_shp_samples = o.sum_shp_samples;
        db_start = o.db_start;
        refpuls_position = o.refpuls_position;
        avg_npulse = o.avg_npulse;
        reference0 = o.reference0;
        reference1 = o.reference1;
        reference2 = o.reference2;
        reference3 = o.reference3;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].sums = o.data[n].sums;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const calib_waveform_L2<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.endianess;
        s << ", " << x.n_shp_samples;
        s << ", " << x.R;
        s << ", " << x.B;
        s << ", " << x.K;
        s << ", " << x.P;
        s << ", " << x.N;
        s << ", " << x.sum_shp_samples;
        s << ", " << x.db_start;
        s << ", " << x.refpuls_position;
        s << ", " << x.avg_npulse;
        s << ", " << x.reference0;
        s << ", " << x.reference1;
        s << ", " << x.reference2;
        s << ", " << x.reference3;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].sums
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, calib_waveform_L2<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.endianess;
        s >> ',' >> x.n_shp_samples;
        s >> ',' >> x.R;
        s >> ',' >> x.B;
        s >> ',' >> x.K;
        s >> ',' >> x.P;
        s >> ',' >> x.N;
        s >> ',' >> x.sum_shp_samples;
        s >> ',' >> x.db_start;
        s >> ',' >> x.refpuls_position;
        s >> ',' >> x.avg_npulse;
        s >> ',' >> x.reference0;
        s >> ',' >> x.reference1;
        s >> ',' >> x.reference2;
        s >> ',' >> x.reference3;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].sums)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct channel_combination_table
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef channel_combination_table<rebind_it> type;
    };

    typedef it iterator_type;

    channel_combination_table(it begin, it end, bool dirty=false)
        : limits(begin, end, limits_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 56, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 512};
    it begin() const { return limits.begin(); }
    it end() const { return limits.end(); }

    #endif //DOXYGEN

    std::size_t limits_size;
    enum { limits_max_size = 16 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : source(begin, begin_bit)
            , destination(begin, begin_bit)
            , minmax(begin, begin_bit)
            , limit(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint8_t source;
        #else
        field<uint8_t, sc_uint8, 0, it> source;
        #endif
        #ifdef DOXYGEN
        uint8_t destination;
        #else
        field<uint8_t, sc_uint8, 8, it> destination;
        #endif
        #ifdef DOXYGEN
        uint8_t minmax;
        #else
        field<uint8_t, sc_bit, 16, it> minmax;
        #endif
        #ifdef DOXYGEN
        uint16_t limit;
        #else
        field<uint16_t, sc_uint15, 17, it> limit;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition limits[16];
    #else
    sequence<channel_combination_table, 32, 0, it> limits;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    channel_combination_table& operator=(const channel_combination_table<ito>& o) {
        limits_size = o.limits_size;
        for(unsigned n=0; n<limits_size; ++n){
            limits[n].source = o.limits[n].source;
            limits[n].destination = o.limits[n].destination;
            limits[n].minmax = o.limits[n].minmax;
            limits[n].limit = o.limits[n].limit;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct channel_combination_table<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef channel_combination_table<rebind_it> type;
    };

    enum { id_main = 56, id_sub = 0};

    std::size_t limits_size;
    enum { limits_max_size = 16 };
    struct sequence_definition {
        uint8_t          source;
        uint8_t          destination;
        uint8_t          minmax;
        uint16_t         limit;
    } limits[16];

    channel_combination_table() {}
    template<class it>
    channel_combination_table(const channel_combination_table<it>& o) {
        limits_size = o.limits.size();
        for(unsigned n=0; n<limits_size; ++n){
            limits[n].source = o.limits[n].source;
            limits[n].destination = o.limits[n].destination;
            limits[n].minmax = o.limits[n].minmax;
            limits[n].limit = o.limits[n].limit;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const channel_combination_table<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {

        s << ", [";
        for (std::size_t n=0; n<x.limits_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.limits[n].source
            << ", " << x.limits[n].destination
            << ", " << x.limits[n].minmax
            << ", " << x.limits[n].limit
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, channel_combination_table<it>& x) {
    package_istream_entry ok(s);
    if (ok) {

        s >> ',' >> '[' >> std::ws;
        x.limits_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.limits_size;
            if (s
                >> '{' >> (x.limits[x.limits_size-1].source)
                >> ',' >> (x.limits[x.limits_size-1].destination)
                >> ',' >> (x.limits[x.limits_size-1].minmax)
                >> ',' >> (x.limits[x.limits_size-1].limit)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct context_end
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef context_end<rebind_it> type;
    };

    typedef it iterator_type;

    context_end(it begin, it end, bool dirty=false)
        : sub(begin)
        , main(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 21, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 32};
    it begin() const { return sub.begin(); }
    it end() const { return main.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t sub;
    #else
    field<uint16_t, sc_uint16, 0, it> sub;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t main;
    #else
    field<uint16_t, sc_uint16, 16, it> main;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    context_end& operator=(const context_end<ito>& o) {
        sub = o.sub;
        main = o.main;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct context_end<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef context_end<rebind_it> type;
    };

    enum { id_main = 21, id_sub = 1};

    uint16_t         sub;
    uint16_t         main;

    context_end() {}
    template<class it>
    context_end(const context_end<it>& o) {
        sub = o.sub;
        main = o.main;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const context_end<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.sub;
        s << ", " << x.main;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, context_end<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.sub;
        s >> ',' >> x.main;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! external synchronization input

//! <para>This package belongs to the predefined selectors:</para>
//! <para>data, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct counter_sync
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef counter_sync<rebind_it> type;
    };

    typedef it iterator_type;

    counter_sync(it begin, it end, bool dirty=false)
        : systime(begin)
        , count(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 13, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 64};
    it begin() const { return systime.begin(); }
    it end() const { return count.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  internal time stamp of external event in units of units.time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t count;//!<  number of events of the external event input 
    #else
    field<uint32_t, sc_uint32, 32, it> count;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    counter_sync& operator=(const counter_sync<ito>& o) {
        systime = o.systime;
        count = o.count;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct counter_sync<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef counter_sync<rebind_it> type;
    };

    enum { id_main = 13, id_sub = 0};

    uint32_t         systime;
    uint32_t         count;

    counter_sync() {}
    template<class it>
    counter_sync(const counter_sync<it>& o) {
        systime = o.systime;
        count = o.count;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const counter_sync<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.count;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, counter_sync<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.count;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct crc32_check
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef crc32_check<rebind_it> type;
    };

    typedef it iterator_type;

    crc32_check(it begin, it end, bool dirty=false)
        : stream_id(begin)
        , sequence(begin)
        , crc(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 50, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 64};
    it begin() const { return stream_id.begin(); }
    it end() const { return crc.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t stream_id;
    #else
    field<uint8_t, sc_uint8, 0, it> stream_id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t sequence;
    #else
    field<uint32_t, sc_uint24, 8, it> sequence;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t crc;
    #else
    field<uint32_t, sc_uint32, 32, it> crc;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    crc32_check& operator=(const crc32_check<ito>& o) {
        stream_id = o.stream_id;
        sequence = o.sequence;
        crc = o.crc;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct crc32_check<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef crc32_check<rebind_it> type;
    };

    enum { id_main = 50, id_sub = 0};

    uint8_t          stream_id;
    uint32_t         sequence;
    uint32_t         crc;

    crc32_check() {}
    template<class it>
    crc32_check(const crc32_check<it>& o) {
        stream_id = o.stream_id;
        sequence = o.sequence;
        crc = o.crc;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const crc32_check<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.stream_id;
        s << ", " << x.sequence;
        s << ", " << x.crc;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, crc32_check<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.stream_id;
        s >> ',' >> x.sequence;
        s >> ',' >> x.crc;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct crc32_header
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef crc32_header<rebind_it> type;
    };

    typedef it iterator_type;

    crc32_header(it begin, it end, bool dirty=false)
        : stream_id(begin)
        , package_ids(begin, end, package_ids_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 49, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 8160};
    it begin() const { return stream_id.begin(); }
    it end() const { return package_ids.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t stream_id;
    #else
    field<uint8_t, sc_uint8, 0, it> stream_id;
    #endif //DOXYGEN

    std::size_t package_ids_size;
    enum { package_ids_max_size = 254 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sub(begin, begin_bit)
            , main(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint16_t sub;
        #else
        field<uint16_t, sc_uint16, 0, it> sub;
        #endif
        #ifdef DOXYGEN
        uint16_t main;
        #else
        field<uint16_t, sc_uint16, 16, it> main;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition package_ids[254];
    #else
    sequence<crc32_header, 32, 32, it> package_ids;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    crc32_header& operator=(const crc32_header<ito>& o) {
        stream_id = o.stream_id;
        package_ids_size = o.package_ids_size;
        for(unsigned n=0; n<package_ids_size; ++n){
            package_ids[n].sub = o.package_ids[n].sub;
            package_ids[n].main = o.package_ids[n].main;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct crc32_header<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef crc32_header<rebind_it> type;
    };

    enum { id_main = 49, id_sub = 0};

    uint8_t          stream_id;
    std::size_t package_ids_size;
    enum { package_ids_max_size = 254 };
    struct sequence_definition {
        uint16_t         sub;
        uint16_t         main;
    } package_ids[254];

    crc32_header() {}
    template<class it>
    crc32_header(const crc32_header<it>& o) {
        stream_id = o.stream_id;
        package_ids_size = o.package_ids.size();
        for(unsigned n=0; n<package_ids_size; ++n){
            package_ids[n].sub = o.package_ids[n].sub;
            package_ids[n].main = o.package_ids[n].main;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const crc32_header<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.stream_id;

        s << ", [";
        for (std::size_t n=0; n<x.package_ids_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.package_ids[n].sub
            << ", " << x.package_ids[n].main
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, crc32_header<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.stream_id;

        s >> ',' >> '[' >> std::ws;
        x.package_ids_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.package_ids_size;
            if (s
                >> '{' >> (x.package_ids[x.package_ids_size-1].sub)
                >> ',' >> (x.package_ids[x.package_ids_size-1].main)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct datagram_separator
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef datagram_separator<rebind_it> type;
    };

    typedef it iterator_type;

    datagram_separator(it begin, it end, bool dirty=false)
        : sequence_number(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 65226, id_sub = 60926};
    #ifndef DOXYGEN
    enum { max_bit_width = 32};
    it begin() const { return sequence_number.begin(); }
    it end() const { return sequence_number.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t sequence_number;
    #else
    field<uint32_t, sc_uint32, 0, it> sequence_number;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    datagram_separator& operator=(const datagram_separator<ito>& o) {
        sequence_number = o.sequence_number;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct datagram_separator<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef datagram_separator<rebind_it> type;
    };

    enum { id_main = 65226, id_sub = 60926};

    uint32_t         sequence_number;

    datagram_separator() {}
    template<class it>
    datagram_separator(const datagram_separator<it>& o) {
        sequence_number = o.sequence_number;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const datagram_separator<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.sequence_number;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, datagram_separator<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.sequence_number;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct debug_hw_dg
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef debug_hw_dg<rebind_it> type;
    };

    typedef it iterator_type;

    debug_hw_dg(it begin, it end, bool dirty=false)
        : packet_type(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 50101, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 8224};
    it begin() const { return packet_type.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t packet_type;
    #else
    field<uint32_t, sc_uint32, 0, it> packet_type;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 256 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : value(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint32_t value;
        #else
        field<uint32_t, sc_uint32, 0, it> value;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[256];
    #else
    sequence<debug_hw_dg, 32, 32, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    debug_hw_dg& operator=(const debug_hw_dg<ito>& o) {
        packet_type = o.packet_type;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].value = o.data[n].value;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct debug_hw_dg<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef debug_hw_dg<rebind_it> type;
    };

    enum { id_main = 50101, id_sub = 0};

    uint32_t         packet_type;
    std::size_t data_size;
    enum { data_max_size = 256 };
    struct sequence_definition {
        uint32_t         value;
    } data[256];

    debug_hw_dg() {}
    template<class it>
    debug_hw_dg(const debug_hw_dg<it>& o) {
        packet_type = o.packet_type;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].value = o.data[n].value;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const debug_hw_dg<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.packet_type;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].value
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, debug_hw_dg<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.packet_type;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].value)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct debug_sw_dg
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef debug_sw_dg<rebind_it> type;
    };

    typedef it iterator_type;

    debug_sw_dg(it begin, it end, bool dirty=false)
        : packet_type(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 50102, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 8224};
    it begin() const { return packet_type.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t packet_type;
    #else
    field<uint32_t, sc_uint32, 0, it> packet_type;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 256 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : value(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint32_t value;
        #else
        field<uint32_t, sc_uint32, 0, it> value;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[256];
    #else
    sequence<debug_sw_dg, 32, 32, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    debug_sw_dg& operator=(const debug_sw_dg<ito>& o) {
        packet_type = o.packet_type;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].value = o.data[n].value;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct debug_sw_dg<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef debug_sw_dg<rebind_it> type;
    };

    enum { id_main = 50102, id_sub = 0};

    uint32_t         packet_type;
    std::size_t data_size;
    enum { data_max_size = 256 };
    struct sequence_definition {
        uint32_t         value;
    } data[256];

    debug_sw_dg() {}
    template<class it>
    debug_sw_dg(const debug_sw_dg<it>& o) {
        packet_type = o.packet_type;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].value = o.data[n].value;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const debug_sw_dg<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.packet_type;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].value
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, debug_sw_dg<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.packet_type;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].value)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct device_geometry
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef device_geometry<rebind_it> type;
    };

    typedef it iterator_type;

    device_geometry(it begin, it end, bool dirty=false)
        : laser_origin(begin)
        , laser_direction(begin)
        , mirror_axis_origin(begin)
        , mirror_axis_direction(begin)
        , line_angle_0(begin)
        , num_facets(begin)
        , facet(begin, end, facet_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 4, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 1448};
    it begin() const { return laser_origin.begin(); }
    it end() const { return facet.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float laser_origin[3];
    #else
    array<float, 3, sc_float32, 0, it> laser_origin;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float laser_direction[3];
    #else
    array<float, 3, sc_float32, 96, it> laser_direction;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float mirror_axis_origin[3];
    #else
    array<float, 3, sc_float32, 192, it> mirror_axis_origin;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float mirror_axis_direction[3];
    #else
    array<float, 3, sc_float32, 288, it> mirror_axis_direction;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t line_angle_0;
    #else
    field<int32_t, sc_int32, 384, it> line_angle_0;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t num_facets;
    #else
    field<uint8_t, sc_uint8, 416, it> num_facets;
    #endif //DOXYGEN

    std::size_t facet_size;
    enum { facet_max_size = 8 };
    //! facet orientation

    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : nx(begin, begin_bit)
            , ny(begin, begin_bit)
            , nz(begin, begin_bit)
            , d(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        float nx;//!<  x component of facet normal 
        #else
        field<float, sc_float32, 0, it> nx;
        #endif
        #ifdef DOXYGEN
        float ny;//!<  y component of facet normal 
        #else
        field<float, sc_float32, 32, it> ny;
        #endif
        #ifdef DOXYGEN
        float nz;//!<  z component of facet normal 
        #else
        field<float, sc_float32, 64, it> nz;
        #endif
        #ifdef DOXYGEN
        float d;//!<  facet distance to origin 
        #else
        field<float, sc_float32, 96, it> d;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition facet[8];
    #else
    sequence<device_geometry, 128, 424, it> facet;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    device_geometry& operator=(const device_geometry<ito>& o) {
        for(unsigned n=0; n<3; ++n) laser_origin[n] = o.laser_origin[n];
        for(unsigned n=0; n<3; ++n) laser_direction[n] = o.laser_direction[n];
        for(unsigned n=0; n<3; ++n) mirror_axis_origin[n] = o.mirror_axis_origin[n];
        for(unsigned n=0; n<3; ++n) mirror_axis_direction[n] = o.mirror_axis_direction[n];
        line_angle_0 = o.line_angle_0;
        num_facets = o.num_facets;
        facet_size = o.facet_size;
        for(unsigned n=0; n<facet_size; ++n){
            facet[n].nx = o.facet[n].nx;
            facet[n].ny = o.facet[n].ny;
            facet[n].nz = o.facet[n].nz;
            facet[n].d = o.facet[n].d;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct device_geometry<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef device_geometry<rebind_it> type;
    };

    enum { id_main = 4, id_sub = 0};

    float            laser_origin[3];
    float            laser_direction[3];
    float            mirror_axis_origin[3];
    float            mirror_axis_direction[3];
    int32_t          line_angle_0;
    uint8_t          num_facets;
    std::size_t facet_size;
    enum { facet_max_size = 8 };
    struct sequence_definition {
        float            nx;
        float            ny;
        float            nz;
        float            d;
    } facet[8];

    device_geometry() {}
    template<class it>
    device_geometry(const device_geometry<it>& o) {
        for(unsigned n=0; n<3; ++n) laser_origin[n] = o.laser_origin[n];
        for(unsigned n=0; n<3; ++n) laser_direction[n] = o.laser_direction[n];
        for(unsigned n=0; n<3; ++n) mirror_axis_origin[n] = o.mirror_axis_origin[n];
        for(unsigned n=0; n<3; ++n) mirror_axis_direction[n] = o.mirror_axis_direction[n];
        line_angle_0 = o.line_angle_0;
        num_facets = o.num_facets;
        facet_size = o.facet.size();
        for(unsigned n=0; n<facet_size; ++n){
            facet[n].nx = o.facet[n].nx;
            facet[n].ny = o.facet[n].ny;
            facet[n].nz = o.facet[n].nz;
            facet[n].d = o.facet[n].d;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const device_geometry<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", "; write_array(s, 3, x.laser_origin);
        s << ", "; write_array(s, 3, x.laser_direction);
        s << ", "; write_array(s, 3, x.mirror_axis_origin);
        s << ", "; write_array(s, 3, x.mirror_axis_direction);
        s << ", " << x.line_angle_0;
        s << ", " << x.num_facets;

        s << ", [";
        for (std::size_t n=0; n<x.facet_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.facet[n].nx
            << ", " << x.facet[n].ny
            << ", " << x.facet[n].nz
            << ", " << x.facet[n].d
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, device_geometry<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ','; read_array(s, 3, x.laser_origin);
        s >> ','; read_array(s, 3, x.laser_direction);
        s >> ','; read_array(s, 3, x.mirror_axis_origin);
        s >> ','; read_array(s, 3, x.mirror_axis_direction);
        s >> ',' >> x.line_angle_0;
        s >> ',' >> x.num_facets;

        s >> ',' >> '[' >> std::ws;
        x.facet_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.facet_size;
            if (s
                >> '{' >> (x.facet[x.facet_size-1].nx)
                >> ',' >> (x.facet[x.facet_size-1].ny)
                >> ',' >> (x.facet[x.facet_size-1].nz)
                >> ',' >> (x.facet[x.facet_size-1].d)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! geometrical parameters of external devices

//! <para>This package belongs to the predefined selectors:</para>
//! <para>attribute, instrument</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct device_mounting
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef device_mounting<rebind_it> type;
    };

    typedef it iterator_type;

    device_mounting(it begin, it end, bool dirty=false)
        : base_ofs(begin)
        , bat_ofs(begin)
        , gps_ofs(begin)
        , socnb_ofs(begin)
        , soc_ofs(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 30, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 160};
    it begin() const { return base_ofs.begin(); }
    it end() const { return soc_ofs.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float base_ofs;//!<  height of tripod base plate over soil or pavement [m] 
    #else
    field<float, sc_float32, 0, it> base_ofs;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float bat_ofs;//!<  INTERNAL ONLY  the thickness of the battery pack [m] 
    #else
    field<float, sc_float32, 32, it> bat_ofs;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float gps_ofs;//!<  position of GPS antenna in z-axis direction in scanners coordinate system [m] 
    #else
    field<float, sc_float32, 64, it> gps_ofs;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float socnb_ofs;//!<  INTERNAL ONLY  z-axis distance between scanner base plate (WITHOUT BATTERY PACK) and SOC origin in [m] 
    #else
    field<float, sc_float32, 96, it> socnb_ofs;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float soc_ofs;//!<  height of scanners coordinate system origin over tripod base plate [m] 
    #else
    field<float, sc_float32, 128, it> soc_ofs;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    device_mounting& operator=(const device_mounting<ito>& o) {
        base_ofs = o.base_ofs;
        bat_ofs = o.bat_ofs;
        gps_ofs = o.gps_ofs;
        socnb_ofs = o.socnb_ofs;
        soc_ofs = o.soc_ofs;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct device_mounting<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef device_mounting<rebind_it> type;
    };

    enum { id_main = 30, id_sub = 0};

    float            base_ofs;
    float            bat_ofs;
    float            gps_ofs;
    float            socnb_ofs;
    float            soc_ofs;

    device_mounting() {}
    template<class it>
    device_mounting(const device_mounting<it>& o) {
        base_ofs = o.base_ofs;
        bat_ofs = o.bat_ofs;
        gps_ofs = o.gps_ofs;
        socnb_ofs = o.socnb_ofs;
        soc_ofs = o.soc_ofs;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const device_mounting<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.base_ofs;
        s << ", " << x.bat_ofs;
        s << ", " << x.gps_ofs;
        s << ", " << x.socnb_ofs;
        s << ", " << x.soc_ofs;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, device_mounting<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.base_ofs;
        s >> ',' >> x.bat_ofs;
        s >> ',' >> x.gps_ofs;
        s >> ',' >> x.socnb_ofs;
        s >> ',' >> x.soc_ofs;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct echo
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef echo<rebind_it> type;
    };

    typedef it iterator_type;

    echo(it begin, it end, bool dirty=false)
        : range(begin)
        , ampl(begin)
        , refl(begin)
        , flags(begin)
        , dev(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 42, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 96};
    it begin() const { return range.begin(); }
    it end() const { return dev.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t range;
    #else
    field<int32_t, sc_int32, 0, it> range;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t ampl;
    #else
    field<uint16_t, sc_uint16, 32, it> ampl;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t refl;
    #else
    field<int16_t, sc_int16, 48, it> refl;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t flags;
    #else
    field<uint16_t, sc_uint16, 64, it> flags;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t dev;
    #else
    field<uint16_t, sc_uint16, 80, it> dev;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    echo& operator=(const echo<ito>& o) {
        range = o.range;
        ampl = o.ampl;
        refl = o.refl;
        flags = o.flags;
        dev = o.dev;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct echo<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef echo<rebind_it> type;
    };

    enum { id_main = 42, id_sub = 0};

    int32_t          range;
    uint16_t         ampl;
    int16_t          refl;
    uint16_t         flags;
    uint16_t         dev;

    echo() {}
    template<class it>
    echo(const echo<it>& o) {
        range = o.range;
        ampl = o.ampl;
        refl = o.refl;
        flags = o.flags;
        dev = o.dev;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const echo<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.range;
        s << ", " << x.ampl;
        s << ", " << x.refl;
        s << ", " << x.flags;
        s << ", " << x.dev;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, echo<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.range;
        s >> ',' >> x.ampl;
        s >> ',' >> x.refl;
        s >> ',' >> x.flags;
        s >> ',' >> x.dev;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! extents of various data fields

//! <para>This package belongs to the predefined selectors:</para>
//! <para>attribute, instrument</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct extents
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef extents<rebind_it> type;
    };

    typedef it iterator_type;

    extents(it begin, it end, bool dirty=false)
        : range_min(begin)
        , range_max(begin)
        , amplitude_min(begin)
        , amplitude_max(begin)
        , reflectance_min(begin)
        , reflectance_max(begin)
        , systime_bits(begin)
        , backgnd_rad_min(begin)
        , backgnd_rad_max(begin)
        , dev_min(begin)
        , dev_max(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 57, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 272};
    it begin() const { return range_min.begin(); }
    it end() const { return dev_max.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float range_min;//!<  minimum and maximum possible 
    #else
    field<float, sc_float32, 0, it> range_min;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float range_max;//!<  range in meter 
    #else
    field<float, sc_float32, 32, it> range_max;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float amplitude_min;//!<  minimum and maximum possible 
    #else
    field<float, sc_float32, 64, it> amplitude_min;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float amplitude_max;//!<  amplitudes in dB 
    #else
    field<float, sc_float32, 96, it> amplitude_max;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float reflectance_min;//!<  minimum and maximum possible 
    #else
    field<float, sc_float32, 128, it> reflectance_min;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float reflectance_max;//!<  reflectance in dB 
    #else
    field<float, sc_float32, 160, it> reflectance_max;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t systime_bits;//!<  significant bits of systime stamp 
    #else
    field<uint16_t, sc_uint16, 192, it> systime_bits;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t backgnd_rad_min;//!<  minimum and maximum possible 
    #else
    field<uint16_t, sc_uint16, 208, it> backgnd_rad_min;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t backgnd_rad_max;//!<  background radiation 
    #else
    field<uint16_t, sc_uint16, 224, it> backgnd_rad_max;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t dev_min;//!<  minimum and maximum possible 
    #else
    field<uint16_t, sc_uint16, 240, it> dev_min;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t dev_max;//!<  deviation from echo shape 
    #else
    field<uint16_t, sc_uint16, 256, it> dev_max;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    extents& operator=(const extents<ito>& o) {
        range_min = o.range_min;
        range_max = o.range_max;
        amplitude_min = o.amplitude_min;
        amplitude_max = o.amplitude_max;
        reflectance_min = o.reflectance_min;
        reflectance_max = o.reflectance_max;
        systime_bits = o.systime_bits;
        backgnd_rad_min = o.backgnd_rad_min;
        backgnd_rad_max = o.backgnd_rad_max;
        dev_min = o.dev_min;
        dev_max = o.dev_max;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct extents<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef extents<rebind_it> type;
    };

    enum { id_main = 57, id_sub = 0};

    float            range_min;
    float            range_max;
    float            amplitude_min;
    float            amplitude_max;
    float            reflectance_min;
    float            reflectance_max;
    uint16_t         systime_bits;
    uint16_t         backgnd_rad_min;
    uint16_t         backgnd_rad_max;
    uint16_t         dev_min;
    uint16_t         dev_max;

    extents() {}
    template<class it>
    extents(const extents<it>& o) {
        range_min = o.range_min;
        range_max = o.range_max;
        amplitude_min = o.amplitude_min;
        amplitude_max = o.amplitude_max;
        reflectance_min = o.reflectance_min;
        reflectance_max = o.reflectance_max;
        systime_bits = o.systime_bits;
        backgnd_rad_min = o.backgnd_rad_min;
        backgnd_rad_max = o.backgnd_rad_max;
        dev_min = o.dev_min;
        dev_max = o.dev_max;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const extents<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.range_min;
        s << ", " << x.range_max;
        s << ", " << x.amplitude_min;
        s << ", " << x.amplitude_max;
        s << ", " << x.reflectance_min;
        s << ", " << x.reflectance_max;
        s << ", " << x.systime_bits;
        s << ", " << x.backgnd_rad_min;
        s << ", " << x.backgnd_rad_max;
        s << ", " << x.dev_min;
        s << ", " << x.dev_max;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, extents<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.range_min;
        s >> ',' >> x.range_max;
        s >> ',' >> x.amplitude_min;
        s >> ',' >> x.amplitude_max;
        s >> ',' >> x.reflectance_min;
        s >> ',' >> x.reflectance_max;
        s >> ',' >> x.systime_bits;
        s >> ',' >> x.backgnd_rad_min;
        s >> ',' >> x.backgnd_rad_max;
        s >> ',' >> x.dev_min;
        s >> ',' >> x.dev_max;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct firmware
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef firmware<rebind_it> type;
    };

    typedef it iterator_type;

    firmware(it begin, it end, bool dirty=false)
        : id(begin)
        , v_uboot(begin)
        , v_linux(begin)
        , v_ctrlproc(begin)
        , v_scnproc(begin)
        , v_rcvpic(begin)
        , v_eslpic(begin)
        , v_mb_fpga(begin)
        , v_acq_fpga(begin)
        , v_acq_commpb(begin)
        , v_acq_rprpb(begin)
        , v_acq_dlec(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 8, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 3072};
    it begin() const { return id.begin(); }
    it end() const { return v_acq_dlec.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    char id[64];
    #else
    array<char, 64, sc_char, 0, it> id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_uboot[64];
    #else
    array<char, 64, sc_char, 512, it> v_uboot;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_linux[16];
    #else
    array<char, 16, sc_char, 1024, it> v_linux;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_ctrlproc[64];
    #else
    array<char, 64, sc_char, 1152, it> v_ctrlproc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_scnproc[64];
    #else
    array<char, 64, sc_char, 1664, it> v_scnproc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_rcvpic[16];
    #else
    array<char, 16, sc_char, 2176, it> v_rcvpic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_eslpic[16];
    #else
    array<char, 16, sc_char, 2304, it> v_eslpic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_mb_fpga[16];
    #else
    array<char, 16, sc_char, 2432, it> v_mb_fpga;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_fpga[16];
    #else
    array<char, 16, sc_char, 2560, it> v_acq_fpga;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_commpb[16];
    #else
    array<char, 16, sc_char, 2688, it> v_acq_commpb;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_rprpb[16];
    #else
    array<char, 16, sc_char, 2816, it> v_acq_rprpb;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_dlec[16];
    #else
    array<char, 16, sc_char, 2944, it> v_acq_dlec;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    firmware& operator=(const firmware<ito>& o) {
        for(unsigned n=0; n<64; ++n) id[n] = o.id[n];
        for(unsigned n=0; n<64; ++n) v_uboot[n] = o.v_uboot[n];
        for(unsigned n=0; n<16; ++n) v_linux[n] = o.v_linux[n];
        for(unsigned n=0; n<64; ++n) v_ctrlproc[n] = o.v_ctrlproc[n];
        for(unsigned n=0; n<64; ++n) v_scnproc[n] = o.v_scnproc[n];
        for(unsigned n=0; n<16; ++n) v_rcvpic[n] = o.v_rcvpic[n];
        for(unsigned n=0; n<16; ++n) v_eslpic[n] = o.v_eslpic[n];
        for(unsigned n=0; n<16; ++n) v_mb_fpga[n] = o.v_mb_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_fpga[n] = o.v_acq_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_commpb[n] = o.v_acq_commpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_rprpb[n] = o.v_acq_rprpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_dlec[n] = o.v_acq_dlec[n];
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct firmware<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef firmware<rebind_it> type;
    };

    enum { id_main = 8, id_sub = 0};

    char             id[64];
    char             v_uboot[64];
    char             v_linux[16];
    char             v_ctrlproc[64];
    char             v_scnproc[64];
    char             v_rcvpic[16];
    char             v_eslpic[16];
    char             v_mb_fpga[16];
    char             v_acq_fpga[16];
    char             v_acq_commpb[16];
    char             v_acq_rprpb[16];
    char             v_acq_dlec[16];

    firmware() {}
    template<class it>
    firmware(const firmware<it>& o) {
        for(unsigned n=0; n<64; ++n) id[n] = o.id[n];
        for(unsigned n=0; n<64; ++n) v_uboot[n] = o.v_uboot[n];
        for(unsigned n=0; n<16; ++n) v_linux[n] = o.v_linux[n];
        for(unsigned n=0; n<64; ++n) v_ctrlproc[n] = o.v_ctrlproc[n];
        for(unsigned n=0; n<64; ++n) v_scnproc[n] = o.v_scnproc[n];
        for(unsigned n=0; n<16; ++n) v_rcvpic[n] = o.v_rcvpic[n];
        for(unsigned n=0; n<16; ++n) v_eslpic[n] = o.v_eslpic[n];
        for(unsigned n=0; n<16; ++n) v_mb_fpga[n] = o.v_mb_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_fpga[n] = o.v_acq_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_commpb[n] = o.v_acq_commpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_rprpb[n] = o.v_acq_rprpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_dlec[n] = o.v_acq_dlec[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const firmware<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", "; write_array(s, 64, x.id);
        s << ", "; write_array(s, 64, x.v_uboot);
        s << ", "; write_array(s, 16, x.v_linux);
        s << ", "; write_array(s, 64, x.v_ctrlproc);
        s << ", "; write_array(s, 64, x.v_scnproc);
        s << ", "; write_array(s, 16, x.v_rcvpic);
        s << ", "; write_array(s, 16, x.v_eslpic);
        s << ", "; write_array(s, 16, x.v_mb_fpga);
        s << ", "; write_array(s, 16, x.v_acq_fpga);
        s << ", "; write_array(s, 16, x.v_acq_commpb);
        s << ", "; write_array(s, 16, x.v_acq_rprpb);
        s << ", "; write_array(s, 16, x.v_acq_dlec);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, firmware<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ','; read_array(s, 64, x.id);
        s >> ','; read_array(s, 64, x.v_uboot);
        s >> ','; read_array(s, 16, x.v_linux);
        s >> ','; read_array(s, 64, x.v_ctrlproc);
        s >> ','; read_array(s, 64, x.v_scnproc);
        s >> ','; read_array(s, 16, x.v_rcvpic);
        s >> ','; read_array(s, 16, x.v_eslpic);
        s >> ','; read_array(s, 16, x.v_mb_fpga);
        s >> ','; read_array(s, 16, x.v_acq_fpga);
        s >> ','; read_array(s, 16, x.v_acq_commpb);
        s >> ','; read_array(s, 16, x.v_acq_rprpb);
        s >> ','; read_array(s, 16, x.v_acq_dlec);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct firmware_1
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef firmware_1<rebind_it> type;
    };

    typedef it iterator_type;

    firmware_1(it begin, it end, bool dirty=false)
        : id(begin)
        , v_uboot(begin)
        , v_linux(begin)
        , v_ctrlproc(begin)
        , v_scnproc(begin)
        , v_rcvpic(begin)
        , v_eslpic(begin)
        , v_mb_fpga(begin)
        , v_acq_fpga(begin)
        , v_acq_commpb(begin)
        , v_acq_rprpb(begin)
        , v_acq_dlec(begin)
        , v_inclpic(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 8, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 3200};
    it begin() const { return id.begin(); }
    it end() const { return v_inclpic.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    char id[64];
    #else
    array<char, 64, sc_char, 0, it> id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_uboot[64];
    #else
    array<char, 64, sc_char, 512, it> v_uboot;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_linux[16];
    #else
    array<char, 16, sc_char, 1024, it> v_linux;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_ctrlproc[64];
    #else
    array<char, 64, sc_char, 1152, it> v_ctrlproc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_scnproc[64];
    #else
    array<char, 64, sc_char, 1664, it> v_scnproc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_rcvpic[16];
    #else
    array<char, 16, sc_char, 2176, it> v_rcvpic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_eslpic[16];
    #else
    array<char, 16, sc_char, 2304, it> v_eslpic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_mb_fpga[16];
    #else
    array<char, 16, sc_char, 2432, it> v_mb_fpga;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_fpga[16];
    #else
    array<char, 16, sc_char, 2560, it> v_acq_fpga;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_commpb[16];
    #else
    array<char, 16, sc_char, 2688, it> v_acq_commpb;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_rprpb[16];
    #else
    array<char, 16, sc_char, 2816, it> v_acq_rprpb;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_dlec[16];
    #else
    array<char, 16, sc_char, 2944, it> v_acq_dlec;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_inclpic[16];
    #else
    array<char, 16, sc_char, 3072, it> v_inclpic;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    firmware_1& operator=(const firmware_1<ito>& o) {
        for(unsigned n=0; n<64; ++n) id[n] = o.id[n];
        for(unsigned n=0; n<64; ++n) v_uboot[n] = o.v_uboot[n];
        for(unsigned n=0; n<16; ++n) v_linux[n] = o.v_linux[n];
        for(unsigned n=0; n<64; ++n) v_ctrlproc[n] = o.v_ctrlproc[n];
        for(unsigned n=0; n<64; ++n) v_scnproc[n] = o.v_scnproc[n];
        for(unsigned n=0; n<16; ++n) v_rcvpic[n] = o.v_rcvpic[n];
        for(unsigned n=0; n<16; ++n) v_eslpic[n] = o.v_eslpic[n];
        for(unsigned n=0; n<16; ++n) v_mb_fpga[n] = o.v_mb_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_fpga[n] = o.v_acq_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_commpb[n] = o.v_acq_commpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_rprpb[n] = o.v_acq_rprpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_dlec[n] = o.v_acq_dlec[n];
        for(unsigned n=0; n<16; ++n) v_inclpic[n] = o.v_inclpic[n];
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct firmware_1<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef firmware_1<rebind_it> type;
    };

    enum { id_main = 8, id_sub = 1};

    char             id[64];
    char             v_uboot[64];
    char             v_linux[16];
    char             v_ctrlproc[64];
    char             v_scnproc[64];
    char             v_rcvpic[16];
    char             v_eslpic[16];
    char             v_mb_fpga[16];
    char             v_acq_fpga[16];
    char             v_acq_commpb[16];
    char             v_acq_rprpb[16];
    char             v_acq_dlec[16];
    char             v_inclpic[16];

    firmware_1() {}
    template<class it>
    firmware_1(const firmware_1<it>& o) {
        for(unsigned n=0; n<64; ++n) id[n] = o.id[n];
        for(unsigned n=0; n<64; ++n) v_uboot[n] = o.v_uboot[n];
        for(unsigned n=0; n<16; ++n) v_linux[n] = o.v_linux[n];
        for(unsigned n=0; n<64; ++n) v_ctrlproc[n] = o.v_ctrlproc[n];
        for(unsigned n=0; n<64; ++n) v_scnproc[n] = o.v_scnproc[n];
        for(unsigned n=0; n<16; ++n) v_rcvpic[n] = o.v_rcvpic[n];
        for(unsigned n=0; n<16; ++n) v_eslpic[n] = o.v_eslpic[n];
        for(unsigned n=0; n<16; ++n) v_mb_fpga[n] = o.v_mb_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_fpga[n] = o.v_acq_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_commpb[n] = o.v_acq_commpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_rprpb[n] = o.v_acq_rprpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_dlec[n] = o.v_acq_dlec[n];
        for(unsigned n=0; n<16; ++n) v_inclpic[n] = o.v_inclpic[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const firmware_1<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", "; write_array(s, 64, x.id);
        s << ", "; write_array(s, 64, x.v_uboot);
        s << ", "; write_array(s, 16, x.v_linux);
        s << ", "; write_array(s, 64, x.v_ctrlproc);
        s << ", "; write_array(s, 64, x.v_scnproc);
        s << ", "; write_array(s, 16, x.v_rcvpic);
        s << ", "; write_array(s, 16, x.v_eslpic);
        s << ", "; write_array(s, 16, x.v_mb_fpga);
        s << ", "; write_array(s, 16, x.v_acq_fpga);
        s << ", "; write_array(s, 16, x.v_acq_commpb);
        s << ", "; write_array(s, 16, x.v_acq_rprpb);
        s << ", "; write_array(s, 16, x.v_acq_dlec);
        s << ", "; write_array(s, 16, x.v_inclpic);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, firmware_1<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ','; read_array(s, 64, x.id);
        s >> ','; read_array(s, 64, x.v_uboot);
        s >> ','; read_array(s, 16, x.v_linux);
        s >> ','; read_array(s, 64, x.v_ctrlproc);
        s >> ','; read_array(s, 64, x.v_scnproc);
        s >> ','; read_array(s, 16, x.v_rcvpic);
        s >> ','; read_array(s, 16, x.v_eslpic);
        s >> ','; read_array(s, 16, x.v_mb_fpga);
        s >> ','; read_array(s, 16, x.v_acq_fpga);
        s >> ','; read_array(s, 16, x.v_acq_commpb);
        s >> ','; read_array(s, 16, x.v_acq_rprpb);
        s >> ','; read_array(s, 16, x.v_acq_dlec);
        s >> ','; read_array(s, 16, x.v_inclpic);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct firmware_2
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef firmware_2<rebind_it> type;
    };

    typedef it iterator_type;

    firmware_2(it begin, it end, bool dirty=false)
        : id(begin)
        , v_uboot(begin)
        , v_linux(begin)
        , v_ctrlproc(begin)
        , v_scnproc(begin)
        , v_rcvpic(begin)
        , v_eslpic(begin)
        , v_mb_fpga(begin)
        , v_acq_fpga(begin)
        , v_acq_commpb(begin)
        , v_acq_rprpb(begin)
        , v_acq_dlec(begin)
        , v_inclpic(begin)
        , v_disppic(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 8, id_sub = 2};
    #ifndef DOXYGEN
    enum { max_bit_width = 3328};
    it begin() const { return id.begin(); }
    it end() const { return v_disppic.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    char id[64];
    #else
    array<char, 64, sc_char, 0, it> id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_uboot[64];
    #else
    array<char, 64, sc_char, 512, it> v_uboot;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_linux[16];
    #else
    array<char, 16, sc_char, 1024, it> v_linux;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_ctrlproc[64];
    #else
    array<char, 64, sc_char, 1152, it> v_ctrlproc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_scnproc[64];
    #else
    array<char, 64, sc_char, 1664, it> v_scnproc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_rcvpic[16];
    #else
    array<char, 16, sc_char, 2176, it> v_rcvpic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_eslpic[16];
    #else
    array<char, 16, sc_char, 2304, it> v_eslpic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_mb_fpga[16];
    #else
    array<char, 16, sc_char, 2432, it> v_mb_fpga;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_fpga[16];
    #else
    array<char, 16, sc_char, 2560, it> v_acq_fpga;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_commpb[16];
    #else
    array<char, 16, sc_char, 2688, it> v_acq_commpb;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_rprpb[16];
    #else
    array<char, 16, sc_char, 2816, it> v_acq_rprpb;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_dlec[16];
    #else
    array<char, 16, sc_char, 2944, it> v_acq_dlec;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_inclpic[16];
    #else
    array<char, 16, sc_char, 3072, it> v_inclpic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_disppic[16];
    #else
    array<char, 16, sc_char, 3200, it> v_disppic;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    firmware_2& operator=(const firmware_2<ito>& o) {
        for(unsigned n=0; n<64; ++n) id[n] = o.id[n];
        for(unsigned n=0; n<64; ++n) v_uboot[n] = o.v_uboot[n];
        for(unsigned n=0; n<16; ++n) v_linux[n] = o.v_linux[n];
        for(unsigned n=0; n<64; ++n) v_ctrlproc[n] = o.v_ctrlproc[n];
        for(unsigned n=0; n<64; ++n) v_scnproc[n] = o.v_scnproc[n];
        for(unsigned n=0; n<16; ++n) v_rcvpic[n] = o.v_rcvpic[n];
        for(unsigned n=0; n<16; ++n) v_eslpic[n] = o.v_eslpic[n];
        for(unsigned n=0; n<16; ++n) v_mb_fpga[n] = o.v_mb_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_fpga[n] = o.v_acq_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_commpb[n] = o.v_acq_commpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_rprpb[n] = o.v_acq_rprpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_dlec[n] = o.v_acq_dlec[n];
        for(unsigned n=0; n<16; ++n) v_inclpic[n] = o.v_inclpic[n];
        for(unsigned n=0; n<16; ++n) v_disppic[n] = o.v_disppic[n];
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct firmware_2<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef firmware_2<rebind_it> type;
    };

    enum { id_main = 8, id_sub = 2};

    char             id[64];
    char             v_uboot[64];
    char             v_linux[16];
    char             v_ctrlproc[64];
    char             v_scnproc[64];
    char             v_rcvpic[16];
    char             v_eslpic[16];
    char             v_mb_fpga[16];
    char             v_acq_fpga[16];
    char             v_acq_commpb[16];
    char             v_acq_rprpb[16];
    char             v_acq_dlec[16];
    char             v_inclpic[16];
    char             v_disppic[16];

    firmware_2() {}
    template<class it>
    firmware_2(const firmware_2<it>& o) {
        for(unsigned n=0; n<64; ++n) id[n] = o.id[n];
        for(unsigned n=0; n<64; ++n) v_uboot[n] = o.v_uboot[n];
        for(unsigned n=0; n<16; ++n) v_linux[n] = o.v_linux[n];
        for(unsigned n=0; n<64; ++n) v_ctrlproc[n] = o.v_ctrlproc[n];
        for(unsigned n=0; n<64; ++n) v_scnproc[n] = o.v_scnproc[n];
        for(unsigned n=0; n<16; ++n) v_rcvpic[n] = o.v_rcvpic[n];
        for(unsigned n=0; n<16; ++n) v_eslpic[n] = o.v_eslpic[n];
        for(unsigned n=0; n<16; ++n) v_mb_fpga[n] = o.v_mb_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_fpga[n] = o.v_acq_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_commpb[n] = o.v_acq_commpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_rprpb[n] = o.v_acq_rprpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_dlec[n] = o.v_acq_dlec[n];
        for(unsigned n=0; n<16; ++n) v_inclpic[n] = o.v_inclpic[n];
        for(unsigned n=0; n<16; ++n) v_disppic[n] = o.v_disppic[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const firmware_2<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", "; write_array(s, 64, x.id);
        s << ", "; write_array(s, 64, x.v_uboot);
        s << ", "; write_array(s, 16, x.v_linux);
        s << ", "; write_array(s, 64, x.v_ctrlproc);
        s << ", "; write_array(s, 64, x.v_scnproc);
        s << ", "; write_array(s, 16, x.v_rcvpic);
        s << ", "; write_array(s, 16, x.v_eslpic);
        s << ", "; write_array(s, 16, x.v_mb_fpga);
        s << ", "; write_array(s, 16, x.v_acq_fpga);
        s << ", "; write_array(s, 16, x.v_acq_commpb);
        s << ", "; write_array(s, 16, x.v_acq_rprpb);
        s << ", "; write_array(s, 16, x.v_acq_dlec);
        s << ", "; write_array(s, 16, x.v_inclpic);
        s << ", "; write_array(s, 16, x.v_disppic);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, firmware_2<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ','; read_array(s, 64, x.id);
        s >> ','; read_array(s, 64, x.v_uboot);
        s >> ','; read_array(s, 16, x.v_linux);
        s >> ','; read_array(s, 64, x.v_ctrlproc);
        s >> ','; read_array(s, 64, x.v_scnproc);
        s >> ','; read_array(s, 16, x.v_rcvpic);
        s >> ','; read_array(s, 16, x.v_eslpic);
        s >> ','; read_array(s, 16, x.v_mb_fpga);
        s >> ','; read_array(s, 16, x.v_acq_fpga);
        s >> ','; read_array(s, 16, x.v_acq_commpb);
        s >> ','; read_array(s, 16, x.v_acq_rprpb);
        s >> ','; read_array(s, 16, x.v_acq_dlec);
        s >> ','; read_array(s, 16, x.v_inclpic);
        s >> ','; read_array(s, 16, x.v_disppic);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct firmware_3
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef firmware_3<rebind_it> type;
    };

    typedef it iterator_type;

    firmware_3(it begin, it end, bool dirty=false)
        : id(begin)
        , v_uboot(begin)
        , v_linux(begin)
        , v_ctrlproc(begin)
        , v_scnproc(begin)
        , v_rcvpic(begin)
        , v_eslpic(begin)
        , v_mb_fpga(begin)
        , v_acq_fpga(begin)
        , v_acq_commpb(begin)
        , v_acq_rprpb(begin)
        , v_acq_dlec(begin)
        , v_inclpic(begin)
        , v_disppic(begin)
        , v_inclpic2(begin)
        , v_HTRpic(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 8, id_sub = 3};
    #ifndef DOXYGEN
    enum { max_bit_width = 3584};
    it begin() const { return id.begin(); }
    it end() const { return v_HTRpic.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    char id[64];
    #else
    array<char, 64, sc_char, 0, it> id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_uboot[64];
    #else
    array<char, 64, sc_char, 512, it> v_uboot;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_linux[16];
    #else
    array<char, 16, sc_char, 1024, it> v_linux;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_ctrlproc[64];
    #else
    array<char, 64, sc_char, 1152, it> v_ctrlproc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_scnproc[64];
    #else
    array<char, 64, sc_char, 1664, it> v_scnproc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_rcvpic[16];
    #else
    array<char, 16, sc_char, 2176, it> v_rcvpic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_eslpic[16];
    #else
    array<char, 16, sc_char, 2304, it> v_eslpic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_mb_fpga[16];
    #else
    array<char, 16, sc_char, 2432, it> v_mb_fpga;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_fpga[16];
    #else
    array<char, 16, sc_char, 2560, it> v_acq_fpga;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_commpb[16];
    #else
    array<char, 16, sc_char, 2688, it> v_acq_commpb;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_rprpb[16];
    #else
    array<char, 16, sc_char, 2816, it> v_acq_rprpb;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_acq_dlec[16];
    #else
    array<char, 16, sc_char, 2944, it> v_acq_dlec;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_inclpic[16];
    #else
    array<char, 16, sc_char, 3072, it> v_inclpic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_disppic[16];
    #else
    array<char, 16, sc_char, 3200, it> v_disppic;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_inclpic2[16];
    #else
    array<char, 16, sc_char, 3328, it> v_inclpic2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char v_HTRpic[16];
    #else
    array<char, 16, sc_char, 3456, it> v_HTRpic;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    firmware_3& operator=(const firmware_3<ito>& o) {
        for(unsigned n=0; n<64; ++n) id[n] = o.id[n];
        for(unsigned n=0; n<64; ++n) v_uboot[n] = o.v_uboot[n];
        for(unsigned n=0; n<16; ++n) v_linux[n] = o.v_linux[n];
        for(unsigned n=0; n<64; ++n) v_ctrlproc[n] = o.v_ctrlproc[n];
        for(unsigned n=0; n<64; ++n) v_scnproc[n] = o.v_scnproc[n];
        for(unsigned n=0; n<16; ++n) v_rcvpic[n] = o.v_rcvpic[n];
        for(unsigned n=0; n<16; ++n) v_eslpic[n] = o.v_eslpic[n];
        for(unsigned n=0; n<16; ++n) v_mb_fpga[n] = o.v_mb_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_fpga[n] = o.v_acq_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_commpb[n] = o.v_acq_commpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_rprpb[n] = o.v_acq_rprpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_dlec[n] = o.v_acq_dlec[n];
        for(unsigned n=0; n<16; ++n) v_inclpic[n] = o.v_inclpic[n];
        for(unsigned n=0; n<16; ++n) v_disppic[n] = o.v_disppic[n];
        for(unsigned n=0; n<16; ++n) v_inclpic2[n] = o.v_inclpic2[n];
        for(unsigned n=0; n<16; ++n) v_HTRpic[n] = o.v_HTRpic[n];
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct firmware_3<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef firmware_3<rebind_it> type;
    };

    enum { id_main = 8, id_sub = 3};

    char             id[64];
    char             v_uboot[64];
    char             v_linux[16];
    char             v_ctrlproc[64];
    char             v_scnproc[64];
    char             v_rcvpic[16];
    char             v_eslpic[16];
    char             v_mb_fpga[16];
    char             v_acq_fpga[16];
    char             v_acq_commpb[16];
    char             v_acq_rprpb[16];
    char             v_acq_dlec[16];
    char             v_inclpic[16];
    char             v_disppic[16];
    char             v_inclpic2[16];
    char             v_HTRpic[16];

    firmware_3() {}
    template<class it>
    firmware_3(const firmware_3<it>& o) {
        for(unsigned n=0; n<64; ++n) id[n] = o.id[n];
        for(unsigned n=0; n<64; ++n) v_uboot[n] = o.v_uboot[n];
        for(unsigned n=0; n<16; ++n) v_linux[n] = o.v_linux[n];
        for(unsigned n=0; n<64; ++n) v_ctrlproc[n] = o.v_ctrlproc[n];
        for(unsigned n=0; n<64; ++n) v_scnproc[n] = o.v_scnproc[n];
        for(unsigned n=0; n<16; ++n) v_rcvpic[n] = o.v_rcvpic[n];
        for(unsigned n=0; n<16; ++n) v_eslpic[n] = o.v_eslpic[n];
        for(unsigned n=0; n<16; ++n) v_mb_fpga[n] = o.v_mb_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_fpga[n] = o.v_acq_fpga[n];
        for(unsigned n=0; n<16; ++n) v_acq_commpb[n] = o.v_acq_commpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_rprpb[n] = o.v_acq_rprpb[n];
        for(unsigned n=0; n<16; ++n) v_acq_dlec[n] = o.v_acq_dlec[n];
        for(unsigned n=0; n<16; ++n) v_inclpic[n] = o.v_inclpic[n];
        for(unsigned n=0; n<16; ++n) v_disppic[n] = o.v_disppic[n];
        for(unsigned n=0; n<16; ++n) v_inclpic2[n] = o.v_inclpic2[n];
        for(unsigned n=0; n<16; ++n) v_HTRpic[n] = o.v_HTRpic[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const firmware_3<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", "; write_array(s, 64, x.id);
        s << ", "; write_array(s, 64, x.v_uboot);
        s << ", "; write_array(s, 16, x.v_linux);
        s << ", "; write_array(s, 64, x.v_ctrlproc);
        s << ", "; write_array(s, 64, x.v_scnproc);
        s << ", "; write_array(s, 16, x.v_rcvpic);
        s << ", "; write_array(s, 16, x.v_eslpic);
        s << ", "; write_array(s, 16, x.v_mb_fpga);
        s << ", "; write_array(s, 16, x.v_acq_fpga);
        s << ", "; write_array(s, 16, x.v_acq_commpb);
        s << ", "; write_array(s, 16, x.v_acq_rprpb);
        s << ", "; write_array(s, 16, x.v_acq_dlec);
        s << ", "; write_array(s, 16, x.v_inclpic);
        s << ", "; write_array(s, 16, x.v_disppic);
        s << ", "; write_array(s, 16, x.v_inclpic2);
        s << ", "; write_array(s, 16, x.v_HTRpic);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, firmware_3<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ','; read_array(s, 64, x.id);
        s >> ','; read_array(s, 64, x.v_uboot);
        s >> ','; read_array(s, 16, x.v_linux);
        s >> ','; read_array(s, 64, x.v_ctrlproc);
        s >> ','; read_array(s, 64, x.v_scnproc);
        s >> ','; read_array(s, 16, x.v_rcvpic);
        s >> ','; read_array(s, 16, x.v_eslpic);
        s >> ','; read_array(s, 16, x.v_mb_fpga);
        s >> ','; read_array(s, 16, x.v_acq_fpga);
        s >> ','; read_array(s, 16, x.v_acq_commpb);
        s >> ','; read_array(s, 16, x.v_acq_rprpb);
        s >> ','; read_array(s, 16, x.v_acq_dlec);
        s >> ','; read_array(s, 16, x.v_inclpic);
        s >> ','; read_array(s, 16, x.v_disppic);
        s >> ','; read_array(s, 16, x.v_inclpic2);
        s >> ','; read_array(s, 16, x.v_HTRpic);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct frame_start
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef frame_start<rebind_it> type;
    };

    typedef it iterator_type;

    frame_start(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 14, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    frame_start& operator=(const frame_start<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct frame_start<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef frame_start<rebind_it> type;
    };

    enum { id_main = 14, id_sub = 0};


    frame_start() {}
    template<class it>
    frame_start(const frame_start<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const frame_start<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, frame_start<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! start of a scan frame in down direction.

//! Sent, when a scan with changing frame angle enters
//! the specified frame range in negative (decreasing angle) direction
//! before data packets belonging to this frame
//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct frame_start_dn
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef frame_start_dn<rebind_it> type;
    };

    typedef it iterator_type;

    frame_start_dn(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 23, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    frame_start_dn& operator=(const frame_start_dn<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct frame_start_dn<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef frame_start_dn<rebind_it> type;
    };

    enum { id_main = 23, id_sub = 0};


    frame_start_dn() {}
    template<class it>
    frame_start_dn(const frame_start_dn<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const frame_start_dn<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, frame_start_dn<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! Start of a scan frame in up direction.

//! Sent, when a scan with changing frame angle enters
//! the specified frame range in positive (increasing angle) direction
//! before data packets belonging to this frame
//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct frame_start_up
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef frame_start_up<rebind_it> type;
    };

    typedef it iterator_type;

    frame_start_up(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 22, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    frame_start_up& operator=(const frame_start_up<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct frame_start_up<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef frame_start_up<rebind_it> type;
    };

    enum { id_main = 22, id_sub = 0};


    frame_start_up() {}
    template<class it>
    frame_start_up(const frame_start_up<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const frame_start_up<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, frame_start_up<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! end of a scan frame

//! after data packets belonging to this frame
//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct frame_stop
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef frame_stop<rebind_it> type;
    };

    typedef it iterator_type;

    frame_stop(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 15, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    frame_stop& operator=(const frame_stop<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct frame_stop<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef frame_stop<rebind_it> type;
    };

    enum { id_main = 15, id_sub = 0};


    frame_stop() {}
    template<class it>
    frame_stop(const frame_stop<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const frame_stop<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, frame_stop<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct generic_end
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef generic_end<rebind_it> type;
    };

    typedef it iterator_type;

    generic_end(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 21, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    generic_end& operator=(const generic_end<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct generic_end<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef generic_end<rebind_it> type;
    };

    enum { id_main = 21, id_sub = 0};


    generic_end() {}
    template<class it>
    generic_end(const generic_end<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const generic_end<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, generic_end<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! The mandatory header package.

//! Basic information to identify the scanning device and short id definition.
//! Handling of the short IDs is mostly automatic, i.e. you do not normally
//! need to care about the short ID mechanism.
//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct header
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef header<rebind_it> type;
    };

    typedef it iterator_type;

    header(it begin, it end, bool dirty=false)
        : type_id(begin)
        , build(begin)
        , serial(begin)
        , id_lookup(begin, end, id_lookup_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 1, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 8512};
    it begin() const { return type_id.begin(); }
    it end() const { return id_lookup.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    char type_id[16];//!< instrument type 
    #else
    array<char, 16, sc_char, 0, it> type_id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char build[16];//!< build variant 
    #else
    array<char, 16, sc_char, 128, it> build;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char serial[16];//!< serial number 
    #else
    array<char, 16, sc_char, 256, it> serial;
    #endif //DOXYGEN

    std::size_t id_lookup_size;
    enum { id_lookup_max_size = 254 };
    //! id lookup table for short-id's

    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sub(begin, begin_bit)
            , main(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint16_t sub;//!< sub id 
        #else
        field<uint16_t, sc_uint16, 0, it> sub;
        #endif
        #ifdef DOXYGEN
        uint16_t main;//!< main id 
        #else
        field<uint16_t, sc_uint16, 16, it> main;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition id_lookup[254];
    #else
    sequence<header, 32, 384, it> id_lookup;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    header& operator=(const header<ito>& o) {
        for(unsigned n=0; n<16; ++n) type_id[n] = o.type_id[n];
        for(unsigned n=0; n<16; ++n) build[n] = o.build[n];
        for(unsigned n=0; n<16; ++n) serial[n] = o.serial[n];
        id_lookup_size = o.id_lookup_size;
        for(unsigned n=0; n<id_lookup_size; ++n){
            id_lookup[n].sub = o.id_lookup[n].sub;
            id_lookup[n].main = o.id_lookup[n].main;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct header<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef header<rebind_it> type;
    };

    enum { id_main = 1, id_sub = 0};

    char             type_id[16];
    char             build[16];
    char             serial[16];
    std::size_t id_lookup_size;
    enum { id_lookup_max_size = 254 };
    struct sequence_definition {
        uint16_t         sub;
        uint16_t         main;
    } id_lookup[254];

    header() {}
    template<class it>
    header(const header<it>& o) {
        for(unsigned n=0; n<16; ++n) type_id[n] = o.type_id[n];
        for(unsigned n=0; n<16; ++n) build[n] = o.build[n];
        for(unsigned n=0; n<16; ++n) serial[n] = o.serial[n];
        id_lookup_size = o.id_lookup.size();
        for(unsigned n=0; n<id_lookup_size; ++n){
            id_lookup[n].sub = o.id_lookup[n].sub;
            id_lookup[n].main = o.id_lookup[n].main;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const header<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", "; write_array(s, 16, x.type_id);
        s << ", "; write_array(s, 16, x.build);
        s << ", "; write_array(s, 16, x.serial);

        s << ", [";
        for (std::size_t n=0; n<x.id_lookup_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.id_lookup[n].sub
            << ", " << x.id_lookup[n].main
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, header<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ','; read_array(s, 16, x.type_id);
        s >> ','; read_array(s, 16, x.build);
        s >> ','; read_array(s, 16, x.serial);

        s >> ',' >> '[' >> std::ws;
        x.id_lookup_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.id_lookup_size;
            if (s
                >> '{' >> (x.id_lookup[x.id_lookup_size-1].sub)
                >> ',' >> (x.id_lookup[x.id_lookup_size-1].main)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! Extension header

//! The extension header lists all major and minor numbers of packages
//! which might be actual present in the current data stream
//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct header_ext
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef header_ext<rebind_it> type;
    };

    typedef it iterator_type;

    header_ext(it begin, it end, bool dirty=false)
        : id(begin, end, id_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 43, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 8128};
    it begin() const { return id.begin(); }
    it end() const { return id.end(); }

    #endif //DOXYGEN

    std::size_t id_size;
    enum { id_max_size = 254 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sub(begin, begin_bit)
            , main(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint16_t sub;//!<  sub id 
        #else
        field<uint16_t, sc_uint16, 0, it> sub;
        #endif
        #ifdef DOXYGEN
        uint16_t main;//!<  main id 
        #else
        field<uint16_t, sc_uint16, 16, it> main;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition id[254];
    #else
    sequence<header_ext, 32, 0, it> id;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    header_ext& operator=(const header_ext<ito>& o) {
        id_size = o.id_size;
        for(unsigned n=0; n<id_size; ++n){
            id[n].sub = o.id[n].sub;
            id[n].main = o.id[n].main;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct header_ext<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef header_ext<rebind_it> type;
    };

    enum { id_main = 43, id_sub = 0};

    std::size_t id_size;
    enum { id_max_size = 254 };
    struct sequence_definition {
        uint16_t         sub;
        uint16_t         main;
    } id[254];

    header_ext() {}
    template<class it>
    header_ext(const header_ext<it>& o) {
        id_size = o.id.size();
        for(unsigned n=0; n<id_size; ++n){
            id[n].sub = o.id[n].sub;
            id[n].main = o.id[n].main;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const header_ext<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {

        s << ", [";
        for (std::size_t n=0; n<x.id_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.id[n].sub
            << ", " << x.id[n].main
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, header_ext<it>& x) {
    package_istream_entry ok(s);
    if (ok) {

        s >> ',' >> '[' >> std::ws;
        x.id_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.id_size;
            if (s
                >> '{' >> (x.id[x.id_size-1].sub)
                >> ',' >> (x.id[x.id_size-1].main)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_bat
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_bat<rebind_it> type;
    };

    typedef it iterator_type;

    hk_bat(it begin, it end, bool dirty=false)
        : V_IN_1(begin)
        , V_IN_2(begin)
        , V_IN_3(begin)
        , V_IN_SEL(begin)
        , A_IN_SEL(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_20(begin)
        , RESERVED_21(begin)
        , RESERVED_22(begin)
        , RESERVED_23(begin)
        , RESERVED_24(begin)
        , RESERVED_25(begin)
        , RESERVED_26(begin)
        , RESERVED_27(begin)
        , RESERVED_28(begin)
        , RESERVED_29(begin)
        , RESERVED_30(begin)
        , level(begin)
        , RESERVED_32(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10003, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 384};
    it begin() const { return V_IN_1.begin(); }
    it end() const { return RESERVED_32.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_1;
    #else
    field<int16_t, sc_int16, 0, it> V_IN_1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_2;
    #else
    field<int16_t, sc_int16, 16, it> V_IN_2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_3;
    #else
    field<int16_t, sc_int16, 32, it> V_IN_3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t V_IN_SEL;
    #else
    field<uint16_t, sc_uint16, 48, it> V_IN_SEL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t A_IN_SEL;
    #else
    field<uint16_t, sc_uint16, 64, it> A_IN_SEL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_05;
    #else
    field<uint16_t, sc_uint16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_06;
    #else
    field<uint16_t, sc_uint16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_10;
    #else
    field<uint8_t, sc_uint8, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_11[3];
    #else
    array<uint8_t, 3, sc_uint8, 168, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_20;
    #else
    field<uint16_t, sc_uint16, 192, it> RESERVED_20;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_21;
    #else
    field<uint16_t, sc_uint16, 208, it> RESERVED_21;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_22;
    #else
    field<uint16_t, sc_uint16, 224, it> RESERVED_22;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_23;
    #else
    field<uint16_t, sc_uint16, 240, it> RESERVED_23;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_24;
    #else
    field<uint16_t, sc_uint16, 256, it> RESERVED_24;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_25;
    #else
    field<uint16_t, sc_uint16, 272, it> RESERVED_25;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_26;
    #else
    field<uint16_t, sc_uint16, 288, it> RESERVED_26;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_27;
    #else
    field<uint16_t, sc_uint16, 304, it> RESERVED_27;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_28;
    #else
    field<int16_t, sc_int16, 320, it> RESERVED_28;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_29;
    #else
    field<int16_t, sc_int16, 336, it> RESERVED_29;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_30;
    #else
    field<int16_t, sc_int16, 352, it> RESERVED_30;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t level;
    #else
    field<uint8_t, sc_uint8, 368, it> level;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_32;
    #else
    field<uint8_t, sc_uint8, 376, it> RESERVED_32;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_bat& operator=(const hk_bat<ito>& o) {
        V_IN_1 = o.V_IN_1;
        V_IN_2 = o.V_IN_2;
        V_IN_3 = o.V_IN_3;
        V_IN_SEL = o.V_IN_SEL;
        A_IN_SEL = o.A_IN_SEL;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        for(unsigned n=0; n<3; ++n) RESERVED_11[n] = o.RESERVED_11[n];
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        level = o.level;
        RESERVED_32 = o.RESERVED_32;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_bat<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_bat<rebind_it> type;
    };

    enum { id_main = 10003, id_sub = 1};

    int16_t          V_IN_1;
    int16_t          V_IN_2;
    int16_t          V_IN_3;
    uint16_t         V_IN_SEL;
    uint16_t         A_IN_SEL;
    uint16_t         RESERVED_05;
    uint16_t         RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    uint8_t          RESERVED_10;
    uint8_t          RESERVED_11[3];
    uint16_t         RESERVED_20;
    uint16_t         RESERVED_21;
    uint16_t         RESERVED_22;
    uint16_t         RESERVED_23;
    uint16_t         RESERVED_24;
    uint16_t         RESERVED_25;
    uint16_t         RESERVED_26;
    uint16_t         RESERVED_27;
    int16_t          RESERVED_28;
    int16_t          RESERVED_29;
    int16_t          RESERVED_30;
    uint8_t          level;
    uint8_t          RESERVED_32;

    hk_bat() {}
    template<class it>
    hk_bat(const hk_bat<it>& o) {
        V_IN_1 = o.V_IN_1;
        V_IN_2 = o.V_IN_2;
        V_IN_3 = o.V_IN_3;
        V_IN_SEL = o.V_IN_SEL;
        A_IN_SEL = o.A_IN_SEL;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        for(unsigned n=0; n<3; ++n) RESERVED_11[n] = o.RESERVED_11[n];
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        level = o.level;
        RESERVED_32 = o.RESERVED_32;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_bat<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.V_IN_1;
        s << ", " << x.V_IN_2;
        s << ", " << x.V_IN_3;
        s << ", " << x.V_IN_SEL;
        s << ", " << x.A_IN_SEL;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", "; write_array(s, 3, x.RESERVED_11);
        s << ", " << x.RESERVED_20;
        s << ", " << x.RESERVED_21;
        s << ", " << x.RESERVED_22;
        s << ", " << x.RESERVED_23;
        s << ", " << x.RESERVED_24;
        s << ", " << x.RESERVED_25;
        s << ", " << x.RESERVED_26;
        s << ", " << x.RESERVED_27;
        s << ", " << x.RESERVED_28;
        s << ", " << x.RESERVED_29;
        s << ", " << x.RESERVED_30;
        s << ", " << x.level;
        s << ", " << x.RESERVED_32;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_bat<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.V_IN_1;
        s >> ',' >> x.V_IN_2;
        s >> ',' >> x.V_IN_3;
        s >> ',' >> x.V_IN_SEL;
        s >> ',' >> x.A_IN_SEL;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ','; read_array(s, 3, x.RESERVED_11);
        s >> ',' >> x.RESERVED_20;
        s >> ',' >> x.RESERVED_21;
        s >> ',' >> x.RESERVED_22;
        s >> ',' >> x.RESERVED_23;
        s >> ',' >> x.RESERVED_24;
        s >> ',' >> x.RESERVED_25;
        s >> ',' >> x.RESERVED_26;
        s >> ',' >> x.RESERVED_27;
        s >> ',' >> x.RESERVED_28;
        s >> ',' >> x.RESERVED_29;
        s >> ',' >> x.RESERVED_30;
        s >> ',' >> x.level;
        s >> ',' >> x.RESERVED_32;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_bat_1
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_bat_1<rebind_it> type;
    };

    typedef it iterator_type;

    hk_bat_1(it begin, it end, bool dirty=false)
        : V_IN_1(begin)
        , V_IN_2(begin)
        , V_IN_3(begin)
        , V_IN_SEL(begin)
        , A_IN_SEL(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_20(begin)
        , RESERVED_21(begin)
        , RESERVED_22(begin)
        , RESERVED_23(begin)
        , RESERVED_24(begin)
        , RESERVED_25(begin)
        , RESERVED_26(begin)
        , RESERVED_27(begin)
        , RESERVED_28(begin)
        , RESERVED_29(begin)
        , RESERVED_30(begin)
        , level(begin)
        , RESERVED_32(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10007, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 392};
    it begin() const { return V_IN_1.begin(); }
    it end() const { return RESERVED_32.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_1;
    #else
    field<int16_t, sc_int16, 0, it> V_IN_1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_2;
    #else
    field<int16_t, sc_int16, 16, it> V_IN_2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_3;
    #else
    field<int16_t, sc_int16, 32, it> V_IN_3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t V_IN_SEL;
    #else
    field<uint16_t, sc_uint16, 48, it> V_IN_SEL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t A_IN_SEL;
    #else
    field<uint16_t, sc_uint16, 64, it> A_IN_SEL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_05;
    #else
    field<uint16_t, sc_uint16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_06;
    #else
    field<uint16_t, sc_uint16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_10;
    #else
    field<uint16_t, sc_uint16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_11[3];
    #else
    array<uint8_t, 3, sc_uint8, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_20;
    #else
    field<uint16_t, sc_uint16, 200, it> RESERVED_20;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_21;
    #else
    field<uint16_t, sc_uint16, 216, it> RESERVED_21;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_22;
    #else
    field<uint16_t, sc_uint16, 232, it> RESERVED_22;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_23;
    #else
    field<uint16_t, sc_uint16, 248, it> RESERVED_23;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_24;
    #else
    field<uint16_t, sc_uint16, 264, it> RESERVED_24;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_25;
    #else
    field<uint16_t, sc_uint16, 280, it> RESERVED_25;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_26;
    #else
    field<uint16_t, sc_uint16, 296, it> RESERVED_26;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_27;
    #else
    field<uint16_t, sc_uint16, 312, it> RESERVED_27;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_28;
    #else
    field<int16_t, sc_int16, 328, it> RESERVED_28;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_29;
    #else
    field<int16_t, sc_int16, 344, it> RESERVED_29;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_30;
    #else
    field<int16_t, sc_int16, 360, it> RESERVED_30;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t level;
    #else
    field<uint8_t, sc_uint8, 376, it> level;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_32;
    #else
    field<uint8_t, sc_uint8, 384, it> RESERVED_32;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_bat_1& operator=(const hk_bat_1<ito>& o) {
        V_IN_1 = o.V_IN_1;
        V_IN_2 = o.V_IN_2;
        V_IN_3 = o.V_IN_3;
        V_IN_SEL = o.V_IN_SEL;
        A_IN_SEL = o.A_IN_SEL;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        for(unsigned n=0; n<3; ++n) RESERVED_11[n] = o.RESERVED_11[n];
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        level = o.level;
        RESERVED_32 = o.RESERVED_32;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_bat_1<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_bat_1<rebind_it> type;
    };

    enum { id_main = 10007, id_sub = 1};

    int16_t          V_IN_1;
    int16_t          V_IN_2;
    int16_t          V_IN_3;
    uint16_t         V_IN_SEL;
    uint16_t         A_IN_SEL;
    uint16_t         RESERVED_05;
    uint16_t         RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    uint16_t         RESERVED_10;
    uint8_t          RESERVED_11[3];
    uint16_t         RESERVED_20;
    uint16_t         RESERVED_21;
    uint16_t         RESERVED_22;
    uint16_t         RESERVED_23;
    uint16_t         RESERVED_24;
    uint16_t         RESERVED_25;
    uint16_t         RESERVED_26;
    uint16_t         RESERVED_27;
    int16_t          RESERVED_28;
    int16_t          RESERVED_29;
    int16_t          RESERVED_30;
    uint8_t          level;
    uint8_t          RESERVED_32;

    hk_bat_1() {}
    template<class it>
    hk_bat_1(const hk_bat_1<it>& o) {
        V_IN_1 = o.V_IN_1;
        V_IN_2 = o.V_IN_2;
        V_IN_3 = o.V_IN_3;
        V_IN_SEL = o.V_IN_SEL;
        A_IN_SEL = o.A_IN_SEL;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        for(unsigned n=0; n<3; ++n) RESERVED_11[n] = o.RESERVED_11[n];
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        level = o.level;
        RESERVED_32 = o.RESERVED_32;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_bat_1<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.V_IN_1;
        s << ", " << x.V_IN_2;
        s << ", " << x.V_IN_3;
        s << ", " << x.V_IN_SEL;
        s << ", " << x.A_IN_SEL;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", "; write_array(s, 3, x.RESERVED_11);
        s << ", " << x.RESERVED_20;
        s << ", " << x.RESERVED_21;
        s << ", " << x.RESERVED_22;
        s << ", " << x.RESERVED_23;
        s << ", " << x.RESERVED_24;
        s << ", " << x.RESERVED_25;
        s << ", " << x.RESERVED_26;
        s << ", " << x.RESERVED_27;
        s << ", " << x.RESERVED_28;
        s << ", " << x.RESERVED_29;
        s << ", " << x.RESERVED_30;
        s << ", " << x.level;
        s << ", " << x.RESERVED_32;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_bat_1<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.V_IN_1;
        s >> ',' >> x.V_IN_2;
        s >> ',' >> x.V_IN_3;
        s >> ',' >> x.V_IN_SEL;
        s >> ',' >> x.A_IN_SEL;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ','; read_array(s, 3, x.RESERVED_11);
        s >> ',' >> x.RESERVED_20;
        s >> ',' >> x.RESERVED_21;
        s >> ',' >> x.RESERVED_22;
        s >> ',' >> x.RESERVED_23;
        s >> ',' >> x.RESERVED_24;
        s >> ',' >> x.RESERVED_25;
        s >> ',' >> x.RESERVED_26;
        s >> ',' >> x.RESERVED_27;
        s >> ',' >> x.RESERVED_28;
        s >> ',' >> x.RESERVED_29;
        s >> ',' >> x.RESERVED_30;
        s >> ',' >> x.level;
        s >> ',' >> x.RESERVED_32;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_bat_2
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_bat_2<rebind_it> type;
    };

    typedef it iterator_type;

    hk_bat_2(it begin, it end, bool dirty=false)
        : V_IN_1(begin)
        , V_IN_2(begin)
        , V_IN_3(begin)
        , V_IN_SEL(begin)
        , A_IN_SEL(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_20(begin)
        , RESERVED_21(begin)
        , RESERVED_22(begin)
        , RESERVED_23(begin)
        , RESERVED_24(begin)
        , RESERVED_25(begin)
        , RESERVED_26(begin)
        , RESERVED_27(begin)
        , RESERVED_28(begin)
        , RESERVED_29(begin)
        , RESERVED_30(begin)
        , level(begin)
        , RESERVED_32(begin)
        , RESERVED_33(begin)
        , RESERVED_34(begin)
        , RESERVED_35(begin)
        , RESERVED_36(begin)
        , RESERVED_37(begin)
        , RESERVED_38(begin)
        , RESERVED_39(begin)
        , RESERVED_40(begin)
        , RESERVED_41(begin)
        , RESERVED_42(begin)
        , RESERVED_43(begin)
        , RESERVED_44(begin)
        , RESERVED_45(begin)
        , RESERVED_46(begin)
        , RESERVED_47(begin)
        , RESERVED_48(begin)
        , RESERVED_49(begin)
        , RESERVED_50(begin)
        , RESERVED_51(begin)
        , RESERVED_52(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10007, id_sub = 2};
    #ifndef DOXYGEN
    enum { max_bit_width = 728};
    it begin() const { return V_IN_1.begin(); }
    it end() const { return RESERVED_52.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_1;
    #else
    field<int16_t, sc_int16, 0, it> V_IN_1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_2;
    #else
    field<int16_t, sc_int16, 16, it> V_IN_2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_3;
    #else
    field<int16_t, sc_int16, 32, it> V_IN_3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t V_IN_SEL;
    #else
    field<uint16_t, sc_uint16, 48, it> V_IN_SEL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t A_IN_SEL;
    #else
    field<uint16_t, sc_uint16, 64, it> A_IN_SEL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_05;
    #else
    field<uint16_t, sc_uint16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_06;
    #else
    field<uint16_t, sc_uint16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_10;
    #else
    field<uint16_t, sc_uint16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_11[3];
    #else
    array<uint8_t, 3, sc_uint8, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_20;
    #else
    field<uint16_t, sc_uint16, 200, it> RESERVED_20;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_21;
    #else
    field<uint16_t, sc_uint16, 216, it> RESERVED_21;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_22;
    #else
    field<uint16_t, sc_uint16, 232, it> RESERVED_22;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_23;
    #else
    field<uint16_t, sc_uint16, 248, it> RESERVED_23;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_24;
    #else
    field<uint16_t, sc_uint16, 264, it> RESERVED_24;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_25;
    #else
    field<uint16_t, sc_uint16, 280, it> RESERVED_25;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_26;
    #else
    field<uint16_t, sc_uint16, 296, it> RESERVED_26;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_27;
    #else
    field<uint16_t, sc_uint16, 312, it> RESERVED_27;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_28;
    #else
    field<int16_t, sc_int16, 328, it> RESERVED_28;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_29;
    #else
    field<int16_t, sc_int16, 344, it> RESERVED_29;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_30;
    #else
    field<int16_t, sc_int16, 360, it> RESERVED_30;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t level;
    #else
    field<uint8_t, sc_uint8, 376, it> level;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_32;
    #else
    field<uint8_t, sc_uint8, 384, it> RESERVED_32;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_33;
    #else
    field<uint8_t, sc_uint8, 392, it> RESERVED_33;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_34;
    #else
    field<uint16_t, sc_uint16, 400, it> RESERVED_34;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_35;
    #else
    field<uint32_t, sc_uint32, 416, it> RESERVED_35;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_36;
    #else
    field<uint8_t, sc_uint8, 448, it> RESERVED_36;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_37;
    #else
    field<uint8_t, sc_uint8, 456, it> RESERVED_37;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_38;
    #else
    field<uint16_t, sc_uint16, 464, it> RESERVED_38;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_39;
    #else
    field<uint16_t, sc_uint16, 480, it> RESERVED_39;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_40;
    #else
    field<uint16_t, sc_uint16, 496, it> RESERVED_40;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_41;
    #else
    field<uint16_t, sc_uint16, 512, it> RESERVED_41;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_42;
    #else
    field<uint16_t, sc_uint16, 528, it> RESERVED_42;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_43;
    #else
    field<uint16_t, sc_uint16, 544, it> RESERVED_43;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_44;
    #else
    field<uint16_t, sc_uint16, 560, it> RESERVED_44;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_45;
    #else
    field<uint16_t, sc_uint16, 576, it> RESERVED_45;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_46;
    #else
    field<uint32_t, sc_uint32, 592, it> RESERVED_46;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_47;
    #else
    field<uint32_t, sc_uint32, 624, it> RESERVED_47;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_48;
    #else
    field<uint16_t, sc_uint16, 656, it> RESERVED_48;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_49;
    #else
    field<uint16_t, sc_uint16, 672, it> RESERVED_49;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_50;
    #else
    field<uint8_t, sc_uint8, 688, it> RESERVED_50;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_51;
    #else
    field<uint16_t, sc_uint16, 696, it> RESERVED_51;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_52;
    #else
    field<uint16_t, sc_uint16, 712, it> RESERVED_52;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_bat_2& operator=(const hk_bat_2<ito>& o) {
        V_IN_1 = o.V_IN_1;
        V_IN_2 = o.V_IN_2;
        V_IN_3 = o.V_IN_3;
        V_IN_SEL = o.V_IN_SEL;
        A_IN_SEL = o.A_IN_SEL;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        for(unsigned n=0; n<3; ++n) RESERVED_11[n] = o.RESERVED_11[n];
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        level = o.level;
        RESERVED_32 = o.RESERVED_32;
        RESERVED_33 = o.RESERVED_33;
        RESERVED_34 = o.RESERVED_34;
        RESERVED_35 = o.RESERVED_35;
        RESERVED_36 = o.RESERVED_36;
        RESERVED_37 = o.RESERVED_37;
        RESERVED_38 = o.RESERVED_38;
        RESERVED_39 = o.RESERVED_39;
        RESERVED_40 = o.RESERVED_40;
        RESERVED_41 = o.RESERVED_41;
        RESERVED_42 = o.RESERVED_42;
        RESERVED_43 = o.RESERVED_43;
        RESERVED_44 = o.RESERVED_44;
        RESERVED_45 = o.RESERVED_45;
        RESERVED_46 = o.RESERVED_46;
        RESERVED_47 = o.RESERVED_47;
        RESERVED_48 = o.RESERVED_48;
        RESERVED_49 = o.RESERVED_49;
        RESERVED_50 = o.RESERVED_50;
        RESERVED_51 = o.RESERVED_51;
        RESERVED_52 = o.RESERVED_52;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_bat_2<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_bat_2<rebind_it> type;
    };

    enum { id_main = 10007, id_sub = 2};

    int16_t          V_IN_1;
    int16_t          V_IN_2;
    int16_t          V_IN_3;
    uint16_t         V_IN_SEL;
    uint16_t         A_IN_SEL;
    uint16_t         RESERVED_05;
    uint16_t         RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    uint16_t         RESERVED_10;
    uint8_t          RESERVED_11[3];
    uint16_t         RESERVED_20;
    uint16_t         RESERVED_21;
    uint16_t         RESERVED_22;
    uint16_t         RESERVED_23;
    uint16_t         RESERVED_24;
    uint16_t         RESERVED_25;
    uint16_t         RESERVED_26;
    uint16_t         RESERVED_27;
    int16_t          RESERVED_28;
    int16_t          RESERVED_29;
    int16_t          RESERVED_30;
    uint8_t          level;
    uint8_t          RESERVED_32;
    uint8_t          RESERVED_33;
    uint16_t         RESERVED_34;
    uint32_t         RESERVED_35;
    uint8_t          RESERVED_36;
    uint8_t          RESERVED_37;
    uint16_t         RESERVED_38;
    uint16_t         RESERVED_39;
    uint16_t         RESERVED_40;
    uint16_t         RESERVED_41;
    uint16_t         RESERVED_42;
    uint16_t         RESERVED_43;
    uint16_t         RESERVED_44;
    uint16_t         RESERVED_45;
    uint32_t         RESERVED_46;
    uint32_t         RESERVED_47;
    uint16_t         RESERVED_48;
    uint16_t         RESERVED_49;
    uint8_t          RESERVED_50;
    uint16_t         RESERVED_51;
    uint16_t         RESERVED_52;

    hk_bat_2() {}
    template<class it>
    hk_bat_2(const hk_bat_2<it>& o) {
        V_IN_1 = o.V_IN_1;
        V_IN_2 = o.V_IN_2;
        V_IN_3 = o.V_IN_3;
        V_IN_SEL = o.V_IN_SEL;
        A_IN_SEL = o.A_IN_SEL;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        for(unsigned n=0; n<3; ++n) RESERVED_11[n] = o.RESERVED_11[n];
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        level = o.level;
        RESERVED_32 = o.RESERVED_32;
        RESERVED_33 = o.RESERVED_33;
        RESERVED_34 = o.RESERVED_34;
        RESERVED_35 = o.RESERVED_35;
        RESERVED_36 = o.RESERVED_36;
        RESERVED_37 = o.RESERVED_37;
        RESERVED_38 = o.RESERVED_38;
        RESERVED_39 = o.RESERVED_39;
        RESERVED_40 = o.RESERVED_40;
        RESERVED_41 = o.RESERVED_41;
        RESERVED_42 = o.RESERVED_42;
        RESERVED_43 = o.RESERVED_43;
        RESERVED_44 = o.RESERVED_44;
        RESERVED_45 = o.RESERVED_45;
        RESERVED_46 = o.RESERVED_46;
        RESERVED_47 = o.RESERVED_47;
        RESERVED_48 = o.RESERVED_48;
        RESERVED_49 = o.RESERVED_49;
        RESERVED_50 = o.RESERVED_50;
        RESERVED_51 = o.RESERVED_51;
        RESERVED_52 = o.RESERVED_52;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_bat_2<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.V_IN_1;
        s << ", " << x.V_IN_2;
        s << ", " << x.V_IN_3;
        s << ", " << x.V_IN_SEL;
        s << ", " << x.A_IN_SEL;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", "; write_array(s, 3, x.RESERVED_11);
        s << ", " << x.RESERVED_20;
        s << ", " << x.RESERVED_21;
        s << ", " << x.RESERVED_22;
        s << ", " << x.RESERVED_23;
        s << ", " << x.RESERVED_24;
        s << ", " << x.RESERVED_25;
        s << ", " << x.RESERVED_26;
        s << ", " << x.RESERVED_27;
        s << ", " << x.RESERVED_28;
        s << ", " << x.RESERVED_29;
        s << ", " << x.RESERVED_30;
        s << ", " << x.level;
        s << ", " << x.RESERVED_32;
        s << ", " << x.RESERVED_33;
        s << ", " << x.RESERVED_34;
        s << ", " << x.RESERVED_35;
        s << ", " << x.RESERVED_36;
        s << ", " << x.RESERVED_37;
        s << ", " << x.RESERVED_38;
        s << ", " << x.RESERVED_39;
        s << ", " << x.RESERVED_40;
        s << ", " << x.RESERVED_41;
        s << ", " << x.RESERVED_42;
        s << ", " << x.RESERVED_43;
        s << ", " << x.RESERVED_44;
        s << ", " << x.RESERVED_45;
        s << ", " << x.RESERVED_46;
        s << ", " << x.RESERVED_47;
        s << ", " << x.RESERVED_48;
        s << ", " << x.RESERVED_49;
        s << ", " << x.RESERVED_50;
        s << ", " << x.RESERVED_51;
        s << ", " << x.RESERVED_52;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_bat_2<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.V_IN_1;
        s >> ',' >> x.V_IN_2;
        s >> ',' >> x.V_IN_3;
        s >> ',' >> x.V_IN_SEL;
        s >> ',' >> x.A_IN_SEL;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ','; read_array(s, 3, x.RESERVED_11);
        s >> ',' >> x.RESERVED_20;
        s >> ',' >> x.RESERVED_21;
        s >> ',' >> x.RESERVED_22;
        s >> ',' >> x.RESERVED_23;
        s >> ',' >> x.RESERVED_24;
        s >> ',' >> x.RESERVED_25;
        s >> ',' >> x.RESERVED_26;
        s >> ',' >> x.RESERVED_27;
        s >> ',' >> x.RESERVED_28;
        s >> ',' >> x.RESERVED_29;
        s >> ',' >> x.RESERVED_30;
        s >> ',' >> x.level;
        s >> ',' >> x.RESERVED_32;
        s >> ',' >> x.RESERVED_33;
        s >> ',' >> x.RESERVED_34;
        s >> ',' >> x.RESERVED_35;
        s >> ',' >> x.RESERVED_36;
        s >> ',' >> x.RESERVED_37;
        s >> ',' >> x.RESERVED_38;
        s >> ',' >> x.RESERVED_39;
        s >> ',' >> x.RESERVED_40;
        s >> ',' >> x.RESERVED_41;
        s >> ',' >> x.RESERVED_42;
        s >> ',' >> x.RESERVED_43;
        s >> ',' >> x.RESERVED_44;
        s >> ',' >> x.RESERVED_45;
        s >> ',' >> x.RESERVED_46;
        s >> ',' >> x.RESERVED_47;
        s >> ',' >> x.RESERVED_48;
        s >> ',' >> x.RESERVED_49;
        s >> ',' >> x.RESERVED_50;
        s >> ',' >> x.RESERVED_51;
        s >> ',' >> x.RESERVED_52;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_cam
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_cam<rebind_it> type;
    };

    typedef it iterator_type;

    hk_cam(it begin, it end, bool dirty=false)
        : RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10011, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 1096};
    it begin() const { return RESERVED_01.begin(); }
    it end() const { return RESERVED_07.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_01;
    #else
    field<int8_t, sc_int8, 0, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_02;
    #else
    field<uint8_t, sc_uint8, 8, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_04;
    #else
    field<uint16_t, sc_uint16, 32, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_05;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_06;
    #else
    field<int8_t, sc_int8, 64, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char RESERVED_07[128];
    #else
    array<char, 128, sc_char, 72, it> RESERVED_07;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_cam& operator=(const hk_cam<ito>& o) {
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        for(unsigned n=0; n<128; ++n) RESERVED_07[n] = o.RESERVED_07[n];
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_cam<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_cam<rebind_it> type;
    };

    enum { id_main = 10011, id_sub = 0};

    int8_t           RESERVED_01;
    uint8_t          RESERVED_02;
    uint16_t         RESERVED_03;
    uint16_t         RESERVED_04;
    uint16_t         RESERVED_05;
    int8_t           RESERVED_06;
    char             RESERVED_07[128];

    hk_cam() {}
    template<class it>
    hk_cam(const hk_cam<it>& o) {
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        for(unsigned n=0; n<128; ++n) RESERVED_07[n] = o.RESERVED_07[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_cam<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", "; write_array(s, 128, x.RESERVED_07);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_cam<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ','; read_array(s, 128, x.RESERVED_07);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_ctr
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_ctr<rebind_it> type;
    };

    typedef it iterator_type;

    hk_ctr(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , LINK_STAT(begin)
        , WLAN_QUAL(begin)
        , SCN_PROG(begin)
        , USED_FLASH(begin)
        , TOTAL_FLASH(begin)
        , USED_USBM(begin)
        , TOTAL_USBM(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10004, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 352};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return TOTAL_USBM.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_00;
    #else
    field<int16_t, sc_int16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_02;
    #else
    field<uint16_t, sc_uint16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_04;
    #else
    field<uint16_t, sc_uint16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_05;
    #else
    field<uint16_t, sc_uint16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_06;
    #else
    field<uint16_t, sc_uint16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_09;
    #else
    field<uint16_t, sc_uint16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_10;
    #else
    field<uint16_t, sc_uint16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_12;
    #else
    field<uint8_t, sc_uint8, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t LINK_STAT;
    #else
    field<uint8_t, sc_uint8, 200, it> LINK_STAT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t WLAN_QUAL;
    #else
    field<uint8_t, sc_uint8, 208, it> WLAN_QUAL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t SCN_PROG;
    #else
    field<uint8_t, sc_uint8, 216, it> SCN_PROG;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float USED_FLASH;
    #else
    field<float, sc_float32, 224, it> USED_FLASH;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t TOTAL_FLASH;
    #else
    field<uint32_t, sc_uint32, 256, it> TOTAL_FLASH;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float USED_USBM;
    #else
    field<float, sc_float32, 288, it> USED_USBM;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t TOTAL_USBM;
    #else
    field<uint32_t, sc_uint32, 320, it> TOTAL_USBM;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_ctr& operator=(const hk_ctr<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        LINK_STAT = o.LINK_STAT;
        WLAN_QUAL = o.WLAN_QUAL;
        SCN_PROG = o.SCN_PROG;
        USED_FLASH = o.USED_FLASH;
        TOTAL_FLASH = o.TOTAL_FLASH;
        USED_USBM = o.USED_USBM;
        TOTAL_USBM = o.TOTAL_USBM;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_ctr<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_ctr<rebind_it> type;
    };

    enum { id_main = 10004, id_sub = 0};

    int16_t          RESERVED_00;
    uint16_t         RESERVED_01;
    uint16_t         RESERVED_02;
    uint16_t         RESERVED_03;
    uint16_t         RESERVED_04;
    uint16_t         RESERVED_05;
    uint16_t         RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    uint16_t         RESERVED_09;
    uint16_t         RESERVED_10;
    uint16_t         RESERVED_11;
    uint8_t          RESERVED_12;
    uint8_t          LINK_STAT;
    uint8_t          WLAN_QUAL;
    uint8_t          SCN_PROG;
    float            USED_FLASH;
    uint32_t         TOTAL_FLASH;
    float            USED_USBM;
    uint32_t         TOTAL_USBM;

    hk_ctr() {}
    template<class it>
    hk_ctr(const hk_ctr<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        LINK_STAT = o.LINK_STAT;
        WLAN_QUAL = o.WLAN_QUAL;
        SCN_PROG = o.SCN_PROG;
        USED_FLASH = o.USED_FLASH;
        TOTAL_FLASH = o.TOTAL_FLASH;
        USED_USBM = o.USED_USBM;
        TOTAL_USBM = o.TOTAL_USBM;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_ctr<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.LINK_STAT;
        s << ", " << x.WLAN_QUAL;
        s << ", " << x.SCN_PROG;
        s << ", " << x.USED_FLASH;
        s << ", " << x.TOTAL_FLASH;
        s << ", " << x.USED_USBM;
        s << ", " << x.TOTAL_USBM;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_ctr<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.LINK_STAT;
        s >> ',' >> x.WLAN_QUAL;
        s >> ',' >> x.SCN_PROG;
        s >> ',' >> x.USED_FLASH;
        s >> ',' >> x.TOTAL_FLASH;
        s >> ',' >> x.USED_USBM;
        s >> ',' >> x.TOTAL_USBM;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_ctr_1
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_ctr_1<rebind_it> type;
    };

    typedef it iterator_type;

    hk_ctr_1(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , LINK_STAT(begin)
        , WLAN_QUAL(begin)
        , SCN_PROG(begin)
        , USED_FLASH(begin)
        , TOTAL_FLASH(begin)
        , USED_USBM(begin)
        , TOTAL_USBM(begin)
        , MEAS_BUF_UTIL(begin)
        , MON_BUF_UTIL(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10004, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 416};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return MON_BUF_UTIL.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_00;
    #else
    field<int16_t, sc_int16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_02;
    #else
    field<uint16_t, sc_uint16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_04;
    #else
    field<uint16_t, sc_uint16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_05;
    #else
    field<uint16_t, sc_uint16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_06;
    #else
    field<uint16_t, sc_uint16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_09;
    #else
    field<uint16_t, sc_uint16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_10;
    #else
    field<uint16_t, sc_uint16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_12;
    #else
    field<uint8_t, sc_uint8, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t LINK_STAT;
    #else
    field<uint8_t, sc_uint8, 200, it> LINK_STAT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t WLAN_QUAL;
    #else
    field<uint8_t, sc_uint8, 208, it> WLAN_QUAL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t SCN_PROG;
    #else
    field<uint8_t, sc_uint8, 216, it> SCN_PROG;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float USED_FLASH;
    #else
    field<float, sc_float32, 224, it> USED_FLASH;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t TOTAL_FLASH;
    #else
    field<uint32_t, sc_uint32, 256, it> TOTAL_FLASH;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float USED_USBM;
    #else
    field<float, sc_float32, 288, it> USED_USBM;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t TOTAL_USBM;
    #else
    field<uint32_t, sc_uint32, 320, it> TOTAL_USBM;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float MEAS_BUF_UTIL;
    #else
    field<float, sc_float32, 352, it> MEAS_BUF_UTIL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float MON_BUF_UTIL;
    #else
    field<float, sc_float32, 384, it> MON_BUF_UTIL;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_ctr_1& operator=(const hk_ctr_1<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        LINK_STAT = o.LINK_STAT;
        WLAN_QUAL = o.WLAN_QUAL;
        SCN_PROG = o.SCN_PROG;
        USED_FLASH = o.USED_FLASH;
        TOTAL_FLASH = o.TOTAL_FLASH;
        USED_USBM = o.USED_USBM;
        TOTAL_USBM = o.TOTAL_USBM;
        MEAS_BUF_UTIL = o.MEAS_BUF_UTIL;
        MON_BUF_UTIL = o.MON_BUF_UTIL;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_ctr_1<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_ctr_1<rebind_it> type;
    };

    enum { id_main = 10004, id_sub = 1};

    int16_t          RESERVED_00;
    uint16_t         RESERVED_01;
    uint16_t         RESERVED_02;
    uint16_t         RESERVED_03;
    uint16_t         RESERVED_04;
    uint16_t         RESERVED_05;
    uint16_t         RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    uint16_t         RESERVED_09;
    uint16_t         RESERVED_10;
    uint16_t         RESERVED_11;
    uint8_t          RESERVED_12;
    uint8_t          LINK_STAT;
    uint8_t          WLAN_QUAL;
    uint8_t          SCN_PROG;
    float            USED_FLASH;
    uint32_t         TOTAL_FLASH;
    float            USED_USBM;
    uint32_t         TOTAL_USBM;
    float            MEAS_BUF_UTIL;
    float            MON_BUF_UTIL;

    hk_ctr_1() {}
    template<class it>
    hk_ctr_1(const hk_ctr_1<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        LINK_STAT = o.LINK_STAT;
        WLAN_QUAL = o.WLAN_QUAL;
        SCN_PROG = o.SCN_PROG;
        USED_FLASH = o.USED_FLASH;
        TOTAL_FLASH = o.TOTAL_FLASH;
        USED_USBM = o.USED_USBM;
        TOTAL_USBM = o.TOTAL_USBM;
        MEAS_BUF_UTIL = o.MEAS_BUF_UTIL;
        MON_BUF_UTIL = o.MON_BUF_UTIL;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_ctr_1<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.LINK_STAT;
        s << ", " << x.WLAN_QUAL;
        s << ", " << x.SCN_PROG;
        s << ", " << x.USED_FLASH;
        s << ", " << x.TOTAL_FLASH;
        s << ", " << x.USED_USBM;
        s << ", " << x.TOTAL_USBM;
        s << ", " << x.MEAS_BUF_UTIL;
        s << ", " << x.MON_BUF_UTIL;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_ctr_1<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.LINK_STAT;
        s >> ',' >> x.WLAN_QUAL;
        s >> ',' >> x.SCN_PROG;
        s >> ',' >> x.USED_FLASH;
        s >> ',' >> x.TOTAL_FLASH;
        s >> ',' >> x.USED_USBM;
        s >> ',' >> x.TOTAL_USBM;
        s >> ',' >> x.MEAS_BUF_UTIL;
        s >> ',' >> x.MON_BUF_UTIL;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_extended_external
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_extended_external<rebind_it> type;
    };

    typedef it iterator_type;

    hk_extended_external(it begin, it end, bool dirty=false)
        : RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10009, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 32};
    it begin() const { return RESERVED_01.begin(); }
    it end() const { return RESERVED_04.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_01;
    #else
    field<int8_t, sc_int8, 0, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_02;
    #else
    field<int8_t, sc_int8, 8, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_03;
    #else
    field<uint8_t, sc_uint8, 16, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_04;
    #else
    field<uint8_t, sc_uint8, 24, it> RESERVED_04;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_extended_external& operator=(const hk_extended_external<ito>& o) {
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_extended_external<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_extended_external<rebind_it> type;
    };

    enum { id_main = 10009, id_sub = 0};

    int8_t           RESERVED_01;
    int8_t           RESERVED_02;
    uint8_t          RESERVED_03;
    uint8_t          RESERVED_04;

    hk_extended_external() {}
    template<class it>
    hk_extended_external(const hk_extended_external<it>& o) {
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_extended_external<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_extended_external<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_extended_internal
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_extended_internal<rebind_it> type;
    };

    typedef it iterator_type;

    hk_extended_internal(it begin, it end, bool dirty=false)
        : RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10008, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 104};
    it begin() const { return RESERVED_01.begin(); }
    it end() const { return RESERVED_11.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_02;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_03;
    #else
    field<int8_t, sc_int8, 32, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_04;
    #else
    field<int8_t, sc_int8, 40, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_05;
    #else
    field<int8_t, sc_int8, 48, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_06;
    #else
    field<int8_t, sc_int8, 56, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_07;
    #else
    field<int8_t, sc_int8, 64, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_08;
    #else
    field<int8_t, sc_int8, 72, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_09;
    #else
    field<int8_t, sc_int8, 80, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_10;
    #else
    field<int8_t, sc_int8, 88, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t RESERVED_11;
    #else
    field<int8_t, sc_int8, 96, it> RESERVED_11;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_extended_internal& operator=(const hk_extended_internal<ito>& o) {
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_extended_internal<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_extended_internal<rebind_it> type;
    };

    enum { id_main = 10008, id_sub = 0};

    uint16_t         RESERVED_01;
    uint16_t         RESERVED_02;
    int8_t           RESERVED_03;
    int8_t           RESERVED_04;
    int8_t           RESERVED_05;
    int8_t           RESERVED_06;
    int8_t           RESERVED_07;
    int8_t           RESERVED_08;
    int8_t           RESERVED_09;
    int8_t           RESERVED_10;
    int8_t           RESERVED_11;

    hk_extended_internal() {}
    template<class it>
    hk_extended_internal(const hk_extended_internal<it>& o) {
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_extended_internal<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_extended_internal<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! GPS data

//! <para>This package belongs to the predefined selectors:</para>
//! <para>status</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_gps
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_gps<rebind_it> type;
    };

    typedef it iterator_type;

    hk_gps(it begin, it end, bool dirty=false)
        : TOWms(begin)
        , ECEFX(begin)
        , ECEFY(begin)
        , ECEFZ(begin)
        , POS_ACC(begin)
        , LONG(begin)
        , LAT(begin)
        , HGT_ELL(begin)
        , HGT_SEA(begin)
        , HOR_ACC(begin)
        , VERT_ACC(begin)
        , STATUS1(begin)
        , STATUS2(begin)
        , STATUS3(begin)
        , LEAP_SEC(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10005, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 384};
    it begin() const { return TOWms.begin(); }
    it end() const { return LEAP_SEC.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t TOWms;//!<  GPS data, GPS Time of Week                         [ 1 ms] 
    #else
    field<uint32_t, sc_uint32, 0, it> TOWms;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFX;//!<  GPS data, ECEF X coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 32, it> ECEFX;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFY;//!<  GPS data, ECEF Y coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 64, it> ECEFY;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFZ;//!<  GPS data, ECEF Z coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 96, it> ECEFZ;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t POS_ACC;//!<  GPS data, position accuracy estimate               [ 1 cm] 
    #else
    field<uint32_t, sc_uint32, 128, it> POS_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t LONG;//!<  GPS data, longitude                                [1e-7deg] 
    #else
    field<int32_t, sc_int32, 160, it> LONG;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t LAT;//!<  GPS data, latitude                                 [1e-7deg] 
    #else
    field<int32_t, sc_int32, 192, it> LAT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t HGT_ELL;//!<  GPS data, height above ellipsoid                   [ 1 mm] 
    #else
    field<int32_t, sc_int32, 224, it> HGT_ELL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t HGT_SEA;//!<  GPS data, height above mean sea level              [ 1 mm] 
    #else
    field<int32_t, sc_int32, 256, it> HGT_SEA;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t HOR_ACC;//!<  GPS data, horizontal accuracy estimate             [ 1 mm] 
    #else
    field<uint32_t, sc_uint32, 288, it> HOR_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t VERT_ACC;//!<  GPS data, vertical accuracy estimate               [ 1 mm] 
    #else
    field<uint32_t, sc_uint32, 320, it> VERT_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS1;//!<  GPS data, 0=no fix; 1=dead reckoning; 2=2D-fix;  3=3D-fix; 4=GPS+dead reckoning;5=Time only fix;255=undefined fix 
    #else
    field<uint8_t, sc_uint8, 352, it> STATUS1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS2;//!<  GPS data, 1 .. gpsfixOK; 2 .. diffSoln;  4 .. wknSet; 8 .. towSet 
    #else
    field<uint8_t, sc_uint8, 360, it> STATUS2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS3;//!<  GPS data, 0= no DGPS; 1=PR+PRR Correction;  2=PR+PRR+CP Correction;  3=high accuracy PR+PRR+CP Correction 
    #else
    field<uint8_t, sc_uint8, 368, it> STATUS3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t LEAP_SEC;//!<  GPS data, leap seconds (GPS-UTC)                   [ 1 s ] 
    #else
    field<int8_t, sc_int8, 376, it> LEAP_SEC;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_gps& operator=(const hk_gps<ito>& o) {
        TOWms = o.TOWms;
        ECEFX = o.ECEFX;
        ECEFY = o.ECEFY;
        ECEFZ = o.ECEFZ;
        POS_ACC = o.POS_ACC;
        LONG = o.LONG;
        LAT = o.LAT;
        HGT_ELL = o.HGT_ELL;
        HGT_SEA = o.HGT_SEA;
        HOR_ACC = o.HOR_ACC;
        VERT_ACC = o.VERT_ACC;
        STATUS1 = o.STATUS1;
        STATUS2 = o.STATUS2;
        STATUS3 = o.STATUS3;
        LEAP_SEC = o.LEAP_SEC;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct hk_gps<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_gps<rebind_it> type;
    };

    enum { id_main = 10005, id_sub = 0};

    uint32_t         TOWms;
    int32_t          ECEFX;
    int32_t          ECEFY;
    int32_t          ECEFZ;
    uint32_t         POS_ACC;
    int32_t          LONG;
    int32_t          LAT;
    int32_t          HGT_ELL;
    int32_t          HGT_SEA;
    uint32_t         HOR_ACC;
    uint32_t         VERT_ACC;
    uint8_t          STATUS1;
    uint8_t          STATUS2;
    uint8_t          STATUS3;
    int8_t           LEAP_SEC;

    hk_gps() {}
    template<class it>
    hk_gps(const hk_gps<it>& o) {
        TOWms = o.TOWms;
        ECEFX = o.ECEFX;
        ECEFY = o.ECEFY;
        ECEFZ = o.ECEFZ;
        POS_ACC = o.POS_ACC;
        LONG = o.LONG;
        LAT = o.LAT;
        HGT_ELL = o.HGT_ELL;
        HGT_SEA = o.HGT_SEA;
        HOR_ACC = o.HOR_ACC;
        VERT_ACC = o.VERT_ACC;
        STATUS1 = o.STATUS1;
        STATUS2 = o.STATUS2;
        STATUS3 = o.STATUS3;
        LEAP_SEC = o.LEAP_SEC;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_gps<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.TOWms;
        s << ", " << x.ECEFX;
        s << ", " << x.ECEFY;
        s << ", " << x.ECEFZ;
        s << ", " << x.POS_ACC;
        s << ", " << x.LONG;
        s << ", " << x.LAT;
        s << ", " << x.HGT_ELL;
        s << ", " << x.HGT_SEA;
        s << ", " << x.HOR_ACC;
        s << ", " << x.VERT_ACC;
        s << ", " << x.STATUS1;
        s << ", " << x.STATUS2;
        s << ", " << x.STATUS3;
        s << ", " << x.LEAP_SEC;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_gps<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.TOWms;
        s >> ',' >> x.ECEFX;
        s >> ',' >> x.ECEFY;
        s >> ',' >> x.ECEFZ;
        s >> ',' >> x.POS_ACC;
        s >> ',' >> x.LONG;
        s >> ',' >> x.LAT;
        s >> ',' >> x.HGT_ELL;
        s >> ',' >> x.HGT_SEA;
        s >> ',' >> x.HOR_ACC;
        s >> ',' >> x.VERT_ACC;
        s >> ',' >> x.STATUS1;
        s >> ',' >> x.STATUS2;
        s >> ',' >> x.STATUS3;
        s >> ',' >> x.LEAP_SEC;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! GPS data

//! <para>This package belongs to the predefined selectors:</para>
//! <para>status</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_gps_ts
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_gps_ts<rebind_it> type;
    };

    typedef it iterator_type;

    hk_gps_ts(it begin, it end, bool dirty=false)
        : TOWms(begin)
        , ECEFX(begin)
        , ECEFY(begin)
        , ECEFZ(begin)
        , POS_ACC(begin)
        , LONG(begin)
        , LAT(begin)
        , HGT_ELL(begin)
        , HGT_SEA(begin)
        , HOR_ACC(begin)
        , VERT_ACC(begin)
        , STATUS1(begin)
        , STATUS2(begin)
        , STATUS3(begin)
        , LEAP_SEC(begin)
        , systime(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10005, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 416};
    it begin() const { return TOWms.begin(); }
    it end() const { return systime.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t TOWms;//!<  GPS data, GPS Time of Week                         [ 1 ms] 
    #else
    field<uint32_t, sc_uint32, 0, it> TOWms;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFX;//!<  GPS data, ECEF X coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 32, it> ECEFX;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFY;//!<  GPS data, ECEF Y coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 64, it> ECEFY;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFZ;//!<  GPS data, ECEF Z coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 96, it> ECEFZ;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t POS_ACC;//!<  GPS data, position accuracy estimate               [ 1 cm] 
    #else
    field<uint32_t, sc_uint32, 128, it> POS_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t LONG;//!<  GPS data, longitude                                [1e-7deg] 
    #else
    field<int32_t, sc_int32, 160, it> LONG;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t LAT;//!<  GPS data, latitude                                 [1e-7deg] 
    #else
    field<int32_t, sc_int32, 192, it> LAT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t HGT_ELL;//!<  GPS data, height above ellipsoid                   [ 1 mm] 
    #else
    field<int32_t, sc_int32, 224, it> HGT_ELL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t HGT_SEA;//!<  GPS data, height above mean sea level              [ 1 mm] 
    #else
    field<int32_t, sc_int32, 256, it> HGT_SEA;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t HOR_ACC;//!<  GPS data, horizontal accuracy estimate             [ 1 mm] 
    #else
    field<uint32_t, sc_uint32, 288, it> HOR_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t VERT_ACC;//!<  GPS data, vertical accuracy estimate               [ 1 mm] 
    #else
    field<uint32_t, sc_uint32, 320, it> VERT_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS1;//!<  GPS data, 0=no fix; 1=dead reckoning; 2=2D-fix;  3=3D-fix; 4=GPS+dead reckoning;5=Time only fix;255=undefined fix 
    #else
    field<uint8_t, sc_uint8, 352, it> STATUS1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS2;//!<  GPS data, 1 .. gpsfixOK; 2 .. diffSoln;  4 .. wknSet; 8 .. towSet 
    #else
    field<uint8_t, sc_uint8, 360, it> STATUS2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS3;//!<  GPS data, 0= no DGPS; 1=PR+PRR Correction;  2=PR+PRR+CP Correction;  3=high accuracy PR+PRR+CP Correction 
    #else
    field<uint8_t, sc_uint8, 368, it> STATUS3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t LEAP_SEC;//!<  GPS data, leap seconds (GPS-UTC)                   [ 1 s ] 
    #else
    field<int8_t, sc_int8, 376, it> LEAP_SEC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  time stamp of PPS signal measured with internal clock in units of units.time_unit 
    #else
    field<uint32_t, sc_uint32, 384, it> systime;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_gps_ts& operator=(const hk_gps_ts<ito>& o) {
        TOWms = o.TOWms;
        ECEFX = o.ECEFX;
        ECEFY = o.ECEFY;
        ECEFZ = o.ECEFZ;
        POS_ACC = o.POS_ACC;
        LONG = o.LONG;
        LAT = o.LAT;
        HGT_ELL = o.HGT_ELL;
        HGT_SEA = o.HGT_SEA;
        HOR_ACC = o.HOR_ACC;
        VERT_ACC = o.VERT_ACC;
        STATUS1 = o.STATUS1;
        STATUS2 = o.STATUS2;
        STATUS3 = o.STATUS3;
        LEAP_SEC = o.LEAP_SEC;
        systime = o.systime;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct hk_gps_ts<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_gps_ts<rebind_it> type;
    };

    enum { id_main = 10005, id_sub = 1};

    uint32_t         TOWms;
    int32_t          ECEFX;
    int32_t          ECEFY;
    int32_t          ECEFZ;
    uint32_t         POS_ACC;
    int32_t          LONG;
    int32_t          LAT;
    int32_t          HGT_ELL;
    int32_t          HGT_SEA;
    uint32_t         HOR_ACC;
    uint32_t         VERT_ACC;
    uint8_t          STATUS1;
    uint8_t          STATUS2;
    uint8_t          STATUS3;
    int8_t           LEAP_SEC;
    uint32_t         systime;

    hk_gps_ts() {}
    template<class it>
    hk_gps_ts(const hk_gps_ts<it>& o) {
        TOWms = o.TOWms;
        ECEFX = o.ECEFX;
        ECEFY = o.ECEFY;
        ECEFZ = o.ECEFZ;
        POS_ACC = o.POS_ACC;
        LONG = o.LONG;
        LAT = o.LAT;
        HGT_ELL = o.HGT_ELL;
        HGT_SEA = o.HGT_SEA;
        HOR_ACC = o.HOR_ACC;
        VERT_ACC = o.VERT_ACC;
        STATUS1 = o.STATUS1;
        STATUS2 = o.STATUS2;
        STATUS3 = o.STATUS3;
        LEAP_SEC = o.LEAP_SEC;
        systime = o.systime;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_gps_ts<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.TOWms;
        s << ", " << x.ECEFX;
        s << ", " << x.ECEFY;
        s << ", " << x.ECEFZ;
        s << ", " << x.POS_ACC;
        s << ", " << x.LONG;
        s << ", " << x.LAT;
        s << ", " << x.HGT_ELL;
        s << ", " << x.HGT_SEA;
        s << ", " << x.HOR_ACC;
        s << ", " << x.VERT_ACC;
        s << ", " << x.STATUS1;
        s << ", " << x.STATUS2;
        s << ", " << x.STATUS3;
        s << ", " << x.LEAP_SEC;
        s << ", " << x.systime;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_gps_ts<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.TOWms;
        s >> ',' >> x.ECEFX;
        s >> ',' >> x.ECEFY;
        s >> ',' >> x.ECEFZ;
        s >> ',' >> x.POS_ACC;
        s >> ',' >> x.LONG;
        s >> ',' >> x.LAT;
        s >> ',' >> x.HGT_ELL;
        s >> ',' >> x.HGT_SEA;
        s >> ',' >> x.HOR_ACC;
        s >> ',' >> x.VERT_ACC;
        s >> ',' >> x.STATUS1;
        s >> ',' >> x.STATUS2;
        s >> ',' >> x.STATUS3;
        s >> ',' >> x.LEAP_SEC;
        s >> ',' >> x.systime;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! GPS data

//! <para>This package belongs to the predefined selectors:</para>
//! <para>status</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_gps_ts_status
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_gps_ts_status<rebind_it> type;
    };

    typedef it iterator_type;

    hk_gps_ts_status(it begin, it end, bool dirty=false)
        : TOWms(begin)
        , ECEFX(begin)
        , ECEFY(begin)
        , ECEFZ(begin)
        , POS_ACC(begin)
        , LONG(begin)
        , LAT(begin)
        , HGT_ELL(begin)
        , HGT_SEA(begin)
        , HOR_ACC(begin)
        , VERT_ACC(begin)
        , STATUS1(begin)
        , STATUS2(begin)
        , STATUS3(begin)
        , LEAP_SEC(begin)
        , systime(begin)
        , SYNC_STATUS(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10005, id_sub = 2};
    #ifndef DOXYGEN
    enum { max_bit_width = 424};
    it begin() const { return TOWms.begin(); }
    it end() const { return SYNC_STATUS.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t TOWms;//!<  GPS data, GPS Time of Week                         [ 1 ms] 
    #else
    field<uint32_t, sc_uint32, 0, it> TOWms;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFX;//!<  GPS data, ECEF X coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 32, it> ECEFX;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFY;//!<  GPS data, ECEF Y coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 64, it> ECEFY;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFZ;//!<  GPS data, ECEF Z coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 96, it> ECEFZ;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t POS_ACC;//!<  GPS data, position accuracy estimate               [ 1 cm] 
    #else
    field<uint32_t, sc_uint32, 128, it> POS_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t LONG;//!<  GPS data, longitude                                [1e-7deg] 
    #else
    field<int32_t, sc_int32, 160, it> LONG;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t LAT;//!<  GPS data, latitude                                 [1e-7deg] 
    #else
    field<int32_t, sc_int32, 192, it> LAT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t HGT_ELL;//!<  GPS data, height above ellipsoid                   [ 1 mm] 
    #else
    field<int32_t, sc_int32, 224, it> HGT_ELL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t HGT_SEA;//!<  GPS data, height above mean sea level              [ 1 mm] 
    #else
    field<int32_t, sc_int32, 256, it> HGT_SEA;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t HOR_ACC;//!<  GPS data, horizontal accuracy estimate             [ 1 mm] 
    #else
    field<uint32_t, sc_uint32, 288, it> HOR_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t VERT_ACC;//!<  GPS data, vertical accuracy estimate               [ 1 mm] 
    #else
    field<uint32_t, sc_uint32, 320, it> VERT_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS1;//!<  GPS data, 0=no fix; 1=dead reckoning; 2=2D-fix;  3=3D-fix; 4=GPS+dead reckoning;5=Time only fix;255=undefined fix 
    #else
    field<uint8_t, sc_uint8, 352, it> STATUS1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS2;//!<  GPS data, 1 .. gpsfixOK; 2 .. diffSoln;  4 .. wknSet; 8 .. towSet 
    #else
    field<uint8_t, sc_uint8, 360, it> STATUS2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS3;//!<  GPS data, 0= no DGPS; 1=PR+PRR Correction;  2=PR+PRR+CP Correction;  3=high accuracy PR+PRR+CP Correction 
    #else
    field<uint8_t, sc_uint8, 368, it> STATUS3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t LEAP_SEC;//!<  GPS data, leap seconds (GPS-UTC)                   [ 1 s ] 
    #else
    field<int8_t, sc_int8, 376, it> LEAP_SEC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  internal time in units of units.time_unit 
    #else
    field<uint32_t, sc_uint32, 384, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t SYNC_STATUS;//!<  PPS pulse and GPS data receive indicator  bit 0: at least one valid gps information seen  bit 1: valid gps information seen during last second 
    #else
    field<uint8_t, sc_uint8, 416, it> SYNC_STATUS;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_gps_ts_status& operator=(const hk_gps_ts_status<ito>& o) {
        TOWms = o.TOWms;
        ECEFX = o.ECEFX;
        ECEFY = o.ECEFY;
        ECEFZ = o.ECEFZ;
        POS_ACC = o.POS_ACC;
        LONG = o.LONG;
        LAT = o.LAT;
        HGT_ELL = o.HGT_ELL;
        HGT_SEA = o.HGT_SEA;
        HOR_ACC = o.HOR_ACC;
        VERT_ACC = o.VERT_ACC;
        STATUS1 = o.STATUS1;
        STATUS2 = o.STATUS2;
        STATUS3 = o.STATUS3;
        LEAP_SEC = o.LEAP_SEC;
        systime = o.systime;
        SYNC_STATUS = o.SYNC_STATUS;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct hk_gps_ts_status<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_gps_ts_status<rebind_it> type;
    };

    enum { id_main = 10005, id_sub = 2};

    uint32_t         TOWms;
    int32_t          ECEFX;
    int32_t          ECEFY;
    int32_t          ECEFZ;
    uint32_t         POS_ACC;
    int32_t          LONG;
    int32_t          LAT;
    int32_t          HGT_ELL;
    int32_t          HGT_SEA;
    uint32_t         HOR_ACC;
    uint32_t         VERT_ACC;
    uint8_t          STATUS1;
    uint8_t          STATUS2;
    uint8_t          STATUS3;
    int8_t           LEAP_SEC;
    uint32_t         systime;
    uint8_t          SYNC_STATUS;

    hk_gps_ts_status() {}
    template<class it>
    hk_gps_ts_status(const hk_gps_ts_status<it>& o) {
        TOWms = o.TOWms;
        ECEFX = o.ECEFX;
        ECEFY = o.ECEFY;
        ECEFZ = o.ECEFZ;
        POS_ACC = o.POS_ACC;
        LONG = o.LONG;
        LAT = o.LAT;
        HGT_ELL = o.HGT_ELL;
        HGT_SEA = o.HGT_SEA;
        HOR_ACC = o.HOR_ACC;
        VERT_ACC = o.VERT_ACC;
        STATUS1 = o.STATUS1;
        STATUS2 = o.STATUS2;
        STATUS3 = o.STATUS3;
        LEAP_SEC = o.LEAP_SEC;
        systime = o.systime;
        SYNC_STATUS = o.SYNC_STATUS;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_gps_ts_status<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.TOWms;
        s << ", " << x.ECEFX;
        s << ", " << x.ECEFY;
        s << ", " << x.ECEFZ;
        s << ", " << x.POS_ACC;
        s << ", " << x.LONG;
        s << ", " << x.LAT;
        s << ", " << x.HGT_ELL;
        s << ", " << x.HGT_SEA;
        s << ", " << x.HOR_ACC;
        s << ", " << x.VERT_ACC;
        s << ", " << x.STATUS1;
        s << ", " << x.STATUS2;
        s << ", " << x.STATUS3;
        s << ", " << x.LEAP_SEC;
        s << ", " << x.systime;
        s << ", " << x.SYNC_STATUS;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_gps_ts_status<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.TOWms;
        s >> ',' >> x.ECEFX;
        s >> ',' >> x.ECEFY;
        s >> ',' >> x.ECEFZ;
        s >> ',' >> x.POS_ACC;
        s >> ',' >> x.LONG;
        s >> ',' >> x.LAT;
        s >> ',' >> x.HGT_ELL;
        s >> ',' >> x.HGT_SEA;
        s >> ',' >> x.HOR_ACC;
        s >> ',' >> x.VERT_ACC;
        s >> ',' >> x.STATUS1;
        s >> ',' >> x.STATUS2;
        s >> ',' >> x.STATUS3;
        s >> ',' >> x.LEAP_SEC;
        s >> ',' >> x.systime;
        s >> ',' >> x.SYNC_STATUS;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! GPS data

//! <para>This package belongs to the predefined selectors:</para>
//! <para>status</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_gps_ts_status_dop
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_gps_ts_status_dop<rebind_it> type;
    };

    typedef it iterator_type;

    hk_gps_ts_status_dop(it begin, it end, bool dirty=false)
        : TOWms(begin)
        , ECEFX(begin)
        , ECEFY(begin)
        , ECEFZ(begin)
        , POS_ACC(begin)
        , LONG(begin)
        , LAT(begin)
        , HGT_ELL(begin)
        , HGT_SEA(begin)
        , HOR_ACC(begin)
        , VERT_ACC(begin)
        , STATUS1(begin)
        , STATUS2(begin)
        , STATUS3(begin)
        , LEAP_SEC(begin)
        , systime(begin)
        , SYNC_STATUS(begin)
        , SAT_NUM(begin)
        , GEOM_DILUT(begin)
        , TIME_DILUT(begin)
        , VERT_DILUT(begin)
        , HOR_DILUT(begin)
        , POS_DILUT(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10005, id_sub = 3};
    #ifndef DOXYGEN
    enum { max_bit_width = 512};
    it begin() const { return TOWms.begin(); }
    it end() const { return POS_DILUT.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t TOWms;//!<  GPS data, GPS Time of Week                         [ 1 ms] 
    #else
    field<uint32_t, sc_uint32, 0, it> TOWms;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFX;//!<  GPS data, ECEF X coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 32, it> ECEFX;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFY;//!<  GPS data, ECEF Y coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 64, it> ECEFY;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ECEFZ;//!<  GPS data, ECEF Z coordinate                        [ 1 cm] 
    #else
    field<int32_t, sc_int32, 96, it> ECEFZ;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t POS_ACC;//!<  GPS data, position accuracy estimate               [ 1 cm] 
    #else
    field<uint32_t, sc_uint32, 128, it> POS_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t LONG;//!<  GPS data, longitude                                [1e-7deg] 
    #else
    field<int32_t, sc_int32, 160, it> LONG;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t LAT;//!<  GPS data, latitude                                 [1e-7deg] 
    #else
    field<int32_t, sc_int32, 192, it> LAT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t HGT_ELL;//!<  GPS data, height above ellipsoid                   [ 1 mm] 
    #else
    field<int32_t, sc_int32, 224, it> HGT_ELL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t HGT_SEA;//!<  GPS data, height above mean sea level              [ 1 mm] 
    #else
    field<int32_t, sc_int32, 256, it> HGT_SEA;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t HOR_ACC;//!<  GPS data, horizontal accuracy estimate             [ 1 mm] 
    #else
    field<uint32_t, sc_uint32, 288, it> HOR_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t VERT_ACC;//!<  GPS data, vertical accuracy estimate               [ 1 mm] 
    #else
    field<uint32_t, sc_uint32, 320, it> VERT_ACC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS1;//!<  GPS data, 0=no fix; 1=dead reckoning; 2=2D-fix;  3=3D-fix; 4=GPS+dead reckoning;5=Time only fix;255=undefined fix 
    #else
    field<uint8_t, sc_uint8, 352, it> STATUS1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS2;//!<  GPS data, 1 .. gpsfixOK; 2 .. diffSoln;  4 .. wknSet; 8 .. towSet 
    #else
    field<uint8_t, sc_uint8, 360, it> STATUS2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t STATUS3;//!<  GPS data, 0= no DGPS; 1=PR+PRR Correction;  2=PR+PRR+CP Correction;  3=high accuracy PR+PRR+CP Correction 
    #else
    field<uint8_t, sc_uint8, 368, it> STATUS3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t LEAP_SEC;//!<  GPS data, leap seconds (GPS-UTC)                   [ 1 s ] 
    #else
    field<int8_t, sc_int8, 376, it> LEAP_SEC;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  internal time in units of units.time_unit 
    #else
    field<uint32_t, sc_uint32, 384, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t SYNC_STATUS;//!<  PPS pulse and GPS data receive indicator  bit 0: at least one valid gps information seen  bit 1: valid gps information seen during last second 
    #else
    field<uint8_t, sc_uint8, 416, it> SYNC_STATUS;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t SAT_NUM;//!<  GPS data, number of satelites used 
    #else
    field<uint8_t, sc_uint8, 424, it> SAT_NUM;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t GEOM_DILUT;//!<  GPS data, geometric dilution of position        [0.01] 
    #else
    field<uint16_t, sc_uint16, 432, it> GEOM_DILUT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t TIME_DILUT;//!<  GPS data, time dilution        [0.01] 
    #else
    field<uint16_t, sc_uint16, 448, it> TIME_DILUT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t VERT_DILUT;//!<  GPS data, vertical dilution (1D)       [0.01] 
    #else
    field<uint16_t, sc_uint16, 464, it> VERT_DILUT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t HOR_DILUT;//!<  GPS data, horizontal dilution (2D)       [0.01] 
    #else
    field<uint16_t, sc_uint16, 480, it> HOR_DILUT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t POS_DILUT;//!<  GPS data, position dilution (3D)       [0.01] 
    #else
    field<uint16_t, sc_uint16, 496, it> POS_DILUT;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_gps_ts_status_dop& operator=(const hk_gps_ts_status_dop<ito>& o) {
        TOWms = o.TOWms;
        ECEFX = o.ECEFX;
        ECEFY = o.ECEFY;
        ECEFZ = o.ECEFZ;
        POS_ACC = o.POS_ACC;
        LONG = o.LONG;
        LAT = o.LAT;
        HGT_ELL = o.HGT_ELL;
        HGT_SEA = o.HGT_SEA;
        HOR_ACC = o.HOR_ACC;
        VERT_ACC = o.VERT_ACC;
        STATUS1 = o.STATUS1;
        STATUS2 = o.STATUS2;
        STATUS3 = o.STATUS3;
        LEAP_SEC = o.LEAP_SEC;
        systime = o.systime;
        SYNC_STATUS = o.SYNC_STATUS;
        SAT_NUM = o.SAT_NUM;
        GEOM_DILUT = o.GEOM_DILUT;
        TIME_DILUT = o.TIME_DILUT;
        VERT_DILUT = o.VERT_DILUT;
        HOR_DILUT = o.HOR_DILUT;
        POS_DILUT = o.POS_DILUT;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct hk_gps_ts_status_dop<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_gps_ts_status_dop<rebind_it> type;
    };

    enum { id_main = 10005, id_sub = 3};

    uint32_t         TOWms;
    int32_t          ECEFX;
    int32_t          ECEFY;
    int32_t          ECEFZ;
    uint32_t         POS_ACC;
    int32_t          LONG;
    int32_t          LAT;
    int32_t          HGT_ELL;
    int32_t          HGT_SEA;
    uint32_t         HOR_ACC;
    uint32_t         VERT_ACC;
    uint8_t          STATUS1;
    uint8_t          STATUS2;
    uint8_t          STATUS3;
    int8_t           LEAP_SEC;
    uint32_t         systime;
    uint8_t          SYNC_STATUS;
    uint8_t          SAT_NUM;
    uint16_t         GEOM_DILUT;
    uint16_t         TIME_DILUT;
    uint16_t         VERT_DILUT;
    uint16_t         HOR_DILUT;
    uint16_t         POS_DILUT;

    hk_gps_ts_status_dop() {}
    template<class it>
    hk_gps_ts_status_dop(const hk_gps_ts_status_dop<it>& o) {
        TOWms = o.TOWms;
        ECEFX = o.ECEFX;
        ECEFY = o.ECEFY;
        ECEFZ = o.ECEFZ;
        POS_ACC = o.POS_ACC;
        LONG = o.LONG;
        LAT = o.LAT;
        HGT_ELL = o.HGT_ELL;
        HGT_SEA = o.HGT_SEA;
        HOR_ACC = o.HOR_ACC;
        VERT_ACC = o.VERT_ACC;
        STATUS1 = o.STATUS1;
        STATUS2 = o.STATUS2;
        STATUS3 = o.STATUS3;
        LEAP_SEC = o.LEAP_SEC;
        systime = o.systime;
        SYNC_STATUS = o.SYNC_STATUS;
        SAT_NUM = o.SAT_NUM;
        GEOM_DILUT = o.GEOM_DILUT;
        TIME_DILUT = o.TIME_DILUT;
        VERT_DILUT = o.VERT_DILUT;
        HOR_DILUT = o.HOR_DILUT;
        POS_DILUT = o.POS_DILUT;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_gps_ts_status_dop<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.TOWms;
        s << ", " << x.ECEFX;
        s << ", " << x.ECEFY;
        s << ", " << x.ECEFZ;
        s << ", " << x.POS_ACC;
        s << ", " << x.LONG;
        s << ", " << x.LAT;
        s << ", " << x.HGT_ELL;
        s << ", " << x.HGT_SEA;
        s << ", " << x.HOR_ACC;
        s << ", " << x.VERT_ACC;
        s << ", " << x.STATUS1;
        s << ", " << x.STATUS2;
        s << ", " << x.STATUS3;
        s << ", " << x.LEAP_SEC;
        s << ", " << x.systime;
        s << ", " << x.SYNC_STATUS;
        s << ", " << x.SAT_NUM;
        s << ", " << x.GEOM_DILUT;
        s << ", " << x.TIME_DILUT;
        s << ", " << x.VERT_DILUT;
        s << ", " << x.HOR_DILUT;
        s << ", " << x.POS_DILUT;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_gps_ts_status_dop<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.TOWms;
        s >> ',' >> x.ECEFX;
        s >> ',' >> x.ECEFY;
        s >> ',' >> x.ECEFZ;
        s >> ',' >> x.POS_ACC;
        s >> ',' >> x.LONG;
        s >> ',' >> x.LAT;
        s >> ',' >> x.HGT_ELL;
        s >> ',' >> x.HGT_SEA;
        s >> ',' >> x.HOR_ACC;
        s >> ',' >> x.VERT_ACC;
        s >> ',' >> x.STATUS1;
        s >> ',' >> x.STATUS2;
        s >> ',' >> x.STATUS3;
        s >> ',' >> x.LEAP_SEC;
        s >> ',' >> x.systime;
        s >> ',' >> x.SYNC_STATUS;
        s >> ',' >> x.SAT_NUM;
        s >> ',' >> x.GEOM_DILUT;
        s >> ',' >> x.TIME_DILUT;
        s >> ',' >> x.VERT_DILUT;
        s >> ',' >> x.HOR_DILUT;
        s >> ',' >> x.POS_DILUT;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! inclination sensor

//! <para>This package belongs to the predefined selectors:</para>
//! <para>status</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_incl
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_incl<rebind_it> type;
    };

    typedef it iterator_type;

    hk_incl(it begin, it end, bool dirty=false)
        : ROLL(begin)
        , PITCH(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10006, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 64};
    it begin() const { return ROLL.begin(); }
    it end() const { return RESERVED_03.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t ROLL;//!<  inclination angle along x-axis   [0.001 deg ] 
    #else
    field<int16_t, sc_int16, 0, it> ROLL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t PITCH;//!<  inclination angle along y-axis   [0.001 deg ] 
    #else
    field<int16_t, sc_int16, 16, it> PITCH;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;//!<  INTERNAL ONLY inclination sensor 1 temperature [ 0.1 deg C] 
    #else
    field<int16_t, sc_int16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_03;//!<  INTERNAL ONLY inclination sensor 2 temperature [ 0.1 deg C] 
    #else
    field<int16_t, sc_int16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_incl& operator=(const hk_incl<ito>& o) {
        ROLL = o.ROLL;
        PITCH = o.PITCH;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct hk_incl<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_incl<rebind_it> type;
    };

    enum { id_main = 10006, id_sub = 0};

    int16_t          ROLL;
    int16_t          PITCH;
    int16_t          RESERVED_02;
    int16_t          RESERVED_03;

    hk_incl() {}
    template<class it>
    hk_incl(const hk_incl<it>& o) {
        ROLL = o.ROLL;
        PITCH = o.PITCH;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_incl<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.ROLL;
        s << ", " << x.PITCH;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_incl<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.ROLL;
        s >> ',' >> x.PITCH;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! inclination sensor

//! <para>This package belongs to the predefined selectors:</para>
//! <para>status</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_incl_4axes
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_incl_4axes<rebind_it> type;
    };

    typedef it iterator_type;

    hk_incl_4axes(it begin, it end, bool dirty=false)
        : ROLL(begin)
        , PITCH(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10012, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 128};
    it begin() const { return ROLL.begin(); }
    it end() const { return RESERVED_05.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t ROLL;//!<  inclination angle along x-axis [0.001 deg ] 
    #else
    field<int32_t, sc_int32, 0, it> ROLL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t PITCH;//!<  inclination angle along y-axis [0.001 deg ] 
    #else
    field<int32_t, sc_int32, 32, it> PITCH;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;//!<  INTERNAL ONLY inclination sensor 1-1 temperature [ 0.1 deg C] 
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_03;//!<  INTERNAL ONLY inclination sensor 1-2 temperature [ 0.1 deg C] 
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_04;//!<  INTERNAL ONLY inclination sensor 2-1 temperature [ 0.1 deg C] 
    #else
    field<int16_t, sc_int16, 96, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;//!<  INTERNAL ONLY inclination sensor 2-2 temperature [ 0.1 deg C] 
    #else
    field<int16_t, sc_int16, 112, it> RESERVED_05;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_incl_4axes& operator=(const hk_incl_4axes<ito>& o) {
        ROLL = o.ROLL;
        PITCH = o.PITCH;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct hk_incl_4axes<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_incl_4axes<rebind_it> type;
    };

    enum { id_main = 10012, id_sub = 0};

    int32_t          ROLL;
    int32_t          PITCH;
    int16_t          RESERVED_02;
    int16_t          RESERVED_03;
    int16_t          RESERVED_04;
    int16_t          RESERVED_05;

    hk_incl_4axes() {}
    template<class it>
    hk_incl_4axes(const hk_incl_4axes<it>& o) {
        ROLL = o.ROLL;
        PITCH = o.PITCH;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_incl_4axes<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.ROLL;
        s << ", " << x.PITCH;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_incl_4axes<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.ROLL;
        s >> ',' >> x.PITCH;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_pwr
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_pwr<rebind_it> type;
    };

    typedef it iterator_type;

    hk_pwr(it begin, it end, bool dirty=false)
        : V_IN_1(begin)
        , V_IN_2(begin)
        , V_IN_3(begin)
        , V_IN_SEL(begin)
        , A_IN_SEL(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10003, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 192};
    it begin() const { return V_IN_1.begin(); }
    it end() const { return RESERVED_11.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_1;
    #else
    field<int16_t, sc_int16, 0, it> V_IN_1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_2;
    #else
    field<int16_t, sc_int16, 16, it> V_IN_2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_3;
    #else
    field<int16_t, sc_int16, 32, it> V_IN_3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t V_IN_SEL;
    #else
    field<uint16_t, sc_uint16, 48, it> V_IN_SEL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t A_IN_SEL;
    #else
    field<uint16_t, sc_uint16, 64, it> A_IN_SEL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_05;
    #else
    field<uint16_t, sc_uint16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_06;
    #else
    field<uint16_t, sc_uint16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_10;
    #else
    field<uint8_t, sc_uint8, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_11[3];
    #else
    array<uint8_t, 3, sc_uint8, 168, it> RESERVED_11;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_pwr& operator=(const hk_pwr<ito>& o) {
        V_IN_1 = o.V_IN_1;
        V_IN_2 = o.V_IN_2;
        V_IN_3 = o.V_IN_3;
        V_IN_SEL = o.V_IN_SEL;
        A_IN_SEL = o.A_IN_SEL;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        for(unsigned n=0; n<3; ++n) RESERVED_11[n] = o.RESERVED_11[n];
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_pwr<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_pwr<rebind_it> type;
    };

    enum { id_main = 10003, id_sub = 0};

    int16_t          V_IN_1;
    int16_t          V_IN_2;
    int16_t          V_IN_3;
    uint16_t         V_IN_SEL;
    uint16_t         A_IN_SEL;
    uint16_t         RESERVED_05;
    uint16_t         RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    uint8_t          RESERVED_10;
    uint8_t          RESERVED_11[3];

    hk_pwr() {}
    template<class it>
    hk_pwr(const hk_pwr<it>& o) {
        V_IN_1 = o.V_IN_1;
        V_IN_2 = o.V_IN_2;
        V_IN_3 = o.V_IN_3;
        V_IN_SEL = o.V_IN_SEL;
        A_IN_SEL = o.A_IN_SEL;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        for(unsigned n=0; n<3; ++n) RESERVED_11[n] = o.RESERVED_11[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_pwr<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.V_IN_1;
        s << ", " << x.V_IN_2;
        s << ", " << x.V_IN_3;
        s << ", " << x.V_IN_SEL;
        s << ", " << x.A_IN_SEL;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", "; write_array(s, 3, x.RESERVED_11);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_pwr<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.V_IN_1;
        s >> ',' >> x.V_IN_2;
        s >> ',' >> x.V_IN_3;
        s >> ',' >> x.V_IN_SEL;
        s >> ',' >> x.A_IN_SEL;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ','; read_array(s, 3, x.RESERVED_11);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_pwr_1
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_pwr_1<rebind_it> type;
    };

    typedef it iterator_type;

    hk_pwr_1(it begin, it end, bool dirty=false)
        : V_IN_1(begin)
        , V_IN_2(begin)
        , V_IN_3(begin)
        , V_IN_SEL(begin)
        , A_IN_SEL(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10007, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 200};
    it begin() const { return V_IN_1.begin(); }
    it end() const { return RESERVED_11.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_1;
    #else
    field<int16_t, sc_int16, 0, it> V_IN_1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_2;
    #else
    field<int16_t, sc_int16, 16, it> V_IN_2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t V_IN_3;
    #else
    field<int16_t, sc_int16, 32, it> V_IN_3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t V_IN_SEL;
    #else
    field<uint16_t, sc_uint16, 48, it> V_IN_SEL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t A_IN_SEL;
    #else
    field<uint16_t, sc_uint16, 64, it> A_IN_SEL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_05;
    #else
    field<uint16_t, sc_uint16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_06;
    #else
    field<uint16_t, sc_uint16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_10;
    #else
    field<uint16_t, sc_uint16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t RESERVED_11[3];
    #else
    array<uint8_t, 3, sc_uint8, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_pwr_1& operator=(const hk_pwr_1<ito>& o) {
        V_IN_1 = o.V_IN_1;
        V_IN_2 = o.V_IN_2;
        V_IN_3 = o.V_IN_3;
        V_IN_SEL = o.V_IN_SEL;
        A_IN_SEL = o.A_IN_SEL;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        for(unsigned n=0; n<3; ++n) RESERVED_11[n] = o.RESERVED_11[n];
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_pwr_1<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_pwr_1<rebind_it> type;
    };

    enum { id_main = 10007, id_sub = 0};

    int16_t          V_IN_1;
    int16_t          V_IN_2;
    int16_t          V_IN_3;
    uint16_t         V_IN_SEL;
    uint16_t         A_IN_SEL;
    uint16_t         RESERVED_05;
    uint16_t         RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    uint16_t         RESERVED_10;
    uint8_t          RESERVED_11[3];

    hk_pwr_1() {}
    template<class it>
    hk_pwr_1(const hk_pwr_1<it>& o) {
        V_IN_1 = o.V_IN_1;
        V_IN_2 = o.V_IN_2;
        V_IN_3 = o.V_IN_3;
        V_IN_SEL = o.V_IN_SEL;
        A_IN_SEL = o.A_IN_SEL;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        for(unsigned n=0; n<3; ++n) RESERVED_11[n] = o.RESERVED_11[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_pwr_1<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.V_IN_1;
        s << ", " << x.V_IN_2;
        s << ", " << x.V_IN_3;
        s << ", " << x.V_IN_SEL;
        s << ", " << x.A_IN_SEL;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", "; write_array(s, 3, x.RESERVED_11);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_pwr_1<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.V_IN_1;
        s >> ',' >> x.V_IN_2;
        s >> ',' >> x.V_IN_3;
        s >> ',' >> x.V_IN_SEL;
        s >> ',' >> x.A_IN_SEL;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ','; read_array(s, 3, x.RESERVED_11);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rad
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rad<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rad(it begin, it end, bool dirty=false)
        : RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10010, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 96};
    it begin() const { return RESERVED_01.begin(); }
    it end() const { return RESERVED_06.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_02;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 32, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_04;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_06;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rad& operator=(const hk_rad<ito>& o) {
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rad<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rad<rebind_it> type;
    };

    enum { id_main = 10010, id_sub = 0};

    uint16_t         RESERVED_01;
    uint16_t         RESERVED_02;
    uint16_t         RESERVED_03;
    uint16_t         RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;

    hk_rad() {}
    template<class it>
    hk_rad(const hk_rad<it>& o) {
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rad<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rad<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rng
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rng(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , RESERVED_13(begin)
        , RESERVED_14(begin)
        , RESERVED_15(begin)
        , RESERVED_16(begin)
        , RESERVED_17(begin)
        , RESERVED_18(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10001, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 304};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_18.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;
    #else
    field<int16_t, sc_int16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_04;
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_10;
    #else
    field<int16_t, sc_int16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_12;
    #else
    field<uint16_t, sc_uint16, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_13;
    #else
    field<int16_t, sc_int16, 208, it> RESERVED_13;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_14;
    #else
    field<int16_t, sc_int16, 224, it> RESERVED_14;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_15;
    #else
    field<int16_t, sc_int16, 240, it> RESERVED_15;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_16;
    #else
    field<int16_t, sc_int16, 256, it> RESERVED_16;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_17;
    #else
    field<uint16_t, sc_uint16, 272, it> RESERVED_17;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_18;
    #else
    field<uint16_t, sc_uint16, 288, it> RESERVED_18;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rng& operator=(const hk_rng<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rng<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng<rebind_it> type;
    };

    enum { id_main = 10001, id_sub = 0};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    int16_t          RESERVED_02;
    uint16_t         RESERVED_03;
    int16_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    int16_t          RESERVED_10;
    uint16_t         RESERVED_11;
    uint16_t         RESERVED_12;
    int16_t          RESERVED_13;
    int16_t          RESERVED_14;
    int16_t          RESERVED_15;
    int16_t          RESERVED_16;
    uint16_t         RESERVED_17;
    uint16_t         RESERVED_18;

    hk_rng() {}
    template<class it>
    hk_rng(const hk_rng<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rng<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.RESERVED_13;
        s << ", " << x.RESERVED_14;
        s << ", " << x.RESERVED_15;
        s << ", " << x.RESERVED_16;
        s << ", " << x.RESERVED_17;
        s << ", " << x.RESERVED_18;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rng<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.RESERVED_13;
        s >> ',' >> x.RESERVED_14;
        s >> ',' >> x.RESERVED_15;
        s >> ',' >> x.RESERVED_16;
        s >> ',' >> x.RESERVED_17;
        s >> ',' >> x.RESERVED_18;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rng_1
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_1<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rng_1(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , RESERVED_13(begin)
        , RESERVED_14(begin)
        , RESERVED_15(begin)
        , RESERVED_16(begin)
        , RESERVED_17(begin)
        , RESERVED_18(begin)
        , RESERVED_19(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10001, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 336};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_19.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;
    #else
    field<int16_t, sc_int16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_04;
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_10;
    #else
    field<int16_t, sc_int16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_12;
    #else
    field<uint16_t, sc_uint16, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_13;
    #else
    field<int16_t, sc_int16, 208, it> RESERVED_13;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_14;
    #else
    field<int16_t, sc_int16, 224, it> RESERVED_14;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_15;
    #else
    field<int16_t, sc_int16, 240, it> RESERVED_15;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_16;
    #else
    field<int16_t, sc_int16, 256, it> RESERVED_16;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_17;
    #else
    field<uint16_t, sc_uint16, 272, it> RESERVED_17;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_18;
    #else
    field<uint16_t, sc_uint16, 288, it> RESERVED_18;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_19;
    #else
    field<uint32_t, sc_uint32, 304, it> RESERVED_19;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rng_1& operator=(const hk_rng_1<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rng_1<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_1<rebind_it> type;
    };

    enum { id_main = 10001, id_sub = 1};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    int16_t          RESERVED_02;
    uint16_t         RESERVED_03;
    int16_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    int16_t          RESERVED_10;
    uint16_t         RESERVED_11;
    uint16_t         RESERVED_12;
    int16_t          RESERVED_13;
    int16_t          RESERVED_14;
    int16_t          RESERVED_15;
    int16_t          RESERVED_16;
    uint16_t         RESERVED_17;
    uint16_t         RESERVED_18;
    uint32_t         RESERVED_19;

    hk_rng_1() {}
    template<class it>
    hk_rng_1(const hk_rng_1<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rng_1<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.RESERVED_13;
        s << ", " << x.RESERVED_14;
        s << ", " << x.RESERVED_15;
        s << ", " << x.RESERVED_16;
        s << ", " << x.RESERVED_17;
        s << ", " << x.RESERVED_18;
        s << ", " << x.RESERVED_19;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rng_1<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.RESERVED_13;
        s >> ',' >> x.RESERVED_14;
        s >> ',' >> x.RESERVED_15;
        s >> ',' >> x.RESERVED_16;
        s >> ',' >> x.RESERVED_17;
        s >> ',' >> x.RESERVED_18;
        s >> ',' >> x.RESERVED_19;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rng_2
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_2<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rng_2(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , RESERVED_13(begin)
        , RESERVED_14(begin)
        , RESERVED_15(begin)
        , RESERVED_16(begin)
        , RESERVED_17(begin)
        , RESERVED_18(begin)
        , RESERVED_19(begin)
        , RESERVED_20(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10001, id_sub = 2};
    #ifndef DOXYGEN
    enum { max_bit_width = 352};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_20.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;
    #else
    field<int16_t, sc_int16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_04;
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_10;
    #else
    field<int16_t, sc_int16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_12;
    #else
    field<uint16_t, sc_uint16, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_13;
    #else
    field<int16_t, sc_int16, 208, it> RESERVED_13;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_14;
    #else
    field<int16_t, sc_int16, 224, it> RESERVED_14;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_15;
    #else
    field<int16_t, sc_int16, 240, it> RESERVED_15;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_16;
    #else
    field<int16_t, sc_int16, 256, it> RESERVED_16;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_17;
    #else
    field<uint16_t, sc_uint16, 272, it> RESERVED_17;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_18;
    #else
    field<uint16_t, sc_uint16, 288, it> RESERVED_18;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_19;
    #else
    field<uint32_t, sc_uint32, 304, it> RESERVED_19;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_20;
    #else
    field<int16_t, sc_int16, 336, it> RESERVED_20;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rng_2& operator=(const hk_rng_2<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rng_2<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_2<rebind_it> type;
    };

    enum { id_main = 10001, id_sub = 2};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    int16_t          RESERVED_02;
    uint16_t         RESERVED_03;
    int16_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    int16_t          RESERVED_10;
    uint16_t         RESERVED_11;
    uint16_t         RESERVED_12;
    int16_t          RESERVED_13;
    int16_t          RESERVED_14;
    int16_t          RESERVED_15;
    int16_t          RESERVED_16;
    uint16_t         RESERVED_17;
    uint16_t         RESERVED_18;
    uint32_t         RESERVED_19;
    int16_t          RESERVED_20;

    hk_rng_2() {}
    template<class it>
    hk_rng_2(const hk_rng_2<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rng_2<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.RESERVED_13;
        s << ", " << x.RESERVED_14;
        s << ", " << x.RESERVED_15;
        s << ", " << x.RESERVED_16;
        s << ", " << x.RESERVED_17;
        s << ", " << x.RESERVED_18;
        s << ", " << x.RESERVED_19;
        s << ", " << x.RESERVED_20;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rng_2<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.RESERVED_13;
        s >> ',' >> x.RESERVED_14;
        s >> ',' >> x.RESERVED_15;
        s >> ',' >> x.RESERVED_16;
        s >> ',' >> x.RESERVED_17;
        s >> ',' >> x.RESERVED_18;
        s >> ',' >> x.RESERVED_19;
        s >> ',' >> x.RESERVED_20;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rng_3
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_3<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rng_3(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , RESERVED_13(begin)
        , RESERVED_14(begin)
        , RESERVED_15(begin)
        , RESERVED_16(begin)
        , RESERVED_17(begin)
        , RESERVED_18(begin)
        , RESERVED_19(begin)
        , RESERVED_20(begin)
        , RESERVED_21(begin)
        , RESERVED_22(begin)
        , RESERVED_23(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10001, id_sub = 3};
    #ifndef DOXYGEN
    enum { max_bit_width = 400};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_23.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;
    #else
    field<int16_t, sc_int16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_04;
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_10;
    #else
    field<int16_t, sc_int16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_12;
    #else
    field<uint16_t, sc_uint16, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_13;
    #else
    field<int16_t, sc_int16, 208, it> RESERVED_13;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_14;
    #else
    field<int16_t, sc_int16, 224, it> RESERVED_14;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_15;
    #else
    field<int16_t, sc_int16, 240, it> RESERVED_15;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_16;
    #else
    field<int16_t, sc_int16, 256, it> RESERVED_16;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_17;
    #else
    field<uint16_t, sc_uint16, 272, it> RESERVED_17;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_18;
    #else
    field<uint16_t, sc_uint16, 288, it> RESERVED_18;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_19;
    #else
    field<uint32_t, sc_uint32, 304, it> RESERVED_19;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_20;
    #else
    field<int16_t, sc_int16, 336, it> RESERVED_20;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_21;
    #else
    field<int16_t, sc_int16, 352, it> RESERVED_21;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_22;
    #else
    field<uint16_t, sc_uint16, 368, it> RESERVED_22;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_23;
    #else
    field<uint16_t, sc_uint16, 384, it> RESERVED_23;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rng_3& operator=(const hk_rng_3<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rng_3<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_3<rebind_it> type;
    };

    enum { id_main = 10001, id_sub = 3};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    int16_t          RESERVED_02;
    uint16_t         RESERVED_03;
    int16_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    int16_t          RESERVED_10;
    uint16_t         RESERVED_11;
    uint16_t         RESERVED_12;
    int16_t          RESERVED_13;
    int16_t          RESERVED_14;
    int16_t          RESERVED_15;
    int16_t          RESERVED_16;
    uint16_t         RESERVED_17;
    uint16_t         RESERVED_18;
    uint32_t         RESERVED_19;
    int16_t          RESERVED_20;
    int16_t          RESERVED_21;
    uint16_t         RESERVED_22;
    uint16_t         RESERVED_23;

    hk_rng_3() {}
    template<class it>
    hk_rng_3(const hk_rng_3<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rng_3<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.RESERVED_13;
        s << ", " << x.RESERVED_14;
        s << ", " << x.RESERVED_15;
        s << ", " << x.RESERVED_16;
        s << ", " << x.RESERVED_17;
        s << ", " << x.RESERVED_18;
        s << ", " << x.RESERVED_19;
        s << ", " << x.RESERVED_20;
        s << ", " << x.RESERVED_21;
        s << ", " << x.RESERVED_22;
        s << ", " << x.RESERVED_23;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rng_3<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.RESERVED_13;
        s >> ',' >> x.RESERVED_14;
        s >> ',' >> x.RESERVED_15;
        s >> ',' >> x.RESERVED_16;
        s >> ',' >> x.RESERVED_17;
        s >> ',' >> x.RESERVED_18;
        s >> ',' >> x.RESERVED_19;
        s >> ',' >> x.RESERVED_20;
        s >> ',' >> x.RESERVED_21;
        s >> ',' >> x.RESERVED_22;
        s >> ',' >> x.RESERVED_23;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rng_4
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_4<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rng_4(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , RESERVED_13(begin)
        , RESERVED_14(begin)
        , RESERVED_15(begin)
        , RESERVED_16(begin)
        , RESERVED_17(begin)
        , RESERVED_18(begin)
        , RESERVED_19(begin)
        , RESERVED_20(begin)
        , RESERVED_21(begin)
        , RESERVED_22(begin)
        , RESERVED_23(begin)
        , RESERVED_24(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10001, id_sub = 4};
    #ifndef DOXYGEN
    enum { max_bit_width = 432};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_24.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;
    #else
    field<int16_t, sc_int16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_04;
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_10;
    #else
    field<int16_t, sc_int16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_12;
    #else
    field<uint16_t, sc_uint16, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_13;
    #else
    field<int16_t, sc_int16, 208, it> RESERVED_13;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_14;
    #else
    field<int16_t, sc_int16, 224, it> RESERVED_14;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_15;
    #else
    field<int16_t, sc_int16, 240, it> RESERVED_15;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_16;
    #else
    field<int16_t, sc_int16, 256, it> RESERVED_16;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_17;
    #else
    field<uint16_t, sc_uint16, 272, it> RESERVED_17;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_18;
    #else
    field<uint16_t, sc_uint16, 288, it> RESERVED_18;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_19;
    #else
    field<uint32_t, sc_uint32, 304, it> RESERVED_19;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_20;
    #else
    field<int16_t, sc_int16, 336, it> RESERVED_20;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_21;
    #else
    field<int16_t, sc_int16, 352, it> RESERVED_21;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_22;
    #else
    field<uint16_t, sc_uint16, 368, it> RESERVED_22;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_23;
    #else
    field<uint16_t, sc_uint16, 384, it> RESERVED_23;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_24;
    #else
    field<uint32_t, sc_uint32, 400, it> RESERVED_24;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rng_4& operator=(const hk_rng_4<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rng_4<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_4<rebind_it> type;
    };

    enum { id_main = 10001, id_sub = 4};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    int16_t          RESERVED_02;
    uint16_t         RESERVED_03;
    int16_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    int16_t          RESERVED_10;
    uint16_t         RESERVED_11;
    uint16_t         RESERVED_12;
    int16_t          RESERVED_13;
    int16_t          RESERVED_14;
    int16_t          RESERVED_15;
    int16_t          RESERVED_16;
    uint16_t         RESERVED_17;
    uint16_t         RESERVED_18;
    uint32_t         RESERVED_19;
    int16_t          RESERVED_20;
    int16_t          RESERVED_21;
    uint16_t         RESERVED_22;
    uint16_t         RESERVED_23;
    uint32_t         RESERVED_24;

    hk_rng_4() {}
    template<class it>
    hk_rng_4(const hk_rng_4<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rng_4<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.RESERVED_13;
        s << ", " << x.RESERVED_14;
        s << ", " << x.RESERVED_15;
        s << ", " << x.RESERVED_16;
        s << ", " << x.RESERVED_17;
        s << ", " << x.RESERVED_18;
        s << ", " << x.RESERVED_19;
        s << ", " << x.RESERVED_20;
        s << ", " << x.RESERVED_21;
        s << ", " << x.RESERVED_22;
        s << ", " << x.RESERVED_23;
        s << ", " << x.RESERVED_24;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rng_4<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.RESERVED_13;
        s >> ',' >> x.RESERVED_14;
        s >> ',' >> x.RESERVED_15;
        s >> ',' >> x.RESERVED_16;
        s >> ',' >> x.RESERVED_17;
        s >> ',' >> x.RESERVED_18;
        s >> ',' >> x.RESERVED_19;
        s >> ',' >> x.RESERVED_20;
        s >> ',' >> x.RESERVED_21;
        s >> ',' >> x.RESERVED_22;
        s >> ',' >> x.RESERVED_23;
        s >> ',' >> x.RESERVED_24;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rng_5
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_5<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rng_5(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , RESERVED_13(begin)
        , RESERVED_14(begin)
        , RESERVED_15(begin)
        , RESERVED_16(begin)
        , RESERVED_17(begin)
        , RESERVED_18(begin)
        , RESERVED_19(begin)
        , RESERVED_20(begin)
        , RESERVED_21(begin)
        , RESERVED_22(begin)
        , RESERVED_23(begin)
        , RESERVED_24(begin)
        , RESERVED_25(begin)
        , RESERVED_26(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10001, id_sub = 5};
    #ifndef DOXYGEN
    enum { max_bit_width = 464};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_26.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;
    #else
    field<int16_t, sc_int16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_04;
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_10;
    #else
    field<int16_t, sc_int16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_12;
    #else
    field<uint16_t, sc_uint16, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_13;
    #else
    field<int16_t, sc_int16, 208, it> RESERVED_13;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_14;
    #else
    field<int16_t, sc_int16, 224, it> RESERVED_14;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_15;
    #else
    field<int16_t, sc_int16, 240, it> RESERVED_15;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_16;
    #else
    field<int16_t, sc_int16, 256, it> RESERVED_16;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_17;
    #else
    field<uint16_t, sc_uint16, 272, it> RESERVED_17;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_18;
    #else
    field<uint16_t, sc_uint16, 288, it> RESERVED_18;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_19;
    #else
    field<uint32_t, sc_uint32, 304, it> RESERVED_19;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_20;
    #else
    field<int16_t, sc_int16, 336, it> RESERVED_20;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_21;
    #else
    field<int16_t, sc_int16, 352, it> RESERVED_21;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_22;
    #else
    field<uint16_t, sc_uint16, 368, it> RESERVED_22;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_23;
    #else
    field<uint16_t, sc_uint16, 384, it> RESERVED_23;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_24;
    #else
    field<uint32_t, sc_uint32, 400, it> RESERVED_24;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_25;
    #else
    field<uint16_t, sc_uint16, 432, it> RESERVED_25;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_26;
    #else
    field<uint16_t, sc_uint16, 448, it> RESERVED_26;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rng_5& operator=(const hk_rng_5<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rng_5<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_5<rebind_it> type;
    };

    enum { id_main = 10001, id_sub = 5};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    int16_t          RESERVED_02;
    uint16_t         RESERVED_03;
    int16_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    int16_t          RESERVED_10;
    uint16_t         RESERVED_11;
    uint16_t         RESERVED_12;
    int16_t          RESERVED_13;
    int16_t          RESERVED_14;
    int16_t          RESERVED_15;
    int16_t          RESERVED_16;
    uint16_t         RESERVED_17;
    uint16_t         RESERVED_18;
    uint32_t         RESERVED_19;
    int16_t          RESERVED_20;
    int16_t          RESERVED_21;
    uint16_t         RESERVED_22;
    uint16_t         RESERVED_23;
    uint32_t         RESERVED_24;
    uint16_t         RESERVED_25;
    uint16_t         RESERVED_26;

    hk_rng_5() {}
    template<class it>
    hk_rng_5(const hk_rng_5<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rng_5<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.RESERVED_13;
        s << ", " << x.RESERVED_14;
        s << ", " << x.RESERVED_15;
        s << ", " << x.RESERVED_16;
        s << ", " << x.RESERVED_17;
        s << ", " << x.RESERVED_18;
        s << ", " << x.RESERVED_19;
        s << ", " << x.RESERVED_20;
        s << ", " << x.RESERVED_21;
        s << ", " << x.RESERVED_22;
        s << ", " << x.RESERVED_23;
        s << ", " << x.RESERVED_24;
        s << ", " << x.RESERVED_25;
        s << ", " << x.RESERVED_26;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rng_5<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.RESERVED_13;
        s >> ',' >> x.RESERVED_14;
        s >> ',' >> x.RESERVED_15;
        s >> ',' >> x.RESERVED_16;
        s >> ',' >> x.RESERVED_17;
        s >> ',' >> x.RESERVED_18;
        s >> ',' >> x.RESERVED_19;
        s >> ',' >> x.RESERVED_20;
        s >> ',' >> x.RESERVED_21;
        s >> ',' >> x.RESERVED_22;
        s >> ',' >> x.RESERVED_23;
        s >> ',' >> x.RESERVED_24;
        s >> ',' >> x.RESERVED_25;
        s >> ',' >> x.RESERVED_26;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rng_6
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_6<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rng_6(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , RESERVED_13(begin)
        , RESERVED_14(begin)
        , RESERVED_15(begin)
        , RESERVED_16(begin)
        , RESERVED_17(begin)
        , RESERVED_18(begin)
        , RESERVED_19(begin)
        , RESERVED_20(begin)
        , RESERVED_21(begin)
        , RESERVED_22(begin)
        , RESERVED_23(begin)
        , RESERVED_24(begin)
        , RESERVED_25(begin)
        , RESERVED_26(begin)
        , RESERVED_27(begin)
        , RESERVED_28(begin)
        , RESERVED_29(begin)
        , RESERVED_30(begin)
        , RESERVED_31(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10001, id_sub = 6};
    #ifndef DOXYGEN
    enum { max_bit_width = 544};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_31.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;
    #else
    field<int16_t, sc_int16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_04;
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_10;
    #else
    field<int16_t, sc_int16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_12;
    #else
    field<uint16_t, sc_uint16, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_13;
    #else
    field<int16_t, sc_int16, 208, it> RESERVED_13;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_14;
    #else
    field<int16_t, sc_int16, 224, it> RESERVED_14;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_15;
    #else
    field<int16_t, sc_int16, 240, it> RESERVED_15;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_16;
    #else
    field<int16_t, sc_int16, 256, it> RESERVED_16;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_17;
    #else
    field<uint16_t, sc_uint16, 272, it> RESERVED_17;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_18;
    #else
    field<uint16_t, sc_uint16, 288, it> RESERVED_18;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_19;
    #else
    field<uint32_t, sc_uint32, 304, it> RESERVED_19;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_20;
    #else
    field<int16_t, sc_int16, 336, it> RESERVED_20;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_21;
    #else
    field<int16_t, sc_int16, 352, it> RESERVED_21;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_22;
    #else
    field<uint16_t, sc_uint16, 368, it> RESERVED_22;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_23;
    #else
    field<uint16_t, sc_uint16, 384, it> RESERVED_23;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_24;
    #else
    field<uint32_t, sc_uint32, 400, it> RESERVED_24;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_25;
    #else
    field<uint16_t, sc_uint16, 432, it> RESERVED_25;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_26;
    #else
    field<uint16_t, sc_uint16, 448, it> RESERVED_26;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_27;
    #else
    field<int16_t, sc_int16, 464, it> RESERVED_27;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_28;
    #else
    field<uint16_t, sc_uint16, 480, it> RESERVED_28;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_29;
    #else
    field<int16_t, sc_int16, 496, it> RESERVED_29;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_30;
    #else
    field<int16_t, sc_int16, 512, it> RESERVED_30;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_31;
    #else
    field<uint16_t, sc_uint16, 528, it> RESERVED_31;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rng_6& operator=(const hk_rng_6<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        RESERVED_31 = o.RESERVED_31;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rng_6<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_6<rebind_it> type;
    };

    enum { id_main = 10001, id_sub = 6};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    int16_t          RESERVED_02;
    uint16_t         RESERVED_03;
    int16_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    int16_t          RESERVED_10;
    uint16_t         RESERVED_11;
    uint16_t         RESERVED_12;
    int16_t          RESERVED_13;
    int16_t          RESERVED_14;
    int16_t          RESERVED_15;
    int16_t          RESERVED_16;
    uint16_t         RESERVED_17;
    uint16_t         RESERVED_18;
    uint32_t         RESERVED_19;
    int16_t          RESERVED_20;
    int16_t          RESERVED_21;
    uint16_t         RESERVED_22;
    uint16_t         RESERVED_23;
    uint32_t         RESERVED_24;
    uint16_t         RESERVED_25;
    uint16_t         RESERVED_26;
    int16_t          RESERVED_27;
    uint16_t         RESERVED_28;
    int16_t          RESERVED_29;
    int16_t          RESERVED_30;
    uint16_t         RESERVED_31;

    hk_rng_6() {}
    template<class it>
    hk_rng_6(const hk_rng_6<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        RESERVED_31 = o.RESERVED_31;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rng_6<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.RESERVED_13;
        s << ", " << x.RESERVED_14;
        s << ", " << x.RESERVED_15;
        s << ", " << x.RESERVED_16;
        s << ", " << x.RESERVED_17;
        s << ", " << x.RESERVED_18;
        s << ", " << x.RESERVED_19;
        s << ", " << x.RESERVED_20;
        s << ", " << x.RESERVED_21;
        s << ", " << x.RESERVED_22;
        s << ", " << x.RESERVED_23;
        s << ", " << x.RESERVED_24;
        s << ", " << x.RESERVED_25;
        s << ", " << x.RESERVED_26;
        s << ", " << x.RESERVED_27;
        s << ", " << x.RESERVED_28;
        s << ", " << x.RESERVED_29;
        s << ", " << x.RESERVED_30;
        s << ", " << x.RESERVED_31;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rng_6<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.RESERVED_13;
        s >> ',' >> x.RESERVED_14;
        s >> ',' >> x.RESERVED_15;
        s >> ',' >> x.RESERVED_16;
        s >> ',' >> x.RESERVED_17;
        s >> ',' >> x.RESERVED_18;
        s >> ',' >> x.RESERVED_19;
        s >> ',' >> x.RESERVED_20;
        s >> ',' >> x.RESERVED_21;
        s >> ',' >> x.RESERVED_22;
        s >> ',' >> x.RESERVED_23;
        s >> ',' >> x.RESERVED_24;
        s >> ',' >> x.RESERVED_25;
        s >> ',' >> x.RESERVED_26;
        s >> ',' >> x.RESERVED_27;
        s >> ',' >> x.RESERVED_28;
        s >> ',' >> x.RESERVED_29;
        s >> ',' >> x.RESERVED_30;
        s >> ',' >> x.RESERVED_31;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rng_7
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_7<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rng_7(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , RESERVED_13(begin)
        , RESERVED_14(begin)
        , RESERVED_15(begin)
        , RESERVED_16(begin)
        , RESERVED_17(begin)
        , RESERVED_18(begin)
        , RESERVED_19(begin)
        , RESERVED_20(begin)
        , RESERVED_21(begin)
        , RESERVED_22(begin)
        , RESERVED_23(begin)
        , RESERVED_24(begin)
        , RESERVED_25(begin)
        , RESERVED_26(begin)
        , RESERVED_27(begin)
        , RESERVED_28(begin)
        , RESERVED_29(begin)
        , RESERVED_30(begin)
        , RESERVED_31(begin)
        , RESERVED_32(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10001, id_sub = 7};
    #ifndef DOXYGEN
    enum { max_bit_width = 560};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_32.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;
    #else
    field<int16_t, sc_int16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_04;
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_10;
    #else
    field<int16_t, sc_int16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_12;
    #else
    field<uint16_t, sc_uint16, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_13;
    #else
    field<int16_t, sc_int16, 208, it> RESERVED_13;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_14;
    #else
    field<int16_t, sc_int16, 224, it> RESERVED_14;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_15;
    #else
    field<int16_t, sc_int16, 240, it> RESERVED_15;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_16;
    #else
    field<int16_t, sc_int16, 256, it> RESERVED_16;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_17;
    #else
    field<uint16_t, sc_uint16, 272, it> RESERVED_17;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_18;
    #else
    field<uint16_t, sc_uint16, 288, it> RESERVED_18;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_19;
    #else
    field<uint32_t, sc_uint32, 304, it> RESERVED_19;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_20;
    #else
    field<int16_t, sc_int16, 336, it> RESERVED_20;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_21;
    #else
    field<int16_t, sc_int16, 352, it> RESERVED_21;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_22;
    #else
    field<uint16_t, sc_uint16, 368, it> RESERVED_22;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_23;
    #else
    field<uint16_t, sc_uint16, 384, it> RESERVED_23;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_24;
    #else
    field<uint32_t, sc_uint32, 400, it> RESERVED_24;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_25;
    #else
    field<uint16_t, sc_uint16, 432, it> RESERVED_25;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_26;
    #else
    field<uint16_t, sc_uint16, 448, it> RESERVED_26;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_27;
    #else
    field<int16_t, sc_int16, 464, it> RESERVED_27;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_28;
    #else
    field<uint16_t, sc_uint16, 480, it> RESERVED_28;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_29;
    #else
    field<int16_t, sc_int16, 496, it> RESERVED_29;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_30;
    #else
    field<int16_t, sc_int16, 512, it> RESERVED_30;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_31;
    #else
    field<uint16_t, sc_uint16, 528, it> RESERVED_31;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_32;
    #else
    field<int16_t, sc_int16, 544, it> RESERVED_32;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rng_7& operator=(const hk_rng_7<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        RESERVED_31 = o.RESERVED_31;
        RESERVED_32 = o.RESERVED_32;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rng_7<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_7<rebind_it> type;
    };

    enum { id_main = 10001, id_sub = 7};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    int16_t          RESERVED_02;
    uint16_t         RESERVED_03;
    int16_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    int16_t          RESERVED_10;
    uint16_t         RESERVED_11;
    uint16_t         RESERVED_12;
    int16_t          RESERVED_13;
    int16_t          RESERVED_14;
    int16_t          RESERVED_15;
    int16_t          RESERVED_16;
    uint16_t         RESERVED_17;
    uint16_t         RESERVED_18;
    uint32_t         RESERVED_19;
    int16_t          RESERVED_20;
    int16_t          RESERVED_21;
    uint16_t         RESERVED_22;
    uint16_t         RESERVED_23;
    uint32_t         RESERVED_24;
    uint16_t         RESERVED_25;
    uint16_t         RESERVED_26;
    int16_t          RESERVED_27;
    uint16_t         RESERVED_28;
    int16_t          RESERVED_29;
    int16_t          RESERVED_30;
    uint16_t         RESERVED_31;
    int16_t          RESERVED_32;

    hk_rng_7() {}
    template<class it>
    hk_rng_7(const hk_rng_7<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        RESERVED_31 = o.RESERVED_31;
        RESERVED_32 = o.RESERVED_32;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rng_7<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.RESERVED_13;
        s << ", " << x.RESERVED_14;
        s << ", " << x.RESERVED_15;
        s << ", " << x.RESERVED_16;
        s << ", " << x.RESERVED_17;
        s << ", " << x.RESERVED_18;
        s << ", " << x.RESERVED_19;
        s << ", " << x.RESERVED_20;
        s << ", " << x.RESERVED_21;
        s << ", " << x.RESERVED_22;
        s << ", " << x.RESERVED_23;
        s << ", " << x.RESERVED_24;
        s << ", " << x.RESERVED_25;
        s << ", " << x.RESERVED_26;
        s << ", " << x.RESERVED_27;
        s << ", " << x.RESERVED_28;
        s << ", " << x.RESERVED_29;
        s << ", " << x.RESERVED_30;
        s << ", " << x.RESERVED_31;
        s << ", " << x.RESERVED_32;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rng_7<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.RESERVED_13;
        s >> ',' >> x.RESERVED_14;
        s >> ',' >> x.RESERVED_15;
        s >> ',' >> x.RESERVED_16;
        s >> ',' >> x.RESERVED_17;
        s >> ',' >> x.RESERVED_18;
        s >> ',' >> x.RESERVED_19;
        s >> ',' >> x.RESERVED_20;
        s >> ',' >> x.RESERVED_21;
        s >> ',' >> x.RESERVED_22;
        s >> ',' >> x.RESERVED_23;
        s >> ',' >> x.RESERVED_24;
        s >> ',' >> x.RESERVED_25;
        s >> ',' >> x.RESERVED_26;
        s >> ',' >> x.RESERVED_27;
        s >> ',' >> x.RESERVED_28;
        s >> ',' >> x.RESERVED_29;
        s >> ',' >> x.RESERVED_30;
        s >> ',' >> x.RESERVED_31;
        s >> ',' >> x.RESERVED_32;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rng_8
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_8<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rng_8(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
        , RESERVED_12(begin)
        , RESERVED_13(begin)
        , RESERVED_14(begin)
        , RESERVED_15(begin)
        , RESERVED_16(begin)
        , RESERVED_17(begin)
        , RESERVED_18(begin)
        , RESERVED_19(begin)
        , RESERVED_20(begin)
        , RESERVED_21(begin)
        , RESERVED_22(begin)
        , RESERVED_23(begin)
        , RESERVED_24(begin)
        , RESERVED_25(begin)
        , RESERVED_26(begin)
        , RESERVED_27(begin)
        , RESERVED_28(begin)
        , RESERVED_29(begin)
        , RESERVED_30(begin)
        , RESERVED_31(begin)
        , RESERVED_32(begin)
        , RESERVED_33(begin)
        , RESERVED_34(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10001, id_sub = 8};
    #ifndef DOXYGEN
    enum { max_bit_width = 592};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_34.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_02;
    #else
    field<int16_t, sc_int16, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_03;
    #else
    field<uint16_t, sc_uint16, 48, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_04;
    #else
    field<int16_t, sc_int16, 64, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 80, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 96, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_07;
    #else
    field<uint16_t, sc_uint16, 112, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_08;
    #else
    field<uint16_t, sc_uint16, 128, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_09;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_10;
    #else
    field<int16_t, sc_int16, 160, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_11;
    #else
    field<uint16_t, sc_uint16, 176, it> RESERVED_11;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_12;
    #else
    field<uint16_t, sc_uint16, 192, it> RESERVED_12;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_13;
    #else
    field<int16_t, sc_int16, 208, it> RESERVED_13;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_14;
    #else
    field<int16_t, sc_int16, 224, it> RESERVED_14;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_15;
    #else
    field<int16_t, sc_int16, 240, it> RESERVED_15;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_16;
    #else
    field<int16_t, sc_int16, 256, it> RESERVED_16;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_17;
    #else
    field<uint16_t, sc_uint16, 272, it> RESERVED_17;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_18;
    #else
    field<uint16_t, sc_uint16, 288, it> RESERVED_18;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_19;
    #else
    field<uint32_t, sc_uint32, 304, it> RESERVED_19;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_20;
    #else
    field<int16_t, sc_int16, 336, it> RESERVED_20;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_21;
    #else
    field<int16_t, sc_int16, 352, it> RESERVED_21;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_22;
    #else
    field<uint16_t, sc_uint16, 368, it> RESERVED_22;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_23;
    #else
    field<uint16_t, sc_uint16, 384, it> RESERVED_23;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_24;
    #else
    field<uint32_t, sc_uint32, 400, it> RESERVED_24;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_25;
    #else
    field<uint16_t, sc_uint16, 432, it> RESERVED_25;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_26;
    #else
    field<uint16_t, sc_uint16, 448, it> RESERVED_26;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_27;
    #else
    field<int16_t, sc_int16, 464, it> RESERVED_27;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_28;
    #else
    field<uint16_t, sc_uint16, 480, it> RESERVED_28;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_29;
    #else
    field<int16_t, sc_int16, 496, it> RESERVED_29;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_30;
    #else
    field<int16_t, sc_int16, 512, it> RESERVED_30;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_31;
    #else
    field<uint16_t, sc_uint16, 528, it> RESERVED_31;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_32;
    #else
    field<int16_t, sc_int16, 544, it> RESERVED_32;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_33;
    #else
    field<uint16_t, sc_uint16, 560, it> RESERVED_33;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_34;
    #else
    field<uint16_t, sc_uint16, 576, it> RESERVED_34;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rng_8& operator=(const hk_rng_8<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        RESERVED_31 = o.RESERVED_31;
        RESERVED_32 = o.RESERVED_32;
        RESERVED_33 = o.RESERVED_33;
        RESERVED_34 = o.RESERVED_34;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rng_8<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rng_8<rebind_it> type;
    };

    enum { id_main = 10001, id_sub = 8};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    int16_t          RESERVED_02;
    uint16_t         RESERVED_03;
    int16_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;
    uint16_t         RESERVED_07;
    uint16_t         RESERVED_08;
    int16_t          RESERVED_09;
    int16_t          RESERVED_10;
    uint16_t         RESERVED_11;
    uint16_t         RESERVED_12;
    int16_t          RESERVED_13;
    int16_t          RESERVED_14;
    int16_t          RESERVED_15;
    int16_t          RESERVED_16;
    uint16_t         RESERVED_17;
    uint16_t         RESERVED_18;
    uint32_t         RESERVED_19;
    int16_t          RESERVED_20;
    int16_t          RESERVED_21;
    uint16_t         RESERVED_22;
    uint16_t         RESERVED_23;
    uint32_t         RESERVED_24;
    uint16_t         RESERVED_25;
    uint16_t         RESERVED_26;
    int16_t          RESERVED_27;
    uint16_t         RESERVED_28;
    int16_t          RESERVED_29;
    int16_t          RESERVED_30;
    uint16_t         RESERVED_31;
    int16_t          RESERVED_32;
    uint16_t         RESERVED_33;
    uint16_t         RESERVED_34;

    hk_rng_8() {}
    template<class it>
    hk_rng_8(const hk_rng_8<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        RESERVED_12 = o.RESERVED_12;
        RESERVED_13 = o.RESERVED_13;
        RESERVED_14 = o.RESERVED_14;
        RESERVED_15 = o.RESERVED_15;
        RESERVED_16 = o.RESERVED_16;
        RESERVED_17 = o.RESERVED_17;
        RESERVED_18 = o.RESERVED_18;
        RESERVED_19 = o.RESERVED_19;
        RESERVED_20 = o.RESERVED_20;
        RESERVED_21 = o.RESERVED_21;
        RESERVED_22 = o.RESERVED_22;
        RESERVED_23 = o.RESERVED_23;
        RESERVED_24 = o.RESERVED_24;
        RESERVED_25 = o.RESERVED_25;
        RESERVED_26 = o.RESERVED_26;
        RESERVED_27 = o.RESERVED_27;
        RESERVED_28 = o.RESERVED_28;
        RESERVED_29 = o.RESERVED_29;
        RESERVED_30 = o.RESERVED_30;
        RESERVED_31 = o.RESERVED_31;
        RESERVED_32 = o.RESERVED_32;
        RESERVED_33 = o.RESERVED_33;
        RESERVED_34 = o.RESERVED_34;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rng_8<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
        s << ", " << x.RESERVED_12;
        s << ", " << x.RESERVED_13;
        s << ", " << x.RESERVED_14;
        s << ", " << x.RESERVED_15;
        s << ", " << x.RESERVED_16;
        s << ", " << x.RESERVED_17;
        s << ", " << x.RESERVED_18;
        s << ", " << x.RESERVED_19;
        s << ", " << x.RESERVED_20;
        s << ", " << x.RESERVED_21;
        s << ", " << x.RESERVED_22;
        s << ", " << x.RESERVED_23;
        s << ", " << x.RESERVED_24;
        s << ", " << x.RESERVED_25;
        s << ", " << x.RESERVED_26;
        s << ", " << x.RESERVED_27;
        s << ", " << x.RESERVED_28;
        s << ", " << x.RESERVED_29;
        s << ", " << x.RESERVED_30;
        s << ", " << x.RESERVED_31;
        s << ", " << x.RESERVED_32;
        s << ", " << x.RESERVED_33;
        s << ", " << x.RESERVED_34;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rng_8<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
        s >> ',' >> x.RESERVED_12;
        s >> ',' >> x.RESERVED_13;
        s >> ',' >> x.RESERVED_14;
        s >> ',' >> x.RESERVED_15;
        s >> ',' >> x.RESERVED_16;
        s >> ',' >> x.RESERVED_17;
        s >> ',' >> x.RESERVED_18;
        s >> ',' >> x.RESERVED_19;
        s >> ',' >> x.RESERVED_20;
        s >> ',' >> x.RESERVED_21;
        s >> ',' >> x.RESERVED_22;
        s >> ',' >> x.RESERVED_23;
        s >> ',' >> x.RESERVED_24;
        s >> ',' >> x.RESERVED_25;
        s >> ',' >> x.RESERVED_26;
        s >> ',' >> x.RESERVED_27;
        s >> ',' >> x.RESERVED_28;
        s >> ',' >> x.RESERVED_29;
        s >> ',' >> x.RESERVED_30;
        s >> ',' >> x.RESERVED_31;
        s >> ',' >> x.RESERVED_32;
        s >> ',' >> x.RESERVED_33;
        s >> ',' >> x.RESERVED_34;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! built in real time clock of scanning device, local time

//! <para>This package belongs to the predefined selectors:</para>
//! <para>status, legacy</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rtc
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rtc<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rtc(it begin, it end, bool dirty=false)
        : year(begin)
        , month(begin)
        , day(begin)
        , hour(begin)
        , minute(begin)
        , second(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10000, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 48};
    it begin() const { return year.begin(); }
    it end() const { return second.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t year;//!<  years since 1900 
    #else
    field<uint8_t, sc_uint8, 0, it> year;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t month;//!<  months since January 
    #else
    field<uint8_t, sc_uint8, 8, it> month;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t day;//!<  day of the month 
    #else
    field<uint8_t, sc_uint8, 16, it> day;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t hour;//!<  hours after midnight 
    #else
    field<uint8_t, sc_uint8, 24, it> hour;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t minute;//!<  minutes after the hour 
    #else
    field<uint8_t, sc_uint8, 32, it> minute;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t second;//!<  seconds after the minute 
    #else
    field<uint8_t, sc_uint8, 40, it> second;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rtc& operator=(const hk_rtc<ito>& o) {
        year = o.year;
        month = o.month;
        day = o.day;
        hour = o.hour;
        minute = o.minute;
        second = o.second;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct hk_rtc<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rtc<rebind_it> type;
    };

    enum { id_main = 10000, id_sub = 0};

    uint8_t          year;
    uint8_t          month;
    uint8_t          day;
    uint8_t          hour;
    uint8_t          minute;
    uint8_t          second;

    hk_rtc() {}
    template<class it>
    hk_rtc(const hk_rtc<it>& o) {
        year = o.year;
        month = o.month;
        day = o.day;
        hour = o.hour;
        minute = o.minute;
        second = o.second;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rtc<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.year;
        s << ", " << x.month;
        s << ", " << x.day;
        s << ", " << x.hour;
        s << ", " << x.minute;
        s << ", " << x.second;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rtc<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.year;
        s >> ',' >> x.month;
        s >> ',' >> x.day;
        s >> ',' >> x.hour;
        s >> ',' >> x.minute;
        s >> ',' >> x.second;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_rtc_sys
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rtc_sys<rebind_it> type;
    };

    typedef it iterator_type;

    hk_rtc_sys(it begin, it end, bool dirty=false)
        : year(begin)
        , month(begin)
        , day(begin)
        , hour(begin)
        , minute(begin)
        , second(begin)
        , systime(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10000, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 80};
    it begin() const { return year.begin(); }
    it end() const { return systime.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t year;
    #else
    field<uint8_t, sc_uint8, 0, it> year;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t month;
    #else
    field<uint8_t, sc_uint8, 8, it> month;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t day;
    #else
    field<uint8_t, sc_uint8, 16, it> day;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t hour;
    #else
    field<uint8_t, sc_uint8, 24, it> hour;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t minute;
    #else
    field<uint8_t, sc_uint8, 32, it> minute;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t second;
    #else
    field<uint8_t, sc_uint8, 40, it> second;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;
    #else
    field<uint32_t, sc_uint32, 48, it> systime;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_rtc_sys& operator=(const hk_rtc_sys<ito>& o) {
        year = o.year;
        month = o.month;
        day = o.day;
        hour = o.hour;
        minute = o.minute;
        second = o.second;
        systime = o.systime;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_rtc_sys<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_rtc_sys<rebind_it> type;
    };

    enum { id_main = 10000, id_sub = 1};

    uint8_t          year;
    uint8_t          month;
    uint8_t          day;
    uint8_t          hour;
    uint8_t          minute;
    uint8_t          second;
    uint32_t         systime;

    hk_rtc_sys() {}
    template<class it>
    hk_rtc_sys(const hk_rtc_sys<it>& o) {
        year = o.year;
        month = o.month;
        day = o.day;
        hour = o.hour;
        minute = o.minute;
        second = o.second;
        systime = o.systime;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_rtc_sys<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.year;
        s << ", " << x.month;
        s << ", " << x.day;
        s << ", " << x.hour;
        s << ", " << x.minute;
        s << ", " << x.second;
        s << ", " << x.systime;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_rtc_sys<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.year;
        s >> ',' >> x.month;
        s >> ',' >> x.day;
        s >> ',' >> x.hour;
        s >> ',' >> x.minute;
        s >> ',' >> x.second;
        s >> ',' >> x.systime;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_scn
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_scn<rebind_it> type;
    };

    typedef it iterator_type;

    hk_scn(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10002, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 160};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_06.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_02;
    #else
    field<uint32_t, sc_uint32, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t RESERVED_03;
    #else
    field<int32_t, sc_int32, 64, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t RESERVED_04;
    #else
    field<int32_t, sc_int32, 96, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 128, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_06;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_scn& operator=(const hk_scn<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_scn<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_scn<rebind_it> type;
    };

    enum { id_main = 10002, id_sub = 0};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    uint32_t         RESERVED_02;
    int32_t          RESERVED_03;
    int32_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;

    hk_scn() {}
    template<class it>
    hk_scn(const hk_scn<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_scn<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_scn<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_scn_1
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_scn_1<rebind_it> type;
    };

    typedef it iterator_type;

    hk_scn_1(it begin, it end, bool dirty=false)
        : RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
        , RESERVED_03(begin)
        , RESERVED_04(begin)
        , RESERVED_05(begin)
        , RESERVED_06(begin)
        , RESERVED_07(begin)
        , RESERVED_08(begin)
        , RESERVED_09(begin)
        , RESERVED_10(begin)
        , RESERVED_11(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10002, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 320};
    it begin() const { return RESERVED_00.begin(); }
    it end() const { return RESERVED_11.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_00;
    #else
    field<uint16_t, sc_uint16, 0, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t RESERVED_01;
    #else
    field<uint16_t, sc_uint16, 16, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_02;
    #else
    field<uint32_t, sc_uint32, 32, it> RESERVED_02;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t RESERVED_03;
    #else
    field<int32_t, sc_int32, 64, it> RESERVED_03;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t RESERVED_04;
    #else
    field<int32_t, sc_int32, 96, it> RESERVED_04;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_05;
    #else
    field<int16_t, sc_int16, 128, it> RESERVED_05;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t RESERVED_06;
    #else
    field<int16_t, sc_int16, 144, it> RESERVED_06;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t RESERVED_07;
    #else
    field<int32_t, sc_int32, 160, it> RESERVED_07;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_08;
    #else
    field<uint32_t, sc_uint32, 192, it> RESERVED_08;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_09;
    #else
    field<uint32_t, sc_uint32, 224, it> RESERVED_09;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_10;
    #else
    field<uint32_t, sc_uint32, 256, it> RESERVED_10;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_11;
    #else
    field<uint32_t, sc_uint32, 288, it> RESERVED_11;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_scn_1& operator=(const hk_scn_1<ito>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_scn_1<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_scn_1<rebind_it> type;
    };

    enum { id_main = 10002, id_sub = 1};

    uint16_t         RESERVED_00;
    uint16_t         RESERVED_01;
    uint32_t         RESERVED_02;
    int32_t          RESERVED_03;
    int32_t          RESERVED_04;
    int16_t          RESERVED_05;
    int16_t          RESERVED_06;
    int32_t          RESERVED_07;
    uint32_t         RESERVED_08;
    uint32_t         RESERVED_09;
    uint32_t         RESERVED_10;
    uint32_t         RESERVED_11;

    hk_scn_1() {}
    template<class it>
    hk_scn_1(const hk_scn_1<it>& o) {
        RESERVED_00 = o.RESERVED_00;
        RESERVED_01 = o.RESERVED_01;
        RESERVED_02 = o.RESERVED_02;
        RESERVED_03 = o.RESERVED_03;
        RESERVED_04 = o.RESERVED_04;
        RESERVED_05 = o.RESERVED_05;
        RESERVED_06 = o.RESERVED_06;
        RESERVED_07 = o.RESERVED_07;
        RESERVED_08 = o.RESERVED_08;
        RESERVED_09 = o.RESERVED_09;
        RESERVED_10 = o.RESERVED_10;
        RESERVED_11 = o.RESERVED_11;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_scn_1<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.RESERVED_00;
        s << ", " << x.RESERVED_01;
        s << ", " << x.RESERVED_02;
        s << ", " << x.RESERVED_03;
        s << ", " << x.RESERVED_04;
        s << ", " << x.RESERVED_05;
        s << ", " << x.RESERVED_06;
        s << ", " << x.RESERVED_07;
        s << ", " << x.RESERVED_08;
        s << ", " << x.RESERVED_09;
        s << ", " << x.RESERVED_10;
        s << ", " << x.RESERVED_11;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_scn_1<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.RESERVED_00;
        s >> ',' >> x.RESERVED_01;
        s >> ',' >> x.RESERVED_02;
        s >> ',' >> x.RESERVED_03;
        s >> ',' >> x.RESERVED_04;
        s >> ',' >> x.RESERVED_05;
        s >> ',' >> x.RESERVED_06;
        s >> ',' >> x.RESERVED_07;
        s >> ',' >> x.RESERVED_08;
        s >> ',' >> x.RESERVED_09;
        s >> ',' >> x.RESERVED_10;
        s >> ',' >> x.RESERVED_11;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct hk_time
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef hk_time<rebind_it> type;
    };

    typedef it iterator_type;

    hk_time(it begin, it end, bool dirty=false)
        : systime(begin)
        , year(begin)
        , month(begin)
        , day(begin)
        , hour(begin)
        , minute(begin)
        , second(begin)
        , utc_offset(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 40, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 96};
    it begin() const { return systime.begin(); }
    it end() const { return utc_offset.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  systime of message in units of unit units.time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t year;//!<  year 
    #else
    field<uint16_t, sc_uint16, 32, it> year;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t month;//!<  month  1 .. 12 
    #else
    field<uint8_t, sc_uint8, 48, it> month;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t day;//!<  day    1 .. 31 
    #else
    field<uint8_t, sc_uint8, 56, it> day;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t hour;//!<  hour   0 .. 23 
    #else
    field<uint8_t, sc_uint8, 64, it> hour;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t minute;//!<  minute 0 .. 59 
    #else
    field<uint8_t, sc_uint8, 72, it> minute;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t second;//!<  second 0 .. 59 (60 if leap second) 
    #else
    field<uint8_t, sc_uint8, 80, it> second;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t utc_offset;//!<  diff. between localtime and UTC 
    #else
    field<int8_t, sc_int8, 88, it> utc_offset;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    hk_time& operator=(const hk_time<ito>& o) {
        systime = o.systime;
        year = o.year;
        month = o.month;
        day = o.day;
        hour = o.hour;
        minute = o.minute;
        second = o.second;
        utc_offset = o.utc_offset;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct hk_time<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef hk_time<rebind_it> type;
    };

    enum { id_main = 40, id_sub = 0};

    uint32_t         systime;
    uint16_t         year;
    uint8_t          month;
    uint8_t          day;
    uint8_t          hour;
    uint8_t          minute;
    uint8_t          second;
    int8_t           utc_offset;

    hk_time() {}
    template<class it>
    hk_time(const hk_time<it>& o) {
        systime = o.systime;
        year = o.year;
        month = o.month;
        day = o.day;
        hour = o.hour;
        minute = o.minute;
        second = o.second;
        utc_offset = o.utc_offset;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const hk_time<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.year;
        s << ", " << x.month;
        s << ", " << x.day;
        s << ", " << x.hour;
        s << ", " << x.minute;
        s << ", " << x.second;
        s << ", " << x.utc_offset;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, hk_time<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.year;
        s >> ',' >> x.month;
        s >> ',' >> x.day;
        s >> ',' >> x.hour;
        s >> ',' >> x.minute;
        s >> ',' >> x.second;
        s >> ',' >> x.utc_offset;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct ht_dbg_data
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef ht_dbg_data<rebind_it> type;
    };

    typedef it iterator_type;

    ht_dbg_data(it begin, it end, bool dirty=false)
        : systime(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 53, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 65568};
    it begin() const { return systime.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 1024 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : i_ber_led(begin, begin_bit)
            , u_apd(begin, begin_bit)
            , w_ber_led(begin, begin_bit)
            , I_APD_DC(begin, begin_bit)
            , minq_max(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint16_t i_ber_led;
        #else
        field<uint16_t, sc_uint16, 0, it> i_ber_led;
        #endif
        #ifdef DOXYGEN
        uint16_t u_apd;
        #else
        field<uint16_t, sc_uint16, 16, it> u_apd;
        #endif
        #ifdef DOXYGEN
        uint16_t w_ber_led;
        #else
        field<uint16_t, sc_uint16, 32, it> w_ber_led;
        #endif
        #ifdef DOXYGEN
        uint16_t I_APD_DC;
        #else
        field<uint16_t, sc_uint15, 48, it> I_APD_DC;
        #endif
        #ifdef DOXYGEN
        uint8_t minq_max;
        #else
        field<uint8_t, sc_bit, 63, it> minq_max;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[1024];
    #else
    sequence<ht_dbg_data, 64, 32, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    ht_dbg_data& operator=(const ht_dbg_data<ito>& o) {
        systime = o.systime;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].i_ber_led = o.data[n].i_ber_led;
            data[n].u_apd = o.data[n].u_apd;
            data[n].w_ber_led = o.data[n].w_ber_led;
            data[n].I_APD_DC = o.data[n].I_APD_DC;
            data[n].minq_max = o.data[n].minq_max;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct ht_dbg_data<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef ht_dbg_data<rebind_it> type;
    };

    enum { id_main = 53, id_sub = 0};

    uint32_t         systime;
    std::size_t data_size;
    enum { data_max_size = 1024 };
    struct sequence_definition {
        uint16_t         i_ber_led;
        uint16_t         u_apd;
        uint16_t         w_ber_led;
        uint16_t         I_APD_DC;
        uint8_t          minq_max;
    } data[1024];

    ht_dbg_data() {}
    template<class it>
    ht_dbg_data(const ht_dbg_data<it>& o) {
        systime = o.systime;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].i_ber_led = o.data[n].i_ber_led;
            data[n].u_apd = o.data[n].u_apd;
            data[n].w_ber_led = o.data[n].w_ber_led;
            data[n].I_APD_DC = o.data[n].I_APD_DC;
            data[n].minq_max = o.data[n].minq_max;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const ht_dbg_data<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].i_ber_led
            << ", " << x.data[n].u_apd
            << ", " << x.data[n].w_ber_led
            << ", " << x.data[n].I_APD_DC
            << ", " << x.data[n].minq_max
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, ht_dbg_data<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].i_ber_led)
                >> ',' >> (x.data[x.data_size-1].u_apd)
                >> ',' >> (x.data[x.data_size-1].w_ber_led)
                >> ',' >> (x.data[x.data_size-1].I_APD_DC)
                >> ',' >> (x.data[x.data_size-1].minq_max)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct inclination
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef inclination<rebind_it> type;
    };

    typedef it iterator_type;

    inclination(it begin, it end, bool dirty=false)
        : systime(begin)
        , frame_angle(begin)
        , adc1(begin)
        , adc2(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 26, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 96};
    it begin() const { return systime.begin(); }
    it end() const { return adc2.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  internal time of external PPS pulse in units of uints.time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_angle;//!<  frame angle (same units as laser_shot_*angles) 
    #else
    field<uint32_t, sc_uint32, 32, it> frame_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t adc1;//!<  output of adc1, belongs to sensitve axis 1 
    #else
    field<uint16_t, sc_uint16, 64, it> adc1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t adc2;//!<  output of adc2, belongs to sensitive axis 2 
    #else
    field<uint16_t, sc_uint16, 80, it> adc2;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    inclination& operator=(const inclination<ito>& o) {
        systime = o.systime;
        frame_angle = o.frame_angle;
        adc1 = o.adc1;
        adc2 = o.adc2;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct inclination<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef inclination<rebind_it> type;
    };

    enum { id_main = 26, id_sub = 0};

    uint32_t         systime;
    uint32_t         frame_angle;
    uint16_t         adc1;
    uint16_t         adc2;

    inclination() {}
    template<class it>
    inclination(const inclination<it>& o) {
        systime = o.systime;
        frame_angle = o.frame_angle;
        adc1 = o.adc1;
        adc2 = o.adc2;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const inclination<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.frame_angle;
        s << ", " << x.adc1;
        s << ", " << x.adc2;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, inclination<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.frame_angle;
        s >> ',' >> x.adc1;
        s >> ',' >> x.adc2;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct inclination_4axes
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef inclination_4axes<rebind_it> type;
    };

    typedef it iterator_type;

    inclination_4axes(it begin, it end, bool dirty=false)
        : systime(begin)
        , frame_angle(begin)
        , adc1(begin)
        , adc2(begin)
        , adc3(begin)
        , adc4(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 26, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 128};
    it begin() const { return systime.begin(); }
    it end() const { return adc4.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  internal time of external PPS pulse in units of uints.time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_angle;//!<  frame angle (same units as laser_shot_*angles) 
    #else
    field<uint32_t, sc_uint32, 32, it> frame_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t adc1;//!<  output of adc1, belongs to sensitive axis 1 
    #else
    field<uint16_t, sc_uint16, 64, it> adc1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t adc2;//!<  output of adc2, belongs to sensitive axis 2 
    #else
    field<uint16_t, sc_uint16, 80, it> adc2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t adc3;//!<  output of adc2, belongs to sensitive axis 3 
    #else
    field<uint16_t, sc_uint16, 96, it> adc3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t adc4;//!<  output of adc2, belongs to sensitive axis 4 
    #else
    field<uint16_t, sc_uint16, 112, it> adc4;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    inclination_4axes& operator=(const inclination_4axes<ito>& o) {
        systime = o.systime;
        frame_angle = o.frame_angle;
        adc1 = o.adc1;
        adc2 = o.adc2;
        adc3 = o.adc3;
        adc4 = o.adc4;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct inclination_4axes<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef inclination_4axes<rebind_it> type;
    };

    enum { id_main = 26, id_sub = 1};

    uint32_t         systime;
    uint32_t         frame_angle;
    uint16_t         adc1;
    uint16_t         adc2;
    uint16_t         adc3;
    uint16_t         adc4;

    inclination_4axes() {}
    template<class it>
    inclination_4axes(const inclination_4axes<it>& o) {
        systime = o.systime;
        frame_angle = o.frame_angle;
        adc1 = o.adc1;
        adc2 = o.adc2;
        adc3 = o.adc3;
        adc4 = o.adc4;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const inclination_4axes<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.frame_angle;
        s << ", " << x.adc1;
        s << ", " << x.adc2;
        s << ", " << x.adc3;
        s << ", " << x.adc4;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, inclination_4axes<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.frame_angle;
        s >> ',' >> x.adc1;
        s >> ',' >> x.adc2;
        s >> ',' >> x.adc3;
        s >> ',' >> x.adc4;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct inclination_device
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef inclination_device<rebind_it> type;
    };

    typedef it iterator_type;

    inclination_device(it begin, it end, bool dirty=false)
        : s1(begin)
        , s2(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 6, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 192};
    it begin() const { return s1.begin(); }
    it end() const { return s2.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float s1[3];//!<  sensitive axis sensor 1 
    #else
    array<float, 3, sc_float32, 0, it> s1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float s2[3];//!<  sensitive axis sensor 2 
    #else
    array<float, 3, sc_float32, 96, it> s2;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    inclination_device& operator=(const inclination_device<ito>& o) {
        for(unsigned n=0; n<3; ++n) s1[n] = o.s1[n];
        for(unsigned n=0; n<3; ++n) s2[n] = o.s2[n];
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct inclination_device<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef inclination_device<rebind_it> type;
    };

    enum { id_main = 6, id_sub = 0};

    float            s1[3];
    float            s2[3];

    inclination_device() {}
    template<class it>
    inclination_device(const inclination_device<it>& o) {
        for(unsigned n=0; n<3; ++n) s1[n] = o.s1[n];
        for(unsigned n=0; n<3; ++n) s2[n] = o.s2[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const inclination_device<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", "; write_array(s, 3, x.s1);
        s << ", "; write_array(s, 3, x.s2);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, inclination_device<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ','; read_array(s, 3, x.s1);
        s >> ','; read_array(s, 3, x.s2);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct inclination_device_4axes
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef inclination_device_4axes<rebind_it> type;
    };

    typedef it iterator_type;

    inclination_device_4axes(it begin, it end, bool dirty=false)
        : s1(begin)
        , s2(begin)
        , s3(begin)
        , s4(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 6, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 384};
    it begin() const { return s1.begin(); }
    it end() const { return s4.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float s1[3];//!<  sensitive axis sensor 1 
    #else
    array<float, 3, sc_float32, 0, it> s1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float s2[3];//!<  sensitive axis sensor 2 
    #else
    array<float, 3, sc_float32, 96, it> s2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float s3[3];//!<  sensitive axis sensor 3 
    #else
    array<float, 3, sc_float32, 192, it> s3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float s4[3];//!<  sensitive axis sensor 4 
    #else
    array<float, 3, sc_float32, 288, it> s4;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    inclination_device_4axes& operator=(const inclination_device_4axes<ito>& o) {
        for(unsigned n=0; n<3; ++n) s1[n] = o.s1[n];
        for(unsigned n=0; n<3; ++n) s2[n] = o.s2[n];
        for(unsigned n=0; n<3; ++n) s3[n] = o.s3[n];
        for(unsigned n=0; n<3; ++n) s4[n] = o.s4[n];
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct inclination_device_4axes<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef inclination_device_4axes<rebind_it> type;
    };

    enum { id_main = 6, id_sub = 1};

    float            s1[3];
    float            s2[3];
    float            s3[3];
    float            s4[3];

    inclination_device_4axes() {}
    template<class it>
    inclination_device_4axes(const inclination_device_4axes<it>& o) {
        for(unsigned n=0; n<3; ++n) s1[n] = o.s1[n];
        for(unsigned n=0; n<3; ++n) s2[n] = o.s2[n];
        for(unsigned n=0; n<3; ++n) s3[n] = o.s3[n];
        for(unsigned n=0; n<3; ++n) s4[n] = o.s4[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const inclination_device_4axes<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", "; write_array(s, 3, x.s1);
        s << ", "; write_array(s, 3, x.s2);
        s << ", "; write_array(s, 3, x.s3);
        s << ", "; write_array(s, 3, x.s4);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, inclination_device_4axes<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ','; read_array(s, 3, x.s1);
        s >> ','; read_array(s, 3, x.s2);
        s >> ','; read_array(s, 3, x.s3);
        s >> ','; read_array(s, 3, x.s4);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct inclination_device_4axes_offset
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef inclination_device_4axes_offset<rebind_it> type;
    };

    typedef it iterator_type;

    inclination_device_4axes_offset(it begin, it end, bool dirty=false)
        : s1(begin)
        , s2(begin)
        , s3(begin)
        , s4(begin)
        , o1(begin)
        , o2(begin)
        , o3(begin)
        , o4(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 6, id_sub = 2};
    #ifndef DOXYGEN
    enum { max_bit_width = 512};
    it begin() const { return s1.begin(); }
    it end() const { return o4.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float s1[3];//!<  sensitive axis sensor 1 
    #else
    array<float, 3, sc_float32, 0, it> s1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float s2[3];//!<  sensitive axis sensor 2 
    #else
    array<float, 3, sc_float32, 96, it> s2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float s3[3];//!<  sensitive axis sensor 3 
    #else
    array<float, 3, sc_float32, 192, it> s3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float s4[3];//!<  sensitive axis sensor 4 
    #else
    array<float, 3, sc_float32, 288, it> s4;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float o1;//!<  voltage offset sensor 1 
    #else
    field<float, sc_float32, 384, it> o1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float o2;//!<  voltage offset sensor 2 
    #else
    field<float, sc_float32, 416, it> o2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float o3;//!<  voltage offset sensor 3 
    #else
    field<float, sc_float32, 448, it> o3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float o4;//!<  voltage offset sensor 4 
    #else
    field<float, sc_float32, 480, it> o4;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    inclination_device_4axes_offset& operator=(const inclination_device_4axes_offset<ito>& o) {
        for(unsigned n=0; n<3; ++n) s1[n] = o.s1[n];
        for(unsigned n=0; n<3; ++n) s2[n] = o.s2[n];
        for(unsigned n=0; n<3; ++n) s3[n] = o.s3[n];
        for(unsigned n=0; n<3; ++n) s4[n] = o.s4[n];
        o1 = o.o1;
        o2 = o.o2;
        o3 = o.o3;
        o4 = o.o4;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct inclination_device_4axes_offset<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef inclination_device_4axes_offset<rebind_it> type;
    };

    enum { id_main = 6, id_sub = 2};

    float            s1[3];
    float            s2[3];
    float            s3[3];
    float            s4[3];
    float            o1;
    float            o2;
    float            o3;
    float            o4;

    inclination_device_4axes_offset() {}
    template<class it>
    inclination_device_4axes_offset(const inclination_device_4axes_offset<it>& o) {
        for(unsigned n=0; n<3; ++n) s1[n] = o.s1[n];
        for(unsigned n=0; n<3; ++n) s2[n] = o.s2[n];
        for(unsigned n=0; n<3; ++n) s3[n] = o.s3[n];
        for(unsigned n=0; n<3; ++n) s4[n] = o.s4[n];
        o1 = o.o1;
        o2 = o.o2;
        o3 = o.o3;
        o4 = o.o4;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const inclination_device_4axes_offset<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", "; write_array(s, 3, x.s1);
        s << ", "; write_array(s, 3, x.s2);
        s << ", "; write_array(s, 3, x.s3);
        s << ", "; write_array(s, 3, x.s4);
        s << ", " << x.o1;
        s << ", " << x.o2;
        s << ", " << x.o3;
        s << ", " << x.o4;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, inclination_device_4axes_offset<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ','; read_array(s, 3, x.s1);
        s >> ','; read_array(s, 3, x.s2);
        s >> ','; read_array(s, 3, x.s3);
        s >> ','; read_array(s, 3, x.s4);
        s >> ',' >> x.o1;
        s >> ',' >> x.o2;
        s >> ',' >> x.o3;
        s >> ',' >> x.o4;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct laser_echo
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef laser_echo<rebind_it> type;
    };

    typedef it iterator_type;

    laser_echo(it begin, it end, bool dirty=false)
        : range(begin)
        , amplitude(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 11, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 48};
    it begin() const { return range.begin(); }
    it end() const { return amplitude.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t range;
    #else
    field<int32_t, sc_int32, 0, it> range;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t amplitude;
    #else
    field<uint16_t, sc_uint16, 32, it> amplitude;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    laser_echo& operator=(const laser_echo<ito>& o) {
        range = o.range;
        amplitude = o.amplitude;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct laser_echo<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef laser_echo<rebind_it> type;
    };

    enum { id_main = 11, id_sub = 0};

    int32_t          range;
    uint16_t         amplitude;

    laser_echo() {}
    template<class it>
    laser_echo(const laser_echo<it>& o) {
        range = o.range;
        amplitude = o.amplitude;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const laser_echo<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.range;
        s << ", " << x.amplitude;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, laser_echo<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.range;
        s >> ',' >> x.amplitude;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct laser_echo_qual
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef laser_echo_qual<rebind_it> type;
    };

    typedef it iterator_type;

    laser_echo_qual(it begin, it end, bool dirty=false)
        : range(begin)
        , amplitude(begin)
        , flags(begin)
        , quality(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 11, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 64};
    it begin() const { return range.begin(); }
    it end() const { return quality.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t range;
    #else
    field<int32_t, sc_int32, 0, it> range;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t amplitude;
    #else
    field<uint16_t, sc_uint16, 32, it> amplitude;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t flags;
    #else
    field<uint8_t, sc_uint8, 48, it> flags;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t quality;
    #else
    field<uint8_t, sc_uint8, 56, it> quality;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    laser_echo_qual& operator=(const laser_echo_qual<ito>& o) {
        range = o.range;
        amplitude = o.amplitude;
        flags = o.flags;
        quality = o.quality;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct laser_echo_qual<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef laser_echo_qual<rebind_it> type;
    };

    enum { id_main = 11, id_sub = 1};

    int32_t          range;
    uint16_t         amplitude;
    uint8_t          flags;
    uint8_t          quality;

    laser_echo_qual() {}
    template<class it>
    laser_echo_qual(const laser_echo_qual<it>& o) {
        range = o.range;
        amplitude = o.amplitude;
        flags = o.flags;
        quality = o.quality;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const laser_echo_qual<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.range;
        s << ", " << x.amplitude;
        s << ", " << x.flags;
        s << ", " << x.quality;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, laser_echo_qual<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.range;
        s >> ',' >> x.amplitude;
        s >> ',' >> x.flags;
        s >> ',' >> x.quality;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct laser_echo_sw
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef laser_echo_sw<rebind_it> type;
    };

    typedef it iterator_type;

    laser_echo_sw(it begin, it end, bool dirty=false)
        : range(begin)
        , amplitude(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 28, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 48};
    it begin() const { return range.begin(); }
    it end() const { return amplitude.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t range;
    #else
    field<int32_t, sc_int32, 0, it> range;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t amplitude;
    #else
    field<uint16_t, sc_uint16, 32, it> amplitude;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    laser_echo_sw& operator=(const laser_echo_sw<ito>& o) {
        range = o.range;
        amplitude = o.amplitude;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct laser_echo_sw<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef laser_echo_sw<rebind_it> type;
    };

    enum { id_main = 28, id_sub = 0};

    int32_t          range;
    uint16_t         amplitude;

    laser_echo_sw() {}
    template<class it>
    laser_echo_sw(const laser_echo_sw<it>& o) {
        range = o.range;
        amplitude = o.amplitude;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const laser_echo_sw<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.range;
        s << ", " << x.amplitude;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, laser_echo_sw<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.range;
        s >> ',' >> x.amplitude;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct laser_shot
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot<rebind_it> type;
    };

    typedef it iterator_type;

    laser_shot(it begin, it end, bool dirty=false)
        : systime(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 32};
    it begin() const { return systime.begin(); }
    it end() const { return systime.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  time of laser shot in units of units.time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    laser_shot& operator=(const laser_shot<ito>& o) {
        systime = o.systime;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct laser_shot<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot<rebind_it> type;
    };

    enum { id_main = 10, id_sub = 0};

    uint32_t         systime;

    laser_shot() {}
    template<class it>
    laser_shot(const laser_shot<it>& o) {
        systime = o.systime;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const laser_shot<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, laser_shot<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct laser_shot_1angle
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot_1angle<rebind_it> type;
    };

    typedef it iterator_type;

    laser_shot_1angle(it begin, it end, bool dirty=false)
        : systime(begin)
        , line_angle(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 64};
    it begin() const { return systime.begin(); }
    it end() const { return line_angle.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  time of laser shot in units of time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle;//!<  line angle 
    #else
    field<uint32_t, sc_uint32, 32, it> line_angle;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    laser_shot_1angle& operator=(const laser_shot_1angle<ito>& o) {
        systime = o.systime;
        line_angle = o.line_angle;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct laser_shot_1angle<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot_1angle<rebind_it> type;
    };

    enum { id_main = 10, id_sub = 1};

    uint32_t         systime;
    uint32_t         line_angle;

    laser_shot_1angle() {}
    template<class it>
    laser_shot_1angle(const laser_shot_1angle<it>& o) {
        systime = o.systime;
        line_angle = o.line_angle;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const laser_shot_1angle<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.line_angle;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, laser_shot_1angle<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.line_angle;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct laser_shot_2angles
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot_2angles<rebind_it> type;
    };

    typedef it iterator_type;

    laser_shot_2angles(it begin, it end, bool dirty=false)
        : systime(begin)
        , line_angle(begin)
        , frame_angle(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 10, id_sub = 2};
    #ifndef DOXYGEN
    enum { max_bit_width = 96};
    it begin() const { return systime.begin(); }
    it end() const { return frame_angle.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  time of laser shot in units of time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle;//!<  line angle 
    #else
    field<uint32_t, sc_uint32, 32, it> line_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_angle;//!<  frame angle 
    #else
    field<uint32_t, sc_uint32, 64, it> frame_angle;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    laser_shot_2angles& operator=(const laser_shot_2angles<ito>& o) {
        systime = o.systime;
        line_angle = o.line_angle;
        frame_angle = o.frame_angle;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct laser_shot_2angles<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot_2angles<rebind_it> type;
    };

    enum { id_main = 10, id_sub = 2};

    uint32_t         systime;
    uint32_t         line_angle;
    uint32_t         frame_angle;

    laser_shot_2angles() {}
    template<class it>
    laser_shot_2angles(const laser_shot_2angles<it>& o) {
        systime = o.systime;
        line_angle = o.line_angle;
        frame_angle = o.frame_angle;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const laser_shot_2angles<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.line_angle;
        s << ", " << x.frame_angle;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, laser_shot_2angles<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.line_angle;
        s >> ',' >> x.frame_angle;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct laser_shot_2angles_rad
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot_2angles_rad<rebind_it> type;
    };

    typedef it iterator_type;

    laser_shot_2angles_rad(it begin, it end, bool dirty=false)
        : systime(begin)
        , line_angle(begin)
        , frame_angle(begin)
        , backgnd_rad(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 20, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 112};
    it begin() const { return systime.begin(); }
    it end() const { return backgnd_rad.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  time of laser shot in units of time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle;//!<  line angle 
    #else
    field<uint32_t, sc_uint32, 32, it> line_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_angle;//!<  frame angle 
    #else
    field<uint32_t, sc_uint32, 64, it> frame_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t backgnd_rad;//!<  background radiation 
    #else
    field<uint16_t, sc_uint16, 96, it> backgnd_rad;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    laser_shot_2angles_rad& operator=(const laser_shot_2angles_rad<ito>& o) {
        systime = o.systime;
        line_angle = o.line_angle;
        frame_angle = o.frame_angle;
        backgnd_rad = o.backgnd_rad;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct laser_shot_2angles_rad<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot_2angles_rad<rebind_it> type;
    };

    enum { id_main = 20, id_sub = 0};

    uint32_t         systime;
    uint32_t         line_angle;
    uint32_t         frame_angle;
    uint16_t         backgnd_rad;

    laser_shot_2angles_rad() {}
    template<class it>
    laser_shot_2angles_rad(const laser_shot_2angles_rad<it>& o) {
        systime = o.systime;
        line_angle = o.line_angle;
        frame_angle = o.frame_angle;
        backgnd_rad = o.backgnd_rad;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const laser_shot_2angles_rad<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.line_angle;
        s << ", " << x.frame_angle;
        s << ", " << x.backgnd_rad;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, laser_shot_2angles_rad<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.line_angle;
        s >> ',' >> x.frame_angle;
        s >> ',' >> x.backgnd_rad;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct laser_shot_3angles
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot_3angles<rebind_it> type;
    };

    typedef it iterator_type;

    laser_shot_3angles(it begin, it end, bool dirty=false)
        : systime(begin)
        , line_angle(begin)
        , line_angle_r0(begin)
        , line_angle_r1(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 50010, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 128};
    it begin() const { return systime.begin(); }
    it end() const { return line_angle_r1.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle;
    #else
    field<uint32_t, sc_uint32, 32, it> line_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle_r0;
    #else
    field<uint32_t, sc_uint32, 64, it> line_angle_r0;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle_r1;
    #else
    field<uint32_t, sc_uint32, 96, it> line_angle_r1;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    laser_shot_3angles& operator=(const laser_shot_3angles<ito>& o) {
        systime = o.systime;
        line_angle = o.line_angle;
        line_angle_r0 = o.line_angle_r0;
        line_angle_r1 = o.line_angle_r1;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct laser_shot_3angles<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot_3angles<rebind_it> type;
    };

    enum { id_main = 50010, id_sub = 0};

    uint32_t         systime;
    uint32_t         line_angle;
    uint32_t         line_angle_r0;
    uint32_t         line_angle_r1;

    laser_shot_3angles() {}
    template<class it>
    laser_shot_3angles(const laser_shot_3angles<it>& o) {
        systime = o.systime;
        line_angle = o.line_angle;
        line_angle_r0 = o.line_angle_r0;
        line_angle_r1 = o.line_angle_r1;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const laser_shot_3angles<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.line_angle;
        s << ", " << x.line_angle_r0;
        s << ", " << x.line_angle_r1;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, laser_shot_3angles<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.line_angle;
        s >> ',' >> x.line_angle_r0;
        s >> ',' >> x.line_angle_r1;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct laser_shot_6angles
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot_6angles<rebind_it> type;
    };

    typedef it iterator_type;

    laser_shot_6angles(it begin, it end, bool dirty=false)
        : systime(begin)
        , line_angle(begin)
        , frame_angle(begin)
        , line_angle_r0(begin)
        , line_angle_r1(begin)
        , frame_angle_r0(begin)
        , frame_angle_r1(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 50011, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 224};
    it begin() const { return systime.begin(); }
    it end() const { return frame_angle_r1.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle;
    #else
    field<uint32_t, sc_uint32, 32, it> line_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_angle;
    #else
    field<uint32_t, sc_uint32, 64, it> frame_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle_r0;
    #else
    field<uint32_t, sc_uint32, 96, it> line_angle_r0;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle_r1;
    #else
    field<uint32_t, sc_uint32, 128, it> line_angle_r1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_angle_r0;
    #else
    field<uint32_t, sc_uint32, 160, it> frame_angle_r0;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_angle_r1;
    #else
    field<uint32_t, sc_uint32, 192, it> frame_angle_r1;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    laser_shot_6angles& operator=(const laser_shot_6angles<ito>& o) {
        systime = o.systime;
        line_angle = o.line_angle;
        frame_angle = o.frame_angle;
        line_angle_r0 = o.line_angle_r0;
        line_angle_r1 = o.line_angle_r1;
        frame_angle_r0 = o.frame_angle_r0;
        frame_angle_r1 = o.frame_angle_r1;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct laser_shot_6angles<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef laser_shot_6angles<rebind_it> type;
    };

    enum { id_main = 50011, id_sub = 0};

    uint32_t         systime;
    uint32_t         line_angle;
    uint32_t         frame_angle;
    uint32_t         line_angle_r0;
    uint32_t         line_angle_r1;
    uint32_t         frame_angle_r0;
    uint32_t         frame_angle_r1;

    laser_shot_6angles() {}
    template<class it>
    laser_shot_6angles(const laser_shot_6angles<it>& o) {
        systime = o.systime;
        line_angle = o.line_angle;
        frame_angle = o.frame_angle;
        line_angle_r0 = o.line_angle_r0;
        line_angle_r1 = o.line_angle_r1;
        frame_angle_r0 = o.frame_angle_r0;
        frame_angle_r1 = o.frame_angle_r1;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const laser_shot_6angles<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.line_angle;
        s << ", " << x.frame_angle;
        s << ", " << x.line_angle_r0;
        s << ", " << x.line_angle_r1;
        s << ", " << x.frame_angle_r0;
        s << ", " << x.frame_angle_r1;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, laser_shot_6angles<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.line_angle;
        s >> ',' >> x.frame_angle;
        s >> ',' >> x.line_angle_r0;
        s >> ',' >> x.line_angle_r1;
        s >> ',' >> x.frame_angle_r0;
        s >> ',' >> x.frame_angle_r1;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct line_start
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef line_start<rebind_it> type;
    };

    typedef it iterator_type;

    line_start(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 16, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    line_start& operator=(const line_start<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct line_start<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef line_start<rebind_it> type;
    };

    enum { id_main = 16, id_sub = 0};


    line_start() {}
    template<class it>
    line_start(const line_start<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const line_start<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, line_start<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! start of a line scan in down direction

//! before data packets belonging to this line
//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct line_start_dn
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef line_start_dn<rebind_it> type;
    };

    typedef it iterator_type;

    line_start_dn(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 25, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    line_start_dn& operator=(const line_start_dn<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct line_start_dn<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef line_start_dn<rebind_it> type;
    };

    enum { id_main = 25, id_sub = 0};


    line_start_dn() {}
    template<class it>
    line_start_dn(const line_start_dn<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const line_start_dn<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, line_start_dn<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! start of a line scan in up direction

//! before data packets belonging to this line
//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct line_start_up
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef line_start_up<rebind_it> type;
    };

    typedef it iterator_type;

    line_start_up(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 24, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    line_start_up& operator=(const line_start_up<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct line_start_up<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef line_start_up<rebind_it> type;
    };

    enum { id_main = 24, id_sub = 0};


    line_start_up() {}
    template<class it>
    line_start_up(const line_start_up<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const line_start_up<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, line_start_up<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! end of line scan

//! sent, when a scan with changing frame angle escapes the
//! specified frame range
//! after data packets belonging to this line
//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct line_stop
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef line_stop<rebind_it> type;
    };

    typedef it iterator_type;

    line_stop(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 17, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    line_stop& operator=(const line_stop<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct line_stop<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef line_stop<rebind_it> type;
    };

    enum { id_main = 17, id_sub = 0};


    line_stop() {}
    template<class it>
    line_stop(const line_stop<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const line_stop<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, line_stop<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct magnetic_field
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef magnetic_field<rebind_it> type;
    };

    typedef it iterator_type;

    magnetic_field(it begin, it end, bool dirty=false)
        : systime(begin)
        , frame_angle(begin)
        , adc_x(begin)
        , adc_y(begin)
        , adc_z(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 44, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 112};
    it begin() const { return systime.begin(); }
    it end() const { return adc_z.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_angle;
    #else
    field<uint32_t, sc_uint32, 32, it> frame_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t adc_x;
    #else
    field<uint16_t, sc_uint16, 64, it> adc_x;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t adc_y;
    #else
    field<uint16_t, sc_uint16, 80, it> adc_y;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t adc_z;
    #else
    field<uint16_t, sc_uint16, 96, it> adc_z;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    magnetic_field& operator=(const magnetic_field<ito>& o) {
        systime = o.systime;
        frame_angle = o.frame_angle;
        adc_x = o.adc_x;
        adc_y = o.adc_y;
        adc_z = o.adc_z;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct magnetic_field<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef magnetic_field<rebind_it> type;
    };

    enum { id_main = 44, id_sub = 0};

    uint32_t         systime;
    uint32_t         frame_angle;
    uint16_t         adc_x;
    uint16_t         adc_y;
    uint16_t         adc_z;

    magnetic_field() {}
    template<class it>
    magnetic_field(const magnetic_field<it>& o) {
        systime = o.systime;
        frame_angle = o.frame_angle;
        adc_x = o.adc_x;
        adc_y = o.adc_y;
        adc_z = o.adc_z;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const magnetic_field<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.frame_angle;
        s << ", " << x.adc_x;
        s << ", " << x.adc_y;
        s << ", " << x.adc_z;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, magnetic_field<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.frame_angle;
        s >> ',' >> x.adc_x;
        s >> ',' >> x.adc_y;
        s >> ',' >> x.adc_z;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! measurement has started

//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct meas_start
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef meas_start<rebind_it> type;
    };

    typedef it iterator_type;

    meas_start(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 31, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    meas_start& operator=(const meas_start<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct meas_start<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef meas_start<rebind_it> type;
    };

    enum { id_main = 31, id_sub = 0};


    meas_start() {}
    template<class it>
    meas_start(const meas_start<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const meas_start<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, meas_start<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! measurement has stopped

//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct meas_stop
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef meas_stop<rebind_it> type;
    };

    typedef it iterator_type;

    meas_stop(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 32, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    meas_stop& operator=(const meas_stop<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct meas_stop<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef meas_stop<rebind_it> type;
    };

    enum { id_main = 32, id_sub = 0};


    meas_stop() {}
    template<class it>
    meas_stop(const meas_stop<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const meas_stop<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, meas_stop<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! current setting of subdivider for monitoring data stream

//! For explanation please read the user manual of scanner device
//! <para>This package belongs to the predefined selectors:</para>
//! <para>attribute, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct monitoring_info
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef monitoring_info<rebind_it> type;
    };

    typedef it iterator_type;

    monitoring_info(it begin, it end, bool dirty=false)
        : msm_line(begin)
        , msm_frame(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 39, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 32};
    it begin() const { return msm_line.begin(); }
    it end() const { return msm_frame.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t msm_line;//!<  subdivider for measurements within scan line 
    #else
    field<uint16_t, sc_uint16, 0, it> msm_line;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t msm_frame;//!<  subdivider for scan lines within frames 
    #else
    field<uint16_t, sc_uint16, 16, it> msm_frame;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    monitoring_info& operator=(const monitoring_info<ito>& o) {
        msm_line = o.msm_line;
        msm_frame = o.msm_frame;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct monitoring_info<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef monitoring_info<rebind_it> type;
    };

    enum { id_main = 39, id_sub = 0};

    uint16_t         msm_line;
    uint16_t         msm_frame;

    monitoring_info() {}
    template<class it>
    monitoring_info(const monitoring_info<it>& o) {
        msm_line = o.msm_line;
        msm_frame = o.msm_frame;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const monitoring_info<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.msm_line;
        s << ", " << x.msm_frame;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, monitoring_info<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.msm_line;
        s >> ',' >> x.msm_frame;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct packed_frame_echo
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef packed_frame_echo<rebind_it> type;
    };

    typedef it iterator_type;

    packed_frame_echo(it begin, it end, bool dirty=false)
        : deltas(begin, end, deltas_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 48, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 1048512};
    it begin() const { return deltas.begin(); }
    it end() const { return deltas.end(); }

    #endif //DOXYGEN

    std::size_t deltas_size;
    enum { deltas_max_size = 16383 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : range(begin, begin_bit)
            , ampl(begin, begin_bit)
            , refl(begin, begin_bit)
            , flags(begin, begin_bit)
            , dev(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        int32_t range;
        #else
        field<int32_t, sc_int23, 0, it> range;
        #endif
        #ifdef DOXYGEN
        int16_t ampl;
        #else
        field<int16_t, sc_int14, 23, it> ampl;
        #endif
        #ifdef DOXYGEN
        int16_t refl;
        #else
        field<int16_t, sc_int14, 37, it> refl;
        #endif
        #ifdef DOXYGEN
        uint8_t flags;
        #else
        field<uint8_t, sc_uint4, 51, it> flags;
        #endif
        #ifdef DOXYGEN
        int16_t dev;
        #else
        field<int16_t, sc_int9, 55, it> dev;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition deltas[16383];
    #else
    sequence<packed_frame_echo, 64, 0, it> deltas;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    packed_frame_echo& operator=(const packed_frame_echo<ito>& o) {
        deltas_size = o.deltas_size;
        for(unsigned n=0; n<deltas_size; ++n){
            deltas[n].range = o.deltas[n].range;
            deltas[n].ampl = o.deltas[n].ampl;
            deltas[n].refl = o.deltas[n].refl;
            deltas[n].flags = o.deltas[n].flags;
            deltas[n].dev = o.deltas[n].dev;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct packed_frame_echo<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef packed_frame_echo<rebind_it> type;
    };

    enum { id_main = 48, id_sub = 0};

    std::size_t deltas_size;
    enum { deltas_max_size = 16383 };
    struct sequence_definition {
        int32_t          range;
        int16_t          ampl;
        int16_t          refl;
        uint8_t          flags;
        int16_t          dev;
    } deltas[16383];

    packed_frame_echo() {}
    template<class it>
    packed_frame_echo(const packed_frame_echo<it>& o) {
        deltas_size = o.deltas.size();
        for(unsigned n=0; n<deltas_size; ++n){
            deltas[n].range = o.deltas[n].range;
            deltas[n].ampl = o.deltas[n].ampl;
            deltas[n].refl = o.deltas[n].refl;
            deltas[n].flags = o.deltas[n].flags;
            deltas[n].dev = o.deltas[n].dev;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const packed_frame_echo<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {

        s << ", [";
        for (std::size_t n=0; n<x.deltas_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.deltas[n].range
            << ", " << x.deltas[n].ampl
            << ", " << x.deltas[n].refl
            << ", " << x.deltas[n].flags
            << ", " << x.deltas[n].dev
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, packed_frame_echo<it>& x) {
    package_istream_entry ok(s);
    if (ok) {

        s >> ',' >> '[' >> std::ws;
        x.deltas_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.deltas_size;
            if (s
                >> '{' >> (x.deltas[x.deltas_size-1].range)
                >> ',' >> (x.deltas[x.deltas_size-1].ampl)
                >> ',' >> (x.deltas[x.deltas_size-1].refl)
                >> ',' >> (x.deltas[x.deltas_size-1].flags)
                >> ',' >> (x.deltas[x.deltas_size-1].dev)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct packed_frame_laser_shot_2angles
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef packed_frame_laser_shot_2angles<rebind_it> type;
    };

    typedef it iterator_type;

    packed_frame_laser_shot_2angles(it begin, it end, bool dirty=false)
        : systime_diff(begin)
        , num_echoes(begin)
        , line_angle_diff(begin)
        , frame_angle_diff(begin)
        , deltas(begin, end, deltas_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 46, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 262176};
    it begin() const { return systime_diff.begin(); }
    it end() const { return deltas.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t systime_diff;
    #else
    field<uint8_t, sc_uint8, 0, it> systime_diff;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t num_echoes;
    #else
    field<uint8_t, sc_uint8, 8, it> num_echoes;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t line_angle_diff;
    #else
    field<int16_t, sc_int16, 16, it> line_angle_diff;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t frame_angle_diff;
    #else
    field<int16_t, sc_int16, 32, it> frame_angle_diff;
    #endif //DOXYGEN

    std::size_t deltas_size;
    enum { deltas_max_size = 16383 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : systime(begin, begin_bit)
            , line_angle(begin, begin_bit)
            , frame_angle(begin, begin_bit)
            , num_echoes(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        int8_t systime;
        #else
        field<int8_t, sc_int2, 0, it> systime;
        #endif
        #ifdef DOXYGEN
        int8_t line_angle;
        #else
        field<int8_t, sc_int5, 2, it> line_angle;
        #endif
        #ifdef DOXYGEN
        int8_t frame_angle;
        #else
        field<int8_t, sc_int5, 7, it> frame_angle;
        #endif
        #ifdef DOXYGEN
        uint8_t num_echoes;
        #else
        field<uint8_t, sc_uint4, 12, it> num_echoes;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition deltas[16383];
    #else
    sequence<packed_frame_laser_shot_2angles, 16, 48, it> deltas;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    packed_frame_laser_shot_2angles& operator=(const packed_frame_laser_shot_2angles<ito>& o) {
        systime_diff = o.systime_diff;
        num_echoes = o.num_echoes;
        line_angle_diff = o.line_angle_diff;
        frame_angle_diff = o.frame_angle_diff;
        deltas_size = o.deltas_size;
        for(unsigned n=0; n<deltas_size; ++n){
            deltas[n].systime = o.deltas[n].systime;
            deltas[n].line_angle = o.deltas[n].line_angle;
            deltas[n].frame_angle = o.deltas[n].frame_angle;
            deltas[n].num_echoes = o.deltas[n].num_echoes;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct packed_frame_laser_shot_2angles<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef packed_frame_laser_shot_2angles<rebind_it> type;
    };

    enum { id_main = 46, id_sub = 0};

    uint8_t          systime_diff;
    uint8_t          num_echoes;
    int16_t          line_angle_diff;
    int16_t          frame_angle_diff;
    std::size_t deltas_size;
    enum { deltas_max_size = 16383 };
    struct sequence_definition {
        int8_t           systime;
        int8_t           line_angle;
        int8_t           frame_angle;
        uint8_t          num_echoes;
    } deltas[16383];

    packed_frame_laser_shot_2angles() {}
    template<class it>
    packed_frame_laser_shot_2angles(const packed_frame_laser_shot_2angles<it>& o) {
        systime_diff = o.systime_diff;
        num_echoes = o.num_echoes;
        line_angle_diff = o.line_angle_diff;
        frame_angle_diff = o.frame_angle_diff;
        deltas_size = o.deltas.size();
        for(unsigned n=0; n<deltas_size; ++n){
            deltas[n].systime = o.deltas[n].systime;
            deltas[n].line_angle = o.deltas[n].line_angle;
            deltas[n].frame_angle = o.deltas[n].frame_angle;
            deltas[n].num_echoes = o.deltas[n].num_echoes;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const packed_frame_laser_shot_2angles<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime_diff;
        s << ", " << x.num_echoes;
        s << ", " << x.line_angle_diff;
        s << ", " << x.frame_angle_diff;

        s << ", [";
        for (std::size_t n=0; n<x.deltas_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.deltas[n].systime
            << ", " << x.deltas[n].line_angle
            << ", " << x.deltas[n].frame_angle
            << ", " << x.deltas[n].num_echoes
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, packed_frame_laser_shot_2angles<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime_diff;
        s >> ',' >> x.num_echoes;
        s >> ',' >> x.line_angle_diff;
        s >> ',' >> x.frame_angle_diff;

        s >> ',' >> '[' >> std::ws;
        x.deltas_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.deltas_size;
            if (s
                >> '{' >> (x.deltas[x.deltas_size-1].systime)
                >> ',' >> (x.deltas[x.deltas_size-1].line_angle)
                >> ',' >> (x.deltas[x.deltas_size-1].frame_angle)
                >> ',' >> (x.deltas[x.deltas_size-1].num_echoes)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct packed_frame_laser_shot_2angles_rad
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef packed_frame_laser_shot_2angles_rad<rebind_it> type;
    };

    typedef it iterator_type;

    packed_frame_laser_shot_2angles_rad(it begin, it end, bool dirty=false)
        : systime_diff(begin)
        , num_echoes(begin)
        , line_angle_diff(begin)
        , frame_angle_diff(begin)
        , backgnd_rad(begin)
        , deltas(begin, end, deltas_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 52, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 524320};
    it begin() const { return systime_diff.begin(); }
    it end() const { return deltas.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t systime_diff;
    #else
    field<uint8_t, sc_uint8, 0, it> systime_diff;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t num_echoes;
    #else
    field<uint8_t, sc_uint8, 8, it> num_echoes;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t line_angle_diff;
    #else
    field<int16_t, sc_int16, 16, it> line_angle_diff;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t frame_angle_diff;
    #else
    field<int16_t, sc_int16, 32, it> frame_angle_diff;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t backgnd_rad;
    #else
    field<uint16_t, sc_uint16, 48, it> backgnd_rad;
    #endif //DOXYGEN

    std::size_t deltas_size;
    enum { deltas_max_size = 16383 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : systime(begin, begin_bit)
            , line_angle(begin, begin_bit)
            , frame_angle(begin, begin_bit)
            , num_echoes(begin, begin_bit)
            , backgnd_rad(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        int8_t systime;
        #else
        field<int8_t, sc_int2, 0, it> systime;
        #endif
        #ifdef DOXYGEN
        int8_t line_angle;
        #else
        field<int8_t, sc_int5, 2, it> line_angle;
        #endif
        #ifdef DOXYGEN
        int8_t frame_angle;
        #else
        field<int8_t, sc_int5, 7, it> frame_angle;
        #endif
        #ifdef DOXYGEN
        uint8_t num_echoes;
        #else
        field<uint8_t, sc_uint4, 12, it> num_echoes;
        #endif
        #ifdef DOXYGEN
        uint16_t backgnd_rad;
        #else
        field<uint16_t, sc_uint16, 16, it> backgnd_rad;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition deltas[16383];
    #else
    sequence<packed_frame_laser_shot_2angles_rad, 32, 64, it> deltas;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    packed_frame_laser_shot_2angles_rad& operator=(const packed_frame_laser_shot_2angles_rad<ito>& o) {
        systime_diff = o.systime_diff;
        num_echoes = o.num_echoes;
        line_angle_diff = o.line_angle_diff;
        frame_angle_diff = o.frame_angle_diff;
        backgnd_rad = o.backgnd_rad;
        deltas_size = o.deltas_size;
        for(unsigned n=0; n<deltas_size; ++n){
            deltas[n].systime = o.deltas[n].systime;
            deltas[n].line_angle = o.deltas[n].line_angle;
            deltas[n].frame_angle = o.deltas[n].frame_angle;
            deltas[n].num_echoes = o.deltas[n].num_echoes;
            deltas[n].backgnd_rad = o.deltas[n].backgnd_rad;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct packed_frame_laser_shot_2angles_rad<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef packed_frame_laser_shot_2angles_rad<rebind_it> type;
    };

    enum { id_main = 52, id_sub = 0};

    uint8_t          systime_diff;
    uint8_t          num_echoes;
    int16_t          line_angle_diff;
    int16_t          frame_angle_diff;
    uint16_t         backgnd_rad;
    std::size_t deltas_size;
    enum { deltas_max_size = 16383 };
    struct sequence_definition {
        int8_t           systime;
        int8_t           line_angle;
        int8_t           frame_angle;
        uint8_t          num_echoes;
        uint16_t         backgnd_rad;
    } deltas[16383];

    packed_frame_laser_shot_2angles_rad() {}
    template<class it>
    packed_frame_laser_shot_2angles_rad(const packed_frame_laser_shot_2angles_rad<it>& o) {
        systime_diff = o.systime_diff;
        num_echoes = o.num_echoes;
        line_angle_diff = o.line_angle_diff;
        frame_angle_diff = o.frame_angle_diff;
        backgnd_rad = o.backgnd_rad;
        deltas_size = o.deltas.size();
        for(unsigned n=0; n<deltas_size; ++n){
            deltas[n].systime = o.deltas[n].systime;
            deltas[n].line_angle = o.deltas[n].line_angle;
            deltas[n].frame_angle = o.deltas[n].frame_angle;
            deltas[n].num_echoes = o.deltas[n].num_echoes;
            deltas[n].backgnd_rad = o.deltas[n].backgnd_rad;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const packed_frame_laser_shot_2angles_rad<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime_diff;
        s << ", " << x.num_echoes;
        s << ", " << x.line_angle_diff;
        s << ", " << x.frame_angle_diff;
        s << ", " << x.backgnd_rad;

        s << ", [";
        for (std::size_t n=0; n<x.deltas_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.deltas[n].systime
            << ", " << x.deltas[n].line_angle
            << ", " << x.deltas[n].frame_angle
            << ", " << x.deltas[n].num_echoes
            << ", " << x.deltas[n].backgnd_rad
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, packed_frame_laser_shot_2angles_rad<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime_diff;
        s >> ',' >> x.num_echoes;
        s >> ',' >> x.line_angle_diff;
        s >> ',' >> x.frame_angle_diff;
        s >> ',' >> x.backgnd_rad;

        s >> ',' >> '[' >> std::ws;
        x.deltas_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.deltas_size;
            if (s
                >> '{' >> (x.deltas[x.deltas_size-1].systime)
                >> ',' >> (x.deltas[x.deltas_size-1].line_angle)
                >> ',' >> (x.deltas[x.deltas_size-1].frame_angle)
                >> ',' >> (x.deltas[x.deltas_size-1].num_echoes)
                >> ',' >> (x.deltas[x.deltas_size-1].backgnd_rad)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct packed_key_echo
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef packed_key_echo<rebind_it> type;
    };

    typedef it iterator_type;

    packed_key_echo(it begin, it end, bool dirty=false)
        : range(begin)
        , ampl(begin)
        , refl(begin)
        , flags(begin)
        , dev(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 47, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 96};
    it begin() const { return range.begin(); }
    it end() const { return dev.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t range;
    #else
    field<int32_t, sc_int32, 0, it> range;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t ampl;
    #else
    field<uint16_t, sc_uint16, 32, it> ampl;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t refl;
    #else
    field<int16_t, sc_int16, 48, it> refl;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t flags;
    #else
    field<uint16_t, sc_uint16, 64, it> flags;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t dev;
    #else
    field<uint16_t, sc_uint16, 80, it> dev;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    packed_key_echo& operator=(const packed_key_echo<ito>& o) {
        range = o.range;
        ampl = o.ampl;
        refl = o.refl;
        flags = o.flags;
        dev = o.dev;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct packed_key_echo<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef packed_key_echo<rebind_it> type;
    };

    enum { id_main = 47, id_sub = 0};

    int32_t          range;
    uint16_t         ampl;
    int16_t          refl;
    uint16_t         flags;
    uint16_t         dev;

    packed_key_echo() {}
    template<class it>
    packed_key_echo(const packed_key_echo<it>& o) {
        range = o.range;
        ampl = o.ampl;
        refl = o.refl;
        flags = o.flags;
        dev = o.dev;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const packed_key_echo<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.range;
        s << ", " << x.ampl;
        s << ", " << x.refl;
        s << ", " << x.flags;
        s << ", " << x.dev;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, packed_key_echo<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.range;
        s >> ',' >> x.ampl;
        s >> ',' >> x.refl;
        s >> ',' >> x.flags;
        s >> ',' >> x.dev;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct packed_key_laser_shot_2angles
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef packed_key_laser_shot_2angles<rebind_it> type;
    };

    typedef it iterator_type;

    packed_key_laser_shot_2angles(it begin, it end, bool dirty=false)
        : systime(begin)
        , num_echoes(begin)
        , line_angle(begin)
        , frame_angle(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 45, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 96};
    it begin() const { return systime.begin(); }
    it end() const { return frame_angle.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t num_echoes;
    #else
    field<uint8_t, sc_uint8, 32, it> num_echoes;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle;
    #else
    field<uint32_t, sc_uint24, 40, it> line_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_angle;
    #else
    field<uint32_t, sc_uint32, 64, it> frame_angle;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    packed_key_laser_shot_2angles& operator=(const packed_key_laser_shot_2angles<ito>& o) {
        systime = o.systime;
        num_echoes = o.num_echoes;
        line_angle = o.line_angle;
        frame_angle = o.frame_angle;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct packed_key_laser_shot_2angles<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef packed_key_laser_shot_2angles<rebind_it> type;
    };

    enum { id_main = 45, id_sub = 0};

    uint32_t         systime;
    uint8_t          num_echoes;
    uint32_t         line_angle;
    uint32_t         frame_angle;

    packed_key_laser_shot_2angles() {}
    template<class it>
    packed_key_laser_shot_2angles(const packed_key_laser_shot_2angles<it>& o) {
        systime = o.systime;
        num_echoes = o.num_echoes;
        line_angle = o.line_angle;
        frame_angle = o.frame_angle;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const packed_key_laser_shot_2angles<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.num_echoes;
        s << ", " << x.line_angle;
        s << ", " << x.frame_angle;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, packed_key_laser_shot_2angles<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.num_echoes;
        s >> ',' >> x.line_angle;
        s >> ',' >> x.frame_angle;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct packed_key_laser_shot_2angles_rad
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef packed_key_laser_shot_2angles_rad<rebind_it> type;
    };

    typedef it iterator_type;

    packed_key_laser_shot_2angles_rad(it begin, it end, bool dirty=false)
        : systime(begin)
        , num_echoes(begin)
        , line_angle(begin)
        , frame_angle(begin)
        , backgnd_rad(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 51, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 128};
    it begin() const { return systime.begin(); }
    it end() const { return backgnd_rad.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t num_echoes;
    #else
    field<uint8_t, sc_uint8, 32, it> num_echoes;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_angle;
    #else
    field<uint32_t, sc_uint24, 40, it> line_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_angle;
    #else
    field<uint32_t, sc_uint32, 64, it> frame_angle;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t backgnd_rad;
    #else
    field<uint16_t, sc_uint16, 96, it> backgnd_rad;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    packed_key_laser_shot_2angles_rad& operator=(const packed_key_laser_shot_2angles_rad<ito>& o) {
        systime = o.systime;
        num_echoes = o.num_echoes;
        line_angle = o.line_angle;
        frame_angle = o.frame_angle;
        backgnd_rad = o.backgnd_rad;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct packed_key_laser_shot_2angles_rad<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef packed_key_laser_shot_2angles_rad<rebind_it> type;
    };

    enum { id_main = 51, id_sub = 0};

    uint32_t         systime;
    uint8_t          num_echoes;
    uint32_t         line_angle;
    uint32_t         frame_angle;
    uint16_t         backgnd_rad;

    packed_key_laser_shot_2angles_rad() {}
    template<class it>
    packed_key_laser_shot_2angles_rad(const packed_key_laser_shot_2angles_rad<it>& o) {
        systime = o.systime;
        num_echoes = o.num_echoes;
        line_angle = o.line_angle;
        frame_angle = o.frame_angle;
        backgnd_rad = o.backgnd_rad;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const packed_key_laser_shot_2angles_rad<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.num_echoes;
        s << ", " << x.line_angle;
        s << ", " << x.frame_angle;
        s << ", " << x.backgnd_rad;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, packed_key_laser_shot_2angles_rad<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.num_echoes;
        s >> ',' >> x.line_angle;
        s >> ',' >> x.frame_angle;
        s >> ',' >> x.backgnd_rad;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! pulse per second, external time synchronisation

//! <para>This package belongs to the predefined selectors:</para>
//! <para>data, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct pps_sync
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef pps_sync<rebind_it> type;
    };

    typedef it iterator_type;

    pps_sync(it begin, it end, bool dirty=false)
        : systime(begin)
        , pps(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 12, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 64};
    it begin() const { return systime.begin(); }
    it end() const { return pps.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  internal time of external PPS pulse in units of uints.time_uint 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t pps;//!<  absolute time of pps pulse since begin of week or start of day [msec] 
    #else
    field<uint32_t, sc_uint32, 32, it> pps;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    pps_sync& operator=(const pps_sync<ito>& o) {
        systime = o.systime;
        pps = o.pps;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct pps_sync<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef pps_sync<rebind_it> type;
    };

    enum { id_main = 12, id_sub = 0};

    uint32_t         systime;
    uint32_t         pps;

    pps_sync() {}
    template<class it>
    pps_sync(const pps_sync<it>& o) {
        systime = o.systime;
        pps = o.pps;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const pps_sync<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.pps;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, pps_sync<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.pps;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! pulse per second, external time synchronisation

//! <para>This package belongs to the predefined selectors:</para>
//! <para>data, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct pps_sync_ext
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef pps_sync_ext<rebind_it> type;
    };

    typedef it iterator_type;

    pps_sync_ext(it begin, it end, bool dirty=false)
        : systime(begin)
        , pps(begin)
        , year(begin)
        , mon(begin)
        , day(begin)
        , hour(begin)
        , min(begin)
        , sec(begin)
        , flags(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 12, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 128};
    it begin() const { return systime.begin(); }
    it end() const { return flags.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  internal time of external PPS pulse in units of uints.time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t pps;//!<  absolute time of pps pulse since begin of week or start of day [msec] 
    #else
    field<uint32_t, sc_uint32, 32, it> pps;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t year;//!<  UTC year                              [ 1 yr] 
    #else
    field<uint16_t, sc_uint16, 64, it> year;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t mon;//!<  UTC month, 1..12                      [ 1 mon] 
    #else
    field<uint8_t, sc_uint8, 80, it> mon;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t day;//!<  UTC day of month, 1..31               [ 1 day] 
    #else
    field<uint8_t, sc_uint8, 88, it> day;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t hour;//!<  UTC hour of day, 0 .. 23              [ 1 hr] 
    #else
    field<uint8_t, sc_uint8, 96, it> hour;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t min;//!<  UTC minute of hour, 0 .. 59           [ 1 min] 
    #else
    field<uint8_t, sc_uint8, 104, it> min;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t sec;//!<  UTC second of minute, 0 .. 59         [ 1 sec] 
    #else
    field<uint8_t, sc_uint8, 112, it> sec;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t flags;//!<  B0 .. Time of week valid; B1 .. valid week number; B2 .. leap seconds known 
    #else
    field<uint8_t, sc_uint8, 120, it> flags;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    pps_sync_ext& operator=(const pps_sync_ext<ito>& o) {
        systime = o.systime;
        pps = o.pps;
        year = o.year;
        mon = o.mon;
        day = o.day;
        hour = o.hour;
        min = o.min;
        sec = o.sec;
        flags = o.flags;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct pps_sync_ext<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef pps_sync_ext<rebind_it> type;
    };

    enum { id_main = 12, id_sub = 1};

    uint32_t         systime;
    uint32_t         pps;
    uint16_t         year;
    uint8_t          mon;
    uint8_t          day;
    uint8_t          hour;
    uint8_t          min;
    uint8_t          sec;
    uint8_t          flags;

    pps_sync_ext() {}
    template<class it>
    pps_sync_ext(const pps_sync_ext<it>& o) {
        systime = o.systime;
        pps = o.pps;
        year = o.year;
        mon = o.mon;
        day = o.day;
        hour = o.hour;
        min = o.min;
        sec = o.sec;
        flags = o.flags;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const pps_sync_ext<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.pps;
        s << ", " << x.year;
        s << ", " << x.mon;
        s << ", " << x.day;
        s << ", " << x.hour;
        s << ", " << x.min;
        s << ", " << x.sec;
        s << ", " << x.flags;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, pps_sync_ext<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.pps;
        s >> ',' >> x.year;
        s >> ',' >> x.mon;
        s >> ',' >> x.day;
        s >> ',' >> x.hour;
        s >> ',' >> x.min;
        s >> ',' >> x.sec;
        s >> ',' >> x.flags;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct range_calc
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef range_calc<rebind_it> type;
    };

    typedef it iterator_type;

    range_calc(it begin, it end, bool dirty=false)
        : scale(begin)
        , offset(begin)
        , startpos(begin)
        , full_scale_adc_lp(begin)
        , full_scale_adc_hp(begin)
        , full_scale_adc_shp(begin)
        , full_scale_adc_i(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 50005, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 160};
    it begin() const { return scale.begin(); }
    it end() const { return full_scale_adc_i.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float scale;
    #else
    field<float, sc_float32, 0, it> scale;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float offset;
    #else
    field<float, sc_float32, 32, it> offset;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t startpos;
    #else
    field<uint32_t, sc_uint32, 64, it> startpos;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t full_scale_adc_lp;
    #else
    field<uint16_t, sc_uint16, 96, it> full_scale_adc_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t full_scale_adc_hp;
    #else
    field<uint16_t, sc_uint16, 112, it> full_scale_adc_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t full_scale_adc_shp;
    #else
    field<uint16_t, sc_uint16, 128, it> full_scale_adc_shp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t full_scale_adc_i;
    #else
    field<uint16_t, sc_uint16, 144, it> full_scale_adc_i;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    range_calc& operator=(const range_calc<ito>& o) {
        scale = o.scale;
        offset = o.offset;
        startpos = o.startpos;
        full_scale_adc_lp = o.full_scale_adc_lp;
        full_scale_adc_hp = o.full_scale_adc_hp;
        full_scale_adc_shp = o.full_scale_adc_shp;
        full_scale_adc_i = o.full_scale_adc_i;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct range_calc<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef range_calc<rebind_it> type;
    };

    enum { id_main = 50005, id_sub = 0};

    float            scale;
    float            offset;
    uint32_t         startpos;
    uint16_t         full_scale_adc_lp;
    uint16_t         full_scale_adc_hp;
    uint16_t         full_scale_adc_shp;
    uint16_t         full_scale_adc_i;

    range_calc() {}
    template<class it>
    range_calc(const range_calc<it>& o) {
        scale = o.scale;
        offset = o.offset;
        startpos = o.startpos;
        full_scale_adc_lp = o.full_scale_adc_lp;
        full_scale_adc_hp = o.full_scale_adc_hp;
        full_scale_adc_shp = o.full_scale_adc_shp;
        full_scale_adc_i = o.full_scale_adc_i;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const range_calc<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.scale;
        s << ", " << x.offset;
        s << ", " << x.startpos;
        s << ", " << x.full_scale_adc_lp;
        s << ", " << x.full_scale_adc_hp;
        s << ", " << x.full_scale_adc_shp;
        s << ", " << x.full_scale_adc_i;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, range_calc<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.scale;
        s >> ',' >> x.offset;
        s >> ',' >> x.startpos;
        s >> ',' >> x.full_scale_adc_lp;
        s >> ',' >> x.full_scale_adc_hp;
        s >> ',' >> x.full_scale_adc_shp;
        s >> ',' >> x.full_scale_adc_i;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct range_finder_debug_acq
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_debug_acq<rebind_it> type;
    };

    typedef it iterator_type;

    range_finder_debug_acq(it begin, it end, bool dirty=false)
        : sog(begin)
        , eog(begin)
        , acq_min_thresh_lp(begin)
        , acq_min_thresh_hp(begin)
        , thres_tab_time(begin)
        , thres_tab_date(begin)
        , thres_tab_crc(begin)
        , max_targets_lo(begin)
        , max_targets_hi(begin)
        , meas_max_targets(begin)
        , shp_ampl_thresh(begin)
        , shp_ofs_thresh(begin)
        , shp_integ_length(begin)
        , shp_ampl_div(begin)
        , meas_mode(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 35, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 368};
    it begin() const { return sog.begin(); }
    it end() const { return meas_mode.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t sog;
    #else
    field<uint16_t, sc_uint16, 0, it> sog;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t eog;
    #else
    field<uint16_t, sc_uint16, 16, it> eog;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t acq_min_thresh_lp;
    #else
    field<uint32_t, sc_uint32, 32, it> acq_min_thresh_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t acq_min_thresh_hp;
    #else
    field<uint32_t, sc_uint32, 64, it> acq_min_thresh_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char thres_tab_time[9];
    #else
    array<char, 9, sc_char, 96, it> thres_tab_time;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char thres_tab_date[9];
    #else
    array<char, 9, sc_char, 168, it> thres_tab_date;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t thres_tab_crc;
    #else
    field<uint16_t, sc_uint16, 240, it> thres_tab_crc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t max_targets_lo;
    #else
    field<uint8_t, sc_uint8, 256, it> max_targets_lo;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t max_targets_hi;
    #else
    field<uint8_t, sc_uint8, 264, it> max_targets_hi;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t meas_max_targets;
    #else
    field<uint8_t, sc_uint8, 272, it> meas_max_targets;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t shp_ampl_thresh;
    #else
    field<uint32_t, sc_uint32, 280, it> shp_ampl_thresh;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t shp_ofs_thresh;
    #else
    field<uint32_t, sc_uint32, 312, it> shp_ofs_thresh;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t shp_integ_length;
    #else
    field<uint8_t, sc_uint8, 344, it> shp_integ_length;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t shp_ampl_div;
    #else
    field<uint8_t, sc_uint8, 352, it> shp_ampl_div;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t meas_mode;
    #else
    field<uint8_t, sc_uint8, 360, it> meas_mode;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    range_finder_debug_acq& operator=(const range_finder_debug_acq<ito>& o) {
        sog = o.sog;
        eog = o.eog;
        acq_min_thresh_lp = o.acq_min_thresh_lp;
        acq_min_thresh_hp = o.acq_min_thresh_hp;
        for(unsigned n=0; n<9; ++n) thres_tab_time[n] = o.thres_tab_time[n];
        for(unsigned n=0; n<9; ++n) thres_tab_date[n] = o.thres_tab_date[n];
        thres_tab_crc = o.thres_tab_crc;
        max_targets_lo = o.max_targets_lo;
        max_targets_hi = o.max_targets_hi;
        meas_max_targets = o.meas_max_targets;
        shp_ampl_thresh = o.shp_ampl_thresh;
        shp_ofs_thresh = o.shp_ofs_thresh;
        shp_integ_length = o.shp_integ_length;
        shp_ampl_div = o.shp_ampl_div;
        meas_mode = o.meas_mode;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct range_finder_debug_acq<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_debug_acq<rebind_it> type;
    };

    enum { id_main = 35, id_sub = 0};

    uint16_t         sog;
    uint16_t         eog;
    uint32_t         acq_min_thresh_lp;
    uint32_t         acq_min_thresh_hp;
    char             thres_tab_time[9];
    char             thres_tab_date[9];
    uint16_t         thres_tab_crc;
    uint8_t          max_targets_lo;
    uint8_t          max_targets_hi;
    uint8_t          meas_max_targets;
    uint32_t         shp_ampl_thresh;
    uint32_t         shp_ofs_thresh;
    uint8_t          shp_integ_length;
    uint8_t          shp_ampl_div;
    uint8_t          meas_mode;

    range_finder_debug_acq() {}
    template<class it>
    range_finder_debug_acq(const range_finder_debug_acq<it>& o) {
        sog = o.sog;
        eog = o.eog;
        acq_min_thresh_lp = o.acq_min_thresh_lp;
        acq_min_thresh_hp = o.acq_min_thresh_hp;
        for(unsigned n=0; n<9; ++n) thres_tab_time[n] = o.thres_tab_time[n];
        for(unsigned n=0; n<9; ++n) thres_tab_date[n] = o.thres_tab_date[n];
        thres_tab_crc = o.thres_tab_crc;
        max_targets_lo = o.max_targets_lo;
        max_targets_hi = o.max_targets_hi;
        meas_max_targets = o.meas_max_targets;
        shp_ampl_thresh = o.shp_ampl_thresh;
        shp_ofs_thresh = o.shp_ofs_thresh;
        shp_integ_length = o.shp_integ_length;
        shp_ampl_div = o.shp_ampl_div;
        meas_mode = o.meas_mode;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const range_finder_debug_acq<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.sog;
        s << ", " << x.eog;
        s << ", " << x.acq_min_thresh_lp;
        s << ", " << x.acq_min_thresh_hp;
        s << ", "; write_array(s, 9, x.thres_tab_time);
        s << ", "; write_array(s, 9, x.thres_tab_date);
        s << ", " << x.thres_tab_crc;
        s << ", " << x.max_targets_lo;
        s << ", " << x.max_targets_hi;
        s << ", " << x.meas_max_targets;
        s << ", " << x.shp_ampl_thresh;
        s << ", " << x.shp_ofs_thresh;
        s << ", " << x.shp_integ_length;
        s << ", " << x.shp_ampl_div;
        s << ", " << x.meas_mode;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, range_finder_debug_acq<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.sog;
        s >> ',' >> x.eog;
        s >> ',' >> x.acq_min_thresh_lp;
        s >> ',' >> x.acq_min_thresh_hp;
        s >> ','; read_array(s, 9, x.thres_tab_time);
        s >> ','; read_array(s, 9, x.thres_tab_date);
        s >> ',' >> x.thres_tab_crc;
        s >> ',' >> x.max_targets_lo;
        s >> ',' >> x.max_targets_hi;
        s >> ',' >> x.meas_max_targets;
        s >> ',' >> x.shp_ampl_thresh;
        s >> ',' >> x.shp_ofs_thresh;
        s >> ',' >> x.shp_integ_length;
        s >> ',' >> x.shp_ampl_div;
        s >> ',' >> x.meas_mode;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct range_finder_debug_calc
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_debug_calc<rebind_it> type;
    };

    typedef it iterator_type;

    range_finder_debug_calc(it begin, it end, bool dirty=false)
        : pdae_tab_time(begin)
        , pdae_tab_date(begin)
        , pdae_tab_crc(begin)
        , ampl_tab_time(begin)
        , ampl_tab_date(begin)
        , ampl_tab_crc(begin)
        , filt_ampl_min(begin)
        , filt_rng_min(begin)
        , rng_offset(begin)
        , rng_scale(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 36, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 448};
    it begin() const { return pdae_tab_time.begin(); }
    it end() const { return rng_scale.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    char pdae_tab_time[9];
    #else
    array<char, 9, sc_char, 0, it> pdae_tab_time;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char pdae_tab_date[9];
    #else
    array<char, 9, sc_char, 72, it> pdae_tab_date;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t pdae_tab_crc;
    #else
    field<uint16_t, sc_uint16, 144, it> pdae_tab_crc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char ampl_tab_time[9];
    #else
    array<char, 9, sc_char, 160, it> ampl_tab_time;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char ampl_tab_date[9];
    #else
    array<char, 9, sc_char, 232, it> ampl_tab_date;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t ampl_tab_crc;
    #else
    field<uint16_t, sc_uint16, 304, it> ampl_tab_crc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t filt_ampl_min;
    #else
    field<uint32_t, sc_uint32, 320, it> filt_ampl_min;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float filt_rng_min;
    #else
    field<float, sc_float32, 352, it> filt_rng_min;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float rng_offset;
    #else
    field<float, sc_float32, 384, it> rng_offset;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float rng_scale;
    #else
    field<float, sc_float32, 416, it> rng_scale;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    range_finder_debug_calc& operator=(const range_finder_debug_calc<ito>& o) {
        for(unsigned n=0; n<9; ++n) pdae_tab_time[n] = o.pdae_tab_time[n];
        for(unsigned n=0; n<9; ++n) pdae_tab_date[n] = o.pdae_tab_date[n];
        pdae_tab_crc = o.pdae_tab_crc;
        for(unsigned n=0; n<9; ++n) ampl_tab_time[n] = o.ampl_tab_time[n];
        for(unsigned n=0; n<9; ++n) ampl_tab_date[n] = o.ampl_tab_date[n];
        ampl_tab_crc = o.ampl_tab_crc;
        filt_ampl_min = o.filt_ampl_min;
        filt_rng_min = o.filt_rng_min;
        rng_offset = o.rng_offset;
        rng_scale = o.rng_scale;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct range_finder_debug_calc<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_debug_calc<rebind_it> type;
    };

    enum { id_main = 36, id_sub = 0};

    char             pdae_tab_time[9];
    char             pdae_tab_date[9];
    uint16_t         pdae_tab_crc;
    char             ampl_tab_time[9];
    char             ampl_tab_date[9];
    uint16_t         ampl_tab_crc;
    uint32_t         filt_ampl_min;
    float            filt_rng_min;
    float            rng_offset;
    float            rng_scale;

    range_finder_debug_calc() {}
    template<class it>
    range_finder_debug_calc(const range_finder_debug_calc<it>& o) {
        for(unsigned n=0; n<9; ++n) pdae_tab_time[n] = o.pdae_tab_time[n];
        for(unsigned n=0; n<9; ++n) pdae_tab_date[n] = o.pdae_tab_date[n];
        pdae_tab_crc = o.pdae_tab_crc;
        for(unsigned n=0; n<9; ++n) ampl_tab_time[n] = o.ampl_tab_time[n];
        for(unsigned n=0; n<9; ++n) ampl_tab_date[n] = o.ampl_tab_date[n];
        ampl_tab_crc = o.ampl_tab_crc;
        filt_ampl_min = o.filt_ampl_min;
        filt_rng_min = o.filt_rng_min;
        rng_offset = o.rng_offset;
        rng_scale = o.rng_scale;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const range_finder_debug_calc<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", "; write_array(s, 9, x.pdae_tab_time);
        s << ", "; write_array(s, 9, x.pdae_tab_date);
        s << ", " << x.pdae_tab_crc;
        s << ", "; write_array(s, 9, x.ampl_tab_time);
        s << ", "; write_array(s, 9, x.ampl_tab_date);
        s << ", " << x.ampl_tab_crc;
        s << ", " << x.filt_ampl_min;
        s << ", " << x.filt_rng_min;
        s << ", " << x.rng_offset;
        s << ", " << x.rng_scale;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, range_finder_debug_calc<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ','; read_array(s, 9, x.pdae_tab_time);
        s >> ','; read_array(s, 9, x.pdae_tab_date);
        s >> ',' >> x.pdae_tab_crc;
        s >> ','; read_array(s, 9, x.ampl_tab_time);
        s >> ','; read_array(s, 9, x.ampl_tab_date);
        s >> ',' >> x.ampl_tab_crc;
        s >> ',' >> x.filt_ampl_min;
        s >> ',' >> x.filt_rng_min;
        s >> ',' >> x.rng_offset;
        s >> ',' >> x.rng_scale;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct range_finder_debug_laser
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_debug_laser<rebind_it> type;
    };

    typedef it iterator_type;

    range_finder_debug_laser(it begin, it end, bool dirty=false)
        : laser_clock(begin)
        , laser_pump(begin)
        , laser_prr(begin)
        , laser_tec_w(begin)
        , shp_shutdown_th(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 34, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 120};
    it begin() const { return laser_clock.begin(); }
    it end() const { return shp_shutdown_th.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t laser_clock;
    #else
    field<uint8_t, sc_uint8, 0, it> laser_clock;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t laser_pump;
    #else
    field<uint16_t, sc_uint16, 8, it> laser_pump;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float laser_prr;
    #else
    field<float, sc_float32, 24, it> laser_prr;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float laser_tec_w;
    #else
    field<float, sc_float32, 56, it> laser_tec_w;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t shp_shutdown_th;
    #else
    field<uint32_t, sc_uint32, 88, it> shp_shutdown_th;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    range_finder_debug_laser& operator=(const range_finder_debug_laser<ito>& o) {
        laser_clock = o.laser_clock;
        laser_pump = o.laser_pump;
        laser_prr = o.laser_prr;
        laser_tec_w = o.laser_tec_w;
        shp_shutdown_th = o.shp_shutdown_th;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct range_finder_debug_laser<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_debug_laser<rebind_it> type;
    };

    enum { id_main = 34, id_sub = 0};

    uint8_t          laser_clock;
    uint16_t         laser_pump;
    float            laser_prr;
    float            laser_tec_w;
    uint32_t         shp_shutdown_th;

    range_finder_debug_laser() {}
    template<class it>
    range_finder_debug_laser(const range_finder_debug_laser<it>& o) {
        laser_clock = o.laser_clock;
        laser_pump = o.laser_pump;
        laser_prr = o.laser_prr;
        laser_tec_w = o.laser_tec_w;
        shp_shutdown_th = o.shp_shutdown_th;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const range_finder_debug_laser<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.laser_clock;
        s << ", " << x.laser_pump;
        s << ", " << x.laser_prr;
        s << ", " << x.laser_tec_w;
        s << ", " << x.shp_shutdown_th;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, range_finder_debug_laser<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.laser_clock;
        s >> ',' >> x.laser_pump;
        s >> ',' >> x.laser_prr;
        s >> ',' >> x.laser_tec_w;
        s >> ',' >> x.shp_shutdown_th;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct range_finder_debug_rcv
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_debug_rcv<rebind_it> type;
    };

    typedef it iterator_type;

    range_finder_debug_rcv(it begin, it end, bool dirty=false)
        : rcv_rpr_mode(begin)
        , rcv_rpr_w(begin)
        , rcv_hv_w(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 37, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 72};
    it begin() const { return rcv_rpr_mode.begin(); }
    it end() const { return rcv_hv_w.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t rcv_rpr_mode;
    #else
    field<uint8_t, sc_uint8, 0, it> rcv_rpr_mode;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float rcv_rpr_w;
    #else
    field<float, sc_float32, 8, it> rcv_rpr_w;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float rcv_hv_w;
    #else
    field<float, sc_float32, 40, it> rcv_hv_w;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    range_finder_debug_rcv& operator=(const range_finder_debug_rcv<ito>& o) {
        rcv_rpr_mode = o.rcv_rpr_mode;
        rcv_rpr_w = o.rcv_rpr_w;
        rcv_hv_w = o.rcv_hv_w;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct range_finder_debug_rcv<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_debug_rcv<rebind_it> type;
    };

    enum { id_main = 37, id_sub = 0};

    uint8_t          rcv_rpr_mode;
    float            rcv_rpr_w;
    float            rcv_hv_w;

    range_finder_debug_rcv() {}
    template<class it>
    range_finder_debug_rcv(const range_finder_debug_rcv<it>& o) {
        rcv_rpr_mode = o.rcv_rpr_mode;
        rcv_rpr_w = o.rcv_rpr_w;
        rcv_hv_w = o.rcv_hv_w;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const range_finder_debug_rcv<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.rcv_rpr_mode;
        s << ", " << x.rcv_rpr_w;
        s << ", " << x.rcv_hv_w;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, range_finder_debug_rcv<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.rcv_rpr_mode;
        s >> ',' >> x.rcv_rpr_w;
        s >> ',' >> x.rcv_hv_w;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct range_finder_program
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_program<rebind_it> type;
    };

    typedef it iterator_type;

    range_finder_program(it begin, it end, bool dirty=false)
        : meas_prog(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 38, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 8};
    it begin() const { return meas_prog.begin(); }
    it end() const { return meas_prog.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t meas_prog;
    #else
    field<uint8_t, sc_uint8, 0, it> meas_prog;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    range_finder_program& operator=(const range_finder_program<ito>& o) {
        meas_prog = o.meas_prog;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct range_finder_program<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_program<rebind_it> type;
    };

    enum { id_main = 38, id_sub = 0};

    uint8_t          meas_prog;

    range_finder_program() {}
    template<class it>
    range_finder_program(const range_finder_program<it>& o) {
        meas_prog = o.meas_prog;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const range_finder_program<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.meas_prog;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, range_finder_program<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.meas_prog;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct range_finder_settings
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_settings<rebind_it> type;
    };

    typedef it iterator_type;

    range_finder_settings(it begin, it end, bool dirty=false)
        : laser_clock(begin)
        , laser_pump(begin)
        , laser_prr(begin)
        , meas_mode(begin)
        , sog(begin)
        , eog(begin)
        , max_targets_lo(begin)
        , max_targets_hi(begin)
        , rng_offset(begin)
        , pdae_tab_time(begin)
        , pdae_tab_date(begin)
        , pdae_tab_crc(begin)
        , thres_tab_time(begin)
        , thres_tab_date(begin)
        , thres_tab_crc(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 7, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 464};
    it begin() const { return laser_clock.begin(); }
    it end() const { return thres_tab_crc.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t laser_clock;
    #else
    field<uint8_t, sc_uint8, 0, it> laser_clock;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t laser_pump;
    #else
    field<uint16_t, sc_uint16, 8, it> laser_pump;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float laser_prr;
    #else
    field<float, sc_float32, 24, it> laser_prr;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t meas_mode;
    #else
    field<uint8_t, sc_uint8, 56, it> meas_mode;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t sog;
    #else
    field<uint16_t, sc_uint16, 64, it> sog;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t eog;
    #else
    field<uint16_t, sc_uint16, 80, it> eog;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t max_targets_lo;
    #else
    field<uint8_t, sc_uint8, 96, it> max_targets_lo;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t max_targets_hi;
    #else
    field<uint8_t, sc_uint8, 104, it> max_targets_hi;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float rng_offset;
    #else
    field<float, sc_float32, 112, it> rng_offset;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char pdae_tab_time[9];
    #else
    array<char, 9, sc_char, 144, it> pdae_tab_time;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char pdae_tab_date[9];
    #else
    array<char, 9, sc_char, 216, it> pdae_tab_date;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t pdae_tab_crc;
    #else
    field<uint16_t, sc_uint16, 288, it> pdae_tab_crc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char thres_tab_time[9];
    #else
    array<char, 9, sc_char, 304, it> thres_tab_time;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char thres_tab_date[9];
    #else
    array<char, 9, sc_char, 376, it> thres_tab_date;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t thres_tab_crc;
    #else
    field<uint16_t, sc_uint16, 448, it> thres_tab_crc;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    range_finder_settings& operator=(const range_finder_settings<ito>& o) {
        laser_clock = o.laser_clock;
        laser_pump = o.laser_pump;
        laser_prr = o.laser_prr;
        meas_mode = o.meas_mode;
        sog = o.sog;
        eog = o.eog;
        max_targets_lo = o.max_targets_lo;
        max_targets_hi = o.max_targets_hi;
        rng_offset = o.rng_offset;
        for(unsigned n=0; n<9; ++n) pdae_tab_time[n] = o.pdae_tab_time[n];
        for(unsigned n=0; n<9; ++n) pdae_tab_date[n] = o.pdae_tab_date[n];
        pdae_tab_crc = o.pdae_tab_crc;
        for(unsigned n=0; n<9; ++n) thres_tab_time[n] = o.thres_tab_time[n];
        for(unsigned n=0; n<9; ++n) thres_tab_date[n] = o.thres_tab_date[n];
        thres_tab_crc = o.thres_tab_crc;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct range_finder_settings<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef range_finder_settings<rebind_it> type;
    };

    enum { id_main = 7, id_sub = 0};

    uint8_t          laser_clock;
    uint16_t         laser_pump;
    float            laser_prr;
    uint8_t          meas_mode;
    uint16_t         sog;
    uint16_t         eog;
    uint8_t          max_targets_lo;
    uint8_t          max_targets_hi;
    float            rng_offset;
    char             pdae_tab_time[9];
    char             pdae_tab_date[9];
    uint16_t         pdae_tab_crc;
    char             thres_tab_time[9];
    char             thres_tab_date[9];
    uint16_t         thres_tab_crc;

    range_finder_settings() {}
    template<class it>
    range_finder_settings(const range_finder_settings<it>& o) {
        laser_clock = o.laser_clock;
        laser_pump = o.laser_pump;
        laser_prr = o.laser_prr;
        meas_mode = o.meas_mode;
        sog = o.sog;
        eog = o.eog;
        max_targets_lo = o.max_targets_lo;
        max_targets_hi = o.max_targets_hi;
        rng_offset = o.rng_offset;
        for(unsigned n=0; n<9; ++n) pdae_tab_time[n] = o.pdae_tab_time[n];
        for(unsigned n=0; n<9; ++n) pdae_tab_date[n] = o.pdae_tab_date[n];
        pdae_tab_crc = o.pdae_tab_crc;
        for(unsigned n=0; n<9; ++n) thres_tab_time[n] = o.thres_tab_time[n];
        for(unsigned n=0; n<9; ++n) thres_tab_date[n] = o.thres_tab_date[n];
        thres_tab_crc = o.thres_tab_crc;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const range_finder_settings<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.laser_clock;
        s << ", " << x.laser_pump;
        s << ", " << x.laser_prr;
        s << ", " << x.meas_mode;
        s << ", " << x.sog;
        s << ", " << x.eog;
        s << ", " << x.max_targets_lo;
        s << ", " << x.max_targets_hi;
        s << ", " << x.rng_offset;
        s << ", "; write_array(s, 9, x.pdae_tab_time);
        s << ", "; write_array(s, 9, x.pdae_tab_date);
        s << ", " << x.pdae_tab_crc;
        s << ", "; write_array(s, 9, x.thres_tab_time);
        s << ", "; write_array(s, 9, x.thres_tab_date);
        s << ", " << x.thres_tab_crc;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, range_finder_settings<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.laser_clock;
        s >> ',' >> x.laser_pump;
        s >> ',' >> x.laser_prr;
        s >> ',' >> x.meas_mode;
        s >> ',' >> x.sog;
        s >> ',' >> x.eog;
        s >> ',' >> x.max_targets_lo;
        s >> ',' >> x.max_targets_hi;
        s >> ',' >> x.rng_offset;
        s >> ','; read_array(s, 9, x.pdae_tab_time);
        s >> ','; read_array(s, 9, x.pdae_tab_date);
        s >> ',' >> x.pdae_tab_crc;
        s >> ','; read_array(s, 9, x.thres_tab_time);
        s >> ','; read_array(s, 9, x.thres_tab_date);
        s >> ',' >> x.thres_tab_crc;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct rel_refl_table
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef rel_refl_table<rebind_it> type;
    };

    typedef it iterator_type;

    rel_refl_table(it begin, it end, bool dirty=false)
        : entries(begin, end, entries_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 33, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 1024};
    it begin() const { return entries.begin(); }
    it end() const { return entries.end(); }

    #endif //DOXYGEN

    std::size_t entries_size;
    enum { entries_max_size = 16 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : rng(begin, begin_bit)
            , add(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        float rng;
        #else
        field<float, sc_float32, 0, it> rng;
        #endif
        #ifdef DOXYGEN
        float add;
        #else
        field<float, sc_float32, 32, it> add;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition entries[16];
    #else
    sequence<rel_refl_table, 64, 0, it> entries;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    rel_refl_table& operator=(const rel_refl_table<ito>& o) {
        entries_size = o.entries_size;
        for(unsigned n=0; n<entries_size; ++n){
            entries[n].rng = o.entries[n].rng;
            entries[n].add = o.entries[n].add;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct rel_refl_table<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef rel_refl_table<rebind_it> type;
    };

    enum { id_main = 33, id_sub = 0};

    std::size_t entries_size;
    enum { entries_max_size = 16 };
    struct sequence_definition {
        float            rng;
        float            add;
    } entries[16];

    rel_refl_table() {}
    template<class it>
    rel_refl_table(const rel_refl_table<it>& o) {
        entries_size = o.entries.size();
        for(unsigned n=0; n<entries_size; ++n){
            entries[n].rng = o.entries[n].rng;
            entries[n].add = o.entries[n].add;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const rel_refl_table<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {

        s << ", [";
        for (std::size_t n=0; n<x.entries_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.entries[n].rng
            << ", " << x.entries[n].add
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, rel_refl_table<it>& x) {
    package_istream_entry ok(s);
    if (ok) {

        s >> ',' >> '[' >> std::ws;
        x.entries_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.entries_size;
            if (s
                >> '{' >> (x.entries[x.entries_size-1].rng)
                >> ',' >> (x.entries[x.entries_size-1].add)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct sbl_dg_data
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef sbl_dg_data<rebind_it> type;
    };

    typedef it iterator_type;

    sbl_dg_data(it begin, it end, bool dirty=false)
        : smpl_cnt(begin)
        , channel_id(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 19, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 32800};
    it begin() const { return smpl_cnt.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t smpl_cnt;
    #else
    field<uint16_t, sc_uint16, 0, it> smpl_cnt;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t channel_id;
    #else
    field<uint8_t, sc_uint2, 16, it> channel_id;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 2048 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sample(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint16_t sample;
        #else
        field<uint16_t, sc_uint16, 0, it> sample;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[2048];
    #else
    sequence<sbl_dg_data, 16, 32, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    sbl_dg_data& operator=(const sbl_dg_data<ito>& o) {
        smpl_cnt = o.smpl_cnt;
        channel_id = o.channel_id;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct sbl_dg_data<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef sbl_dg_data<rebind_it> type;
    };

    enum { id_main = 19, id_sub = 0};

    uint16_t         smpl_cnt;
    uint8_t          channel_id;
    std::size_t data_size;
    enum { data_max_size = 2048 };
    struct sequence_definition {
        uint16_t         sample;
    } data[2048];

    sbl_dg_data() {}
    template<class it>
    sbl_dg_data(const sbl_dg_data<it>& o) {
        smpl_cnt = o.smpl_cnt;
        channel_id = o.channel_id;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const sbl_dg_data<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.smpl_cnt;
        s << ", " << x.channel_id;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].sample
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, sbl_dg_data<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.smpl_cnt;
        s >> ',' >> x.channel_id;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].sample)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct sbl_dg_data_compressed
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef sbl_dg_data_compressed<rebind_it> type;
    };

    typedef it iterator_type;

    sbl_dg_data_compressed(it begin, it end, bool dirty=false)
        : smpl_cnt(begin)
        , channel_id(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 55, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 32800};
    it begin() const { return smpl_cnt.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t smpl_cnt;
    #else
    field<uint16_t, sc_uint16, 0, it> smpl_cnt;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t channel_id;
    #else
    field<uint8_t, sc_uint2, 16, it> channel_id;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 2048 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sample(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint16_t sample;
        #else
        field<uint16_t, sc_uint16, 0, it> sample;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[2048];
    #else
    sequence<sbl_dg_data_compressed, 16, 32, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    sbl_dg_data_compressed& operator=(const sbl_dg_data_compressed<ito>& o) {
        smpl_cnt = o.smpl_cnt;
        channel_id = o.channel_id;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct sbl_dg_data_compressed<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef sbl_dg_data_compressed<rebind_it> type;
    };

    enum { id_main = 55, id_sub = 0};

    uint16_t         smpl_cnt;
    uint8_t          channel_id;
    std::size_t data_size;
    enum { data_max_size = 2048 };
    struct sequence_definition {
        uint16_t         sample;
    } data[2048];

    sbl_dg_data_compressed() {}
    template<class it>
    sbl_dg_data_compressed(const sbl_dg_data_compressed<it>& o) {
        smpl_cnt = o.smpl_cnt;
        channel_id = o.channel_id;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const sbl_dg_data_compressed<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.smpl_cnt;
        s << ", " << x.channel_id;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].sample
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, sbl_dg_data_compressed<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.smpl_cnt;
        s >> ',' >> x.channel_id;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].sample)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct sbl_dg_header
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef sbl_dg_header<rebind_it> type;
    };

    typedef it iterator_type;

    sbl_dg_header(it begin, it end, bool dirty=false)
        : mean_lp_chn(begin)
        , stddev_lp_chn(begin)
        , mean_hp_chn(begin)
        , stddev_hp_chn(begin)
        , phase(begin)
        , shp_ampl(begin)
        , refpulse_smpl_frac(begin)
        , refpulse_smpl_cnt(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 18, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 128};
    it begin() const { return mean_lp_chn.begin(); }
    it end() const { return refpulse_smpl_cnt.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t mean_lp_chn;
    #else
    field<uint16_t, sc_uint16, 0, it> mean_lp_chn;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t stddev_lp_chn;
    #else
    field<uint16_t, sc_uint16, 16, it> stddev_lp_chn;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t mean_hp_chn;
    #else
    field<uint16_t, sc_uint16, 32, it> mean_hp_chn;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t stddev_hp_chn;
    #else
    field<uint16_t, sc_uint16, 48, it> stddev_hp_chn;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t phase;
    #else
    field<uint8_t, sc_uint8, 64, it> phase;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t shp_ampl;
    #else
    field<uint16_t, sc_uint16, 72, it> shp_ampl;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t refpulse_smpl_frac;
    #else
    field<uint32_t, sc_uint18, 96, it> refpulse_smpl_frac;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t refpulse_smpl_cnt;
    #else
    field<uint16_t, sc_uint14, 114, it> refpulse_smpl_cnt;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    sbl_dg_header& operator=(const sbl_dg_header<ito>& o) {
        mean_lp_chn = o.mean_lp_chn;
        stddev_lp_chn = o.stddev_lp_chn;
        mean_hp_chn = o.mean_hp_chn;
        stddev_hp_chn = o.stddev_hp_chn;
        phase = o.phase;
        shp_ampl = o.shp_ampl;
        refpulse_smpl_frac = o.refpulse_smpl_frac;
        refpulse_smpl_cnt = o.refpulse_smpl_cnt;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct sbl_dg_header<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef sbl_dg_header<rebind_it> type;
    };

    enum { id_main = 18, id_sub = 0};

    uint16_t         mean_lp_chn;
    uint16_t         stddev_lp_chn;
    uint16_t         mean_hp_chn;
    uint16_t         stddev_hp_chn;
    uint8_t          phase;
    uint16_t         shp_ampl;
    uint32_t         refpulse_smpl_frac;
    uint16_t         refpulse_smpl_cnt;

    sbl_dg_header() {}
    template<class it>
    sbl_dg_header(const sbl_dg_header<it>& o) {
        mean_lp_chn = o.mean_lp_chn;
        stddev_lp_chn = o.stddev_lp_chn;
        mean_hp_chn = o.mean_hp_chn;
        stddev_hp_chn = o.stddev_hp_chn;
        phase = o.phase;
        shp_ampl = o.shp_ampl;
        refpulse_smpl_frac = o.refpulse_smpl_frac;
        refpulse_smpl_cnt = o.refpulse_smpl_cnt;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const sbl_dg_header<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.mean_lp_chn;
        s << ", " << x.stddev_lp_chn;
        s << ", " << x.mean_hp_chn;
        s << ", " << x.stddev_hp_chn;
        s << ", " << x.phase;
        s << ", " << x.shp_ampl;
        s << ", " << x.refpulse_smpl_frac;
        s << ", " << x.refpulse_smpl_cnt;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, sbl_dg_header<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.mean_lp_chn;
        s >> ',' >> x.stddev_lp_chn;
        s >> ',' >> x.mean_hp_chn;
        s >> ',' >> x.stddev_hp_chn;
        s >> ',' >> x.phase;
        s >> ',' >> x.shp_ampl;
        s >> ',' >> x.refpulse_smpl_frac;
        s >> ',' >> x.refpulse_smpl_cnt;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! scan pattern description

//! <para>This package belongs to the predefined selectors:</para>
//! <para>attribute, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct scan_rect_fov
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef scan_rect_fov<rebind_it> type;
    };

    typedef it iterator_type;

    scan_rect_fov(it begin, it end, bool dirty=false)
        : theta_min(begin)
        , theta_max(begin)
        , theta_incr(begin)
        , phi_min(begin)
        , phi_max(begin)
        , phi_incr(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 5, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 192};
    it begin() const { return theta_min.begin(); }
    it end() const { return phi_incr.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float theta_min;//!<  minimum of scan angle along line scan axis [deg] 
    #else
    field<float, sc_float32, 0, it> theta_min;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float theta_max;//!<  maximum of scan angle along line scan axis [deg] 
    #else
    field<float, sc_float32, 32, it> theta_max;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float theta_incr;//!<  nominal scan angle increment along line scan axis [deg] 
    #else
    field<float, sc_float32, 64, it> theta_incr;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float phi_min;//!<  minimum of scan angle along frame scan axis [deg] 
    #else
    field<float, sc_float32, 96, it> phi_min;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float phi_max;//!<  maximum of scan angle along frame scan axis [deg] 
    #else
    field<float, sc_float32, 128, it> phi_max;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float phi_incr;//!<  nominal scan angle increment along frame scan axis [deg] 
    #else
    field<float, sc_float32, 160, it> phi_incr;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    scan_rect_fov& operator=(const scan_rect_fov<ito>& o) {
        theta_min = o.theta_min;
        theta_max = o.theta_max;
        theta_incr = o.theta_incr;
        phi_min = o.phi_min;
        phi_max = o.phi_max;
        phi_incr = o.phi_incr;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct scan_rect_fov<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef scan_rect_fov<rebind_it> type;
    };

    enum { id_main = 5, id_sub = 0};

    float            theta_min;
    float            theta_max;
    float            theta_incr;
    float            phi_min;
    float            phi_max;
    float            phi_incr;

    scan_rect_fov() {}
    template<class it>
    scan_rect_fov(const scan_rect_fov<it>& o) {
        theta_min = o.theta_min;
        theta_max = o.theta_max;
        theta_incr = o.theta_incr;
        phi_min = o.phi_min;
        phi_max = o.phi_max;
        phi_incr = o.phi_incr;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const scan_rect_fov<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.theta_min;
        s << ", " << x.theta_max;
        s << ", " << x.theta_incr;
        s << ", " << x.phi_min;
        s << ", " << x.phi_max;
        s << ", " << x.phi_incr;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, scan_rect_fov<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.theta_min;
        s >> ',' >> x.theta_max;
        s >> ',' >> x.theta_incr;
        s >> ',' >> x.phi_min;
        s >> ',' >> x.phi_max;
        s >> ',' >> x.phi_incr;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! scanner pose (position and orientation

//! <para>This package belongs to the predefined selectors:</para>
//! <para>attribute, scan</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct scanner_pose
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef scanner_pose<rebind_it> type;
    };

    typedef it iterator_type;

    scanner_pose(it begin, it end, bool dirty=false)
        : LAT(begin)
        , LON(begin)
        , HEIGHT(begin)
        , HMSL(begin)
        , roll(begin)
        , pitch(begin)
        , yaw(begin)
        , hAcc(begin)
        , vAcc(begin)
        , rAcc(begin)
        , pAcc(begin)
        , yAcc(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 54, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 384};
    it begin() const { return LAT.begin(); }
    it end() const { return yAcc.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float LAT;//!<  Latitude [deg] 
    #else
    field<float, sc_float32, 0, it> LAT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float LON;//!<  Longitude [deg] 
    #else
    field<float, sc_float32, 32, it> LON;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float HEIGHT;//!<  height above ellipsoid [m] 
    #else
    field<float, sc_float32, 64, it> HEIGHT;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float HMSL;//!<  height above mean sea level [m] 
    #else
    field<float, sc_float32, 96, it> HMSL;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float roll;//!<  roll angle about scanner x-axis [deg] 
    #else
    field<float, sc_float32, 128, it> roll;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float pitch;//!<  pitch angle about scanner y-axis [deg] 
    #else
    field<float, sc_float32, 160, it> pitch;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float yaw;//!<  yaw angle about scanner z-axis [deg] 
    #else
    field<float, sc_float32, 192, it> yaw;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float hAcc;//!<  horizontal accuracy [m] 
    #else
    field<float, sc_float32, 224, it> hAcc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float vAcc;//!<  vertical accuracy [m] 
    #else
    field<float, sc_float32, 256, it> vAcc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float rAcc;//!<  roll angle accuracy [deg] 
    #else
    field<float, sc_float32, 288, it> rAcc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float pAcc;//!<  pitch angle accuracy [deg] 
    #else
    field<float, sc_float32, 320, it> pAcc;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float yAcc;//!<  yaw angle accuracy  [deg] 
    #else
    field<float, sc_float32, 352, it> yAcc;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    scanner_pose& operator=(const scanner_pose<ito>& o) {
        LAT = o.LAT;
        LON = o.LON;
        HEIGHT = o.HEIGHT;
        HMSL = o.HMSL;
        roll = o.roll;
        pitch = o.pitch;
        yaw = o.yaw;
        hAcc = o.hAcc;
        vAcc = o.vAcc;
        rAcc = o.rAcc;
        pAcc = o.pAcc;
        yAcc = o.yAcc;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct scanner_pose<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef scanner_pose<rebind_it> type;
    };

    enum { id_main = 54, id_sub = 0};

    float            LAT;
    float            LON;
    float            HEIGHT;
    float            HMSL;
    float            roll;
    float            pitch;
    float            yaw;
    float            hAcc;
    float            vAcc;
    float            rAcc;
    float            pAcc;
    float            yAcc;

    scanner_pose() {}
    template<class it>
    scanner_pose(const scanner_pose<it>& o) {
        LAT = o.LAT;
        LON = o.LON;
        HEIGHT = o.HEIGHT;
        HMSL = o.HMSL;
        roll = o.roll;
        pitch = o.pitch;
        yaw = o.yaw;
        hAcc = o.hAcc;
        vAcc = o.vAcc;
        rAcc = o.rAcc;
        pAcc = o.pAcc;
        yAcc = o.yAcc;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const scanner_pose<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.LAT;
        s << ", " << x.LON;
        s << ", " << x.HEIGHT;
        s << ", " << x.HMSL;
        s << ", " << x.roll;
        s << ", " << x.pitch;
        s << ", " << x.yaw;
        s << ", " << x.hAcc;
        s << ", " << x.vAcc;
        s << ", " << x.rAcc;
        s << ", " << x.pAcc;
        s << ", " << x.yAcc;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, scanner_pose<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.LAT;
        s >> ',' >> x.LON;
        s >> ',' >> x.HEIGHT;
        s >> ',' >> x.HMSL;
        s >> ',' >> x.roll;
        s >> ',' >> x.pitch;
        s >> ',' >> x.yaw;
        s >> ',' >> x.hAcc;
        s >> ',' >> x.vAcc;
        s >> ',' >> x.rAcc;
        s >> ',' >> x.pAcc;
        s >> ',' >> x.yAcc;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct slt_dg
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef slt_dg<rebind_it> type;
    };

    typedef it iterator_type;

    slt_dg(it begin, it end, bool dirty=false)
        : cnt_ls(begin)
        , fg_rp(begin)
        , ch_id(begin)
        , cnt_smpl_hi(begin)
        , cnt_smpl_lo(begin)
        , s_0(begin)
        , s_1(begin)
        , s_2(begin)
        , s_3(begin)
        , ampl(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 50000, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 112};
    it begin() const { return cnt_ls.begin(); }
    it end() const { return ampl.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_ls;
    #else
    field<uint8_t, sc_uint8, 0, it> cnt_ls;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t fg_rp;
    #else
    field<uint8_t, sc_bit, 8, it> fg_rp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t ch_id;
    #else
    field<uint8_t, sc_bit, 9, it> ch_id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_smpl_hi;
    #else
    field<uint8_t, sc_uint2, 14, it> cnt_smpl_hi;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t cnt_smpl_lo;
    #else
    field<uint16_t, sc_uint12, 16, it> cnt_smpl_lo;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_0;
    #else
    field<uint16_t, sc_uint16, 32, it> s_0;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_1;
    #else
    field<uint16_t, sc_uint16, 48, it> s_1;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_2;
    #else
    field<uint16_t, sc_uint16, 64, it> s_2;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_3;
    #else
    field<uint16_t, sc_uint16, 80, it> s_3;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t ampl;
    #else
    field<uint16_t, sc_uint16, 96, it> ampl;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    slt_dg& operator=(const slt_dg<ito>& o) {
        cnt_ls = o.cnt_ls;
        fg_rp = o.fg_rp;
        ch_id = o.ch_id;
        cnt_smpl_hi = o.cnt_smpl_hi;
        cnt_smpl_lo = o.cnt_smpl_lo;
        s_0 = o.s_0;
        s_1 = o.s_1;
        s_2 = o.s_2;
        s_3 = o.s_3;
        ampl = o.ampl;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct slt_dg<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef slt_dg<rebind_it> type;
    };

    enum { id_main = 50000, id_sub = 0};

    uint8_t          cnt_ls;
    uint8_t          fg_rp;
    uint8_t          ch_id;
    uint8_t          cnt_smpl_hi;
    uint16_t         cnt_smpl_lo;
    uint16_t         s_0;
    uint16_t         s_1;
    uint16_t         s_2;
    uint16_t         s_3;
    uint16_t         ampl;

    slt_dg() {}
    template<class it>
    slt_dg(const slt_dg<it>& o) {
        cnt_ls = o.cnt_ls;
        fg_rp = o.fg_rp;
        ch_id = o.ch_id;
        cnt_smpl_hi = o.cnt_smpl_hi;
        cnt_smpl_lo = o.cnt_smpl_lo;
        s_0 = o.s_0;
        s_1 = o.s_1;
        s_2 = o.s_2;
        s_3 = o.s_3;
        ampl = o.ampl;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const slt_dg<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.cnt_ls;
        s << ", " << x.fg_rp;
        s << ", " << x.ch_id;
        s << ", " << x.cnt_smpl_hi;
        s << ", " << x.cnt_smpl_lo;
        s << ", " << x.s_0;
        s << ", " << x.s_1;
        s << ", " << x.s_2;
        s << ", " << x.s_3;
        s << ", " << x.ampl;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, slt_dg<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.cnt_ls;
        s >> ',' >> x.fg_rp;
        s >> ',' >> x.ch_id;
        s >> ',' >> x.cnt_smpl_hi;
        s >> ',' >> x.cnt_smpl_lo;
        s >> ',' >> x.s_0;
        s >> ',' >> x.s_1;
        s >> ',' >> x.s_2;
        s >> ',' >> x.s_3;
        s >> ',' >> x.ampl;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct slt_dg_1
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef slt_dg_1<rebind_it> type;
    };

    typedef it iterator_type;

    slt_dg_1(it begin, it end, bool dirty=false)
        : cnt_ls(begin)
        , fg_rp(begin)
        , ch_id(begin)
        , cnt_smpl_hi(begin)
        , cnt_smpl_lo(begin)
        , s_0_lp(begin)
        , s_1_lp(begin)
        , s_2_lp(begin)
        , s_3_lp(begin)
        , s_4_lp(begin)
        , s_0_hp(begin)
        , s_1_hp(begin)
        , s_2_hp(begin)
        , s_3_hp(begin)
        , s_4_hp(begin)
        , est_ampl_shp(begin)
        , options(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 50007, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 224};
    it begin() const { return cnt_ls.begin(); }
    it end() const { return options.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_ls;
    #else
    field<uint8_t, sc_uint8, 0, it> cnt_ls;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t fg_rp;
    #else
    field<uint8_t, sc_bit, 8, it> fg_rp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t ch_id;
    #else
    field<uint8_t, sc_bit, 9, it> ch_id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_smpl_hi;
    #else
    field<uint8_t, sc_uint2, 14, it> cnt_smpl_hi;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t cnt_smpl_lo;
    #else
    field<uint16_t, sc_uint12, 16, it> cnt_smpl_lo;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_0_lp;
    #else
    field<uint16_t, sc_uint16, 32, it> s_0_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_1_lp;
    #else
    field<uint16_t, sc_uint16, 48, it> s_1_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_2_lp;
    #else
    field<uint16_t, sc_uint16, 64, it> s_2_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_3_lp;
    #else
    field<uint16_t, sc_uint16, 80, it> s_3_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_4_lp;
    #else
    field<uint16_t, sc_uint16, 96, it> s_4_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_0_hp;
    #else
    field<uint16_t, sc_uint16, 112, it> s_0_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_1_hp;
    #else
    field<uint16_t, sc_uint16, 128, it> s_1_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_2_hp;
    #else
    field<uint16_t, sc_uint16, 144, it> s_2_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_3_hp;
    #else
    field<uint16_t, sc_uint16, 160, it> s_3_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_4_hp;
    #else
    field<uint16_t, sc_uint16, 176, it> s_4_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t est_ampl_shp;
    #else
    field<uint16_t, sc_uint16, 192, it> est_ampl_shp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t options;
    #else
    field<uint16_t, sc_uint16, 208, it> options;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    slt_dg_1& operator=(const slt_dg_1<ito>& o) {
        cnt_ls = o.cnt_ls;
        fg_rp = o.fg_rp;
        ch_id = o.ch_id;
        cnt_smpl_hi = o.cnt_smpl_hi;
        cnt_smpl_lo = o.cnt_smpl_lo;
        s_0_lp = o.s_0_lp;
        s_1_lp = o.s_1_lp;
        s_2_lp = o.s_2_lp;
        s_3_lp = o.s_3_lp;
        s_4_lp = o.s_4_lp;
        s_0_hp = o.s_0_hp;
        s_1_hp = o.s_1_hp;
        s_2_hp = o.s_2_hp;
        s_3_hp = o.s_3_hp;
        s_4_hp = o.s_4_hp;
        est_ampl_shp = o.est_ampl_shp;
        options = o.options;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct slt_dg_1<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef slt_dg_1<rebind_it> type;
    };

    enum { id_main = 50007, id_sub = 0};

    uint8_t          cnt_ls;
    uint8_t          fg_rp;
    uint8_t          ch_id;
    uint8_t          cnt_smpl_hi;
    uint16_t         cnt_smpl_lo;
    uint16_t         s_0_lp;
    uint16_t         s_1_lp;
    uint16_t         s_2_lp;
    uint16_t         s_3_lp;
    uint16_t         s_4_lp;
    uint16_t         s_0_hp;
    uint16_t         s_1_hp;
    uint16_t         s_2_hp;
    uint16_t         s_3_hp;
    uint16_t         s_4_hp;
    uint16_t         est_ampl_shp;
    uint16_t         options;

    slt_dg_1() {}
    template<class it>
    slt_dg_1(const slt_dg_1<it>& o) {
        cnt_ls = o.cnt_ls;
        fg_rp = o.fg_rp;
        ch_id = o.ch_id;
        cnt_smpl_hi = o.cnt_smpl_hi;
        cnt_smpl_lo = o.cnt_smpl_lo;
        s_0_lp = o.s_0_lp;
        s_1_lp = o.s_1_lp;
        s_2_lp = o.s_2_lp;
        s_3_lp = o.s_3_lp;
        s_4_lp = o.s_4_lp;
        s_0_hp = o.s_0_hp;
        s_1_hp = o.s_1_hp;
        s_2_hp = o.s_2_hp;
        s_3_hp = o.s_3_hp;
        s_4_hp = o.s_4_hp;
        est_ampl_shp = o.est_ampl_shp;
        options = o.options;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const slt_dg_1<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.cnt_ls;
        s << ", " << x.fg_rp;
        s << ", " << x.ch_id;
        s << ", " << x.cnt_smpl_hi;
        s << ", " << x.cnt_smpl_lo;
        s << ", " << x.s_0_lp;
        s << ", " << x.s_1_lp;
        s << ", " << x.s_2_lp;
        s << ", " << x.s_3_lp;
        s << ", " << x.s_4_lp;
        s << ", " << x.s_0_hp;
        s << ", " << x.s_1_hp;
        s << ", " << x.s_2_hp;
        s << ", " << x.s_3_hp;
        s << ", " << x.s_4_hp;
        s << ", " << x.est_ampl_shp;
        s << ", " << x.options;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, slt_dg_1<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.cnt_ls;
        s >> ',' >> x.fg_rp;
        s >> ',' >> x.ch_id;
        s >> ',' >> x.cnt_smpl_hi;
        s >> ',' >> x.cnt_smpl_lo;
        s >> ',' >> x.s_0_lp;
        s >> ',' >> x.s_1_lp;
        s >> ',' >> x.s_2_lp;
        s >> ',' >> x.s_3_lp;
        s >> ',' >> x.s_4_lp;
        s >> ',' >> x.s_0_hp;
        s >> ',' >> x.s_1_hp;
        s >> ',' >> x.s_2_hp;
        s >> ',' >> x.s_3_hp;
        s >> ',' >> x.s_4_hp;
        s >> ',' >> x.est_ampl_shp;
        s >> ',' >> x.options;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct slt_dg_2
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef slt_dg_2<rebind_it> type;
    };

    typedef it iterator_type;

    slt_dg_2(it begin, it end, bool dirty=false)
        : cnt_ls(begin)
        , fg_rp(begin)
        , ch_id(begin)
        , cnt_smpl_hi(begin)
        , cnt_smpl_lo(begin)
        , s_0_lp(begin)
        , s_1_lp(begin)
        , s_2_lp(begin)
        , s_3_lp(begin)
        , s_4_lp(begin)
        , s_0_hp(begin)
        , s_1_hp(begin)
        , s_2_hp(begin)
        , s_3_hp(begin)
        , s_4_hp(begin)
        , est_ampl_shp(begin)
        , options(begin)
        , para_di(begin)
        , para_da(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 50007, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 256};
    it begin() const { return cnt_ls.begin(); }
    it end() const { return para_da.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_ls;
    #else
    field<uint8_t, sc_uint8, 0, it> cnt_ls;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t fg_rp;
    #else
    field<uint8_t, sc_bit, 8, it> fg_rp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t ch_id;
    #else
    field<uint8_t, sc_bit, 9, it> ch_id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_smpl_hi;
    #else
    field<uint8_t, sc_uint2, 14, it> cnt_smpl_hi;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t cnt_smpl_lo;
    #else
    field<uint16_t, sc_uint12, 16, it> cnt_smpl_lo;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_0_lp;
    #else
    field<uint16_t, sc_uint16, 32, it> s_0_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_1_lp;
    #else
    field<uint16_t, sc_uint16, 48, it> s_1_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_2_lp;
    #else
    field<uint16_t, sc_uint16, 64, it> s_2_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_3_lp;
    #else
    field<uint16_t, sc_uint16, 80, it> s_3_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_4_lp;
    #else
    field<uint16_t, sc_uint16, 96, it> s_4_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_0_hp;
    #else
    field<uint16_t, sc_uint16, 112, it> s_0_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_1_hp;
    #else
    field<uint16_t, sc_uint16, 128, it> s_1_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_2_hp;
    #else
    field<uint16_t, sc_uint16, 144, it> s_2_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_3_hp;
    #else
    field<uint16_t, sc_uint16, 160, it> s_3_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_4_hp;
    #else
    field<uint16_t, sc_uint16, 176, it> s_4_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t est_ampl_shp;
    #else
    field<uint16_t, sc_uint16, 192, it> est_ampl_shp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t options;
    #else
    field<uint16_t, sc_uint16, 208, it> options;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t para_di;
    #else
    field<uint16_t, sc_uint16, 224, it> para_di;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t para_da;
    #else
    field<uint16_t, sc_uint16, 240, it> para_da;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    slt_dg_2& operator=(const slt_dg_2<ito>& o) {
        cnt_ls = o.cnt_ls;
        fg_rp = o.fg_rp;
        ch_id = o.ch_id;
        cnt_smpl_hi = o.cnt_smpl_hi;
        cnt_smpl_lo = o.cnt_smpl_lo;
        s_0_lp = o.s_0_lp;
        s_1_lp = o.s_1_lp;
        s_2_lp = o.s_2_lp;
        s_3_lp = o.s_3_lp;
        s_4_lp = o.s_4_lp;
        s_0_hp = o.s_0_hp;
        s_1_hp = o.s_1_hp;
        s_2_hp = o.s_2_hp;
        s_3_hp = o.s_3_hp;
        s_4_hp = o.s_4_hp;
        est_ampl_shp = o.est_ampl_shp;
        options = o.options;
        para_di = o.para_di;
        para_da = o.para_da;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct slt_dg_2<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef slt_dg_2<rebind_it> type;
    };

    enum { id_main = 50007, id_sub = 1};

    uint8_t          cnt_ls;
    uint8_t          fg_rp;
    uint8_t          ch_id;
    uint8_t          cnt_smpl_hi;
    uint16_t         cnt_smpl_lo;
    uint16_t         s_0_lp;
    uint16_t         s_1_lp;
    uint16_t         s_2_lp;
    uint16_t         s_3_lp;
    uint16_t         s_4_lp;
    uint16_t         s_0_hp;
    uint16_t         s_1_hp;
    uint16_t         s_2_hp;
    uint16_t         s_3_hp;
    uint16_t         s_4_hp;
    uint16_t         est_ampl_shp;
    uint16_t         options;
    uint16_t         para_di;
    uint16_t         para_da;

    slt_dg_2() {}
    template<class it>
    slt_dg_2(const slt_dg_2<it>& o) {
        cnt_ls = o.cnt_ls;
        fg_rp = o.fg_rp;
        ch_id = o.ch_id;
        cnt_smpl_hi = o.cnt_smpl_hi;
        cnt_smpl_lo = o.cnt_smpl_lo;
        s_0_lp = o.s_0_lp;
        s_1_lp = o.s_1_lp;
        s_2_lp = o.s_2_lp;
        s_3_lp = o.s_3_lp;
        s_4_lp = o.s_4_lp;
        s_0_hp = o.s_0_hp;
        s_1_hp = o.s_1_hp;
        s_2_hp = o.s_2_hp;
        s_3_hp = o.s_3_hp;
        s_4_hp = o.s_4_hp;
        est_ampl_shp = o.est_ampl_shp;
        options = o.options;
        para_di = o.para_di;
        para_da = o.para_da;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const slt_dg_2<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.cnt_ls;
        s << ", " << x.fg_rp;
        s << ", " << x.ch_id;
        s << ", " << x.cnt_smpl_hi;
        s << ", " << x.cnt_smpl_lo;
        s << ", " << x.s_0_lp;
        s << ", " << x.s_1_lp;
        s << ", " << x.s_2_lp;
        s << ", " << x.s_3_lp;
        s << ", " << x.s_4_lp;
        s << ", " << x.s_0_hp;
        s << ", " << x.s_1_hp;
        s << ", " << x.s_2_hp;
        s << ", " << x.s_3_hp;
        s << ", " << x.s_4_hp;
        s << ", " << x.est_ampl_shp;
        s << ", " << x.options;
        s << ", " << x.para_di;
        s << ", " << x.para_da;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, slt_dg_2<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.cnt_ls;
        s >> ',' >> x.fg_rp;
        s >> ',' >> x.ch_id;
        s >> ',' >> x.cnt_smpl_hi;
        s >> ',' >> x.cnt_smpl_lo;
        s >> ',' >> x.s_0_lp;
        s >> ',' >> x.s_1_lp;
        s >> ',' >> x.s_2_lp;
        s >> ',' >> x.s_3_lp;
        s >> ',' >> x.s_4_lp;
        s >> ',' >> x.s_0_hp;
        s >> ',' >> x.s_1_hp;
        s >> ',' >> x.s_2_hp;
        s >> ',' >> x.s_3_hp;
        s >> ',' >> x.s_4_hp;
        s >> ',' >> x.est_ampl_shp;
        s >> ',' >> x.options;
        s >> ',' >> x.para_di;
        s >> ',' >> x.para_da;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct slt_dg_3
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef slt_dg_3<rebind_it> type;
    };

    typedef it iterator_type;

    slt_dg_3(it begin, it end, bool dirty=false)
        : cnt_ls(begin)
        , fg_rp(begin)
        , ch_id(begin)
        , cnt_smpl_hi(begin)
        , cnt_smpl_lo(begin)
        , s_0_lp(begin)
        , s_1_lp(begin)
        , s_2_lp(begin)
        , s_3_lp(begin)
        , s_4_lp(begin)
        , s_0_hp(begin)
        , s_1_hp(begin)
        , s_2_hp(begin)
        , s_3_hp(begin)
        , s_4_hp(begin)
        , est_ampl_shp(begin)
        , options(begin)
        , para_di(begin)
        , para_da(begin)
        , offset_lp(begin)
        , offset_hp(begin)
        , sigma_lp(begin)
        , sigma_hp(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 50007, id_sub = 2};
    #ifndef DOXYGEN
    enum { max_bit_width = 320};
    it begin() const { return cnt_ls.begin(); }
    it end() const { return sigma_hp.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_ls;
    #else
    field<uint8_t, sc_uint8, 0, it> cnt_ls;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t fg_rp;
    #else
    field<uint8_t, sc_bit, 8, it> fg_rp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t ch_id;
    #else
    field<uint8_t, sc_bit, 9, it> ch_id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_smpl_hi;
    #else
    field<uint8_t, sc_uint2, 14, it> cnt_smpl_hi;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t cnt_smpl_lo;
    #else
    field<uint16_t, sc_uint12, 16, it> cnt_smpl_lo;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_0_lp;
    #else
    field<uint16_t, sc_uint16, 32, it> s_0_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_1_lp;
    #else
    field<uint16_t, sc_uint16, 48, it> s_1_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_2_lp;
    #else
    field<uint16_t, sc_uint16, 64, it> s_2_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_3_lp;
    #else
    field<uint16_t, sc_uint16, 80, it> s_3_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_4_lp;
    #else
    field<uint16_t, sc_uint16, 96, it> s_4_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_0_hp;
    #else
    field<uint16_t, sc_uint16, 112, it> s_0_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_1_hp;
    #else
    field<uint16_t, sc_uint16, 128, it> s_1_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_2_hp;
    #else
    field<uint16_t, sc_uint16, 144, it> s_2_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_3_hp;
    #else
    field<uint16_t, sc_uint16, 160, it> s_3_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t s_4_hp;
    #else
    field<uint16_t, sc_uint16, 176, it> s_4_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t est_ampl_shp;
    #else
    field<uint16_t, sc_uint16, 192, it> est_ampl_shp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t options;
    #else
    field<uint16_t, sc_uint16, 208, it> options;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t para_di;
    #else
    field<uint16_t, sc_uint16, 224, it> para_di;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t para_da;
    #else
    field<uint16_t, sc_uint16, 240, it> para_da;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t offset_lp;
    #else
    field<uint16_t, sc_uint16, 256, it> offset_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t offset_hp;
    #else
    field<uint16_t, sc_uint16, 272, it> offset_hp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t sigma_lp;
    #else
    field<uint16_t, sc_uint16, 288, it> sigma_lp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t sigma_hp;
    #else
    field<uint16_t, sc_uint16, 304, it> sigma_hp;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    slt_dg_3& operator=(const slt_dg_3<ito>& o) {
        cnt_ls = o.cnt_ls;
        fg_rp = o.fg_rp;
        ch_id = o.ch_id;
        cnt_smpl_hi = o.cnt_smpl_hi;
        cnt_smpl_lo = o.cnt_smpl_lo;
        s_0_lp = o.s_0_lp;
        s_1_lp = o.s_1_lp;
        s_2_lp = o.s_2_lp;
        s_3_lp = o.s_3_lp;
        s_4_lp = o.s_4_lp;
        s_0_hp = o.s_0_hp;
        s_1_hp = o.s_1_hp;
        s_2_hp = o.s_2_hp;
        s_3_hp = o.s_3_hp;
        s_4_hp = o.s_4_hp;
        est_ampl_shp = o.est_ampl_shp;
        options = o.options;
        para_di = o.para_di;
        para_da = o.para_da;
        offset_lp = o.offset_lp;
        offset_hp = o.offset_hp;
        sigma_lp = o.sigma_lp;
        sigma_hp = o.sigma_hp;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct slt_dg_3<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef slt_dg_3<rebind_it> type;
    };

    enum { id_main = 50007, id_sub = 2};

    uint8_t          cnt_ls;
    uint8_t          fg_rp;
    uint8_t          ch_id;
    uint8_t          cnt_smpl_hi;
    uint16_t         cnt_smpl_lo;
    uint16_t         s_0_lp;
    uint16_t         s_1_lp;
    uint16_t         s_2_lp;
    uint16_t         s_3_lp;
    uint16_t         s_4_lp;
    uint16_t         s_0_hp;
    uint16_t         s_1_hp;
    uint16_t         s_2_hp;
    uint16_t         s_3_hp;
    uint16_t         s_4_hp;
    uint16_t         est_ampl_shp;
    uint16_t         options;
    uint16_t         para_di;
    uint16_t         para_da;
    uint16_t         offset_lp;
    uint16_t         offset_hp;
    uint16_t         sigma_lp;
    uint16_t         sigma_hp;

    slt_dg_3() {}
    template<class it>
    slt_dg_3(const slt_dg_3<it>& o) {
        cnt_ls = o.cnt_ls;
        fg_rp = o.fg_rp;
        ch_id = o.ch_id;
        cnt_smpl_hi = o.cnt_smpl_hi;
        cnt_smpl_lo = o.cnt_smpl_lo;
        s_0_lp = o.s_0_lp;
        s_1_lp = o.s_1_lp;
        s_2_lp = o.s_2_lp;
        s_3_lp = o.s_3_lp;
        s_4_lp = o.s_4_lp;
        s_0_hp = o.s_0_hp;
        s_1_hp = o.s_1_hp;
        s_2_hp = o.s_2_hp;
        s_3_hp = o.s_3_hp;
        s_4_hp = o.s_4_hp;
        est_ampl_shp = o.est_ampl_shp;
        options = o.options;
        para_di = o.para_di;
        para_da = o.para_da;
        offset_lp = o.offset_lp;
        offset_hp = o.offset_hp;
        sigma_lp = o.sigma_lp;
        sigma_hp = o.sigma_hp;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const slt_dg_3<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.cnt_ls;
        s << ", " << x.fg_rp;
        s << ", " << x.ch_id;
        s << ", " << x.cnt_smpl_hi;
        s << ", " << x.cnt_smpl_lo;
        s << ", " << x.s_0_lp;
        s << ", " << x.s_1_lp;
        s << ", " << x.s_2_lp;
        s << ", " << x.s_3_lp;
        s << ", " << x.s_4_lp;
        s << ", " << x.s_0_hp;
        s << ", " << x.s_1_hp;
        s << ", " << x.s_2_hp;
        s << ", " << x.s_3_hp;
        s << ", " << x.s_4_hp;
        s << ", " << x.est_ampl_shp;
        s << ", " << x.options;
        s << ", " << x.para_di;
        s << ", " << x.para_da;
        s << ", " << x.offset_lp;
        s << ", " << x.offset_hp;
        s << ", " << x.sigma_lp;
        s << ", " << x.sigma_hp;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, slt_dg_3<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.cnt_ls;
        s >> ',' >> x.fg_rp;
        s >> ',' >> x.ch_id;
        s >> ',' >> x.cnt_smpl_hi;
        s >> ',' >> x.cnt_smpl_lo;
        s >> ',' >> x.s_0_lp;
        s >> ',' >> x.s_1_lp;
        s >> ',' >> x.s_2_lp;
        s >> ',' >> x.s_3_lp;
        s >> ',' >> x.s_4_lp;
        s >> ',' >> x.s_0_hp;
        s >> ',' >> x.s_1_hp;
        s >> ',' >> x.s_2_hp;
        s >> ',' >> x.s_3_hp;
        s >> ',' >> x.s_4_hp;
        s >> ',' >> x.est_ampl_shp;
        s >> ',' >> x.options;
        s >> ',' >> x.para_di;
        s >> ',' >> x.para_da;
        s >> ',' >> x.offset_lp;
        s >> ',' >> x.offset_hp;
        s >> ',' >> x.sigma_lp;
        s >> ',' >> x.sigma_hp;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct tgt_dg
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef tgt_dg<rebind_it> type;
    };

    typedef it iterator_type;

    tgt_dg(it begin, it end, bool dirty=false)
        : cnt_ls(begin)
        , fg_rp(begin)
        , ch_id(begin)
        , cnt_smpl_hi(begin)
        , cnt_smpl_lo(begin)
        , del_r(begin)
        , del_parabola(begin)
        , ampl_r(begin)
        , ampl_parabola(begin)
        , flags(begin)
        , dev(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 50001, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 128};
    it begin() const { return cnt_ls.begin(); }
    it end() const { return dev.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_ls;
    #else
    field<uint8_t, sc_uint8, 0, it> cnt_ls;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t fg_rp;
    #else
    field<uint8_t, sc_bit, 8, it> fg_rp;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t ch_id;
    #else
    field<uint8_t, sc_bit, 9, it> ch_id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_smpl_hi;
    #else
    field<uint8_t, sc_uint2, 14, it> cnt_smpl_hi;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t cnt_smpl_lo;
    #else
    field<uint16_t, sc_uint12, 16, it> cnt_smpl_lo;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t del_r;
    #else
    field<uint16_t, sc_uint16, 32, it> del_r;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t del_parabola;
    #else
    field<uint16_t, sc_uint16, 48, it> del_parabola;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t ampl_r;
    #else
    field<uint16_t, sc_uint16, 64, it> ampl_r;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t ampl_parabola;
    #else
    field<uint16_t, sc_uint16, 80, it> ampl_parabola;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t flags;
    #else
    field<uint16_t, sc_uint16, 96, it> flags;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t dev;
    #else
    field<uint16_t, sc_uint16, 112, it> dev;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    tgt_dg& operator=(const tgt_dg<ito>& o) {
        cnt_ls = o.cnt_ls;
        fg_rp = o.fg_rp;
        ch_id = o.ch_id;
        cnt_smpl_hi = o.cnt_smpl_hi;
        cnt_smpl_lo = o.cnt_smpl_lo;
        del_r = o.del_r;
        del_parabola = o.del_parabola;
        ampl_r = o.ampl_r;
        ampl_parabola = o.ampl_parabola;
        flags = o.flags;
        dev = o.dev;
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct tgt_dg<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef tgt_dg<rebind_it> type;
    };

    enum { id_main = 50001, id_sub = 0};

    uint8_t          cnt_ls;
    uint8_t          fg_rp;
    uint8_t          ch_id;
    uint8_t          cnt_smpl_hi;
    uint16_t         cnt_smpl_lo;
    uint16_t         del_r;
    uint16_t         del_parabola;
    uint16_t         ampl_r;
    uint16_t         ampl_parabola;
    uint16_t         flags;
    uint16_t         dev;

    tgt_dg() {}
    template<class it>
    tgt_dg(const tgt_dg<it>& o) {
        cnt_ls = o.cnt_ls;
        fg_rp = o.fg_rp;
        ch_id = o.ch_id;
        cnt_smpl_hi = o.cnt_smpl_hi;
        cnt_smpl_lo = o.cnt_smpl_lo;
        del_r = o.del_r;
        del_parabola = o.del_parabola;
        ampl_r = o.ampl_r;
        ampl_parabola = o.ampl_parabola;
        flags = o.flags;
        dev = o.dev;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const tgt_dg<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.cnt_ls;
        s << ", " << x.fg_rp;
        s << ", " << x.ch_id;
        s << ", " << x.cnt_smpl_hi;
        s << ", " << x.cnt_smpl_lo;
        s << ", " << x.del_r;
        s << ", " << x.del_parabola;
        s << ", " << x.ampl_r;
        s << ", " << x.ampl_parabola;
        s << ", " << x.flags;
        s << ", " << x.dev;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, tgt_dg<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.cnt_ls;
        s >> ',' >> x.fg_rp;
        s >> ',' >> x.ch_id;
        s >> ',' >> x.cnt_smpl_hi;
        s >> ',' >> x.cnt_smpl_lo;
        s >> ',' >> x.del_r;
        s >> ',' >> x.del_parabola;
        s >> ',' >> x.ampl_r;
        s >> ',' >> x.ampl_parabola;
        s >> ',' >> x.flags;
        s >> ',' >> x.dev;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct ublox_lea5t_rxm
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef ublox_lea5t_rxm<rebind_it> type;
    };

    typedef it iterator_type;

    ublox_lea5t_rxm(it begin, it end, bool dirty=false)
        : systime(begin)
        , iTOW(begin)
        , week(begin)
        , numSV(begin)
        , sv_info(begin, end, sv_info_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 59999, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 3936};
    it begin() const { return systime.begin(); }
    it end() const { return sv_info.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int32_t iTOW;
    #else
    field<int32_t, sc_int32, 32, it> iTOW;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int16_t week;
    #else
    field<int16_t, sc_int16, 64, it> week;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t numSV;
    #else
    field<uint8_t, sc_uint8, 80, it> numSV;
    #endif //DOXYGEN

    std::size_t sv_info_size;
    enum { sv_info_max_size = 20 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : cpMes(begin, begin_bit)
            , prMes(begin, begin_bit)
            , doMes(begin, begin_bit)
            , sv(begin, begin_bit)
            , mesQI(begin, begin_bit)
            , cno(begin, begin_bit)
            , lli(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        double cpMes;
        #else
        field<double, sc_float64, 0, it> cpMes;
        #endif
        #ifdef DOXYGEN
        double prMes;
        #else
        field<double, sc_float64, 64, it> prMes;
        #endif
        #ifdef DOXYGEN
        float doMes;
        #else
        field<float, sc_float32, 128, it> doMes;
        #endif
        #ifdef DOXYGEN
        uint8_t sv;
        #else
        field<uint8_t, sc_uint8, 160, it> sv;
        #endif
        #ifdef DOXYGEN
        int8_t mesQI;
        #else
        field<int8_t, sc_int8, 168, it> mesQI;
        #endif
        #ifdef DOXYGEN
        int8_t cno;
        #else
        field<int8_t, sc_int8, 176, it> cno;
        #endif
        #ifdef DOXYGEN
        uint8_t lli;
        #else
        field<uint8_t, sc_uint8, 184, it> lli;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition sv_info[20];
    #else
    sequence<ublox_lea5t_rxm, 192, 96, it> sv_info;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    ublox_lea5t_rxm& operator=(const ublox_lea5t_rxm<ito>& o) {
        systime = o.systime;
        iTOW = o.iTOW;
        week = o.week;
        numSV = o.numSV;
        sv_info_size = o.sv_info_size;
        for(unsigned n=0; n<sv_info_size; ++n){
            sv_info[n].cpMes = o.sv_info[n].cpMes;
            sv_info[n].prMes = o.sv_info[n].prMes;
            sv_info[n].doMes = o.sv_info[n].doMes;
            sv_info[n].sv = o.sv_info[n].sv;
            sv_info[n].mesQI = o.sv_info[n].mesQI;
            sv_info[n].cno = o.sv_info[n].cno;
            sv_info[n].lli = o.sv_info[n].lli;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct ublox_lea5t_rxm<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef ublox_lea5t_rxm<rebind_it> type;
    };

    enum { id_main = 59999, id_sub = 0};

    uint32_t         systime;
    int32_t          iTOW;
    int16_t          week;
    uint8_t          numSV;
    std::size_t sv_info_size;
    enum { sv_info_max_size = 20 };
    struct sequence_definition {
        double           cpMes;
        double           prMes;
        float            doMes;
        uint8_t          sv;
        int8_t           mesQI;
        int8_t           cno;
        uint8_t          lli;
    } sv_info[20];

    ublox_lea5t_rxm() {}
    template<class it>
    ublox_lea5t_rxm(const ublox_lea5t_rxm<it>& o) {
        systime = o.systime;
        iTOW = o.iTOW;
        week = o.week;
        numSV = o.numSV;
        sv_info_size = o.sv_info.size();
        for(unsigned n=0; n<sv_info_size; ++n){
            sv_info[n].cpMes = o.sv_info[n].cpMes;
            sv_info[n].prMes = o.sv_info[n].prMes;
            sv_info[n].doMes = o.sv_info[n].doMes;
            sv_info[n].sv = o.sv_info[n].sv;
            sv_info[n].mesQI = o.sv_info[n].mesQI;
            sv_info[n].cno = o.sv_info[n].cno;
            sv_info[n].lli = o.sv_info[n].lli;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const ublox_lea5t_rxm<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.iTOW;
        s << ", " << x.week;
        s << ", " << x.numSV;

        s << ", [";
        for (std::size_t n=0; n<x.sv_info_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.sv_info[n].cpMes
            << ", " << x.sv_info[n].prMes
            << ", " << x.sv_info[n].doMes
            << ", " << x.sv_info[n].sv
            << ", " << x.sv_info[n].mesQI
            << ", " << x.sv_info[n].cno
            << ", " << x.sv_info[n].lli
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, ublox_lea5t_rxm<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.iTOW;
        s >> ',' >> x.week;
        s >> ',' >> x.numSV;

        s >> ',' >> '[' >> std::ws;
        x.sv_info_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.sv_info_size;
            if (s
                >> '{' >> (x.sv_info[x.sv_info_size-1].cpMes)
                >> ',' >> (x.sv_info[x.sv_info_size-1].prMes)
                >> ',' >> (x.sv_info[x.sv_info_size-1].doMes)
                >> ',' >> (x.sv_info[x.sv_info_size-1].sv)
                >> ',' >> (x.sv_info[x.sv_info_size-1].mesQI)
                >> ',' >> (x.sv_info[x.sv_info_size-1].cno)
                >> ',' >> (x.sv_info[x.sv_info_size-1].lli)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct ublox_lea5t_rxm_sfrb
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef ublox_lea5t_rxm_sfrb<rebind_it> type;
    };

    typedef it iterator_type;

    ublox_lea5t_rxm_sfrb(it begin, it end, bool dirty=false)
        : systime(begin)
        , chn(begin)
        , svid(begin)
        , dwrd(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 59998, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 1328};
    it begin() const { return systime.begin(); }
    it end() const { return dwrd.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t chn;
    #else
    field<uint8_t, sc_uint8, 32, it> chn;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t svid;
    #else
    field<uint8_t, sc_uint8, 40, it> svid;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t dwrd[40];
    #else
    array<uint32_t, 40, sc_uint32, 48, it> dwrd;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    ublox_lea5t_rxm_sfrb& operator=(const ublox_lea5t_rxm_sfrb<ito>& o) {
        systime = o.systime;
        chn = o.chn;
        svid = o.svid;
        for(unsigned n=0; n<40; ++n) dwrd[n] = o.dwrd[n];
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct ublox_lea5t_rxm_sfrb<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef ublox_lea5t_rxm_sfrb<rebind_it> type;
    };

    enum { id_main = 59998, id_sub = 0};

    uint32_t         systime;
    uint8_t          chn;
    uint8_t          svid;
    uint32_t         dwrd[40];

    ublox_lea5t_rxm_sfrb() {}
    template<class it>
    ublox_lea5t_rxm_sfrb(const ublox_lea5t_rxm_sfrb<it>& o) {
        systime = o.systime;
        chn = o.chn;
        svid = o.svid;
        for(unsigned n=0; n<40; ++n) dwrd[n] = o.dwrd[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const ublox_lea5t_rxm_sfrb<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.chn;
        s << ", " << x.svid;
        s << ", "; write_array(s, 40, x.dwrd);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, ublox_lea5t_rxm_sfrb<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.chn;
        s >> ',' >> x.svid;
        s >> ','; read_array(s, 40, x.dwrd);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! units information

//! <para>This package belongs to the predefined selectors:</para>
//! <para>attribute, instrument</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct units
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef units<rebind_it> type;
    };

    typedef it iterator_type;

    units(it begin, it end, bool dirty=false)
        : range_unit(begin)
        , line_circle_count(begin)
        , frame_circle_count(begin)
        , time_unit(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 2, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 128};
    it begin() const { return range_unit.begin(); }
    it end() const { return time_unit.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float range_unit;//!<  length of 1 LSB of range in meter 
    #else
    field<float, sc_float32, 0, it> range_unit;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_circle_count;//!<  number of LSBs per full rotation about line axis 
    #else
    field<uint32_t, sc_uint32, 32, it> line_circle_count;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_circle_count;//!<  number of LSBs per full rotation about frame axis 
    #else
    field<uint32_t, sc_uint32, 64, it> frame_circle_count;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float time_unit;//!<  duration of 1 LSB of time in seconds 
    #else
    field<float, sc_float32, 96, it> time_unit;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    units& operator=(const units<ito>& o) {
        range_unit = o.range_unit;
        line_circle_count = o.line_circle_count;
        frame_circle_count = o.frame_circle_count;
        time_unit = o.time_unit;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct units<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef units<rebind_it> type;
    };

    enum { id_main = 2, id_sub = 0};

    float            range_unit;
    uint32_t         line_circle_count;
    uint32_t         frame_circle_count;
    float            time_unit;

    units() {}
    template<class it>
    units(const units<it>& o) {
        range_unit = o.range_unit;
        line_circle_count = o.line_circle_count;
        frame_circle_count = o.frame_circle_count;
        time_unit = o.time_unit;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const units<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.range_unit;
        s << ", " << x.line_circle_count;
        s << ", " << x.frame_circle_count;
        s << ", " << x.time_unit;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, units<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.range_unit;
        s >> ',' >> x.line_circle_count;
        s >> ',' >> x.frame_circle_count;
        s >> ',' >> x.time_unit;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! units information

//! <para>This package belongs to the predefined selectors:</para>
//! <para>attribute, instrument</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct units_1
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef units_1<rebind_it> type;
    };

    typedef it iterator_type;

    units_1(it begin, it end, bool dirty=false)
        : range_unit(begin)
        , line_circle_count(begin)
        , frame_circle_count(begin)
        , time_unit(begin)
        , amplitude_unit(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 2, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 160};
    it begin() const { return range_unit.begin(); }
    it end() const { return amplitude_unit.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    float range_unit;//!<  length of 1 LSB of range in meter 
    #else
    field<float, sc_float32, 0, it> range_unit;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t line_circle_count;//!<  number of LSBs per full rotation about line axis 
    #else
    field<uint32_t, sc_uint32, 32, it> line_circle_count;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t frame_circle_count;//!<  number of LSBs per full rotation about frame axis 
    #else
    field<uint32_t, sc_uint32, 64, it> frame_circle_count;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float time_unit;//!<  duration of 1 LSB of time in [sec] 
    #else
    field<float, sc_float32, 96, it> time_unit;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    float amplitude_unit;//!<  width of 1 LSB of amplitude and reflectance in dB 
    #else
    field<float, sc_float32, 128, it> amplitude_unit;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    units_1& operator=(const units_1<ito>& o) {
        range_unit = o.range_unit;
        line_circle_count = o.line_circle_count;
        frame_circle_count = o.frame_circle_count;
        time_unit = o.time_unit;
        amplitude_unit = o.amplitude_unit;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct units_1<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef units_1<rebind_it> type;
    };

    enum { id_main = 2, id_sub = 1};

    float            range_unit;
    uint32_t         line_circle_count;
    uint32_t         frame_circle_count;
    float            time_unit;
    float            amplitude_unit;

    units_1() {}
    template<class it>
    units_1(const units_1<it>& o) {
        range_unit = o.range_unit;
        line_circle_count = o.line_circle_count;
        frame_circle_count = o.frame_circle_count;
        time_unit = o.time_unit;
        amplitude_unit = o.amplitude_unit;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const units_1<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.range_unit;
        s << ", " << x.line_circle_count;
        s << ", " << x.frame_circle_count;
        s << ", " << x.time_unit;
        s << ", " << x.amplitude_unit;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, units_1<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.range_unit;
        s >> ',' >> x.line_circle_count;
        s >> ',' >> x.frame_circle_count;
        s >> ',' >> x.time_unit;
        s >> ',' >> x.amplitude_unit;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! spontaneous information and error messages

//! <para>This package belongs to the predefined selectors:</para>
//! <para>notify</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct unsolicited_message
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef unsolicited_message<rebind_it> type;
    };

    typedef it iterator_type;

    unsolicited_message(it begin, it end, bool dirty=false)
        : systime(begin)
        , year(begin)
        , month(begin)
        , day(begin)
        , hour(begin)
        , minute(begin)
        , second(begin)
        , utc_offset(begin)
        , type(begin)
        , intended_followup_actions(begin)
        , id(begin)
        , message(begin)
        , RESERVED_00(begin)
        , RESERVED_01(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 41, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 1736};
    it begin() const { return systime.begin(); }
    it end() const { return RESERVED_01.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  systime of message in units of uints.time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t year;//!<  year 
    #else
    field<uint16_t, sc_uint16, 32, it> year;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t month;//!<  month  1 .. 12 
    #else
    field<uint8_t, sc_uint8, 48, it> month;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t day;//!<  day    1 .. 31 
    #else
    field<uint8_t, sc_uint8, 56, it> day;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t hour;//!<  hour   0 .. 23 
    #else
    field<uint8_t, sc_uint8, 64, it> hour;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t minute;//!<  minute 0 .. 59 
    #else
    field<uint8_t, sc_uint8, 72, it> minute;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t second;//!<  second 0 .. 59 (60 if leap second) 
    #else
    field<uint8_t, sc_uint8, 80, it> second;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t utc_offset;//!<  diff. between localtime and UTC 
    #else
    field<int8_t, sc_int8, 88, it> utc_offset;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t type;//!<  (INFO=1,WARNING=2,ERROR=3,FATAL=4) 
    #else
    field<uint8_t, sc_uint8, 96, it> type;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t intended_followup_actions;//!<  bitmapped actions (1 .. shutting down laser power supply                     2 .. stopping current scan                     4 .. aborting current scan                     8 .. initiating shutdown ) 
    #else
    field<uint32_t, sc_uint32, 104, it> intended_followup_actions;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t id;//!<  unique (error) number 
    #else
    field<uint32_t, sc_uint32, 136, it> id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char message[128];//!<  displayable user info 
    #else
    array<char, 128, sc_char, 168, it> message;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_00;
    #else
    field<uint32_t, sc_uint32, 1192, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char RESERVED_01[64];
    #else
    array<char, 64, sc_char, 1224, it> RESERVED_01;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    unsolicited_message& operator=(const unsolicited_message<ito>& o) {
        systime = o.systime;
        year = o.year;
        month = o.month;
        day = o.day;
        hour = o.hour;
        minute = o.minute;
        second = o.second;
        utc_offset = o.utc_offset;
        type = o.type;
        intended_followup_actions = o.intended_followup_actions;
        id = o.id;
        for(unsigned n=0; n<128; ++n) message[n] = o.message[n];
        RESERVED_00 = o.RESERVED_00;
        for(unsigned n=0; n<64; ++n) RESERVED_01[n] = o.RESERVED_01[n];
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct unsolicited_message<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef unsolicited_message<rebind_it> type;
    };

    enum { id_main = 41, id_sub = 0};

    uint32_t         systime;
    uint16_t         year;
    uint8_t          month;
    uint8_t          day;
    uint8_t          hour;
    uint8_t          minute;
    uint8_t          second;
    int8_t           utc_offset;
    uint8_t          type;
    uint32_t         intended_followup_actions;
    uint32_t         id;
    char             message[128];
    uint32_t         RESERVED_00;
    char             RESERVED_01[64];

    unsolicited_message() {}
    template<class it>
    unsolicited_message(const unsolicited_message<it>& o) {
        systime = o.systime;
        year = o.year;
        month = o.month;
        day = o.day;
        hour = o.hour;
        minute = o.minute;
        second = o.second;
        utc_offset = o.utc_offset;
        type = o.type;
        intended_followup_actions = o.intended_followup_actions;
        id = o.id;
        for(unsigned n=0; n<128; ++n) message[n] = o.message[n];
        RESERVED_00 = o.RESERVED_00;
        for(unsigned n=0; n<64; ++n) RESERVED_01[n] = o.RESERVED_01[n];
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const unsolicited_message<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.year;
        s << ", " << x.month;
        s << ", " << x.day;
        s << ", " << x.hour;
        s << ", " << x.minute;
        s << ", " << x.second;
        s << ", " << x.utc_offset;
        s << ", " << x.type;
        s << ", " << x.intended_followup_actions;
        s << ", " << x.id;
        s << ", "; write_array(s, 128, x.message);
        s << ", " << x.RESERVED_00;
        s << ", "; write_array(s, 64, x.RESERVED_01);
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, unsolicited_message<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.year;
        s >> ',' >> x.month;
        s >> ',' >> x.day;
        s >> ',' >> x.hour;
        s >> ',' >> x.minute;
        s >> ',' >> x.second;
        s >> ',' >> x.utc_offset;
        s >> ',' >> x.type;
        s >> ',' >> x.intended_followup_actions;
        s >> ',' >> x.id;
        s >> ','; read_array(s, 128, x.message);
        s >> ',' >> x.RESERVED_00;
        s >> ','; read_array(s, 64, x.RESERVED_01);
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! spontaneous information and error messages

//! <para>This package belongs to the predefined selectors:</para>
//! <para>notify</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct unsolicited_message_1
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef unsolicited_message_1<rebind_it> type;
    };

    typedef it iterator_type;

    unsolicited_message_1(it begin, it end, bool dirty=false)
        : systime(begin)
        , year(begin)
        , month(begin)
        , day(begin)
        , hour(begin)
        , minute(begin)
        , second(begin)
        , utc_offset(begin)
        , type(begin)
        , intended_followup_actions(begin)
        , id(begin)
        , message(begin)
        , RESERVED_00(begin)
        , RESERVED_01(begin)
        , RESERVED_02(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 41, id_sub = 1};
    #ifndef DOXYGEN
    enum { max_bit_width = 1768};
    it begin() const { return systime.begin(); }
    it end() const { return RESERVED_02.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t systime;//!<  systime of message in units of uints.time_unit 
    #else
    field<uint32_t, sc_uint32, 0, it> systime;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t year;//!<  year 
    #else
    field<uint16_t, sc_uint16, 32, it> year;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t month;//!<  month  1 .. 12 
    #else
    field<uint8_t, sc_uint8, 48, it> month;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t day;//!<  day    1 .. 31 
    #else
    field<uint8_t, sc_uint8, 56, it> day;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t hour;//!<  hour   0 .. 23 
    #else
    field<uint8_t, sc_uint8, 64, it> hour;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t minute;//!<  minute 0 .. 59 
    #else
    field<uint8_t, sc_uint8, 72, it> minute;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t second;//!<  second 0 .. 59 (60 if leap second) 
    #else
    field<uint8_t, sc_uint8, 80, it> second;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    int8_t utc_offset;//!<  diff. between localtime and UTC 
    #else
    field<int8_t, sc_int8, 88, it> utc_offset;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t type;//!<  (INFO=1,WARNING=2,ERROR=3,FATAL=4) 
    #else
    field<uint8_t, sc_uint8, 96, it> type;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t intended_followup_actions;//!<  bitmapped actions (1 .. shutting down laser power supply                     2 .. stopping current scan                     4 .. aborting current scan                     8 .. initiating shutdown ) 
    #else
    field<uint32_t, sc_uint32, 104, it> intended_followup_actions;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t id;//!<  unique (error) number 
    #else
    field<uint32_t, sc_uint32, 136, it> id;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char message[128];//!<  displayable user info 
    #else
    array<char, 128, sc_char, 168, it> message;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_00;
    #else
    field<uint32_t, sc_uint32, 1192, it> RESERVED_00;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    char RESERVED_01[64];
    #else
    array<char, 64, sc_char, 1224, it> RESERVED_01;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint32_t RESERVED_02;
    #else
    field<uint32_t, sc_uint32, 1736, it> RESERVED_02;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    unsolicited_message_1& operator=(const unsolicited_message_1<ito>& o) {
        systime = o.systime;
        year = o.year;
        month = o.month;
        day = o.day;
        hour = o.hour;
        minute = o.minute;
        second = o.second;
        utc_offset = o.utc_offset;
        type = o.type;
        intended_followup_actions = o.intended_followup_actions;
        id = o.id;
        for(unsigned n=0; n<128; ++n) message[n] = o.message[n];
        RESERVED_00 = o.RESERVED_00;
        for(unsigned n=0; n<64; ++n) RESERVED_01[n] = o.RESERVED_01[n];
        RESERVED_02 = o.RESERVED_02;
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct unsolicited_message_1<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef unsolicited_message_1<rebind_it> type;
    };

    enum { id_main = 41, id_sub = 1};

    uint32_t         systime;
    uint16_t         year;
    uint8_t          month;
    uint8_t          day;
    uint8_t          hour;
    uint8_t          minute;
    uint8_t          second;
    int8_t           utc_offset;
    uint8_t          type;
    uint32_t         intended_followup_actions;
    uint32_t         id;
    char             message[128];
    uint32_t         RESERVED_00;
    char             RESERVED_01[64];
    uint32_t         RESERVED_02;

    unsolicited_message_1() {}
    template<class it>
    unsolicited_message_1(const unsolicited_message_1<it>& o) {
        systime = o.systime;
        year = o.year;
        month = o.month;
        day = o.day;
        hour = o.hour;
        minute = o.minute;
        second = o.second;
        utc_offset = o.utc_offset;
        type = o.type;
        intended_followup_actions = o.intended_followup_actions;
        id = o.id;
        for(unsigned n=0; n<128; ++n) message[n] = o.message[n];
        RESERVED_00 = o.RESERVED_00;
        for(unsigned n=0; n<64; ++n) RESERVED_01[n] = o.RESERVED_01[n];
        RESERVED_02 = o.RESERVED_02;
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const unsolicited_message_1<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.systime;
        s << ", " << x.year;
        s << ", " << x.month;
        s << ", " << x.day;
        s << ", " << x.hour;
        s << ", " << x.minute;
        s << ", " << x.second;
        s << ", " << x.utc_offset;
        s << ", " << x.type;
        s << ", " << x.intended_followup_actions;
        s << ", " << x.id;
        s << ", "; write_array(s, 128, x.message);
        s << ", " << x.RESERVED_00;
        s << ", "; write_array(s, 64, x.RESERVED_01);
        s << ", " << x.RESERVED_02;
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, unsolicited_message_1<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.systime;
        s >> ',' >> x.year;
        s >> ',' >> x.month;
        s >> ',' >> x.day;
        s >> ',' >> x.hour;
        s >> ',' >> x.minute;
        s >> ',' >> x.second;
        s >> ',' >> x.utc_offset;
        s >> ',' >> x.type;
        s >> ',' >> x.intended_followup_actions;
        s >> ',' >> x.id;
        s >> ','; read_array(s, 128, x.message);
        s >> ',' >> x.RESERVED_00;
        s >> ','; read_array(s, 64, x.RESERVED_01);
        s >> ',' >> x.RESERVED_02;
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! empty helper package

//! This package is used for data alignment purposes.
//! <para>This package belongs to the predefined selectors:</para>
//! <para>protocol</para>
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct void_data
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef void_data<rebind_it> type;
    };

    typedef it iterator_type;

    void_data(it begin, it end, bool dirty=false)
        : end_(begin)
    {}
    #endif //DOXYGEN

    enum { id_main = 65535, id_sub = 65535};
    #ifndef DOXYGEN
    enum { max_bit_width = 0};
    it begin() const { return end_; }
    it end() const { return end_; }

    it end_;
    #endif //DOXYGEN
    #ifndef DOXYGEN

    template<class ito>
    void_data& operator=(const void_data<ito>& o) {
        return *this;
    }
    #endif //DOXYGEN
};
#ifndef DOXYGEN
template<>
struct void_data<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef void_data<rebind_it> type;
    };

    enum { id_main = 65535, id_sub = 65535};


    void_data() {}
    template<class it>
    void_data(const void_data<it>& o) {
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const void_data<it>& x) {
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, void_data<it>& x) {
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct wfm_dg_hp
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef wfm_dg_hp<rebind_it> type;
    };

    typedef it iterator_type;

    wfm_dg_hp(it begin, it end, bool dirty=false)
        : cnt_ls(begin)
        , mean(begin)
        , stddev(begin)
        , rnggt_start(begin)
        , rnggt_stop(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 50002, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 131168};
    it begin() const { return cnt_ls.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_ls;
    #else
    field<uint8_t, sc_uint8, 0, it> cnt_ls;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t mean;
    #else
    field<uint16_t, sc_uint16, 32, it> mean;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t stddev;
    #else
    field<uint16_t, sc_uint16, 48, it> stddev;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t rnggt_start;
    #else
    field<uint16_t, sc_uint16, 64, it> rnggt_start;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t rnggt_stop;
    #else
    field<uint16_t, sc_uint16, 80, it> rnggt_stop;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 8192 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sample(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint16_t sample;
        #else
        field<uint16_t, sc_uint16, 0, it> sample;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[8192];
    #else
    sequence<wfm_dg_hp, 16, 96, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    wfm_dg_hp& operator=(const wfm_dg_hp<ito>& o) {
        cnt_ls = o.cnt_ls;
        mean = o.mean;
        stddev = o.stddev;
        rnggt_start = o.rnggt_start;
        rnggt_stop = o.rnggt_stop;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct wfm_dg_hp<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef wfm_dg_hp<rebind_it> type;
    };

    enum { id_main = 50002, id_sub = 0};

    uint8_t          cnt_ls;
    uint16_t         mean;
    uint16_t         stddev;
    uint16_t         rnggt_start;
    uint16_t         rnggt_stop;
    std::size_t data_size;
    enum { data_max_size = 8192 };
    struct sequence_definition {
        uint16_t         sample;
    } data[8192];

    wfm_dg_hp() {}
    template<class it>
    wfm_dg_hp(const wfm_dg_hp<it>& o) {
        cnt_ls = o.cnt_ls;
        mean = o.mean;
        stddev = o.stddev;
        rnggt_start = o.rnggt_start;
        rnggt_stop = o.rnggt_stop;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const wfm_dg_hp<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.cnt_ls;
        s << ", " << x.mean;
        s << ", " << x.stddev;
        s << ", " << x.rnggt_start;
        s << ", " << x.rnggt_stop;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].sample
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, wfm_dg_hp<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.cnt_ls;
        s >> ',' >> x.mean;
        s >> ',' >> x.stddev;
        s >> ',' >> x.rnggt_start;
        s >> ',' >> x.rnggt_stop;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].sample)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct wfm_dg_lp
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef wfm_dg_lp<rebind_it> type;
    };

    typedef it iterator_type;

    wfm_dg_lp(it begin, it end, bool dirty=false)
        : cnt_ls(begin)
        , mean(begin)
        , stddev(begin)
        , rnggt_start(begin)
        , rnggt_stop(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 50003, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 131168};
    it begin() const { return cnt_ls.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_ls;
    #else
    field<uint8_t, sc_uint8, 0, it> cnt_ls;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t mean;
    #else
    field<uint16_t, sc_uint16, 32, it> mean;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t stddev;
    #else
    field<uint16_t, sc_uint16, 48, it> stddev;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t rnggt_start;
    #else
    field<uint16_t, sc_uint16, 64, it> rnggt_start;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t rnggt_stop;
    #else
    field<uint16_t, sc_uint16, 80, it> rnggt_stop;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 8192 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sample(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint16_t sample;
        #else
        field<uint16_t, sc_uint16, 0, it> sample;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[8192];
    #else
    sequence<wfm_dg_lp, 16, 96, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    wfm_dg_lp& operator=(const wfm_dg_lp<ito>& o) {
        cnt_ls = o.cnt_ls;
        mean = o.mean;
        stddev = o.stddev;
        rnggt_start = o.rnggt_start;
        rnggt_stop = o.rnggt_stop;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct wfm_dg_lp<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef wfm_dg_lp<rebind_it> type;
    };

    enum { id_main = 50003, id_sub = 0};

    uint8_t          cnt_ls;
    uint16_t         mean;
    uint16_t         stddev;
    uint16_t         rnggt_start;
    uint16_t         rnggt_stop;
    std::size_t data_size;
    enum { data_max_size = 8192 };
    struct sequence_definition {
        uint16_t         sample;
    } data[8192];

    wfm_dg_lp() {}
    template<class it>
    wfm_dg_lp(const wfm_dg_lp<it>& o) {
        cnt_ls = o.cnt_ls;
        mean = o.mean;
        stddev = o.stddev;
        rnggt_start = o.rnggt_start;
        rnggt_stop = o.rnggt_stop;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const wfm_dg_lp<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.cnt_ls;
        s << ", " << x.mean;
        s << ", " << x.stddev;
        s << ", " << x.rnggt_start;
        s << ", " << x.rnggt_stop;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].sample
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, wfm_dg_lp<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.cnt_ls;
        s >> ',' >> x.mean;
        s >> ',' >> x.stddev;
        s >> ',' >> x.rnggt_start;
        s >> ',' >> x.rnggt_stop;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].sample)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
#ifdef DOXYGEN
template<class it>
#else
template<class it = void>
#endif
struct wfm_dg_shp
{
    #ifndef DOXYGEN
    template<class rebind_it = void>
    struct rebind {
        typedef wfm_dg_shp<rebind_it> type;
    };

    typedef it iterator_type;

    wfm_dg_shp(it begin, it end, bool dirty=false)
        : cnt_ls(begin)
        , phase(begin)
        , mean(begin)
        , stddev(begin)
        , rnggt_start(begin)
        , num_samples(begin)
        , data(begin, end, data_size, dirty)
    {}
    #endif //DOXYGEN

    enum { id_main = 50004, id_sub = 0};
    #ifndef DOXYGEN
    enum { max_bit_width = 131168};
    it begin() const { return cnt_ls.begin(); }
    it end() const { return data.end(); }

    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t cnt_ls;
    #else
    field<uint8_t, sc_uint8, 0, it> cnt_ls;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint8_t phase;
    #else
    field<uint8_t, sc_uint8, 8, it> phase;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t mean;
    #else
    field<uint16_t, sc_uint16, 32, it> mean;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t stddev;
    #else
    field<uint16_t, sc_uint16, 48, it> stddev;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t rnggt_start;
    #else
    field<uint16_t, sc_uint16, 64, it> rnggt_start;
    #endif //DOXYGEN
    #ifdef DOXYGEN
    uint16_t num_samples;
    #else
    field<uint16_t, sc_uint16, 80, it> num_samples;
    #endif //DOXYGEN

    std::size_t data_size;
    enum { data_max_size = 8192 };
    struct sequence_definition
    {
        #ifndef DOXYGEN
        sequence_definition(it begin, unsigned begin_bit)
            : sample(begin, begin_bit)
        {}
        #endif //DOXYGEN

        #ifdef DOXYGEN
        uint16_t sample;
        #else
        field<uint16_t, sc_uint16, 0, it> sample;
        #endif
    };
    #ifdef DOXYGEN
    sequence_definition data[8192];
    #else
    sequence<wfm_dg_shp, 16, 96, it> data;
    #endif
    #ifndef DOXYGEN

    template<class ito>
    wfm_dg_shp& operator=(const wfm_dg_shp<ito>& o) {
        cnt_ls = o.cnt_ls;
        phase = o.phase;
        mean = o.mean;
        stddev = o.stddev;
        rnggt_start = o.rnggt_start;
        num_samples = o.num_samples;
        data_size = o.data_size;
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
        return *this;
    }
    #endif //DOXYGEN
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<>
struct wfm_dg_shp<void>
{
    template<class rebind_it = void>
    struct rebind {
        typedef wfm_dg_shp<rebind_it> type;
    };

    enum { id_main = 50004, id_sub = 0};

    uint8_t          cnt_ls;
    uint8_t          phase;
    uint16_t         mean;
    uint16_t         stddev;
    uint16_t         rnggt_start;
    uint16_t         num_samples;
    std::size_t data_size;
    enum { data_max_size = 8192 };
    struct sequence_definition {
        uint16_t         sample;
    } data[8192];

    wfm_dg_shp() {}
    template<class it>
    wfm_dg_shp(const wfm_dg_shp<it>& o) {
        cnt_ls = o.cnt_ls;
        phase = o.phase;
        mean = o.mean;
        stddev = o.stddev;
        rnggt_start = o.rnggt_start;
        num_samples = o.num_samples;
        data_size = o.data.size();
        for(unsigned n=0; n<data_size; ++n){
            data[n].sample = o.data[n].sample;
        }
    }
};
#endif //DOXYGEN
#ifndef DOXYGEN
template<class it>
std::ostream& operator<<(std::ostream& s, const wfm_dg_shp<it>& x) {
    package_ostream_entry ok(s);
    if (ok) {
        s << ", " << x.cnt_ls;
        s << ", " << x.phase;
        s << ", " << x.mean;
        s << ", " << x.stddev;
        s << ", " << x.rnggt_start;
        s << ", " << x.num_samples;

        s << ", [";
        for (std::size_t n=0; n<x.data_size; ++n) {
            if (n>0) s << ", ";
            s
            << "{" << x.data[n].sample
            << "}";
        }
        s << "]";
    }
    return s;
}

template<class it>
std::istream& operator>>(std::istream& s, wfm_dg_shp<it>& x) {
    package_istream_entry ok(s);
    if (ok) {
        s >> ',' >> x.cnt_ls;
        s >> ',' >> x.phase;
        s >> ',' >> x.mean;
        s >> ',' >> x.stddev;
        s >> ',' >> x.rnggt_start;
        s >> ',' >> x.num_samples;

        s >> ',' >> '[' >> std::ws;
        x.data_size = 0;
        while(s.good() && s.peek() == '{') {
            ++x.data_size;
            if (s
                >> '{' >> (x.data[x.data_size-1].sample)
                >> '}' >> std::ws
            ) {
                if (s.peek() == ',') s.ignore(1);
                s >> std::ws;
            }
        }
        s >> ']';
    }
    return s;
}
#endif //DOXYGEN
//-----------------------------------------------------------------------------
//! The base for packet dispatcher classes.
//
//! An instance of this class implements package dispatching.
//! Raw packages are fed to the class via the dispatch member function.
//! The raw package will be decoded and a virtual function corresponding
//! to its type will be called. The overloaded function will receive the
//! decoded package as a paramter. Each packet has several fields which
//! are separately documented. The actual implementation of packets differs
//! from how they appear in the documentation. Instead of the shown packets
//! the handler funtions receive a proxy of the packet, that acts as if it
//! was the real packet. This means that one can use the various fields in
//! read (and write, where apropriate) expressions, but one cannot e.g. take
//! the address of a field or make a reference to it. If you need to know the
//! real implementation, please look at the source code.
//!
//! The "it" template paramater specifies an iterator to a raw package,
//! which in principle can be a random access pointer to unsigned of various
//! bitsizes. In practice the iterator always shall be taken from the one
//! defined in the buffer class.
//!
//! Package dispatching can be fine controlled by setting the selector, which
//! is a bitset, indexed by the package id. The package id to use for indexing
//! is one of the enum values defined in struct package_id. A couple of predefined
//! sets are available. Please note, that while it is possible to create arbitrary
//! selector combinations, care should be taken to always include the mandatory
//! header packet.
//!
class basic_packets
{
public:
    typedef decoder_rxpmarker::const_iterator iterator_type; //!< embedded iterator type definition
    selector_type selector; //!< package dispatch selection

    basic_packets()
        : selector(select_all)
        , package_probes(false)
        , dispatch_end_requested(false)
    {}
    virtual ~basic_packets()
    {}

    //! dispatch a raw packet
    //
    //! This function invokes the dispatch process. It is fed with a buffer taken from
    //! the decoder_rxpmarker. The dispatcher first infers the type of the raw packet from
    //! its embeded id and a lookup table which has been provided from the instrument in the
    //! header packet. Then the dispatcher uses the major and minor packet id numbers to
    //! map the packet to a handler function. A received packet with a higher minor number
    //! is compatible to any packet of the same major but smaller minor number. The dispatcher
    //! will use such packets in place of these smaller numbered packets.
    //!
    //! The dispatching process is subject to a selector filter which has a flag for each
    //! packet.
    bool dispatch(const iterator_type& begin, const iterator_type& end);

protected:

    virtual void on_id(const package_id& arg);
    //!\param arg  environmental information
    virtual void on_atmosphere(const atmosphere<iterator_type>& arg);
    //!\param arg  extended environmental information
    virtual void on_atmosphere_1(const atmosphere_1<iterator_type>& arg);
    //!\param arg  laser beam description
    virtual void on_beam_geometry(const beam_geometry<iterator_type>& arg);
    //!\param arg  external synchronization input
    virtual void on_counter_sync(const counter_sync<iterator_type>& arg);
    //!\param arg  geometrical parameters of external devices
    virtual void on_device_mounting(const device_mounting<iterator_type>& arg);
    //!\param arg  extents of various data fields
    virtual void on_extents(const extents<iterator_type>& arg);
    //!\param arg  start of a scan frame in down direction.
    virtual void on_frame_start_dn(const frame_start_dn<iterator_type>& arg);
    //!\param arg  Start of a scan frame in up direction.
    virtual void on_frame_start_up(const frame_start_up<iterator_type>& arg);
    //!\param arg  end of a scan frame
    virtual void on_frame_stop(const frame_stop<iterator_type>& arg);
    //!\param arg  The mandatory header package.
    virtual void on_header(const header<iterator_type>& arg);
    //!\param arg  Extension header
    virtual void on_header_ext(const header_ext<iterator_type>& arg);
    //!\param arg  GPS data
    virtual void on_hk_gps(const hk_gps<iterator_type>& arg);
    //!\param arg  GPS data
    virtual void on_hk_gps_ts(const hk_gps_ts<iterator_type>& arg);
    //!\param arg  GPS data
    virtual void on_hk_gps_ts_status(const hk_gps_ts_status<iterator_type>& arg);
    //!\param arg  GPS data
    virtual void on_hk_gps_ts_status_dop(const hk_gps_ts_status_dop<iterator_type>& arg);
    //!\param arg  inclination sensor
    virtual void on_hk_incl(const hk_incl<iterator_type>& arg);
    //!\param arg  inclination sensor
    virtual void on_hk_incl_4axes(const hk_incl_4axes<iterator_type>& arg);
    //!\param arg  built in real time clock of scanning device, local time
    virtual void on_hk_rtc(const hk_rtc<iterator_type>& arg);
    //!\param arg  start of a line scan in down direction
    virtual void on_line_start_dn(const line_start_dn<iterator_type>& arg);
    //!\param arg  start of a line scan in up direction
    virtual void on_line_start_up(const line_start_up<iterator_type>& arg);
    //!\param arg  end of line scan
    virtual void on_line_stop(const line_stop<iterator_type>& arg);
    //!\param arg  measurement has started
    virtual void on_meas_start(const meas_start<iterator_type>& arg);
    //!\param arg  measurement has stopped
    virtual void on_meas_stop(const meas_stop<iterator_type>& arg);
    //!\param arg  current setting of subdivider for monitoring data stream
    virtual void on_monitoring_info(const monitoring_info<iterator_type>& arg);
    //!\param arg  pulse per second, external time synchronisation
    virtual void on_pps_sync(const pps_sync<iterator_type>& arg);
    //!\param arg  pulse per second, external time synchronisation
    virtual void on_pps_sync_ext(const pps_sync_ext<iterator_type>& arg);
    //!\param arg  scan pattern description
    virtual void on_scan_rect_fov(const scan_rect_fov<iterator_type>& arg);
    //!\param arg  scanner pose (position and orientation
    virtual void on_scanner_pose(const scanner_pose<iterator_type>& arg);
    //!\param arg  units information
    virtual void on_units(const units<iterator_type>& arg);
    //!\param arg  units information
    virtual void on_units_1(const units_1<iterator_type>& arg);
    //!\param arg  spontaneous information and error messages
    virtual void on_unsolicited_message(const unsolicited_message<iterator_type>& arg);
    //!\param arg  spontaneous information and error messages
    virtual void on_unsolicited_message_1(const unsolicited_message_1<iterator_type>& arg);
    //!\param arg  empty helper package
    virtual void on_void_data(const void_data<iterator_type>& arg);
    #ifndef DOXYGEN
    virtual void on_alert(const alert<iterator_type>& arg);
    virtual void on_avg_fine_ref_dg(const avg_fine_ref_dg<iterator_type>& arg);
    virtual void on_calib_waveform(const calib_waveform<iterator_type>& arg);
    virtual void on_calib_waveform_L2(const calib_waveform_L2<iterator_type>& arg);
    virtual void on_channel_combination_table(const channel_combination_table<iterator_type>& arg);
    virtual void on_context_end(const context_end<iterator_type>& arg);
    virtual void on_crc32_check(const crc32_check<iterator_type>& arg);
    virtual void on_crc32_header(const crc32_header<iterator_type>& arg);
    virtual void on_datagram_separator(const datagram_separator<iterator_type>& arg);
    virtual void on_debug_hw_dg(const debug_hw_dg<iterator_type>& arg);
    virtual void on_debug_sw_dg(const debug_sw_dg<iterator_type>& arg);
    virtual void on_device_geometry(const device_geometry<iterator_type>& arg);
    virtual void on_echo(const echo<iterator_type>& arg);
    virtual void on_firmware(const firmware<iterator_type>& arg);
    virtual void on_firmware_1(const firmware_1<iterator_type>& arg);
    virtual void on_firmware_2(const firmware_2<iterator_type>& arg);
    virtual void on_firmware_3(const firmware_3<iterator_type>& arg);
    virtual void on_frame_start(const frame_start<iterator_type>& arg);
    virtual void on_generic_end(const generic_end<iterator_type>& arg);
    virtual void on_hk_bat(const hk_bat<iterator_type>& arg);
    virtual void on_hk_bat_1(const hk_bat_1<iterator_type>& arg);
    virtual void on_hk_bat_2(const hk_bat_2<iterator_type>& arg);
    virtual void on_hk_cam(const hk_cam<iterator_type>& arg);
    virtual void on_hk_ctr(const hk_ctr<iterator_type>& arg);
    virtual void on_hk_ctr_1(const hk_ctr_1<iterator_type>& arg);
    virtual void on_hk_extended_external(const hk_extended_external<iterator_type>& arg);
    virtual void on_hk_extended_internal(const hk_extended_internal<iterator_type>& arg);
    virtual void on_hk_pwr(const hk_pwr<iterator_type>& arg);
    virtual void on_hk_pwr_1(const hk_pwr_1<iterator_type>& arg);
    virtual void on_hk_rad(const hk_rad<iterator_type>& arg);
    virtual void on_hk_rng(const hk_rng<iterator_type>& arg);
    virtual void on_hk_rng_1(const hk_rng_1<iterator_type>& arg);
    virtual void on_hk_rng_2(const hk_rng_2<iterator_type>& arg);
    virtual void on_hk_rng_3(const hk_rng_3<iterator_type>& arg);
    virtual void on_hk_rng_4(const hk_rng_4<iterator_type>& arg);
    virtual void on_hk_rng_5(const hk_rng_5<iterator_type>& arg);
    virtual void on_hk_rng_6(const hk_rng_6<iterator_type>& arg);
    virtual void on_hk_rng_7(const hk_rng_7<iterator_type>& arg);
    virtual void on_hk_rng_8(const hk_rng_8<iterator_type>& arg);
    virtual void on_hk_rtc_sys(const hk_rtc_sys<iterator_type>& arg);
    virtual void on_hk_scn(const hk_scn<iterator_type>& arg);
    virtual void on_hk_scn_1(const hk_scn_1<iterator_type>& arg);
    virtual void on_hk_time(const hk_time<iterator_type>& arg);
    virtual void on_ht_dbg_data(const ht_dbg_data<iterator_type>& arg);
    virtual void on_inclination(const inclination<iterator_type>& arg);
    virtual void on_inclination_4axes(const inclination_4axes<iterator_type>& arg);
    virtual void on_inclination_device(const inclination_device<iterator_type>& arg);
    virtual void on_inclination_device_4axes(const inclination_device_4axes<iterator_type>& arg);
    virtual void on_inclination_device_4axes_offset(const inclination_device_4axes_offset<iterator_type>& arg);
    virtual void on_laser_echo(const laser_echo<iterator_type>& arg);
    virtual void on_laser_echo_qual(const laser_echo_qual<iterator_type>& arg);
    virtual void on_laser_echo_sw(const laser_echo_sw<iterator_type>& arg);
    virtual void on_laser_shot(const laser_shot<iterator_type>& arg);
    virtual void on_laser_shot_1angle(const laser_shot_1angle<iterator_type>& arg);
    virtual void on_laser_shot_2angles(const laser_shot_2angles<iterator_type>& arg);
    virtual void on_laser_shot_2angles_rad(const laser_shot_2angles_rad<iterator_type>& arg);
    virtual void on_laser_shot_3angles(const laser_shot_3angles<iterator_type>& arg);
    virtual void on_laser_shot_6angles(const laser_shot_6angles<iterator_type>& arg);
    virtual void on_line_start(const line_start<iterator_type>& arg);
    virtual void on_magnetic_field(const magnetic_field<iterator_type>& arg);
    virtual void on_packed_frame_echo(const packed_frame_echo<iterator_type>& arg);
    virtual void on_packed_frame_laser_shot_2angles(const packed_frame_laser_shot_2angles<iterator_type>& arg);
    virtual void on_packed_frame_laser_shot_2angles_rad(const packed_frame_laser_shot_2angles_rad<iterator_type>& arg);
    virtual void on_packed_key_echo(const packed_key_echo<iterator_type>& arg);
    virtual void on_packed_key_laser_shot_2angles(const packed_key_laser_shot_2angles<iterator_type>& arg);
    virtual void on_packed_key_laser_shot_2angles_rad(const packed_key_laser_shot_2angles_rad<iterator_type>& arg);
    virtual void on_range_calc(const range_calc<iterator_type>& arg);
    virtual void on_range_finder_debug_acq(const range_finder_debug_acq<iterator_type>& arg);
    virtual void on_range_finder_debug_calc(const range_finder_debug_calc<iterator_type>& arg);
    virtual void on_range_finder_debug_laser(const range_finder_debug_laser<iterator_type>& arg);
    virtual void on_range_finder_debug_rcv(const range_finder_debug_rcv<iterator_type>& arg);
    virtual void on_range_finder_program(const range_finder_program<iterator_type>& arg);
    virtual void on_range_finder_settings(const range_finder_settings<iterator_type>& arg);
    virtual void on_rel_refl_table(const rel_refl_table<iterator_type>& arg);
    virtual void on_sbl_dg_data(const sbl_dg_data<iterator_type>& arg);
    virtual void on_sbl_dg_data_compressed(const sbl_dg_data_compressed<iterator_type>& arg);
    virtual void on_sbl_dg_header(const sbl_dg_header<iterator_type>& arg);
    virtual void on_slt_dg(const slt_dg<iterator_type>& arg);
    virtual void on_slt_dg_1(const slt_dg_1<iterator_type>& arg);
    virtual void on_slt_dg_2(const slt_dg_2<iterator_type>& arg);
    virtual void on_slt_dg_3(const slt_dg_3<iterator_type>& arg);
    virtual void on_tgt_dg(const tgt_dg<iterator_type>& arg);
    virtual void on_ublox_lea5t_rxm(const ublox_lea5t_rxm<iterator_type>& arg);
    virtual void on_ublox_lea5t_rxm_sfrb(const ublox_lea5t_rxm_sfrb<iterator_type>& arg);
    virtual void on_wfm_dg_hp(const wfm_dg_hp<iterator_type>& arg);
    virtual void on_wfm_dg_lp(const wfm_dg_lp<iterator_type>& arg);
    virtual void on_wfm_dg_shp(const wfm_dg_shp<iterator_type>& arg);
    #endif //DOXYGEN
    //! This function will be called for packets unknown to this version of the library.
    //!\param id_main the main id
    //!\param id_sub the sub id
    virtual void on_unknown(unsigned id_main, unsigned id_sub, const iterator_type& begin, const iterator_type& end);

    //! By default packets are only dispatched to the most derived packet version known
    //! to the library. If you want the function that handles the base version also to
    //! be called, you need to call this function.
    //!
    void handle_parent() {
        has_been_handled = false;
    }
    //! This function will cause the dispatch function to return with a value of true.
    void request_dispatch_end() {
        dispatch_end_requested = true;
    }

    lookup_table lookup;
    package_id   id;

protected:

    bool package_probes;

private:

    bool has_been_handled;
    bool dispatch_end_requested;
};
//-----------------------------------------------------------------------------
//! convert to string representation
//
//! This class converts a package into a string representation.
//! By default all packets are converted. The selector can be used
//! to filter out the intended subset of packets. Dont forget to
//! specify the header packet in the selector!
class ostream_packets
    : public basic_packets
{
public:
    typedef basic_packets::iterator_type iterator_type; //!< embedded iterator type definition

    //! construct with stream reference
    //
    //!\param out a reference to an out stream
    ostream_packets(std::ostream& out)
        : out(&out)
    {}
    virtual ~ostream_packets()
    {}

#ifndef DOXYGEN
protected:
    ostream_packets() : out(0) {}
    #endif //DOXYGEN

    #ifndef DOXYGEN
    void on_id(const package_id& arg);
    virtual void on_alert(const alert<iterator_type>& arg);
    virtual void on_atmosphere(const atmosphere<iterator_type>& arg);
    virtual void on_atmosphere_1(const atmosphere_1<iterator_type>& arg);
    virtual void on_avg_fine_ref_dg(const avg_fine_ref_dg<iterator_type>& arg);
    virtual void on_beam_geometry(const beam_geometry<iterator_type>& arg);
    virtual void on_calib_waveform(const calib_waveform<iterator_type>& arg);
    virtual void on_calib_waveform_L2(const calib_waveform_L2<iterator_type>& arg);
    virtual void on_channel_combination_table(const channel_combination_table<iterator_type>& arg);
    virtual void on_context_end(const context_end<iterator_type>& arg);
    virtual void on_counter_sync(const counter_sync<iterator_type>& arg);
    virtual void on_crc32_check(const crc32_check<iterator_type>& arg);
    virtual void on_crc32_header(const crc32_header<iterator_type>& arg);
    virtual void on_datagram_separator(const datagram_separator<iterator_type>& arg);
    virtual void on_debug_hw_dg(const debug_hw_dg<iterator_type>& arg);
    virtual void on_debug_sw_dg(const debug_sw_dg<iterator_type>& arg);
    virtual void on_device_geometry(const device_geometry<iterator_type>& arg);
    virtual void on_device_mounting(const device_mounting<iterator_type>& arg);
    virtual void on_echo(const echo<iterator_type>& arg);
    virtual void on_extents(const extents<iterator_type>& arg);
    virtual void on_firmware(const firmware<iterator_type>& arg);
    virtual void on_firmware_1(const firmware_1<iterator_type>& arg);
    virtual void on_firmware_2(const firmware_2<iterator_type>& arg);
    virtual void on_firmware_3(const firmware_3<iterator_type>& arg);
    virtual void on_frame_start(const frame_start<iterator_type>& arg);
    virtual void on_frame_start_dn(const frame_start_dn<iterator_type>& arg);
    virtual void on_frame_start_up(const frame_start_up<iterator_type>& arg);
    virtual void on_frame_stop(const frame_stop<iterator_type>& arg);
    virtual void on_generic_end(const generic_end<iterator_type>& arg);
    virtual void on_header(const header<iterator_type>& arg);
    virtual void on_header_ext(const header_ext<iterator_type>& arg);
    virtual void on_hk_bat(const hk_bat<iterator_type>& arg);
    virtual void on_hk_bat_1(const hk_bat_1<iterator_type>& arg);
    virtual void on_hk_bat_2(const hk_bat_2<iterator_type>& arg);
    virtual void on_hk_cam(const hk_cam<iterator_type>& arg);
    virtual void on_hk_ctr(const hk_ctr<iterator_type>& arg);
    virtual void on_hk_ctr_1(const hk_ctr_1<iterator_type>& arg);
    virtual void on_hk_extended_external(const hk_extended_external<iterator_type>& arg);
    virtual void on_hk_extended_internal(const hk_extended_internal<iterator_type>& arg);
    virtual void on_hk_gps(const hk_gps<iterator_type>& arg);
    virtual void on_hk_gps_ts(const hk_gps_ts<iterator_type>& arg);
    virtual void on_hk_gps_ts_status(const hk_gps_ts_status<iterator_type>& arg);
    virtual void on_hk_gps_ts_status_dop(const hk_gps_ts_status_dop<iterator_type>& arg);
    virtual void on_hk_incl(const hk_incl<iterator_type>& arg);
    virtual void on_hk_incl_4axes(const hk_incl_4axes<iterator_type>& arg);
    virtual void on_hk_pwr(const hk_pwr<iterator_type>& arg);
    virtual void on_hk_pwr_1(const hk_pwr_1<iterator_type>& arg);
    virtual void on_hk_rad(const hk_rad<iterator_type>& arg);
    virtual void on_hk_rng(const hk_rng<iterator_type>& arg);
    virtual void on_hk_rng_1(const hk_rng_1<iterator_type>& arg);
    virtual void on_hk_rng_2(const hk_rng_2<iterator_type>& arg);
    virtual void on_hk_rng_3(const hk_rng_3<iterator_type>& arg);
    virtual void on_hk_rng_4(const hk_rng_4<iterator_type>& arg);
    virtual void on_hk_rng_5(const hk_rng_5<iterator_type>& arg);
    virtual void on_hk_rng_6(const hk_rng_6<iterator_type>& arg);
    virtual void on_hk_rng_7(const hk_rng_7<iterator_type>& arg);
    virtual void on_hk_rng_8(const hk_rng_8<iterator_type>& arg);
    virtual void on_hk_rtc(const hk_rtc<iterator_type>& arg);
    virtual void on_hk_rtc_sys(const hk_rtc_sys<iterator_type>& arg);
    virtual void on_hk_scn(const hk_scn<iterator_type>& arg);
    virtual void on_hk_scn_1(const hk_scn_1<iterator_type>& arg);
    virtual void on_hk_time(const hk_time<iterator_type>& arg);
    virtual void on_ht_dbg_data(const ht_dbg_data<iterator_type>& arg);
    virtual void on_inclination(const inclination<iterator_type>& arg);
    virtual void on_inclination_4axes(const inclination_4axes<iterator_type>& arg);
    virtual void on_inclination_device(const inclination_device<iterator_type>& arg);
    virtual void on_inclination_device_4axes(const inclination_device_4axes<iterator_type>& arg);
    virtual void on_inclination_device_4axes_offset(const inclination_device_4axes_offset<iterator_type>& arg);
    virtual void on_laser_echo(const laser_echo<iterator_type>& arg);
    virtual void on_laser_echo_qual(const laser_echo_qual<iterator_type>& arg);
    virtual void on_laser_echo_sw(const laser_echo_sw<iterator_type>& arg);
    virtual void on_laser_shot(const laser_shot<iterator_type>& arg);
    virtual void on_laser_shot_1angle(const laser_shot_1angle<iterator_type>& arg);
    virtual void on_laser_shot_2angles(const laser_shot_2angles<iterator_type>& arg);
    virtual void on_laser_shot_2angles_rad(const laser_shot_2angles_rad<iterator_type>& arg);
    virtual void on_laser_shot_3angles(const laser_shot_3angles<iterator_type>& arg);
    virtual void on_laser_shot_6angles(const laser_shot_6angles<iterator_type>& arg);
    virtual void on_line_start(const line_start<iterator_type>& arg);
    virtual void on_line_start_dn(const line_start_dn<iterator_type>& arg);
    virtual void on_line_start_up(const line_start_up<iterator_type>& arg);
    virtual void on_line_stop(const line_stop<iterator_type>& arg);
    virtual void on_magnetic_field(const magnetic_field<iterator_type>& arg);
    virtual void on_meas_start(const meas_start<iterator_type>& arg);
    virtual void on_meas_stop(const meas_stop<iterator_type>& arg);
    virtual void on_monitoring_info(const monitoring_info<iterator_type>& arg);
    virtual void on_packed_frame_echo(const packed_frame_echo<iterator_type>& arg);
    virtual void on_packed_frame_laser_shot_2angles(const packed_frame_laser_shot_2angles<iterator_type>& arg);
    virtual void on_packed_frame_laser_shot_2angles_rad(const packed_frame_laser_shot_2angles_rad<iterator_type>& arg);
    virtual void on_packed_key_echo(const packed_key_echo<iterator_type>& arg);
    virtual void on_packed_key_laser_shot_2angles(const packed_key_laser_shot_2angles<iterator_type>& arg);
    virtual void on_packed_key_laser_shot_2angles_rad(const packed_key_laser_shot_2angles_rad<iterator_type>& arg);
    virtual void on_pps_sync(const pps_sync<iterator_type>& arg);
    virtual void on_pps_sync_ext(const pps_sync_ext<iterator_type>& arg);
    virtual void on_range_calc(const range_calc<iterator_type>& arg);
    virtual void on_range_finder_debug_acq(const range_finder_debug_acq<iterator_type>& arg);
    virtual void on_range_finder_debug_calc(const range_finder_debug_calc<iterator_type>& arg);
    virtual void on_range_finder_debug_laser(const range_finder_debug_laser<iterator_type>& arg);
    virtual void on_range_finder_debug_rcv(const range_finder_debug_rcv<iterator_type>& arg);
    virtual void on_range_finder_program(const range_finder_program<iterator_type>& arg);
    virtual void on_range_finder_settings(const range_finder_settings<iterator_type>& arg);
    virtual void on_rel_refl_table(const rel_refl_table<iterator_type>& arg);
    virtual void on_sbl_dg_data(const sbl_dg_data<iterator_type>& arg);
    virtual void on_sbl_dg_data_compressed(const sbl_dg_data_compressed<iterator_type>& arg);
    virtual void on_sbl_dg_header(const sbl_dg_header<iterator_type>& arg);
    virtual void on_scan_rect_fov(const scan_rect_fov<iterator_type>& arg);
    virtual void on_scanner_pose(const scanner_pose<iterator_type>& arg);
    virtual void on_slt_dg(const slt_dg<iterator_type>& arg);
    virtual void on_slt_dg_1(const slt_dg_1<iterator_type>& arg);
    virtual void on_slt_dg_2(const slt_dg_2<iterator_type>& arg);
    virtual void on_slt_dg_3(const slt_dg_3<iterator_type>& arg);
    virtual void on_tgt_dg(const tgt_dg<iterator_type>& arg);
    virtual void on_ublox_lea5t_rxm(const ublox_lea5t_rxm<iterator_type>& arg);
    virtual void on_ublox_lea5t_rxm_sfrb(const ublox_lea5t_rxm_sfrb<iterator_type>& arg);
    virtual void on_units(const units<iterator_type>& arg);
    virtual void on_units_1(const units_1<iterator_type>& arg);
    virtual void on_unsolicited_message(const unsolicited_message<iterator_type>& arg);
    virtual void on_unsolicited_message_1(const unsolicited_message_1<iterator_type>& arg);
    virtual void on_void_data(const void_data<iterator_type>& arg);
    virtual void on_wfm_dg_hp(const wfm_dg_hp<iterator_type>& arg);
    virtual void on_wfm_dg_lp(const wfm_dg_lp<iterator_type>& arg);
    virtual void on_wfm_dg_shp(const wfm_dg_shp<iterator_type>& arg);
    virtual void on_unknown(unsigned id_main, unsigned id_sub, const iterator_type& begin, const iterator_type& end);

    std::ostream* out;
#endif //DOXYGEN
};
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
class encoder_packets
    : public basic_packets
{
public:
    typedef basic_packets::iterator_type iterator_type;
    typedef encoder_rxpmarker::iterator encoder_iterator_type;

    encoder_packets(encoder_rxpmarker& encoder_)
        : encoder(&encoder_)
    { encoder->put(encoder_buffer); }
    virtual ~encoder_packets()
    {}

protected:
    encoder_packets() : encoder(0) {}
    typedef encoder_rxpmarker::iterator it;
    typedef basic_packets::iterator_type cit;

    virtual void on_alert(const alert<cit>& arg);
    virtual void on_atmosphere(const atmosphere<cit>& arg);
    virtual void on_atmosphere_1(const atmosphere_1<cit>& arg);
    virtual void on_avg_fine_ref_dg(const avg_fine_ref_dg<cit>& arg);
    virtual void on_beam_geometry(const beam_geometry<cit>& arg);
    virtual void on_calib_waveform(const calib_waveform<cit>& arg);
    virtual void on_calib_waveform_L2(const calib_waveform_L2<cit>& arg);
    virtual void on_channel_combination_table(const channel_combination_table<cit>& arg);
    virtual void on_context_end(const context_end<cit>& arg);
    virtual void on_counter_sync(const counter_sync<cit>& arg);
    virtual void on_crc32_check(const crc32_check<cit>& arg);
    virtual void on_crc32_header(const crc32_header<cit>& arg);
    virtual void on_datagram_separator(const datagram_separator<cit>& arg);
    virtual void on_debug_hw_dg(const debug_hw_dg<cit>& arg);
    virtual void on_debug_sw_dg(const debug_sw_dg<cit>& arg);
    virtual void on_device_geometry(const device_geometry<cit>& arg);
    virtual void on_device_mounting(const device_mounting<cit>& arg);
    virtual void on_echo(const echo<cit>& arg);
    virtual void on_extents(const extents<cit>& arg);
    virtual void on_firmware(const firmware<cit>& arg);
    virtual void on_firmware_1(const firmware_1<cit>& arg);
    virtual void on_firmware_2(const firmware_2<cit>& arg);
    virtual void on_firmware_3(const firmware_3<cit>& arg);
    virtual void on_frame_start(const frame_start<cit>& arg);
    virtual void on_frame_start_dn(const frame_start_dn<cit>& arg);
    virtual void on_frame_start_up(const frame_start_up<cit>& arg);
    virtual void on_frame_stop(const frame_stop<cit>& arg);
    virtual void on_generic_end(const generic_end<cit>& arg);
    virtual void on_header(const header<cit>& arg);
    virtual void on_header_ext(const header_ext<cit>& arg);
    virtual void on_hk_bat(const hk_bat<cit>& arg);
    virtual void on_hk_bat_1(const hk_bat_1<cit>& arg);
    virtual void on_hk_bat_2(const hk_bat_2<cit>& arg);
    virtual void on_hk_cam(const hk_cam<cit>& arg);
    virtual void on_hk_ctr(const hk_ctr<cit>& arg);
    virtual void on_hk_ctr_1(const hk_ctr_1<cit>& arg);
    virtual void on_hk_extended_external(const hk_extended_external<cit>& arg);
    virtual void on_hk_extended_internal(const hk_extended_internal<cit>& arg);
    virtual void on_hk_gps(const hk_gps<cit>& arg);
    virtual void on_hk_gps_ts(const hk_gps_ts<cit>& arg);
    virtual void on_hk_gps_ts_status(const hk_gps_ts_status<cit>& arg);
    virtual void on_hk_gps_ts_status_dop(const hk_gps_ts_status_dop<cit>& arg);
    virtual void on_hk_incl(const hk_incl<cit>& arg);
    virtual void on_hk_incl_4axes(const hk_incl_4axes<cit>& arg);
    virtual void on_hk_pwr(const hk_pwr<cit>& arg);
    virtual void on_hk_pwr_1(const hk_pwr_1<cit>& arg);
    virtual void on_hk_rad(const hk_rad<cit>& arg);
    virtual void on_hk_rng(const hk_rng<cit>& arg);
    virtual void on_hk_rng_1(const hk_rng_1<cit>& arg);
    virtual void on_hk_rng_2(const hk_rng_2<cit>& arg);
    virtual void on_hk_rng_3(const hk_rng_3<cit>& arg);
    virtual void on_hk_rng_4(const hk_rng_4<cit>& arg);
    virtual void on_hk_rng_5(const hk_rng_5<cit>& arg);
    virtual void on_hk_rng_6(const hk_rng_6<cit>& arg);
    virtual void on_hk_rng_7(const hk_rng_7<cit>& arg);
    virtual void on_hk_rng_8(const hk_rng_8<cit>& arg);
    virtual void on_hk_rtc(const hk_rtc<cit>& arg);
    virtual void on_hk_rtc_sys(const hk_rtc_sys<cit>& arg);
    virtual void on_hk_scn(const hk_scn<cit>& arg);
    virtual void on_hk_scn_1(const hk_scn_1<cit>& arg);
    virtual void on_hk_time(const hk_time<cit>& arg);
    virtual void on_ht_dbg_data(const ht_dbg_data<cit>& arg);
    virtual void on_inclination(const inclination<cit>& arg);
    virtual void on_inclination_4axes(const inclination_4axes<cit>& arg);
    virtual void on_inclination_device(const inclination_device<cit>& arg);
    virtual void on_inclination_device_4axes(const inclination_device_4axes<cit>& arg);
    virtual void on_inclination_device_4axes_offset(const inclination_device_4axes_offset<cit>& arg);
    virtual void on_laser_echo(const laser_echo<cit>& arg);
    virtual void on_laser_echo_qual(const laser_echo_qual<cit>& arg);
    virtual void on_laser_echo_sw(const laser_echo_sw<cit>& arg);
    virtual void on_laser_shot(const laser_shot<cit>& arg);
    virtual void on_laser_shot_1angle(const laser_shot_1angle<cit>& arg);
    virtual void on_laser_shot_2angles(const laser_shot_2angles<cit>& arg);
    virtual void on_laser_shot_2angles_rad(const laser_shot_2angles_rad<cit>& arg);
    virtual void on_laser_shot_3angles(const laser_shot_3angles<cit>& arg);
    virtual void on_laser_shot_6angles(const laser_shot_6angles<cit>& arg);
    virtual void on_line_start(const line_start<cit>& arg);
    virtual void on_line_start_dn(const line_start_dn<cit>& arg);
    virtual void on_line_start_up(const line_start_up<cit>& arg);
    virtual void on_line_stop(const line_stop<cit>& arg);
    virtual void on_magnetic_field(const magnetic_field<cit>& arg);
    virtual void on_meas_start(const meas_start<cit>& arg);
    virtual void on_meas_stop(const meas_stop<cit>& arg);
    virtual void on_monitoring_info(const monitoring_info<cit>& arg);
    virtual void on_packed_frame_echo(const packed_frame_echo<cit>& arg);
    virtual void on_packed_frame_laser_shot_2angles(const packed_frame_laser_shot_2angles<cit>& arg);
    virtual void on_packed_frame_laser_shot_2angles_rad(const packed_frame_laser_shot_2angles_rad<cit>& arg);
    virtual void on_packed_key_echo(const packed_key_echo<cit>& arg);
    virtual void on_packed_key_laser_shot_2angles(const packed_key_laser_shot_2angles<cit>& arg);
    virtual void on_packed_key_laser_shot_2angles_rad(const packed_key_laser_shot_2angles_rad<cit>& arg);
    virtual void on_pps_sync(const pps_sync<cit>& arg);
    virtual void on_pps_sync_ext(const pps_sync_ext<cit>& arg);
    virtual void on_range_calc(const range_calc<cit>& arg);
    virtual void on_range_finder_debug_acq(const range_finder_debug_acq<cit>& arg);
    virtual void on_range_finder_debug_calc(const range_finder_debug_calc<cit>& arg);
    virtual void on_range_finder_debug_laser(const range_finder_debug_laser<cit>& arg);
    virtual void on_range_finder_debug_rcv(const range_finder_debug_rcv<cit>& arg);
    virtual void on_range_finder_program(const range_finder_program<cit>& arg);
    virtual void on_range_finder_settings(const range_finder_settings<cit>& arg);
    virtual void on_rel_refl_table(const rel_refl_table<cit>& arg);
    virtual void on_sbl_dg_data(const sbl_dg_data<cit>& arg);
    virtual void on_sbl_dg_data_compressed(const sbl_dg_data_compressed<cit>& arg);
    virtual void on_sbl_dg_header(const sbl_dg_header<cit>& arg);
    virtual void on_scan_rect_fov(const scan_rect_fov<cit>& arg);
    virtual void on_scanner_pose(const scanner_pose<cit>& arg);
    virtual void on_slt_dg(const slt_dg<cit>& arg);
    virtual void on_slt_dg_1(const slt_dg_1<cit>& arg);
    virtual void on_slt_dg_2(const slt_dg_2<cit>& arg);
    virtual void on_slt_dg_3(const slt_dg_3<cit>& arg);
    virtual void on_tgt_dg(const tgt_dg<cit>& arg);
    virtual void on_ublox_lea5t_rxm(const ublox_lea5t_rxm<cit>& arg);
    virtual void on_ublox_lea5t_rxm_sfrb(const ublox_lea5t_rxm_sfrb<cit>& arg);
    virtual void on_units(const units<cit>& arg);
    virtual void on_units_1(const units_1<cit>& arg);
    virtual void on_unsolicited_message(const unsolicited_message<cit>& arg);
    virtual void on_unsolicited_message_1(const unsolicited_message_1<cit>& arg);
    virtual void on_void_data(const void_data<cit>& arg);
    virtual void on_wfm_dg_hp(const wfm_dg_hp<cit>& arg);
    virtual void on_wfm_dg_lp(const wfm_dg_lp<cit>& arg);
    virtual void on_wfm_dg_shp(const wfm_dg_shp<cit>& arg);

public:
    encoder_rxpmarker* encoder;
    buffer encoder_buffer;
    lookup_table encoder_lookup;
};
#endif //DOXYGEN
#ifndef DOXYGEN

template<class pkg_type>
struct encoder_package
    : public buffer_package<pkg_type>
{
    explicit encoder_package(encoder_packets* ep)
        : buffer_package<pkg_type>(ep->encoder_lookup, ep->encoder_buffer)
        , ep_(ep)
    {
    }

    void commit()
    {
        buffer_package<pkg_type>::resize();
        ep_->encoder->put(ep_->encoder_buffer);
    }

    template<class src_pkg_type>
    encoder_package& operator= (const src_pkg_type& y)
    {
        pkg_type& x(*this);
        x = y;
        return *this;
    }

private:
    encoder_packets* ep_;
};
#endif //DOXYGEN
//-----------------------------------------------------------------------------
#ifndef DOXYGEN
template<class charT, class traits, class B>
bool ridataspec_read(std::vector<B>& buffer, unsigned id_main, unsigned id_sub, std::basic_istream<charT, traits>& in) {
    package_id::type type = package_id(id_main, id_sub);
    switch(type) {
        case package_id::alert: {
            buffer.resize(alert<B*>::max_bit_width/(8*sizeof(B))+1);
            alert<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::atmosphere: {
            buffer.resize(atmosphere<B*>::max_bit_width/(8*sizeof(B))+1);
            atmosphere<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::atmosphere_1: {
            buffer.resize(atmosphere_1<B*>::max_bit_width/(8*sizeof(B))+1);
            atmosphere_1<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::avg_fine_ref_dg: {
            buffer.resize(avg_fine_ref_dg<B*>::max_bit_width/(8*sizeof(B))+1);
            avg_fine_ref_dg<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::beam_geometry: {
            buffer.resize(beam_geometry<B*>::max_bit_width/(8*sizeof(B))+1);
            beam_geometry<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::calib_waveform: {
            buffer.resize(calib_waveform<B*>::max_bit_width/(8*sizeof(B))+1);
            calib_waveform<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::calib_waveform_L2: {
            buffer.resize(calib_waveform_L2<B*>::max_bit_width/(8*sizeof(B))+1);
            calib_waveform_L2<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::channel_combination_table: {
            buffer.resize(channel_combination_table<B*>::max_bit_width/(8*sizeof(B))+1);
            channel_combination_table<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::context_end: {
            buffer.resize(context_end<B*>::max_bit_width/(8*sizeof(B))+1);
            context_end<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::counter_sync: {
            buffer.resize(counter_sync<B*>::max_bit_width/(8*sizeof(B))+1);
            counter_sync<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::crc32_check: {
            buffer.resize(crc32_check<B*>::max_bit_width/(8*sizeof(B))+1);
            crc32_check<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::crc32_header: {
            buffer.resize(crc32_header<B*>::max_bit_width/(8*sizeof(B))+1);
            crc32_header<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::datagram_separator: {
            buffer.resize(datagram_separator<B*>::max_bit_width/(8*sizeof(B))+1);
            datagram_separator<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::debug_hw_dg: {
            buffer.resize(debug_hw_dg<B*>::max_bit_width/(8*sizeof(B))+1);
            debug_hw_dg<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::debug_sw_dg: {
            buffer.resize(debug_sw_dg<B*>::max_bit_width/(8*sizeof(B))+1);
            debug_sw_dg<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::device_geometry: {
            buffer.resize(device_geometry<B*>::max_bit_width/(8*sizeof(B))+1);
            device_geometry<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::device_mounting: {
            buffer.resize(device_mounting<B*>::max_bit_width/(8*sizeof(B))+1);
            device_mounting<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::echo: {
            buffer.resize(echo<B*>::max_bit_width/(8*sizeof(B))+1);
            echo<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::extents: {
            buffer.resize(extents<B*>::max_bit_width/(8*sizeof(B))+1);
            extents<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::firmware: {
            buffer.resize(firmware<B*>::max_bit_width/(8*sizeof(B))+1);
            firmware<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::firmware_1: {
            buffer.resize(firmware_1<B*>::max_bit_width/(8*sizeof(B))+1);
            firmware_1<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::firmware_2: {
            buffer.resize(firmware_2<B*>::max_bit_width/(8*sizeof(B))+1);
            firmware_2<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::firmware_3: {
            buffer.resize(firmware_3<B*>::max_bit_width/(8*sizeof(B))+1);
            firmware_3<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::frame_start: {
            buffer.resize(frame_start<B*>::max_bit_width/(8*sizeof(B))+1);
            frame_start<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::frame_start_dn: {
            buffer.resize(frame_start_dn<B*>::max_bit_width/(8*sizeof(B))+1);
            frame_start_dn<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::frame_start_up: {
            buffer.resize(frame_start_up<B*>::max_bit_width/(8*sizeof(B))+1);
            frame_start_up<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::frame_stop: {
            buffer.resize(frame_stop<B*>::max_bit_width/(8*sizeof(B))+1);
            frame_stop<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::generic_end: {
            buffer.resize(generic_end<B*>::max_bit_width/(8*sizeof(B))+1);
            generic_end<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::header: {
            buffer.resize(header<B*>::max_bit_width/(8*sizeof(B))+1);
            header<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::header_ext: {
            buffer.resize(header_ext<B*>::max_bit_width/(8*sizeof(B))+1);
            header_ext<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_bat: {
            buffer.resize(hk_bat<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_bat<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_bat_1: {
            buffer.resize(hk_bat_1<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_bat_1<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_bat_2: {
            buffer.resize(hk_bat_2<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_bat_2<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_cam: {
            buffer.resize(hk_cam<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_cam<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_ctr: {
            buffer.resize(hk_ctr<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_ctr<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_ctr_1: {
            buffer.resize(hk_ctr_1<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_ctr_1<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_extended_external: {
            buffer.resize(hk_extended_external<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_extended_external<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_extended_internal: {
            buffer.resize(hk_extended_internal<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_extended_internal<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_gps: {
            buffer.resize(hk_gps<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_gps<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_gps_ts: {
            buffer.resize(hk_gps_ts<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_gps_ts<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_gps_ts_status: {
            buffer.resize(hk_gps_ts_status<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_gps_ts_status<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_gps_ts_status_dop: {
            buffer.resize(hk_gps_ts_status_dop<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_gps_ts_status_dop<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_incl: {
            buffer.resize(hk_incl<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_incl<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_incl_4axes: {
            buffer.resize(hk_incl_4axes<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_incl_4axes<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_pwr: {
            buffer.resize(hk_pwr<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_pwr<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_pwr_1: {
            buffer.resize(hk_pwr_1<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_pwr_1<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rad: {
            buffer.resize(hk_rad<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rad<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rng: {
            buffer.resize(hk_rng<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rng<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rng_1: {
            buffer.resize(hk_rng_1<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rng_1<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rng_2: {
            buffer.resize(hk_rng_2<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rng_2<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rng_3: {
            buffer.resize(hk_rng_3<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rng_3<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rng_4: {
            buffer.resize(hk_rng_4<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rng_4<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rng_5: {
            buffer.resize(hk_rng_5<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rng_5<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rng_6: {
            buffer.resize(hk_rng_6<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rng_6<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rng_7: {
            buffer.resize(hk_rng_7<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rng_7<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rng_8: {
            buffer.resize(hk_rng_8<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rng_8<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rtc: {
            buffer.resize(hk_rtc<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rtc<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_rtc_sys: {
            buffer.resize(hk_rtc_sys<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_rtc_sys<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_scn: {
            buffer.resize(hk_scn<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_scn<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_scn_1: {
            buffer.resize(hk_scn_1<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_scn_1<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::hk_time: {
            buffer.resize(hk_time<B*>::max_bit_width/(8*sizeof(B))+1);
            hk_time<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::ht_dbg_data: {
            buffer.resize(ht_dbg_data<B*>::max_bit_width/(8*sizeof(B))+1);
            ht_dbg_data<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::inclination: {
            buffer.resize(inclination<B*>::max_bit_width/(8*sizeof(B))+1);
            inclination<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::inclination_4axes: {
            buffer.resize(inclination_4axes<B*>::max_bit_width/(8*sizeof(B))+1);
            inclination_4axes<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::inclination_device: {
            buffer.resize(inclination_device<B*>::max_bit_width/(8*sizeof(B))+1);
            inclination_device<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::inclination_device_4axes: {
            buffer.resize(inclination_device_4axes<B*>::max_bit_width/(8*sizeof(B))+1);
            inclination_device_4axes<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::inclination_device_4axes_offset: {
            buffer.resize(inclination_device_4axes_offset<B*>::max_bit_width/(8*sizeof(B))+1);
            inclination_device_4axes_offset<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::laser_echo: {
            buffer.resize(laser_echo<B*>::max_bit_width/(8*sizeof(B))+1);
            laser_echo<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::laser_echo_qual: {
            buffer.resize(laser_echo_qual<B*>::max_bit_width/(8*sizeof(B))+1);
            laser_echo_qual<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::laser_echo_sw: {
            buffer.resize(laser_echo_sw<B*>::max_bit_width/(8*sizeof(B))+1);
            laser_echo_sw<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::laser_shot: {
            buffer.resize(laser_shot<B*>::max_bit_width/(8*sizeof(B))+1);
            laser_shot<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::laser_shot_1angle: {
            buffer.resize(laser_shot_1angle<B*>::max_bit_width/(8*sizeof(B))+1);
            laser_shot_1angle<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::laser_shot_2angles: {
            buffer.resize(laser_shot_2angles<B*>::max_bit_width/(8*sizeof(B))+1);
            laser_shot_2angles<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::laser_shot_2angles_rad: {
            buffer.resize(laser_shot_2angles_rad<B*>::max_bit_width/(8*sizeof(B))+1);
            laser_shot_2angles_rad<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::laser_shot_3angles: {
            buffer.resize(laser_shot_3angles<B*>::max_bit_width/(8*sizeof(B))+1);
            laser_shot_3angles<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::laser_shot_6angles: {
            buffer.resize(laser_shot_6angles<B*>::max_bit_width/(8*sizeof(B))+1);
            laser_shot_6angles<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::line_start: {
            buffer.resize(line_start<B*>::max_bit_width/(8*sizeof(B))+1);
            line_start<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::line_start_dn: {
            buffer.resize(line_start_dn<B*>::max_bit_width/(8*sizeof(B))+1);
            line_start_dn<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::line_start_up: {
            buffer.resize(line_start_up<B*>::max_bit_width/(8*sizeof(B))+1);
            line_start_up<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::line_stop: {
            buffer.resize(line_stop<B*>::max_bit_width/(8*sizeof(B))+1);
            line_stop<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::magnetic_field: {
            buffer.resize(magnetic_field<B*>::max_bit_width/(8*sizeof(B))+1);
            magnetic_field<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::meas_start: {
            buffer.resize(meas_start<B*>::max_bit_width/(8*sizeof(B))+1);
            meas_start<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::meas_stop: {
            buffer.resize(meas_stop<B*>::max_bit_width/(8*sizeof(B))+1);
            meas_stop<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::monitoring_info: {
            buffer.resize(monitoring_info<B*>::max_bit_width/(8*sizeof(B))+1);
            monitoring_info<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::packed_frame_echo: {
            buffer.resize(packed_frame_echo<B*>::max_bit_width/(8*sizeof(B))+1);
            packed_frame_echo<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::packed_frame_laser_shot_2angles: {
            buffer.resize(packed_frame_laser_shot_2angles<B*>::max_bit_width/(8*sizeof(B))+1);
            packed_frame_laser_shot_2angles<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::packed_frame_laser_shot_2angles_rad: {
            buffer.resize(packed_frame_laser_shot_2angles_rad<B*>::max_bit_width/(8*sizeof(B))+1);
            packed_frame_laser_shot_2angles_rad<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::packed_key_echo: {
            buffer.resize(packed_key_echo<B*>::max_bit_width/(8*sizeof(B))+1);
            packed_key_echo<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::packed_key_laser_shot_2angles: {
            buffer.resize(packed_key_laser_shot_2angles<B*>::max_bit_width/(8*sizeof(B))+1);
            packed_key_laser_shot_2angles<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::packed_key_laser_shot_2angles_rad: {
            buffer.resize(packed_key_laser_shot_2angles_rad<B*>::max_bit_width/(8*sizeof(B))+1);
            packed_key_laser_shot_2angles_rad<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::pps_sync: {
            buffer.resize(pps_sync<B*>::max_bit_width/(8*sizeof(B))+1);
            pps_sync<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::pps_sync_ext: {
            buffer.resize(pps_sync_ext<B*>::max_bit_width/(8*sizeof(B))+1);
            pps_sync_ext<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::range_calc: {
            buffer.resize(range_calc<B*>::max_bit_width/(8*sizeof(B))+1);
            range_calc<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::range_finder_debug_acq: {
            buffer.resize(range_finder_debug_acq<B*>::max_bit_width/(8*sizeof(B))+1);
            range_finder_debug_acq<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::range_finder_debug_calc: {
            buffer.resize(range_finder_debug_calc<B*>::max_bit_width/(8*sizeof(B))+1);
            range_finder_debug_calc<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::range_finder_debug_laser: {
            buffer.resize(range_finder_debug_laser<B*>::max_bit_width/(8*sizeof(B))+1);
            range_finder_debug_laser<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::range_finder_debug_rcv: {
            buffer.resize(range_finder_debug_rcv<B*>::max_bit_width/(8*sizeof(B))+1);
            range_finder_debug_rcv<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::range_finder_program: {
            buffer.resize(range_finder_program<B*>::max_bit_width/(8*sizeof(B))+1);
            range_finder_program<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::range_finder_settings: {
            buffer.resize(range_finder_settings<B*>::max_bit_width/(8*sizeof(B))+1);
            range_finder_settings<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::rel_refl_table: {
            buffer.resize(rel_refl_table<B*>::max_bit_width/(8*sizeof(B))+1);
            rel_refl_table<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::sbl_dg_data: {
            buffer.resize(sbl_dg_data<B*>::max_bit_width/(8*sizeof(B))+1);
            sbl_dg_data<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::sbl_dg_data_compressed: {
            buffer.resize(sbl_dg_data_compressed<B*>::max_bit_width/(8*sizeof(B))+1);
            sbl_dg_data_compressed<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::sbl_dg_header: {
            buffer.resize(sbl_dg_header<B*>::max_bit_width/(8*sizeof(B))+1);
            sbl_dg_header<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::scan_rect_fov: {
            buffer.resize(scan_rect_fov<B*>::max_bit_width/(8*sizeof(B))+1);
            scan_rect_fov<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::scanner_pose: {
            buffer.resize(scanner_pose<B*>::max_bit_width/(8*sizeof(B))+1);
            scanner_pose<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::slt_dg: {
            buffer.resize(slt_dg<B*>::max_bit_width/(8*sizeof(B))+1);
            slt_dg<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::slt_dg_1: {
            buffer.resize(slt_dg_1<B*>::max_bit_width/(8*sizeof(B))+1);
            slt_dg_1<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::slt_dg_2: {
            buffer.resize(slt_dg_2<B*>::max_bit_width/(8*sizeof(B))+1);
            slt_dg_2<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::slt_dg_3: {
            buffer.resize(slt_dg_3<B*>::max_bit_width/(8*sizeof(B))+1);
            slt_dg_3<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::tgt_dg: {
            buffer.resize(tgt_dg<B*>::max_bit_width/(8*sizeof(B))+1);
            tgt_dg<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::ublox_lea5t_rxm: {
            buffer.resize(ublox_lea5t_rxm<B*>::max_bit_width/(8*sizeof(B))+1);
            ublox_lea5t_rxm<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::ublox_lea5t_rxm_sfrb: {
            buffer.resize(ublox_lea5t_rxm_sfrb<B*>::max_bit_width/(8*sizeof(B))+1);
            ublox_lea5t_rxm_sfrb<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::units: {
            buffer.resize(units<B*>::max_bit_width/(8*sizeof(B))+1);
            units<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::units_1: {
            buffer.resize(units_1<B*>::max_bit_width/(8*sizeof(B))+1);
            units_1<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::unsolicited_message: {
            buffer.resize(unsolicited_message<B*>::max_bit_width/(8*sizeof(B))+1);
            unsolicited_message<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::unsolicited_message_1: {
            buffer.resize(unsolicited_message_1<B*>::max_bit_width/(8*sizeof(B))+1);
            unsolicited_message_1<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::void_data: {
            buffer.resize(void_data<B*>::max_bit_width/(8*sizeof(B))+1);
            void_data<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::wfm_dg_hp: {
            buffer.resize(wfm_dg_hp<B*>::max_bit_width/(8*sizeof(B))+1);
            wfm_dg_hp<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::wfm_dg_lp: {
            buffer.resize(wfm_dg_lp<B*>::max_bit_width/(8*sizeof(B))+1);
            wfm_dg_lp<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        case package_id::wfm_dg_shp: {
            buffer.resize(wfm_dg_shp<B*>::max_bit_width/(8*sizeof(B))+1);
            wfm_dg_shp<B*> arg(&buffer[0], &buffer[0]+buffer.size());
            in >> arg;
            buffer.resize(arg.end()-&buffer[0]);
        } break;
        default : {
            package_istream_entry ok(in);
            if (ok) {
                std::ios_base::fmtflags f = in.flags();
                in.setf(std::ios_base::hex, std::ios_base::basefield);
                B t;
                while(in.good() && in.peek() == ',') {
                    in >> ',' >> t;
                    buffer.push_back(t);
                }
                in.flags(f);
            }
        }
    }

    return in.good();
}
#endif
#ifndef DOXYGEN
template<class charT, class traits, class B>
bool ridataspec_write(B* begin, B* end, unsigned id_main, unsigned id_sub, std::basic_ostream<charT, traits>& out) {
    package_id::type type = package_id(id_main, id_sub);
    switch(type) {
        case package_id::alert: {
            alert<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::atmosphere: {
            atmosphere<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::atmosphere_1: {
            atmosphere_1<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::avg_fine_ref_dg: {
            avg_fine_ref_dg<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::beam_geometry: {
            beam_geometry<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::calib_waveform: {
            calib_waveform<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::calib_waveform_L2: {
            calib_waveform_L2<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::channel_combination_table: {
            channel_combination_table<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::context_end: {
            context_end<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::counter_sync: {
            counter_sync<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::crc32_check: {
            crc32_check<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::crc32_header: {
            crc32_header<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::datagram_separator: {
            datagram_separator<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::debug_hw_dg: {
            debug_hw_dg<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::debug_sw_dg: {
            debug_sw_dg<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::device_geometry: {
            device_geometry<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::device_mounting: {
            device_mounting<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::echo: {
            echo<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::extents: {
            extents<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::firmware: {
            firmware<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::firmware_1: {
            firmware_1<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::firmware_2: {
            firmware_2<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::firmware_3: {
            firmware_3<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::frame_start: {
            frame_start<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::frame_start_dn: {
            frame_start_dn<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::frame_start_up: {
            frame_start_up<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::frame_stop: {
            frame_stop<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::generic_end: {
            generic_end<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::header: {
            header<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::header_ext: {
            header_ext<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_bat: {
            hk_bat<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_bat_1: {
            hk_bat_1<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_bat_2: {
            hk_bat_2<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_cam: {
            hk_cam<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_ctr: {
            hk_ctr<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_ctr_1: {
            hk_ctr_1<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_extended_external: {
            hk_extended_external<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_extended_internal: {
            hk_extended_internal<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_gps: {
            hk_gps<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_gps_ts: {
            hk_gps_ts<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_gps_ts_status: {
            hk_gps_ts_status<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_gps_ts_status_dop: {
            hk_gps_ts_status_dop<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_incl: {
            hk_incl<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_incl_4axes: {
            hk_incl_4axes<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_pwr: {
            hk_pwr<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_pwr_1: {
            hk_pwr_1<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rad: {
            hk_rad<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rng: {
            hk_rng<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rng_1: {
            hk_rng_1<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rng_2: {
            hk_rng_2<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rng_3: {
            hk_rng_3<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rng_4: {
            hk_rng_4<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rng_5: {
            hk_rng_5<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rng_6: {
            hk_rng_6<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rng_7: {
            hk_rng_7<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rng_8: {
            hk_rng_8<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rtc: {
            hk_rtc<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_rtc_sys: {
            hk_rtc_sys<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_scn: {
            hk_scn<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_scn_1: {
            hk_scn_1<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::hk_time: {
            hk_time<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::ht_dbg_data: {
            ht_dbg_data<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::inclination: {
            inclination<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::inclination_4axes: {
            inclination_4axes<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::inclination_device: {
            inclination_device<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::inclination_device_4axes: {
            inclination_device_4axes<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::inclination_device_4axes_offset: {
            inclination_device_4axes_offset<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::laser_echo: {
            laser_echo<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::laser_echo_qual: {
            laser_echo_qual<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::laser_echo_sw: {
            laser_echo_sw<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::laser_shot: {
            laser_shot<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::laser_shot_1angle: {
            laser_shot_1angle<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::laser_shot_2angles: {
            laser_shot_2angles<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::laser_shot_2angles_rad: {
            laser_shot_2angles_rad<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::laser_shot_3angles: {
            laser_shot_3angles<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::laser_shot_6angles: {
            laser_shot_6angles<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::line_start: {
            line_start<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::line_start_dn: {
            line_start_dn<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::line_start_up: {
            line_start_up<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::line_stop: {
            line_stop<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::magnetic_field: {
            magnetic_field<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::meas_start: {
            meas_start<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::meas_stop: {
            meas_stop<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::monitoring_info: {
            monitoring_info<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::packed_frame_echo: {
            packed_frame_echo<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::packed_frame_laser_shot_2angles: {
            packed_frame_laser_shot_2angles<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::packed_frame_laser_shot_2angles_rad: {
            packed_frame_laser_shot_2angles_rad<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::packed_key_echo: {
            packed_key_echo<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::packed_key_laser_shot_2angles: {
            packed_key_laser_shot_2angles<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::packed_key_laser_shot_2angles_rad: {
            packed_key_laser_shot_2angles_rad<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::pps_sync: {
            pps_sync<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::pps_sync_ext: {
            pps_sync_ext<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::range_calc: {
            range_calc<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::range_finder_debug_acq: {
            range_finder_debug_acq<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::range_finder_debug_calc: {
            range_finder_debug_calc<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::range_finder_debug_laser: {
            range_finder_debug_laser<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::range_finder_debug_rcv: {
            range_finder_debug_rcv<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::range_finder_program: {
            range_finder_program<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::range_finder_settings: {
            range_finder_settings<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::rel_refl_table: {
            rel_refl_table<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::sbl_dg_data: {
            sbl_dg_data<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::sbl_dg_data_compressed: {
            sbl_dg_data_compressed<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::sbl_dg_header: {
            sbl_dg_header<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::scan_rect_fov: {
            scan_rect_fov<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::scanner_pose: {
            scanner_pose<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::slt_dg: {
            slt_dg<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::slt_dg_1: {
            slt_dg_1<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::slt_dg_2: {
            slt_dg_2<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::slt_dg_3: {
            slt_dg_3<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::tgt_dg: {
            tgt_dg<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::ublox_lea5t_rxm: {
            ublox_lea5t_rxm<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::ublox_lea5t_rxm_sfrb: {
            ublox_lea5t_rxm_sfrb<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::units: {
            units<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::units_1: {
            units_1<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::unsolicited_message: {
            unsolicited_message<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::unsolicited_message_1: {
            unsolicited_message_1<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::void_data: {
            void_data<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::wfm_dg_hp: {
            wfm_dg_hp<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::wfm_dg_lp: {
            wfm_dg_lp<B*> arg(begin, end, true);
            out << arg;
        } break;
        case package_id::wfm_dg_shp: {
            wfm_dg_shp<B*> arg(begin, end, true);
            out << arg;
        } break;
        default : {
            std::ios_base::fmtflags f = out.flags();
            out.setf(std::ios_base::showbase);
            out.setf(std::ios_base::hex, std::ios_base::basefield);
            for (B* it = begin; it != end; ++it)
                out << ", " << *it;
            out.flags(f);
        }
    }

    return out.good();
}
#endif

//! INTERNAL ONLY
#define basic_scanner ostream_packets


} // namespace scanlib

#endif /*__cplusplus*/
#endif /* RIDATASPEC_HPP*/
