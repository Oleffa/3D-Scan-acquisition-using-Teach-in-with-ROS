// $Id: fullwave.hpp 440 2010-05-25 11:37:08Z rs $

#ifndef FULLWAVE_HPP
#define FULLWAVE_HPP

#include <riegl/config.hpp>
#include <riegl/ridataspec.hpp>

namespace scanlib {

class RIVLIB_API fullwave
    : virtual public basic_packets
{
public:

    fullwave();
    fullwave(std::ostream& out);

    uint32_t threshold;
    unsigned searchindex;

protected:

    // overridden
    void on_range_calc(const range_calc<iterator_type>& arg);
    void on_laser_shot(const laser_shot<iterator_type>& arg);
    void on_laser_shot_1angle(const laser_shot_1angle<iterator_type>& arg);
    void on_laser_shot_2angles(const laser_shot_2angles<iterator_type>& arg);
    void on_laser_shot_2angles_rad(const laser_shot_2angles_rad<iterator_type>& arg);
    void on_wfm_dg_lp(const wfm_dg_lp<iterator_type>& arg);
    void on_wfm_dg_hp(const wfm_dg_hp<iterator_type>& arg);
    void on_wfm_dg_shp(const wfm_dg_shp<iterator_type>& arg);

    // most recent packages, and received indicators
    bool have_lp;
    bool have_hp;
    bool have_shp;
    wfm_dg_lp<> mr_wfm_dg_lp;
    wfm_dg_hp<> mr_wfm_dg_hp;
    wfm_dg_shp<> mr_wfm_dg_shp;

    double scale;
    unsigned full_scale_adc_lp;
    unsigned full_scale_adc_hp;
    unsigned full_scale_adc_shp;

    std::ostream* out;
    bool header_written;

private:
	void init();
    typedef decoder_rxpmarker::iterator synthetic_it;
    enum { synthetic_buf_size = 16 };
    decoder_rxpmarker::value_type synthetic_buf[synthetic_buf_size];

    void synthesize_echo();

    // not copyable
    fullwave(const fullwave&);
};

} // namespace scanlib

#endif // FULLWAVE_HPP
