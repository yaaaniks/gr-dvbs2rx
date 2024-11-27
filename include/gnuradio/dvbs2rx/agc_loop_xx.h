/**
 * @file agc_loop_xx.h
 * @author Semikozov Ian (y.semikozov@geoscan.ru)
 * @brief
 * @version 0.1
 * @date 24.07.2024
 *
 */
#ifndef AGC_LOOP_XX_H_INCLUDED
#define AGC_LOOP_XX_H_INCLUDED

#include <gnuradio/dvbs2rx/api.h>
#include <gnuradio/sync_block.h>
#include <memory>

namespace gr {
namespace dvbs2rx {

class DVBS2RX_API agc_loop_xx : virtual public sync_block
{
public:
    typedef std::shared_ptr<agc_loop_xx> sptr;
    static sptr make(int vlen,
                     float hw_agc_if_gain,
                     float hw_agc_lna_state,
                     float sw_agc_rate = 1e-4,
                     float sw_agc_reference = 1.0,
                     float sw_agc_gain = 1.0,
                     float sw_agc_max_gain = 0.0);

    virtual double get_current_rf_gain() = 0;

    virtual double get_hw_if_gain() = 0;

    virtual double get_hw_lna_gain() = 0;

    virtual void set_hw_agc_if_gain(const double if_gain) = 0;

    virtual void set_hw_agc_lna_state(const double lha_state) = 0;

    virtual void set_sw_agc_rate(const double agc_rate) = 0;

    virtual void set_sw_agc_reference(const double agc_ref) = 0;

    virtual void set_sw_agc_gain(const double agc_gain) = 0;

    virtual void set_sw_agc_max_gain(const double agc_max_gain) = 0;
};

}; // namespace dvbs2rx
}; // namespace gr

#endif // AGC_LOOP_XX_H_INCLUDED
