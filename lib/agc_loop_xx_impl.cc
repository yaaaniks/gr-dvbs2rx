#include "agc_loop_xx_impl.h"
#include <gnuradio/gr_complex.h>
#include <gnuradio/sptr_magic.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/thread/thread.h>
#include <pmt/pmt.h>
#include <unistd.h>
#include <cstring>

namespace gr {
namespace dvbs2rx {

agc_loop_xx::sptr agc_loop_xx::make(int vlen,
                                    float hw_agc_if_gain,
                                    float hw_agc_lna_state,
                                    float sw_agc_rate,
                                    float sw_agc_reference,
                                    float sw_agc_gain,
                                    float sw_agc_max_gain)
{
    return gnuradio::get_initial_sptr(new agc_loop_xx_impl{ vlen,
                                                            hw_agc_if_gain,
                                                            hw_agc_lna_state,
                                                            sw_agc_rate,
                                                            sw_agc_reference,
                                                            sw_agc_gain,
                                                            sw_agc_max_gain });
}

agc_loop_xx_impl::agc_loop_xx_impl(int vlen,
                                   float hw_agc_if_gain,
                                   float hw_agc_lna_state,
                                   float sw_agc_rate,
                                   float sw_agc_reference,
                                   float sw_agc_gain,
                                   float sw_agc_max_gain)
    : gr::sync_block("agc_loop_xx",
                     io_signature::make(1, 1, sizeof(float)),
                     io_signature::make(0, 0, 0)),
      d_current_rf_gain(0),
      d_sw_agc_gain(0),
      d_buffer(size, 0)
{

    /* Message port */
    message_port_register_out(d_port_id);

    d_hw_agc_if_gain = hw_agc_if_gain;
    d_hw_att_state = static_cast<att_state_t>(hw_agc_lna_state);
    d_sw_agc_gain = sw_agc_gain;
}

agc_loop_xx_impl::~agc_loop_xx_impl() {}

int agc_loop_xx_impl::work(int noutput_items,
                           gr_vector_const_void_star& input_items,
                           gr_vector_void_star& output_items)
{
    auto input = reinterpret_cast<const float*>(input_items.data());

    int size = input_items.size();

    std::memcpy(d_buffer.data(), input, size * sizeof(float));

    d_current_rf_gain = calculate_rf_gain(input);
    /**< ?Dummy comparison of RF gain with got parameters and set it on RSP */
    if (d_dummy_cnt++ == 100) {
        d_logger->debug("Current size: {:+f}", size);
        d_dummy_cnt = 0;
        control_rsp_device();
    }
    return noutput_items;
}

void agc_loop_xx_impl::control_rsp_device()
{
    gr::thread::scoped_lock lock{ d_mtx };

    static const pmt::pmt_t if_gain_key = pmt::intern("if_gain");
    static const pmt::pmt_t lna_state_key = pmt::intern("lna_state");

    pmt::pmt_t msg = pmt::make_dict();

    msg = pmt::dict_add(msg, if_gain_key, pmt::from_double(d_current_rf_gain));
    msg = pmt::dict_add(
        msg, lna_state_key, pmt::from_double(static_cast<double>(d_hw_att_state)));

    message_port_pub(d_port_id, msg);
}

double agc_loop_xx_impl::calculate_rf_gain(const float* samples) { return 0; }

double agc_loop_xx_impl::get_current_rf_gain() { return 0; }

double agc_loop_xx_impl::get_hw_if_gain() { return 0; }

double agc_loop_xx_impl::get_hw_lna_gain() { return 0; }

void agc_loop_xx_impl::set_hw_agc_if_gain(const double if_gain) {}

void agc_loop_xx_impl::set_hw_agc_lna_state(const double lha_state) {}

void agc_loop_xx_impl::set_sw_agc_rate(const double agc_rate) {}

void agc_loop_xx_impl::set_sw_agc_reference(const double agc_ref) {}

void agc_loop_xx_impl::set_sw_agc_gain(const double agc_gain) {}

void agc_loop_xx_impl::set_sw_agc_max_gain(const double agc_max_gain) {}


}; // namespace dvbs2rx
}; // namespace gr