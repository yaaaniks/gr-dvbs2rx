/**
 * @file agc_loop_xx_impl.h
 * @author Semikozov Ian (y.semikozov@geoscan.ru)
 * @brief
 * @version 0.1
 * @date 24.07.2024
 *
 */
#ifndef AGC_LOOP_XX_IMPL_H_INCLUDED
#define AGC_LOOP_XX_IMPL_H_INCLUDED

#include <gnuradio/dvbs2rx/agc_loop_xx.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/thread/thread.h>
#include <atomic>
#include <vector>

namespace gr {
namespace dvbs2rx {

enum class att_state_t { att0 = 5, att1 = 19, att2 = 24, att3 = 43 };

class agc_loop_xx_impl : public agc_loop_xx
{
public:
    agc_loop_xx_impl(int vlen,
                     float hw_agc_if_gain,
                     float hw_agc_lna_state,
                     float sw_agc_rate = 1e-4,
                     float sw_agc_reference = 1.0,
                     float sw_agc_gain = 1.0,
                     float sw_agc_max_gain = 0.0);
    ~agc_loop_xx_impl();

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);

    double get_current_rf_gain();

    double get_hw_if_gain();

    double get_hw_lna_gain();

    void set_hw_agc_if_gain(const double if_gain);

    void set_hw_agc_lna_state(const double lha_state);

    void set_sw_agc_rate(const double agc_rate);

    void set_sw_agc_reference(const double agc_ref);

    void set_sw_agc_gain(const double agc_gain);

    void set_sw_agc_max_gain(const double agc_max_gain);

private:
    double calculate_rf_gain(const float* samples);
    void control_rsp_device();

private:
    double d_current_rf_gain; /**< Center frequency gain [dB] */
    double d_hw_agc_if_gain;  /**< Hardware manual intermediate frequency gain control */
    double d_sw_agc_gain;     /**< Software AGC gain */
    att_state_t d_hw_att_state; /**< Hardware low noise amplifier manual gain control */

    std::vector<float> d_buffer; /**< Buffer with signal's magnitudes from FFT block */

    gr::thread::mutex d_mtx;
    gr::thread::condition_variable d_cv;
    std::atomic_bool d_enable;
    int d_dummy_cnt{ 0 };
    const pmt::pmt_t d_port_id = pmt::mp("gain_cmd");
};

}; // namespace dvbs2rx
}; // namespace gr

#endif // AGC_LOOP_XX_IMPL_H_INCLUDED
