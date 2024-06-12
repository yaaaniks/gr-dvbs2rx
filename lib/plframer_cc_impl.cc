#include "../debug_level.h"
#include "../pl_defs.h"
#include "../plsync_cc_impl.h"
#include "pl_correlator.h"
#include "plframer_cc_impl.h"
#include "gnuradio/dvbs2rx/plframer_cc.h"
#include <gnuradio/expj.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/math.h>
#include <pmt/pmt.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace gr {
namespace dvbs2rx {


plframer_cc::sptr
plframer_cc::make(int gold_code, double sps, int debug_level, uint8_t pls_code)
{
    return gnuradio::get_initial_sptr(
        new plframer_cc_impl(gold_code, sps, debug_level, pls_code));
}

plframer_cc_impl::plframer_cc_impl(int gold_code,
                                   double sps,
                                   int debug_level,
                                   uint8_t pls_code)
    : gr::block("plframer_cc",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_debug_level(debug_level),
      d_sps(sps),
      d_locked(false),
      d_payload_state(payload_state_t::searching),
      d_sof_cnt(0),
      d_frame_cnt(0),
      d_rejected_cnt(0),
      d_dummy_cnt(0)
{
    d_ccm_sis_pls = new pls_info_t{ pls_code };


    d_plsc_decoder = new plsc_decoder{ std::vector<uint8_t>{ pls_code }, debug_level };
    d_pl_descrambler = new pl_descrambler{ gold_code };
    d_pl_correlator = new pl_correlator{ debug_level };
    // When the PLSC decoder is disabled, set a fixed PLFRAME length on the frame
    // synchronizer instead of updating the length for every detected frame.
    d_pl_correlator->set_frame_len(d_ccm_sis_pls->plframe_len);
    d_plframe_info.pls = *d_ccm_sis_pls;

    /* Message port */
    message_port_register_out(d_port_id);

    /* This block only outputs data symbols, while pilot symbols are
     * retained. So until we find the need, tags are not propagated */
    set_tag_propagation_policy(TPP_DONT);

    // Make sure the output buffer always fits at least a slot (90 symbols)
    set_output_multiple(SLOT_LEN);
}

plframer_cc_impl::~plframer_cc_impl()
{
    delete d_plsc_decoder;
    delete d_pl_descrambler;
    delete d_pl_correlator;
}

void plframer_cc_impl::handle_plheader(uint64_t abs_sof_idx,
                                       const gr_complex* p_plheader,
                                       plframe_info_t& frame_info)
{
    frame_info.abs_sof_idx = abs_sof_idx;
}


int plframer_cc_impl::handle_payload(int noutput_items,
                                     gr_complex* out,
                                     const gr_complex* p_payload,
                                     plframe_info_t& frame_info)
{
    constexpr int bytes_per_blk = SLOTS_PER_PILOT_BLK * SLOT_LEN * sizeof(gr_complex);

    d_pl_descrambler->descramble(p_payload, frame_info.pls.payload_len);
    const gr_complex* p_descrambled_payload = d_pl_descrambler->get_payload();
    int produced{ 0 }, pilot_cnt{ 0 };

    while (produced <= frame_info.pls.plframe_len) {
        std::memcpy(out + produced,
                    p_descrambled_payload + produced + pilot_cnt * PILOT_BLK_LEN - 1,
                    bytes_per_blk);
        produced += bytes_per_blk;
        pilot_cnt++;
    }

    d_payload_state = payload_state_t::searching;

    return frame_info.pls.payload_len;
}

void plframer_cc_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    if (d_locked) {
        // if current state is searching for frames - just say we need size of frame
        if (d_payload_state == payload_state_t::searching) {
            ninput_items_required[0] =
                d_plframe_info.pls.plframe_len - d_pl_correlator->get_sym_count();
        } else {
            // else if current payload state is found or pending - we need nothing for
            // input
            ninput_items_required[0] = 0;
        }
    } else {
        ninput_items_required[0] = noutput_items;
    }

    ninput_items_required[0] = std::min(noutput_items, ninput_items_required[0]);
}

int plframer_cc_impl::general_work(int noutput_items,
                                   gr_vector_int& ninput_items,
                                   gr_vector_const_void_star& input_items,
                                   gr_vector_void_star& output_items)
{
    const gr_complex* in = reinterpret_cast<const gr_complex*>(input_items[0]);
    gr_complex* out = reinterpret_cast<gr_complex*>(output_items[0]);

    const bool empty_input = ninput_items[0] == 0;
    int consumed{ 0 }, produced{ 0 };
    bool full_output = false;
    while ((consumed < ninput_items[0] ||
            (empty_input && d_payload_state != payload_state_t::searching)) &&
           (produced < noutput_items ||
            (full_output && d_payload_state == payload_state_t::searching))) {
        bool is_sof = d_pl_correlator->step(in[consumed]);
        consumed++;
        d_locked = d_pl_correlator->is_locked();

        if (!is_sof)
            continue;
        this->send_corr_factor(d_pl_correlator->get_timing_metric());
        d_payload_state = payload_state_t::partial;

        // for first sof we cant to produce anything
        static bool first_sof = true;
        if (first_sof) {
            first_sof = false;
            continue;
        }

        produced += this->handle_payload(noutput_items,
                                         out + produced,
                                         d_pl_correlator->get_payload(),
                                         d_plframe_info);
        d_sof_cnt++;
        const uint64_t abs_sof_idx = nitems_read(0) + consumed - PLHEADER_LEN - 1;
        GR_LOG_DEBUG_LEVEL(2, "SOF count: {:d}; Index: {:d}", d_sof_cnt, abs_sof_idx);
    }

    consume_each(consumed);
    return produced;
}

void plframer_cc_impl::send_corr_factor(float&& corr)
{
    static const pmt::pmt_t r_sum_key = pmt::intern("r_sum");
    pmt::pmt_t msg = pmt::make_dict();
    msg = pmt::dict_add(msg, r_sum_key, pmt::from_float(corr));
    message_port_pub(d_port_id, msg);
}

}; // namespace dvbs2rx
}; // namespace gr