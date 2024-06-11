#include "debug_level.h"
#include "pl_correlator.h"
#include "plframer_cc_impl.h"
#include "gnuradio/dvbs2rx/plframer_cc.h"
#include <gnuradio/expj.h>
#include <gnuradio/math.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <cstring>

namespace gr {
namespace dvbs2rx {


plframer_cc::sptr plframer_cc::make(int gold_code,
                                    double sps,
                                    int debug_level,
                                    bool acm_vcm,
                                    uint64_t pls_filter_lo,
                                    uint64_t pls_filter_hi,
                                    uint64_t pls_code)
{
    return gnuradio::get_initial_sptr(new plframer_cc_impl(
        gold_code, sps, debug_level, acm_vcm, pls_filter_lo, pls_filter_hi, pls_code));
}

plframer_cc_impl::plframer_cc_impl(int gold_code,
                                   double sps,
                                   int debug_level,
                                   bool acm_vcm,
                                   uint64_t pls_filter_lo,
                                   uint64_t pls_filter_hi,
                                   uint64_t pls_code)
    : gr::block("plframer_cc",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_debug_level(debug_level),
      d_sps(sps),
      d_pls_filter_lo(pls_filter_lo),
      d_pls_filter_hi(pls_filter_hi),
      d_acm_vcm(false),
      d_plsc_decoder_enabled(false),
      d_locked(false),
      d_closed_loop(false),
      d_payload_state(payload_state_t::searching),
      d_sof_cnt(0),
      d_frame_cnt(0),
      d_rejected_cnt(0),
      d_dummy_cnt(0)
{
    // Validate the PLS filters based on their population counts (Hamming weights)
    //
    // NOTE: a popcnt of 2 is ok in CCM if the target PLSs differ on the pilots flag
    // only. This is verified in the sequel after the PLS filters are parsed. On the
    // other hand, a popcnt greater than 2 is certainly a problem in CCM mode.
    // uint64_t popcnt1, popcnt2;
    // volk_64u_popcnt(&popcnt1, pls_filter_lo);
    // volk_64u_popcnt(&popcnt2, pls_filter_hi);
    // if (!acm_vcm && (popcnt1 + popcnt2) > 2) {
    //     throw std::runtime_error{
    //         "PLS filter configured for multiple MODCOD or frame sizes in CCM mode"
    //     };
    // }
    // if ((popcnt1 + popcnt2) == 0) {
    //     throw std::runtime_error{ "At least one PLS should be enabled in the filters"
    //     };
    // }
    gr_complex encoded_plsc[64];
    d_plsc_encoder = new plsc_encoder{};
    d_plsc_encoder->encode(encoded_plsc, static_cast<uint8_t>(pls_code & 0xFF));

    // PLS filters
    //
    // NOTE the "d_pls_enabled" and "expected_plsc" arrays play distinct roles:
    //
    // - The "d_pls_enabled" array determines whether this block shall output
    // XFECFRAMEs
    //   embedded on PLFRAMEs with the given PLS.
    //
    // - The "expected_plsc" vector specifies the distinct PLS values that can be
    // found in
    //   the input stream, which are equivalent to the expected PLSC (codeword)
    //   indexes to be processed by the PLSC decoder. More specifically, the
    //   "expected_plsc" vector holds a priori knowledge used to reduce the set of
    //   possibilities searched by the PLSC decoder. In ACM/VCM mode, it will either
    //   contain all 128 possible DVB-S2 PLS values, or a selected number of them. In
    //   CCM mode, it must contain a single value, in which case the PLSC decoder is
    //   effectively disabled for simplicity.
    //
    // - When operating in CCM mode with multiple TS streams (MIS mode), instead of
    // single
    //   stream (SIS) mode, the input PLFRAME stream may contain dummy frames in
    //   addition to the selected PLS. Similarly, in ACM/VCM mode, even if running
    //   with a single TS stream, dummy frames must be expected and processed by the
    //   PLSC decoder). See the second row in Table D.2 of the standard.
    std::vector<uint8_t> expected_plsc;
    for (uint8_t pls = 0; pls < n_plsc_codewords; pls++) {
        bool enabled = (pls < 64) ? (pls_filter_lo & (1ULL << pls))
                                  : (pls_filter_hi & (1ULL << (pls - 64)));
        d_pls_enabled[pls] = enabled;
        if (enabled) {
            expected_plsc.push_back(pls);
        }
    }

    // In CCM mode, up to two PLSs are allowed, as long as they refer to the same MODCOD
    // and frame size (differring only on the pilots flag).
    if (!acm_vcm && expected_plsc.size() == 2) {
        pls_info_t info1(expected_plsc[0]);
        pls_info_t info2(expected_plsc[1]);
        if (info1.modcod != info2.modcod ||
            info1.short_fecframe != info2.short_fecframe) {
            throw std::runtime_error(
                "A single MODCOD and frame size should be selected in "
                "CCM mode");
        }
    }
    // Include the PLSs of the dummy PLFRAMEs allowed in MIS or ACM/VCM mode
    //
    // Dummy PLFRAMES have modcod=0, so the corresponding PLS should be 0. However, there
    // are no guarantees that the Tx sets short_fecframe=0 and pilots=0 when sending dummy
    // frames, so PLS values from 0 to 3 can be expected (TODO: confirm).
    // If a single PLS is expected in the end (CCM/SIS mode with pilot configuration
    // known), then cache the constant PLS info and simply disable the PLSC decoder. It
    // would return the same PLS for every frame anyway.
    d_plsc_decoder_enabled = false;
    d_ccm_sis_pls = pls_info_t(static_cast<uint8_t>(pls_code));

    std::sort(expected_plsc.begin(), expected_plsc.end());

    d_plsc_decoder = new plsc_decoder{ std::move(expected_plsc), debug_level };
    d_pl_descrambler = new pl_descrambler{ gold_code };
    d_pl_correlator = new pl_correlator{ debug_level };
    // When the PLSC decoder is disabled, set a fixed PLFRAME length on the frame
    // synchronizer instead of updating the length for every detected frame.
    // if (!d_plsc_decoder_enabled) {
    d_pl_correlator->set_frame_len(d_ccm_sis_pls.plframe_len);
    // }

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
    frame_info.pls = d_ccm_sis_pls;
}


int plframer_cc_impl::handle_payload(int noutput_items,
                                     gr_complex* out,
                                     const gr_complex* p_payload,
                                     plframe_info_t& frame_info,
                                     const plframe_info_t& next_frame_info)
{
    d_pl_descrambler->descramble(p_payload, frame_info.pls.payload_len);
    auto descrambled = d_pl_descrambler->get_payload();
    std::memcpy(out, descrambled, frame_info.pls.payload_len);

    return frame_info.pls.payload_len;
}


// Where all the action really happens
void plframer_cc_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    if (d_locked) {
        // The work function can process multiple PLFRAMEs in one call. Hence, the number
        // of required input symbols depends on the output space available for the current
        // and future frames. Let's start by the requirement for the current frame.
        if (d_payload_state == payload_state_t::searching) {
            // If the frame synchronizer is locked, and while still searching for the next
            // payload, the work function will only produce output if it processes enough
            // symbols to actually find a payload between two consecutive SOFs. Hence,
            // require the symbols remaining until the end of the subsequent PLHEADER.
            ninput_items_required[0] = d_next_frame_info.pls.payload_len -
                                       d_pl_correlator->get_sym_count() + PLHEADER_LEN;
        } else {
            // The PL Sync block has the full payload buffered internally and is trying to
            // complete the payload processing. Hence, it doesn't need any further input
            // symbols for the current frame.
            ninput_items_required[0] = 0;
        }
        // If the output buffer can fit more than the remaining samples of the current
        // XFECFRAME (i.e., `n_remaining` below), the work function will be able to
        // proceed to the next PLFRAME. In this case, increase the input size forecast
        // assuming the subsequent frame(s) will have the same length as the current (in
        // VCM/ACM mode, this will likely be a poor forecast).
        int n_remaining = d_next_frame_info.pls.xfecframe_len - (d_idx.i_slot * SLOT_LEN);
        int n_excess = noutput_items - n_remaining; // beyond the current XFECFRAME
        if (n_excess > 0) {
            int n_extra_frames =
                1 + ((n_excess - 1) / d_next_frame_info.pls.xfecframe_len); // ceil
            ninput_items_required[0] +=
                d_next_frame_info.pls.plframe_len * n_extra_frames;
        }
    } else {
        // While the frame synchronizer is still trying to lock, assume conservatively
        // that we need an equal number of input items as output items. In reality,
        // however, at this point, it's more likely we will only consume the input but
        // won't produce any output until locked.
        ninput_items_required[0] = noutput_items;
    }

    // Unfortunately, it seems the runtime executor won't let the forecast request more
    // input items than output items (sort of, it's a little more complex than that). As a
    // workaround, cap the required input items based on the available output items.
    ninput_items_required[0] = std::min(ninput_items_required[0], noutput_items);
}

int plframer_cc_impl::general_work(int noutput_items,
                                   gr_vector_int& ninput_items,
                                   gr_vector_const_void_star& input_items,
                                   gr_vector_void_star& output_items)
{
    int produced{ 0 }, consumed{ 0 };
    auto in = reinterpret_cast<const gr_complex*>(input_items[0]);
    auto out = reinterpret_cast<gr_complex*>(output_items[0]);
    const bool empty_input = ninput_items[0] == 0;
    bool full_output = false;
    while ((consumed < ninput_items[0] ||
            (empty_input && d_payload_state != payload_state_t::searching)) &&
           (produced < noutput_items ||
            (full_output && d_payload_state == payload_state_t::searching))) {
        // If there is no payload waiting to be processed, consume the input stream until
        // the next SOF/PLHEADER is found by the frame synchronizer.
        if (d_payload_state == payload_state_t::searching) {
            for (int i = consumed; i < ninput_items[0]; i++) {
                // Step the frame synchronizer and keep refreshing the locked state. It
                // could change any time.
                bool is_sof = d_pl_correlator->step(in[i]);
                // this->send_message(d_pl_correlator->get_timing_metric());
                d_locked = d_pl_correlator->is_locked();
                consumed++;
                if (!is_sof)
                    continue;
                d_sof_cnt++;


                // Convert the relative SOF detection index to an absolute index
                // corresponding to the first SOF/PLHEADER symbol. Consider that
                // the SOF detection happens at the last PLHEADER symbol.
                const uint64_t abs_sof_idx = nitems_read(0) + i - 89;
                GR_LOG_DEBUG_LEVEL(
                    2, "SOF count: {:d}; Index: {:d}", d_sof_cnt, abs_sof_idx);

                // Cache some information from the last PLFRAME before handling the new
                // PLHEADER. As soon as the PLHEADER is handled, the PLSC decoder will
                // update its PL signaling info, and other state variables will change.
                // However, because we always process the payload between two SOFs, the
                // payload of interest is the one corresponding to the preceding PLHEADER,
                // not the succeeding PLHEADER that is about to be handled.
                //
                // In addition to the PL signaling information, the following variables
                // also need to be cached:
                //
                // - The coarse corrected state. We want to know if the frequency
                //   synchronizer was already coarse-corrected at the time of the PLFRAME
                //   being processed, not at the time of the next PLHEADER that is about
                //   to be processed.
                //
                // - The PLHEADER sequence itself. We need it in order to estimate the
                //   PLHEADER phase when processing the payload.
                //
                // This metadata is contained within the plframe_info_t structure. Once
                // `handle_plheader()` is called, it updates `d_next_frame_info`. Hence,
                // cache the `d_next_frame_info` from the previous SOF, which is the
                // current frame to be processed by the `handle_payload()` function.
                // d_curr_frame_info = d_next_frame_info;

                // The PLHEADER can always be processed right away because it doesn't
                // produce any output (no need to worry about noutput_items). Also, at
                // this point, and only at this point, the PLHEADER is available inside
                // the frame synchronizer and can be fetched via the
                // `frame_sync.get_plheader()` method.
                // handle_plheader(
                //     abs_sof_idx, d_pl_correlator->get_plheader(), d_next_frame_info);

                // If this is the first SOF ever, keep going until the next. We take the
                // PLFRAME payload as the sequence of symbols between two SOFs. Hence, we
                // need at least two SOF detections.
                static bool first_sof = true;
                if (first_sof) {
                    first_sof = false;
                    continue;
                }


                auto pl = d_pl_correlator->get_payload();

                // std::memcpy(out, pl, d_ccm_sis_pls.payload_len);
                // produced += d_ccm_sis_pls;
                // Before locking, it's hard to tell with some confidence that a valid
                // PLFRAME lies between the preceding and the current SOF. Keep going
                // until frame lock is achieved.
                // out += d_ccm_sis_pls.plframe_len;
                if (!d_locked)
                    continue;

                // Reject the frame if its PLS value is not enabled for processing. In CCM
                // mode, this rejection ensures the downstream blocks won't get any
                // accidental XFECFRAME of differing size, which could break the block's
                // notion of the XFECFRAME boundaries. In ACM/VCM mode, this rejection is
                // useful to prevent wrong frame detection when it's known a priori that
                // certain PLS values cannot be found in the input stream.


                // if (!d_pls_enabled[d_curr_frame_info.pls.plsc]) {
                //     // GR_LOG_DEBUG_LEVEL(
                //     //     2, "PLFRAME rejected (PLS={:d})",
                //     d_curr_frame_info.pls.plsc); d_rejected_cnt++; continue;
                // }

                // If the PLFRAME between the present and the past SOFs is a dummy frame,
                // it doesn't produce any output. Skip it and keep going until the next.


                // if (d_curr_frame_info.pls.dummy_frame) {
                //     d_dummy_cnt++;
                //     continue;
                // }

                // The payload can only be processed if the expected XFECFRAME output fits
                // in the output buffer. Hence, unlike the PLHEADER, it may not be
                // processed right away. Mark the processing as pending for now and don't
                // consume any more input samples until this payload is handled.
                d_payload_state = payload_state_t::pending;
                d_frame_cnt++;
                produced += d_ccm_sis_pls.payload_len;

                // If running in ACM/VCM mode, tag the beginning of the XFECFRAME to
                // follow in the output. Include the MODCOD and the FECFRAME length so
                // that downstream blocks can de-map and decode this frame.
                break;
            }
        }

        // if (d_payload_state != payload_state_t::searching) {
        //     produced +=
        //         handle_payload((noutput_items - produced), // remaining output items
        //                        out + produced, // pointer to the next output item
        //                        d_pl_correlator->get_payload(), // buffered frame
        //                        payload d_curr_frame_info, d_next_frame_info);
        //     assert(produced <= noutput_items);
        //     full_output = produced == noutput_items;
        // }
    }
    GR_LOG_DEBUG_LEVEL(
        1, "Consumed items: {:d}; Produced items: {:d}", consumed, produced);
    consume_each(consumed);

    return produced;
}

void plframer_cc_impl::send_message(gr_complex&& corr)
{
    static const pmt::pmt_t inc_key = pmt::intern("r_sum");
    pmt::pmt_t msg = pmt::make_dict();
    msg = pmt::dict_add(msg, inc_key, pmt::from_complex(corr));
    message_port_pub(d_port_id, msg);
}

}; // namespace dvbs2rx
}; // namespace gr