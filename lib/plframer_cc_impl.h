/* -*- c++ -*- */
/*
 * Copyright (c) 2019-2023 Igor Freire.
 *
 * This file is part of gr-dvbs2rx.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DVBS2RX_PLFRAMER_CC_IMPL_H
#define INCLUDED_DVBS2RX_PLFRAMER_CC_IMPL_H

#include "pl_defs.h"
#include "pl_descrambler.h"
#include "pl_frame_sync.h"
#include "pl_freq_sync.h"
#include "pl_signaling.h"
#include "plsync_cc_impl.h"
#include "gnuradio/dvbs2rx/plframer_cc.h"
#include <gnuradio/dvbs2rx/plsync_cc.h>
#include <gnuradio/gr_complex.h>
#include <volk/volk_alloc.hh>
#include <array>
#include <queue>

namespace gr {
namespace dvbs2rx {

class plframer_cc_impl : public plframer_cc
{
private:
    /* Parameters */
    int d_debug_level;  /** debug level */
    const double d_sps; /** samples per symbol */
    /* NOTE: the PLSYNC block requires a symbol-spaced stream at its
     * input. Hence, sps does not refer to the input stream. Instead, it
     * refers to the oversampling ratio that is adopted in the receiver
     * flowgraph prior to the matched filter. This is so that this block
     * can control the external rotator phase properly */
    std::array<uint8_t, n_plsc_codewords> d_pls_enabled; /** PLSs to process */
    bool d_plsc_decoder_enabled; /**< Whether the PLSC decoder is enabled */

    /* State */
    bool d_locked;      /**< Whether the frame timing is locked */
    bool d_closed_loop; /**< Whether any freq. correction has been applied to the
                           external rotator. False while still waiting for the first
                           correction (i.e., while effectively in open loop) */
    payload_state_t d_payload_state; /**< Payload processing state machine */
    rot_ctrl_t d_rot_ctrl;           /**< Upstream rotator control */
    plframe_idx_t d_idx;             /**< PLFRAME index state */
    gr_complex d_phase_corr;         /**< Phase correction */
    double d_cum_freq_offset;        /**< Cumulative frequency offset estimate */

    /* Frame counts */
    uint64_t d_sof_cnt;      /**< Total detected SOFs (including false-positives) */
    uint64_t d_frame_cnt;    /**< Accepted/processed PLFRAMEs */
    uint64_t d_rejected_cnt; /**< Rejected PLFRAMEs */
    uint64_t d_dummy_cnt;    /**< Dummy PLFRAMEs */

    /* Frame metadata from the current PLFRAME (whose payload may be under processing if
     * locked) and from the next PLFRAME (the PLHEADER ahead, processed in advance). */
    plframe_info_t d_curr_frame_info; /**< PLFRAME under processing */
    plframe_info_t d_next_frame_info; /**< Next PLFRAME */

    // Constant PLS info used in CCM/SIS mode
    pls_info_t d_ccm_sis_pls;

    const pmt::pmt_t d_port_id = pmt::mp("corr");

    /* Objects */
    frame_sync* d_frame_sync;         /**< frame synchronizer */
    freq_sync* d_freq_sync;           /**< frequency synchronizer */
    plsc_decoder* d_plsc_decoder;     /**< PLSC decoder */
    plsc_encoder* d_plsc_encoder;     /**< PLSC encoder */
    pl_descrambler* d_pl_descrambler; /**< PL descrambler */
    /**
     * @brief Save the tags in the current work range within the local queue
     *
     * The motivation is to avoid missing tags due to pruning. The PL sync block processes
     * tags only when locked (via the `calibrate_tag_delay` function). Hence, when tags
     * are placed while the block is unlocked, these tags can be occasionally lost due to
     * GR runtime's block tag pruning. This function avoids the problem by saving all the
     * desired tags ("rot_phase_inc" tags) locally on a queue to be processed later by the
     * calibrate_tag_delay function.
     *
     * @param ninput_items (int) Number of samples available on the input buffer.
     */
    void handle_tags(int ninput_items);

    /**
     * @brief Send new frequency correction to the upstream rotator.
     *
     * The PL Sync block is designed to be used in conjunction with a rotator
     * block for frequency correction. The processing chain is expected to be
     * somewhat similar to the following:
     *
     *  AGC --> Rotator --> Symbol Sync (MF/Decimator) --> DVB-S2 PL Sync
     *
     * That is, the rotator is upstream after an intermediate symbol sync block,
     * which, in turn, implements symbol timing recovery, matched filtering, and
     * decimation. As a result, the frequency correction applied by the rotator
     * promotes better performance both on the symbol timing recovery and on the
     * DVB-S2 PL synchronization.
     *
     * In this architecture, the PL Sync block is responsible for controlling
     * the rotator frequency through a message mechanism. This function
     * implements such control.
     *
     * More specifically, this function coordinates the moment when the
     * frequency correction is supposed to change on the upstream rotator. If
     * the frequency correction was allowed to change at any moment, it could
     * change in the middle of a frame and degrade the decoding performance
     * significantly. Hence, this function attempts to schedule every change at
     * the start of a PLHEADER instead. To do so, it relies on the tag delay
     * calibrated by function `calibrate_tag_delay()`.
     *
     * @param abs_sof_idx (uint64_t) Absolute index where the SOF starts.
     * @param plframe_len (uint16_t) Corresponding PLFRAME length.
     * @param rot_freq_adj (double) Frequency adjustment to be applied.
     * @param ref_is_past_frame (bool) Whether the reference for the frequency
     *                           adjustment is the previous frame (when true) or
     *                           the frame corresponding to the most recently
     *                           handled PLHEADER (when false).
     *
     * @note A relative index is relative to the start of the current work
     * buffer. An absolute index is counted monotonically since the start of the
     * flowgraph execution. The `abs_sof_idx` parameter is an absolute index.
     *
     * @note The current architecture only processes a PLFRAME payload when both
     * the preceding and succeeding SOFs are detected. Hence, by the time the
     * n-th PLHEADER is processed, the payload behind it (payload n-1) is yet to
     * be processed. However, both the header and payload can trigger rotator
     * frequency adjustments. The header can produce a coarse frequency offset
     * update, whereas the payload can produce a fine estimate in case it has
     * pilots. Hence, care should be taken when applying parameter
     * `rot_freq_adj`.
     *
     * Regardless of which handler calls this function (header or payload), the
     * frequency adjustment is always scheduled for the start of PLHEADER
     * n+1. However, the adjustment itself can be summed to either the frequency
     * offset state at frame n-1 or frame n. The coarse estimate produced by the
     * PLHEADER handler must be added to the frequency state at frame n (at the
     * most recently processed PLHEADER). Hence, for the PLHEADER,
     * `ref_is_past_frame` must be false. In contrast, the pilot-mode fine
     * estimate produced by the payload handler must be added to the frequency
     * state of frame n-1 (i.e., the PLFRAME preceding the most recent
     * PLHEADER). In this case, `ref_is_past_frame` must be true.
     */
    void control_rotator_freq(uint64_t abs_sof_idx,
                              uint16_t plframe_len,
                              double rot_freq_adj,
                              bool ref_is_past_frame);

    /**
     * @brief Calibrate the delay between the upstream rotator and this block.
     *
     * The rotator places a tag on the sample where the new phase increment
     * starts to take effect. However, this sample typically traverses a matched
     * filter and decimator block, which can alter the tag index. Hence, the tag
     * may not come exactly where expected.
     *
     * This function calibrates the tag delay by comparing the index on which
     * the tag came to where it was expected in the symbol-spaced stream.
     * Ultimately, the measured tag delay can be used to pre-compensate for it
     * when scheduling the next phase increment update.
     *
     * @param abs_sof_idx Absolute index where the SOF was detected.
     * @param tolerance Tag delay tolerance. If exceeded, a warning is printed.
     *
     * @note This function should be called whenever a SOF is detected.
     **/
    void calibrate_tag_delay(uint64_t abs_sof_idx, int tolerance = 300);

    /**
     * @brief Process a PLHEADER.
     * @param abs_sof_idx (uint64_t) Absolute index where the PLHEADER starts.
     * @param p_plheader (const gr_complex*) Pointer to the PLHEADER buffer.
     * @param frame_info (plframe_info&) Reference to the plframe_info_t object
     *                   on which the frame information should be cached once
     *                   the PLHEADER is decoded.
     */
    void handle_plheader(uint64_t abs_sof_idx,
                         const gr_complex* p_plheader,
                         plframe_info_t& frame_info);

    /**
     * @brief Process a PLFRAME payload (data and pilot symbols).
     * @param noutput_items (int) Output buffer capacity.
     * @param out (gr_complex*) Pointer to the output buffer.
     * @param p_payload (gr_complex*) Pointer to the payload to be processed.
     * @param frame_info (plframe_info_t&) Reference to the plframe_info_t
     *                   object with the information corresponding to the
     *                   payload being processed.
     * @param next_frame_info (plframe_info_t&) Reference to the plframe_info_t
     *                        object with the information corresponding to the
     *                        frame ahead of the frame being processed. Used
     *                        only to schedule rotator frequency updates.
     * @return (int) Number of output symbols produced in this call.
     */
    int handle_payload(int noutput_items,
                       gr_complex* out,
                       const gr_complex* p_payload,
                       plframe_info_t& frame_info,
                       const plframe_info_t& next_frame_info);


public:
    plframer_cc_impl(int gold_code,
                     int freq_est_period,
                     double sps,
                     int debug_level,
                     uint8_t pls_code);
    ~plframer_cc_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

    /**
     * @brief Get the cumulative frequency offset.
     *
     * When an external rotator is used to handle the frequency corrections,
     * eventually the PL Sync block estimates low frequency offsets once the
     * external frequency corrections start to take effect. This function
     * returns not the last frequency offset estimate but the actual cumulative
     * frequency offset configured on the external rotator.
     *
     * @return (float) Cumulative frequency offset.
     */
    float get_freq_offset() const { return d_cum_freq_offset; }

    /* Other externally readable stats */
    bool get_coarse_freq_corr_state() const { return d_freq_sync->is_coarse_corrected(); }
    bool get_locked() const { return d_locked; }
    uint64_t get_sof_count() const { return d_sof_cnt; }
    uint64_t get_frame_count() const { return d_frame_cnt; }
    uint64_t get_rejected_count() const { return d_rejected_cnt; }
    uint64_t get_dummy_count() const { return d_dummy_cnt; }
    std::chrono::system_clock::time_point get_lock_time() const
    {
        return d_frame_sync->get_lock_time();
    };
};

} // namespace dvbs2rx
} // namespace gr

#endif /* INCLUDED_DVBS2RX_PLFRAMER_CC_IMPL_H */
