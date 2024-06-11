#ifndef PLFRAMER_CC_IMPL_INCLUDED
#define PLFRAMER_CC_IMPL_INCLUDED

#include "../pl_signaling.h"
#include "../plsync_cc_impl.h"
#include "pl_correlator.h"
#include "gnuradio/dvbs2rx/plframer_cc.h"
#include <gnuradio/gr_complex.h>

namespace gr {
namespace dvbs2rx {

class plframer_cc_impl : public plframer_cc
{
private:
    /* Parameters */
    int d_debug_level;  /** debug level */
    const double d_sps; /** samples per symbol */
    uint64_t d_pls_filter_lo{ 0xFFFFFFFFFFFFFFFF };
    uint64_t d_pls_filter_hi{ 0xFFFFFFFFFFFFFFFF };
    /* NOTE: the PLSYNC block requires a symbol-spaced stream at its
     * input. Hence, sps does not refer to the input stream. Instead, it
     * refers to the oversampling ratio that is adopted in the receiver
     * flowgraph prior to the matched filter. This is so that this block
     * can control the external rotator phase properly */
    bool d_acm_vcm;                                      /**< ACM/VCM mode */
    std::array<uint8_t, n_plsc_codewords> d_pls_enabled; /** PLSs to process */
    bool d_plsc_decoder_enabled{ true }; /**< Whether the PLSC decoder is enabled */

    /* State */
    bool d_locked{ false };      /**< Whether the frame timing is locked */
    bool d_closed_loop{ false }; /**< Whether any freq. correction has been applied to the
                             external rotator. False while still waiting for the first
                             correction (i.e., while effectively in open loop) */
    gr_complex d_phase_corr;
    payload_state_t d_payload_state{
        payload_state_t::searching
    }; /**< Payload processing state machine */
    plframe_idx_t d_idx; /**< PLFRAME index state */

    /* Frame counts */
    uint64_t d_sof_cnt{ 0 };      /**< Total detected SOFs (including false-positives) */
    uint64_t d_frame_cnt{ 0 };    /**< Accepted/processed PLFRAMEs */
    uint64_t d_rejected_cnt{ 0 }; /**< Rejected PLFRAMEs */
    uint64_t d_dummy_cnt{ 0 };    /**< Dummy PLFRAMEs */

    /* Frame metadata from the current PLFRAME (whose payload may be under processing if
     * locked) and from the next PLFRAME (the PLHEADER ahead, processed in advance). */
    plframe_info_t d_curr_frame_info; /**< PLFRAME under processing */
    plframe_info_t d_next_frame_info; /**< Next PLFRAME */

    // Constant PLS info used in CCM/SIS mode
    pls_info_t d_ccm_sis_pls;

    plsc_decoder* d_plsc_decoder{ nullptr };     /**< PLSC decoder */
    plsc_encoder* d_plsc_encoder{ nullptr };     /*< PLSC encoder */
    freq_sync* d_freq_sync{ nullptr };           /*< Frequency synchronizer*/
    pl_descrambler* d_pl_descrambler{ nullptr }; /**< PL descrambler */
    pl_correlator* d_pl_correlator{ nullptr };

    volk::vector<gr_complex> pp_plheader; /**< derotated PLHEADER symbols */

    const pmt::pmt_t d_port_id = pmt::mp("pls_corr");
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

    void derotate_plheader(const gr_complex* in, bool open_loop);

    /**
     * \brief Estimate the average phase of the SOF.
     * \param in (gr_complex *) Pointer to the SOF symbol array.
     * \return (float) The phase estimate in radians within -pi to +pi.
     */
    float estimate_sof_phase(const gr_complex* in);
    float estimate_phase_data_aided(const gr_complex* in,
                                    const gr_complex* expected,
                                    unsigned int len);

public:
    plframer_cc_impl(int gold_code,
                     double sps,
                     int debug_level,
                     bool acm_vcm,
                     uint64_t pls_filter_lo,
                     uint64_t pls_filter_hi,
                     uint64_t pls_code);
    ~plframer_cc_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);
    void send_message(gr_complex&& corr);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

    bool get_locked() { return d_locked; }
    uint64_t get_sof_count() { return d_sof_cnt; }
    uint64_t get_frame_count() { return d_frame_cnt; }
    uint64_t get_rejected_count() { return d_rejected_cnt; }
    uint64_t get_dummy_count() { return d_dummy_cnt; }
};

}; // namespace dvbs2rx
}; // namespace gr

#endif // PLFRAMER_CC_IMPL_INCLUDED