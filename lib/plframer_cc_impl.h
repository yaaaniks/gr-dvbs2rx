#ifndef PLFRAMER_CC_IMPL_INCLUDED
#define PLFRAMER_CC_IMPL_INCLUDED

#include "../pl_signaling.h"
#include "../plsync_cc_impl.h"
#include "pl_correlator.h"
#include "gnuradio/dvbs2rx/plframer_cc.h"
#include <gnuradio/gr_complex.h>
#include <gnuradio/types.h>
#include <cstdint>

namespace gr {
namespace dvbs2rx {

class plframer_cc_impl : public plframer_cc
{
private:
    /* Parameters */
    int d_debug_level;  /** debug level */
    const double d_sps; /** samples per symbol */

    /* State */
    bool d_locked{ false };          /**< Whether the frame timing is locked */
    payload_state_t d_payload_state; /**< Payload processing state machine */
    plframe_idx_t d_idx;             /**< PLFRAME index state */

    /* Frame counts */
    uint64_t d_sof_cnt{ 0 };      /**< Total detected SOFs (including false-positives) */
    uint64_t d_frame_cnt{ 0 };    /**< Accepted/processed PLFRAMEs */
    uint64_t d_rejected_cnt{ 0 }; /**< Rejected PLFRAMEs */
    uint64_t d_dummy_cnt{ 0 };    /**< Dummy PLFRAMEs */

    /* Frame metadata from the current PLFRAME (whose payload may be under processing if
     * locked) and from the next PLFRAME (the PLHEADER ahead, processed in advance). */
    plframe_info_t d_plframe_info; /**< PLFRAME under processing */

    // Constant PLS info used in CCM/SIS mode
    pls_info_t* d_ccm_sis_pls{ nullptr };

    /* Physical layer instances */
    plsc_decoder* d_plsc_decoder{ nullptr }; /**< PLSC decoder */
    plsc_encoder* d_plsc_encoder{ nullptr }; /**< PLSC encoder */

    pl_descrambler* d_pl_descrambler{ nullptr }; /**< PL descrambler */
    pl_correlator* d_pl_correlator{ nullptr };

    const pmt::pmt_t d_port_id = pmt::mp("pls_corr");

private:
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
                       plframe_info_t& frame_info);

    void send_corr_factor(float&& corr);

public:
    plframer_cc_impl(int gold_code, double sps, int debug_level, uint8_t pls_code);
    ~plframer_cc_impl();
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);
    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

    bool get_locked() const { return d_locked; }

    uint64_t get_sof_count() const { return d_sof_cnt; }
    uint64_t get_frame_count() const { return d_frame_cnt; }
    uint64_t get_rejected_count() const { return d_rejected_cnt; }
    uint64_t get_dummy_count() const { return d_dummy_cnt; }

    std::chrono::system_clock::time_point get_lock_time() const
    {
        return d_pl_correlator->get_lock_time();
    };
};

}; // namespace dvbs2rx
}; // namespace gr

#endif // PLFRAMER_CC_IMPL_INCLUDED