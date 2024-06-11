#ifndef PL_CORRELATOR_INCLUDED
#define PL_CORRELATOR_INCLUDED

#include "../pl_frame_sync.h"
#include "../pl_submodule.h"
#include "gnuradio/dvbs2rx/api.h"

#include <gnuradio/gr_complex.h>
#include <gnuradio/types.h>
#include <vector>

namespace gr {
namespace dvbs2rx {


class DVBS2RX_API pl_correlator : public pl_submodule
{
    using gr_complex_v = std::vector<gr_complex>;

public:
    pl_correlator(int debug_level, float threshold = 30, int frame_threshold = 3);
    ~pl_correlator() = default;

    bool step(const gr_complex& in);

    gr_complex get_timing_metric();
    inline bool is_locked() const { return d_state == frame_sync_state_t::locked; }

    void set_frame_len(uint32_t len);

    void set_expected_plsc(const gr_complex* enc_expected_plsc);

    /**
     * \brief Get the PLFRAME payload (data + pilots) buffered internally.
     *
     * The payload observed between consecutive SOFs is buffered internally. If
     * a SOF is missed such that the last two observed SOFs are spaced by more
     * than the maximum payload length, only up to MAX_PLFRAME_PAYLOAD symbols
     * are buffered internally.
     *
     * \return (const gr_complex*) Pointer to the internal payload buffer.
     */
    const gr_complex* get_payload() const { return d_payload_buf.data(); }


    /**
     * \brief Check whether frame lock has been achieved or a SOF has been found.
     * \return (bool) True if locked or if at least a SOF has been found.
     */
    bool is_locked_or_almost() const { return d_state != frame_sync_state_t::searching; }

    /**
     * @brief Get the symbol count on the internal payload buffer
     *
     * @return uint32_t Current number of payload symbols buffered internally if locked.
     */
    uint32_t get_sym_count() const
    {
        return std::min(d_sym_cnt, (uint32_t)MAX_PLFRAME_PAYLOAD);
    }

    /**
     * \brief Get the interval between the last two detected SOFs.
     * \return (uint32_t) Interval in symbol periods.
     */
    uint32_t get_sof_interval() const { return d_sof_interval; }

    /**
     * \brief Get the PLHEADER buffered internally.
     * \return (const gr_complex*) Pointer to the internal PLHEADER buffer.
     */
    const gr_complex* get_plheader() const { return &d_plheader_buf.back(); }

    /**
     * \brief Get the SOF correlator taps.
     * \return (const gr_complex*) Pointer to the SOF correlator taps.
     */
    const gr_complex* get_sof_corr_taps() const { return d_sof_taps.data(); }

    /**
     * \brief Get the PLSC correlator taps.
     * \return (const gr_complex*) Pointer to the PLSC correlator taps.
     */
    const gr_complex* get_plsc_corr_taps() const { return d_plsc_taps.data(); }

    /**
     * \brief Get the last evaluated timing metric.
     *
     * Once locked, the timing metric updates only once per frame. Before that,
     * it updates after every input symbol.
     *
     * \return (float) Last evaluated timing metric.
     */
    float get_timing_metric() const { return d_timing_metric; }

    /**
     * @brief Get the frame lock timestamp
     *
     * @return std::chrono::system_clock::time_point Timestamp in UTC time corresponding
     * to when the frame synchronizer locked the frame timing. Valid only when locked.
     */
    std::chrono::system_clock::time_point get_lock_time() { return d_lock_time; }

private:
    void correlate(delay_line<gr_complex>& d_line,
                   volk::vector<gr_complex>& taps,
                   gr_complex& res);
    void calculate_pearson(const std::vector<gr_complex>& input,
                           const std::vector<gr_complex>& taps,
                           gr_complex& res);

private:
    uint8_t d_unlock_thresh; /**< Number of frame detection failures before unlocking */

    /* State */
    uint32_t d_sym_cnt;         /**< Symbol count since the last SOF */
    gr_complex d_last_in;       /**< Last input complex symbol */
    float d_timing_metric;      /**< Most recent timing metric */
    uint32_t d_sof_interval;    /**< Interval between the last two SOFs */
    frame_sync_state_t d_state; /**< Frame timing recovery state */
    uint32_t d_frame_len;       /**< Current PLFRAME length */
    uint8_t d_unlock_cnt;       /**< Count of consecutive frame detection failures */
    std::chrono::system_clock::time_point d_lock_time; /**< Frame lock timestamp */

    delay_line<gr_complex> d_plsc_delay_buf; /**< Buffer used as delay line */
    delay_line<gr_complex> d_sof_buf;        /**< SOF correlator buffer */
    delay_line<gr_complex> d_plsc_e_buf;     /**< Even PLSC correlator buffer  */
    delay_line<gr_complex> d_plsc_o_buf;     /**< Odd PLSC correlator buffer */
    cdeque<gr_complex> d_plheader_buf;       /**< Buffer to store the PLHEADER symbols */
    volk::vector<gr_complex> d_payload_buf;  /**< Buffer to store the PLFRAME payload */
    volk::vector<gr_complex> d_sof_taps;     /**< SOF cross-correlation taps */
    volk::vector<gr_complex> d_plsc_taps;    /**< PLSC cross-correlation taps */

    /* Timing metric threshold for inferring a start of frame.
     *
     * When unlocked, use a conservative threshold, as it is important
     * to avoid false positive SOF detection. In contrast, when locked,
     * we only want to periodically check whether the correlation is
     * sufficiently strong where it is expected to be (at the start of
     * the next frame). Since it is very important not to unlock
     * unnecessarily, use a lower threshold for this task. */
    const float threshold_u = 30; /** unlocked threshold */
                                  /* TODO: make this a top-level parameter */
    const float threshold_l = 25; /** locked threshold */
};


}; // namespace dvbs2rx
}; // namespace gr

#endif // PL_CORRELATOR_INCLUDED