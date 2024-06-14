#ifndef PL_FRAMER_INCLUDED
#define PL_FRAMER_INCLUDED


#include <gnuradio/block.h>
#include <gnuradio/dvbs2rx/api.h>

namespace gr {
namespace dvbs2rx {

/*!
 * \brief DVB-S2 Physical Layer (PL) Synchronizer
 * \ingroup dvbs2rx
 *
 * \details
 *
 * This block finds DVB-S2 PLFRAMEs on the input symbol-spaced IQ stream and outputs the
 * corresponding XFECFRAMEs towards a downstream constellation de-mapper block.
 * Internally, it implements PL frame timing recovery, coarse and fine frequency offset
 * estimation, carrier phase tracking, PLSC decoding, and PL descrambling. Furthermore, it
 * manages frequency corrections carried out by an external rotator block connected via a
 * message port.
 *
 * This block can also filter PLFRAMEs based on target PL signaling (PLS) values. In
 * constant coding and modulation (CCM) mode, the PLS filter must specify a single option
 * (i.e., a single MODCOD, frame size, and pilot configuration). In contrast, in adaptive
 * or variable coding and modulation (ACM/VCM) mode, the filter can be configured to allow
 * multiple PLS values, including all of them. In this case, since the output XFECFRAMEs
 * can vary in length and format, this block tags the first sample of each output
 * XFECFRAME with the frame's PLS information.
 */
class DVBS2RX_API plframer_cc : virtual public gr::block
{
public:
    typedef std::shared_ptr<plframer_cc> sptr;

    /*!
     * \brief Make physical layer deframer block.
     *
     * \param gold_code (int) Gold code used for physical layer scrambling.
     * \param sps (double) Oversampling ratio at the input to the upstream MF.
     * \param debug_level (int) Debug level.
     * \param pls_code (uint64_t) PLSC bit sequence.
     *
     * \note The oversampling ratio (sps) parameter is only used to schedule phase
     * increment updates (i.e., frequency corrections) to an external rotator. This block
     * attempts to schedule frequency corrections at the start of PLFRAMEs. Nevertheless,
     * while this block processes a symbol-spaced IQ stream, it assumes the external
     * rotator lies before the matched filter (MF) and, as such, processes a
     * fractionally-spaced IQ stream. Hence, when scheduling a frequency correction, this
     * block uses the sps paramter to adjust the symbol-spaced sample offset of a PLFRAME
     * to the corresponding fractionally-spaced offset in the rotator's input.
     */
    static sptr make(int gold_code,
                     int freq_est_period,
                     double sps,
                     int debug_level,
                     uint8_t pls_code);

    /*!
     * \brief Get the current frequency offset estimate.
     * \return (float) Frequency offset.
     */
    virtual float get_freq_offset() const = 0;

    /*!
     * \brief Get the coarse frequency offset correction state.
     * \return (bool) True when the frequency offset is coarsely corrected.
     */
    virtual bool get_coarse_freq_corr_state() const = 0;

    /*!
     * \brief Get the current lock status.
     * \return (bool) True when the frame synchronizer is locked.
     */
    virtual bool get_locked() const = 0;

    /*!
     * \brief Get the current count of detected start-of-frame (SOF) instants.
     *
     * This count includes all detected SOFs, including false positives. Note that
     * detecting a SOF does not mean that instant will lead to a processed frame. Frames
     * are only processed after frame timing lock, which requires two consecutive SOFs
     * detected with the correct interval between them. Hence, the SOF count is always
     * greater than or equal to the processed frame count.
     *
     * \return (uint64_t) Detected SOF count.
     */
    virtual uint64_t get_sof_count() const = 0;

    /*!
     * \brief Get the current count of processed (accepted) PLFRAMEs.
     *
     * A PLFRAME is processed after frame timing lock and after being accepted by the PLS
     * filter, in which case its XFECFRAME is output to the next block. Frames rejected by
     * the PLS filter and dummy frames are not included in this count.
     *
     * \return (uint64_t) Processed frame count.
     */
    virtual uint64_t get_frame_count() const = 0;

    /*!
     * \brief Get the current count of rejected PLFRAMEs.
     * \return (uint64_t) Rejected frame count.
     */
    virtual uint64_t get_rejected_count() const = 0;

    /*!
     * \brief Get the current count of received dummy PLFRAMEs.
     * \return (uint64_t) Dummy frame count.
     */
    virtual uint64_t get_dummy_count() const = 0;

    /*!
     * \brief Get the timestamp of the last frame synchronization lock.
     * \return (std::chrono::system_clock::time_point) Last frame lock timestamp in UTC.
     * \note The timestamp is only valid after the first frame lock.
     */
    virtual std::chrono::system_clock::time_point get_lock_time() const = 0;
};

}; // namespace dvbs2rx
}; // namespace gr

#endif // PL_FRAMER_INCLUDED