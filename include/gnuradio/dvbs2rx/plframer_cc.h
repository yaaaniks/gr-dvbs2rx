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
     * \param acm_vcm (bool) Whether running in ACM/VCM mode. Determines whether the PLS
     * filter can include multiple options.
     * \param pls_filter_lo (uint64_t) Lower 64 bits of the PLS filter bitmask. A value of
     * 1 in the n-th position indicates PLS "n" (for n in 0 to 63) should be enabled.
     * \param pls_filter_hi (uint64_t) Upper 64 bits of the PLS filter bitmask. A value of
     * 1 in the n-th position indicates PLS "n" (for n in 64 to 127) should be enabled.
     * \param pls_code (uint64_t) PLS Code.
     * \note When `acm_vcm=false`, the constructor throws an exception if `pls_filter_lo`
     * and `pls_filter_hi` collectively select more than one PLS value (i.e., if their
     * aggregate population count is greater than one).
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
                     double sps,
                     int debug_level,
                     bool acm_vcm,
                     uint64_t pls_filter_lo,
                     uint64_t pls_filter_hi,
                     uint64_t pls_code);
};

}; // namespace dvbs2rx
}; // namespace gr

#endif // PL_FRAMER_INCLUDED