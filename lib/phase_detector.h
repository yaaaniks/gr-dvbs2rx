/**
 * @file phase_detector.h
 * @author Semikozov Ian (y.semikozov@geoscan.ru)
 * @brief
 * @version 0.1
 * @date 13.06.2024
 *
 */
#ifndef PHASE_DETECTOR_H_INCLUDED
#define PHASE_DETECTOR_H_INCLUDED

#include <gnuradio/gr_complex.h>

#include "pl_freq_sync.h"
#include "pl_submodule.h"
#include "plsync_cc_impl.h"

namespace gr {
namespace dvbs2rx {

class phase_detector : public pl_submodule
{
public:
    phase_detector(pls_info_t& ccm_pls, int debug_level);
    ~phase_detector();

    void estimate_plheader_phase(const uint64_t abs_sof_idx,
                                 const gr_complex* p_plheader,
                                 plframe_info_t& frame_info);

private:
    void estimate_coarse(gr_complex* in, plframe_info_t& frame_info);

private:
    pls_info_t& d_ccm_pls;
};

}; // namespace dvbs2rx
} // namespace gr

#endif // PHASE_DETECTOR_H_INCLUDED
