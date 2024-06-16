#include "phase_detector.h"

namespace gr {
namespace dvbs2rx {

phase_detector::phase_detector(pls_info_t& ccm_pls, int debug_level)
    : pl_submodule("phase_detector", debug_level), d_ccm_pls(ccm_pls)
{
}

void phase_detector::estimate_plheader_phase(const uint64_t abs_sof_idx,
                                             const gr_complex* p_plheader,
                                             plframe_info_t& frame_info)
{
    frame_info.abs_sof_idx = abs_sof_idx;

    float estimated_phase;
    frame_info.plheader_phase = estimated_phase;
}

void phase_detector::estimate_coarse(gr_complex* in, plframe_info_t& frame_info)
{
    frame_info.coarse_corrected = true;
}

}; // namespace dvbs2rx
}; // namespace gr