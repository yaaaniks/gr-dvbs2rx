/**
 * @file common.h
 * @author Semikozov Ian (y.semikozov@geoscan.ru)
 * @brief
 * @version 0.1
 * @date 18.07.2024
 *
 */
#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED


#include <unordered_map>
#include <string>

#include <gnuradio/dvbs2rx/dvb_config.h>

namespace gr {
namespace dvbs2rx {

inline std::unordered_map<std::string, dvb_standard_t> string_to_dvb_standard = {
    { "DVB-S2", STANDARD_DVBS2 }, { "DVB-T2", STANDARD_DVBT2 }
};

inline std::unordered_map<std::string, dvb_code_rate_t> string_to_dvb_code_rate = {
    { "1/4", C1_4 },
    { "1/3", C1_3 },
    { "2/5", C2_5 },
    { "1/2", C1_2 },
    { "3/5", C3_5 },
    { "2/3", C2_3 },
    { "3/4", C3_4 },
    { "4/5", C4_5 },
    { "5/6", C5_6 },
    { "7/8", C7_8 },
    { "8/9", C8_9 },
    { "9/10", C9_10 },
    { "13/45", C13_45 },
    { "9/20", C9_20 },
    { "90/180", C90_180 },
    { "96/180", C96_180 },
    { "11/20", C11_20 },
    { "100/180", C100_180 },
    { "104/180", C104_180 },
    { "26/45", C26_45 },
    { "18/30", C18_30 },
    { "28/45", C28_45 },
    { "23/36", C23_36 },
    { "116/180", C116_180 },
    { "20/30", C20_30 },
    { "124/180", C124_180 },
    { "25/36", C25_36 },
    { "128/180", C128_180 },
    { "13/18", C13_18 },
    { "132/180", C132_180 },
    { "22/30", C22_30 },
    { "135/180", C135_180 },
    { "140/180", C140_180 },
    { "7/9", C7_9 },
    { "154/180", C154_180 },
    { "11/45", C11_45 },
    { "4/15", C4_15 },
    { "14/45", C14_45 },
    { "7/15", C7_15 },
    { "8/15", C8_15 },
    { "32/45", C32_45 },
    { "2/9/VLSNR", C2_9_VLSNR },
    { "1/5/MEDIUM", C1_5_MEDIUM },
    { "11/45/MEDIUM", C11_45_MEDIUM },
    { "1/3/MEDIUM", C1_3_MEDIUM },
    { "1/5/VLSNR/SF2", C1_5_VLSNR_SF2 },
    { "11/45/VLSNR/SF2", C11_45_VLSNR_SF2 },
    { "1/5/VLSNR", C1_5_VLSNR },
    { "4/15/VLSNR", C4_15_VLSNR },
    { "1/3/VLSNR", C1_3_VLSNR },
    { "OTHER", C_OTHER }
};

inline std::unordered_map<std::string, dvb_framesize_t> string_to_dvb_framesize = {
    { "short", FECFRAME_SHORT },
    { "normal", FECFRAME_NORMAL },
    { "medium", FECFRAME_MEDIUM }
};

inline std::unordered_map<std::string, dvb_constellation_t>
    string_to_dvb_constellation = { { "QPSK", MOD_QPSK },
                                    { "16QAM", MOD_16QAM },
                                    { "64QAM", MOD_64QAM },
                                    { "256QAM", MOD_256QAM },
                                    { "8PSK", MOD_8PSK },
                                    { "8APSK", MOD_8APSK },
                                    { "16APSK", MOD_16APSK },
                                    { "8_8APSK", MOD_8_8APSK },
                                    { "32APSK", MOD_32APSK },
                                    { "4_12_16APSK", MOD_4_12_16APSK },
                                    { "4_8_4_16APSK", MOD_4_8_4_16APSK },
                                    { "64APSK", MOD_64APSK },
                                    { "8_16_20_20APSK", MOD_8_16_20_20APSK },
                                    { "4_12_20_28APSK", MOD_4_12_20_28APSK },
                                    { "128APSK", MOD_128APSK },
                                    { "256APSK", MOD_256APSK },
                                    { "BPSK", MOD_BPSK },
                                    { "BPSK_SF2", MOD_BPSK_SF2 },
                                    { "8VSB", MOD_8VSB },
                                    { "OTHER", MOD_OTHER } };

inline std::unordered_map<std::string, dvb_guardinterval_t>
    string_to_dvb_guardinterval = { { "1_32", GI_1_32 },    { "1_16", GI_1_16 },
                                    { "1_8", GI_1_8 },      { "1_4", GI_1_4 },
                                    { "1_128", GI_1_128 },  { "19_128", GI_19_128 },
                                    { "19_256", GI_19_256 } };

}; // namespace dvbs2rx
}; // namespace gr

#endif // COMMON_H_INCLUDED
