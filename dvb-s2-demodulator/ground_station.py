#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Ground Station Release
# Author: Semikozov Ian
# GNU Radio version: 3.10.9.2

from gnuradio import analog
from gnuradio import blocks
from gnuradio import channels
from gnuradio.filter import firdes
from gnuradio import digital
from gnuradio import filter
from gnuradio import dvbs2rx
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import sdrplay3




class ground_station(gr.top_block):

    def __init__(self, agc_gain=1, agc_rate=0.2, agc_ref=1, costas_loop_bw=0.0018, frame_size='normal', gold_code=1, modcod='QPSK1/2', pl_freq_est_period=10, polyphase_filterbanks=128, rolloff=0.2, rrc_delay=25, rrc_nfilts=128, sps=5, sym_rate=2000000, sym_sync_damping=1.6, sym_sync_loop_bw=0.003):
        gr.top_block.__init__(self, "Ground Station Release", catch_exceptions=True)

        ##################################################
        # Parameters
        ##################################################
        self.agc_gain = agc_gain
        self.agc_rate = agc_rate
        self.agc_ref = agc_ref
        self.costas_loop_bw = costas_loop_bw
        self.frame_size = frame_size
        self.gold_code = gold_code
        self.modcod = modcod
        self.pl_freq_est_period = pl_freq_est_period
        self.polyphase_filterbanks = polyphase_filterbanks
        self.rolloff = rolloff
        self.rrc_delay = rrc_delay
        self.rrc_nfilts = rrc_nfilts
        self.sps = sps
        self.sym_rate = sym_rate
        self.sym_sync_damping = sym_sync_damping
        self.sym_sync_loop_bw = sym_sync_loop_bw

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = sps*sym_rate
        self.n_taps = n_taps = 20
        self.code_rate = code_rate = modcod.upper().replace("8PSK", "").replace("QPSK", "")
        self.freq = freq = 1000000
        self.fir_taps = fir_taps = firdes.root_raised_cosine(1.0, float(samp_rate),float(sym_rate), 0.35, (int(sps*n_taps)))
        self.constellation = constellation = modcod.replace(code_rate, "")
        self.cl_order = cl_order = 4

        ##################################################
        # Blocks
        ##################################################

        self.sdrplay3_rsp1_0 = sdrplay3.rsp1(
            '',
            stream_args=sdrplay3.stream_args(
                output_type='fc32',
                channels_size=1
            ),
        )
        self.sdrplay3_rsp1_0.set_sample_rate(samp_rate)
        self.sdrplay3_rsp1_0.set_center_freq(0)
        self.sdrplay3_rsp1_0.set_bandwidth(0)
        self.sdrplay3_rsp1_0.set_gain_mode(False)
        self.sdrplay3_rsp1_0.set_gain(-(40), 'IF')
        self.sdrplay3_rsp1_0.set_gain(-(0), 'RF')
        self.sdrplay3_rsp1_0.set_freq_corr(0)
        self.sdrplay3_rsp1_0.set_dc_offset_mode(False)
        self.sdrplay3_rsp1_0.set_iq_balance_mode(False)
        self.sdrplay3_rsp1_0.set_agc_setpoint((-30))
        self.sdrplay3_rsp1_0.set_debug_mode(False)
        self.sdrplay3_rsp1_0.set_sample_sequence_gaps_check(False)
        self.sdrplay3_rsp1_0.set_show_gain_changes(False)
        self.fir_filter_xxx_0 = filter.fir_filter_ccc(1, fir_taps)
        self.fir_filter_xxx_0.declare_sample_delay(0)
        self.dvbs2rx_xfecframe_demapper_cb_0 = dvbs2rx.xfecframe_demapper_cb(
          *dvbs2rx.params.translate("DVB-S2", frame_size, code_rate,
          constellation)[1:])
        self.dvbs2rx_plframer_cc_0 = dvbs2rx.plframer_cc(0, pl_freq_est_period, sps, 0, 5)
        self.dvbs2rx_ldpc_decoder_bb_0 = dvbs2rx.ldpc_decoder_bb(
            *dvbs2rx.params.translate('DVB-S2',
                frame_size,
                code_rate,
                constellation
            ),
            dvbs2rx.OM_MESSAGE,
            dvbs2rx.INFO_OFF,
            25,
            0)
        self.dvbs2rx_bch_decoder_bb_0 = dvbs2rx.bch_decoder_bb(
            *dvbs2rx.params.translate('DVB-S2',
                frame_size,
                code_rate
            ),
            dvbs2rx.OM_MESSAGE,
            0)
        self.dvbs2rx_bbdescrambler_bb_0 = dvbs2rx.bbdescrambler_bb(
            *dvbs2rx.params.translate('DVB-S2',
                frame_size,
                code_rate
            )
        )
        self.dvbs2rx_bbdeheader_bb_0 = dvbs2rx.bbdeheader_bb(
            *dvbs2rx.params.translate('DVB-S2',
                frame_size,
                code_rate
            ),
            0
        )
        self.digital_symbol_sync_xx_1 = digital.symbol_sync_cc(
            digital.TED_GARDNER,
            sps,
            sym_sync_loop_bw,
            sym_sync_damping,
            1,
            1.0,
            1,
            digital.constellation_bpsk().base(),
            digital.IR_PFB_NO_MF,
            polyphase_filterbanks,
            firdes.root_raised_cosine(rrc_nfilts, samp_rate*rrc_nfilts, sym_rate, rolloff, (int(2*rrc_delay*sps) + 1)*rrc_nfilts))
        self.digital_costas_loop_cc_0 = digital.costas_loop_cc(costas_loop_bw, cl_order, False)
        self.blocks_rms_xx_0 = blocks.rms_cf(0.0001)
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_float*1)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_char*1, '/home/semikozov/example.ts', False)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.analog_agc_xx_0 = analog.agc_cc(agc_rate, 1.0, 1.0, 65536)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.dvbs2rx_ldpc_decoder_bb_0, 'llr_pdu'), (self.dvbs2rx_xfecframe_demapper_cb_0, 'llr_pdu'))
        self.connect((self.analog_agc_xx_0, 0), (self.blocks_rms_xx_0, 0))
        self.connect((self.analog_agc_xx_0, 0), (self.fir_filter_xxx_0, 0))
        self.connect((self.blocks_rms_xx_0, 0), (self.blocks_null_sink_0, 0))
        self.connect((self.digital_costas_loop_cc_0, 0), (self.dvbs2rx_plframer_cc_0, 0))
        self.connect((self.digital_symbol_sync_xx_1, 0), (self.digital_costas_loop_cc_0, 0))
        self.connect((self.dvbs2rx_bbdeheader_bb_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.dvbs2rx_bbdescrambler_bb_0, 0), (self.dvbs2rx_bbdeheader_bb_0, 0))
        self.connect((self.dvbs2rx_bch_decoder_bb_0, 0), (self.dvbs2rx_bbdescrambler_bb_0, 0))
        self.connect((self.dvbs2rx_ldpc_decoder_bb_0, 0), (self.dvbs2rx_bch_decoder_bb_0, 0))
        self.connect((self.dvbs2rx_plframer_cc_0, 0), (self.dvbs2rx_xfecframe_demapper_cb_0, 0))
        self.connect((self.dvbs2rx_xfecframe_demapper_cb_0, 0), (self.dvbs2rx_ldpc_decoder_bb_0, 0))
        self.connect((self.fir_filter_xxx_0, 0), (self.digital_symbol_sync_xx_1, 0))
        self.connect((self.sdrplay3_rsp1_0, 0), (self.analog_agc_xx_0, 0))


    def get_agc_gain(self):
        return self.agc_gain

    def set_agc_gain(self, agc_gain):
        self.agc_gain = agc_gain

    def get_agc_rate(self):
        return self.agc_rate

    def set_agc_rate(self, agc_rate):
        self.agc_rate = agc_rate
        self.analog_agc_xx_0.set_rate(self.agc_rate)

    def get_agc_ref(self):
        return self.agc_ref

    def set_agc_ref(self, agc_ref):
        self.agc_ref = agc_ref

    def get_costas_loop_bw(self):
        return self.costas_loop_bw

    def set_costas_loop_bw(self, costas_loop_bw):
        self.costas_loop_bw = costas_loop_bw
        self.digital_costas_loop_cc_0.set_loop_bandwidth(self.costas_loop_bw)

    def get_frame_size(self):
        return self.frame_size

    def set_frame_size(self, frame_size):
        self.frame_size = frame_size

    def get_gold_code(self):
        return self.gold_code

    def set_gold_code(self, gold_code):
        self.gold_code = gold_code

    def get_modcod(self):
        return self.modcod

    def set_modcod(self, modcod):
        self.modcod = modcod

    def get_pl_freq_est_period(self):
        return self.pl_freq_est_period

    def set_pl_freq_est_period(self, pl_freq_est_period):
        self.pl_freq_est_period = pl_freq_est_period

    def get_polyphase_filterbanks(self):
        return self.polyphase_filterbanks

    def set_polyphase_filterbanks(self, polyphase_filterbanks):
        self.polyphase_filterbanks = polyphase_filterbanks

    def get_rolloff(self):
        return self.rolloff

    def set_rolloff(self, rolloff):
        self.rolloff = rolloff

    def get_rrc_delay(self):
        return self.rrc_delay

    def set_rrc_delay(self, rrc_delay):
        self.rrc_delay = rrc_delay

    def get_rrc_nfilts(self):
        return self.rrc_nfilts

    def set_rrc_nfilts(self, rrc_nfilts):
        self.rrc_nfilts = rrc_nfilts

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_fir_taps(firdes.root_raised_cosine(1.0, float(self.samp_rate), float(self.sym_rate), 0.35, (int(self.sps*self.n_taps))))
        self.set_samp_rate(self.sps*self.sym_rate)
        self.digital_symbol_sync_xx_1.set_sps(self.sps)

    def get_sym_rate(self):
        return self.sym_rate

    def set_sym_rate(self, sym_rate):
        self.sym_rate = sym_rate
        self.set_fir_taps(firdes.root_raised_cosine(1.0, float(self.samp_rate), float(self.sym_rate), 0.35, (int(self.sps*self.n_taps))))
        self.set_samp_rate(self.sps*self.sym_rate)

    def get_sym_sync_damping(self):
        return self.sym_sync_damping

    def set_sym_sync_damping(self, sym_sync_damping):
        self.sym_sync_damping = sym_sync_damping
        self.digital_symbol_sync_xx_1.set_damping_factor(self.sym_sync_damping)

    def get_sym_sync_loop_bw(self):
        return self.sym_sync_loop_bw

    def set_sym_sync_loop_bw(self, sym_sync_loop_bw):
        self.sym_sync_loop_bw = sym_sync_loop_bw
        self.digital_symbol_sync_xx_1.set_loop_bandwidth(self.sym_sync_loop_bw)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_fir_taps(firdes.root_raised_cosine(1.0, float(self.samp_rate), float(self.sym_rate), 0.35, (int(self.sps*self.n_taps))))
        self.sdrplay3_rsp1_0.set_sample_rate(self.samp_rate)

    def get_n_taps(self):
        return self.n_taps

    def set_n_taps(self, n_taps):
        self.n_taps = n_taps
        self.set_fir_taps(firdes.root_raised_cosine(1.0, float(self.samp_rate), float(self.sym_rate), 0.35, (int(self.sps*self.n_taps))))

    def get_code_rate(self):
        return self.code_rate

    def set_code_rate(self, code_rate):
        self.code_rate = code_rate
        self.set_constellation(modcod.replace(self.code_rate, ""))

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq

    def get_fir_taps(self):
        return self.fir_taps

    def set_fir_taps(self, fir_taps):
        self.fir_taps = fir_taps
        self.fir_filter_xxx_0.set_taps(self.fir_taps)

    def get_constellation(self):
        return self.constellation

    def set_constellation(self, constellation):
        self.constellation = constellation

    def get_cl_order(self):
        return self.cl_order

    def set_cl_order(self, cl_order):
        self.cl_order = cl_order



def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--agc-gain", dest="agc_gain", type=eng_float, default=eng_notation.num_to_str(float(1)),
        help="Set AGC gain [default=%(default)r]")
    parser.add_argument(
        "--agc-rate", dest="agc_rate", type=eng_float, default=eng_notation.num_to_str(float(0.2)),
        help="Set AGC update rate [default=%(default)r]")
    parser.add_argument(
        "--agc-ref", dest="agc_ref", type=eng_float, default=eng_notation.num_to_str(float(1)),
        help="Set AGC's reference value [default=%(default)r]")
    parser.add_argument(
        "-c", "--costas-loop-bw", dest="costas_loop_bw", type=eng_float, default=eng_notation.num_to_str(float(0.0018)),
        help="Set Costas Loop Bandwidth [default=%(default)r]")
    parser.add_argument(
        "-f", "--frame-size", dest="frame_size", type=str, default='normal',
        help="Set FECFRAME size [default=%(default)r]")
    parser.add_argument(
        "--gold-code", dest="gold_code", type=intx, default=1,
        help="Set Gold code [default=%(default)r]")
    parser.add_argument(
        "-m", "--modcod", dest="modcod", type=str, default='QPSK1/2',
        help="Set MODCOD [default=%(default)r]")
    parser.add_argument(
        "--pl-freq-est-period", dest="pl_freq_est_period", type=intx, default=10,
        help="Set PL synchronizer's frequency estimation period in frames [default=%(default)r]")
    parser.add_argument(
        "-b", "--polyphase-filterbanks", dest="polyphase_filterbanks", type=intx, default=128,
        help="Set Count of polyphase filter banks [default=%(default)r]")
    parser.add_argument(
        "-r", "--rolloff", dest="rolloff", type=eng_float, default=eng_notation.num_to_str(float(0.2)),
        help="Set rolloff factor [default=%(default)r]")
    parser.add_argument(
        "--rrc-delay", dest="rrc_delay", type=intx, default=25,
        help="Set RRC filter delay in symbol periods [default=%(default)r]")
    parser.add_argument(
        "--rrc-nfilts", dest="rrc_nfilts", type=intx, default=128,
        help="Set number of branches on the polyphase RRC filter [default=%(default)r]")
    parser.add_argument(
        "-o", "--sps", dest="sps", type=eng_float, default=eng_notation.num_to_str(float(5)),
        help="Set oversampling ratio in samples per symbol [default=%(default)r]")
    parser.add_argument(
        "-s", "--sym-rate", dest="sym_rate", type=intx, default=2000000,
        help="Set symbol rate in bauds [default=%(default)r]")
    parser.add_argument(
        "--sym-sync-damping", dest="sym_sync_damping", type=eng_float, default=eng_notation.num_to_str(float(1.6)),
        help="Set symbol synchronizer's damping factor [default=%(default)r]")
    parser.add_argument(
        "--sym-sync-loop-bw", dest="sym_sync_loop_bw", type=eng_float, default=eng_notation.num_to_str(float(0.003)),
        help="Set Symbol Synchronizer Loop BandWidth  [default=%(default)r]")
    return parser


def main(top_block_cls=ground_station, options=None):
    if options is None:
        options = argument_parser().parse_args()
    tb = top_block_cls(agc_gain=options.agc_gain, agc_rate=options.agc_rate, agc_ref=options.agc_ref, costas_loop_bw=options.costas_loop_bw, frame_size=options.frame_size, gold_code=options.gold_code, modcod=options.modcod, pl_freq_est_period=options.pl_freq_est_period, polyphase_filterbanks=options.polyphase_filterbanks, rolloff=options.rolloff, rrc_delay=options.rrc_delay, rrc_nfilts=options.rrc_nfilts, sps=options.sps, sym_rate=options.sym_rate, sym_sync_damping=options.sym_sync_damping, sym_sync_loop_bw=options.sym_sync_loop_bw)

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
