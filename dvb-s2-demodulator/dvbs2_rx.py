#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: DVB-S2 Rx
# Description: Full DVB-S2 receiver. Processes IQ samples from stdin and outputs MPEG TS packets to stdout.
# GNU Radio version: 3.10.9.2

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from gnuradio import analog
from gnuradio import blocks
from gnuradio import channels
from gnuradio.filter import firdes
from gnuradio import digital
from gnuradio import filter
from gnuradio import dvbs2rx
from gnuradio import fft
from gnuradio.fft import window
from gnuradio import gr
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import sdrplay3
import dvbs2_rx_epy_block_0 as epy_block_0  # embedded python block
import dvbs2_rx_epy_block_1 as epy_block_1  # embedded python block
import math
import sip



class dvbs2_rx(gr.top_block, Qt.QWidget):

    def __init__(self, agc_gain=1, agc_rate=0.2, agc_ref=1, debug=0, frame_size='normal', gold_code=1, modcod='QPSK1/2', pl_freq_est_period=10, rolloff=0.2, rrc_delay=25, rrc_nfilts=128, snr=1.2, sps=5, sym_rate=2000000, sym_sync_damping=1.6):
        gr.top_block.__init__(self, "DVB-S2 Rx", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("DVB-S2 Rx")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "dvbs2_rx")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)

        ##################################################
        # Parameters
        ##################################################
        self.agc_gain = agc_gain
        self.agc_rate = agc_rate
        self.agc_ref = agc_ref
        self.debug = debug
        self.frame_size = frame_size
        self.gold_code = gold_code
        self.modcod = modcod
        self.pl_freq_est_period = pl_freq_est_period
        self.rolloff = rolloff
        self.rrc_delay = rrc_delay
        self.rrc_nfilts = rrc_nfilts
        self.snr = snr
        self.sps = sps
        self.sym_rate = sym_rate
        self.sym_sync_damping = sym_sync_damping

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = sym_rate * sps
        self.qpsk = qpsk = digital.constellation_rect([0.707+0.707j, -0.707+0.707j, -0.707-0.707j, 0.707-0.707j], [0, 1, 2, 3],
        4, 2, 2, 1, 1).base()
        self.code_rate = code_rate = modcod.upper().replace("8PSK", "").replace("QPSK", "")
        self.variable_adaptive_algorithm = variable_adaptive_algorithm = digital.adaptive_algorithm_cma( qpsk, .0001, 4).base()
        self.sym_sync_loop_bw = sym_sync_loop_bw = 0.003
        self.qtgui_range_lna_gain = qtgui_range_lna_gain = 0
        self.qtgui_range_if_gain = qtgui_range_if_gain = 40
        self.qtgui_range_agc_setpoint = qtgui_range_agc_setpoint = -90
        self.plheader_len = plheader_len = 90
        self.plframe_len = plframe_len = 33282
        self.pilot_len = pilot_len = int((360-1)/16)*36
        self.noise = noise = snr
        self.n_taps = n_taps = 25
        self.freq = freq = 130000000
        self.fir_taps = fir_taps = firdes.root_raised_cosine(1.0, float(samp_rate),float(sym_rate), 0.35, (int(sps*20)))
        self.costas_loop_bw = costas_loop_bw = 0.0018
        self.constellation = constellation = modcod.replace(code_rate, "")
        self.cl_order = cl_order = 4
        self.Es = Es = 1

        ##################################################
        # Blocks
        ##################################################

        self.tabs = Qt.QTabWidget()
        self.tabs_widget_0 = Qt.QWidget()
        self.tabs_layout_0 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_0)
        self.tabs_grid_layout_0 = Qt.QGridLayout()
        self.tabs_layout_0.addLayout(self.tabs_grid_layout_0)
        self.tabs.addTab(self.tabs_widget_0, 'Waterfall')
        self.tabs_widget_1 = Qt.QWidget()
        self.tabs_layout_1 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_1)
        self.tabs_grid_layout_1 = Qt.QGridLayout()
        self.tabs_layout_1.addLayout(self.tabs_grid_layout_1)
        self.tabs.addTab(self.tabs_widget_1, 'Frequency Correction')
        self.tabs_widget_2 = Qt.QWidget()
        self.tabs_layout_2 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_2)
        self.tabs_grid_layout_2 = Qt.QGridLayout()
        self.tabs_layout_2.addLayout(self.tabs_grid_layout_2)
        self.tabs.addTab(self.tabs_widget_2, 'Synchronization')
        self.tabs_widget_3 = Qt.QWidget()
        self.tabs_layout_3 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_3)
        self.tabs_grid_layout_3 = Qt.QGridLayout()
        self.tabs_layout_3.addLayout(self.tabs_grid_layout_3)
        self.tabs.addTab(self.tabs_widget_3, 'Frame Recovery')
        self.tabs_widget_4 = Qt.QWidget()
        self.tabs_layout_4 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_4)
        self.tabs_grid_layout_4 = Qt.QGridLayout()
        self.tabs_layout_4.addLayout(self.tabs_grid_layout_4)
        self.tabs.addTab(self.tabs_widget_4, 'Eye Diagrams')
        self.tabs_widget_5 = Qt.QWidget()
        self.tabs_layout_5 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_5)
        self.tabs_grid_layout_5 = Qt.QGridLayout()
        self.tabs_layout_5.addLayout(self.tabs_grid_layout_5)
        self.tabs.addTab(self.tabs_widget_5, 'Error Rate')
        self.tabs_widget_6 = Qt.QWidget()
        self.tabs_layout_6 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_6)
        self.tabs_grid_layout_6 = Qt.QGridLayout()
        self.tabs_layout_6.addLayout(self.tabs_grid_layout_6)
        self.tabs.addTab(self.tabs_widget_6, 'Input SIgnal')
        self.tabs_widget_7 = Qt.QWidget()
        self.tabs_layout_7 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tabs_widget_7)
        self.tabs_grid_layout_7 = Qt.QGridLayout()
        self.tabs_layout_7.addLayout(self.tabs_grid_layout_7)
        self.tabs.addTab(self.tabs_widget_7, 'Test')
        self.top_grid_layout.addWidget(self.tabs, 0, 0, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._sym_sync_loop_bw_range = qtgui.Range(0.0001, 0.5, 0.0001, 0.003, 200)
        self._sym_sync_loop_bw_win = qtgui.RangeWidget(self._sym_sync_loop_bw_range, self.set_sym_sync_loop_bw, "SymSync Loop BW", "counter_slider", float, QtCore.Qt.Horizontal)
        self.tabs_grid_layout_2.addWidget(self._sym_sync_loop_bw_win, 1, 0, 1, 1)
        for r in range(1, 2):
            self.tabs_grid_layout_2.setRowStretch(r, 1)
        for c in range(0, 1):
            self.tabs_grid_layout_2.setColumnStretch(c, 1)
        self._qtgui_range_lna_gain_range = qtgui.Range(0, 3, 1, 0, 200)
        self._qtgui_range_lna_gain_win = qtgui.RangeWidget(self._qtgui_range_lna_gain_range, self.set_qtgui_range_lna_gain, "LNA gain", "counter_slider", int, QtCore.Qt.Horizontal)
        self.tabs_layout_6.addWidget(self._qtgui_range_lna_gain_win)
        self._qtgui_range_if_gain_range = qtgui.Range(20, 59, 1, 40, 200)
        self._qtgui_range_if_gain_win = qtgui.RangeWidget(self._qtgui_range_if_gain_range, self.set_qtgui_range_if_gain, "IF Gain", "counter_slider", float, QtCore.Qt.Horizontal)
        self.tabs_layout_6.addWidget(self._qtgui_range_if_gain_win)
        self._noise_range = qtgui.Range(0, 5, 0.0001, snr, 200)
        self._noise_win = qtgui.RangeWidget(self._noise_range, self.set_noise, "Noise", "counter_slider", float, QtCore.Qt.Horizontal)
        self.tabs_layout_2.addWidget(self._noise_win)
        self._costas_loop_bw_range = qtgui.Range(0.0001, 0.5, 0.0001, 0.0018, 200)
        self._costas_loop_bw_win = qtgui.RangeWidget(self._costas_loop_bw_range, self.set_costas_loop_bw, "Costas Loop BW ", "counter_slider", float, QtCore.Qt.Horizontal)
        self.tabs_grid_layout_2.addWidget(self._costas_loop_bw_win, 1, 1, 1, 1)
        for r in range(1, 2):
            self.tabs_grid_layout_2.setRowStretch(r, 1)
        for c in range(1, 2):
            self.tabs_grid_layout_2.setColumnStretch(c, 1)
        self.sdrplay3_rsp1_0 = sdrplay3.rsp1(
            '',
            stream_args=sdrplay3.stream_args(
                output_type='fc32',
                channels_size=1
            ),
        )
        self.sdrplay3_rsp1_0.set_sample_rate(samp_rate)
        self.sdrplay3_rsp1_0.set_center_freq(freq)
        self.sdrplay3_rsp1_0.set_bandwidth(8000e3)
        self.sdrplay3_rsp1_0.set_gain_mode(False)
        self.sdrplay3_rsp1_0.set_gain(-(qtgui_range_if_gain), 'IF')
        self.sdrplay3_rsp1_0.set_gain(qtgui_range_lna_gain, 'LNAstate')
        self.sdrplay3_rsp1_0.set_freq_corr(0)
        self.sdrplay3_rsp1_0.set_dc_offset_mode(True)
        self.sdrplay3_rsp1_0.set_iq_balance_mode(True)
        self.sdrplay3_rsp1_0.set_agc_setpoint((-30))
        self.sdrplay3_rsp1_0.set_debug_mode(True)
        self.sdrplay3_rsp1_0.set_sample_sequence_gaps_check(True)
        self.sdrplay3_rsp1_0.set_show_gain_changes(True)
        self.qtgui_waterfall_sink_x_0 = qtgui.waterfall_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "Spectrogram", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_waterfall_sink_x_0.set_update_time(0.10)
        self.qtgui_waterfall_sink_x_0.enable_grid(False)
        self.qtgui_waterfall_sink_x_0.enable_axis_labels(True)



        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        colors = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_waterfall_sink_x_0.set_color_map(i, colors[i])
            self.qtgui_waterfall_sink_x_0.set_line_alpha(i, alphas[i])

        self.qtgui_waterfall_sink_x_0.set_intensity_range(-140, 10)

        self._qtgui_waterfall_sink_x_0_win = sip.wrapinstance(self.qtgui_waterfall_sink_x_0.qwidget(), Qt.QWidget)

        self.tabs_grid_layout_0.addWidget(self._qtgui_waterfall_sink_x_0_win, 0, 0, 1, 3)
        for r in range(0, 1):
            self.tabs_grid_layout_0.setRowStretch(r, 1)
        for c in range(0, 3):
            self.tabs_grid_layout_0.setColumnStretch(c, 1)
        self.qtgui_vector_sink_f_0 = qtgui.vector_sink_f(
            4096,
            0,
            1.0,
            "x-Axis",
            "y-Axis",
            "",
            1, # Number of inputs
            None # parent
        )
        self.qtgui_vector_sink_f_0.set_update_time(0.10)
        self.qtgui_vector_sink_f_0.set_y_axis((-140), 10)
        self.qtgui_vector_sink_f_0.enable_autoscale(False)
        self.qtgui_vector_sink_f_0.enable_grid(False)
        self.qtgui_vector_sink_f_0.set_x_axis_units("")
        self.qtgui_vector_sink_f_0.set_y_axis_units("")
        self.qtgui_vector_sink_f_0.set_ref_level(0)


        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_vector_sink_f_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_vector_sink_f_0.set_line_label(i, labels[i])
            self.qtgui_vector_sink_f_0.set_line_width(i, widths[i])
            self.qtgui_vector_sink_f_0.set_line_color(i, colors[i])
            self.qtgui_vector_sink_f_0.set_line_alpha(i, alphas[i])

        self._qtgui_vector_sink_f_0_win = sip.wrapinstance(self.qtgui_vector_sink_f_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_vector_sink_f_0_win)
        self.qtgui_time_sink_x_1 = qtgui.time_sink_c(
            (plframe_len - pilot_len - plheader_len), #size
            samp_rate, #samp_rate
            "", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_1.set_update_time(0.10)
        self.qtgui_time_sink_x_1.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_1.set_y_label('PLFRAME Symbols', "")

        self.qtgui_time_sink_x_1.enable_tags(True)
        self.qtgui_time_sink_x_1.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_1.enable_autoscale(True)
        self.qtgui_time_sink_x_1.enable_grid(True)
        self.qtgui_time_sink_x_1.enable_axis_labels(True)
        self.qtgui_time_sink_x_1.enable_control_panel(False)
        self.qtgui_time_sink_x_1.enable_stem_plot(False)


        labels = ['I', 'Q', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                if (i % 2 == 0):
                    self.qtgui_time_sink_x_1.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_1.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_1.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_1.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_1.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_1.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_1.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_1.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_1_win = sip.wrapinstance(self.qtgui_time_sink_x_1.qwidget(), Qt.QWidget)
        self.tabs_layout_3.addWidget(self._qtgui_time_sink_x_1_win)
        self._qtgui_range_agc_setpoint_range = qtgui.Range(-100, 100, 1, -90, 200)
        self._qtgui_range_agc_setpoint_win = qtgui.RangeWidget(self._qtgui_range_agc_setpoint_range, self.set_qtgui_range_agc_setpoint, "AGC setpoint", "counter_slider", float, QtCore.Qt.Horizontal)
        self.tabs_layout_6.addWidget(self._qtgui_range_agc_setpoint_win)
        self.qtgui_number_sink_2 = qtgui.number_sink(
            gr.sizeof_float,
            0,
            qtgui.NUM_GRAPH_HORIZ,
            1,
            None # parent
        )
        self.qtgui_number_sink_2.set_update_time(0.10)
        self.qtgui_number_sink_2.set_title("SNR")

        labels = ['', '', '', '', '',
            '', '', '', '', '']
        units = ['', '', '', '', '',
            '', '', '', '', '']
        colors = [("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"),
            ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black")]
        factor = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]

        for i in range(1):
            self.qtgui_number_sink_2.set_min(i, -100)
            self.qtgui_number_sink_2.set_max(i, 100)
            self.qtgui_number_sink_2.set_color(i, colors[i][0], colors[i][1])
            if len(labels[i]) == 0:
                self.qtgui_number_sink_2.set_label(i, "Data {0}".format(i))
            else:
                self.qtgui_number_sink_2.set_label(i, labels[i])
            self.qtgui_number_sink_2.set_unit(i, units[i])
            self.qtgui_number_sink_2.set_factor(i, factor[i])

        self.qtgui_number_sink_2.enable_autoscale(False)
        self._qtgui_number_sink_2_win = sip.wrapinstance(self.qtgui_number_sink_2.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_number_sink_2_win)
        self.qtgui_number_sink_1 = qtgui.number_sink(
            gr.sizeof_float,
            0,
            qtgui.NUM_GRAPH_HORIZ,
            2,
            None # parent
        )
        self.qtgui_number_sink_1.set_update_time(0.001)
        self.qtgui_number_sink_1.set_title("Packet error rate (%)")

        labels = ['FER', 'BER', '', '', '',
            '', '', '', '', '']
        units = ['', '', '', '', '',
            '', '', '', '', '']
        colors = [("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"),
            ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black")]
        factor = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]

        for i in range(2):
            self.qtgui_number_sink_1.set_min(i, 0)
            self.qtgui_number_sink_1.set_max(i, 100)
            self.qtgui_number_sink_1.set_color(i, colors[i][0], colors[i][1])
            if len(labels[i]) == 0:
                self.qtgui_number_sink_1.set_label(i, "Data {0}".format(i))
            else:
                self.qtgui_number_sink_1.set_label(i, labels[i])
            self.qtgui_number_sink_1.set_unit(i, units[i])
            self.qtgui_number_sink_1.set_factor(i, factor[i])

        self.qtgui_number_sink_1.enable_autoscale(False)
        self._qtgui_number_sink_1_win = sip.wrapinstance(self.qtgui_number_sink_1.qwidget(), Qt.QWidget)
        self.tabs_layout_5.addWidget(self._qtgui_number_sink_1_win)
        self.qtgui_number_sink_0 = qtgui.number_sink(
            gr.sizeof_float,
            0,
            qtgui.NUM_GRAPH_VERT,
            1,
            None # parent
        )
        self.qtgui_number_sink_0.set_update_time(0.10)
        self.qtgui_number_sink_0.set_title("RMS Level")

        labels = ['', '', '', '', '',
            '', '', '', '', '']
        units = ['', '', '', '', '',
            '', '', '', '', '']
        colors = [("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"),
            ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black")]
        factor = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]

        for i in range(1):
            self.qtgui_number_sink_0.set_min(i, -1)
            self.qtgui_number_sink_0.set_max(i, 1)
            self.qtgui_number_sink_0.set_color(i, colors[i][0], colors[i][1])
            if len(labels[i]) == 0:
                self.qtgui_number_sink_0.set_label(i, "Data {0}".format(i))
            else:
                self.qtgui_number_sink_0.set_label(i, labels[i])
            self.qtgui_number_sink_0.set_unit(i, units[i])
            self.qtgui_number_sink_0.set_factor(i, factor[i])

        self.qtgui_number_sink_0.enable_autoscale(False)
        self._qtgui_number_sink_0_win = sip.wrapinstance(self.qtgui_number_sink_0.qwidget(), Qt.QWidget)
        self.tabs_grid_layout_0.addWidget(self._qtgui_number_sink_0_win, 0, 3, 1, 1)
        for r in range(0, 1):
            self.tabs_grid_layout_0.setRowStretch(r, 1)
        for c in range(3, 4):
            self.tabs_grid_layout_0.setColumnStretch(c, 1)
        self.qtgui_freq_sink_x_1_0 = qtgui.freq_sink_c(
            4096, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_x_1_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_1_0.set_y_axis((-80), (-30))
        self.qtgui_freq_sink_x_1_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_1_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_1_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_1_0.enable_grid(True)
        self.qtgui_freq_sink_x_1_0.set_fft_average(0.05)
        self.qtgui_freq_sink_x_1_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_1_0.enable_control_panel(False)
        self.qtgui_freq_sink_x_1_0.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_1_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_1_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_1_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_1_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_1_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_1_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_1_0.qwidget(), Qt.QWidget)
        self.tabs_layout_6.addWidget(self._qtgui_freq_sink_x_1_0_win)
        self.qtgui_freq_sink_x_1 = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "", #name
            2,
            None # parent
        )
        self.qtgui_freq_sink_x_1.set_update_time(0.10)
        self.qtgui_freq_sink_x_1.set_y_axis((-140), 10)
        self.qtgui_freq_sink_x_1.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_1.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_1.enable_autoscale(True)
        self.qtgui_freq_sink_x_1.enable_grid(True)
        self.qtgui_freq_sink_x_1.set_fft_average(0.1)
        self.qtgui_freq_sink_x_1.enable_axis_labels(True)
        self.qtgui_freq_sink_x_1.enable_control_panel(False)
        self.qtgui_freq_sink_x_1.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(2):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_1.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_1.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_1.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_1.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_1.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_1_win = sip.wrapinstance(self.qtgui_freq_sink_x_1.qwidget(), Qt.QWidget)
        self.tabs_layout_2.addWidget(self._qtgui_freq_sink_x_1_win)
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "Frequency", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis((-140), 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(True)
        self.qtgui_freq_sink_x_0.set_fft_average(0.05)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(False)
        self.qtgui_freq_sink_x_0.set_fft_window_normalized(False)



        labels = ['Before', 'After', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.qwidget(), Qt.QWidget)
        self.tabs_layout_1.addWidget(self._qtgui_freq_sink_x_0_win)
        self.qtgui_eye_sink_x_0 = qtgui.eye_sink_c(
            1024, #size
            samp_rate, #samp_rate
            1, #number of inputs
            None
        )
        self.qtgui_eye_sink_x_0.set_update_time(0.1)
        self.qtgui_eye_sink_x_0.set_samp_per_symbol(10)
        self.qtgui_eye_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_eye_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_eye_sink_x_0.enable_tags(True)
        self.qtgui_eye_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_eye_sink_x_0.enable_autoscale(True)
        self.qtgui_eye_sink_x_0.enable_grid(False)
        self.qtgui_eye_sink_x_0.enable_axis_labels(True)
        self.qtgui_eye_sink_x_0.enable_control_panel(False)


        labels = ['Im', 'Re', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'blue', 'blue', 'blue', 'blue',
            'blue', 'blue', 'blue', 'blue', 'blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                if (i % 2 == 0):
                    self.qtgui_eye_sink_x_0.set_line_label(i, "Eye [Re{{Data {0}}}]".format(round(i/2)))
                else:
                    self.qtgui_eye_sink_x_0.set_line_label(i, "Eye [Im{{Data {0}}}]".format(round((i-1)/2)))
            else:
                self.qtgui_eye_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_eye_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_eye_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_eye_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_eye_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_eye_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_eye_sink_x_0_win = sip.wrapinstance(self.qtgui_eye_sink_x_0.qwidget(), Qt.QWidget)
        self.tabs_layout_4.addWidget(self._qtgui_eye_sink_x_0_win)
        self.qtgui_edit_box_msg_1 = qtgui.edit_box_msg(qtgui.STRING, '', 'Current frame errors', False, True, '', None)
        self._qtgui_edit_box_msg_1_win = sip.wrapinstance(self.qtgui_edit_box_msg_1.qwidget(), Qt.QWidget)
        self.tabs_layout_5.addWidget(self._qtgui_edit_box_msg_1_win)
        self.qtgui_edit_box_msg_0 = qtgui.edit_box_msg(qtgui.STRING, '', 'Current bit errors', False, True, '', None)
        self._qtgui_edit_box_msg_0_win = sip.wrapinstance(self.qtgui_edit_box_msg_0.qwidget(), Qt.QWidget)
        self.tabs_layout_5.addWidget(self._qtgui_edit_box_msg_0_win)
        self.qtgui_const_sink_x_1_0 = qtgui.const_sink_c(
            1024, #size
            "Symbol Sync Output", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_const_sink_x_1_0.set_update_time(0.10)
        self.qtgui_const_sink_x_1_0.set_y_axis((-2), 2)
        self.qtgui_const_sink_x_1_0.set_x_axis((-2), 2)
        self.qtgui_const_sink_x_1_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, "")
        self.qtgui_const_sink_x_1_0.enable_autoscale(False)
        self.qtgui_const_sink_x_1_0.enable_grid(False)
        self.qtgui_const_sink_x_1_0.enable_axis_labels(True)

        self.qtgui_const_sink_x_1_0.disable_legend()

        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "red", "red", "red",
            "red", "red", "red", "red", "red"]
        styles = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        markers = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_const_sink_x_1_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_const_sink_x_1_0.set_line_label(i, labels[i])
            self.qtgui_const_sink_x_1_0.set_line_width(i, widths[i])
            self.qtgui_const_sink_x_1_0.set_line_color(i, colors[i])
            self.qtgui_const_sink_x_1_0.set_line_style(i, styles[i])
            self.qtgui_const_sink_x_1_0.set_line_marker(i, markers[i])
            self.qtgui_const_sink_x_1_0.set_line_alpha(i, alphas[i])

        self._qtgui_const_sink_x_1_0_win = sip.wrapinstance(self.qtgui_const_sink_x_1_0.qwidget(), Qt.QWidget)
        self.tabs_grid_layout_2.addWidget(self._qtgui_const_sink_x_1_0_win, 0, 0, 1, 1)
        for r in range(0, 1):
            self.tabs_grid_layout_2.setRowStretch(r, 1)
        for c in range(0, 1):
            self.tabs_grid_layout_2.setColumnStretch(c, 1)
        self.qtgui_const_sink_x_1 = qtgui.const_sink_c(
            1024, #size
            "PL Framer Output", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_const_sink_x_1.set_update_time(0.10)
        self.qtgui_const_sink_x_1.set_y_axis((-2), 2)
        self.qtgui_const_sink_x_1.set_x_axis((-2), 2)
        self.qtgui_const_sink_x_1.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, "")
        self.qtgui_const_sink_x_1.enable_autoscale(False)
        self.qtgui_const_sink_x_1.enable_grid(False)
        self.qtgui_const_sink_x_1.enable_axis_labels(True)

        self.qtgui_const_sink_x_1.disable_legend()

        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "red", "red", "red",
            "red", "red", "red", "red", "red"]
        styles = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        markers = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_const_sink_x_1.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_const_sink_x_1.set_line_label(i, labels[i])
            self.qtgui_const_sink_x_1.set_line_width(i, widths[i])
            self.qtgui_const_sink_x_1.set_line_color(i, colors[i])
            self.qtgui_const_sink_x_1.set_line_style(i, styles[i])
            self.qtgui_const_sink_x_1.set_line_marker(i, markers[i])
            self.qtgui_const_sink_x_1.set_line_alpha(i, alphas[i])

        self._qtgui_const_sink_x_1_win = sip.wrapinstance(self.qtgui_const_sink_x_1.qwidget(), Qt.QWidget)
        self.tabs_grid_layout_2.addWidget(self._qtgui_const_sink_x_1_win, 0, 1, 1, 1)
        for r in range(0, 1):
            self.tabs_grid_layout_2.setRowStretch(r, 1)
        for c in range(1, 2):
            self.tabs_grid_layout_2.setColumnStretch(c, 1)
        self.fir_filter_xxx_0_1 = filter.fir_filter_ccc(1, fir_taps)
        self.fir_filter_xxx_0_1.declare_sample_delay(0)
        self.fir_filter_xxx_0_0 = filter.fir_filter_ccc(1, fir_taps)
        self.fir_filter_xxx_0_0.declare_sample_delay(0)
        self.fir_filter_xxx_0 = filter.fir_filter_ccc(1, fir_taps)
        self.fir_filter_xxx_0.declare_sample_delay(0)
        self.fft_vxx_0 = fft.fft_vcc(4096, True, window.blackmanharris(4096), True, 1)
        self.epy_block_1 = epy_block_1.blk(ber_count=1000, debug_level=0)
        self.epy_block_0 = epy_block_0.blk(fer_count=1000, debug_level=0)
        self.dvbs2rx_xfecframe_demapper_cb_0 = dvbs2rx.xfecframe_demapper_cb(
          *dvbs2rx.params.translate("DVB-S2", frame_size, code_rate,
          constellation)[1:])
        self.dvbs2rx_plsync_cc_0 = dvbs2rx.plsync_cc(0, 10, sps, 0, True, False, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF)
        self.dvbs2rx_ldpc_decoder_bb_0 = dvbs2rx.ldpc_decoder_bb(
            *dvbs2rx.params.translate('DVB-S2',
                frame_size,
                code_rate,
                constellation
            ),
            dvbs2rx.OM_MESSAGE,
            dvbs2rx.INFO_OFF,
            25,
            debug)
        self.dvbs2rx_bch_decoder_bb_0 = dvbs2rx.bch_decoder_bb(
            *dvbs2rx.params.translate('DVB-S2',
                frame_size,
                code_rate
            ),
            dvbs2rx.OM_MESSAGE,
            debug)
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
            debug
        )
        self.dvbs2rx_agc_loop_xx_0 = dvbs2rx.agc_loop_xx(4096, (1e-4), 0.0, (1e-4), 1.0, 1.0, 65536)
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
            rrc_nfilts,
            firdes.root_raised_cosine(rrc_nfilts, samp_rate*rrc_nfilts, sym_rate, rolloff, (int(2*rrc_delay*sps) + 1)*rrc_nfilts))
        self.digital_costas_loop_cc_0 = digital.costas_loop_cc(costas_loop_bw, cl_order, False)
        self.blocks_sub_xx_0 = blocks.sub_ff(1)
        self.blocks_stream_to_vector_1 = blocks.stream_to_vector(gr.sizeof_gr_complex*1, 4096)
        self.blocks_rms_xx_1_0 = blocks.rms_cf(0.0001)
        self.blocks_rms_xx_1 = blocks.rms_cf(0.0001)
        self.blocks_rms_xx_0 = blocks.rms_cf(0.0001)
        self.blocks_nlog10_ff_1 = blocks.nlog10_ff(10, 4096, 0)
        self.blocks_nlog10_ff_0_0 = blocks.nlog10_ff(1, 1, 0)
        self.blocks_nlog10_ff_0 = blocks.nlog10_ff(1, 1, 0)
        self.blocks_multiply_const_xx_0_1_0 = blocks.multiply_const_cc(0.000976562 / 4, 4096)
        self.blocks_multiply_const_xx_0_0 = blocks.multiply_const_ff(20, 1)
        self.blocks_multiply_const_xx_0 = blocks.multiply_const_ff(20, 1)
        self.blocks_moving_average_xx_0 = blocks.moving_average_ff(100, 0.01, 400, 4096)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_char*1, '/home/semikozov/code/radio/dvb-s2-demodulator/demodulated.ts', False)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.blocks_complex_to_mag_squared_1_0 = blocks.complex_to_mag_squared(4096)
        self.blocks_add_xx_0 = blocks.add_vcc(1)
        self.analog_noise_source_x_0 = analog.noise_source_c(analog.GR_GAUSSIAN, noise, 0)
        self.analog_agc_xx_0 = analog.agc_cc(0.01, 1.0, 1.0, 65536)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.dvbs2rx_agc_loop_xx_0, 'gain_cmd'), (self.sdrplay3_rsp1_0, 'cmd'))
        self.msg_connect((self.dvbs2rx_ldpc_decoder_bb_0, 'llr_pdu'), (self.dvbs2rx_xfecframe_demapper_cb_0, 'llr_pdu'))
        self.msg_connect((self.epy_block_0, 'fer_counter'), (self.qtgui_edit_box_msg_1, 'val'))
        self.msg_connect((self.epy_block_1, 'ber_counter'), (self.qtgui_edit_box_msg_0, 'val'))
        self.connect((self.analog_agc_xx_0, 0), (self.blocks_add_xx_0, 0))
        self.connect((self.analog_agc_xx_0, 0), (self.fir_filter_xxx_0_0, 0))
        self.connect((self.analog_agc_xx_0, 0), (self.qtgui_freq_sink_x_1, 0))
        self.connect((self.analog_agc_xx_0, 0), (self.qtgui_freq_sink_x_1_0, 0))
        self.connect((self.analog_noise_source_x_0, 0), (self.blocks_add_xx_0, 1))
        self.connect((self.analog_noise_source_x_0, 0), (self.fir_filter_xxx_0_1, 0))
        self.connect((self.analog_noise_source_x_0, 0), (self.qtgui_freq_sink_x_1, 1))
        self.connect((self.blocks_add_xx_0, 0), (self.blocks_rms_xx_0, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.blocks_stream_to_vector_1, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.fir_filter_xxx_0, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.qtgui_freq_sink_x_0, 0))
        self.connect((self.blocks_complex_to_mag_squared_1_0, 0), (self.blocks_nlog10_ff_1, 0))
        self.connect((self.blocks_moving_average_xx_0, 0), (self.dvbs2rx_agc_loop_xx_0, 0))
        self.connect((self.blocks_moving_average_xx_0, 0), (self.qtgui_vector_sink_f_0, 0))
        self.connect((self.blocks_multiply_const_xx_0, 0), (self.blocks_sub_xx_0, 1))
        self.connect((self.blocks_multiply_const_xx_0_0, 0), (self.blocks_sub_xx_0, 0))
        self.connect((self.blocks_multiply_const_xx_0_1_0, 0), (self.blocks_complex_to_mag_squared_1_0, 0))
        self.connect((self.blocks_nlog10_ff_0, 0), (self.blocks_multiply_const_xx_0, 0))
        self.connect((self.blocks_nlog10_ff_0_0, 0), (self.blocks_multiply_const_xx_0_0, 0))
        self.connect((self.blocks_nlog10_ff_1, 0), (self.blocks_moving_average_xx_0, 0))
        self.connect((self.blocks_rms_xx_0, 0), (self.qtgui_number_sink_0, 0))
        self.connect((self.blocks_rms_xx_1, 0), (self.blocks_nlog10_ff_0_0, 0))
        self.connect((self.blocks_rms_xx_1_0, 0), (self.blocks_nlog10_ff_0, 0))
        self.connect((self.blocks_stream_to_vector_1, 0), (self.fft_vxx_0, 0))
        self.connect((self.blocks_sub_xx_0, 0), (self.qtgui_number_sink_2, 0))
        self.connect((self.digital_costas_loop_cc_0, 0), (self.dvbs2rx_plsync_cc_0, 0))
        self.connect((self.digital_costas_loop_cc_0, 0), (self.qtgui_eye_sink_x_0, 0))
        self.connect((self.digital_symbol_sync_xx_1, 0), (self.digital_costas_loop_cc_0, 0))
        self.connect((self.digital_symbol_sync_xx_1, 0), (self.qtgui_const_sink_x_1_0, 0))
        self.connect((self.dvbs2rx_bbdeheader_bb_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.dvbs2rx_bbdeheader_bb_0, 0), (self.epy_block_0, 0))
        self.connect((self.dvbs2rx_bbdeheader_bb_0, 0), (self.epy_block_1, 0))
        self.connect((self.dvbs2rx_bbdescrambler_bb_0, 0), (self.dvbs2rx_bbdeheader_bb_0, 0))
        self.connect((self.dvbs2rx_bch_decoder_bb_0, 0), (self.dvbs2rx_bbdescrambler_bb_0, 0))
        self.connect((self.dvbs2rx_ldpc_decoder_bb_0, 0), (self.dvbs2rx_bch_decoder_bb_0, 0))
        self.connect((self.dvbs2rx_plsync_cc_0, 0), (self.dvbs2rx_xfecframe_demapper_cb_0, 0))
        self.connect((self.dvbs2rx_plsync_cc_0, 0), (self.qtgui_const_sink_x_1, 0))
        self.connect((self.dvbs2rx_plsync_cc_0, 0), (self.qtgui_time_sink_x_1, 0))
        self.connect((self.dvbs2rx_xfecframe_demapper_cb_0, 0), (self.dvbs2rx_ldpc_decoder_bb_0, 0))
        self.connect((self.epy_block_0, 0), (self.qtgui_number_sink_1, 0))
        self.connect((self.epy_block_1, 0), (self.qtgui_number_sink_1, 1))
        self.connect((self.fft_vxx_0, 0), (self.blocks_multiply_const_xx_0_1_0, 0))
        self.connect((self.fir_filter_xxx_0, 0), (self.digital_symbol_sync_xx_1, 0))
        self.connect((self.fir_filter_xxx_0_0, 0), (self.blocks_rms_xx_1, 0))
        self.connect((self.fir_filter_xxx_0_1, 0), (self.blocks_rms_xx_1_0, 0))
        self.connect((self.sdrplay3_rsp1_0, 0), (self.analog_agc_xx_0, 0))
        self.connect((self.sdrplay3_rsp1_0, 0), (self.qtgui_waterfall_sink_x_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "dvbs2_rx")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_agc_gain(self):
        return self.agc_gain

    def set_agc_gain(self, agc_gain):
        self.agc_gain = agc_gain

    def get_agc_rate(self):
        return self.agc_rate

    def set_agc_rate(self, agc_rate):
        self.agc_rate = agc_rate

    def get_agc_ref(self):
        return self.agc_ref

    def set_agc_ref(self, agc_ref):
        self.agc_ref = agc_ref

    def get_debug(self):
        return self.debug

    def set_debug(self, debug):
        self.debug = debug

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

    def get_snr(self):
        return self.snr

    def set_snr(self, snr):
        self.snr = snr
        self.set_noise(self.snr)

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_fir_taps(firdes.root_raised_cosine(1.0, float(self.samp_rate), float(self.sym_rate), 0.35, (int(self.sps*20))))
        self.set_samp_rate(self.sym_rate * self.sps)
        self.digital_symbol_sync_xx_1.set_sps(self.sps)

    def get_sym_rate(self):
        return self.sym_rate

    def set_sym_rate(self, sym_rate):
        self.sym_rate = sym_rate
        self.set_fir_taps(firdes.root_raised_cosine(1.0, float(self.samp_rate), float(self.sym_rate), 0.35, (int(self.sps*20))))
        self.set_samp_rate(self.sym_rate * self.sps)

    def get_sym_sync_damping(self):
        return self.sym_sync_damping

    def set_sym_sync_damping(self, sym_sync_damping):
        self.sym_sync_damping = sym_sync_damping
        self.digital_symbol_sync_xx_1.set_damping_factor(self.sym_sync_damping)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_fir_taps(firdes.root_raised_cosine(1.0, float(self.samp_rate), float(self.sym_rate), 0.35, (int(self.sps*20))))
        self.qtgui_eye_sink_x_0.set_samp_rate(self.samp_rate)
        self.qtgui_freq_sink_x_0.set_frequency_range(0, self.samp_rate)
        self.qtgui_freq_sink_x_1.set_frequency_range(0, self.samp_rate)
        self.qtgui_freq_sink_x_1_0.set_frequency_range(0, self.samp_rate)
        self.qtgui_time_sink_x_1.set_samp_rate(self.samp_rate)
        self.qtgui_waterfall_sink_x_0.set_frequency_range(0, self.samp_rate)
        self.sdrplay3_rsp1_0.set_sample_rate(self.samp_rate)

    def get_qpsk(self):
        return self.qpsk

    def set_qpsk(self, qpsk):
        self.qpsk = qpsk

    def get_code_rate(self):
        return self.code_rate

    def set_code_rate(self, code_rate):
        self.code_rate = code_rate
        self.set_constellation(modcod.replace(self.code_rate, ""))

    def get_variable_adaptive_algorithm(self):
        return self.variable_adaptive_algorithm

    def set_variable_adaptive_algorithm(self, variable_adaptive_algorithm):
        self.variable_adaptive_algorithm = variable_adaptive_algorithm

    def get_sym_sync_loop_bw(self):
        return self.sym_sync_loop_bw

    def set_sym_sync_loop_bw(self, sym_sync_loop_bw):
        self.sym_sync_loop_bw = sym_sync_loop_bw
        self.digital_symbol_sync_xx_1.set_loop_bandwidth(self.sym_sync_loop_bw)

    def get_qtgui_range_lna_gain(self):
        return self.qtgui_range_lna_gain

    def set_qtgui_range_lna_gain(self, qtgui_range_lna_gain):
        self.qtgui_range_lna_gain = qtgui_range_lna_gain
        self.sdrplay3_rsp1_0.set_gain(self.qtgui_range_lna_gain, 'LNAstate')

    def get_qtgui_range_if_gain(self):
        return self.qtgui_range_if_gain

    def set_qtgui_range_if_gain(self, qtgui_range_if_gain):
        self.qtgui_range_if_gain = qtgui_range_if_gain
        self.sdrplay3_rsp1_0.set_gain(-(self.qtgui_range_if_gain), 'IF')

    def get_qtgui_range_agc_setpoint(self):
        return self.qtgui_range_agc_setpoint

    def set_qtgui_range_agc_setpoint(self, qtgui_range_agc_setpoint):
        self.qtgui_range_agc_setpoint = qtgui_range_agc_setpoint

    def get_plheader_len(self):
        return self.plheader_len

    def set_plheader_len(self, plheader_len):
        self.plheader_len = plheader_len

    def get_plframe_len(self):
        return self.plframe_len

    def set_plframe_len(self, plframe_len):
        self.plframe_len = plframe_len

    def get_pilot_len(self):
        return self.pilot_len

    def set_pilot_len(self, pilot_len):
        self.pilot_len = pilot_len

    def get_noise(self):
        return self.noise

    def set_noise(self, noise):
        self.noise = noise
        self.analog_noise_source_x_0.set_amplitude(self.noise)

    def get_n_taps(self):
        return self.n_taps

    def set_n_taps(self, n_taps):
        self.n_taps = n_taps

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.sdrplay3_rsp1_0.set_center_freq(self.freq)

    def get_fir_taps(self):
        return self.fir_taps

    def set_fir_taps(self, fir_taps):
        self.fir_taps = fir_taps
        self.fir_filter_xxx_0.set_taps(self.fir_taps)
        self.fir_filter_xxx_0_0.set_taps(self.fir_taps)
        self.fir_filter_xxx_0_1.set_taps(self.fir_taps)

    def get_costas_loop_bw(self):
        return self.costas_loop_bw

    def set_costas_loop_bw(self, costas_loop_bw):
        self.costas_loop_bw = costas_loop_bw
        self.digital_costas_loop_cc_0.set_loop_bandwidth(self.costas_loop_bw)

    def get_constellation(self):
        return self.constellation

    def set_constellation(self, constellation):
        self.constellation = constellation

    def get_cl_order(self):
        return self.cl_order

    def set_cl_order(self, cl_order):
        self.cl_order = cl_order

    def get_Es(self):
        return self.Es

    def set_Es(self, Es):
        self.Es = Es



def argument_parser():
    description = 'Full DVB-S2 receiver. Processes IQ samples from stdin and outputs MPEG TS packets to stdout.'
    parser = ArgumentParser(description=description)
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
        "-d", "--debug", dest="debug", type=intx, default=0,
        help="Set debugging level [default=%(default)r]")
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
        "-r", "--rolloff", dest="rolloff", type=eng_float, default=eng_notation.num_to_str(float(0.2)),
        help="Set rolloff factor [default=%(default)r]")
    parser.add_argument(
        "--rrc-delay", dest="rrc_delay", type=intx, default=25,
        help="Set RRC filter delay in symbol periods [default=%(default)r]")
    parser.add_argument(
        "--rrc-nfilts", dest="rrc_nfilts", type=intx, default=128,
        help="Set number of branches on the polyphase RRC filter [default=%(default)r]")
    parser.add_argument(
        "--snr", dest="snr", type=eng_float, default=eng_notation.num_to_str(float(1.2)),
        help="Set starting SNR in dB [default=%(default)r]")
    parser.add_argument(
        "-o", "--sps", dest="sps", type=eng_float, default=eng_notation.num_to_str(float(5)),
        help="Set oversampling ratio in samples per symbol [default=%(default)r]")
    parser.add_argument(
        "-s", "--sym-rate", dest="sym_rate", type=intx, default=2000000,
        help="Set symbol rate in bauds [default=%(default)r]")
    parser.add_argument(
        "--sym-sync-damping", dest="sym_sync_damping", type=eng_float, default=eng_notation.num_to_str(float(1.6)),
        help="Set symbol synchronizer's damping factor [default=%(default)r]")
    return parser


def main(top_block_cls=dvbs2_rx, options=None):
    if options is None:
        options = argument_parser().parse_args()

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls(agc_gain=options.agc_gain, agc_rate=options.agc_rate, agc_ref=options.agc_ref, debug=options.debug, frame_size=options.frame_size, gold_code=options.gold_code, modcod=options.modcod, pl_freq_est_period=options.pl_freq_est_period, rolloff=options.rolloff, rrc_delay=options.rrc_delay, rrc_nfilts=options.rrc_nfilts, snr=options.snr, sps=options.sps, sym_rate=options.sym_rate, sym_sync_damping=options.sym_sync_damping)

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
