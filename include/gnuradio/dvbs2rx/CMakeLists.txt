# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file was generated by gr_modtool, a tool from the GNU Radio framework
# This file is a part of gr-dvbs2rx
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Install public header files
########################################################################
install(FILES
    api.h
    bbdeheader_bb.h
    bbdescrambler_bb.h
    bch_decoder_bb.h
    ldpc_decoder_bb.h
    plsync_cc.h
    rotator_cc.h
    symbol_sync_cc.h
    xfecframe_demapper_cb.h
    plframer_cc.h
    DESTINATION include/gnuradio/dvbs2rx
)
