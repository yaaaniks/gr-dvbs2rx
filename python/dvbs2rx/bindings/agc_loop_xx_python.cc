/*
 * Copyright 2020 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(bbdeheader_bb.h)                                           */
/* BINDTOOL_HEADER_FILE_HASH(578413664762f19a0cacbaaec4388df9)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/dvbs2rx/agc_loop_xx.h>
// pydoc.h is automatically generated in the build directory
#include <agc_loop_xx_pydoc.h>

void bind_agc_loop_xx(py::module& m)
{

    using agc_loop_xx = ::gr::dvbs2rx::agc_loop_xx;


    py::class_<agc_loop_xx,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<agc_loop_xx>>(m, "agc_loop_xx", D(agc_loop_xx))

        .def(py::init(&agc_loop_xx::make),
             py::arg("vlen"),
             py::arg("hw_agc_if_gain"),
             py::arg("hw_agc_lna_state"),
             py::arg("sw_agc_rate"),
             py::arg("sw_agc_reference"),
             py::arg("sw_agc_gain"),
             py::arg("sw_agc_max_gain") = 0,
             D(agc_loop_xx, make))


        .def("get_current_rf_gain",
             &agc_loop_xx::get_current_rf_gain,
             D(agc_loop_xx, get_current_rf_gain))

        .def("get_hw_if_gain",
             &agc_loop_xx::get_hw_if_gain,
             D(agc_loop_xx, get_hw_if_gain))

        .def("get_hw_lna_gain",
             &agc_loop_xx::get_hw_lna_gain,
             D(agc_loop_xx, get_hw_lna_gain))

        .def("set_hw_agc_if_gain",
             &agc_loop_xx::set_hw_agc_if_gain,
             D(agc_loop_xx, set_hw_agc_if_gain))

        .def("set_hw_agc_lna_state",
             &agc_loop_xx::set_hw_agc_lna_state,
             D(agc_loop_xx, set_hw_agc_lna_state))

        .def("set_sw_agc_rate",
             &agc_loop_xx::set_sw_agc_rate,
             D(agc_loop_xx, set_sw_agc_rate))

        .def("set_sw_agc_reference",
             &agc_loop_xx::set_sw_agc_reference,
             D(agc_loop_xx, set_sw_agc_reference))

        .def("set_sw_agc_gain",
             &agc_loop_xx::set_sw_agc_gain,
             D(agc_loop_xx, set_sw_agc_gain))

        .def("set_sw_agc_max_gain",
             &agc_loop_xx::set_sw_agc_max_gain,
             D(agc_loop_xx, set_sw_agc_max_gain));
}