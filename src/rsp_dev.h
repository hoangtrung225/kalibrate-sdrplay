/* -*- c++ -*- */
/*
 * Copyright 2018 HB9FXQ, Frank Werner-Krippendorf.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef GR_SDRPLAY_RSP_DEV_H
#define GR_SDRPLAY_RSP_DEV_H

#include <mirsdrapi-rsp.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <gnuradio/gr_complex.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace gr {
    namespace sdrplay {

        class rsp_dev {
        public:
            rsp_dev();

            ~rsp_dev();

            void list_available_rsp_devices();

            double set_sample_rate(double rate);

            double get_sample_rate(void) const;

            double set_center_freq(double freq);

            double get_center_freq() const;

            bool set_gain_mode(bool automatic);

            bool get_gain_mode() const;

            double set_gain(double gain);

            double set_gain(double gain, const std::string &name);

            double get_gain() const;

            double get_gain(const std::string &name) const;

            std::string set_antenna(const std::string &antenna);

            std::string get_antenna() const;

            void set_dc_offset_mode(int mode);

            void set_iq_balance_mode(int mode);

            void set_debug_mode(int mode);

            double set_bandwidth(double bandwidth);

            double get_bandwidth() const;

            void startStreaming(void);

            void stopStreaming(void);

            int fetch_work_buffer(gr_complex *grWorkBuffer, int noutput_items);

            void set_if_type(int ifType);

            void set_lo_mode(int lo_mode);

            void set_biasT(bool biasT);

            void set_deviceIndexOrSerial(const std::string &deviceIndexOrSerial);

        private:
            void reinitDevice(int reason);

            int checkLNA(int lna);

            void streamCallback(short *xi, short *xq, unsigned int firstSampleNum,
                                int grChanged, int rfChanged, int fsChanged,
                                unsigned int numSamples, unsigned int reset);

            void gainChangeCallback(unsigned int gRdB, unsigned int lnaGRdB);

            static void streamCallbackWrap(short *xi, short *xq, unsigned int firstSampleNum,
                                           int grChanged, int rfChanged, int fsChanged,
                                           unsigned int numSamples, unsigned int reset,
                                           unsigned int hwRemoved, void *cbContext);

            static void gainChangeCallbackWrap(unsigned int gRdB, unsigned int lnaGRdB, void *cbContext);

            bool _auto_gain;
            int _gRdB;
            int _lna;
            int _bcastNotch;
            int _dabNotch;
            double _fsHz;
            int _decim;
            double _rfHz;
            mir_sdr_Bw_MHzT _bwType;
            mir_sdr_If_kHzT _ifType;
            mir_sdr_LoModeT _loMode;
            int _samplesPerPacket;
            bool _dcMode;
            bool _iqMode;
            bool _debug;
            unsigned char _hwVer;
            unsigned int _devIndex;
            std::string _antenna;
            int _biasT;
            std::string _deviceIndexOrSerial;

            bool _streaming;
            gr_complex *_buffer;
            int _bufferOffset;
            int _bufferSpaceRemaining;
            boost::mutex _bufferMutex;
            boost::condition_variable _bufferReady;  // buffer is ready to move to other thread

            bool _reinit;  // signal streamer to return after a reinit

        };
    }
}


#endif //GR_SDRPLAY_RSP_DEV_H
