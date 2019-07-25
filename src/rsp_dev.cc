/* -*- c++ -*- */
/*
 * gr-sdrplay Copyright 2018 HB9FXQ, Frank Werner-Krippendorf.
 *
 * Credits for the rsp_dev class goes go to:
 * gr-osmosdr Copyright 2018 Jeff Long <willcode4@gmail.com> of the gr-osmosdr-fork!
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

#include "rsp_dev.h"
#include <gnuradio/io_signature.h>

#include <boost/assign.hpp>
#include <boost/format.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono.hpp>

#include <iostream>
#include <string>
#include <mutex>
#include <mirsdrapi-rsp.h>

namespace gr
{
namespace sdrplay
{

#define MAX_SUPPORTED_DEVICES 4

using namespace boost::assign;

// Index by mir_sdr_Bw_MHzT
static std::vector<double> bandwidths = {
    0, // Dummy
    200e3,
    300e3,
    600e3,
    1536e3,
    5000e3,
    6000e3,
    7000e3,
    8000e3};

static std::string hwName(int hwVer)
{
    if (hwVer == 1)
        return "RSP1";
    if (hwVer == 2)
        return "RSP2";
    if (hwVer == 255)
        return "RSP1A";
    if (hwVer == 3)
        return "RSPduo";
    return "UNK";
}

void rsp_dev::
    list_available_rsp_devices()
{

    unsigned int _devIndex = 0;
    unsigned int numDevices;

    mir_sdr_DeviceT mirDevices[MAX_SUPPORTED_DEVICES];
    mir_sdr_GetDevices(mirDevices, &numDevices, MAX_SUPPORTED_DEVICES);

    if (((_devIndex + 1 > numDevices) || !mirDevices[_devIndex].devAvail))
    {
        std::cerr << "Failed to open SDRplay device " << std::endl;
        throw std::runtime_error("Failed to open SDRplay device ");
    }

    for (int i = 0; i < numDevices; i++)
    {

        std::cerr << "RSP devIndex: [" << i << "] " << hwName(mirDevices[i].hwVer) << " " << mirDevices[i].SerNo
                  << "\r\n";
    }
}

rsp_dev::rsp_dev()
{
    _samplesPerPacket = -1;
    _hwVer = -1;
    _biasT = 0;
    _bufferOffset = 0;
    _bufferSpaceRemaining = 0;
    _auto_gain = true;
    _gRdB = 40;
    _lna = 0;
    _bcastNotch = 0;
    _dabNotch = 0;
    _fsHz = 2e6;
    _decim = 1;
    _rfHz = 100e6;
    _bwType = mir_sdr_BW_0_300;
    _ifType = mir_sdr_IF_Zero;
    _loMode = mir_sdr_LO_Auto;
    _dcMode = true;
    _iqMode = true;
    _buffer = NULL;
    _streaming = false;
    _reinit = false;
    _devIndex = 0;
    _debug = 0;
}

rsp_dev::~rsp_dev()
{
    if (_streaming)
    {
        stopStreaming();
    }
}

// Called by sdrplay streamer thread when data is available
void rsp_dev::streamCallback(short *xi, short *xq,
                                          unsigned int firstSampleNum,
                                          int grChanged, int rfChanged, int fsChanged,
                                          unsigned int numSamples, unsigned int reset)
{
    unsigned int i = 0;
    _reinit = false;

    boost::mutex::scoped_lock lock(_bufferMutex);

    while (i < numSamples)
    {
        if (!_streaming || _reinit)
        {
            return;
        }

        while (!_buffer)
        {
            if (boost::cv_status::timeout ==
                _bufferReady.wait_for(lock, boost::chrono::milliseconds(250)))
                return;
        }

        while ((i < numSamples) && (_bufferSpaceRemaining > 0))
        {
            _buffer[_bufferOffset] =
                gr_complex(float(xi[i]) / 32768.0, float(xq[i]) / 32768.0);
            i++;
            _bufferOffset++;
            _bufferSpaceRemaining--;
        }

        if (_bufferSpaceRemaining == 0)
        {
            _buffer = NULL;
            _bufferReady.notify_one();
        }
    }
}

// Callback wrapper
void rsp_dev::streamCallbackWrap(short *xi, short *xq,
                                              unsigned int firstSampleNum,
                                              int grChanged, int rfChanged, int fsChanged,
                                              unsigned int numSamples, unsigned int reset, unsigned int hwRemoved,
                                              void *cbContext)
{
    rsp_dev *obj = (rsp_dev *)cbContext;
    obj->streamCallback(xi, xq,
                        firstSampleNum,
                        grChanged, rfChanged, fsChanged,
                        numSamples, reset);
}

// Called by strplay streamer thread when gain reduction is changed.
void rsp_dev::gainChangeCallback(unsigned int gRdB,
                                              unsigned int lnaGRdB)
{
    mir_sdr_GainValuesT gainVals;
    mir_sdr_GetCurrentGain(&gainVals);

    if (gRdB < 200 && _debug)
    {
        std::cerr << "GR change, BB+MIX -" << gRdB << "dB, LNA -" << lnaGRdB << std::endl;
    }

    if (gRdB < mir_sdr_GAIN_MESSAGE_START_ID)
    {
        // gainVals.curr is a calibrated gain value
    }
    else if (gRdB == mir_sdr_ADC_OVERLOAD_DETECTED)
    {
        mir_sdr_GainChangeCallbackMessageReceived();
        // OVERLOAD DETECTED
    }
    else
    {
        mir_sdr_GainChangeCallbackMessageReceived();
        // OVERLOAD CORRECTED
    }
}

// Callback wrapper
void rsp_dev::gainChangeCallbackWrap(unsigned int gRdB,
                                                  unsigned int lnaGRdB,
                                                  void *cbContext)
{
    rsp_dev *obj = (rsp_dev *)cbContext;
    obj->gainChangeCallback(gRdB, lnaGRdB);
}

void rsp_dev::startStreaming(void)
{
    if (_streaming)
    {
        return;
    }

    unsigned int numDevices;
    mir_sdr_DeviceT mirDevices[MAX_SUPPORTED_DEVICES];
    mir_sdr_ReleaseDeviceIdx();
    mir_sdr_GetDevices(mirDevices, &numDevices, MAX_SUPPORTED_DEVICES);

    if (_deviceIndexOrSerial.length() > 2 /*It's a SerialNo*/)
    {

        bool match = false;

        for (int i = 0; i < numDevices; i++)
        {
            if (_deviceIndexOrSerial.compare(std::string(mirDevices[i].SerNo)) == 0)
            {
                std::cerr << "Found requested RSP with SerialNO: " << mirDevices[i].SerNo << "\r\n";
                _devIndex = i;
                match = true;
                break;
            }
        }

        if (!match)
        {
            std::cerr << "FALLBACK TO DEV INDEX = !!!! Could NOT find RSP SerialNO: " << _deviceIndexOrSerial << "\r\n";
        }
    }

    _hwVer = mirDevices[_devIndex].hwVer;
    mir_sdr_SetDeviceIdx(_devIndex);

    std::cerr << "Using SDRplay " << hwName(_hwVer) << " "
              << mirDevices[_devIndex].SerNo << std::endl;

    set_biasT(_biasT);

    // Set first LO frequency
    mir_sdr_SetLoMode(_loMode);

    _streaming = true;

    set_gain_mode(get_gain_mode());

    int gRdB = _gRdB;
    int gRdBsystem = 0;

    mir_sdr_StreamInit(&gRdB,
                       _fsHz / 1e6,
                       _rfHz / 1e6,
                       _bwType,
                       _ifType,
                       checkLNA(_lna),
                       &gRdBsystem,
                       mir_sdr_USE_RSP_SET_GR,
                       &_samplesPerPacket,
                       &streamCallbackWrap,
                       &gainChangeCallbackWrap,
                       this);

    // Set decimation with halfband filter
    mir_sdr_DecimateControl(_decim != 1, _decim, 1);

    mir_sdr_DCoffsetIQimbalanceControl(_dcMode, _iqMode);

    // Model-specific initialization
    if (_hwVer == 2)
    {
        set_antenna(get_antenna());
        mir_sdr_RSPII_RfNotchEnable(_bcastNotch);
    }
    else if (_hwVer == 3)
    {
        set_antenna(get_antenna());
        if (_antenna == "HIGHZ")
            mir_sdr_rspDuo_Tuner1AmNotch(_bcastNotch);
        else
            mir_sdr_rspDuo_BroadcastNotch(_bcastNotch);
        mir_sdr_rspDuo_DabNotch(_dabNotch);
    }
    else if (_hwVer == 255)
    {
        mir_sdr_rsp1a_BroadcastNotch(_bcastNotch);
        mir_sdr_rsp1a_DabNotch(_dabNotch);
    }
}

void rsp_dev::stopStreaming(void)
{
    if (!_streaming)
        return;

    _streaming = false;

    mir_sdr_StreamUninit();
    mir_sdr_ReleaseDeviceIdx();
}

void rsp_dev::reinitDevice(int reason)
{
    // If no reason given, reinit everything
    if (reason == (int)mir_sdr_CHANGE_NONE)
        reason = (mir_sdr_CHANGE_GR |
                  mir_sdr_CHANGE_FS_FREQ |
                  mir_sdr_CHANGE_RF_FREQ |
                  mir_sdr_CHANGE_BW_TYPE |
                  mir_sdr_CHANGE_IF_TYPE |
                  mir_sdr_CHANGE_LO_MODE |
                  mir_sdr_CHANGE_AM_PORT);

    int gRdB;
    int gRdBsystem; // Returned overall system gain reduction

    gRdB = _gRdB;

    // Tell stream CB to return
    _reinit = true;

    mir_sdr_Reinit(&gRdB,
                   _fsHz / 1e6,
                   _rfHz / 1e6,
                   _bwType,
                   _ifType,
                   _loMode,
                   checkLNA(_lna),
                   &gRdBsystem,
                   mir_sdr_USE_RSP_SET_GR,
                   &_samplesPerPacket,
                   (mir_sdr_ReasonForReinitT)reason);

    // Set decimation with halfband filter
    if (reason & (int)mir_sdr_CHANGE_FS_FREQ)
        mir_sdr_DecimateControl(_decim != 1, _decim, 1);

    _bufferReady.notify_one();
}

double rsp_dev::set_sample_rate(double rate)
{
    rate = std::min(std::max(rate, 62.5e3), 10e6);
    _fsHz = rate;

    // Decimation is required for rates below 2MS/s
    _decim = 1;
    while (_fsHz < 2e6)
    {
        _decim *= 2;
        _fsHz *= 2;
    }

    if (_streaming)
        reinitDevice((int)mir_sdr_CHANGE_FS_FREQ);

    return get_sample_rate();
}

double rsp_dev::get_sample_rate() const
{
    return _fsHz / _decim;
}

double rsp_dev::set_center_freq(double freq)
{
    _rfHz = freq;

    if (_streaming)
    {
        reinitDevice((int)mir_sdr_CHANGE_RF_FREQ);
    }

    return get_center_freq();
}

double rsp_dev::get_center_freq() const
{
    return _rfHz;
}

bool rsp_dev::set_gain_mode(bool automatic)
{
    _auto_gain = automatic;
    if (_streaming)
    {
        if (automatic)
        {
            mir_sdr_AgcControl(mir_sdr_AGC_5HZ /*TODO: expose argument */, -30 /*TODO: magic number */, 0, 0, 0, 0,
                               checkLNA(_lna));
        }
        else
        {
            mir_sdr_AgcControl(mir_sdr_AGC_DISABLE, -30 /*TODO: magic number */, 0, 0, 0, 0, checkLNA(_lna));
        }

        reinitDevice((int)mir_sdr_CHANGE_GR);
    }

    return _auto_gain;
}

bool rsp_dev::get_gain_mode() const
{
    return _auto_gain;
}

int rsp_dev::checkLNA(int lna)
{
    // Maaaagic - see gain tables in API doc
    if (_hwVer == 1)
    {
        lna = std::min(3, lna);
    }
    else if (_hwVer == 255)
    {
        if (_rfHz < 60000000)
            lna = std::min(6, lna);
        else if (_rfHz >= 1000000000)
            lna = std::min(8, lna);
        else
            lna = std::min(9, lna);
    }
    else if (_hwVer == 2)
    {
        if (_rfHz >= 420000000)
            lna = std::min(5, lna);
        else if (_rfHz < 60000000 && _antenna == "HIGHZ")
            lna = std::min(4, lna);
        else
            lna = std::min(8, lna);
    }
    else if (_hwVer == 3)
    {
        if (_rfHz >= 1000000000)
            lna = std::min(8, lna);
        else if (_rfHz < 60000000 && _antenna == "HIGHZ")
            lna = std::min(4, lna);
        else if (_rfHz < 60000000)
            lna = std::min(6, lna);
        else
            lna = std::min(9, lna);
    }

    return lna;
}

double rsp_dev::set_gain(double gain)
{
    set_gain(gain, "IF_ATTEN_DB");
    return get_gain("IF_ATTEN_DB");
}

double rsp_dev::set_gain(double gain, const std::string &name)
{
    bool bcastNotchChanged = false;
    bool dabNotchChanged = false;
    bool gainChanged = false;

    if (name == "LNA_ATTEN_STEP")
    {
        if (gain != _lna) //RSP1 will only send bool / 0||1
            gainChanged = true;
        _lna = int(gain);
    }
    else if (name == "IF_ATTEN_DB")
    {
        // Ignore out-of-bounds values, since caller knows limits. (GQRX spurious calls).
        if (gain >= 20.0 && gain <= 59.0 && gain != _gRdB)
        {
            gainChanged = true;
            _gRdB = int(gain);
        }
    }
    // RSP1A, RSP2
    else if (name == "BCAST_NOTCH" && (_hwVer == 2 || _hwVer == 3 || _hwVer == 255))
    {
        if (int(gain) != _bcastNotch)
            bcastNotchChanged = true;
        _bcastNotch = int(gain);
    }
    // RSP1A
    else if (name == "DAB_NOTCH" && (_hwVer == 3 || _hwVer == 255))
    {
        if (int(gain) != _dabNotch)
            dabNotchChanged = true;
        _dabNotch = int(gain);
    }

    if (_streaming)
    {
        if (gainChanged)
            mir_sdr_RSP_SetGr(_gRdB, checkLNA(_lna), 1 /*absolute*/, 0 /*immediate*/);

        if (bcastNotchChanged)
        {
            if (_hwVer == 255)
            {
                mir_sdr_rsp1a_BroadcastNotch(_bcastNotch);
            }
            else if (_hwVer == 2)
            {
                mir_sdr_RSPII_RfNotchEnable(_bcastNotch);
            }
            else if (_hwVer == 3)
            {
                if (_antenna == "HIGHZ")
                    mir_sdr_rspDuo_Tuner1AmNotch(_bcastNotch);
                else
                    mir_sdr_rspDuo_BroadcastNotch(_bcastNotch);
            }
        }

        if (dabNotchChanged)
        {
            if (_hwVer == 255)
            {
                mir_sdr_rsp1a_DabNotch(_dabNotch);
            }
            else if (_hwVer == 3)
            {
                mir_sdr_rspDuo_DabNotch(_dabNotch);
            }
        }

        if (_streaming)
            reinitDevice((int)mir_sdr_CHANGE_GR);
    }

    return get_gain();
}

double rsp_dev::get_gain() const
{
    return get_gain("IF_ATTEN_DB");
}

double rsp_dev::get_gain(const std::string &name) const
{
    if (name == "LNA_ATTEN_STEP")
        return _lna;
    else if (name == "BCAST_NOTCH")
        return _bcastNotch;
    else if (name == "DAB_NOTCH")
        return _dabNotch;
    else if (name == "IF_ATTEN_DB")
        return _gRdB;
    else
        return 0;
}

std::string rsp_dev::set_antenna(const std::string &antenna)
{
    _antenna = antenna;

    if (_streaming)
    {
        if (_hwVer == 2)
        {
            // HIGHZ is ANTENNA_B with AmPortSelect
            if (antenna == "HIGHZ")
            {
                mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_B);
                mir_sdr_AmPortSelect(1);
            }
            else
            {
                if (antenna == "A")
                    mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_A);
                else
                    mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_B);
                mir_sdr_AmPortSelect(0);
            }

            reinitDevice((int)mir_sdr_CHANGE_AM_PORT);
        }
        else if (_hwVer == 3)
        {
            if (antenna == "HIGHZ")
            {
                mir_sdr_rspDuo_TunerSel(mir_sdr_rspDuo_Tuner_1);
                mir_sdr_AmPortSelect(1);
            }
            else if (antenna == "T1_50ohm")
            {
                mir_sdr_rspDuo_TunerSel(mir_sdr_rspDuo_Tuner_1);
                mir_sdr_AmPortSelect(0);
            }
            else
            {
                mir_sdr_rspDuo_TunerSel(mir_sdr_rspDuo_Tuner_2);
                mir_sdr_AmPortSelect(0);
            }

            reinitDevice((int)mir_sdr_CHANGE_AM_PORT);
        }
    }
    return antenna;
}

std::string rsp_dev::get_antenna() const
{
    return _antenna;
}

void rsp_dev::set_dc_offset_mode(int mode)
{
    _dcMode = mode == 1;
    mir_sdr_DCoffsetIQimbalanceControl(_dcMode, _iqMode);
}

void rsp_dev::set_iq_balance_mode(int mode)
{
    _iqMode = mode == 1;
    mir_sdr_DCoffsetIQimbalanceControl(_dcMode, _iqMode);
}

void rsp_dev::set_debug_mode(int mode)
{
    _debug = mode == 1;
    mir_sdr_DebugEnable(_debug);
}

double rsp_dev::set_bandwidth(double bandwidth)
{
    _bwType = mir_sdr_BW_8_000;

    for (double bw : bandwidths)
    {
        if (bw == 0)
            continue;
        if (bandwidth <= bw)
        {
            _bwType = (mir_sdr_Bw_MHzT)(bw / 1e3);
            break;
        }
    }

    int actual = get_bandwidth();
    std::cerr << "SDRplay bandwidth requested=" << bandwidth
              << " actual=" << actual << std::endl;

    if (_streaming)
    {
        reinitDevice((int)mir_sdr_CHANGE_BW_TYPE);
    }

    return actual;
}

double rsp_dev::get_bandwidth() const
{
    return (double)_bwType * 1e3;
}

int rsp_dev::fetch_work_buffer(gr_complex *grWorkBuffer, int noutput_items)
{
    if (!_streaming)
        startStreaming();

    {
        boost::mutex::scoped_lock lock(_bufferMutex);
        _buffer = grWorkBuffer;
        _bufferSpaceRemaining = noutput_items;
        _bufferOffset = 0;
        _bufferReady.notify_one();

        while (_buffer && _streaming)
        {
            _bufferReady.wait(lock);
        }
    }

    if (_streaming)
    {
        return 0;
    }
}

void rsp_dev::set_if_type(int ifType)
{
    _ifType = (mir_sdr_If_kHzT)ifType;

    if (_streaming)
    {
        reinitDevice((int)mir_sdr_CHANGE_IF_TYPE);
    }
}

void rsp_dev::set_lo_mode(int lo_mode)
{
    _loMode = (mir_sdr_LoModeT)lo_mode;

    if (_streaming)
    {
        reinitDevice((int)mir_sdr_CHANGE_LO_MODE);
    }
}

void rsp_dev::set_biasT(bool biasT)
{
    if (_hwVer == 2)
        mir_sdr_RSPII_BiasTControl(biasT);
    else if (_hwVer == 3)
        mir_sdr_rspDuo_BiasT(biasT);
    else if (_hwVer == 255)
        mir_sdr_rsp1a_BiasT(biasT);
}

void rsp_dev::set_deviceIndexOrSerial(const std::string &deviceIndexOrSerial)
{
    _deviceIndexOrSerial = deviceIndexOrSerial;
}

} // namespace sdrplay
} // namespace gr