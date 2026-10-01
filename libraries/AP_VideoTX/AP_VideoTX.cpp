/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_VideoTX.h"
#include "AP_VideoTX_VT7550.h"

#if AP_VIDEOTX_ENABLED

#include <AP_RCTelemetry/AP_CRSF_Telem.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_VideoTX *AP_VideoTX::singleton;
constexpr uint16_t AP_VideoTX::D1_MAX_POWER_MW;
constexpr uint8_t AP_VideoTX::D1_MAX_POWER_DBM;
static_assert(AP_VideoTX::is_valid_band_channel(0, 0), "first VTX band/channel must be valid");
static_assert(AP_VideoTX::is_valid_band_channel(AP_VideoTX::MAX_BANDS - 1, VTX_MAX_CHANNELS - 1),
              "last VTX band/channel must be valid");
static_assert(!AP_VideoTX::is_valid_band_channel(-1, 0), "negative VTX band must be invalid");
static_assert(!AP_VideoTX::is_valid_band_channel(AP_VideoTX::MAX_BANDS, 0), "upper VTX band bound must be invalid");
static_assert(!AP_VideoTX::is_valid_band_channel(0, -1), "negative VTX channel must be invalid");
static_assert(!AP_VideoTX::is_valid_band_channel(0, VTX_MAX_CHANNELS), "upper VTX channel bound must be invalid");
static_assert(AP_VideoTX::d1_status_is_dbm(34), "34 must be accepted as a D1 dBm status");
static_assert(!AP_VideoTX::d1_status_is_dbm(35), "35 must not be accepted as a D1 dBm status");
static_assert(!AP_VideoTX::d1_status_is_dbm(200), "mW status must not be accepted as D1 dBm");

const AP_Param::GroupInfo AP_VideoTX::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Is the Video Transmitter enabled or not
    // @Description: Toggles the Video Transmitter on and off
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_VideoTX, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: POWER
    // @DisplayName: Video Transmitter Power Level
    // @Description: Video Transmitter Power Level. Different VTXs support different power levels, the power level chosen will be rounded down to the nearest supported power level. Ignored with MODEL=2; use PWR_LEVEL instead. MODEL=3 uses the normal Tramp power setting.
    // @Range: 1 2500
    AP_GROUPINFO("POWER",    2, AP_VideoTX, _power_mw, 0),

    // @Param: CHANNEL
    // @DisplayName: Video Transmitter Channel
    // @Description: Video Transmitter Channel
    // @User: Standard
    // @Range: 0 7
    AP_GROUPINFO("CHANNEL",  3, AP_VideoTX, _channel, 0),

    // @Param: BAND
    // @DisplayName: Video Transmitter Band
    // @Description: Video Transmitter Band
    // @User: Standard
    // @Values: 0:Band A,1:Band B,2:Band E,3:Airwave,4:RaceBand,5:Low RaceBand,6:1G3 Band A,7:1G3 Band B,8:Band X,9:3G3 Band A,10:3G3 Band B,11:Skyzone A,12:Skyzone B,13:Skyzone C,14:Skyzone D,15:Skyzone E,16:Skyzone F,17:Skyzone H,18:Skyzone J,19:MILELRS 12G A,20:MILELRS 12G B,21:MILELRS 12G C
    AP_GROUPINFO("BAND",  4, AP_VideoTX, _band, 0),

    // @Param: FREQ
    // @DisplayName: Video Transmitter Frequency
    // @Description: Video Transmitter Frequency. The frequency is derived from the setting of BAND and CHANNEL
    // @User: Standard
    // @ReadOnly: True
    // @Range: 1000 11900
    AP_GROUPINFO("FREQ",  5, AP_VideoTX, _frequency_mhz, 0),

    // @Param: OPTIONS
    // @DisplayName: Video Transmitter Options
    // @Description: Video Transmitter Options. Pitmode puts the VTX in a low power state. Unlocked enables certain restricted frequencies and power levels. Do not enable the Unlocked option unless you have appropriate permissions in your jurisdiction to transmit at high power levels. One stop-bit may be required for VTXs that erroneously mimic iNav behaviour.
    // @User: Advanced
    // @Bitmask: 0:Pitmode,1:Pitmode until armed,2:Pitmode when disarmed,3:Unlocked,4:Add leading zero byte to requests,5:Use 1 stop-bit in SmartAudio,6:Ignore CRC in SmartAudio,7:Ignore status updates in CRSF and blindly set VTX options
    AP_GROUPINFO("OPTIONS",  6, AP_VideoTX, _options, 0),

    // @Param: MAX_POWER
    // @DisplayName: Video Transmitter Max Power Level
    // @Description: Video Transmitter Maximum Power Level. Different VTXs support different power levels, this prevents the power aux switch from requesting too high a power level. The switch supports 6 power levels and the selected power will be a subdivision between 0 and this setting. Ignored with MODEL=2; use PWR_MAX instead.
    // @Range: 25 2500
    AP_GROUPINFO("MAX_POWER", 7, AP_VideoTX, _max_power_mw, 800),
    // @Param: BC1_BAND
    // @DisplayName: Band/Channel preset 1 - band
    // @Description: Video band for switch-selectable preset 1
    // @Values: 0:Band A,1:Band B,2:Band E,3:Airwave,4:RaceBand,5:Low RaceBand,6:1G3 Band A,7:1G3 Band B,8:Band X,9:3G3 Band A,10:3G3 Band B,11:Skyzone A,12:Skyzone B,13:Skyzone C,14:Skyzone D,15:Skyzone E,16:Skyzone F,17:Skyzone H,18:Skyzone J,19:MILELRS 12G A,20:MILELRS 12G B,21:MILELRS 12G C
    // @User: Standard
    AP_GROUPINFO("BC1_BAND", 8, AP_VideoTX, _bc_band[0], 4),

    // @Param: BC1_CH
    // @DisplayName: Band/Channel preset 1 - channel
    // @Description: Video channel for switch-selectable preset 1
    // @Range: 0 7
    // @User: Standard
    AP_GROUPINFO("BC1_CH",   9, AP_VideoTX, _bc_channel[0], 0),

    // @Param: BC2_BAND
    // @DisplayName: Band/Channel preset 2 - band
    // @Description: Video band for switch-selectable preset 2
    // @Values: 0:Band A,1:Band B,2:Band E,3:Airwave,4:RaceBand,5:Low RaceBand,6:1G3 Band A,7:1G3 Band B,8:Band X,9:3G3 Band A,10:3G3 Band B,11:Skyzone A,12:Skyzone B,13:Skyzone C,14:Skyzone D,15:Skyzone E,16:Skyzone F,17:Skyzone H,18:Skyzone J,19:MILELRS 12G A,20:MILELRS 12G B,21:MILELRS 12G C
    // @User: Standard
    AP_GROUPINFO("BC2_BAND", 10, AP_VideoTX, _bc_band[1], 4),

    // @Param: BC2_CH
    // @DisplayName: Band/Channel preset 2 - channel
    // @Description: Video channel for switch-selectable preset 2
    // @Range: 0 7
    // @User: Standard
    AP_GROUPINFO("BC2_CH",   11, AP_VideoTX, _bc_channel[1], 1),

    // @Param: BC3_BAND
    // @DisplayName: Band/Channel preset 3 - band
    // @Description: Video band for switch-selectable preset 3
    // @Values: 0:Band A,1:Band B,2:Band E,3:Airwave,4:RaceBand,5:Low RaceBand,6:1G3 Band A,7:1G3 Band B,8:Band X,9:3G3 Band A,10:3G3 Band B,11:Skyzone A,12:Skyzone B,13:Skyzone C,14:Skyzone D,15:Skyzone E,16:Skyzone F,17:Skyzone H,18:Skyzone J,19:MILELRS 12G A,20:MILELRS 12G B,21:MILELRS 12G C
    // @User: Standard
    AP_GROUPINFO("BC3_BAND", 12, AP_VideoTX, _bc_band[2], 4),

    // @Param: BC3_CH
    // @DisplayName: Band/Channel preset 3 - channel
    // @Description: Video channel for switch-selectable preset 3
    // @Range: 0 7
    // @User: Standard
    AP_GROUPINFO("BC3_CH",   13, AP_VideoTX, _bc_channel[2], 2),

    // @Param: BC4_BAND
    // @DisplayName: Band/Channel preset 4 - band
    // @Description: Video band for switch-selectable preset 4
    // @Values: 0:Band A,1:Band B,2:Band E,3:Airwave,4:RaceBand,5:Low RaceBand,6:1G3 Band A,7:1G3 Band B,8:Band X,9:3G3 Band A,10:3G3 Band B,11:Skyzone A,12:Skyzone B,13:Skyzone C,14:Skyzone D,15:Skyzone E,16:Skyzone F,17:Skyzone H,18:Skyzone J,19:MILELRS 12G A,20:MILELRS 12G B,21:MILELRS 12G C
    // @User: Standard
    AP_GROUPINFO("BC4_BAND", 14, AP_VideoTX, _bc_band[3], 4),

    // @Param: BC4_CH
    // @DisplayName: Band/Channel preset 4 - channel
    // @Description: Video channel for switch-selectable preset 4
    // @Range: 0 7
    // @User: Standard
    AP_GROUPINFO("BC4_CH",   15, AP_VideoTX, _bc_channel[3], 3),

    // @Param: BC5_BAND
    // @DisplayName: Band/Channel preset 5 - band
    // @Description: Video band for switch-selectable preset 5
    // @Values: 0:Band A,1:Band B,2:Band E,3:Airwave,4:RaceBand,5:Low RaceBand,6:1G3 Band A,7:1G3 Band B,8:Band X,9:3G3 Band A,10:3G3 Band B,11:Skyzone A,12:Skyzone B,13:Skyzone C,14:Skyzone D,15:Skyzone E,16:Skyzone F,17:Skyzone H,18:Skyzone J,19:MILELRS 12G A,20:MILELRS 12G B,21:MILELRS 12G C
    // @User: Standard
    AP_GROUPINFO("BC5_BAND", 16, AP_VideoTX, _bc_band[4], 4),

    // @Param: BC5_CH
    // @DisplayName: Band/Channel preset 5 - channel
    // @Description: Video channel for switch-selectable preset 5
    // @Range: 0 7
    // @User: Standard
    AP_GROUPINFO("BC5_CH",   17, AP_VideoTX, _bc_channel[4], 4),

    // @Param: BC6_BAND
    // @DisplayName: Band/Channel preset 6 - band
    // @Description: Video band for switch-selectable preset 6
    // @Values: 0:Band A,1:Band B,2:Band E,3:Airwave,4:RaceBand,5:Low RaceBand,6:1G3 Band A,7:1G3 Band B,8:Band X,9:3G3 Band A,10:3G3 Band B,11:Skyzone A,12:Skyzone B,13:Skyzone C,14:Skyzone D,15:Skyzone E,16:Skyzone F,17:Skyzone H,18:Skyzone J,19:MILELRS 12G A,20:MILELRS 12G B,21:MILELRS 12G C
    // @User: Standard
    AP_GROUPINFO("BC6_BAND", 18, AP_VideoTX, _bc_band[5], 4),

    // @Param: BC6_CH
    // @DisplayName: Band/Channel preset 6 - channel
    // @Description: Video channel for switch-selectable preset 6
    // @Range: 0 7
    // @User: Standard
    AP_GROUPINFO("BC6_CH",   19, AP_VideoTX, _bc_channel[5], 5),

    // @Param: MODEL
    // @DisplayName: VTX Model
    // @Description: Selects model-specific VTX protocol behavior
    // @Values: 0:Generic,1:D1,2:Skyzone VT7550,3:MILELRS 12GHz
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("MODEL", 20, AP_VideoTX, _model, 1),

    // @Param: PWR_LEVEL
    // @DisplayName: VT7550 power level
    // @Description: Skyzone VT7550 power selection, limited by PWR_MAX. Used only with MODEL=2; POWER and MAX_POWER do not control this model. HIG and MAX have no verified milliwatt rating.
    // @Values: 0:25mW,1:200mW,2:500mW,3:HIG,4:MAX
    // @User: Standard
    AP_GROUPINFO("PWR_LEVEL", 21, AP_VideoTX, _vt_power_level, 0),

    // @Param: PWR_MAX
    // @DisplayName: VT7550 maximum power level
    // @Description: Maximum permitted VT7550 level for direct parameter selection and RC power control. Used only with MODEL=2.
    // @Values: 0:25mW,1:200mW,2:500mW,3:HIG,4:MAX
    // @User: Standard
    AP_GROUPINFO("PWR_MAX", 22, AP_VideoTX, _vt_power_max, 4),

    AP_GROUPEND
};

//#define VTX_DEBUG
#ifdef VTX_DEBUG
# define debug(fmt, args...)	hal.console->printf("VTX: " fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL& hal;

const char * AP_VideoTX::band_names[] = {"A","B","E","F","R","L","1G3_A","1G3_B","X","3G3_A","3G3_B",
                                      "SZ_A","SZ_B","SZ_C","SZ_D","SZ_E","SZ_F","SZ_H","SZ_J",
                                      "12G_A","12G_B","12G_C"};

const uint16_t AP_VideoTX::VIDEO_CHANNELS[AP_VideoTX::MAX_BANDS][VTX_MAX_CHANNELS] =
{
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, /* Band A */
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, /* Band B */
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, /* Band E */
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, /* Airwave */
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, /* Race */
    { 5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621}, /* LO Race */
    { 1080, 1120, 1160, 1200, 1240, 1280, 1320, 1360}, /* Band 1G3_A */
    { 1080, 1120, 1160, 1200, 1258, 1280, 1320, 1360}, /* Band 1G3_B */
    { 4990, 5020, 5050, 5080, 5110, 5140, 5170, 5200}, /* Band X */
    { 3330, 3350, 3370, 3390, 3410, 3430, 3450, 3470}, /* Band 3G3_A */
    { 3170, 3190, 3210, 3230, 3250, 3270, 3290, 3310}, /* Band 3G3_B */
    { 6240, 6260, 6280, 6300, 6320, 6340, 6360, 6380}, /* Skyzone A */
    { 6400, 6420, 6440, 6460, 6480, 6500, 6520, 6540}, /* Skyzone B */
    { 6560, 6580, 6600, 6620, 6640, 6660, 6680, 6700}, /* Skyzone C */
    { 6720, 6740, 6760, 6780, 6800, 6820, 6840, 6860}, /* Skyzone D */
    { 6880, 6900, 6920, 6940, 6960, 6980, 7000, 7020}, /* Skyzone E */
    { 7040, 7060, 7080, 7100, 7120, 7140, 7160, 7180}, /* Skyzone F */
    { 7200, 7220, 7240, 7260, 7280, 7300, 7320, 7340}, /* Skyzone H */
    { 7360, 7380, 7400, 7420, 7440, 7460, 7480, 7500}, /* Skyzone J */
    { 9500, 9600, 9700, 9800, 9900, 10000, 0, 0}, /* MILELRS 12G A */
    { 10100, 10200, 10300, 10400, 10500, 10600, 10700, 10800}, /* MILELRS 12G B */
    { 10900, 11000, 11100, 11900, 0, 0, 0, 0} /* MILELRS 12G C */
};

// mapping of power level to milliwatt to dbm
// valid power levels from SmartAudio spec, the adjacent levels might be the actual values
// so these are marked as level + 0x10 and will be switched if a dbm message proves it
AP_VideoTX::PowerLevel AP_VideoTX::_power_levels[VTX_MAX_POWER_LEVELS] = {
    // level, mw, dbm, dac
    { 0xFF,  0,    0, 0    }, // only in SA 2.1
    { 0,    25,   14, 7    },
    { 0x11, 100,  20, 0xFF }, // only in SA 2.1
    { 1,    200,  23, 16   },
    { 0x12, 400,  26, 0xFF }, // only in SA 2.1
    { 2,    500,  27, 25   },
    { 0x12, 600,  28, 0xFF }, // Tramp lies above power levels and always returns 25/100/200/400/600
    { 3,    800,  29, 40   },
    { 0x13, 1000, 30, 0xFF }, // only in SA 2.1
    { 0x23, 2500, 34, 0xFF }, // 2.5W level, only in SA 2.1
    { 0xFF, 0,    0,  0XFF, PowerActive::Inactive }  // slot reserved for a custom power level
};

AP_VideoTX::AP_VideoTX()
{
    if (singleton) {
        AP_HAL::panic("Too many VTXs");
        return;
    }
    singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    switch (model()) {
    case Model::D1: {
        for (uint8_t i = VTX_MAX_POWER_LEVELS - 1; i > 0; --i) {
            if (_power_levels[i].active != PowerActive::Inactive) {
                if (_power_levels[i].mw > D1_MAX_POWER_MW) {
                    _power_levels[i].active = PowerActive::Inactive;
                } else {
                    break;
                }
            }
        }
        break;
    }
    default:
        break;
    }
}

AP_VideoTX::~AP_VideoTX(void)
{
    singleton = nullptr;
}

bool AP_VideoTX::init(void)
{
    if (_initialized) {
        return false;
    }

    // PARAMETER_CONVERSION - Added: Sept-2022
    _options.convert_parameter_width(AP_PARAM_INT16);

    _active_model = static_cast<Model>(static_cast<uint8_t>(_model));
    if (is_vt7550()) {
        // Reported state must come from Tramp, never from configured values.
        _current_frequency = 0;
        _current_band = MAX_BANDS;
        _current_channel = VTX_MAX_CHANNELS;
        _current_enabled = _enabled;
        _initialized = true;
        return true;
    }

    // find the index into the power table
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (_power_mw <= _power_levels[i].mw) {
            if (_power_mw != _power_levels[i].mw) {
                if (i > 0) {
                    _current_power = i - 1;
                }
                _power_mw.set_and_save(get_power_mw());
            } else {
                _current_power = i;
            }
            break;
        }
    }
    _current_band = _band;
    _current_channel = _channel;
    _current_frequency = _frequency_mhz;
    _current_options = _options;
    _current_enabled = _enabled;
    _initialized = true;

    return true;
}

bool AP_VideoTX::valid_frequency_for_model(uint16_t mhz) const
{
    if (is_milelrs_12ghz()) {
        VideoBand band;
        uint8_t channel;
        return get_band_and_channel(mhz, band, channel) && band >= MILELRS_12G_A;
    }
    return true;
}

bool AP_VideoTX::get_band_and_channel(uint16_t freq, VideoBand& band, uint8_t& channel)
{
    if (freq == 0) {
        return false;
    }
    for (uint8_t i = 0; i < AP_VideoTX::MAX_BANDS; i++) {
        for (uint8_t j = 0; j < VTX_MAX_CHANNELS; j++) {
            if (VIDEO_CHANNELS[i][j] == freq) {
                band = VideoBand(i);
                channel = j;
                return true;
            }
        }
    }
    return false;
}

// set the current power
void AP_VideoTX::set_configured_power_mw(uint16_t power)
{
    _power_mw.set_and_save_ifchanged(power);
}

uint8_t AP_VideoTX::find_current_power() const
{
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (_power_mw == _power_levels[i].mw) {
            return i;
        }
    }
    return 0;
}

// set the power in dbm, rounding appropriately
void AP_VideoTX::set_power_dbm(uint8_t power, PowerActive active)
{
    if (power == _power_levels[_current_power].dbm
        && _power_levels[_current_power].active == active) {
        return;
    }

    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (power == _power_levels[i].dbm) {
            _current_power = i;
            _power_levels[i].active = active;
            debug("learned power %ddbm", power);
            // now unlearn the "other" power level since we have no other way of guessing
            // the supported levels
            if ((_power_levels[i].level & 0xF0) == 0x10) {
                _power_levels[i].level = _power_levels[i].level & 0xF;
            }
            if (i > 0 && _power_levels[i-1].level == _power_levels[i].level) {
                debug("invalidated power %dwm, level %d is now %dmw", _power_levels[i-1].mw, _power_levels[i].level, _power_levels[i].mw);
                _power_levels[i-1].level = 0xFF;
                _power_levels[i-1].active = PowerActive::Inactive;
            } else if (i < VTX_MAX_POWER_LEVELS-1 && _power_levels[i+1].level == _power_levels[i].level) {
                debug("invalidated power %dwm, level %d is now %dmw", _power_levels[i+1].mw, _power_levels[i].level, _power_levels[i].mw);
                _power_levels[i+1].level = 0xFF;
                _power_levels[i+1].active = PowerActive::Inactive;
            }
            return;
        }
    }
    // learn the non-standard power
    _current_power = update_power_dbm(power, active);
}

// add an active power setting in dbm
uint8_t AP_VideoTX::update_power_dbm(uint8_t power, PowerActive active)
{
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (power == _power_levels[i].dbm) {
            if (_power_levels[i].active != active) {
                _power_levels[i].active = active;
                debug("%s power %ddbm", active == PowerActive::Active ? "learned" : "invalidated", power);
            }
            return i;
        }
    }
    // handed a non-standard value, use the last slot
    _power_levels[VTX_MAX_POWER_LEVELS-1].dbm = power;
    _power_levels[VTX_MAX_POWER_LEVELS-1].level = 255;
    _power_levels[VTX_MAX_POWER_LEVELS-1].dac = 255;
    _power_levels[VTX_MAX_POWER_LEVELS-1].mw = uint16_t(roundf(powf(10, power / 10.0f)));
    _power_levels[VTX_MAX_POWER_LEVELS-1].active = active;
    debug("non-standard power %ddbm -> %dmw", power, _power_levels[VTX_MAX_POWER_LEVELS-1].mw);
    return VTX_MAX_POWER_LEVELS-1;
}

// add all active power setting in dbm
void AP_VideoTX::update_all_power_dbm(uint8_t nlevels, const uint8_t power[])
{
    for (uint8_t i = 0; i < nlevels; i++) {
        update_power_dbm(power[i], PowerActive::Active);
    }
    // invalidate the remaining ones
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (_power_levels[i].active == PowerActive::Unknown) {
            _power_levels[i].active = PowerActive::Inactive;
        }
    }
}

// set the power in mw
void AP_VideoTX::set_power_mw(uint16_t power)
{
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (power == _power_levels[i].mw) {
            _current_power = i;
            break;
        }
    }
}

// set the power "level"
void AP_VideoTX::set_power_level(uint8_t level, PowerActive active)
{
    if (level == _power_levels[_current_power].level
        && _power_levels[_current_power].active == active) {
        return;
    }

    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (level == _power_levels[i].level) {
            _current_power = i;
            _power_levels[i].active = active;
            debug("learned power level %d: %dmw", level, get_power_mw());
            break;
        }
    }
}

// set the power dac
void AP_VideoTX::set_power_dac(uint16_t power, PowerActive active)
{
    if (power == _power_levels[_current_power].dac
        && _power_levels[_current_power].active == active) {
        return;
    }

    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (power == _power_levels[i].dac) {
            _current_power = i;
            _power_levels[i].active = active;
            debug("learned power %dmw", get_power_mw());
        }
    }
}

// set the current channel
void AP_VideoTX::set_enabled(bool enabled)
{
    _current_enabled = enabled;
    if (!_enabled.configured()) {
        _enabled.set_and_save(enabled);
    }
}

void AP_VideoTX::set_power_is_current()
{
    set_power_dbm(get_configured_power_dbm());
}

void AP_VideoTX::set_freq_is_current()
{
    _current_frequency = _frequency_mhz;
    _current_band = _band;
    _current_channel = _channel;
}

// periodic update
void AP_VideoTX::update(void)
{
    if (!_enabled) {
        return;
    }

    // manipulate pitmode if pitmode-on-disarm or power-on-arm is set
    if (has_option(VideoOptions::VTX_PITMODE_ON_DISARM) || has_option(VideoOptions::VTX_PITMODE_UNTIL_ARM)) {
        if (hal.util->get_soft_armed() && has_option(VideoOptions::VTX_PITMODE)) {
            _options.set(_options & ~uint8_t(VideoOptions::VTX_PITMODE));
        } else if (!hal.util->get_soft_armed() && !has_option(VideoOptions::VTX_PITMODE)
            && has_option(VideoOptions::VTX_PITMODE_ON_DISARM)) {
            _options.set(_options | uint8_t(VideoOptions::VTX_PITMODE));
        }
    }
    // check that the requested power is actually allowed
    // reset if not
    if (!is_vt7550() && _power_mw != get_power_mw()) {
        if (_power_levels[find_current_power()].active == PowerActive::Inactive) {
            // reset to something we know works
            debug("power reset to %dmw from %dmw", get_power_mw(), _power_mw.get());
            _power_mw.set_and_save(get_power_mw());
        }
    }
}

bool AP_VideoTX::update_options() const
{
    if (!_defaults_set) {
        return false;
    }
    // check pitmode
    if ((_options & uint8_t(VideoOptions::VTX_PITMODE))
        != (_current_options & uint8_t(VideoOptions::VTX_PITMODE))) {
        return true;
    }

#if HAL_CRSF_TELEM_ENABLED
    // using CRSF so unlock is not an option
    if (AP::crsf_telem() != nullptr) {
        return false;
    }
#endif
    // check unlock only
    if (is_vt7550()) {
        return false;
    }
    if ((_options & uint8_t(VideoOptions::VTX_UNLOCKED)) != 0
        && (_current_options & uint8_t(VideoOptions::VTX_UNLOCKED)) == 0) {
        return true;
    }

    // ignore everything else
    return false;
}

bool AP_VideoTX::update_power() const {
    if (is_vt7550()) {
        return _defaults_set && VT7550::valid_level(get_vt7550_power_level()) &&
               get_vt7550_power_level() != _vt_current_level && !get_pitmode();
    }
    if (!_defaults_set || _power_mw == get_power_mw() || get_pitmode()) {
        return false;
    }
    // check that the requested power is actually allowed
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (_power_mw == _power_levels[i].mw
            && _power_levels[i].active != PowerActive::Inactive) {
            return true;
        }
    }
    // asked for something unsupported - only SA2.1 allows this and will have already provided a list
    return false;
}

bool AP_VideoTX::have_params_changed() const
{
    return _enabled
        && (update_power()
        || update_band()
        || update_channel()
        || update_frequency()
        || update_options());
}

// update the configured frequency to match the channel and band
void AP_VideoTX::update_configured_frequency()
{
    const int16_t band = _band.get();
    const int16_t channel = _channel.get();
    if (!is_valid_band_channel(band, channel)) {
        debug("invalid configured band/channel %d/%d", band, channel);
        return;
    }
    _frequency_mhz.set_and_save(get_frequency_mhz(band, channel));
}

// update the configured channel and band to match the frequency
void AP_VideoTX::update_configured_channel_and_band()
{
    VideoBand band;
    uint8_t channel;
    if (get_band_and_channel(_frequency_mhz, band, channel)) {
        _band.set_and_save(band);
        _channel.set_and_save(channel);
    } else {
        update_configured_frequency();
    }
}

// set the current configured values if not currently set in storage
// this is necessary so that the current settings can be seen
bool AP_VideoTX::set_defaults()
{
    if (_defaults_set) {
        return false;
    }

    if (is_vt7550()) {
        // Explicit setup selects the target frequency; do not invent readback
        // or fall back to an unrelated 5.8 GHz band on an unknown response.
        _defaults_set = true;
        return true;
    }

    // check that our current view of frequency matches band/channel
    // if not then force one to be correct
    uint16_t calced_freq = get_frequency_mhz(_current_band, _current_channel);
    if (_current_frequency != calced_freq) {
        if (_current_frequency > 0) {
            VideoBand band;
            uint8_t channel;
            if (get_band_and_channel(_current_frequency, band, channel)) {
                _current_band = band;
                _current_channel = channel;
            } else {
                _current_frequency = calced_freq;
            }
        } else {
            _current_frequency = calced_freq;
        }
    }

    if (!_options.configured()) {
        _options.set_and_save(_current_options);
    }
    if (!_channel.configured()) {
        _channel.set_and_save(_current_channel);
    }
    if (!_band.configured()) {
        _band.set_and_save(_current_band);
    }
    if (!_power_mw.configured()) {
        _power_mw.set_and_save(get_power_mw());
    }
    if (!_frequency_mhz.configured()) {
        _frequency_mhz.set_and_save(_current_frequency);
    }

    // Now check that the user didn't screw up by selecting incompatible options
    if (_frequency_mhz != get_frequency_mhz(_band, _channel)) {
        if (_frequency_mhz > 0) {
            update_configured_channel_and_band();
        } else {
            update_configured_frequency();
        }
    }

    _defaults_set = true;

    announce_vtx_settings();

    return true;
}

void AP_VideoTX::announce_vtx_settings() const
{
    if (is_vt7550()) {
        if (vt7550_configuration_matches()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VTX: VT7550 %uMHz PWR:%s", unsigned(_current_frequency),
                          get_pitmode() ? "PIT" : get_vt7550_power_label());
        }
        return;
    }
    const int16_t band = _band.get();
    const int16_t channel = _channel.get();
    if (!is_valid_band_channel(band, channel)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTX: invalid band/channel %d/%d", band, channel);
        return;
    }
    // Output a friendly message so the user knows the VTX has been detected
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VTX: %s%d %dMHz, PWR: %dmW",
        band_names[band], channel + 1, _frequency_mhz.get(),
        has_option(VideoOptions::VTX_PITMODE) ? 0 : _power_mw.get());
}

// change the video power based on switch input
// 6-pos range is in the middle of the available range
void AP_VideoTX::change_power(int8_t position)
{
    if (!_enabled || position < 0 || position > 5) {
        return;
    }
    if (is_vt7550()) {
        const int8_t level = VT7550::switch_level(position, _vt_power_max);
        if (VT7550::valid_level(level)) {
            _vt_power_level.set_and_notify(level);
            _options.set_and_notify(_options & ~uint8_t(VideoOptions::VTX_PITMODE));
            _configuration_finished = false;
        }
        return;
    }
    // first find out how many possible levels there are
    uint16_t max_power_mw = _max_power_mw > 0 ? static_cast<uint16_t>(_max_power_mw.get()) : 0;
    if (model() == Model::D1) {
        max_power_mw = MIN(max_power_mw, D1_MAX_POWER_MW);
    }
    uint8_t num_active_levels = 0;
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (_power_levels[i].active != PowerActive::Inactive && _power_levels[i].mw <= max_power_mw) {
            num_active_levels++;
        }
    }
    if (num_active_levels == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTX: no power level <= %umW", unsigned(max_power_mw));
        return;
    }
    // iterate through to find the level
    uint16_t level = constrain_int16(roundf((num_active_levels * (position + 1)/ 6.0f) - 1), 0, num_active_levels - 1);
    debug("looking for pos %d power level %d from %d", position, level, num_active_levels);
    uint16_t power = 0;
    for (uint8_t i = 0, j = 0; i < num_active_levels; i++, j++) {
        while (j < VTX_MAX_POWER_LEVELS-1 && _power_levels[j].active == PowerActive::Inactive) {
            j++;
        }
        if (i == level) {
            power = _power_levels[j].mw;
            debug("selected power %dmw", power);
            break;
        }
    }

    if (power == 0) {
        if (!hal.util->get_soft_armed()) {    // don't allow pitmode to be entered if already armed
            set_configured_options(get_configured_options() | uint8_t(VideoOptions::VTX_PITMODE));
        }
    } else {
        if (has_option(VideoOptions::VTX_PITMODE)) {
            set_configured_options(get_configured_options() & ~uint8_t(VideoOptions::VTX_PITMODE));
        }
        set_configured_power_mw(power);
    }
}

// change the band/channel preset based on switch input
// 6-pos range maps directly to the 6 stored presets
void AP_VideoTX::change_band_channel(int8_t position)
{
    if (!_enabled) {
        return;
    }
    if (position < 0 || position > 5) {
        debug("invalid preset position %d", position);
        return;
    }
    const int16_t band = _bc_band[position].get();
    const int16_t channel = _bc_channel[position].get();
    if (!is_valid_band_channel(band, channel)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTX: preset %d invalid (%d/%d)", position + 1, band, channel);
        return;
    }
    if (is_vt7550() && !VT7550::valid_frequency(get_frequency_mhz(band, channel))) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTX: preset %d outside VT7550 table", position + 1);
        return;
    }

    if (is_milelrs_12ghz() && !valid_frequency_for_model(get_frequency_mhz(band, channel))) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTX: preset %d outside 12GHz table", position + 1);
        return;
    }

    // RC selection is intentionally volatile: do not wear parameter storage.
    _band.set_and_notify(band);
    _channel.set_and_notify(channel);
    _frequency_mhz.set_and_notify(get_frequency_mhz(band, channel));
    _configuration_finished = false;
}

int8_t AP_VideoTX::get_vt7550_power_level() const
{
    if (!VT7550::valid_level(_vt_power_level) || !VT7550::valid_level(_vt_power_max)) {
        return -1;
    }
    return MIN(_vt_power_level.get(), _vt_power_max.get());
}

void AP_VideoTX::set_vt7550_power_code(uint16_t code)
{
    _vt_current_level = VT7550::power_level(code);
}

const char *AP_VideoTX::get_vt7550_power_label() const
{
    return VT7550::power_label(_vt_current_level);
}

bool AP_VideoTX::vt7550_request_valid() const
{
    return VT7550::valid_frequency(_frequency_mhz) &&
           get_frequency_mhz(_band, _channel) == _frequency_mhz &&
           VT7550::valid_level(get_vt7550_power_level());
}

bool AP_VideoTX::vt7550_configuration_matches() const
{
    return _defaults_set && vt7550_request_valid() && _current_frequency == _frequency_mhz &&
           _vt_current_level == get_vt7550_power_level() && get_pitmode() == get_configured_pitmode();
}

namespace AP {
    AP_VideoTX& vtx() {
        return *AP_VideoTX::get_singleton();
    }
};

#endif
