/**
 * color_sensor_system.h - Alliance color & sorting logic for sbot.
 */

#ifndef _SBOT_COLOR_SENSOR_SYSTEM_H_
#define _SBOT_COLOR_SENSOR_SYSTEM_H_

#include <cstdint>
#include "api.h"
#include "config_sbot.h"
#include "indexer.h"

class SbotColorSensorSystem {
public:
    SbotColorSensorSystem();

    void setAllianceColor(AllianceColor color);
    AllianceColor getAllianceColor() const { return alliance_color; }

    void setSortingEnabled(bool enabled);
    bool isSortingEnabled() const { return sorting_enabled; }

    // Call periodically from opcontrol; may override indexer mode for ejection
    void update(SbotIndexer& indexer);

private:
    pros::Optical color_sensor;
    AllianceColor alliance_color;
    bool sorting_enabled;
    std::uint32_t eject_end_time_ms; // 0 when no timed eject in progress
};

#endif // _SBOT_COLOR_SENSOR_SYSTEM_H_
