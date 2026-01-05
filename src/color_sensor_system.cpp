/**
 * color_sensor_system.cpp - Color-based sorting behavior for sbot.
 */

#include "color_sensor_system.h"

SbotColorSensorSystem::SbotColorSensorSystem()
    : color_sensor(SBOT_COLOR_SENSOR_PORT),
      alliance_color(AllianceColor::UNKNOWN),
      sorting_enabled(false),
      eject_end_time_ms(0) {
    color_sensor.set_led_pwm(100);
}

void SbotColorSensorSystem::setAllianceColor(AllianceColor color) {
    alliance_color = color;
}

void SbotColorSensorSystem::setSortingEnabled(bool enabled) {
    sorting_enabled = enabled;
    if (!enabled) {
        eject_end_time_ms = 0;
    }
}

void SbotColorSensorSystem::update(SbotIndexer& indexer) {
    std::uint32_t now = pros::millis();

    // If we are in the middle of a timed eject, keep reversing indexer
    if (eject_end_time_ms != 0 && now < eject_end_time_ms) {
        indexer.setMode(IndexerMode::FEED_BACKWARD_EJECT);
        indexer.update();
        return;
    }

    // No active eject
    eject_end_time_ms = 0;

    if (!sorting_enabled) return;
    if (alliance_color == AllianceColor::UNKNOWN) return;

    // Use basic hue detection; exact thresholds can be tuned later
    double hue = color_sensor.get_hue();
    bool sees_red = (hue > 330 || hue < 30);
    bool sees_blue = (hue > 180 && hue < 260);

    bool bad_ball = false;
    if (alliance_color == AllianceColor::RED && sees_blue) bad_ball = true;
    if (alliance_color == AllianceColor::BLUE && sees_red) bad_ball = true;

    if (bad_ball && indexer.getMode() == IndexerMode::FEED_FORWARD) {
        indexer.setMode(IndexerMode::FEED_BACKWARD_EJECT);
        eject_end_time_ms = now + SBOT_COLOR_EJECT_TIME_MS;
    }
}
