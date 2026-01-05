/**
 * indexer.cpp - Indexer implementation for sbot.
 */

#include "indexer.h"

SbotIndexer::SbotIndexer()
    : indexer_motor((SBOT_INDEXER_MOTOR_REVERSED ? -SBOT_INDEXER_MOTOR_PORT : SBOT_INDEXER_MOTOR_PORT),
              pros::v5::MotorGears::green),
            mode(IndexerMode::OFF) {}

void SbotIndexer::setMode(IndexerMode newMode) {
    mode = newMode;
}

void SbotIndexer::update() {
    int speed = 0;

    switch (mode) {
        case IndexerMode::FEED_FORWARD:
            speed = SBOT_INDEXER_FORWARD_FEED;
            break;
        case IndexerMode::FEED_BACKWARD_MIDDLE:
        case IndexerMode::FEED_BACKWARD_EJECT:
            speed = SBOT_INDEXER_REVERSE_MIDDLE;
            break;
        case IndexerMode::OFF:
        default:
            speed = 0;
            break;
    }

    indexer_motor.move_velocity(speed);
}
