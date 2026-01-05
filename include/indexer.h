/**
 * indexer.h - Indexer motor control for sbot.
 */

#ifndef _SBOT_INDEXER_H_
#define _SBOT_INDEXER_H_

#include "api.h"
#include "config_sbot.h"

enum class IndexerMode {
    OFF = 0,
    FEED_FORWARD,          // Toward top goal / storage
    FEED_BACKWARD_MIDDLE,  // Middle goal scoring
    FEED_BACKWARD_EJECT    // Color rejection / eject
};

class SbotIndexer {
public:
    SbotIndexer();

    void setMode(IndexerMode mode);
    IndexerMode getMode() const { return mode; }

    void update();

private:
    pros::Motor indexer_motor;
    IndexerMode mode;
};

#endif // _SBOT_INDEXER_H_
