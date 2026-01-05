/**
 * \file color_sensor.h
 *
 * Color sensing and sorting system for detecting and sorting colored balls.
 * Uses two optical sensors in sequence to detect ball color and direction,
 * then automatically drops undesired balls from the back mid out position.
 */

#ifndef _COLOR_SENSOR_H_
#define _COLOR_SENSOR_H_

#include "api.h"
#include "config.h"

// Forward declaration
class IndexerSystem;

// =============================================================================
// COLOR SENSOR CONFIGURATION
// =============================================================================

// Color sensor ports (V5 smart ports)
#define COLOR_SENSOR_1_PORT     19  // Lower color sensor (entry detection)
#define COLOR_SENSOR_2_PORT     20  // Upper color sensor (confirmation/direction)

// Ball color detection thresholds and values
#define RED_HUE_MIN             0     // Red hue range minimum
#define RED_HUE_MAX             30    // Red hue range maximum
#define RED_HUE_HIGH_MIN        330   // Red hue upper range minimum (wraps around)
#define RED_HUE_HIGH_MAX        360   // Red hue upper range maximum

#define BLUE_HUE_MIN            200   // Blue hue range minimum  
#define BLUE_HUE_MAX            250   // Blue hue range maximum

#define MIN_SATURATION          0     // Minimum saturation for valid color detection (DISABLED - sensor not seeing color)
#define MIN_BRIGHTNESS          0     // Minimum brightness for valid detection (DISABLED - sensor LEDs appear off)
#define MIN_PROXIMITY_THRESHOLD 100   // Minimum proximity value indicating ball presence (sensors return HIGH when object is CLOSE)

// Detection timing and confirmation settings
#define COLOR_DETECTION_DELAY_MS    50    // Delay between sensor readings (ms)
#define COLOR_CONFIRMATION_COUNT    5     // Number of consistent readings required (INCREASED for more reliable detection)
#define BALL_PASSAGE_TIMEOUT_MS     2000  // Max time for ball to pass between sensors
#define BALL_DIRECTION_TIMEOUT_MS   1000  // Max time between sensor triggers for direction

// Ball ejection settings
#define BALL_EJECT_DELAY_MS         200   // Delay before ejecting ball (ms)
#define BALL_EJECT_DURATION_MS      500   // Duration to run ejection mechanism (ms) - TUNE THIS VALUE
#define BALL_EJECT_MIN_DURATION     300   // Minimum safe ejection duration (ms)
#define BALL_EJECT_MAX_DURATION     800   // Maximum ejection duration before timeout (ms)

// =============================================================================
// ENUMERATIONS
// =============================================================================

/**
 * Detected ball colors
 */
enum class BallColor {
    UNKNOWN = 0,
    RED = 1,
    BLUE = 2,
    NO_BALL = 3
};

/**
 * Ball movement direction through the system
 */
enum class BallDirection {
    UNKNOWN = 0,
    FORWARD = 1,    // Sensor 1 -> Sensor 2 (normal intake direction)
    REVERSE = 2,    // Sensor 2 -> Sensor 1 (ball moving backwards)
    STATIONARY = 3  // Ball detected but not moving
};

/**
 * Color sorting mode - which color to keep vs eject
 */
enum class SortingMode {
    COLLECT_RED = 0,    // Keep red balls, eject blue balls
    COLLECT_BLUE = 1,   // Keep blue balls, eject red balls
    COLLECT_ALL = 2,    // Keep all balls (sorting disabled)
    EJECT_ALL = 3       // Eject all balls (defensive mode)
};

// =============================================================================
// COLOR SENSOR SYSTEM CLASS
// =============================================================================

/**
 * Color sensing and sorting system class.
 * Manages two optical sensors to detect ball color and direction,
 * then automatically ejects undesired balls using the existing indexer system.
 */
class ColorSensorSystem {
private:
    // Hardware components
    pros::Optical* sensor1;          // First color sensor (entry)
    pros::Optical* sensor2;          // Second color sensor (confirmation)
    
    // Reference to indexer system for ball ejection
    class IndexerSystem* indexer_system; // Forward declaration, will be set during initialization
    
    // Comprehensive state preservation for seamless auto-resume
    struct SavedState {
        bool was_scoring_active;          // Was indexer actively scoring?
        bool was_input_active;            // Was input motor running?
        int saved_scoring_mode;           // Saved ScoringMode (as int to avoid enum issues)
        int saved_execution_direction;    // Saved ExecutionDirection (as int)
        bool valid;                       // Is this saved state valid?
    } saved_indexer_state;
    
    // System state
    SortingMode current_mode;        // Current sorting mode
    BallColor last_detected_color;   // Last detected ball color
    BallColor sensor1_confirmed_color; // Confirmed color from sensor 1
    BallColor sensor2_confirmed_color; // Confirmed color from sensor 2
    BallDirection last_direction;    // Last detected ball direction
    bool sensor1_triggered;          // Is sensor 1 currently detecting a ball
    bool sensor2_triggered;          // Is sensor 2 currently detecting a ball
    uint32_t sensor1_trigger_time;   // Time when sensor 1 was triggered
    uint32_t sensor2_trigger_time;   // Time when sensor 2 was triggered
    bool ejection_active;            // Is ball ejection currently active
    uint32_t ejection_start_time;    // Time when ejection started
    uint32_t ejection_duration;     // Configurable ejection duration (ms)
    
    // Detection state tracking
    BallColor sensor1_color_buffer[COLOR_CONFIRMATION_COUNT];  // Color reading buffer for sensor 1
    BallColor sensor2_color_buffer[COLOR_CONFIRMATION_COUNT];  // Color reading buffer for sensor 2
    int sensor1_buffer_index;        // Current buffer index for sensor 1
    int sensor2_buffer_index;        // Current buffer index for sensor 2
    
    // Statistics and debugging
    int red_balls_detected;          // Count of red balls detected
    int blue_balls_detected;         // Count of blue balls detected
    int balls_ejected;               // Count of balls ejected
    int false_detections;            // Count of false/invalid detections

public:
    /**
     * Constructor - initializes the color sensor system
     */
    ColorSensorSystem();
    
    /**
     * Destructor - cleans up allocated resources
     */
    ~ColorSensorSystem();
    
    /**
     * Initialize the color sensor system
     * Sets up sensors, calibrates, and prepares for operation
     * @param indexer_ref Pointer to the indexer system for ball ejection
     * @return true if initialization successful, false otherwise
     */
    bool initialize(class IndexerSystem* indexer_ref);
    
    /**
     * Main update function - call this continuously in a loop
     * Handles color detection, direction tracking, and ball ejection
     */
    void update();
    
    /**
     * Set the sorting mode (which color to keep vs eject)
     * @param mode The desired sorting mode
     */
    void setSortingMode(SortingMode mode);
    
    /**
     * Get the current sorting mode
     * @return Current sorting mode
     */
    SortingMode getSortingMode() const;
    
    /**
     * Get the last detected ball color
     * @return Last confirmed ball color
     */
    BallColor getLastDetectedColor() const;
    
    /**
     * Get the last detected ball direction
     * @return Last detected ball movement direction
     */
    BallDirection getLastDirection() const;
    
    /**
     * Check if a ball is currently being detected by either sensor
     * @return true if ball detected, false otherwise
     */
    bool isBallDetected() const;
    
    /**
     * Manually trigger ball ejection (for testing or emergency)
     */
    void triggerEjection();
    
    /**
     * Get detection statistics
     * @param red_count Reference to store red ball count
     * @param blue_count Reference to store blue ball count
     * @param ejected_count Reference to store ejected ball count
     * @param false_count Reference to store false detection count
     */
    void getStatistics(int& red_count, int& blue_count, int& ejected_count, int& false_count) const;
    
    /**
     * Reset detection statistics
     */
    void resetStatistics();
    
    /**
     * Manually reset all detection state (public interface)
     * Useful for recovery after jams or manual interventions
     */
    void resetDetectionState();
    
    /**
     * Print current status to console (for debugging)
     */
    void printStatus() const;
    
    /**
     * Set custom ejection duration for tuning
     * @param duration_ms Ejection duration in milliseconds (300-800ms recommended)
     */
    void setEjectionDuration(uint32_t duration_ms);
    
    /**
     * Get current ejection duration setting
     * @return Current ejection duration in milliseconds
     */
    uint32_t getEjectionDuration() const;
    
    /**
     * Test function to verify sensor functionality
     * @return true if both sensors are working properly
     */
    bool testSensors();

private:
    /**
     * Read color from a specific sensor
     * @param sensor Pointer to the optical sensor
     * @return Detected ball color
     */
    BallColor readColorFromSensor(pros::Optical* sensor);
    
    /**
     * Check if a ball is present based on proximity reading
     * @param sensor Pointer to the optical sensor
     * @return true if ball is present, false otherwise
     */
    bool isBallPresent(pros::Optical* sensor);
    
    /**
     * Add a color reading to the confirmation buffer
     * @param sensor_num Sensor number (1 or 2)
     * @param color Detected color to add to buffer
     * @return Confirmed color if buffer is consistent, UNKNOWN otherwise
     */
    BallColor addToColorBuffer(int sensor_num, BallColor color);
    
    /**
     * Determine ball direction based on sensor trigger timing
     * @return Detected ball direction
     */
    BallDirection determineBallDirection();
    
    /**
     * Check if ball should be ejected based on color and current mode
     * @param color Ball color to check
     * @return true if ball should be ejected, false if should be kept
     */
    bool shouldEjectBall(BallColor color);
    
    /**
     * Start ball ejection sequence using indexer system
     * Temporarily runs mid back scoring to eject ball from back mid out
     */
    void startEjection();
    
    /**
     * Stop ball ejection sequence and return indexer to previous state
     */
    void stopEjection();
    
    /**
     * Save current indexer system state before ejection
     */
    void saveIndexerState();
    
    /**
     * Restore previous indexer system state after ejection
     */
    void restoreIndexerState();
    
    /**
     * Convert BallColor enum to string for printing
     * @param color Ball color enum value
     * @return String representation of the color
     */
    const char* colorToString(BallColor color) const;
    
    /**
     * Convert BallDirection enum to string for printing
     * @param direction Ball direction enum value
     * @return String representation of the direction
     */
    const char* directionToString(BallDirection direction) const;
    
    /**
     * Convert SortingMode enum to string for printing
     * @param mode Sorting mode enum value
     * @return String representation of the mode
     */
    const char* sortingModeToString(SortingMode mode) const;
};

// =============================================================================
// GLOBAL INSTANCE DECLARATION
// =============================================================================

// Global color sensor system instance (to be initialized in main.cpp)
extern ColorSensorSystem* color_sensor_system;

#endif // _COLOR_SENSOR_H_