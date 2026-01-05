/**
 * \file autonomous.h
 *
 * Autonomous system header file for VEX Push Back 2024-25.
 * Implements odometry-based navigation with AWP-focused routines.
 */

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "config.h"
#include "pto.h"
#include "indexer.h"
#include "lemlib/api.hpp"
#include <cmath>

/**
 * Position structure for robot coordinates
 */
struct Position {
    double x;      // X coordinate in inches
    double y;      // Y coordinate in inches  
    double heading; // Heading in degrees (0 = facing positive X)
    
    Position(double x = 0, double y = 0, double heading = 0) 
        : x(x), y(y), heading(heading) {}
};

/**
 * PID Controller class for movement control
 */
class PIDController {
private:
    double kp, ki, kd;
    double previous_error;
    double integral;
    uint32_t last_time;
    
public:
    PIDController(double kp, double ki, double kd);
    double calculate(double error);
    void reset();
};

/**
 * Autonomous Selector class for LCD-based mode selection
 */
class AutoSelector {
private:
    AutoMode selected_mode;
    int selector_position;
    bool mode_confirmed;
    
public:
    AutoSelector();
    void displayOptions();
    void handleInput();
    AutoMode getSelectedMode();
    bool isModeConfirmed();
    bool update();  // Now returns true when mode is confirmed
};

/**
 * Main Autonomous System class
 * Thin wrapper over LemLib for autonomous routines
 */
class AutonomousSystem {
private:
    // System references (no hardware - LemLib handles that)
    PTO* pto_system;
    IndexerSystem* indexer_system;
    AutoSelector auto_selector;
    
    // State tracking
    bool autonomous_running;
    
    // Standard scoring sequences
    void executePostScoringBump(double bump_distance = 8.0, double retreat_distance = 10.0);
    
public:
    /**
     * Constructor - thin wrapper initialization
     */
    AutonomousSystem(PTO* pto, IndexerSystem* indexer);
    
    /**
     * Initialize autonomous system - call during initialize()
     */
    void initialize();
    
    /**
     * Update selector - call continuously in disabled/competition_initialize
     */
    void update();
    
    /**
     * LemLib wrapper functions (all movement goes through LemLib)
     */
    void driveToPoint(double x, double y, double timeout_ms = 5000);
    void turnToHeading(double heading, double timeout_ms = 3000);
    void driveDistance(double distance, double heading = 0, double timeout_ms = 5000);
    
    /**
     * Position functions (LemLib wrappers)
     */
    void setPosition(double x, double y, double heading);
    lemlib::Pose getPosition();
    void printPosition();
    
    /**
     * Autonomous route functions
     */
    // Bonus Point Routes (Primary Strategy)
    void executeRedLeftBonus();
    void executeRedRightBonus();
    void executeBlueLeftBonus();
    void executeBlueRightBonus();
    
    // AWP Routes (Backup Strategy)
    void executeRedLeftAWP();
    void executeRedRightAWP();
    void executeBlueLeftAWP();
    void executeBlueRightAWP();
    
    void executeSkillsRoutine();
    
    /**
     * Main autonomous entry point - call from autonomous()
     */
    void runAutonomous();
    
    /**
     * Utility functions
     */
    void stopAllMovement();
    
    /**
     * Get the autonomous selector for external access
     */
    AutoSelector& getSelector();
};

// External test function (defined in system_tests.cpp)
// All individual test functions are internal to system_tests.cpp
void runTest(AutoMode mode, lemlib::Chassis* chassis);

#endif // _AUTONOMOUS_H_