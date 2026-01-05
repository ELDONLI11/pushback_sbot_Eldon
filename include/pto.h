/**
 * \file pto.h
 *
 * PTO (Power Take-Off) system header file.
 * Manages pneumatic cylinders that switch between drivetrain and scorer modes.
 */

#ifndef _PTO_H_
#define _PTO_H_

#include "api.h"
#include "config.h"

/**
 * PTO (Power Take-Off) class
 * 
 * This class manages the pneumatic cylinders that control whether the middle wheels
 * are connected to the drivetrain (3-wheel drive) or to the scorer mechanism (2-wheel drive).
 * 
 * States:
 * - EXTENDED (drivetrain mode): Middle wheels help with driving - 3-wheel drive per side
 * - RETRACTED (scorer mode): Middle wheels power the scorer - 2-wheel drive per side
 */
class PTO {
private:
    pros::adi::DigitalOut left_pneumatic;   ///< Left side PTO pneumatic cylinder
    pros::adi::DigitalOut right_pneumatic;  ///< Right side PTO pneumatic cylinder
    bool current_state;                   ///< Current PTO state (true = extended/drive, false = retracted/scorer)
    bool last_button_state;              ///< Last state of toggle button (for edge detection)

public:
    /**
     * Constructor - initializes PTO pneumatics
     */
    PTO();

    /**
     * Set PTO to drivetrain mode (extended)
     * Middle wheels are connected to drivetrain for 3-wheel drive
     */
    void setDrivetrainMode();

    /**
     * Set PTO to scorer mode (retracted)  
     * Middle wheels are connected to scorer mechanism for 2-wheel drive
     */
    void setScorerMode();

    /**
     * Toggle between drivetrain and scorer modes
     */
    void toggle();

    /**
     * Get current PTO state
     * @return true if in drivetrain mode, false if in scorer mode
     */
    bool isDrivetrainMode() const;

    /**
     * Get current PTO state
     * @return true if in scorer mode, false if in drivetrain mode  
     */
    bool isScorerMode() const;

    /**
     * Update PTO system - call this in opcontrol loop
     * Handles button press detection for toggling
     * @param controller Reference to the master controller
     */
    void update(pros::Controller& controller);

    /**
     * Get string representation of current mode for debugging
     * @return "Drivetrain" or "Scorer" depending on current mode
     */
    const char* getCurrentModeString() const;
};

#endif // _PTO_H_