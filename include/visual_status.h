/**
 * \file visual_status.h
 *
 * Enhanced visual status display system for quick driver recognition.
 * Provides graphical and symbolic representations of robot state.
 */

#ifndef _VISUAL_STATUS_H_
#define _VISUAL_STATUS_H_

#include "api.h"
#include "pto.h"
#include "indexer.h"

/**
 * Visual Status Display class
 * 
 * Provides enhanced visual feedback to the driver using:
 * - Unicode symbols for instant recognition
 * - Color-coded patterns (via positioning)
 * - Consistent visual language across all states
 * - Haptic feedback integration
 */
class VisualStatusDisplay {
private:
    char display_lines[3][17];           ///< Formatted display lines
    uint32_t last_update;                ///< Last display update time
    bool force_update;                   ///< Force immediate update
    
    // Display refresh rate
    static const uint32_t REFRESH_MS = 100;
    
    // Visual symbols for different states
    static const char* DRIVE_SYMBOLS[];
    static const char* MODE_SYMBOLS[];
    static const char* STATUS_SYMBOLS[];

public:
    /**
     * Constructor
     */
    VisualStatusDisplay();
    
    /**
     * Update the visual display with current robot status
     * @param controller Controller to display on
     * @param pto PTO system for drive mode
     * @param indexer Indexer system for scoring mode
     */
    void updateDisplay(pros::Controller& controller, 
                      const PTO* pto, 
                      const IndexerSystem* indexer);
    
    /**
     * Force immediate display update
     */
    void forceUpdate();
    
    /**
     * Clear the display
     * @param controller Controller to clear
     */
    void clearDisplay(pros::Controller& controller);
    
    /**
     * Provide haptic feedback for state changes
     * @param controller Controller to rumble
     * @param mode New scoring mode
     */
    static void provideModeChangeFeedback(pros::Controller& controller, ScoringMode mode);
    
    /**
     * Provide haptic feedback for PTO changes
     * @param controller Controller to rumble
     * @param drive_mode True if switching to drive mode
     */
    static void providePTOChangeFeedback(pros::Controller& controller, bool drive_mode);
    
    /**
     * Provide haptic feedback for activation
     * @param controller Controller to rumble
     */
    static void provideActivationFeedback(pros::Controller& controller);
    
private:
    /**
     * Format drive mode display
     * @param pto PTO system
     * @return Formatted string for drive mode
     */
    const char* formatDriveMode(const PTO* pto);
    
    /**
     * Format scoring mode display
     * @param indexer Indexer system
     * @return Formatted string for scoring mode
     */
    const char* formatScoringMode(const IndexerSystem* indexer);
    
    /**
     * Format status display
     * @param indexer Indexer system
     * @return Formatted string for current status
     */
    const char* formatStatus(const IndexerSystem* indexer);
};

#endif // _VISUAL_STATUS_H_