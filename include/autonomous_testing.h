/**
 * \file autonomous_testing.h
 * 
 * Testing and analysis framework for autonomous routes.
 * Provides timing analysis, success rate tracking, and route optimization tools.
 */

#ifndef _AUTONOMOUS_TESTING_H_
#define _AUTONOMOUS_TESTING_H_

#include "api.h"
#include <vector>
#include <string>

/**
 * Route performance metrics
 */
struct RouteMetrics {
    std::string route_name;
    uint32_t execution_time_ms;
    bool completed_successfully;
    int estimated_points;
    bool awp_completed;
    std::vector<std::string> failure_points;
    
    RouteMetrics(const std::string& name) 
        : route_name(name), execution_time_ms(0), completed_successfully(false), 
          estimated_points(0), awp_completed(false) {}
};

/**
 * Testing framework for autonomous routes
 */
class AutonomousTester {
private:
    std::vector<RouteMetrics> test_results;
    uint32_t current_test_start_time;
    RouteMetrics* current_test;
    
public:
    AutonomousTester();
    
    /**
     * Start timing a new test
     */
    void startTest(const std::string& route_name);
    
    /**
     * Mark a checkpoint in the current test
     */
    void checkpoint(const std::string& checkpoint_name);
    
    /**
     * Mark test as failed with reason
     */
    void markFailure(const std::string& failure_reason);
    
    /**
     * Complete the current test
     */
    void completeTest(int points_scored, bool awp_status);
    
    /**
     * Print comprehensive test results
     */
    void printResults();
    
    /**
     * Print comparison of all routes
     */
    void printRouteComparison();
    
    /**
     * Get success rate for a specific route
     */
    double getSuccessRate(const std::string& route_name);
    
    /**
     * Get average execution time for a route
     */
    double getAverageTime(const std::string& route_name);
    
    /**
     * Clear all test data
     */
    void clearResults();
};

/**
 * Global tester instance
 */
extern AutonomousTester autonomous_tester;

/**
 * Convenience macros for testing
 */
#define START_AUTO_TEST(name) autonomous_tester.startTest(name)
#define AUTO_CHECKPOINT(desc) autonomous_tester.checkpoint(desc)
#define AUTO_FAILURE(reason) autonomous_tester.markFailure(reason)
#define COMPLETE_AUTO_TEST(pts, awp) autonomous_tester.completeTest(pts, awp)

#endif // _AUTONOMOUS_TESTING_H_
