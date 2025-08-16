#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/duration.h>
#include <franka/model.h>

// Structure to define a dance move (a joint configuration)
struct DanceMove {
    int move_index;                // Index of the move (1, 2, 3, ...)
    std::array<double, 7> joints;  // Joint configuration for this move
    double move_time;              // Time to take for moving to this position (in seconds)
};

// Function to read dance moves from a configuration file
std::vector<DanceMove> readDanceMovesFromConfig(const std::string& config_file_path) {
    std::vector<DanceMove> dance_moves;
    std::ifstream config_file(config_file_path);
    
    if (!config_file.is_open()) {
        throw std::runtime_error("Failed to open configuration file: " + config_file_path);
    }
    
    std::string line;
    while (std::getline(config_file, line)) {
        // Skip empty lines and comments (lines starting with #)
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::istringstream iss(line);
        DanceMove move;
        
        // Read move index
        if (!(iss >> move.move_index)) {
            std::cerr << "Error parsing move index in line: " << line << std::endl;
            continue;
        }
        
        // Read joint values
        for (size_t i = 0; i < 7; ++i) {
            if (!(iss >> move.joints[i])) {
                std::cerr << "Error parsing joint " << i << " in line: " << line << std::endl;
                continue;
            }
        }
        
        // Read move time
        if (!(iss >> move.move_time)) {
            std::cerr << "Error parsing move time in line: " << line << std::endl;
            continue;
        }
        
        dance_moves.push_back(move);
        std::cout << "Loaded move " << move.move_index << " with move time " << move.move_time << "s" << std::endl;
    }
    
    if (dance_moves.empty()) {
        throw std::runtime_error("No valid dance moves found in configuration file");
    }
    
    return dance_moves;
}

// Helper function for quintic (5th order) path interpolation
double quinticPath(double t, double T) {
    if (t <= 0) return 0.0;
    if (t >= T) return 1.0;
    
    double normalized_t = t / T;
    return 10 * std::pow(normalized_t, 3) - 15 * std::pow(normalized_t, 4) + 6 * std::pow(normalized_t, 5);
}

// Function to recover the robot if an error occurs
void recoverRobot(franka::Robot& robot) {
    std::cout << "Attempting to recover robot from error state..." << std::endl;
    
    try {
        robot.automaticErrorRecovery();
        std::cout << "Robot recovery successful!" << std::endl;
    } catch (const franka::Exception& e) {
        std::cerr << "Error during recovery: " << e.what() << std::endl;
        std::cout << "Waiting 5 seconds before continuing..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

// Check if the desired movement time is safe with respect to the robot's joint velocity limits.
double getSafeMovementTime(const std::array<double, 7>& q_start, 
                           const std::array<double, 7>& q_end,
                           double desired_time) {
    // Configurable max joint velocity (rad/s) - increased from 1.5 for better performance
    const double MAX_JOINT_VELOCITY = 2.0;
    
    double max_delta = 0.0;
    int critical_joint = -1;
    
    for (size_t i = 0; i < 7; i++) {
        double delta = std::abs(q_end[i] - q_start[i]);
        if (delta > max_delta) {
            max_delta = delta;
            critical_joint = i;
        }
    }
    
    double min_safe_time = max_delta / MAX_JOINT_VELOCITY;
    
    if (desired_time >= min_safe_time) {
        return desired_time;
    } else {
        std::cout << "WARNING: Requested time (" << desired_time << "s) is too fast!" << std::endl;
        std::cout << "Joint " << critical_joint+1 << " would need to move at " 
                  << (max_delta / desired_time) << " rad/s (limit: " << MAX_JOINT_VELOCITY << " rad/s)" << std::endl;
        std::cout << "Automatically increasing time to " << min_safe_time << "s for safety\n";
        return min_safe_time;
    }
}

// Moves the robot's joints to a target configuration over the desired duration.
// A quintic polynomial is used to interpolate between the current and target joint positions.
double moveJoints(franka::Robot& robot, const std::array<double, 7>& q_target, double desired_duration, bool recover_on_error = true) {
    try {
        // Read current joint positions
        franka::RobotState state = robot.readOnce();
        std::array<double, 7> q_current = state.q;
        
        // Calculate a safe duration based on the joint velocities
        double safe_duration = getSafeMovementTime(q_current, q_target, desired_duration);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        double time_total = 0.0;
        
        // Control loop: generates a smooth trajectory using quintic interpolation
        robot.control([=, &q_current, &q_target, &time_total](const franka::RobotState& state, 
                                                               franka::Duration period) -> franka::JointPositions {
            double time_passed = period.toSec();
            time_total += time_passed;
            
            double factor = quinticPath(time_total, safe_duration);
            std::array<double, 7> q_desired{};
            for (size_t i = 0; i < 7; i++) {
                q_desired[i] = q_current[i] + factor * (q_target[i] - q_current[i]);
            }
            
            if (time_total >= safe_duration * 1.01) {  // Allow slight overshoot for smooth stop
                return franka::MotionFinished(franka::JointPositions(q_desired));
            }
            
            return franka::JointPositions(q_desired);
        });
        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        double actual_duration = elapsed.count();
        std::cout << "Move completed! Desired: " << desired_duration 
                  << "s, Actual: " << actual_duration << "s\n";
        std::cout.flush();  // Explicit flush only when needed
        
        // Reduced settling time from 300ms to 100ms for faster movements
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        return actual_duration;
    } catch (const franka::Exception& e) {
        std::cerr << "Franka exception during joint motion: " << e.what() << std::endl;
        if (recover_on_error) {
            std::cout << "Attempting to recover and retry..." << std::endl;
            recoverRobot(robot);
            return moveJoints(robot, q_target, desired_duration, false);
        }
        return -1.0;
    }
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname> <config-file-path>" << std::endl;
        return 1;
    }
    
    try {
        // Connect to the robot.
        std::cout << "Connecting to robot at " << argv[1] << "..." << std::endl;
        franka::Robot robot(argv[1]);
        
        // Load the robot model.
        std::cout << "Loading robot model..." << std::endl;
        franka::Model model = robot.loadModel();
        
        // Set the default collision behavior (using conservative limits).
        std::cout << "Setting collision behavior..." << std::endl;
        robot.setCollisionBehavior(
            std::array<double, 7>{{40.0, 40.0, 38.0, 38.0, 36.0, 34.0, 32.0}},
            std::array<double, 7>{{45.0, 45.0, 43.0, 43.0, 41.0, 39.0, 37.0}},
            std::array<double, 6>{{40.0, 40.0, 38.0, 38.0, 36.0, 34.0}},
            std::array<double, 6>{{45.0, 45.0, 43.0, 43.0, 41.0, 39.0}}
        );
        
        // Read dance moves from configuration file
        std::string config_file_path = argv[2];
        std::cout << "Reading dance moves from configuration file: " << config_file_path << std::endl;
        std::vector<DanceMove> dance_moves = readDanceMovesFromConfig(config_file_path);
        
        std::cout << "Dance sequence starting..." << std::endl;
        std::cout << "----------------------------" << std::endl;
        std::cout << "| From | To | Desired | Actual |" << std::endl;
        std::cout << "----------------------------" << std::endl;
        
        // Move to the first dance pose as the starting position.
        std::cout << "Moving to initial dance pose (Move " << dance_moves[0].move_index << ")..." << std::endl;
        double initial_move_time = moveJoints(robot, dance_moves[0].joints, dance_moves[0].move_time);  // Use time from config
        if (initial_move_time < 0) {
            std::cerr << "Failed to move to initial pose. Exiting." << std::endl;
            return 1;
        }
        
        bool repeat = true;
        // Repeat the dance cycle until the user decides to stop.
        while (repeat) {
            for (size_t i = 0; i < dance_moves.size(); i++) {
                // Calculate the next index (wraps back to the first pose at the end).
                size_t next_index = (i + 1) % dance_moves.size();
                int from_move = dance_moves[i].move_index;
                int to_move = dance_moves[next_index].move_index;
                double desired_time = dance_moves[next_index].move_time;
                
                std::cout << "Moving from pose " << from_move << " to pose " << to_move 
                          << " (Target: " << desired_time << "s)..." << std::endl;
                double actual_time = moveJoints(robot, dance_moves[next_index].joints, desired_time);
                std::cout << "| " << from_move << " | " << to_move 
                          << " | " << desired_time << "s | " 
                          << (actual_time >= 0 ? std::to_string(actual_time) + "s" : "FAILED") 
                          << " |" << std::endl;
                
                if (actual_time < 0) {
                    recoverRobot(robot);
                }
            }
            std::cout << "\nCompleted one full dance cycle. Continue? (y/n): ";
            char response;
            std::cin >> response;
            if(response != 'y' && response != 'Y') {
                repeat = false;
            }
        }
        
        std::cout << "Dance sequence completed!" << std::endl;
        
    } catch (const franka::Exception& e) {
        std::cerr << "Franka exception: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
