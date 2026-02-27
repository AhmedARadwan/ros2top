#ifndef ROS2TOP_HPP
#define ROS2TOP_HPP

#include <string>
#include <map>
#include <fstream>
#include <filesystem>
#include <ctime>
#include <unistd.h>
#include <sys/types.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>

namespace ros2top {

using json = nlohmann::json;

/**
 * @brief File locking utility for safe concurrent access to registry
 */
class FileLock {
private:
    std::string lock_file_;
    bool locked_;
    
public:
    FileLock(const std::string& lock_file) : lock_file_(lock_file), locked_(false) {}
    
    ~FileLock() {
        if (locked_) {
            unlock();
        }
    }
    
    bool try_lock(int timeout_ms = 1000) {
        int attempts = timeout_ms / 10;
        for (int i = 0; i < attempts; ++i) {
            if (!std::filesystem::exists(lock_file_)) {
                std::ofstream lock(lock_file_);
                if (lock.is_open()) {
                    lock << getpid() << std::endl;
                    lock.close();
                    locked_ = true;
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }
    
    void unlock() {
        if (locked_) {
            std::filesystem::remove(lock_file_);
            locked_ = false;
        }
    }
};

/**
 * @brief Main node registration class
 */
class NodeRegistrar {
private:
    static std::string get_registry_path() {
        const char* home = getenv("HOME");
        if (!home) return "/tmp/.ros2top/registry";
        return std::string(home) + "/.ros2top/registry";
    }
    
    static std::string get_registry_file() {
        return get_registry_path() + "/nodes.json";
    }
    
    static std::string get_lock_file() {
        return get_registry_path() + "/nodes.lock";
    }
    
    static void ensure_registry_dir() {
        std::filesystem::create_directories(get_registry_path());
    }
    
    static std::string normalize_node_name(const std::string& node_name) {
        return (node_name.length() > 0 && node_name[0] == '/') ? node_name : "/" + node_name;
    }
    
    static json read_registry_json() {
        std::string registry_file = get_registry_file();
        if (!std::filesystem::exists(registry_file)) {
            return json::object();
        }
        
        std::ifstream file(registry_file);
        if (!file.is_open()) {
            return json::object();
        }
        
        try {
            json j;
            file >> j;
            return j;
        } catch (const json::exception&) {
            // If JSON is corrupted, return empty object
            return json::object();
        }
    }
    
    static bool write_registry_json(const json& j) {
        std::string registry_file = get_registry_file();
        std::ofstream file(registry_file);
        if (!file.is_open()) {
            return false;
        }
        
        try {
            file << j.dump(2) << std::endl;
            return file.good();
        } catch (const json::exception&) {
            return false;
        }
    }
    
    static std::string get_process_name() {
        try {
            std::ifstream cmdline("/proc/self/cmdline");
            if (cmdline.is_open()) {
                std::string line;
                std::getline(cmdline, line);
                if (!line.empty()) {
                    // Extract just the executable name
                    size_t last_slash = line.find_last_of('/');
                    if (last_slash != std::string::npos) {
                        return line.substr(last_slash + 1);
                    }
                    return line;
                }
            }
        } catch (...) {
            // Fall back to unknown if we can't read process info
        }
        return "unknown";
    }

public:
    /**
     * @brief Register a ROS2 node with ros2top monitoring
     * @param node_name Name of the ROS2 node
     * @param additional_info Optional additional information about the node
     * @return true if registration was successful, false otherwise
     */
    static bool register_node(const std::string& node_name, 
                             const std::map<std::string, std::string>& additional_info = {}) {
        json additional_json;
        for (const auto& [key, value] : additional_info) {
            additional_json[key] = value;
        }
        return register_node_impl(node_name, additional_json);
    }

    /**
     * @brief Register a ROS2 node with ros2top monitoring (JSON overload)
     * @param node_name Name of the ROS2 node
     * @param additional_info Optional additional information as JSON
     * @return true if registration was successful, false otherwise
     */
    static bool register_node(const std::string& node_name, 
                             const json& additional_info) {
        return register_node_impl(node_name, additional_info);
    }

private:
    static bool register_node_impl(const std::string& node_name, 
                                   const json& additional_info) {
        try {
            ensure_registry_dir();
            
            // Acquire file lock
            FileLock lock(get_lock_file());
            if (!lock.try_lock()) {
                std::cerr << "ros2top: Failed to acquire registry lock for registration" << std::endl;
                return false;
            }
            
            // Read existing registry
            json registry = read_registry_json();
            
            // Normalize node name to match Python format
            std::string normalized_name = normalize_node_name(node_name);
            
            // Get PID as string key (matching Python format)
            std::string pid_key = std::to_string(getpid());
            
            // Create node registration data (matching Python format)
            json node_data = {
                {"node_name", normalized_name},
                {"pid", getpid()},
                {"ppid", getppid()},
                {"process_name", get_process_name()},
                {"cmdline", json::array()}, // Simplified for C++
                {"registration_time", std::time(nullptr)},
                {"last_seen", std::time(nullptr)}
            };
            
            // Add additional info if provided
            if (!additional_info.is_null() && !additional_info.empty()) {
                node_data["additional_info"] = additional_info;
            }
            
            // Use PID as key (matching Python format)
            registry[pid_key] = node_data;
            
            return write_registry_json(registry);
            
        } catch (const std::exception& e) {
            std::cerr << "ros2top: Registration failed: " << e.what() << std::endl;
            return false;
        } catch (...) {
            std::cerr << "ros2top: Registration failed with unknown error" << std::endl;
            return false;
        }
    }

public:
    
    /**
     * @brief Unregister a ROS2 node from ros2top monitoring
     * @param node_name Name of the ROS2 node to unregister (unused, kept for API compat)
     * @return true if unregistration was successful, false otherwise
     */
    static bool unregister_node(const std::string& node_name) {
        (void)node_name; // Lookup is by PID, matching Python behaviour
        try {
            ensure_registry_dir();
            
            // Acquire file lock
            FileLock lock(get_lock_file());
            if (!lock.try_lock()) {
                std::cerr << "ros2top: Failed to acquire registry lock for unregistration" << std::endl;
                return false;
            }
            
            // Read existing registry
            json registry = read_registry_json();
            
            // Use PID as key (matching Python format)
            std::string pid_key = std::to_string(getpid());
            
            // Remove the node if it exists
            if (registry.contains(pid_key)) {
                registry.erase(pid_key);
                return write_registry_json(registry);
            }
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "ros2top: Unregistration failed: " << e.what() << std::endl;
            return false;
        } catch (...) {
            std::cerr << "ros2top: Unregistration failed with unknown error" << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Send heartbeat to indicate the node is still alive
     * @param node_name Name of the ROS2 node (unused, kept for API compat)
     * @return true if heartbeat was successful, false otherwise
     */
    static bool heartbeat(const std::string& node_name) {
        (void)node_name; // Lookup is by PID, matching Python behaviour
        try {
            ensure_registry_dir();
            
            // Acquire file lock
            FileLock lock(get_lock_file());
            if (!lock.try_lock()) {
                return false;
            }
            
            // Read existing registry
            json registry = read_registry_json();
            
            // Use PID as key (matching Python format)
            std::string pid_key = std::to_string(getpid());
            
            // Update heartbeat if node exists
            if (registry.contains(pid_key)) {
                registry[pid_key]["last_seen"] = std::time(nullptr);
                return write_registry_json(registry);
            }
            
            return false;
        } catch (...) {
            // Ignore heartbeat errors - not critical
            return false;
        }
    }
};

/**
 * @brief Convenience functions for easier usage
 */
inline bool register_node(const std::string& node_name, 
                         const std::map<std::string, std::string>& additional_info = {}) {
    return NodeRegistrar::register_node(node_name, additional_info);
}

inline bool register_node(const std::string& node_name, 
                         const nlohmann::json& additional_info) {
    return NodeRegistrar::register_node(node_name, additional_info);
}

inline bool unregister_node(const std::string& node_name) {
    return NodeRegistrar::unregister_node(node_name);
}

inline bool heartbeat(const std::string& node_name) {
    return NodeRegistrar::heartbeat(node_name);
}

/**
 * @brief RAII wrapper for automatic node registration/unregistration
 */
class AutoNodeRegistrar {
private:
    std::string node_name_;
    
public:
    AutoNodeRegistrar(const std::string& node_name, 
                     const std::map<std::string, std::string>& additional_info = {})
        : node_name_(node_name) {
        register_node(node_name_, additional_info);
    }
    
    ~AutoNodeRegistrar() {
        unregister_node(node_name_);
    }
    
    // Delete copy constructor and assignment operator
    AutoNodeRegistrar(const AutoNodeRegistrar&) = delete;
    AutoNodeRegistrar& operator=(const AutoNodeRegistrar&) = delete;
};

} // namespace ros2top

#endif // ROS2TOP_HPP
