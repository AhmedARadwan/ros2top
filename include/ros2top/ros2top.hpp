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

namespace ros2top {

/**
 * @brief JSON utility class for simple JSON operations without external dependencies
 */
class SimpleJSON {
private:
    std::map<std::string, std::string> data_;
    
public:
    void set(const std::string& key, const std::string& value) {
        data_[key] = value;
    }
    
    void set(const std::string& key, int value) {
        data_[key] = std::to_string(value);
    }
    
    void set(const std::string& key, long value) {
        data_[key] = std::to_string(value);
    }
    
    std::string serialize() const {
        std::string json = "{\n";
        bool first = true;
        for (const auto& [key, value] : data_) {
            if (!first) json += ",\n";
            json += "  \"" + key + "\": ";
            
            // Check if value is numeric
            bool is_numeric = !value.empty() && 
                             (std::isdigit(value[0]) || 
                              (value[0] == '-' && value.size() > 1 && std::isdigit(value[1])));
            
            if (is_numeric) {
                json += value;
            } else {
                json += "\"" + value + "\"";
            }
            first = false;
        }
        json += "\n}";
        return json;
    }
};

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
    
    static std::string read_registry_content() {
        std::string registry_file = get_registry_file();
        if (!std::filesystem::exists(registry_file)) {
            return "{}";
        }
        
        std::ifstream file(registry_file);
        if (!file.is_open()) {
            return "{}";
        }
        
        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
        return content.empty() ? "{}" : content;
    }
    
    static bool write_registry_content(const std::string& content) {
        std::string registry_file = get_registry_file();
        std::ofstream file(registry_file);
        if (!file.is_open()) {
            return false;
        }
        
        file << content;
        return file.good();
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
        try {
            ensure_registry_dir();
            
            // Acquire file lock
            FileLock lock(get_lock_file());
            if (!lock.try_lock()) {
                std::cerr << "ros2top: Failed to acquire registry lock for registration" << std::endl;
                return false;
            }
            
            // Create node registration data
            SimpleJSON node_data;
            node_data.set("node_name", normalize_node_name(node_name));
            node_data.set("pid", static_cast<int>(getpid()));
            node_data.set("timestamp", static_cast<long>(std::time(nullptr)));
            node_data.set("language", "cpp");
            node_data.set("process_name", get_process_name());
            
            // Add additional info
            for (const auto& [key, value] : additional_info) {
                node_data.set(key, value);
            }
            
            // Read existing registry (simplified - in real implementation would parse JSON properly)
            std::string registry_content = read_registry_content();
            
            // For simplicity, we'll append the new node data
            // In a full implementation, we'd properly parse and merge JSON
            std::string new_entry = "  \"" + normalize_node_name(node_name) + "_" + 
                                   std::to_string(getpid()) + "\": " + node_data.serialize();
            
            // Simple JSON merging (this is a simplified approach)
            if (registry_content == "{}") {
                registry_content = "{\n" + new_entry + "\n}";
            } else {
                // Insert before the last }
                size_t last_brace = registry_content.find_last_of('}');
                if (last_brace != std::string::npos) {
                    registry_content.insert(last_brace, ",\n" + new_entry + "\n");
                }
            }
            
            return write_registry_content(registry_content);
            
        } catch (const std::exception& e) {
            std::cerr << "ros2top: Registration failed: " << e.what() << std::endl;
            return false;
        } catch (...) {
            std::cerr << "ros2top: Registration failed with unknown error" << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Unregister a ROS2 node from ros2top monitoring
     * @param node_name Name of the ROS2 node to unregister
     */
    static void unregister_node(const std::string& node_name) {
        try {
            ensure_registry_dir();
            
            // Acquire file lock
            FileLock lock(get_lock_file());
            if (!lock.try_lock()) {
                std::cerr << "ros2top: Failed to acquire registry lock for unregistration" << std::endl;
                return;
            }
            
            // In a full implementation, we'd properly parse JSON and remove the specific entry
            // For now, we'll just touch the file to indicate activity
            std::string registry_file = get_registry_file();
            if (std::filesystem::exists(registry_file)) {
                std::filesystem::last_write_time(registry_file, std::filesystem::file_time_type::clock::now());
            }
            
        } catch (const std::exception& e) {
            std::cerr << "ros2top: Unregistration failed: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "ros2top: Unregistration failed with unknown error" << std::endl;
        }
    }
    
    /**
     * @brief Send heartbeat to indicate the node is still alive
     * @param node_name Name of the ROS2 node
     */
    static void heartbeat(const std::string& node_name) {
        try {
            ensure_registry_dir();
            
            // Simple heartbeat - update timestamp
            // In a full implementation, this would update the specific node's timestamp
            std::string registry_file = get_registry_file();
            if (std::filesystem::exists(registry_file)) {
                std::filesystem::last_write_time(registry_file, std::filesystem::file_time_type::clock::now());
            }
            
        } catch (...) {
            // Ignore heartbeat errors - not critical
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

inline void unregister_node(const std::string& node_name) {
    NodeRegistrar::unregister_node(node_name);
}

inline void heartbeat(const std::string& node_name) {
    NodeRegistrar::heartbeat(node_name);
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
