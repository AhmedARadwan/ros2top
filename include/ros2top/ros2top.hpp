#ifndef ROS2TOP_HPP
#define ROS2TOP_HPP

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <filesystem>
#include <ctime>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <cerrno>
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
            // O_EXCL, rather than exists() followed by a write: the check and
            // the create have to be one step, or two nodes starting together
            // can both believe they hold the lock.
            int fd = ::open(lock_file_.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
            if (fd >= 0) {
                std::string owner = std::to_string(getpid()) + "\n";
                ssize_t written = ::write(fd, owner.data(), owner.size());
                (void)written;
                ::close(fd);
                locked_ = true;
                return true;
            }

            if (errno == EEXIST && steal_if_stale()) {
                continue;   // the holder is dead; retry at once
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

private:
    /**
     * @brief Drop a lock file whose owning process is gone
     *
     * The lock is a plain file, so a node killed while holding it never cleans
     * up and every later registration -- from any process, in either language
     * -- fails forever after. A lock owned by a dead PID protects nothing.
     *
     * @return true if a stale lock was removed, i.e. it is worth retrying
     */
    bool steal_if_stale() {
        std::ifstream lock(lock_file_);
        if (!lock.is_open()) {
            return false;
        }

        pid_t owner = 0;
        if (!(lock >> owner) || owner <= 0) {
            // Mid-write by a live process, or truncated. Waiting is correct;
            // a genuinely broken file is caught by the caller's timeout.
            return false;
        }
        lock.close();

        if (owner == getpid()) {
            return false;       // our own lock, held further up the stack
        }
        if (::kill(owner, 0) == 0 || errno == EPERM) {
            return false;       // still alive (EPERM: alive, another user)
        }

        std::error_code ignored;
        return std::filesystem::remove(lock_file_, ignored);
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
    
    /**
     * @brief argv of this process, as separate strings
     *
     * /proc/self/cmdline separates arguments with NUL, not newline, and has no
     * trailing newline at all. Reading it with std::getline therefore returns
     * the *whole* command line as one string with NULs embedded in it, which is
     * why this used to report a process_name like "launch_params_mu_ch5t_\0":
     * find_last_of('/') was landing in the last argument (a --params-file path)
     * rather than in argv[0]. Split on NUL instead.
     */
    static std::vector<std::string> read_cmdline() {
        std::vector<std::string> argv;
        std::ifstream cmdline("/proc/self/cmdline", std::ios::binary);
        if (!cmdline.is_open()) {
            return argv;
        }
        std::string raw((std::istreambuf_iterator<char>(cmdline)),
                        std::istreambuf_iterator<char>());
        size_t start = 0;
        while (start < raw.size()) {
            size_t end = raw.find('\0', start);
            if (end == std::string::npos) {
                end = raw.size();
            }
            if (end > start) {
                argv.push_back(raw.substr(start, end - start));
            }
            start = end + 1;
        }
        return argv;
    }

    static std::string get_process_name(const std::vector<std::string>& argv) {
        // argv[0] is the executable. /proc/self/comm would also do, but it is
        // truncated to 15 characters, which mangles the long executable names
        // ROS 2 packages tend to have.
        if (!argv.empty() && !argv[0].empty()) {
            const std::string& exe = argv[0];
            size_t last_slash = exe.find_last_of('/');
            return last_slash == std::string::npos ? exe
                                                   : exe.substr(last_slash + 1);
        }

        std::ifstream comm("/proc/self/comm");
        if (comm.is_open()) {
            std::string name;
            if (std::getline(comm, name) && !name.empty()) {
                return name;
            }
        }
        return "unknown";
    }

public:
    /**
     * @brief Register a ROS2 node with ros2top monitoring
     * @param node_name Name of the ROS2 node
     * @param additional_info Optional metadata about the node. Any JSON object
     *        is accepted, so values may be lists or numbers as well as strings
     *        - matching the Python API, whose additional_info is a plain dict.
     *
     *        This used to be a std::map<std::string, std::string>. Passing the
     *        JSON object that the example and the docs both showed therefore
     *        threw type_error.302 ("type must be string, but is array") on the
     *        implicit conversion, the registration was abandoned, and the node
     *        never appeared as registered. A std::map still converts
     *        implicitly, so existing callers are unaffected.
     * @return true if registration was successful, false otherwise
     */
    static bool register_node(const std::string& node_name, 
                             const json& additional_info = json::object()) {
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
            
            // Create node registration data (matching Python format)
            const std::vector<std::string> argv = read_cmdline();
            json node_data = {
                {"node_name", normalized_name},
                {"pid", getpid()},
                {"ppid", getppid()},
                {"process_name", get_process_name(argv)},
                {"cmdline", argv},
                {"registration_time", std::time(nullptr)},
                {"last_seen", std::time(nullptr)}
            };
            
            // Add additional info if provided. Anything that is not a JSON
            // object is ignored rather than thrown over: losing optional
            // metadata must never cost the registration itself.
            json info = additional_info.is_object() ? additional_info : json::object();
            if (!info.contains("language")) {
                info["language"] = "cpp";   // the Python API records this too
            }
            node_data["additional_info"] = info;
            
            // Key on PID + name, matching the Python format exactly. Node name
            // alone is not unique: a component container hosts several nodes in
            // one process, and they would overwrite each other.
            registry[std::to_string(getpid()) + ":" + normalized_name] = node_data;
            
            return write_registry_json(registry);
            
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
     * @return true if the node is no longer registered, false if the registry
     *         could not be updated. Unregistering a node that was never
     *         registered succeeds: the requested state already holds.
     */
    static bool unregister_node(const std::string& node_name) {
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
            
            // Normalize node name
            std::string normalized_name = normalize_node_name(node_name);
            
            // Remove the node if it exists
            std::string key = std::to_string(getpid()) + ":" + normalized_name;
            if (registry.contains(key)) {
                registry.erase(key);
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
     * @param node_name Name of the ROS2 node
     * @return true if the heartbeat was recorded. false means this node is not
     *         in the registry, or the registry could not be written - a missed
     *         heartbeat is not fatal, but it is worth logging.
     */
    static bool heartbeat(const std::string& node_name) {
        try {
            ensure_registry_dir();
            
            // Acquire file lock
            FileLock lock(get_lock_file());
            if (!lock.try_lock()) {
                return false; // A missed heartbeat is not critical
            }
            
            // Read existing registry
            json registry = read_registry_json();
            
            // Normalize node name
            std::string normalized_name = normalize_node_name(node_name);
            
            // Update heartbeat if node exists
            std::string key = std::to_string(getpid()) + ":" + normalized_name;
            if (registry.contains(key)) {
                registry[key]["last_seen"] = std::time(nullptr);
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
                         const json& additional_info = json::object()) {
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
                     const json& additional_info = json::object())
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
