// Description: Embedded Temperature Sensor Monitoring System with CAN Bus Communication
// Author: Senior Software Engineer
// Date: 2024-01-15
// Version: 1.0.0
// 
// This system implements a comprehensive temperature monitoring solution for embedded
// automotive applications with real-time data processing, CAN bus communication,
// and intelligent alert management following industry best practices.

#ifndef EMBEDDED_TEMPERATURE_MONITOR_H
#define EMBEDDED_TEMPERATURE_MONITOR_H

#include <iostream>
#include <vector>
#include <deque>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <cstdint>
#include <limits>
#include <mutex>
#include <condition_variable>
#include <cmath>        // for std::sin, std::sqrt, std::isnan, std::isinf
#include <numeric>      // for std::accumulate


// ============================================================================
// CONSTANTS AND CONFIGURATION
// ============================================================================

namespace EmbeddedTempMonitor {
    // System configuration constants
    constexpr float DEFAULT_ALERT_THRESHOLD = 80.0f;
    constexpr float MIN_VALID_TEMPERATURE = -50.0f;
    constexpr float MAX_VALID_TEMPERATURE = 150.0f;
    constexpr uint32_t DEFAULT_CAN_ID = 0x123;
    constexpr uint32_t DEFAULT_BAUD_RATE = 500000;
    constexpr size_t DEFAULT_BUFFER_SIZE = 10;
    constexpr int SENSOR_ERROR_VALUE = -999;
    
    // Performance optimization constants
    constexpr std::chrono::milliseconds DEFAULT_READ_INTERVAL{1000};
    constexpr std::chrono::minutes DEFAULT_COOLDOWN_PERIOD{5};
}

// ============================================================================
// ALERT SYSTEM ENUMERATIONS AND STRUCTURES
// ============================================================================

namespace EmbeddedTempMonitor {
    
    /**
     * @brief Alert severity levels for temperature monitoring
     */
    enum class AlertSeverity : uint8_t {
        LOW = 0,
        MEDIUM = 1,
        HIGH = 2,
        CRITICAL = 3
    };
    
    /**
     * @brief Alert data structure with comprehensive information
     */
    struct Alert {
        uint32_t id;
        std::string message;
        float temperature;
        AlertSeverity severity;
        std::chrono::system_clock::time_point timestamp;
        bool isAcknowledged;
        
        Alert() : id(0), temperature(0.0f), severity(AlertSeverity::LOW), 
                 isAcknowledged(false) {}
    };
    
    /**
     * @brief CAN message structure for embedded communication
     */
    struct CANMessage {
        uint32_t canId;
        uint8_t dataLength;
        uint8_t data[8];
        uint32_t timestamp;
        bool isExtended;
        
        CANMessage() : canId(0), dataLength(0), timestamp(0), isExtended(false) {
            std::fill(std::begin(data), std::end(data), 0);
        }
    };
}

// ============================================================================
// SENSOR MANAGER CLASS - Hardware Abstraction Layer
// ============================================================================

namespace EmbeddedTempMonitor {
    
    /**
     * @brief Manages temperature sensor operations with hardware abstraction
     * 
     * Implements RAII principles and provides thread-safe sensor operations
     * with comprehensive error handling and calibration support.
     */
    class SensorManager {
    private:
        const int sensorId_;
        const std::string sensorType_;
        std::atomic<bool> isActive_;
        std::atomic<float> calibrationFactor_;
        std::chrono::system_clock::time_point lastReadTime_;
        mutable std::mutex sensorMutex_;
        
        // Hardware interface simulation
        std::unique_ptr<void, void(*)(void*)> sensorHandle_;
        
        /**
         * @brief Validate temperature reading against acceptable range
         */
        bool validateTemperature(float temperature) const noexcept {
            return temperature >= MIN_VALID_TEMPERATURE && 
                   temperature <= MAX_VALID_TEMPERATURE;
        }
        
        /**
         * @brief Simulate hardware temperature reading with realistic behavior
         */
        float readRawTemperature() const {
            // Simulate ADC reading with some variation
            static float baseTemp = 25.0f;
            static int counter = 0;
            
            // Add realistic temperature variation
            float variation = std::sin(counter * 0.1f) * 2.0f + 
                             (std::rand() % 100 - 50) * 0.01f;
            counter++;
            
            return baseTemp + variation;
        }
        
    public:
        /**
         * @brief Constructor with sensor configuration
         * @param id Unique sensor identifier
         * @param type Sensor type description
         */
        explicit SensorManager(int id, const std::string& type = "TEMPERATURE") 
            : sensorId_(id), sensorType_(type), isActive_(false), 
              calibrationFactor_(1.0f), sensorHandle_(nullptr, [](void*){}) {
            
            if (id <= 0) {
                throw std::invalid_argument("Sensor ID must be positive");
            }
        }
        
        /**
         * @brief Destructor with proper resource cleanup
         */
        ~SensorManager() noexcept {
            try {
                std::lock_guard<std::mutex> lock(sensorMutex_);
                isActive_ = false;
                // Hardware cleanup would go here
            } catch (...) {
                // Suppress exceptions in destructor
            }
        }
        
        // Disable copy operations for resource safety
        SensorManager(const SensorManager&) = delete;
        SensorManager& operator=(const SensorManager&) = delete;
        
        // Enable move operations
        SensorManager(SensorManager&&) = default;
        SensorManager& operator=(SensorManager&&) = default;
        
        /**
         * @brief Initialize sensor hardware with error handling
         * @return true if initialization successful
         */
        bool initializeSensor() noexcept {
            try {
                std::lock_guard<std::mutex> lock(sensorMutex_);
                
                // Simulate hardware initialization
                // In real implementation: configure ADC, set sampling rate, etc.
                
                isActive_ = true;
                lastReadTime_ = std::chrono::system_clock::now();
                
                return true;
            } catch (const std::exception&) {
                isActive_ = false;
                return false;
            }
        }
        
        /**
         * @brief Read calibrated temperature with comprehensive error handling
         * @return Temperature in Celsius, SENSOR_ERROR_VALUE on error
         */
        float readTemperature() noexcept {
            try {
                std::lock_guard<std::mutex> lock(sensorMutex_);
                
                if (!isActive_) {
                    return static_cast<float>(SENSOR_ERROR_VALUE);
                }
                
                float rawTemp = readRawTemperature();
                
                if (!validateTemperature(rawTemp)) {
                    return static_cast<float>(SENSOR_ERROR_VALUE);
                }
                
                float calibratedTemp = rawTemp * calibrationFactor_.load();
                lastReadTime_ = std::chrono::system_clock::now();
                
                return calibratedTemp;
                
            } catch (const std::exception&) {
                return static_cast<float>(SENSOR_ERROR_VALUE);
            }
        }
        
        /**
         * @brief Get sensor operational status
         */
        bool getSensorStatus() const noexcept {
            return isActive_.load();
        }
        
        /**
         * @brief Calibrate sensor with reference temperature
         * @param referenceTemp Known reference temperature
         */
        void calibrateSensor(float referenceTemp) {
            if (!validateTemperature(referenceTemp)) {
                throw std::invalid_argument("Invalid reference temperature");
            }
            
            std::lock_guard<std::mutex> lock(sensorMutex_);
            
            if (!isActive_) {
                throw std::runtime_error("Cannot calibrate inactive sensor");
            }
            
            float currentReading = readRawTemperature();
            if (currentReading != 0.0f) {
                calibrationFactor_ = referenceTemp / currentReading;
            }
        }
        
        /**
         * @brief Get sensor identifier
         */
        int getSensorId() const noexcept { return sensorId_; }
        
        /**
         * @brief Set sensor active state
         */
        void setActive(bool active) noexcept { isActive_ = active; }
    };
}

// ============================================================================
// DATA PROCESSOR CLASS - Signal Processing and Filtering
// ============================================================================

namespace EmbeddedTempMonitor {
    
    /**
     * @brief Advanced data processing with digital filtering and validation
     * 
     * Implements moving average filter, outlier detection, and data validation
     * optimized for embedded systems with minimal memory footprint.
     */
    class DataProcessor {
    private:
        const float filterThreshold_;
        const float normalizationFactor_;
        std::atomic<float> lastProcessedValue_;
        
        std::deque<float> dataBuffer_;
        const size_t bufferSize_;
        mutable std::mutex processingMutex_;
        
        // Filter coefficients for enhanced signal processing
        const std::vector<float> filterCoefficients_;
        
        /**
         * @brief Detect outliers using statistical analysis
         */
        bool isOutlier(float value) const {
            if (dataBuffer_.size() < 3) return false;
            
            // Calculate mean and standard deviation
            float sum = std::accumulate(dataBuffer_.begin(), dataBuffer_.end(), 0.0f);
            float mean = sum / dataBuffer_.size();
            
            float variance = 0.0f;
            for (float val : dataBuffer_) {
                variance += (val - mean) * (val - mean);
            }
            variance /= dataBuffer_.size();
            float stdDev = std::sqrt(variance);
            
            // Use 2-sigma rule for outlier detection
            return std::abs(value - mean) > (2.0f * stdDev);
        }
        
    public:
        /**
         * @brief Constructor with configurable parameters
         */
        explicit DataProcessor(float threshold = 0.5f, float normFactor = 1.0f, 
                             size_t bufferSz = DEFAULT_BUFFER_SIZE)
            : filterThreshold_(threshold), normalizationFactor_(normFactor),
              lastProcessedValue_(0.0f), bufferSize_(bufferSz),
              filterCoefficients_{0.1f, 0.2f, 0.4f, 0.2f, 0.1f} {
            
            if (bufferSz == 0) {
                throw std::invalid_argument("Buffer size must be positive");
            }
        }
        
        /**
         * @brief Apply advanced digital filtering with outlier rejection
         */
        float filterData(float rawData) {
            std::lock_guard<std::mutex> lock(processingMutex_);
            
            // Reject obvious outliers before processing
            if (isOutlier(rawData)) {
                return lastProcessedValue_.load();
            }
            
            // Add to circular buffer
            dataBuffer_.push_back(rawData);
            if (dataBuffer_.size() > bufferSize_) {
                dataBuffer_.pop_front();
            }
            
            // Apply moving average filter
            float sum = std::accumulate(dataBuffer_.begin(), dataBuffer_.end(), 0.0f);
            return sum / dataBuffer_.size();
        }
        
        /**
         * @brief Normalize data to standard range with bounds checking
         */
        float normalizeData(float filteredData) const noexcept {
            float normalized = filteredData * normalizationFactor_;
            
            // Clamp to valid range
            return std::clamp(normalized, MIN_VALID_TEMPERATURE, MAX_VALID_TEMPERATURE);
        }
        
        /**
         * @brief Comprehensive data validation with multiple checks
         */
        bool validateData(float data) const noexcept {
            return !std::isnan(data) && 
                   !std::isinf(data) && 
                   data >= MIN_VALID_TEMPERATURE && 
                   data <= MAX_VALID_TEMPERATURE;
        }
        
        /**
         * @brief Complete processing pipeline with error recovery
         */
        float processTemperatureData(float rawTemp) {
            if (!validateData(rawTemp)) {
                return lastProcessedValue_.load();
            }
            
            try {
                float filtered = filterData(rawTemp);
                float normalized = normalizeData(filtered);
                
                if (validateData(normalized)) {
                    lastProcessedValue_ = normalized;
                    return normalized;
                }
                
                return lastProcessedValue_.load();
                
            } catch (const std::exception&) {
                return lastProcessedValue_.load();
            }
        }
        
        /**
         * @brief Reset processor state
         */
        void reset() noexcept {
            try {
                std::lock_guard<std::mutex> lock(processingMutex_);
                dataBuffer_.clear();
                lastProcessedValue_ = 0.0f;
            } catch (...) {
                // Suppress exceptions
            }
        }
        
        /**
         * @brief Get last processed value
         */
        float getLastProcessedValue() const noexcept {
            return lastProcessedValue_.load();
        }
    };
}

// ============================================================================
// CAN BUS INTERFACE CLASS - Automotive Communication
// ============================================================================

namespace EmbeddedTempMonitor {
    
    /**
     * @brief CAN bus interface for automotive embedded systems
     * 
     * Implements robust CAN communication with message formatting,
     * error handling, and transmission statistics for automotive applications.
     */
    class CANBusInterface {
    private:
        const uint32_t canId_;
        const uint32_t baudRate_;
        std::atomic<bool> isConnected_;
        
        mutable std::mutex canMutex_;
        uint32_t transmissionCount_;
        uint32_t errorCount_;
        
        // CAN hardware simulation
        std::unique_ptr<void, void(*)(void*)> canHandle_;
        
        /**
         * @brief Calculate simple checksum for message integrity
         */
        uint8_t calculateChecksum(const uint8_t* data, size_t length) const noexcept {
            uint8_t checksum = 0;
            for (size_t i = 0; i < length; ++i) {
                checksum ^= data[i];
            }
            return checksum;
        }
        
        /**
         * @brief Simulate hardware CAN transmission
         */
        bool transmitToHardware(const CANMessage& message) noexcept {
            try {
                // Simulate transmission delay and potential failures
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                
                // Simulate 99% success rate
                return (std::rand() % 100) < 99;
                
            } catch (...) {
                return false;
            }
        }
        
    public:
        /**
         * @brief Constructor with CAN configuration
         */
        explicit CANBusInterface(uint32_t id = DEFAULT_CAN_ID, 
                               uint32_t baud = DEFAULT_BAUD_RATE)
            : canId_(id), baudRate_(baud), isConnected_(false),
              transmissionCount_(0), errorCount_(0),
              canHandle_(nullptr, [](void*){}) {
            
            if (baud == 0) {
                throw std::invalid_argument("Baud rate must be positive");
            }
        }
        
        /**
         * @brief Destructor with proper cleanup
         */
        ~CANBusInterface() noexcept {
            try {
                std::lock_guard<std::mutex> lock(canMutex_);
                isConnected_ = false;
                // Hardware cleanup
            } catch (...) {
                // Suppress exceptions in destructor
            }
        }
        
        // Disable copy operations
        CANBusInterface(const CANBusInterface&) = delete;
        CANBusInterface& operator=(const CANBusInterface&) = delete;
        
        /**
         * @brief Initialize CAN bus hardware
         */
        bool initializeCANBus() noexcept {
            try {
                std::lock_guard<std::mutex> lock(canMutex_);
                
                // Simulate CAN controller initialization
                // Real implementation: configure CAN controller, set filters, etc.
                
                isConnected_ = true;
                transmissionCount_ = 0;
                errorCount_ = 0;
                
                return true;
                
            } catch (const std::exception&) {
                isConnected_ = false;
                return false;
            }
        }
        
        /**
         * @brief Send temperature data with automatic message formatting
         */
        bool sendData(float data) {
            if (!isConnected_.load()) {
                return false;
            }
            
            try {
                CANMessage message = formatCANMessage(data);
                return sendCANMessage(message);
                
            } catch (const std::exception&) {
                std::lock_guard<std::mutex> lock(canMutex_);
                ++errorCount_;
                return false;
            }
        }
        
        /**
         * @brief Format temperature data into CAN message with protocol compliance
         */
        CANMessage formatCANMessage(float data) const {
            CANMessage message;
            message.canId = canId_;
            message.dataLength = 8;
            message.isExtended = false;
            message.timestamp = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count()
            );
            
            // Convert float to bytes (IEEE 754 format)
            union {
                float f;
                uint8_t bytes[4];
            } converter;
            converter.f = data;
            
            // Message format: [TYPE][NODE_ID][TEMP_BYTES][RESERVED][CHECKSUM]
            message.data[0] = 0x01;  // Message type: temperature data
            message.data[1] = 0x01;  // Node ID
            message.data[2] = converter.bytes[0];
            message.data[3] = converter.bytes[1];
            message.data[4] = converter.bytes[2];
            message.data[5] = converter.bytes[3];
            message.data[6] = 0x00;  // Reserved
            message.data[7] = calculateChecksum(message.data, 7);
            
            return message;
        }
        
        /**
         * @brief Send CAN message with retry logic and error handling
         */
        bool sendCANMessage(const CANMessage& message) {
            std::lock_guard<std::mutex> lock(canMutex_);
            
            if (!isConnected_) {
                ++errorCount_;
                return false;
            }
            
            // Retry logic for robust transmission
            constexpr int MAX_RETRIES = 3;
            for (int retry = 0; retry < MAX_RETRIES; ++retry) {
                if (transmitToHardware(message)) {
                    ++transmissionCount_;
                    return true;
                }
                
                // Brief delay before retry
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            ++errorCount_;
            return false;
        }
        
        /**
         * @brief Get connection status
         */
        bool getConnectionStatus() const noexcept {
            return isConnected_.load();
        }
        
        /**
         * @brief Get transmission statistics
         */
        std::pair<uint32_t, uint32_t> getTransmissionStats() const {
            std::lock_guard<std::mutex> lock(canMutex_);
            return {transmissionCount_, errorCount_};
        }
    };
}

// ============================================================================
// ALERT SERVICE CLASS - Intelligent Alert Management
// ============================================================================

namespace EmbeddedTempMonitor {
    
    /**
     * @brief Advanced alert service with intelligent threshold management
     * 
     * Implements sophisticated alert logic with cooldown periods, severity
     * classification, and callback mechanisms for flexible notification.
     */
    class AlertService {
    private:
        std::atomic<float> alertThreshold_;
        std::atomic<bool> alertEnabled_;
        std::chrono::system_clock::time_point lastAlertTime_;
        const std::chrono::minutes cooldownPeriod_;
        
        std::vector<Alert> alertHistory_;
        std::atomic<uint32_t> nextAlertId_;
        
        std::vector<std::function<void(const Alert&)>> alertCallbacks_;
        mutable std::mutex alertMutex_;
        
        /**
         * @brief Determine alert severity based on temperature excess
         */
        AlertSeverity determineSeverity(float temperature) const noexcept {
            float excess = temperature - alertThreshold_.load();
            
            if (excess > 20.0f) return AlertSeverity::CRITICAL;
            if (excess > 10.0f) return AlertSeverity::HIGH;
            if (excess > 5.0f) return AlertSeverity::MEDIUM;
            return AlertSeverity::LOW;
        }
        
        /**
         * @brief Check if cooldown period has elapsed
         */
        bool isCooldownElapsed() const noexcept {
            auto now = std::chrono::system_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(
                now - lastAlertTime_);
            return elapsed >= cooldownPeriod_;
        }
        
        /**
         * @brief Format alert message with comprehensive information
         */
        std::string formatAlertMessage(float temperature, AlertSeverity severity) const {
            const char* severityStr[] = {"LOW", "MEDIUM", "HIGH", "CRITICAL"};
            
            return "Temperature Alert [" + 
                   std::string(severityStr[static_cast<int>(severity)]) + 
                   "]: " + std::to_string(temperature) + "°C exceeds threshold " +
                   std::to_string(alertThreshold_.load()) + "°C";
        }
        
    public:
        /**
         * @brief Constructor with configurable parameters
         */
        explicit AlertService(float threshold = DEFAULT_ALERT_THRESHOLD,
                            std::chrono::minutes cooldown = DEFAULT_COOLDOWN_PERIOD)
            : alertThreshold_(threshold), alertEnabled_(true),
              cooldownPeriod_(cooldown), nextAlertId_(1) {
            
            if (threshold < MIN_VALID_TEMPERATURE || threshold > MAX_VALID_TEMPERATURE) {
                throw std::invalid_argument("Invalid alert threshold");
            }
            
            lastAlertTime_ = std::chrono::system_clock::time_point::min();
        }
        
        /**
         * @brief Trigger alert with intelligent processing
         */
        void triggerAlert(float temperature) {
            if (!alertEnabled_.load() || !checkThreshold(temperature)) {
                return;
            }
            
            std::lock_guard<std::mutex> lock(alertMutex_);
            
            if (!isCooldownElapsed()) {
                return;  // Still in cooldown period
            }
            
            // Create new alert
            Alert newAlert;
            newAlert.id = nextAlertId_++;
            newAlert.temperature = temperature;
            newAlert.severity = determineSeverity(temperature);
            newAlert.timestamp = std::chrono::system_clock::now();
            newAlert.message = formatAlertMessage(temperature, newAlert.severity);
            newAlert.isAcknowledged = false;
            
            // Store in history (maintain reasonable size)
            alertHistory_.push_back(newAlert);
            if (alertHistory_.size() > 100) {  // Keep last 100 alerts
                alertHistory_.erase(alertHistory_.begin());
            }
            
            lastAlertTime_ = newAlert.timestamp;
            
            // Notify all registered callbacks
            for (const auto& callback : alertCallbacks_) {
                try {
                    callback(newAlert);
                } catch (const std::exception&) {
                    // Continue with other callbacks even if one fails
                }
            }
        }
        
        /**
         * @brief Check if temperature exceeds threshold
         */
        bool checkThreshold(float temperature) const noexcept {
            return temperature > alertThreshold_.load();
        }
        
        /**
         * @brief Register alert callback function
         */
        void registerAlertCallback(std::function<void(const Alert&)> callback) {
            std::lock_guard<std::mutex> lock(alertMutex_);
            alertCallbacks_.push_back(std::move(callback));
        }
        
        /**
         * @brief Get recent alert history
         */
        std::vector<Alert> getAlertHistory(size_t maxCount = 10) const {
            std::lock_guard<std::mutex> lock(alertMutex_);
            
            std::vector<Alert> result;
            size_t startIndex = alertHistory_.size() > maxCount ? 
                               alertHistory_.size() - maxCount : 0;
            
            for (size_t i = startIndex; i < alertHistory_.size(); ++i) {
                result.push_back(alertHistory_[i]);
            }
            
            return result;
        }
        
        /**
         * @brief Acknowledge alert by ID
         */
        bool acknowledgeAlert(uint32_t alertId) {
            std::lock_guard<std::mutex> lock(alertMutex_);
            
            auto it = std::find_if(alertHistory_.begin(), alertHistory_.end(),
                [alertId](const Alert& alert) { return alert.id == alertId; });
            
            if (it != alertHistory_.end()) {
                it->isAcknowledged = true;
                return true;
            }
            
            return false;
        }
        
        /**
         * @brief Get/Set alert threshold
         */
        float getAlertThreshold() const noexcept { return alertThreshold_.load(); }
        void setAlertThreshold(float threshold) { alertThreshold_ = threshold; }
        
        /**
         * @brief Enable/Disable alerts
         */
        void setAlertEnabled(bool enabled) noexcept { alertEnabled_ = enabled; }
        bool isAlertEnabled() const noexcept { return alertEnabled_.load(); }
    };
}

// ============================================================================
// MAIN APPLICATION CLASS - System Integration and Orchestration
// ============================================================================

namespace EmbeddedTempMonitor {
    
    /**
     * @brief Main application orchestrating the complete temperature monitoring system
     * 
     * Integrates all components with robust error handling, performance monitoring,
     * and graceful shutdown capabilities for embedded automotive applications.
     */
    class TemperatureMonitorApp {
    private:
        // Core system components
        std::unique_ptr<SensorManager> sensorManager_;
        std::unique_ptr<DataProcessor> dataProcessor_;
        std::unique_ptr<CANBusInterface> canInterface_;
        std::unique_ptr<AlertService> alertService_;
        
        // Application control
        std::atomic<bool> isRunning_;
        std::unique_ptr<std::thread> monitoringThread_;
        const std::chrono::milliseconds readInterval_;
        
        // System statistics
        std::atomic<uint32_t> totalReadings_;
        std::atomic<uint32_t> successfulTransmissions_;
        std::atomic<uint32_t> alertsTriggered_;
        std::atomic<uint32_t> errorCount_;
        
        mutable std::mutex statsMutex_;
        
        /**
         * @brief Main monitoring loop with comprehensive error handling
         */
        void monitoringLoop() noexcept {
            while (isRunning_.load()) {
                auto startTime = std::chrono::steady_clock::now();
                
                try {
                    processSingleCycle();
                } catch (const std::exception&) {
                    ++errorCount_;
                    // Continue operation despite errors
                }
                
                // Maintain consistent timing
                auto endTime = std::chrono::steady_clock::now();
                auto processingTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                    endTime - startTime);
                
                if (processingTime < readInterval_) {
                    std::this_thread::sleep_for(readInterval_ - processingTime);
                }
            }
        }
        
        /**
         * @brief Handle alert notifications with CAN bus integration
         */
        void handleAlert(const Alert& alert) noexcept {
            try {
                ++alertsTriggered_;
                
                // Send special alert message via CAN bus
                if (canInterface_ && canInterface_->getConnectionStatus()) {
                    // Use negative temperature to indicate alert condition
                    canInterface_->sendData(-1.0f * static_cast<float>(
                        static_cast<int>(alert.severity) + 1));
                }
                
                // Additional alert handling could be added here
                // (e.g., GPIO outputs, LED indicators, etc.)
                
            } catch (const std::exception&) {
                ++errorCount_;
            }
        }
        
    public:
        /**
         * @brief Constructor with comprehensive system initialization
         */
        explicit TemperatureMonitorApp(
            int sensorId = 1,
            std::chrono::milliseconds interval = DEFAULT_READ_INTERVAL,
            float alertThreshold = DEFAULT_ALERT_THRESHOLD)
            : readInterval_(interval), isRunning_(false),
              totalReadings_(0), successfulTransmissions_(0),
              alertsTriggered_(0), errorCount_(0) {
            
            if (sensorId <= 0) {
                throw std::invalid_argument("Invalid sensor ID");
            }
            
            // Initialize components with proper error handling
            try {
                sensorManager_ = std::make_unique<SensorManager>(sensorId, "TEMPERATURE");
                dataProcessor_ = std::make_unique<DataProcessor>(0.5f, 1.0f, DEFAULT_BUFFER_SIZE);
                canInterface_ = std::make_unique<CANBusInterface>(DEFAULT_CAN_ID, DEFAULT_BAUD_RATE);
                alertService_ = std::make_unique<AlertService>(alertThreshold, DEFAULT_COOLDOWN_PERIOD);
                
                // Register alert callback
                alertService_->registerAlertCallback(
                    [this](const Alert& alert) { handleAlert(alert); });
                    
            } catch (const std::exception& e) {
                throw std::runtime_error("Failed to initialize system components: " + 
                                       std::string(e.what()));
            }
        }
        
        /**
         * @brief Destructor with graceful shutdown
         */
        ~TemperatureMonitorApp() noexcept {
            stop();
        }
        
        // Disable copy operations
        TemperatureMonitorApp(const TemperatureMonitorApp&) = delete;
        TemperatureMonitorApp& operator=(const TemperatureMonitorApp&) = delete;
        
        /**
         * @brief Initialize all system components
         */
        bool initialize() noexcept {
            try {
                bool success = true;
                
                // Initialize components in proper order
                success &= sensorManager_->initializeSensor();
                success &= canInterface_->initializeCANBus();
                
                if (!success) {
                    return false;
                }
                
                // Reset statistics
                totalReadings_ = 0;
                successfulTransmissions_ = 0;
                alertsTriggered_ = 0;
                errorCount_ = 0;
                
                return true;
                
            } catch (const std::exception&) {
                return false;
            }
        }
        
        /**
         * @brief Start the monitoring system
         */
        bool start() {
            if (isRunning_.load()) {
                return false;  // Already running
            }
            
            if (!initialize()) {
                throw std::runtime_error("System initialization failed");
            }
            
            isRunning_ = true;
            
            // Start monitoring thread
            monitoringThread_ = std::make_unique<std::thread>(
                &TemperatureMonitorApp::monitoringLoop, this);
            
            return true;
        }
        
        /**
         * @brief Stop the monitoring system gracefully
         */
        void stop() noexcept {
            isRunning_ = false;
            
            if (monitoringThread_ && monitoringThread_->joinable()) {
                monitoringThread_->join();
            }
            
            monitoringThread_.reset();
        }
        
        /**
         * @brief Process single monitoring cycle
         */
        bool processSingleCycle() {
            if (!sensorManager_ || !dataProcessor_ || !canInterface_ || !alertService_) {
                throw std::runtime_error("System components not initialized");
            }
            
            // Read temperature from sensor
            float rawTemperature = sensorManager_->readTemperature();
            if (rawTemperature == static_cast<float>(SENSOR_ERROR_VALUE)) {
                ++errorCount_;
                return false;
            }
            
            ++totalReadings_;
            
            // Process the data
            float processedTemperature = dataProcessor_->processTemperatureData(rawTemperature);
            
            // Send via CAN bus
            if (canInterface_->sendData(processedTemperature)) {
                ++successfulTransmissions_;
            } else {
                ++errorCount_;
            }
            
            // Check for alerts
            if (alertService_->checkThreshold(processedTemperature)) {
                alertService_->triggerAlert(processedTemperature);
            }
            
            return true;
        }
        
        /**
         * @brief Get comprehensive system status
         */
        std::string getSystemStatus() const {
            std::lock_guard<std::mutex> lock(statsMutex_);
            
            std::string status = "=== Temperature Monitor System Status ===\n";
            status += "System Running: " + std::string(isRunning_.load() ? "YES" : "NO") + "\n";
            status += "Total Readings: " + std::to_string(totalReadings_.load()) + "\n";
            status += "Successful Transmissions: " + std::to_string(successfulTransmissions_.load()) + "\n";
            status += "Alerts Triggered: " + std::to_string(alertsTriggered_.load()) + "\n";
            status += "Error Count: " + std::to_string(errorCount_.load()) + "\n";
            
            if (sensorManager_) {
                status += "Sensor Status: " + 
                         std::string(sensorManager_->getSensorStatus() ? "ACTIVE" : "INACTIVE") + "\n";
            }
            
            if (canInterface_) {
                status += "CAN Bus Status: " + 
                         std::string(canInterface_->getConnectionStatus() ? "CONNECTED" : "DISCONNECTED") + "\n";
                
                auto [txCount, errCount] = canInterface_->getTransmissionStats();
                status += "CAN TX Count: " + std::to_string(txCount) + "\n";
                status += "CAN Errors: " + std::to_string(errCount) + "\n";
            }
            
            if (alertService_) {
                status += "Alert Threshold: " + std::to_string(alertService_->getAlertThreshold()) + "°C\n";
                status += "Alerts Enabled: " + 
                         std::string(alertService_->isAlertEnabled() ? "YES" : "NO") + "\n";
            }
            
            if (dataProcessor_) {
                status += "Last Processed Value: " + 
                         std::to_string(dataProcessor_->getLastProcessedValue()) + "°C\n";
            }
            
            return status;
        }
        
        /**
         * @brief Get system performance metrics
         */
        struct SystemMetrics {
            uint32_t totalReadings;
            uint32_t successfulTransmissions;
            uint32_t alertsTriggered;
            uint32_t errorCount;
            float successRate;
            bool isOperational;
        };
        
        SystemMetrics getSystemMetrics() const noexcept {
            SystemMetrics metrics;
            metrics.totalReadings = totalReadings_.load();
            metrics.successfulTransmissions = successfulTransmissions_.load();
            metrics.alertsTriggered = alertsTriggered_.load();
            metrics.errorCount = errorCount_.load();
            
            metrics.successRate = metrics.totalReadings > 0 ? 
                static_cast<float>(metrics.successfulTransmissions) / metrics.totalReadings : 0.0f;
            
            metrics.isOperational = isRunning_.load() && 
                                   (sensorManager_ ? sensorManager_->getSensorStatus() : false) &&
                                   (canInterface_ ? canInterface_->getConnectionStatus() : false);
            
            return metrics;
        }
        
        // Component access methods for advanced configuration
        SensorManager* getSensorManager() const noexcept { return sensorManager_.get(); }
        DataProcessor* getDataProcessor() const noexcept { return dataProcessor_.get(); }
        CANBusInterface* getCANInterface() const noexcept { return canInterface_.get(); }
        AlertService* getAlertService() const noexcept { return alertService_.get(); }
    };
}

// ============================================================================
// MAIN FUNCTION - Application Entry Point
// ============================================================================

/**
 * @brief Main function demonstrating the embedded temperature monitoring system
 * 
 * Showcases system initialization, operation, and graceful shutdown with
 * comprehensive error handling and performance monitoring.
 */
int main() {
    using namespace EmbeddedTempMonitor;
    
    try {
        std::cout << "=== Embedded Temperature Sensor Monitoring System ===" << std::endl;
        std::cout << "Initializing system components..." << std::endl;
        
        // Create and configure the monitoring application
        TemperatureMonitorApp app(1, std::chrono::milliseconds(1000), 75.0f);
        
        // Start the monitoring system
        if (!app.start()) {
            std::cerr << "Failed to start monitoring system!" << std::endl;
            return 1;
        }
        
        std::cout << "System started successfully. Monitoring temperature..." << std::endl;
        std::cout << "Press Enter to view status, 'q' + Enter to quit." << std::endl;
        
        // Main interaction loop
        std::string input;
        while (std::getline(std::cin, input)) {
            if (input == "q" || input == "quit") {
                break;
            }
            
            // Display system status
            std::cout << "\n" << app.getSystemStatus() << std::endl;
            
            // Display performance metrics
            auto metrics = app.getSystemMetrics();
            std::cout << "=== Performance Metrics ===" << std::endl;
            std::cout << "Success Rate: " << (metrics.successRate * 100.0f) << "%" << std::endl;
            std::cout << "System Operational: " << (metrics.isOperational ? "YES" : "NO") << std::endl;
            
            // Display recent alerts
            if (app.getAlertService()) {
                auto alerts = app.getAlertService()->getAlertHistory(5);
                if (!alerts.empty()) {
                    std::cout << "\n=== Recent Alerts ===" << std::endl;
                    for (const auto& alert : alerts) {
                        std::cout << "Alert ID " << alert.id << ": " << alert.message << std::endl;
                    }
                }
            }
            
            std::cout << "\nPress Enter for status update, 'q' + Enter to quit." << std::endl;
        }
        
        std::cout << "Shutting down system..." << std::endl;
        app.stop();
        
        // Final status report
        std::cout << "\n=== Final System Report ===" << std::endl;
        std::cout << app.getSystemStatus() << std::endl;
        
        auto finalMetrics = app.getSystemMetrics();
        std::cout << "Final Success Rate: " << (finalMetrics.successRate * 100.0f) << "%" << std::endl;
        
        std::cout << "System shutdown complete." << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "System error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown system error occurred!" << std::endl;
        return 1;
    }
}

#endif // EMBEDDED_TEMPERATURE_MONITOR_H
