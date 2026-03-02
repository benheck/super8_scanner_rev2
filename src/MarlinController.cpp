#include "MarlinController.h"
#include <iostream>
#include <thread>
#include <chrono>

MarlinController::MarlinController(const std::string& port, unsigned int baudRate)
    : port_(port), baudRate_(baudRate) {
    ioService_ = std::make_unique<boost::asio::io_service>();
}

MarlinController::~MarlinController() {
    disconnect();
}

bool MarlinController::connect() {
    if (connected_) return true;
    
    try {
        serialPort_ = std::make_unique<boost::asio::serial_port>(*ioService_, port_);
        
        serialPort_->set_option(boost::asio::serial_port_base::baud_rate(baudRate_));
        serialPort_->set_option(boost::asio::serial_port_base::character_size(8));
        serialPort_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serialPort_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serialPort_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        
        connected_ = true;
        std::cout << "Connected to Marlin at " << port_ << std::endl;

        sendAndWait("G91"); // G91 sets relative positioning mode
        std::cout << "Marlin set to relative positioning mode (G91)." << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to connect to Marlin: " << e.what() << std::endl;
        connected_ = false;
        return false;
    }
}

void MarlinController::disconnect() {
    if (!connected_) return;
    
    try {
        if (serialPort_ && serialPort_->is_open()) {
            serialPort_->close();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error closing serial port: " << e.what() << std::endl;
    }
    
    connected_ = false;
    std::cout << "Disconnected from Marlin" << std::endl;
}

void MarlinController::sendGcode(const std::string& gcode) {
    if (!connected_) return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    try {
        std::string command = gcode + "\n";
        boost::asio::write(*serialPort_, boost::asio::buffer(command));
        std::cout << "Sent: " << gcode << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error sending G-code: " << e.what() << std::endl;
    }
}

std::string MarlinController::getLastResponse() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lastResponse_;
}

std::string MarlinController::readResponse() {
    if (!connected_ || !serialPort_ || !serialPort_->is_open()) {
        return "";
    }
    
    try {
        boost::asio::streambuf responseBuffer;
        boost::asio::read_until(*serialPort_, responseBuffer, "\n");
        std::istream responseStream(&responseBuffer);
        std::string response;
        std::getline(responseStream, response);
        
        // Remove carriage return if present
        if (!response.empty() && response.back() == '\r') {
            response.pop_back();
        }
        
        std::lock_guard<std::mutex> lock(mutex_);
        lastResponse_ = response;
        
        std::cout << "Response: " << response << std::endl;
        return response;
    } catch (const std::exception &e) {
        std::cerr << "Error reading response: " << e.what() << std::endl;
        return "";
    }
}

void MarlinController::waitForOk(int timeout_ms) {
    if (!connected_ || !serialPort_ || !serialPort_->is_open()) return;
    
    auto start = std::chrono::steady_clock::now();
    
    while (true) {
        try {
            // Try to read with a short timeout
            boost::asio::streambuf responseBuffer;
            
            // Set a deadline for the read operation
            boost::asio::deadline_timer timer(*ioService_);
            timer.expires_from_now(boost::posix_time::milliseconds(100));
            
            boost::system::error_code ec;
            bool dataRead = false;
            
            // Start async read
            boost::asio::async_read_until(*serialPort_, responseBuffer, "\n",
                [&](const boost::system::error_code& error, std::size_t) {
                    ec = error;
                    dataRead = true;
                    timer.cancel();
                });
            
            // Start timer
            timer.async_wait([&](const boost::system::error_code& error) {
                if (!error) {
                    serialPort_->cancel();
                }
            });
            
            // Run the IO service
            ioService_->reset();
            ioService_->run();
            
            if (dataRead && !ec) {
                std::istream responseStream(&responseBuffer);
                std::string response;
                std::getline(responseStream, response);
                
                // Remove carriage return if present
                if (!response.empty() && response.back() == '\r') {
                    response.pop_back();
                }
                
                std::lock_guard<std::mutex> lock(mutex_);
                lastResponse_ = response;
                
                std::cout << "Response: " << response << std::endl;
                
                if (response.find("ok") != std::string::npos) {
                    break;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in waitForOk: " << e.what() << std::endl;
        }
        
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
            
        if (elapsed > timeout_ms) {
            std::cerr << "Timeout waiting for OK (" << timeout_ms << "ms)" << std::endl;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void MarlinController::enableMotors() {
    sendGcode("M17");
    waitForOk();
}

void MarlinController::disableMotors() {
    sendGcode("M18");
    waitForOk();
}

void MarlinController::setLight(bool on) {

    if (on) {
        sendGcode("M42 P205 S255");  // Manual pin # for BTT SKR 1.3
    } else {
        sendGcode("M42 P205 S0");   //This is heated bed you have to disable other Marlin checks in firmware to allow this raw manual control
    }

    waitForOk();

}

void MarlinController::setFan(bool on) {

    if (on) {
        sendGcode("M106 S255");
    } else {
        sendGcode("M107");
    }

    waitForOk();

}

void MarlinController::advanceFilm2(float mmY, float unitsX, float speed, bool wait) {
    // Generate G-code for moving the Y-axis
    std::string gcode = "G1 Y" + std::to_string(mmY) + " X" + std::to_string(unitsX) + " F" + std::to_string(speed);

    if (wait) {
        sendAndWait(gcode); // Send the G-code and wait for OK
        waitForMoveCompletion(); // Ensure all movements are complete
    } else {
        sendGcode(gcode); // Send the G-code without waiting
    }
}

void MarlinController::home() {
    sendGcode("G28");
    waitForOk(30000);  // 30 second timeout for homing
}

void MarlinController::sendAndWait(const std::string& gcode) {
    sendGcode(gcode);
    waitForOk();
}

void MarlinController::waitForMoveCompletion() {
    // Send M400 to wait for all moves to complete
    sendGcode("M400");
    waitForOk();
}

bool MarlinController::checkForOK() {
    if (!connected_ || !serialPort_ || !serialPort_->is_open()) {
        return false;
    }
    
    try {
        boost::asio::streambuf responseBuffer;
        boost::asio::deadline_timer timer(*ioService_);
        timer.expires_from_now(boost::posix_time::milliseconds(1)); // Very short timeout
        
        boost::system::error_code ec;
        bool dataRead = false;
        
        // Start async read
        boost::asio::async_read_until(*serialPort_, responseBuffer, "\n",
            [&](const boost::system::error_code& error, std::size_t) {
                ec = error;
                dataRead = true;
                timer.cancel();
            });
        
        // Start timer
        timer.async_wait([&](const boost::system::error_code& error) {
            if (!error) {
                serialPort_->cancel();
            }
        });
        
        // Run the IO service
        ioService_->reset();
        ioService_->run();
        
        if (dataRead && !ec) {
            std::istream responseStream(&responseBuffer);
            std::string response;
            std::getline(responseStream, response);
            
            // Remove carriage return if present
            if (!response.empty() && response.back() == '\r') {
                response.pop_back();
            }
            
            std::lock_guard<std::mutex> lock(mutex_);
            lastResponse_ = response;
            
            std::cout << "Response: " << response << std::endl;
            
            return response.find("ok") != std::string::npos;
        }
        
        return false;
    } catch (const std::exception &e) {
        // No data available or error reading
        return false;
    }
}
