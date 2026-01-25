#pragma once

#include <string>
#include <boost/asio.hpp>
#include <memory>
#include <mutex>

class MarlinController {
public:
    MarlinController(const std::string& port, unsigned int baudRate);
    ~MarlinController();

    // Connection
    bool connect();
    void disconnect();
    bool isConnected() const { return connected_; }

    // Commands
    void sendGcode(const std::string& gcode);
    void enableMotors();
    void disableMotors();
    void setLight(bool on);
    void setFan(bool on);
    
    // Movement
    void advanceFilm2(float mmY, float unitsX, float speed = 100.0f, bool wait = false); // Move the Y-axis by a specified distance in mm
    void home();
    
    void sendAndWait(const std::string &gcode);

    void waitForMoveCompletion();
    bool checkForOK();

    // Status
    std::string getLastResponse() const;
    void waitForOk(int timeout_ms = 5000);

    std::string readResponse();

private:
   
    std::string port_;
    unsigned int baudRate_;
    bool connected_ = false;
    int eatOKs = 0; // Number of OKs to eat

    std::unique_ptr<boost::asio::io_service> ioService_;
    std::unique_ptr<boost::asio::serial_port> serialPort_;
    
    std::string lastResponse_;
    mutable std::mutex mutex_;
};
