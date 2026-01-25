#include "Camera.h"
#include "MarlinController.h"
#include "Config.h"

#include <thread>
#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QLineEdit>
#include <QFileDialog>
#include <QMessageBox>
#include <QImage>
#include <QPixmap>
#include <QtConcurrent>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <QStandardPaths>
#include <QPalette>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

// Application state
enum State {
    PREVIEW,        // Live video preview with alignment guides
    SETUP,          // Still image mode for sprocket calibration
    SCANNING        // Automated scanning mode
};

class ScannerApp : public QWidget {
    Q_OBJECT

public:
    ScannerApp(QWidget* parent = nullptr);
    ~ScannerApp();

private slots:
    void updateFrame();
    void onStartScan();
    void onStopScan();
    void onSwitchToSetup();
    void onSwitchToPreview();
    void onLightToggle();
    void onFanToggle();
    void onMotorsToggle();
    void onFocusToggle();
    void onSelectExportFolder();
    void onSprocketThresholdChanged(int value);
    void initializeHardware();

private:
    // Core components
    PiCamera camera_;
    MarlinController* marlin_;
    QTimer* updateTimer_;
    std::atomic<bool> initializationComplete_;
    
    // State
    State currentState_;
    bool scanning_;
    bool lightOn_;
    bool fanOn_;
    bool motorsEnabled_;
    int frameCount_;
    std::string exportFolder_;
    std::string imageFilePrefix_ = "image_";     //Prefix for saved image files ie "old_cars" + .png
    bool focusZoomedIn_;                         //In preview, show the frame non-scaled
    
    // Film parameters
    float spoolDiameter_;
    float movePerFrame_;
    
    // Sprocket detection
    int sprocketBrightnessThreshold_;
    int sprocketMinHeight_;
    cv::Point sprocketCenter_;
    bool sprocketDetected_;

    int leftSprocketEdge = 500;
    int rightSprocketEdge = 750;
    int sprocketDetectHeight = 1000;
    int leftScanEdge = 900;
    int rightScanEdge = 3000;
    int verticalScanHeight = 1500;



    float film_thickness_mm = 0.15f;        // Film thickness per wrap
    float super8frameHeight = 4.234f;
    float base_spool_diameter = 31.3f;  //Diameter "pully" Marlin is configed to
    float movement_mm_turn_target = base_spool_diameter * M_PI; // mm per rotation of the spool
    float spool_diameter_mm_ = 31.3f;         // Set this to your empty take-up spool diameter
    float movement_mm_turn;
    float mmPerFrame;
    float focusScore = 0.0f; // Focus score for the image
    int frameNumber = 0; // Frame number for the current scan
    bool autoExposureOn = true; // Flag to indicate if auto exposure is locked
    bool stopScanFlag = false; // Flag to indicate if the scan should be stopped
    int skipFrameCount = 0;
    int skipFrameTarget = 50;



    // UI components
    QLabel* videoLabel_;        //Preview window, left uppper
    QLabel* adjustmentLabel_;   //Under preview window, left lower

    QPushButton* focusButton_ = new QPushButton("Focus Off");
    QPushButton* previewButton_ = new QPushButton("Preview Mode");
    QPushButton* setupButton_ = new QPushButton("Setup Mode");
    QPushButton* scanButton_ = new QPushButton("Start Scan");
    QPushButton* stopButton_ = new QPushButton("Stop Scan");

    QPushButton* setSpoolDiameterButton = new QPushButton("Set Spool Diameter");
    QPushButton* fineButtonForward = new QPushButton("<< Fine Forward");
    QPushButton* nudgeButtonForward = new QPushButton("<< Nudge Forward");
    QPushButton* advance1Button = new QPushButton("<< Frame +1");
    QPushButton* fineButtonBack = new QPushButton(">> Fine Backward");
    QPushButton* nudgeButtonBack = new QPushButton(">> Nudge Backward");
    QPushButton* rewind1Button = new QPushButton(">> Frame -1");
    QPushButton* advance10Button = new QPushButton("<< Frame +10");
    QPushButton* advance50Button = new QPushButton("<< Frame +50");
    QPushButton* advance100Button = new QPushButton("<< Frame +100");

    QPushButton* lightButton_ = new QPushButton("Light: OFF");
    QPushButton* fanButton_ = new QPushButton("Fan: OFF");
    QPushButton* motorsButton_ = new QPushButton("Motors: OFF");
    QPushButton* setImagePrefixButton_ = new QPushButton("Set Image Prefix");
    QPushButton* exportFolderButton_ = new QPushButton("Set Export Folder");

    QSlider* sprocketBrightSlider_;
    QSlider* sprocketHeightSlider_;
    QSlider* sprocketDetectHeightSlider_;
    QLineEdit* spoolDiameterEdit_;
    QSlider* leftSprocketDetectSlider_;
    QSlider* rightSprocketDetectSlider_;
    QSlider* leftScanEdgeSlider_;
    QSlider* rightScanEdgeSlider_;
    QSlider* verticalScanHeightSlider_;
    QLineEdit* exportPrefixEdit_;
    
    // Current frames
    cv::Mat frame;              //Temp
    cv::Mat currentFrame_;
    cv::Mat setupFrame_;
    cv::Mat scanFrame_;
    
    // Helper methods
    void initializeUI();
    QLabel* createLabel(const QString &text, int = 14);
    void connectSignals();
    void updateVideoDisplay(const cv::Mat& frame);
    void compute_move_per_frame();
    void advanceFilmTracking(float frames, float speed, bool waitForResponse = false);
    int computeSkipFrameTarget(float spool_diameter_mm);
    void computeMovePerFrame();
    void drawPreviewOverlay(cv::Mat& frame);
    float computeFocusScore(const cv::Mat &image);
    void drawSetupOverlay(cv::Mat& frame);
    void drawScanningOverlay(cv::Mat& frame);
    void scanSingleFrame();
    cv::Mat processFrame(const cv::Mat& input);

    void saveSetupSettings();
    void loadSetupSettings();

    void drawText(cv::Mat& frame, std::string text, int x, int y, double fontScale = 1.0);

};

ScannerApp::ScannerApp(QWidget* parent)
    : QWidget(parent),
      marlin_(nullptr),
      initializationComplete_(false),
      currentState_(PREVIEW),
      scanning_(false),
      lightOn_(false),
      fanOn_(false),
      motorsEnabled_(false),
      frameCount_(0),
      exportFolder_(Config::DEFAULT_EXPORT_FOLDER),
      spoolDiameter_(Config::BASE_SPOOL_DIAMETER),
      sprocketBrightnessThreshold_(Config::SPROCKET_THRESHOLD_DEFAULT),
      sprocketDetected_(false) {
    
    initializeUI();
    connectSignals();
    
    setWindowTitle("Super 8mm Scanner - libcamera");
    resize(1800, 900);
    setStyleSheet("QWidget#ScannerApp { background-color: #333333; }");
    setObjectName("ScannerApp");
    
    // Defer hardware initialization until after window is shown
    QTimer::singleShot(100, this, &ScannerApp::initializeHardware);
}

ScannerApp::~ScannerApp() {
    // Stop initialization if still running
    initializationComplete_ = true;
    
    if (marlin_) {
        marlin_->setLight(false);
        marlin_->setFan(false);
        marlin_->disableMotors();
        delete marlin_;
    }
    camera_.shutdown();
}

void ScannerApp::initializeUI() {
    QHBoxLayout* mainLayout = new QHBoxLayout(this);
    
    // Left panel - Video display
    QVBoxLayout* leftLayout = new QVBoxLayout();
    
    videoLabel_ = new QLabel();
    videoLabel_->setFixedSize(1280, 720);       //Preview window size (pixels on screen)
    videoLabel_->setStyleSheet("background-color: black; border: 2px solid #666;");
    videoLabel_->setScaledContents(false);
    videoLabel_->setAlignment(Qt::AlignCenter);

    leftLayout->addWidget(videoLabel_);

    QGridLayout* adjustmentLayout = new QGridLayout();      //Below the video on the lower left
    adjustmentLayout->setAlignment(Qt::AlignTop);

    //Sprocket and scan edge sliders UNDER VIDEO WINDOW
    adjustmentLayout->addWidget(createLabel("SPROCKET BRIGHTNESS THRESHOLD"), 0, 0);
    sprocketBrightSlider_ = new QSlider(Qt::Horizontal);
    sprocketBrightSlider_->setRange(0, 100);
    sprocketBrightSlider_->setValue(sprocketBrightnessThreshold_);
    sprocketBrightSlider_->setStyleSheet("padding: 5px;");
    adjustmentLayout->addWidget(sprocketBrightSlider_, 1, 0);

    adjustmentLayout->addWidget(createLabel("SPROCKET MINIMUM HEIGHT"), 2, 0);
    sprocketHeightSlider_ = new QSlider(Qt::Horizontal);
    sprocketHeightSlider_->setRange(300, 500);              //Pixels
    sprocketHeightSlider_->setValue(sprocketMinHeight_);
    sprocketHeightSlider_->setStyleSheet("padding: 5px;");
    adjustmentLayout->addWidget(sprocketHeightSlider_, 3, 0);

    adjustmentLayout->addWidget(createLabel("SPROCKET DETECTION HEIGHT"), 4, 0);
    sprocketDetectHeightSlider_ = new QSlider(Qt::Horizontal);
    sprocketDetectHeightSlider_->setRange(300, 1500);       //Pixels
    sprocketDetectHeightSlider_->setValue(sprocketDetectHeight);
    sprocketDetectHeightSlider_->setStyleSheet("padding: 5px;");
    adjustmentLayout->addWidget(sprocketDetectHeightSlider_, 5, 0);

    adjustmentLayout->addWidget(createLabel("LEFT SPROCKET DETECT"), 0, 1);
    leftSprocketDetectSlider_ = new QSlider(Qt::Horizontal);
    leftSprocketDetectSlider_->setRange(0, 1500);
    leftSprocketDetectSlider_->setValue(leftSprocketEdge);
    leftSprocketDetectSlider_->setStyleSheet("padding: 5px;");
    adjustmentLayout->addWidget(leftSprocketDetectSlider_, 1, 1);

    adjustmentLayout->addWidget(createLabel("RIGHT SPROCKET DETECT"), 2, 1);
    rightSprocketDetectSlider_ = new QSlider(Qt::Horizontal);
    rightSprocketDetectSlider_->setRange(0, 1500);
    rightSprocketDetectSlider_->setValue(rightSprocketEdge);
    rightSprocketDetectSlider_->setStyleSheet("padding: 5px;");
    adjustmentLayout->addWidget(rightSprocketDetectSlider_, 3, 1);

    adjustmentLayout->addWidget(createLabel("LEFT SCAN EDGE"), 0, 2);
    leftScanEdgeSlider_ = new QSlider(Qt::Horizontal);
    leftScanEdgeSlider_->setRange(0, Config::PHOTO_WIDTH / 2);
    leftScanEdgeSlider_->setValue(leftScanEdge);
    leftScanEdgeSlider_->setStyleSheet("padding: 5px;");
    adjustmentLayout->addWidget(leftScanEdgeSlider_, 1, 2);

    adjustmentLayout->addWidget(createLabel("RIGHT SCAN EDGE"), 2, 2);
    rightScanEdgeSlider_ = new QSlider(Qt::Horizontal);
    rightScanEdgeSlider_->setRange(Config::PHOTO_WIDTH / 2, Config::PHOTO_WIDTH - 1);
    rightScanEdgeSlider_->setValue(rightScanEdge);
    rightScanEdgeSlider_->setStyleSheet("padding: 5px;");
    adjustmentLayout->addWidget(rightScanEdgeSlider_, 3, 2);

    adjustmentLayout->addWidget(createLabel("VERTICAL SCAN HEIGHT"), 4, 2);
    verticalScanHeightSlider_ = new QSlider(Qt::Horizontal);
    verticalScanHeightSlider_->setRange(Config::PHOTO_HEIGHT / 4, Config::PHOTO_HEIGHT - 1);                //Pixels 4k
    verticalScanHeightSlider_->setValue(verticalScanHeight);
    verticalScanHeightSlider_->setStyleSheet("padding: 5px;");
    adjustmentLayout->addWidget(verticalScanHeightSlider_, 5, 2);
    adjustmentLayout->addWidget(createLabel("       "), 4, 3);         //Right column spacer

    leftLayout->addLayout(adjustmentLayout);        //Put controls under video preview window on left
  
    

    // Right panel - Controls in a grid on a plane
    QGridLayout* rightLayout = new QGridLayout();
    rightLayout->setAlignment(Qt::AlignTop);
    rightLayout->setColumnStretch(0, 1);  // Left column gets equal width
    rightLayout->setColumnStretch(1, 1);  // Right column gets equal width

    //RIGHT COLUMN CONTROLS----------------------
    // Spool diameter
    rightLayout->addWidget(createLabel("Spool Diameter (mm):"), 0, 0);      //The label
    spoolDiameterEdit_ = new QLineEdit(QString::number(spoolDiameter_, 'f', 2));
    spoolDiameterEdit_->setStyleSheet("background-color: white; color: black; padding: 5px; font-size: 14px;");
    rightLayout->addWidget(spoolDiameterEdit_, 1, 0);       //The text
    rightLayout->addWidget(setSpoolDiameterButton, 2, 0);   //The set button

    // Hardware controls buttons
    rightLayout->addWidget(lightButton_, 3, 0);
    rightLayout->addWidget(fanButton_, 4, 0);
    rightLayout->addWidget(motorsButton_, 5, 0);

    // Film control buttons
    rightLayout->addWidget(fineButtonForward, 6, 0);
    rightLayout->addWidget(nudgeButtonForward, 7, 0);
    rightLayout->addWidget(advance1Button, 8, 0);
    rightLayout->addWidget(fineButtonBack, 9, 0);
    rightLayout->addWidget(nudgeButtonBack, 10, 0);
    rightLayout->addWidget(rewind1Button, 11, 0);
    rightLayout->addWidget(advance10Button, 12, 0);
    rightLayout->addWidget(advance50Button, 13, 0);
    rightLayout->addWidget(advance100Button, 14, 0);


    //RIGHT COLUMN CONTROLS----------------------

    //Modes and overall control buttons
    rightLayout->addWidget(focusButton_, 0, 1);
    rightLayout->addWidget(previewButton_, 1, 1);
    rightLayout->addWidget(setupButton_, 2, 1); 
    rightLayout->addWidget(scanButton_, 3, 1);
    rightLayout->addWidget(stopButton_, 4, 1);

    // Export folder
    rightLayout->addWidget(createLabel("Image Filename Prefix:"), 5, 1);      //The label
    exportPrefixEdit_ = new QLineEdit("image_");   //Default prefix
    exportPrefixEdit_->setStyleSheet("background-color: white; color: black; padding: 5px; font-size: 14px;");
    rightLayout->addWidget(exportPrefixEdit_, 6, 1);       //The text
    rightLayout->addWidget(setImagePrefixButton_, 7, 1);   //The set button
    rightLayout->addWidget(exportFolderButton_, 8, 1);     //Opens a file dialog

    // Add panels to main layout
    mainLayout->addLayout(leftLayout, 2);
    mainLayout->addLayout(rightLayout, 1);

}

void ScannerApp::connectSignals() {

    // Connect all the other sliders with validation
    connect(sprocketHeightSlider_, &QSlider::valueChanged, [this](int value) {
        sprocketMinHeight_ = value;
    });
    connect(sprocketDetectHeightSlider_, &QSlider::valueChanged, [this](int value) {
        sprocketDetectHeight = value;
    });
    connect(leftSprocketDetectSlider_, &QSlider::valueChanged, [this](int value) {
        leftSprocketEdge = value;
        if (leftSprocketEdge >= rightSprocketEdge) {
            leftSprocketEdge = rightSprocketEdge - 1;
            rightSprocketDetectSlider_->blockSignals(true);         //Mask control
            rightSprocketDetectSlider_->setValue(leftSprocketEdge);
            rightSprocketDetectSlider_->blockSignals(false);
        }
    });
    connect(rightSprocketDetectSlider_, &QSlider::valueChanged, [this](int value) {
        rightSprocketEdge = value;

        if (rightSprocketEdge <= leftSprocketEdge) {
            rightSprocketEdge = leftSprocketEdge + 1;
            rightSprocketDetectSlider_->blockSignals(true);
            rightSprocketDetectSlider_->setValue(rightSprocketEdge);
            rightSprocketDetectSlider_->blockSignals(false);
        }
    });
    connect(leftScanEdgeSlider_, &QSlider::valueChanged, [this](int value) {
        leftScanEdge = value;

        if (leftScanEdge <= rightSprocketEdge) {
            leftScanEdge = rightSprocketEdge + 1;
            leftScanEdgeSlider_->blockSignals(true);
            leftScanEdgeSlider_->setValue(leftScanEdge);
            leftScanEdgeSlider_->blockSignals(false);
        }

        if (leftScanEdge >= rightScanEdge) {
            leftScanEdge = rightScanEdge - 1;
            leftScanEdgeSlider_->blockSignals(true);
            leftScanEdgeSlider_->setValue(leftScanEdge);
            leftScanEdgeSlider_->blockSignals(false);
        }

    });
    connect(rightScanEdgeSlider_, &QSlider::valueChanged, [this](int value) {
        // Ensure right edge doesn't go below left edge
        if (value < leftScanEdge) {
            rightScanEdgeSlider_->blockSignals(true);
            rightScanEdgeSlider_->setValue(leftScanEdge);
            rightScanEdgeSlider_->blockSignals(false);
            rightScanEdge = leftScanEdge;
        } else {
            rightScanEdge = value;
        }
    });
    connect(verticalScanHeightSlider_, &QSlider::valueChanged, [this](int value) {
        verticalScanHeight = value;
    });
    
    //Film track controls

    connect(setSpoolDiameterButton, &QPushButton::clicked, [this]() {
        bool ok;
        float newDiameter = spoolDiameterEdit_->text().toFloat(&ok);
        if (ok && newDiameter > 0.0f) {
            spoolDiameter_ = newDiameter;
            spool_diameter_mm_ = newDiameter;
            computeMovePerFrame();
            skipFrameTarget = computeSkipFrameTarget(spool_diameter_mm_);
            std::cout << "Spool diameter set to " << spoolDiameter_ << " mm" << std::endl;
        } else {
            QMessageBox::warning(this, "Invalid Input", "Please enter a valid positive number for spool diameter.");
            spoolDiameterEdit_->setText(QString::number(spoolDiameter_, 'f', 2));
        }
    });

    connect(fineButtonForward, &QPushButton::clicked, [this]() {
        advanceFilmTracking(0.025f, 250.0f);
    });
    connect(nudgeButtonForward, &QPushButton::clicked, [this]() {
        advanceFilmTracking(0.25f, 250.0f);
    });
    connect(fineButtonBack, &QPushButton::clicked, [this]() {
        advanceFilmTracking(-0.025f, 250.0f);
    });
    connect(nudgeButtonBack, &QPushButton::clicked, [this]() {
        advanceFilmTracking(-0.25f, 250.0f);
    });
    connect(rewind1Button, &QPushButton::clicked, [this]() {
        advanceFilmTracking(-1.0f, 1000.0f); // Move the film forward by one frame
    });
    connect(advance1Button, &QPushButton::clicked, [this]() {
        advanceFilmTracking(1.0f, 1000.0f); // Move the film forward by one frame
    });
    connect(advance10Button, &QPushButton::clicked, [this]() {
        advanceFilmTracking(10.0f, 1000.0f); // Move the film forward by one frame
    });
    connect(advance50Button, &QPushButton::clicked, [this]() {
        advanceFilmTracking(50.0f, 2000.0f); // Move the film forward by one frame
    });
    connect(advance100Button, &QPushButton::clicked, [this]() {
        advanceFilmTracking(100.0f, 3000.0f); // Move the film forward by one frame
    });

    connect(scanButton_, &QPushButton::clicked, this, &ScannerApp::onStartScan);
    connect(stopButton_, &QPushButton::clicked, this, &ScannerApp::onStopScan);
    connect(setupButton_, &QPushButton::clicked, this, &ScannerApp::onSwitchToSetup);
    connect(previewButton_, &QPushButton::clicked, this, &ScannerApp::onSwitchToPreview);
    connect(lightButton_, &QPushButton::clicked, this, &ScannerApp::onLightToggle);
    connect(fanButton_, &QPushButton::clicked, this, &ScannerApp::onFanToggle);
    connect(focusButton_, &QPushButton::clicked, this, &ScannerApp::onFocusToggle);
    connect(motorsButton_, &QPushButton::clicked, this, &ScannerApp::onMotorsToggle);

    connect(setImagePrefixButton_, &QPushButton::clicked, [this]() {
        imageFilePrefix_ = exportPrefixEdit_->text().toStdString();
        std::cout << "Image file prefix set to: " << imageFilePrefix_ << std::endl;
    });

    connect(exportFolderButton_, &QPushButton::clicked, this, &ScannerApp::onSelectExportFolder);

    updateTimer_ = new QTimer(this);

    //Start the app loop
    connect(updateTimer_, &QTimer::timeout, this, &ScannerApp::updateFrame);

}

QLabel* ScannerApp::createLabel(const QString &text, int size) {
    QLabel *label = new QLabel(text);
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet(QString("color: white; font-size: %1px;").arg(size)); // Default style
    return label;
}

void ScannerApp::initializeHardware() {

    std::cout << "Initializing camera..." << std::endl;
    QApplication::processEvents();
    
    // Initialize camera on main thread (libcamera needs this)
    std::cout << "Setting camera parameters..." << std::endl;
    camera_.setVideoResolution(Config::VIDEO_WIDTH, Config::VIDEO_HEIGHT);
    camera_.setPhotoResolution(Config::PHOTO_WIDTH, Config::PHOTO_HEIGHT);
    camera_.setFramerate(Config::VIDEO_FPS);
    camera_.setWhiteBalance("auto");
    camera_.setDenoise(false);
    
    std::cout << "Initializing camera..." << std::endl;
    if (!camera_.initialize()) {
        QMessageBox::critical(this, "Error", "Failed to initialize camera");
        return;
    }
    
    QApplication::processEvents();
    
    std::cout << "Starting video stream..." << std::endl;
    if (!camera_.startVideo()) {
        QMessageBox::critical(this, "Error", "Failed to start video stream");
        return;
    }
    std::cout << "Video stream started successfully" << std::endl;
    
    loadSetupSettings();    

    // Start update timer immediately
    computeMovePerFrame();
    std::cout << "Starting update timer..." << std::endl;
    updateTimer_->start(33);  // ~30 FPS update
    std::cout << "Camera initialization complete!" << std::endl;

    // Initialize Marlin controller
    std::cout << "Initializing Marlin controller..." << std::endl;
    marlin_ = new MarlinController(Config::MARLIN_PORT, Config::MARLIN_BAUD);
    
    if (marlin_->connect()) {
        std::cout << "Marlin connected successfully" << std::endl;
    } else {
        std::cerr << "Warning: Failed to connect to Marlin" << std::endl;
        QMessageBox::warning(this, "Warning", 
            "Failed to connect to Marlin controller.\n"
            "Camera will work but motor control will be disabled.");
    }
    
    initializationComplete_ = true;
}

void ScannerApp::saveSetupSettings() {
    QJsonObject settings;
    settings["leftSprocketEdge"] = leftSprocketEdge;
    settings["rightSprocketEdge"] = rightSprocketEdge;
    settings["sprocketDetectHeight"] = sprocketDetectHeight;
    settings["leftScanEdge"] = leftScanEdge;
    settings["rightScanEdge"] = rightScanEdge;
    settings["verticalScanHeight"] = verticalScanHeight;
    settings["sprocketBrightnessThreshold"] = sprocketBrightnessThreshold_;
    settings["sprocketMinHeight"] = sprocketMinHeight_;
    
    QJsonDocument doc(settings);
    
    QString filePath = "scanner_settings.json";
    
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
        file.close();
        std::cout << "Settings saved to: " << filePath.toStdString() << std::endl;
    } else {
        std::cerr << "Failed to save settings to: " << filePath.toStdString() << std::endl;
    }
}

void ScannerApp::loadSetupSettings() {
    QString filePath = "scanner_settings.json";
    
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        std::cout << "No saved settings found, using defaults" << std::endl;
        return;
    }
    
    QByteArray data = file.readAll();
    file.close();
    
    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isObject()) {
        std::cerr << "Invalid settings file format" << std::endl;
        return;
    }
    
    QJsonObject settings = doc.object();
    
    if (settings.contains("leftSprocketEdge"))
        leftSprocketEdge = settings["leftSprocketEdge"].toInt();
    if (settings.contains("rightSprocketEdge"))
        rightSprocketEdge = settings["rightSprocketEdge"].toInt();
    if (settings.contains("sprocketDetectHeight"))
        sprocketDetectHeight = settings["sprocketDetectHeight"].toInt();
    if (settings.contains("leftScanEdge"))
        leftScanEdge = settings["leftScanEdge"].toInt();
    if (settings.contains("rightScanEdge"))
        rightScanEdge = settings["rightScanEdge"].toInt();
    if (settings.contains("verticalScanHeight"))
        verticalScanHeight = settings["verticalScanHeight"].toInt();
    if (settings.contains("sprocketBrightnessThreshold"))
        sprocketBrightnessThreshold_ = settings["sprocketBrightnessThreshold"].toInt();
    if (settings.contains("sprocketMinHeight"))
        sprocketMinHeight_ = settings["sprocketMinHeight"].toInt();
    
    // Update UI sliders to reflect loaded values
    leftSprocketDetectSlider_->setValue(leftSprocketEdge);
    rightSprocketDetectSlider_->setValue(rightSprocketEdge);
    sprocketDetectHeightSlider_->setValue(sprocketDetectHeight);
    leftScanEdgeSlider_->setValue(leftScanEdge);
    rightScanEdgeSlider_->setValue(rightScanEdge);
    verticalScanHeightSlider_->setValue(verticalScanHeight);
    sprocketBrightSlider_->setValue(sprocketBrightnessThreshold_);
    sprocketHeightSlider_->setValue(sprocketMinHeight_);
    
    std::cout << "Settings loaded from: " << filePath.toStdString() << std::endl;
}

void ScannerApp::drawText(cv::Mat& frame, std::string text, int x, int y, double fontScale) {

    //Draw thick outline, then thinner text on top
    cv::putText(frame, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 0, 0), 5);
    cv::putText(frame, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(255, 255, 255), 2);

}

void ScannerApp::updateFrame() {

    switch(currentState_) {

        case PREVIEW:
            if (camera_.getVideoFrame(frame)) {

                cv::flip(frame, frame, 1);                //Flip frame horizontally

                // Convert to 8-bit first (if need be)
                if (frame.depth() == CV_16U) {
                    frame.convertTo(currentFrame_, CV_8U, 1.0 / 256.0);
                }
                else {
                    currentFrame_ = frame.clone();
                }

                drawPreviewOverlay(currentFrame_);
                updateVideoDisplay(currentFrame_);
            }

            break;

        case SETUP:
            if (!setupFrame_.empty()) {                 //Base raw still exists?

                cv::Mat displayFrame = setupFrame_.clone();   //Copy backup for next overlay

                cv::flip(displayFrame, displayFrame, 1);    //Flip frame horizontally

                // Convert to 8-bit first (if need be), then draw overlay
                if (displayFrame.depth() == CV_16U) {
                    displayFrame.convertTo(displayFrame, CV_8U, 1.0 / 256.0);
                }
                else {
                    displayFrame = displayFrame.clone();
                }

                //detectSprocket(displayFrame);
                drawSetupOverlay(displayFrame);
                updateVideoDisplay(displayFrame);

            }

            break;

        case SCANNING:
            scanSingleFrame();
            break;

    }
   

}

void ScannerApp::updateVideoDisplay(const cv::Mat& frame) {

    if (frame.empty()) return;
    
    // Frame should already be 8-bit at this point
    if (frame.depth() != CV_8U || frame.channels() != 3) {
        std::cerr << "ERROR: Expected CV_8UC3, got depth=" << frame.depth() 
                  << " channels=" << frame.channels() << std::endl;
        return;
    }
    
    // Ensure continuous memory
    cv::Mat continuous = frame.isContinuous() ? frame : frame.clone();

    // Create QImage and copy row by row for proper alignment
    QImage qImage(continuous.cols, continuous.rows, QImage::Format_RGB888);

    for (int y = 0; y < continuous.rows; ++y) {
        memcpy(qImage.scanLine(y), continuous.ptr(y), continuous.cols * 3);
    }
    
    // Convert to pixmap and scale
    QPixmap pixmap = QPixmap::fromImage(qImage);

    if (focusZoomedIn_ == false) {          //Scaled to fit window
        pixmap = pixmap.scaled(videoLabel_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    }
    else {
        //Leave zoomied in and get the computeFocusScore
        focusScore = computeFocusScore(frame);
    }
    
    videoLabel_->setPixmap(pixmap);

}

void ScannerApp::computeMovePerFrame() {

    float framesPerTurn = (spool_diameter_mm_ * M_PI) / super8frameHeight; // Frames per rotation of the spool
    
    movement_mm_turn = 0.0f;
    movement_mm_turn_target = base_spool_diameter * M_PI; // mm per rotation of the spool

    mmPerFrame = movement_mm_turn_target / framesPerTurn; // mm per frame of film

}

void ScannerApp::drawPreviewOverlay(cv::Mat& frame) {

    if (frame.empty()) return;
    
    if (focusZoomedIn_ == true) {        // Display focus score and no crosshairs

        std::ostringstream oss;
        oss << "Focus Score: " << std::fixed << std::setprecision(2) << focusScore;

        drawText(frame, oss.str(), 375, 340 - 20, 2.0);
        drawText(frame, "HIGHER IS BETTER", 375, 340 + 20, 1.0);

        return;

    }

    int centerX = frame.cols / 2;
    int centerY = frame.rows / 2;
    
    // Draw center crosshair
    cv::line(frame, cv::Point(0, centerY), cv::Point(frame.cols, centerY), 
             cv::Scalar(0, 0, 0), 5);
    cv::line(frame, cv::Point(centerX, centerY - 250), cv::Point(centerX, centerY + 250), 
             cv::Scalar(0, 0, 0), 5);
    cv::line(frame, cv::Point(0, centerY), cv::Point(frame.cols, centerY), 
             cv::Scalar(255, 255, 255), 2);
    cv::line(frame, cv::Point(centerX, centerY - 250), cv::Point(centerX, centerY + 250), 
             cv::Scalar(255, 255, 255), 2);
        
    drawText(frame, "PREVIEW", 10, 30, 2);

}

float ScannerApp::computeFocusScore(const cv::Mat &image) {
    // Extract a strip of pixels from the center of the image
    int stripHeight = 100; // Height of the strip
    int centerY = image.rows / 2;
    cv::Rect centerStrip(0, centerY - stripHeight / 2, image.cols, stripHeight);
    cv::Mat strip = image(centerStrip);

    // Convert the strip to grayscale if it's not already
    cv::Mat grayStrip;
    if (strip.channels() == 3) {
        cv::cvtColor(strip, grayStrip, cv::COLOR_BGR2GRAY);
    } else {
        grayStrip = strip;
    }

    // Apply the Laplacian operator to detect edges
    cv::Mat laplacian;
    cv::Laplacian(grayStrip, laplacian, CV_64F);

    // Compute the variance of the Laplacian
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    double variance = stddev[0] * stddev[0];

    // Return the focus score (lower is better focus)
    return static_cast<float>(variance);

}

void ScannerApp::drawSetupOverlay(cv::Mat& frame) {
    
    if (frame.empty()) {
        return;
    }

    int centerY = frame.rows / 2;

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // Ensure 8-bit format (photos may be 16-bit)
    if (gray.depth() != CV_8U) {
        gray.convertTo(gray, CV_8U, 1.0 / 256.0);
    }
    
    // Define region of interest for sprocket detection using user-defined bounds
    int roiX = leftSprocketEdge;
    int roiY = centerY - (sprocketDetectHeight / 2);
    int roiWidth = rightSprocketEdge - leftSprocketEdge;
    int roiHeight = sprocketDetectHeight;
    
    if (roiX < 0 || roiY < 0 || roiX + roiWidth > frame.cols || roiY + roiHeight > frame.rows) {
        return;
    }
    
    cv::Rect roi(roiX, roiY, roiWidth, roiHeight);
    cv::Mat roiGray = gray(roi);
    
    // Threshold
    cv::Mat binary;
    cv::threshold(roiGray, binary, sprocketBrightnessThreshold_, 255, cv::THRESH_BINARY);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    sprocketDetected_ = false;
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 100 && area < 5000) {  // Adjust based on your setup
            cv::Moments m = cv::moments(contour);
            if (m.m00 > 0) {
                int cx = static_cast<int>(m.m10 / m.m00) + roiX;
                int cy = static_cast<int>(m.m01 / m.m00) + roiY;
                sprocketCenter_ = cv::Point(cx, cy);
                sprocketDetected_ = true;
                break;
            }
        }
    }

    // Draw frame scan guide
    cv::Rect frameRect(leftScanEdge, centerY - (verticalScanHeight / 2), 
                      rightScanEdge - leftScanEdge, verticalScanHeight);
    cv::rectangle(frame, frameRect, cv::Scalar(0, 255, 255), 8);
    

    drawText(frame, "SPROCKET DETECTION AREA", 20, centerY - (sprocketDetectHeight / 2) - 20, 1.0);

    cv::line(frame, cv:Point(0, centerY - (sprocketDetectHeight / 2)), 
             cv::Point(frame.cols, centerY - (sprocketDetectHeight / 2)), 
             cv::Scalar(255, 255, 255), 2);

    cv::line(frame, cv::Point(0, centerY + (sprocketDetectHeight / 2)), 
             cv::Point(frame.cols, centerY + (sprocketDetectHeight / 2)), 
             cv::Scalar(255, 255, 255), 2);

    
    // Draw detected sprocket
    if (sprocketDetected_) {
        cv::circle(frame, sprocketCenter_, 10, cv::Scalar(0, 0, 255), -1);
        cv::putText(frame, "SPROCKET", cv::Point(sprocketCenter_.x - 40, sprocketCenter_.y - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    }

    //Draw vertical lines
    cv::line(frame, cv::Point(leftSprocketEdge, 0), cv::Point(leftSprocketEdge, frame.rows), 
             cv::Scalar(0, 0, 255), 8);
    cv::line(frame, cv::Point(rightSprocketEdge, 0), cv::Point(rightSprocketEdge, frame.rows), 
             cv::Scalar(255, 0, 0), 8);
    cv::line(frame, cv::Point(leftScanEdge, 0), cv::Point(leftScanEdge, frame.rows), 
             cv::Scalar(0, 255, 255), 8);
    cv::line(frame, cv::Point(rightScanEdge, 0), cv::Point(rightScanEdge, frame.rows), 
             cv::Scalar(0, 255, 255), 8);

    // Draw overlay specific to setup mode
    cv::putText(frame, "SETUP", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                5.0, cv::Scalar(0, 255, 0), 2);

}

void ScannerApp::scanSingleFrame() {
    if (!scanning_) return;
    
    // Capture high-resolution still
    cv::Mat photo;
    if (!camera_.capturePhoto(photo)) {
        std::cerr << "Failed to capture photo" << std::endl;
        return;
    }
    
    // Process and save
    std::ostringstream oss;
    oss << exportFolder_ << "/" << imageFilePrefix_ 
        << std::setfill('0') << std::setw(5) << frameCount_ << ".png";
    std::string filename = oss.str();
    cv::imwrite(filename, photo);
    
    frameCount_++;
    //TODO: REDRAW FRAMES HERE
    
    std::cout << "Saved: " << filename << std::endl;
    
    // Advance film
    computeMovePerFrame();
    //advanceFilm(movePerFrame_, true);
    

}

void ScannerApp::advanceFilmTracking(float frames, float speed, bool waitForResponse) {

    float takeupMoveMM = frames * mmPerFrame;                               // Convert frames to mm using the mm per frame value

    if (currentState_ == SCANNING) {

        if (skipFrameTarget > 0) {                      //If enabled (!0) skip a takeup frame every x frames
            if (++skipFrameCount >= skipFrameTarget) {      //If enabled skip a takeup frame every x frames
                skipFrameCount = 0;                         // Goal here is to have some slack in the takeup spool
                takeupMoveMM = 0.0f;          //Send a zero for takeup movement this frame
            }
        }

    }

    marlin_->advanceFilm2(takeupMoveMM, frames, speed, waitForResponse);     // Advance the film by mm
    movement_mm_turn += takeupMoveMM;                                       // Update the total movement tracker

    if (movement_mm_turn < 0) {     //Backwards and went past a rotation edge?
        float leftover = 0 - movement_mm_turn;          //Save delta past 0
        spool_diameter_mm_ -= (film_thickness_mm * 2);   // Increase the spool diameter by twice the film thickness
        computeMovePerFrame();       // Recompute the movement per frame based on diameter
        skipFrameTarget = computeSkipFrameTarget(spool_diameter_mm_); // Recompute the skip frame target based on the new spool diameter
        movement_mm_turn = movement_mm_turn_target - leftover;    // Reset the movement to the leftover value
        return;
    }

    if (movement_mm_turn >= movement_mm_turn_target) {      //One rotation?
        float leftover = movement_mm_turn - movement_mm_turn_target;  //Save this
        spool_diameter_mm_ += (film_thickness_mm * 2);   // Increase the spool diameter by twice the film thickness
        computeMovePerFrame();       // Recompute the movement per frame based on diameter
        movement_mm_turn = leftover;    // Reset the movement to the leftover value
        skipFrameTarget = computeSkipFrameTarget(spool_diameter_mm_); // Recompute the skip frame target based on the new spool diameter

        std::cout << "Skip frame target: " << skipFrameTarget << " frames" << std::endl;

        return;
    }

}

int ScannerApp::computeSkipFrameTarget(float spool_diameter_mm) {
    return int(0.266f * spool_diameter_mm + 41.5f + 0.5f); // Add 0.5 for rounding
}

// Slot implementations
void ScannerApp::onStartScan() {

    saveSetupSettings();

    computeMovePerFrame();
    scanning_ = true;
    currentState_ = SCANNING;
    frameCount_ = 0;
    
    scanButton_->setEnabled(false);
    stopButton_->setEnabled(true);
   
    std::cout << "Scan started" << std::endl;

}

void ScannerApp::onStopScan() {
    scanning_ = false;
    currentState_ = PREVIEW;
    
    scanButton_->setEnabled(true);
    stopButton_->setEnabled(false);
    
    std::cout << "Scan stopped. Captured " << frameCount_ << " frames." << std::endl;

}

void ScannerApp::onSwitchToSetup() {
    camera_.stopVideo();
    
    // Give camera time to fully stop, then reconfigure for 4K photos
    QTimer::singleShot(200, this, [this]() {

        std::cout << "Switching to setup mode..." << std::endl;
        
        // Shutdown and reinitialize for photo mode
        // camera_.shutdown();
        // camera_.setVideoResolution(Config::PHOTO_WIDTH, Config::PHOTO_HEIGHT);
        // camera_.setPhotoResolution(Config::PHOTO_WIDTH, Config::PHOTO_HEIGHT);
        
        // if (!camera_.initialize()) {
        //     statusLabel_->setText("Status: Failed to initialize 4K mode");
        //     return;
        // }
        
        // Capture a single photo for setup mode
        if (camera_.capturePhoto(frame)) {
            setupFrame_ = frame.clone();
            std::cout << "Setup photo captured" << std::endl;
        } else {
            std::cerr << "Failed to capture setup photo" << std::endl;
        }
        
        currentState_ = SETUP;
    });

}

void ScannerApp::onSwitchToPreview() {

    saveSetupSettings();

    std::cout << "Switching to preview mode..." << std::endl;
    
    // Shutdown and reinitialize for video mode
    // camera_.shutdown();
    // camera_.setVideoResolution(Config::VIDEO_WIDTH, Config::VIDEO_HEIGHT);
    // camera_.setPhotoResolution(Config::PHOTO_WIDTH, Config::PHOTO_HEIGHT);
    
    // if (!camera_.initialize()) {
    //     statusLabel_->setText("Status: Failed to initialize video mode");
    //     return;
    // }
    
    if (!camera_.startVideo()) {
        std::cerr << "Failed to start video stream" << std::endl;
        return;
    }
    
    currentState_ = PREVIEW;
    std::cout << "Switched to preview mode" << std::endl;

}

void ScannerApp::onFocusToggle() {

    if (currentState_ != PREVIEW) return;

    focusZoomedIn_ = !focusZoomedIn_;
    focusButton_->setText(focusZoomedIn_ ? "Focus On" : "Focus Off");

}

void ScannerApp::onLightToggle() {
    lightOn_ = !lightOn_;
    if (marlin_) {
        marlin_->setLight(lightOn_);
        lightButton_->setText(lightOn_ ? "Light: ON" : "Light: OFF");
    }
}

void ScannerApp::onFanToggle() {
    fanOn_ = !fanOn_;
    if (marlin_) {
        marlin_->setFan(fanOn_);
        fanButton_->setText(fanOn_ ? "Fan: ON" : "Fan: OFF");
    }
}

void ScannerApp::onMotorsToggle() {
    motorsEnabled_ = !motorsEnabled_;
    if (marlin_) {
        if (motorsEnabled_) {
            marlin_->enableMotors();
            motorsButton_->setText("Motors: ON");
        } else {
            marlin_->disableMotors();
            motorsButton_->setText("Motors: OFF");
        }
    }
}

void ScannerApp::onSelectExportFolder() {
    QString folder = QFileDialog::getExistingDirectory(this, "Select Export Folder", "",
                                                         QFileDialog::DontUseNativeDialog);
    if (!folder.isEmpty()) {
        exportFolder_ = folder.toStdString();
        QMessageBox::information(this, "Export Folder", 
            QString("Export folder set to:\n%1").arg(folder));
    }
}

void ScannerApp::onSprocketThresholdChanged(int value) {
    sprocketBrightnessThreshold_ = value;
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    
    // Set up a palette to ensure dialogs are readable
    QPalette palette;
    palette.setColor(QPalette::Window, QColor(240, 240, 240));
    palette.setColor(QPalette::WindowText, Qt::black);
    palette.setColor(QPalette::Base, Qt::white);
    palette.setColor(QPalette::AlternateBase, QColor(233, 233, 233));
    palette.setColor(QPalette::Text, Qt::black);
    palette.setColor(QPalette::Button, QColor(240, 240, 240));
    palette.setColor(QPalette::ButtonText, Qt::black);
    app.setPalette(palette);
    
    ScannerApp scanner;
    scanner.show();
    
    return app.exec();
}

#include "main.moc"
