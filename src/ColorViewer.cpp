#include "window.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>

int main(int argc, char **argv) try {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    // Configure which streams to enable or disable for the Pipeline by creating a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR);

    // Start the pipeline with config
    pipe.start(config);
    auto currentProfile = pipe.getEnabledStreamProfileList()->getProfile(0)->as<ob::VideoStreamProfile>();

    // Create a window for rendering, and set the resolution of the window
    Window app("QR Code Viewer with Metadata", currentProfile->width(), currentProfile->height());

    // Initialize OpenCV QR code detector
    cv::QRCodeDetector qrDetector;

    while(app) {
        // Wait for up to 100ms for a frameset in blocking mode
        auto frameSet = pipe.waitForFrames(100);
        if (frameSet == nullptr) {
            continue; // No frameset available, try again
        }
        
        // Get color frame from frameset
        auto colorFrame = frameSet->colorFrame();
        if (colorFrame == nullptr || !colorFrame->data()) {
            continue; // Invalid or empty colorFrame, skip this iteration
        }

        // Check dimensions
        int height = colorFrame->height();
        int width = colorFrame->width();
        if (height <= 0 || width <= 0) {
            std::cerr << "Invalid dimensions: Height=" << height << ", Width=" << width << std::endl;
            continue;
        }

        // Convert color frame to OpenCV Mat
    
        // colorFrame type
        //std::cout << "Color Frame Type: " << colorFrame->type() << std::endl;

        // colorFrame format
        //std::cout << "Color Frame Format: " << colorFrame->format() << std::endl;
        
        // Get the MJPEG data from the color frame
        std::vector<uint8_t> mjpegData((uint8_t*)colorFrame->data(), 
                                    (uint8_t*)colorFrame->data() + colorFrame->dataSize());

        // Decode MJPEG into an OpenCV Mat
        cv::Mat frame = cv::imdecode(mjpegData, cv::IMREAD_COLOR);
        if (frame.empty()) {
            std::cerr << "Failed to decode MJPEG frame!" << std::endl;
            continue;
        }

        // Convert to grayscale for QR detection
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        if (frame.empty()) {
            std::cerr << "Failed to create cv::Mat from colorFrame!" << std::endl;
            continue;
        }

        // Detect and decode QR codes
        cv::Mat points;
        std::string qrData = qrDetector.detectAndDecode(gray, points);
	
	    // Print detected QR data
        if (!qrData.empty()) {
             std::cout << "Detected QR code: " << qrData << std::endl;
        }

        // Render the modified frame in the window
        app.addToRender(colorFrame);
    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
