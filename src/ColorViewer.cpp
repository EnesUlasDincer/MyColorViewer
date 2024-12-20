// #include "window.hpp"

// #include "libobsensor/hpp/Pipeline.hpp"
// #include "libobsensor/hpp/Error.hpp"

// const char *metaDataTypes[] = { "TIMESTAMP",
//                                 "SENSOR_TIMESTAMP",
//                                 "FRAME_NUMBER",
//                                 "AUTO_EXPOSURE",
//                                 "EXPOSURE",
//                                 "GAIN",
//                                 "AUTO_WHITE_BALANCE",
//                                 "WHITE_BALANCE",
//                                 "BRIGHTNESS",
//                                 "CONTRAST",
//                                 "SATURATION",
//                                 "SHARPNESS",
//                                 "BACKLIGHT_COMPENSATION",
//                                 "HUE",
//                                 "GAMMA",
//                                 "POWER_LINE_FREQUENCY",
//                                 "LOW_LIGHT_COMPENSATION",
//                                 "MANUAL_WHITE_BALANCE",
//                                 "ACTUAL_FRAME_RATE",
//                                 "FRAME_RATE",
//                                 "AE_ROI_LEFT",
//                                 "AE_ROI_TOP",
//                                 "AE_ROI_RIGHT",
//                                 "AE_ROI_BOTTOM",
//                                 "EXPOSURE_PRIORITY",
//                                 "HDR_SEQUENCE_NAME",
//                                 "HDR_SEQUENCE_SIZE",
//                                 "HDR_SEQUENCE_INDEX",
//                                 "LASER_POWER",
//                                 "LASER_POWER_LEVEL",
//                                 "LASER_STATUS",
//                                 "GPIO_INPUT_DATA" };


// int main(int argc, char **argv) try {
//     // Create a pipeline with default device
//     ob::Pipeline pipe;
    
//     // Configure which streams to enable or disable for the Pipeline by creating a Config
//     std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
//     config->enableVideoStream(OB_STREAM_COLOR);
    
//     // Start the pipeline with config
//     pipe.start(config);
//     auto currentProfile = pipe.getEnabledStreamProfileList()->getProfile(0)->as<ob::VideoStreamProfile>();
//     // Create a window for rendering, and set the resolution of the window
//     Window app("ColorViewer", currentProfile->width(), currentProfile->height());
//     while(app) {
//         // Wait for up to 100ms for a frameset in blocking mode.
//         auto frameSet = pipe.waitForFrames(100);
//         if(frameSet == nullptr) {
//             continue;
//         }
        
//         // get color frame from frameset
//         auto colorFrame = frameSet->colorFrame();
//         if(colorFrame == nullptr) {
//             continue;
//         }

//         // print metadata every 30 frames
//         auto index = colorFrame->index();
//         if(index % 30 == 0) {
//             std::cout << "*************************** Color Frame #" << index << " Metadata List ********************************" << std::endl;
//             for(int metaDataType = 0; metaDataType < OB_FRAME_METADATA_TYPE_COUNT; metaDataType++) {
//                 // Check if it is supported metaDataType for current frame
//                 if(colorFrame->hasMetadata((OBFrameMetadataType)metaDataType)) {
//                     // Get the value of the metadata
//                     std::cout << metaDataTypes[metaDataType] << ": " << colorFrame->getMetadataValue((OBFrameMetadataType)metaDataType) << std::endl;
//                 }
//                 else {
//                     std::cout << metaDataTypes[metaDataType] << ": "
//                               << "unsupported" << std::endl;
//                 }
//             }
//             std::cout << "********************************************************************************" << std::endl << std::endl;
//         }

//         // Render frameset in the window, only color frames are rendered here.
//         app.addToRender(colorFrame);
//     }

//     // Stop the Pipeline, no frame data will be generated
//     pipe.stop();

//     return 0;
// }
// catch(ob::Error &e) {
//     std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
//     exit(EXIT_FAILURE);
// }

#include "window.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>

const char *metaDataTypes[] = { "TIMESTAMP",
                                "SENSOR_TIMESTAMP",
                                "FRAME_NUMBER",
                                "AUTO_EXPOSURE",
                                "EXPOSURE",
                                "GAIN",
                                "AUTO_WHITE_BALANCE",
                                "WHITE_BALANCE",
                                "BRIGHTNESS",
                                "CONTRAST",
                                "SATURATION",
                                "SHARPNESS",
                                "BACKLIGHT_COMPENSATION",
                                "HUE",
                                "GAMMA",
                                "POWER_LINE_FREQUENCY",
                                "LOW_LIGHT_COMPENSATION",
                                "MANUAL_WHITE_BALANCE",
                                "ACTUAL_FRAME_RATE",
                                "FRAME_RATE",
                                "AE_ROI_LEFT",
                                "AE_ROI_TOP",
                                "AE_ROI_RIGHT",
                                "AE_ROI_BOTTOM",
                                "EXPOSURE_PRIORITY",
                                "HDR_SEQUENCE_NAME",
                                "HDR_SEQUENCE_SIZE",
                                "HDR_SEQUENCE_INDEX",
                                "LASER_POWER",
                                "LASER_POWER_LEVEL",
                                "LASER_STATUS",
                                "GPIO_INPUT_DATA" };


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

        //cv::Mat frame(height, width, CV_8U, colorFrame->data());
        // cv::Mat frame(height, width, CV_8UC3, colorFrame->data());
        
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
        
        // // Get frame data and properties
        // void* rawData = colorFrame->data();
        // uint32_t width2 = colorFrame->width();
        // uint32_t height2 = colorFrame->height();
        // uint8_t* pixelData = static_cast<uint8_t*>(rawData); // Assume CV_8UC3 (BGR image)

        // if (!pixelData) {
        //     std::cerr << "Frame data is null." << std::endl;
        //     continue;
        // }

        // // Display pixel data
        // for (uint32_t y = 0; y < height2; ++y) {
        //     for (uint32_t x = 0; x < width2; ++x) {
        //         uint8_t b = pixelData[(y * width2 + x) * 3 + 0]; // Blue channel
        //         uint8_t g = pixelData[(y * width2 + x) * 3 + 1]; // Green channel
        //         uint8_t r = pixelData[(y * width2 + x) * 3 + 2]; // Red channel

        //         std::cout << "Pixel (" << x << ", " << y << "): "
        //                 << "B=" << (int)b << ", G=" << (int)g << ", R=" << (int)r << std::endl;
        //     }
        // }

        if (frame.empty()) {
            std::cerr << "Failed to create cv::Mat from colorFrame!" << std::endl;
            continue;
        }

        // Detect and decode QR codes
        cv::Mat points;
        //std::string qrData = qrDetector.detectAndDecode(frame, points);
        std::string qrData = qrDetector.detectAndDecode(gray, points);
        //std::string qrData = qrDetector.detectAndDecode(frame, points);
        //std::string qrData = qrDetector.detectAndDecode(bgrFrame);
	
	    // Print detected QR data
        if (!qrData.empty()) {
             std::cout << "Detected QR code: " << qrData << std::endl;
        }

        // if(!qrData.empty()) {
        //     std::cout << "Detected QR Code: " << qrData << std::endl;

        //     // Draw bounding box around QR code
        //     if(points.size() == 4) {
        //         for(size_t i = 0; i < points.size(); i++) {
        //             cv::line(frame, points[i], points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        //         }

        //         // Calculate orientation angle
        //         double angle = atan2(points[1].y - points[0].y, points[1].x - points[0].x) * 180.0 / CV_PI;
        //         std::cout << "QR Code Orientation: " << angle << " degrees" << std::endl;

        //         // Mark corners for reference
        //         cv::circle(frame, points[0], 5, cv::Scalar(0, 0, 255), -1); // Top-left
        //         cv::circle(frame, points[1], 5, cv::Scalar(255, 0, 0), -1); // Top-right
        //     }
        // }

        // get color frame from frameset
        // auto colorFrame = frameSet->colorFrame();

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
