#include "window.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/aruco.hpp>

// Global queue to store the last 50 transformation matrices
std::deque<cv::Mat> transformationQueue;
int POSE_counter = 50;

void addTransformationMatrix(const cv::Mat& transformationMatrix) {
    // Add the transformation matrix to the queue
    transformationQueue.push_back(transformationMatrix);

    // Keep only the last 50 matrices
    if (transformationQueue.size() > POSE_counter) {
        transformationQueue.pop_front();
    }
}

// Function to calculate the mean transformation matrix
cv::Mat calculateMeanTransformation() {
    if (transformationQueue.empty()) {
        std::cerr << "No transformation matrices available!" << std::endl;
        return cv::Mat();
    }

    cv::Mat meanMatrix = cv::Mat::zeros(4, 4, CV_64F);

    for (const auto& matrix : transformationQueue) {
        meanMatrix += matrix;
    }

    meanMatrix /= static_cast<double>(transformationQueue.size());
    return meanMatrix;
}


void estimatePose(cv::Mat &frame, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, 
                  const std::vector<cv::Point2f> &imagePoints, float qrWidth, float qrHeight, int cameraChoice) {
    // Define the 3D points of the QR code in the QR code's coordinate system
    std::vector<cv::Point3f> objectPoints = {
        {0, 0, 0},                    // Top-left
        {qrWidth, 0, 0},              // Top-right
        {qrWidth, qrHeight, 0},       // Bottom-right
        {0, qrHeight, 0}              // Bottom-left
    };

    // Output rotation and translation vectors
    cv::Mat rvec, tvec;

    // Solve the PnP problem to find the pose
    bool success = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    if (success) {
        std::cout << "Pose estimation successful!" << std::endl;

        // Draw the coordinate axes on the frame
        std::vector<cv::Point3f> axisPoints = {
            {qrWidth, 0, 0},           // X-axis endpoint
            {0, qrHeight, 0},          // Y-axis endpoint
            {0, 0, -qrWidth}           // Z-axis endpoint
        };

        std::vector<cv::Point2f> imageAxisPoints;
        cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imageAxisPoints);

        // Draw the axes on the frame
        cv::line(frame, imagePoints[0], imageAxisPoints[0], cv::Scalar(0, 0, 255), 2); // X-axis in red
        cv::line(frame, imagePoints[0], imageAxisPoints[1], cv::Scalar(0, 255, 0), 2); // Y-axis in green
        cv::line(frame, imagePoints[0], imageAxisPoints[2], cv::Scalar(255, 0, 0), 2); // Z-axis in blue

        // Display rotation and translation vectors
        std::cout << "Rotation Vector: " << rvec.t() << std::endl;
        std::cout << "Translation Vector: " << tvec.t() << std::endl;

        // Convert the rotation vector to a rotation matrix
        cv::Mat rotationMatrix;
        cv::Rodrigues(rvec, rotationMatrix);

        // Create the transformation matrix
        cv::Mat transformationMatrix = cv::Mat::eye(4, 4, CV_64F);
        rotationMatrix.copyTo(transformationMatrix(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(transformationMatrix(cv::Rect(3, 0, 1, 3)));

        // Store the transformation matrix
        addTransformationMatrix(transformationMatrix);

        // Calculate the mean transformation matrix
        cv::Mat meanMatrix = calculateMeanTransformation();

        std::cout << "Mean Transformation Matrix:" << std::endl
                  << meanMatrix << std::endl;

        // if transformationQueue is full, save the mean transformation matrix to a file
        if (transformationQueue.size() == POSE_counter) {
            std::string filename = "Cam_" + std::to_string(cameraChoice) + "_" + "pose_estimation" + ".txt";

            std::ofstream outFile(filename);

            if (outFile.is_open()) {
                outFile << "Mean Transformation Matrix:\n";
                for (int i = 0; i < meanMatrix.rows; ++i) {
                    for (int j = 0; j < meanMatrix.cols; ++j) {
                        outFile << meanMatrix.at<double>(i, j);
                        if (j < meanMatrix.cols - 1) {
                            outFile << " ";
                        }
                    }
                    outFile << "\n";
                }
                    outFile.close();
                    std::cout << "Mean transformation matrix saved to " << filename << std::endl;
                } else {
                    std::cerr << "Failed to open file for writing: " << filename << std::endl;
                }
        }
    } else {
        std::cerr << "Pose estimation failed!" << std::endl;
    }
}


int main(int argc, char **argv) try {

    ob::Context context;

    // Query the list of connected devices
    auto devList = context.queryDeviceList();

    // ask user to choose the camera
    std::cout << "Choose the camera (1 or 2): ";
    int cameraChoice;
    std::cin >> cameraChoice;

    //auto dev = devList->getDevice(cameraChoice);
    std::shared_ptr<ob::Device> dev;

    // check the camera serial number if it is CL8A8420179 or CL8A84201GW 
    // select the correct device
    // Get the number of connected devices
    int devCount = devList->deviceCount();
    for (int i = 0; i < devCount; i++) {
        // if devList->serialNumber(i) == "CL8A8420179" or "CL8A84201GW"
        // select the device
        if (strcmp(devList->serialNumber(i), "CL8A8420179") == 0 && cameraChoice == 1) {
            dev = devList->getDevice(i);
        } else if (strcmp(devList->serialNumber(i), "CL8A84201GW") == 0 && cameraChoice == 2) {
            dev = devList->getDevice(i);
        }
    }


    // Create a pipeline with default device
    ob::Pipeline pipe(dev);

    // Configure which streams to enable or disable for the Pipeline by creating a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR);

    // Start the pipeline with config
    pipe.start(config);
    auto currentProfile = pipe.getEnabledStreamProfileList()->getProfile(0)->as<ob::VideoStreamProfile>();

    // Create a window for rendering, and set the resolution of the window
    //Window app("QR Code Viewer with Metadata", currentProfile->width(), currentProfile->height());

    // chose the camera matrix and distortion coefficients
    // ask input from user
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    
    // Camera parameters (replace with actual calibration data)
    // Define the camera matrix and distortion coefficients
    cv::Mat camera_1_Matrix = (cv::Mat_<double>(3, 3) << 
        1123.55, 0, 951.093,
        0, 1122.8, 537.485,
        0, 0, 1);

    cv::Mat dist_1_Coeffs = (cv::Mat_<double>(1, 8) << 
        0.0737156, -0.10446, 0.043283, 0, 0, 0, -0.000494045, -0.000249066);

    //// CAM 1
    // Camera calibration data 
    // Serial Number: CL8A8420179
    // depthIntrinsic fx:1123.55, fy1122.8, cx:951.093, cy:537.485 ,width:1920, height:1080
    // rgbIntrinsic fx:1123.55, fy1122.8, cx:951.093, cy:537.485, width:1920, height:1080
    // depthDistortion k1:0.0737156, k2:-0.10446, k3:0.043283, k4:0, k5:0, k6:0, p1:-0.000494045, p2:-0.000249066
    // rgbDistortion k1:0.0737156, k2:-0.10446, k3:0.043283, k4:0, k5:0, k6:0, p1:-0.000494045, p2:-0.000249066
    // transform-rot: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    // transform-trans: [ 0,  0,  0]
    ////

    cv::Mat camera_2_Matrix = (cv::Mat_<double>(3, 3) << 
        1118.76, 0, 965.197,
        0, 1118.42, 544.13,
        0, 0, 1);

    cv::Mat dist_2_Coeffs = (cv::Mat_<double>(1, 8) << 
        0.0764899, -0.105664, 0.0430727, 0, 0, 0, -0.000198552, -0.000445639);

    //// CAM 2
    // Camera calibration data
    // Serial Number: CL8A84201GW
    // depthIntrinsic fx:1118.76, fy1118.42, cx:965.197, cy:544.13 ,width:1920, height:1080
    // rgbIntrinsic fx:1118.76, fy1118.42, cx:965.197, cy:544.13, width:1920, height:1080
    // depthDistortion k1:0.0764899, k2:-0.105664, k3:0.0430727, k4:0, k5:0, k6:0, p1:-0.000198552, p2:0.000445639
    // rgbDistortion k1:0.0764899, k2:-0.105664, k3:0.0430727, k4:0, k5:0, k6:0, p1:-0.000198552, p2:0.000445639
    // transform-rot: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    // transform-trans: [ 0,  0,  0]
    ////

    if (cameraChoice == 1) {
        cameraMatrix = camera_1_Matrix;
        distCoeffs = dist_1_Coeffs;
    } else if (cameraChoice == 2) {
        cameraMatrix = camera_2_Matrix;
        distCoeffs = dist_2_Coeffs;
    } else {
        std::cerr << "Invalid camera choice!" << std::endl;
        return 1;
    }


    float qrWidth = 0.1f;  // 10 cm
    float qrHeight = 0.1f; // 10 cm


    // Initialize OpenCV QR code detector
    cv::QRCodeDetector qrDetector;
    int counterQR = 0;
    while(true) {
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
            counterQR++;
            std::cout << "Detected QR code: " << qrData << " counter: " << counterQR << std::endl;

            if (!points.empty() && points.rows == 1 && points.cols == 4) {
                // Extract the 4 corner points from `points`
                cv::Point2f p1(points.at<float>(0, 0), points.at<float>(0, 1)); // Top-left
                cv::Point2f p2(points.at<float>(0, 2), points.at<float>(0, 3)); // Top-right
                cv::Point2f p3(points.at<float>(0, 4), points.at<float>(0, 5)); // Bottom-right
                cv::Point2f p4(points.at<float>(0, 6), points.at<float>(0, 7)); // Bottom-left

                // Draw the rectangle by connecting the points
                cv::line(frame, p1, p2, cv::Scalar(0, 255, 0), 2);
                cv::line(frame, p2, p3, cv::Scalar(0, 255, 0), 2);
                cv::line(frame, p3, p4, cv::Scalar(0, 255, 0), 2);
                cv::line(frame, p4, p1, cv::Scalar(0, 255, 0), 2);

                std::cout << "QR code bounding box drawn!" << std::endl;

                // Populate imagePoints with the extracted points
                std::vector<cv::Point2f> imagePoints = {p1, p2, p3, p4};

                // Now you can pass imagePoints to the pose estimation function
                estimatePose(frame, cameraMatrix, distCoeffs, imagePoints, qrWidth, qrHeight, cameraChoice);

            } else {
                std::cerr << "Invalid points matrix for bounding box!" << std::endl;
            }

            

            
        }
        

        // Render the modified frame in the window
        //app.addToRender(colorFrame);        
        //app.waitKey(10);

        // Render the modified frame in the window (use OpenCV frame directly)
        cv::imshow("QR Code Viewer with Metadata", frame);
        if (cv::waitKey(10) >= 27) break;

    }

    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
