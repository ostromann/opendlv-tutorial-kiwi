/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <math.h>
#include <chrono>


// using namespace cv;


cv::Mat dilate_erode(cv::Mat img, uint32_t ndilate, uint32_t nerode) {
    cv::Mat dilate;
    cv::dilate(img, dilate, cv::Mat(), cv::Point(-1,-1), ndilate, 1,1);
    cv::Mat erode;
    cv::erode(dilate, erode, cv::Mat(), cv::Point(-1,-1), nerode, 1, 1);

    return erode;
}

cv::Mat drawContours(cv::Mat img, std::vector<std::vector<cv::Point>> contours, cv::Scalar color, float MIN_CIRCLE_SIZE, float MAX_CIRCLE_SIZE) {
    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<cv::Point2f>centers(contours.size());
    std::vector<float>radius(contours.size());
    for( size_t i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = cv::boundingRect(contours_poly[i]);
        cv::minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
    }

    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::drawContours( img, contours_poly, (int)i, color );
        if (radius[i] > MIN_CIRCLE_SIZE && radius[i] < MAX_CIRCLE_SIZE) {
          cv::circle( img, centers[i], (int)radius[i], color, 2 );
        }
    }

    return img;
}

cv::Point2f ij2xy (cv::Point2f pt, uint32_t scaled_width, uint32_t scaled_height) {
    // TODO: make adaptive - use CROP_WIDTH and CROP_HEIGHT
    cv::Point2f origin(scaled_width/2 ,scaled_height);
    cv::Point2f xyPt(pt.x - origin.x, origin.y - pt.y);


    return xyPt;
}

cv::Point2f getContourCoordinates(std::vector<cv::Point> contour) {
    std::vector<cv::Point> contour_poly;
    cv::Point2f center;
    float radius;

    cv::approxPolyDP(contour, contour_poly, 3, true);
    cv::minEnclosingCircle(contour_poly, center, radius);

    // return ij2xy(center);
    return center;
}

std::vector<cv::Point2f> getAllContourCoordinates(std::vector<std::vector<cv::Point>> contours, float MIN_CIRCLE_SIZE, float MAX_CIRCLE_SIZE) {
    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Point2f>  centers(contours.size());
    std::vector<float>radius(contours.size());

    for( size_t i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        cv::minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
    }

    std::vector<cv::Point2f> result;
    for (size_t i=0; i< centers.size(); i++) {
      if (MIN_CIRCLE_SIZE < radius[i] && radius[i] < MAX_CIRCLE_SIZE) {
        result.push_back(centers[i]);
      }
    }
    if (result.size() == 0) {
      result = centers;
    }

    return result;
}

cv::Point2f getMeanCoordinatesOfContours(std::vector<std::vector<cv::Point>> contours, float MIN_CIRCLE_SIZE, float MAX_CIRCLE_SIZE, bool isBlue, uint32_t BLUE_THRESH, uint32_t YELLOW_THRESH) {
    /*
    Return coordinates of mean of all contours in ij-system.
    */
    std::vector<cv::Point2f> points;
    points = getAllContourCoordinates(contours, MIN_CIRCLE_SIZE, MAX_CIRCLE_SIZE);
    std::vector<cv::Point2f> valid_points;
    if (isBlue) {
      for (size_t i = 0; i<points.size(); i++) {
        if (BLUE_THRESH < points[i].x ) {
          valid_points.push_back(points[i]);
        }
      }
    } else {
      for (size_t i =0; i<points.size(); i++) {
        if ( points[i].x < YELLOW_THRESH) {
          valid_points.push_back(points[i]);
        }
      }
    }
    if (0 == valid_points.size()) {
      valid_points = points;
    }
    cv::Mat mean_;
    cv::reduce(valid_points, mean_, 01, CV_REDUCE_AVG);
    return cv::Point2f(mean_.at<float>(0,0), mean_.at<float>(0,1));
}

cv::Point2f getMidpoint(cv::Point2f pt1, cv::Point2f pt2) {
    /*
    Return coordinate of a midpoint between pt1 and pt2
    */
    std::vector<cv::Point2f> points{pt1, pt2};
    cv::Mat mean_;
    cv::reduce(points, mean_, 01, CV_REDUCE_AVG);
    cv::Point2f midpoint(mean_.at<float>(0,0), mean_.at<float>(0,1));
    return midpoint;
}

std::vector<std::string> tokenize(std::string s) {
  std::vector<std::string> tokens;
  size_t pos = 0;
  std::string delimiter = ",";
  std::string token;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    tokens.push_back(token);
    s.erase(0, pos + delimiter.length());
  }
  tokens.push_back(s);
  return tokens;
}

cv::Scalar toScalar(std::vector<std::string> data) {
  std::vector<int> int_data;
  for (auto const& i : data) {
    int_data.push_back(std::stoi(i));
  }
  cv::Scalar scal(int_data[0], int_data[1], int_data[2]);
  return scal;
}


int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.argb --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        const uint32_t CROP_WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["crop_width"]))};
        const uint32_t CROP_HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["crop_height"]))};
        const uint32_t X_DISTANCE{static_cast<uint32_t>(std::stoi(commandlineArguments["x_distance"]))};

        cv::Scalar hsvLowYellow = toScalar(tokenize(commandlineArguments["ylow"]));
        cv::Scalar hsvHiYellow = toScalar(tokenize(commandlineArguments["yhigh"]));
        cv::Scalar hsvLowBlue = toScalar(tokenize(commandlineArguments["blow"]));
        cv::Scalar hsvHiBlue = toScalar(tokenize(commandlineArguments["bhigh"]));
        const uint32_t NERODE{static_cast<uint32_t>(std::stoi(commandlineArguments["nerode"]))};
        const uint32_t NDILATE{static_cast<uint32_t>(std::stoi(commandlineArguments["ndilate"]))};
        const uint32_t MAX_CIRCLE_SIZE{static_cast<uint32_t>(std::stoi(commandlineArguments["max_circle_size"]))};
        const uint32_t MIN_CIRCLE_SIZE{static_cast<uint32_t>(std::stoi(commandlineArguments["min_circle_size"]))};
        const uint32_t BLUE_THRESH{static_cast<uint32_t>(std::stoi(commandlineArguments["blue_thresh"]))};
        const uint32_t YELLOW_THRESH{static_cast<uint32_t>(std::stoi(commandlineArguments["yellow_thresh"]))};


        // For monitoring execution time
        using std::chrono::high_resolution_clock;
        using std::chrono::duration_cast;
        using std::chrono::duration;
        using std::chrono::milliseconds;


        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Handler to receive distance readings (realized as C++ lambda).
            std::mutex distancesMutex;
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env){
                auto senderStamp = env.senderStamp();
                // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
                opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));

                // Store distance readings.
                std::lock_guard<std::mutex> lck(distancesMutex);
                switch (senderStamp) {
                    case 0: front = dr.distance(); break;
                    case 2: rear = dr.distance(); break;
                    case 1: left = dr.distance(); break;
                    case 3: right = dr.distance(); break;
                }
            };
            // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy image into cvMat structure.
                    // Be aware of that any code between lock/unlock is blocking
                    // the camera to provide the next frame. Thus, any
                    // computationally heavy algorithms should be placed outside
                    // lock/unlock
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                auto t1 = high_resolution_clock::now();
                // TODO: Do something with the frame.

                // Crop Image //TODO: make adaptive, use CROP_WIDTH and CROP_HEIGHT
                cv::Rect myROI(10, 10, 100, 100);
                cv::Mat croppedImage = img(myROI);

                uint32_t break_width{(WIDTH-CROP_WIDTH)/2};
	              cv::Mat crop_img = img(cv::Range(HEIGHT-CROP_HEIGHT,HEIGHT), cv::Range(break_width, WIDTH-break_width));

                int cropped_width{crop_img.cols};
                int cropped_height{crop_img.rows};
                // Scale image to half resolution
                cv::resize(crop_img, crop_img, cv::Size(cropped_width/2, cropped_height/2), cv::INTER_LINEAR);
                int scaled_width{crop_img.cols};
                int scaled_height{crop_img.rows};
                uint32_t scaled_xdistance{X_DISTANCE/2};

                // Drawing an ellipse over vehicle parts visible in camera feed
                cv::ellipse(crop_img, cv::Point(scaled_width/2, scaled_height),
                  cv::Size(scaled_width*4/10, scaled_height*2/5), 0, 0, 360,
                  cv::Scalar(0, 0, 0),-1, cv::LINE_AA);


                // Convert to HSV
                cv::Mat hsv;
                cv::cvtColor(crop_img, hsv, cv::COLOR_BGR2HSV);

                // Detect blue coloured pixels
                cv::Mat blueCones;
                cv::inRange(hsv, hsvLowBlue, hsvHiBlue, blueCones);

                // Dilate and erode to fill detected cones
                cv::Mat blueConesMask = dilate_erode(blueCones, NERODE, NDILATE);

                // Detect yellow coloured pixels
                cv::Mat yellowCones;
                cv::inRange(hsv, hsvLowYellow, hsvHiYellow, yellowCones);

                // Dilate and erode to fill detected cones
                cv::Mat yellowConesMask = dilate_erode(yellowCones, NERODE, NDILATE);

                // Use Masks from cones as pre-filter for edge detection
                cv::Mat combinedMask;
                combinedMask = yellowConesMask + blueConesMask;

                cv::Mat masked_img;
                crop_img.copyTo(masked_img, combinedMask);


                // Canny edge detection specific for yellow cones
                cv::Mat cannyYellow;
                cv::Canny(yellowConesMask, cannyYellow, 30, 90, 3);


                // Canny edge detection specific for blue cones
                cv::Mat cannyBlue;
                cv::Canny(blueConesMask, cannyBlue, 30, 90, 3);


                // ---- contours of Yellow Cones
                std::vector<std::vector<cv::Point>> contoursYellow;
                cv::findContours(cannyYellow, contoursYellow, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);



                //---- contours of Blue Cones
                std::vector<std::vector<cv::Point>> contoursBlue;
                cv::findContours(cannyBlue, contoursBlue, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

                // -------- Compute middle point------------
                // compute mean of blue cones
                cv::Point2f meanBlue;
                if (contoursBlue.size() > 0) {
                    meanBlue = getMeanCoordinatesOfContours(contoursBlue, MIN_CIRCLE_SIZE, MAX_CIRCLE_SIZE, true, BLUE_THRESH, YELLOW_THRESH);
                } else {
                    meanBlue = cv::Point2f(scaled_width, scaled_xdistance);
                }

                // // compute mean of yellow conescv::Point2f meanYellow;
                cv::Point2f meanYellow;
                if (contoursYellow.size() > 0) {
                    meanYellow = getMeanCoordinatesOfContours(contoursYellow, MIN_CIRCLE_SIZE, MAX_CIRCLE_SIZE, false, BLUE_THRESH, YELLOW_THRESH);
                } else {
                    meanYellow = cv::Point2f(0, scaled_xdistance);
                }

                // Compute midpoint (Between mean of yellow cones and mean of blue cones)
                cv::Point2f midpoint;
                midpoint = getMidpoint(meanBlue, meanYellow);

                // Project midpoint onto fixed-distance horizontal line
                // midpoint.y corresponds to x-distance in local frame!!!
                midpoint.y = scaled_height - scaled_xdistance;

                // Calculate Angle
                // midpoint.x corresponds to y-distance in local frame!!!
                float angle = atan(ij2xy(midpoint, scaled_width, scaled_height).x/scaled_xdistance);
                std::string str(std::to_string(angle));


                // -------- Compute middle point end------------

                auto t2 = high_resolution_clock::now();
                // Display image.
                if (VERBOSE) {
                    // Draw line from bottom center to midpoint
                    cv::line(crop_img, cv::Point2d(scaled_width/2, scaled_height),
                      midpoint, cv::Scalar(255, 255, 0), 3);
                    // Draw line from bottom center to projected midpoint
                    cv::line(crop_img, cv::Point2d(scaled_width/2, scaled_height),
                      midpoint, cv::Scalar(0, 0, 255), 5);
                    // Draw horizontal line
                    cv::line(crop_img, cv::Point2d(0, scaled_height-scaled_xdistance),
                      cv::Point2d(scaled_width, scaled_height-scaled_xdistance),
                      cv::Scalar(0, 0, 0), 1);
                    drawContours(crop_img, contoursYellow, cv::Scalar(0,255,255), MIN_CIRCLE_SIZE, MAX_CIRCLE_SIZE);
                    drawContours(crop_img, contoursBlue, cv::Scalar(255,0,0), MIN_CIRCLE_SIZE, MAX_CIRCLE_SIZE);
                    cv::putText(crop_img,str,cv::Point(scaled_width/2, scaled_height-scaled_xdistance),
                      cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,255),2,false);
                    // Print coordinates of yellow contours
                    if (contoursYellow.size() > 0){
                        std::cout << "Yellow contours!" << std::endl;
                        // for ( size_t i = 0; i < contoursYellow.size(); i++)
                        // {
                        //     std::cout << "ij: " << getContourCoordinates(contoursYellow[i]) << " xy: "<< ij2xy(getContourCoordinates(contoursYellow[i])) << std::endl;
                        // }
                    }

                    // Print coordinates of yellow contours
                    if (contoursBlue.size() > 0){
                        std::cout << "Blue contours!" << std::endl;
                        // for ( size_t i = 0; i < contoursBlue.size(); i++)
                        // {
                        //     std::cout << "ij: " << getContourCoordinates(contoursBlue[i]) << " xy: "<< ij2xy(getContourCoordinates(contoursBlue[i])) << std::endl;
                        // }
                    }

                    // Print coordinates of mean points
                    std::cout<< "mean Yellow: " << meanYellow << std::endl;
                    std::cout<< "mean Blue: " << meanBlue << std::endl;
                    std::cout<< "midpoint: " << midpoint << " xy:" <<  ij2xy(midpoint, scaled_width, scaled_height) << std::endl;
                    std::cout << "angle: " << angle << std::endl;

                    // Display image
                    cv::imshow("Cropped", crop_img);

                    cv::waitKey(1);
                }

                auto ms_int = duration_cast<milliseconds>(t2 - t1);
                std::cout << "Execution time = " << ms_int.count() << "ms" << std::endl;

                ////////////////////////////////////////////////////////////////
                // Do something with the distance readings if wanted.
                // {
                //     std::lock_guard<std::mutex> lck(distancesMutex);
                //     std::cout << "front = " << front << ", "
                //               << "rear = " << rear << ", "
                //               << "left = " << left << ", "
                //               << "right = " << right << "." << std::endl;
                // }

                ////////////////////////////////////////////////////////////////
                // Example for creating and sending a message to other microservices; can
                // be removed when not needed.
                opendlv::proxy::AngleReading ar;
                ar.angle(angle);
                od4.send(ar);

                ////////////////////////////////////////////////////////////////
                // Steering and acceleration/decelration.
                //
                // Uncomment the following linMes to steer; range: +38deg (left) .. -38deg (right).
                // Value groundSteeringRequest.groundSteering must be given in radians (DEG/180. * PI).
                //opendlv::proxy::GroundSteeringRequest gsr;
                //gsr.groundSteering(0);
                //od4.send(gsr);

                // Uncomment the following lines to accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
                // Be careful!
                //opendlv::proxy::PedalPositionRequest ppr;
                //ppr.position(0);
                //od4.send(ppr);
            }
        }
        retCode = 0;
    }
    return retCode;
}
