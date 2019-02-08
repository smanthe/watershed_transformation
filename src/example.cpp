#include "watershed/Watershed.h"
#include <iostream>
#include <opencv2/core.hpp>

#include <random>

void printHelp()
{
    std::cout << "Example: ./watershed -i INPUT_IMAGE_PATH -o OUTPUT_IMAGE_PATH" << std::endl;

    std::cout << "-i INPUT_PATH, --input_path INPUT_PATH" << std::endl;
    std::cout << "                  Path to the input image with an OpenCV supported image type."
              << std::endl
              << std::endl;
    std::cout << "-o OUTPUT_PATH, --output_path OUTPUT_PATH" << std::endl;
    std::cout << "                  Path to the output image with an OpenCV supported image type."
              << std::endl
              << std::endl;
    std::cout << "--flood_increment FLOOD_INCREMENT" << std::endl;
    std::cout << "                  Value by which the flood level is increased. (default 15.0)"
              << std::endl
              << std::endl;
    std::cout << "--minimal_lake_depth MINIMAL_LAKE_DEPTH" << std::endl;
    std::cout << "                  Minimal value of depth for a new segment. (default 30.0)"
              << std::endl
              << std::endl;
    std::cout << "--minimal_segment_size MINIMAL_SEGMENT_SIZE (default 25)" << std::endl;
    std::cout << "                  Minimal number of pixels which a segment must contain."
              << std::endl
              << std::endl;
    std::cout << "--preflood_threshold PREFLOOD_THRESHOLD" << std::endl;
    std::cout << "                  Value for the initial flood level (default 35.0)." << std::endl
              << std::endl;

    std::cout << "--remove_watersheds" << std::endl;
    std::cout << "                  Enables the removement of watersheds from the result image."
              << std::endl;
}

cv::Mat visualizeSegments(const cv::Mat labels)
{
    double min, max;
    cv::minMaxLoc(labels, &min, &max);
    std::vector<std::array<std::uint8_t, 3> > LUT{ (size_t)max };

    //std::random_device rd(0);
    std::mt19937 gen{ };
    std::uniform_int_distribution<> dis{ 0, 255 };

    for (size_t i = 0; i < LUT.size(); ++i)
    {
        LUT[i][0] = dis(gen);
        LUT[i][1] = dis(gen);
        LUT[i][2] = dis(gen);
    }


    cv::Mat vis = cv::Mat::zeros(labels.rows, labels.cols, CV_8UC3);
    for (int i = 0; i < vis.rows; ++i)
        for (int j = 0; j < vis.cols; j++)
        {
            const int idx = labels.at<int>(i, j);
            if (idx == -1)
                continue;

            vis.at<cv::Vec3b>(i, j)[0] = LUT[idx][0];
            vis.at<cv::Vec3b>(i, j)[1] = LUT[idx][1];
            vis.at<cv::Vec3b>(i, j)[2] = LUT[idx][2];
        }
    return vis;
}

int main(int argc, char* argv[])
{
    WatershedParameters params{ 35.f, 15.f, 30.f, 25 };

    std::string input, output;
    bool rmWatersheds = false;
    for (int i = 0; i < argc; ++i)
    {
        std::string value = argv[i];
        if (value == "--help" || value == "-h")
        {
            printHelp();
            exit(0);
        }
        else if ((value == "--output_path" || value == "-o") && i + 1 < argc)
            output = argv[i + 1];
        else if ((value == "--input_path" || value == "-i") && i + 1 < argc)
            input = argv[i + 1];
        else if (value == "--flood_increment" && i + 1 < argc)
            params.floodIncrement = atof(argv[i + 1]);
        else if (value == "--minimal_lake_depth" && i + 1 < argc)
            params.minimalLakeDepth = atof(argv[i + 1]);
        else if (value == "--minimal_segment_size" && i + 1 < argc)
            params.minimalSegementSize = atoi(argv[i + 1]);
        else if (value == "--preflood_threshold" && i + 1 < argc)
            params.prefloodThreshold = atof(argv[i + 1]);
        else if (value == "--remove_watersheds" || value == "-r")
            rmWatersheds = true;
    }

    if (input == "")
    {
        std::cout << "Error: Please pass the input image." << std::endl << std::endl;
        printHelp();
        exit(0);
    }

    if (output == "")
    {
        std::cout << "Error: Please pass the output image." << std::endl << std::endl;
        printHelp();
        exit(0);
    }

    cv::Mat image = cv::imread(input, cv::IMREAD_GRAYSCALE);
    cv::equalizeHist(image, image);

    // Gradient X
    cv::Mat gradX;
    cv::Sobel(image, gradX, CV_32F, 1, 0, 3);

    // Gradient Y
    cv::Mat gradY;
    cv::Sobel(image, gradY, CV_32F, 0, 1, 3);

    // Total Gradient (approximate)
    cv::Mat gradientStrengths;
    cv::magnitude(gradX, gradY, gradientStrengths);

    try
    {
        cv::Mat labelMatrix = doWatershedSegmentation(gradientStrengths, params);

        if (rmWatersheds)
            labelMatrix = removeWatersheds(labelMatrix, image);

        const cv::Mat vis = visualizeSegments(labelMatrix);
        cv::imwrite(output, vis);
    }
    catch (const std::runtime_error& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        printHelp();
    }
    return 0;
}
