/*
 *  Watershed.h
 *
 *  Created on: 26.08.18
 *      Author: Stephan Manthe
 */

#ifndef WATERSHED_H
#define WATERSHED_H

#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

struct WatershedParameters
{
    WatershedParameters(float prefloodThreshold,
        float floodIncrement,
        float minimalLakeDepth,
        unsigned char minimalSegementSize)
        : prefloodThreshold(prefloodThreshold)
        , floodIncrement(floodIncrement)
        , minimalLakeDepth(minimalLakeDepth)
        , minimalSegementSize(minimalSegementSize)
    {
    }

    float prefloodThreshold;
    float floodIncrement;
    float minimalLakeDepth;
    size_t minimalSegementSize;
};

struct Pixel
{
    std::pair<int, int> coordinate;
    float value;
};

enum Mark : std::int8_t
{
    MASK = -2, // Means that this pixel is not labeled at this flood level.
    WATERSHED = 0, // Means that this pixel is part of a watershed.
    INIT = -1, // Means that this pixel was not processed for now.
    INQUEUE = -3 // Means that this pixel is in the queue.
};

std::vector<std::pair<int, int> > getNeighbors(
    const std::pair<int, int>& pixel, const int rows, const int cols);

cv::Mat doWatershedSegmentation(
    const cv::Mat& gradientStrength, const WatershedParameters& parameters);

cv::Mat removeWatersheds(const cv::Mat& labelMatrix, const cv::Mat image);
#endif
