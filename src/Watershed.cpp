/*
 *  Watershed.cpp
 *
 *  Created on: 26.08.18
 *      Author: Stephan Manthe
 */

#include "Watershed.h"
#include <opencv2/opencv.hpp>
#include <vector>

std::vector<std::pair<int, int> > getNeighbors4(
    const std::pair<int, int>& pixel, const int rows, const int cols)
{
    std::vector<std::pair<int, int> > neighbors;
    neighbors.reserve(4);

    // top
    if (pixel.second > 0)
        neighbors.push_back(std::make_pair(pixel.first, pixel.second - 1));

    // bottom
    int tmp = pixel.second + 1;
    if (tmp < rows)
        neighbors.push_back(std::make_pair(pixel.first, tmp));

    // right
    tmp = pixel.first + 1;
    if (tmp < cols)
        neighbors.push_back(std::make_pair(tmp, pixel.second));

    // left
    if (pixel.first > 0)
        neighbors.push_back(std::make_pair(pixel.first - 1, pixel.second));

    return neighbors;
}

std::vector<std::pair<int, int> > getNeighbors9(
    const std::pair<int, int>& pixel, const int rows, const int cols)
{
    std::vector<std::pair<int, int> > neighbors;
    neighbors.reserve(8);

    int tmp, tmp1;

    // top
    if (pixel.second > 0)
    {
        tmp1 = pixel.second - 1;
        neighbors.push_back(std::make_pair(pixel.first, tmp1));

        if (pixel.first > 0)
            neighbors.push_back(std::make_pair(pixel.first - 1, tmp1));

        tmp = pixel.first + 1;
        if (tmp < cols)
            neighbors.push_back(std::make_pair(tmp, tmp1));
    }

    // bottom
    tmp = pixel.second + 1;
    if (tmp < rows)
    {
        neighbors.push_back(std::make_pair(pixel.first, tmp));

        if (pixel.first > 0)
            neighbors.push_back(std::make_pair(pixel.first - 1, tmp));

        tmp1 = pixel.first + 1;
        if (tmp1 < cols)
            neighbors.push_back(std::make_pair(tmp1, tmp));
    }


    // right
    tmp = pixel.first + 1;
    if (tmp < cols)
        neighbors.push_back(std::make_pair(tmp, pixel.second));

    // left
    if (pixel.first > 0)
        neighbors.push_back(std::make_pair(pixel.first - 1, pixel.second));

    return neighbors;
}


cv::Mat doWatershedSegmentation(
    const cv::Mat& gradientStrength, const WatershedParameters& parameters)
{
    // calculate the the stepsize for increasing the flood
    if (parameters.floodIncrement < 0.f)
        throw std::runtime_error("Number of flood steps is smaller than 0.0 .");

    const int width = gradientStrength.cols;
    const int height = gradientStrength.rows;

    // contains the maximum gradient strength
    float gradientStrengthMax = 0;

    // the counter for the labels
    int currentLabel = 0;

    // set the right format for the label image
    cv::Mat labelMatrix = cv::Mat{ height, width, CV_32S, Mark::INIT };

    // put all pixels into a vector
    std::vector<Pixel> vectorPixelCoordinates;
    vectorPixelCoordinates.reserve(width * height);
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
        {
            Pixel sortPixel;
            sortPixel.coordinate = std::make_pair(x, y);
            sortPixel.value = gradientStrength.at<float>(y, x);
            gradientStrengthMax = std::max(gradientStrengthMax, sortPixel.value);
            vectorPixelCoordinates.push_back(std::move(sortPixel));
        }

    // it is not allowed that the preFloodThreshold is bigger than the maximum
    // gradient strength
    if (parameters.prefloodThreshold > gradientStrengthMax)
        throw std::runtime_error(
            "The maximum gradient strength is larger then the preflood threshold.");

    // sort the pixels
    sort(vectorPixelCoordinates.begin(),
        vectorPixelCoordinates.end(),
        [](const Pixel& p1, const Pixel& p2) { return p1.value < p2.value; });

    // let the flood start at the preflood level
    float currentFloodLevel = parameters.prefloodThreshold;


    // put the pixels into different flood levels
    // a vector with lists of pixels which must be processed at each flood level
    std::vector<std::vector<std::pair<int, int> > > floodLevelPixels;
    std::vector<float> floodLevel;
    int floodIndex = 0;
    int coordIndex = 0;
    while (coordIndex < vectorPixelCoordinates.size())
    {
        std::vector<std::pair<int, int> > pixelsAtThisFloodLevel;

        // find pixels which have a lower value than the current flood level
        while (coordIndex < vectorPixelCoordinates.size())
        {

            if (vectorPixelCoordinates[coordIndex].value > currentFloodLevel)
                break;

            // push the coordinate into the vector with the pixels with a specific
            // flood level
            pixelsAtThisFloodLevel.push_back(vectorPixelCoordinates[coordIndex].coordinate);

            // increment the pixel index
            coordIndex++;
        }

        // create a new list for the next flood level
        if (!pixelsAtThisFloodLevel.empty())
        {
            floodLevel.push_back(currentFloodLevel);
            floodLevelPixels.push_back(pixelsAtThisFloodLevel);
            floodIndex++;
        }

        currentFloodLevel += parameters.floodIncrement;
    }
    
    int numFloodLevels
        = (gradientStrengthMax - parameters.prefloodThreshold) / parameters.floodIncrement;

    // two queues for neigbours of a pixel and for pixels which must be processed
    std::list<std::pair<int, int> > fifo;

    // start the segmentation
    for (size_t floodLevelIndex = 0; floodLevelIndex < floodLevelPixels.size(); ++floodLevelIndex)
    {
        const std::vector<std::pair<int, int> >& currentPixels = floodLevelPixels[floodLevelIndex];
        currentFloodLevel = floodLevel[floodLevelIndex];

        // get all pixels which have a neighbor with a pixel in the neighborhood
        // which is part of an segment or a watershed.
        for (const auto& currentPixel : currentPixels)
        {
            // the pixels which are labeled with mask will be processed during this
            // iteration
            labelMatrix.at<int>(currentPixel.second, currentPixel.first) = Mark::MASK;

            // OPTIMIZATION
            // searching for neighors with labels greater > 0 makes no sence, if
            // there are no pixels with labels greater 0 if there are no pixels
            // with labels there can't be pixels with neighbors
            //
            // TODO do this with Marks
            // if (currentLabel == 0)
            //     continue;

            // check the neighbor pixels
            const std::vector<std::pair<int, int> > neighbors
                = getNeighbors4(currentPixel, height, width);
            for (const auto& neighbor : neighbors)
            {
                const int labelVal = labelMatrix.at<int>(neighbor.second, neighbor.first);
                if (labelVal > 0 || labelVal == Mark::WATERSHED)
                {
                    labelMatrix.at<int>(currentPixel.second, currentPixel.first) = Mark::INQUEUE;
                    fifo.push_back(currentPixel);
                    // break;
                }
            }
        }

        // grow segments 
        while (!fifo.empty())
        {
            const std::pair<int, int>& p = fifo.front();
            const std::vector<std::pair<int, int> > neighbors = getNeighbors4(p, height, width);
            // This flag defines if a watershed pixel comes from another watershed pixel or from two
            // neighbored pixels with different labels.
            bool flag = false;

            for (const auto& neighbor : neighbors)
            {
                const int labelValNeighbor = labelMatrix.at<int>(neighbor.second, neighbor.first);
                const int labelValP = labelMatrix.at<int>(p.second, p.first);
                // the current pixel is member of an label or segment already
                if (labelValNeighbor > 0)
                {
                    if (labelValP == Mark::INQUEUE || (labelValP == Mark::WATERSHED && flag))
                        labelMatrix.at<int>(p.second, p.first) = labelValNeighbor;
                    else if (labelValP > 0 && labelValP != labelValNeighbor)
                    {
                        labelMatrix.at<int>(p.second, p.first) = Mark::WATERSHED;
                        flag = false;
                    }
                }
                // the neighbor is a watershed and the current pixel is in the queue
                else if (labelValNeighbor == Mark::WATERSHED && labelValP == Mark::INQUEUE)
                {
                    labelMatrix.at<int>(p.second, p.first) = Mark::WATERSHED;
                    flag = true;
                }
                else if (labelValNeighbor == Mark::MASK)
                {
                    labelMatrix.at<int>(neighbor.second, neighbor.first) = Mark::INQUEUE;
                    fifo.push_back(neighbor);
                }
            }
            fifo.pop_front();
        }

        // this queue is used to transfer pixels from one flood level into an other
        std::vector<std::pair<int, int> > queueTransfer;

        // start new segments
        for (const auto& currentPixel : currentPixels)
        {
            // next round if this pixel has a label already
            if (labelMatrix.at<int>(currentPixel.second, currentPixel.first) != Mark::MASK)
                continue;

            // Minimal lake depth: check if this pixel can create a lake which is deeper than the
            // minimal depth.
            const float lakeDepth = currentFloodLevel
                - gradientStrength.at<float>(currentPixel.second, currentPixel.first);

            if (lakeDepth < parameters.minimalLakeDepth
                && floodLevelIndex + 1 < floodLevelPixels.size())
            {
                // reset the label of the pixel
                labelMatrix.at<int>(currentPixel.second, currentPixel.first) = Mark::INIT;

                // save this pixel for processing at the next flood level
                queueTransfer.push_back(currentPixel);
                continue;
            }

            if (parameters.minimalSegementSize <= 0)
            {
                // start a new segment
                ++currentLabel;

                fifo.push_back(currentPixel);
                labelMatrix.at<int>(currentPixel.second, currentPixel.first) = currentLabel;

                while (!fifo.empty())
                {
                    const std::pair<int, int>& tempCoordinate = fifo.front();

                    // get the neightbours of that pixel
                    const std::vector<std::pair<int, int> > neighbors
                        = getNeighbors4(tempCoordinate, height, width);

                    // find neighbors which are part of the new segment
                    for (const auto& neightborLabel : neighbors)
                    {
                        // Set the new label
                        if (labelMatrix.at<int>(neightborLabel.second, neightborLabel.first)
                            == Mark::MASK)
                        {
                            labelMatrix.at<int>(neightborLabel.second, neightborLabel.first)
                                = currentLabel;
                            fifo.push_back(neightborLabel);
                        }
                    }

                    fifo.pop_front();
                }
            }
            else
            {
                ++currentLabel;
                fifo.push_back(currentPixel);
                labelMatrix.at<int>(currentPixel.second, currentPixel.first) = currentLabel;
                std::vector<std::pair<int, int> > newSegmentPixels;
                newSegmentPixels.push_back(currentPixel);
                while (!fifo.empty())
                {
                    const std::pair<int, int>& tempCoordinate = fifo.front();

                    // get the neightbours of that pixel
                    const std::vector<std::pair<int, int> > neighbors
                        = getNeighbors4(tempCoordinate, height, width);

                    // find neighbors which are part of the new segment
                    for (const auto& neightborLabel : neighbors)
                    {
                        // Set the new label
                        if (labelMatrix.at<int>(neightborLabel.second, neightborLabel.first)
                            == Mark::MASK)
                        {
                            labelMatrix.at<int>(neightborLabel.second, neightborLabel.first)
                                = currentLabel;
                            newSegmentPixels.push_back(neightborLabel);
                            fifo.push_back(neightborLabel);
                        }
                    }

                    fifo.pop_front();
                }

                if (newSegmentPixels.size() < parameters.minimalSegementSize)
                {
                    // start a new segment
                    --currentLabel;

                    for (const auto& pixel : newSegmentPixels)
                        labelMatrix.at<int>(pixel.second, pixel.first) = Mark::MASK;
                }
            }
        }

        // MINIMAL LAKE DEPTH
        // put the pixel which must be transfered into the next flood level into the
        // next list
        if (floodLevelIndex + 1 < floodLevelPixels.size() && !queueTransfer.empty())
        {
            floodLevelPixels[floodLevelIndex + 1].insert(
                floodLevelPixels[floodLevelIndex + 1].begin(),
                queueTransfer.begin(),
                queueTransfer.end());
        }
    }

    return labelMatrix;
}

cv::Mat removeWatersheds(const cv::Mat& labelMatrix, const cv::Mat image)
{
    if(image.type() !=  CV_8UC1)
        throw std::runtime_error("Image is of wrong type.");

    if(labelMatrix.type() != CV_32S)
        throw std::runtime_error("LabelMatrix is of wrong type.");

    cv::Mat labelMatrixClean = labelMatrix.clone();
    for (int i = 0; i < labelMatrix.rows; ++i)
        for (int j = 0; j < labelMatrix.cols; ++j)
            if (labelMatrix.at<int>(i, j) != Mark::WATERSHED)
                continue;
            else
            {
                const std::vector<std::pair<int, int> > neighbors = getNeighbors9(
                    std::make_pair(j, i), labelMatrixClean.rows, labelMatrixClean.cols);

                std::pair<int, int> closestPixel;
                unsigned char minimalDistance = std::numeric_limits<unsigned char>::max();
                
                const unsigned char& val = image.at<unsigned char>(i, j);
                for (const std::pair<int, int>& neighbor : neighbors)
                {
                    const unsigned char currentDistance
                        = std::abs(val - image.at<unsigned char>(neighbor.second, neighbor.first));

                    if (currentDistance < minimalDistance
                        && labelMatrix.at<int>(neighbor.second, neighbor.first) != Mark::WATERSHED
                        && neighbor.first > 0 && neighbor.first + 1 < labelMatrix.cols
                        && neighbor.second > 0 && neighbor.second + 1 < labelMatrix.rows)
                    {
                        closestPixel = neighbor;
                        minimalDistance = currentDistance;
                    }
                }

                if (minimalDistance != std::numeric_limits<unsigned char>::max())
                    labelMatrixClean.at<int>(i, j)
                        = labelMatrix.at<int>(closestPixel.second, closestPixel.first);
            }

    return labelMatrixClean;
}
