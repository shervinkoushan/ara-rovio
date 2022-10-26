
#ifndef IMAGEPATCH_HPP_
#define IMAGEPATCH_HPP_

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/utils/filesystem.hpp>

namespace rovio
{

    // From https://superkogito.github.io/blog/2020/10/01/divide_image_using_opencv.html
    int divideImage(const cv::Mat &img, const int blockWidth, const int blockHeight, std::vector<cv::Mat> &blocks)
    {
        // Checking if the image was passed correctly
        if (!img.data || img.empty())
        {
            std::wcout << "Image Error: Cannot load image to divide." << std::endl;
            return EXIT_FAILURE;
        }

        // init image dimensions
        int imgWidth = img.cols;
        int imgHeight = img.rows;
        std::wcout << "IMAGE SIZE: "
                   << "(" << imgWidth << "," << imgHeight << ")" << std::endl;

        // init block dimensions
        int bwSize;
        int bhSize;

        int y0 = 0;
        while (y0 < imgHeight)
        {
            // compute the block height
            bhSize = ((y0 + blockHeight) > imgHeight) * (blockHeight - (y0 + blockHeight - imgHeight)) + ((y0 + blockHeight) <= imgHeight) * blockHeight;

            int x0 = 0;
            while (x0 < imgWidth)
            {
                // compute the block height
                bwSize = ((x0 + blockWidth) > imgWidth) * (blockWidth - (x0 + blockWidth - imgWidth)) + ((x0 + blockWidth) <= imgWidth) * blockWidth;

                // crop block
                blocks.push_back(img(cv::Rect(x0, y0, bwSize, bhSize)).clone());

                // update x-coordinate
                x0 = x0 + blockWidth;
            }

            // update y-coordinate
            y0 = y0 + blockHeight;
        }
        return EXIT_SUCCESS;
    }

    // Computes the median of the given patch.
    // Disregards the 0 values, as these are pixels where we have no information.
    float medianOfMatrix(const cv::Mat &mat){

        // Find indices of non zero pizels
        cv::Mat matNonZero;
        cv::findNonZero(mat, matNonZero);

        // Create a vector of the non zero values
        std::vector<float> nonZeroValues;
        for (int i = 0; i < matNonZero.rows; i++)
        {
            nonZeroValues.push_back(mat.at<float>(matNonZero.at<cv::Point>(i)));
        }

        // Sort the vector
        std::sort(nonZeroValues.begin(), nonZeroValues.end());

        // Compute the median
        float median;
        if (nonZeroValues.size() % 2 == 0)
        {
            median = (nonZeroValues[nonZeroValues.size() / 2 - 1] + nonZeroValues[nonZeroValues.size() / 2]) / 2;
        }
        else
        {
            median = nonZeroValues[nonZeroValues.size() / 2];
        }

        return median;
    }

} // namespace rovio

#endif /* IMAGEPATCH_HPP_ */
