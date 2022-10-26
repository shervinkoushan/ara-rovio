
#ifndef IMAGEPATCH_HPP_
#define IMAGEPATCH_HPP_

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/utils/filesystem.hpp>

namespace rovio
{

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
        }

#endif /* IMAGEPYRAMID_HPP_ */
