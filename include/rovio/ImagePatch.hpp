
#ifndef IMAGEPATCH_HPP_
#define IMAGEPATCH_HPP_

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/utils/filesystem.hpp>

namespace rovio
{
    // Computes the median of the given patch.
    // Disregards the 0 values, as these are pixels where we have no information.
    float medianOfMatrix(const cv::Mat &mat)
    {

        // Find indices of non zero pizels
        cv::Mat matNonZero;
        cv::findNonZero(mat, matNonZero);

        // Create a vector of the non zero values
        std::vector<float> nonZeroValues;
        for (int i = 0; i < matNonZero.rows; i++)
        {
            nonZeroValues.push_back(mat.at<float>(matNonZero.at<cv::Point>(i)));
        }

        if (nonZeroValues.size() == 0)
        {
            return 0.0;
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

    // From https://superkogito.github.io/blog/2020/10/01/divide_image_using_opencv.html
    int divideImage(const cv::Mat &img, const int blockWidth, const int blockHeight, std::vector<cv::Mat> &blocks, cv::Mat &newImg)
    {
        // Checking if the image was passed correctly
        if (!img.data || img.empty())
        {
            std::wcout << "Image Error: Cannot load image to divide." << std::endl;
            return EXIT_FAILURE;
        }

        newImg = img.clone();

        const int patchSize = 6;
        // Padded img
        cv::Mat paddedImg;
        cv::copyMakeBorder(img, paddedImg, patchSize, patchSize, patchSize, patchSize, cv::BORDER_WRAP);

        for (int x = 0; x < img.rows; x++)
        {
            for (int y = 0; y < img.cols; y++)
            {
                int x_real = x + patchSize;
                int y_real = y + patchSize;

                int x_i = x_real - patchSize;
                int x_s = x_real + patchSize + 1;
                int y_i = y_real - patchSize;
                int y_s = y_real + patchSize + 1;

                cv::Mat curr_patch = paddedImg(cv::Range(x_i, x_s), cv::Range(y_i, y_s));

                float median = medianOfMatrix(curr_patch);
                newImg.at<float>(x, y) = median;
            }
        }

        return EXIT_SUCCESS;
    }

    // Compute patches, calculate median and return original image with median values in the patches
    void computeMedianPatches(const cv::Mat &img, const int blockWidth, const int blockHeight, cv::Mat &imgMedianPatches)
    {
        // Divide the image into patches
        std::vector<cv::Mat> blocks;
        divideImage(img, blockWidth, blockHeight, blocks, imgMedianPatches);
        return;

        // Compute the median of each patch
        std::vector<float> medians;
        for (int i = 0; i < blocks.size(); i++)
        {
            medians.push_back(medianOfMatrix(blocks[i]));
        }

        // Create a new image with the same size as the original image
        imgMedianPatches = cv::Mat::zeros(img.size(), img.type());

        // Fill the new image with the median values
        int y0 = 0;
        int i = 0;
        while (y0 < img.rows)
        {
            int x0 = 0;
            while (x0 < img.cols)
            {
                imgMedianPatches(cv::Rect(x0, y0, blockWidth, blockHeight)) = medians[i];
                x0 = x0 + blockWidth;
                i++;
            }
            y0 = y0 + blockHeight;
        }
    }
} // namespace rovio

#endif /* IMAGEPATCH_HPP_ */
