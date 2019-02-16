#include "GripHatchPipeline.h"

namespace grip {

GripHatchPipeline::GripHatchPipeline() {
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void GripHatchPipeline::Process(cv::Mat& source0){
	//Step Resize_Image0:
	//input
	cv::Mat resizeImageInput = source0;
	double resizeImageWidth = 240.0;  // default Double
	double resizeImageHeight = 180.0;  // default Double
	int resizeImageInterpolation = cv::INTER_CUBIC;
	resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, this->resizeImageOutput);
	//Step Blur0:
	//input
	cv::Mat blurInput = resizeImageOutput;
	BlurType blurType = BlurType::GAUSSIAN;
	double blurRadius = 1.801801801801803;  // default Double
	blur(blurInput, blurType, blurRadius, this->blurOutput);
	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThresholdInput = blurOutput;
	double hsvThresholdHue[] = {8.093525179856115, 55.597269624573386};
	double hsvThresholdSaturation[] = {84.84712230215827, 255.0};
	double hsvThresholdValue[] = {158.22841726618705, 220.1877133105802};
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
}

/**
 * This method is a generated getter for the output of a Resize_Image.
 * @return Mat output from Resize_Image.
 */
cv::Mat* GripHatchPipeline::GetResizeImageOutput(){
	return &(this->resizeImageOutput);
}
/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
cv::Mat* GripHatchPipeline::GetBlurOutput(){
	return &(this->blurOutput);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* GripHatchPipeline::GetHsvThresholdOutput(){
	return &(this->hsvThresholdOutput);
}
	/**
	 * Scales and image to an exact size.
	 *
	 * @param input The image on which to perform the Resize.
	 * @param width The width of the output in pixels.
	 * @param height The height of the output in pixels.
	 * @param interpolation The type of interpolation.
	 * @param output The image in which to store the output.
	 */
	void GripHatchPipeline::resizeImage(cv::Mat &input, double width, double height, int interpolation, cv::Mat &output) {
		cv::resize(input, output, cv::Size(width, height), 0.0, 0.0, interpolation);
	}

	/**
	 * Softens an image using one of several filters.
	 *
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	void GripHatchPipeline::blur(cv::Mat &input, BlurType &type, double doubleRadius, cv::Mat &output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type) {
			case BOX:
				kernelSize = 2 * radius + 1;
				cv::blur(input,output,cv::Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				cv::medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				cv::bilateralFilter(input, output, -1, radius, radius);
				break;
        }
	}
	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param val The min and max value.
	 * @param output The image in which to store the output.
	 */
	void GripHatchPipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
	}



} // end grip namespace

