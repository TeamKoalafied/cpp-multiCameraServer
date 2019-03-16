#include "GripCargoPipeline.h"

namespace cargoGrip {

GripCargoPipeline::GripCargoPipeline() {
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void GripCargoPipeline::Process(cv::Mat& source0){
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
	BlurType blurType = BlurType::BOX;
	double blurRadius = 12.612612612612613;  // default Double
	blur(blurInput, blurType, blurRadius, this->blurOutput);
	//Step HSL_Threshold0:
	//input
	cv::Mat hslThresholdInput = blurOutput;
	double hslThresholdHue[] = {0.0, 50.98976109215017};
	double hslThresholdSaturation[] = {188.03956834532374, 255.0};
	double hslThresholdLuminance[] = {98.60611510791367, 204.95733788395904};
	hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, this->hslThresholdOutput);
	//Step Mask0:
	//input
	cv::Mat maskInput = blurOutput;
	cv::Mat maskMask = hslThresholdOutput;
	mask(maskInput, maskMask, this->maskOutput);
	//Step RGB_Threshold0:
	//input
	cv::Mat rgbThresholdInput = maskOutput;
	double rgbThresholdRed[] = {206.38489208633092, 255.0};
	double rgbThresholdGreen[] = {64.20863309352518, 215.83617747440275};
	double rgbThresholdBlue[] = {11.465827338129495, 141.86006825938566};
	rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, this->rgbThresholdOutput);
}

/**
 * This method is a generated getter for the output of a Resize_Image.
 * @return Mat output from Resize_Image.
 */
cv::Mat* GripCargoPipeline::GetResizeImageOutput(){
	return &(this->resizeImageOutput);
}
/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
cv::Mat* GripCargoPipeline::GetBlurOutput(){
	return &(this->blurOutput);
}
/**
 * This method is a generated getter for the output of a HSL_Threshold.
 * @return Mat output from HSL_Threshold.
 */
cv::Mat* GripCargoPipeline::GetHslThresholdOutput(){
	return &(this->hslThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Mask.
 * @return Mat output from Mask.
 */
cv::Mat* GripCargoPipeline::GetMaskOutput(){
	return &(this->maskOutput);
}
/**
 * This method is a generated getter for the output of a RGB_Threshold.
 * @return Mat output from RGB_Threshold.
 */
cv::Mat* GripCargoPipeline::GetRgbThresholdOutput(){
	return &(this->rgbThresholdOutput);
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
	void GripCargoPipeline::resizeImage(cv::Mat &input, double width, double height, int interpolation, cv::Mat &output) {
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
	void GripCargoPipeline::blur(cv::Mat &input, BlurType &type, double doubleRadius, cv::Mat &output) {
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
	 * Segment an image based on hue, saturation, and luminance ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param lum The min and max luminance.
	 * @param output The image in which to store the output.
	 */
	//void hslThreshold(Mat *input, double hue[], double sat[], double lum[], Mat *out) {
	void GripCargoPipeline::hslThreshold(cv::Mat &input, double hue[], double sat[], double lum[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HLS);
		cv::inRange(out, cv::Scalar(hue[0], lum[0], sat[0]), cv::Scalar(hue[1], lum[1], sat[1]), out);
	}

		/**
		 * Filter out an area of an image using a binary mask.
		 *
		 * @param input The image on which the mask filters.
		 * @param mask The binary image that is used to filter.
		 * @param output The image in which to store the output.
		 */
		void GripCargoPipeline::mask(cv::Mat &input, cv::Mat &mask, cv::Mat &output) {
			mask.convertTo(mask, CV_8UC1);
			cv::bitwise_xor(output, output, output);
			input.copyTo(output, mask);
		}

	/**
	 * Segment an image based on color ranges.
	 *
	 * @param input The image on which to perform the RGB threshold.
	 * @param red The min and max red.
	 * @param green The min and max green.
	 * @param blue The min and max blue.
	 * @param output The image in which to store the output.
	 */
	void GripCargoPipeline::rgbThreshold(cv::Mat &input, double red[], double green[], double blue[], cv::Mat &output) {
		cv::cvtColor(input, output, cv::COLOR_BGR2RGB);
		cv::inRange(output, cv::Scalar(red[0], green[0], blue[0]), cv::Scalar(red[1], green[1], blue[1]), output);
	}



} // end grip namespace
