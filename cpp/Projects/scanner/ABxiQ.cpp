#include "ABxiQ.hpp"

using namespace std;

// \brief default constructor
ABxiQ::ABxiQ() : m_xiQ(INVALID_HANDLE_VALUE), m_buffer_size(0)
{
	memset(m_deviceinfo, '\0', MAX_CHAR);
}

// \brief constructor
// @param camera_number : camera number
ABxiQ::ABxiQ(const int camera_number) : m_xiQ(INVALID_HANDLE_VALUE), m_buffer_size(0)
{
	memset(m_deviceinfo, '\0', MAX_CHAR);
	setCameraNumber(camera_number);
}

// \brief destructor
ABxiQ::~ABxiQ()
{
	closeDevice();
	cvReleaseImage(&m_iplimage);
}

// \brief function that sets cameras number
void ABxiQ::setCameraNumber(const int camera_number)
{
	m_camera_number = camera_number;
	return;
}

// function that sets acquisition image properties
// Note : if user does not specify parameters, image can not be converted to OpenCV image
void ABxiQ::setImageProperties(void *buffer, DWORD buffer_size, XI_IMG_FORMAT img_format)
{
	// set image properties
	m_image.size = sizeof(XI_IMG);
	m_image.bp = buffer;
	m_image.bp_size = m_buffer_size = buffer_size;
	m_image.frm = img_format;
	m_image.width = WIDTH_RES;
	m_image.height = HEIGHT_RES;
	return;
}

// function that opens camera and fixes neccesity configuration
bool ABxiQ::openDevice(XI_BP buffer_policy, DWORD exposure_time)
{
	// check buffer pointer and buffer size
	if(XI_BP_SAFE == buffer_policy && m_buffer_size == 0) {
		errorMessage("You should specify a buffer and buffer size");
		return false;
	}
	if(XI_BP_SAFE != buffer_policy && m_buffer_size != 0) {
		errorMessage("You should not specify a buffer and buffer size to use XI_BP_UNSAFE");
		return false;
	}

	int fps = 0;

	// set width and height of camera resolution
	if(XI_WRONG_PARAM_VALUE == xiSetParamInt(m_xiQ, XI_PRM_WIDTH, WIDTH_RES) || XI_WRONG_PARAM_VALUE == xiSetParamInt(m_xiQ, XI_PRM_HEIGHT, HEIGHT_RES)) {
		errorMessage("DEVICE BUFFER POLICY NOT CONFIGURED");
		closeDevice();
		return false;
	}

	// open device
	if(XI_OK != xiOpenDevice(m_camera_number, &m_xiQ)) {
		errorMessage("DEVICE NOT OPENED");
		return false;
	}

	// set buffer policy of device
	if(XI_OK != xiSetParam(m_xiQ, XI_PRM_BUFFER_POLICY, &buffer_policy, sizeof(DWORD), xiTypeInteger)) {
		errorMessage("DEVICE BUFFER POLICY NOT CONFIGURED");
		closeDevice();
		return false;
	}
	// set exposure time of device
	if(XI_OK != xiSetParam(m_xiQ, XI_PRM_EXPOSURE, &exposure_time, sizeof(exposure_time), xiTypeInteger)) {
		errorMessage("DEVICE EXPOSURE TIME NOT CONFIGURED");
		closeDevice();
		return false;
	}
	// set auto exposure/gain rate of device
	if(XI_OK != xiSetParamInt(m_xiQ, XI_PRM_AEAG, 1)) {
		errorMessage("DEVICE AUTO EXPOSURE/GAIN NOT CONFIGURED");
		closeDevice();
		return false;
	}

	// allocate memory for image and set video stream format
	switch(m_image.frm) {
		// XI_MONO8 Image Format
		case XI_MONO8: 
			m_iplimage = cvCreateImage(cvSize(WIDTH_RES, HEIGHT_RES), IPL_DEPTH_8U, 1);
			// set image format of device
			if(XI_OK != xiSetParamInt(m_xiQ, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8)) {
				errorMessage("DEVICE IMAGE FORMAT NOT CONFIGURED");
				closeDevice();
				return false;
			}
		break;
		// XI_MONO16 Image Format
		case XI_MONO16:
			m_iplimage = cvCreateImage(cvSize(WIDTH_RES, HEIGHT_RES), IPL_DEPTH_16S, 1);
			// set image format of device
			if(XI_OK != xiSetParamInt(m_xiQ, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO16)) {
				errorMessage("DEVICE IMAGE FORMAT NOT CONFIGURED");
				closeDevice();
				return false;
			}
		break;
		// XI_RGB24 Image Format
		case XI_RGB24:
			m_iplimage = cvCreateImage(cvSize(WIDTH_RES, HEIGHT_RES), IPL_DEPTH_8U, 3);
			// set image format of device
			if(XI_OK != xiSetParamInt(m_xiQ, XI_PRM_IMAGE_DATA_FORMAT, XI_RGB24)) {
				errorMessage("DEVICE IMAGE FORMAT NOT CONFIGURED");
				closeDevice();
				return false;
			}
		break;
		// XI_RGB32 Image Format
		case XI_RGB32:
			m_iplimage = cvCreateImage(cvSize(WIDTH_RES, HEIGHT_RES), IPL_DEPTH_8U, 4);
			// set image format of device
			if(XI_OK != xiSetParamInt(m_xiQ, XI_PRM_IMAGE_DATA_FORMAT, XI_RGB32)) {
				errorMessage("DEVICE IMAGE FORMAT NOT CONFIGURED");
				closeDevice();
				return false;
			}
		break;		
		default:
			errorMessage("DATA FORMAT CAN NOT SET or IMAGE CAN NOT CREATED");
			closeDevice();
		return false;
	} // end of switch
	
	// get fps rate
	xiGetParamInt(m_xiQ, XI_PRM_FRAMERATE, &fps);
	// get data rate
	xiGetParamInt(m_xiQ, XI_PRM_AVAILABLE_BANDWIDTH, &m_interface_data_rate);
	// get device name
	xiGetParamString(m_xiQ, XI_PRM_DEVICE_NAME, m_device_name, MAX_CHAR);
	cout << "DEVICE " << m_device_name << " " << m_camera_number << " IS OPENED [ " << WIDTH_RES << "x" << HEIGHT_RES << " , " << fps <<  "fps , " << m_interface_data_rate << " Mbps ]" << endl;
	sprintf(m_deviceinfo, "DEVICE %d-%d IS OPENED [ %dx%d Resolution, %d fps, %d Mbps ]\n", m_device_name, m_camera_number, WIDTH_RES, HEIGHT_RES, fps, m_interface_data_rate);

	return true;
}

// function that starts data acqusition
bool ABxiQ::startAcquisition()
{
	// start acquisition
	if(XI_OK != xiStartAcquisition(m_xiQ)) {
		errorMessage("CAMERA NOT OPENED");
		closeDevice();
		return false;
	}
	
	return true;
}

// function that closes camera
void ABxiQ::closeDevice()
{
	// close device
	xiStopAcquisition(m_xiQ);
	xiCloseDevice(m_xiQ);
	return;
}

// function that converts xiQ image to OpenCV image
IplImage *ABxiQ::xiQImage2IplImage()
{
	// get new image from device and copy it to iplimage
	xiGetImage(m_xiQ, 1000, &m_image);
	strcpy(m_iplimage->imageData, (char *)m_image.bp);
	return m_iplimage;
}

// function that returns xiQ image
XI_IMG ABxiQ::getxiQImage() 
{
	// get new image from device and copy it to iplimage
	xiGetImage(m_xiQ, 1000, &m_image);
	return m_image; 
}

// function that set bandwidth
void ABxiQ::setBandwidth(const int bandwidth)
{
	xiSetParamInt(m_xiQ, XI_PRM_LIMIT_BANDWIDTH , bandwidth);
	return;
}

// function that returns device interface data rate
int ABxiQ::getDataRate() const 
{ 
	return m_interface_data_rate; 
}

// function that gives report about occupied error
void ABxiQ::errorMessage(const char *error) const {
	std::cerr << "ERROR : " << error << std::endl;
	return;
}

// function that returns device information after opening
const char *ABxiQ::getDeviceInfo() const
{
	return m_deviceinfo;
}

// function that returns device temperature
float ABxiQ::getTemperature() 
{
	xiSetParamFloat(m_xiQ, XI_PRM_TARGET_TEMP, m_temperature);
	return m_temperature;
}

