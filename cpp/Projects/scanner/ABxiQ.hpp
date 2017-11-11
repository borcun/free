/**
 * @file:   ABxiQ.h
 * @title:  XIMEA xiQ Camera Capture Class
 * @author: Burak Orcun OZKABLAN
 * @date:   30.04.2013
 */

#ifndef ABXIQ_H
#define ABXIQ_H

#include "ABRobotScannerUtil.hpp"

// image resolution
#define WIDTH_RES 2040
#define HEIGHT_RES 2040

class ABxiQ 
{
public:
	/// \brief default constructor
	ABxiQ();
	/// \brief constructor
	/// @param camera_number - camera index
	explicit ABxiQ(const int camera_number);
	/// \brief destructor
	virtual ~ABxiQ();
	/// \brief function that sets cameras number
	/// @param camera_number : camera number
	void setCameraNumber(const int camera_number);
	/// \brief function that sets acquisition image properties
	/// @param img_format - image format (defaultly RGB24)
	/// @param buffer - image buffer (defaultly NULL)
	/// @param buffer_size - image buffer size (defaultly 0)
	/// @return -
	void setImageProperties(void *buffer, DWORD buffer_size, XI_IMG_FORMAT img_format = XI_RGB24);
	/// \brief function that opens camera and fix neccesity connection
	/// @param buffer_policy - buffer policy for SAFE or UNSAFE buffer 
	/// @param exposure_time - exposure time of camera
	/// @return boolean
	bool openDevice(XI_BP buffer_policy, DWORD exposure_time);
	/// function that starts data acquisition
	/// @return boolean
	bool startAcquisition();
	/// \brief function that closes camera
	/// @return -
	void closeDevice();
	/// \brief function that converts xiQ image to OpenCV image
	/// @return IplImage pointer
	IplImage *xiQImage2IplImage();
	/// \brief function that returns xiQ image
	/// @return XI_IMG object
	XI_IMG getxiQImage();	
	/// \brief function that returns handle object
	/// @param bandwidth - bandwidth of camera
	/// @return -
	void setBandwidth(const int bandwidth);
	/// \brief function that returns device interface data rate
	/// @return int
	int getDataRate() const;
	/// \brief function that returns device information after opening
	/// @return char pointer
	const char *getDeviceInfo() const;
	/// \brief function that returns device temperature
	/// @return float
	float getTemperature();

private:
	HANDLE m_xiQ;
	XI_IMG m_image;
	DWORD m_buffer_size;
	IplImage *m_iplimage;
	int m_camera_number;
	char m_device_name[MAX_CHAR];
	int m_interface_data_rate;
	char m_deviceinfo[MAX_CHAR];
	float m_temperature;

	/// \brief function that gives report about occupied error
	/// @param error - error contain
	/// @return -
	void errorMessage(const char *error) const;

}; // end of class

#endif