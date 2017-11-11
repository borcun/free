#include "ABRobotScanner.hpp"

// constructor
ABRobotScanner::ABRobotScanner(QWidget *parent, Qt::WFlags flags) : QWidget(parent, flags)
{
	ui.setupUi(this);
	init();
	connectSignalsToSlots();
}

// destructor
ABRobotScanner::~ABRobotScanner()
{
    delete m_ab_calibration_1;
    delete m_ab_calibration_2;
    delete m_ab_fd;
    delete m_ab_fm;
    delete m_ab_em;
    delete m_ab_epilines_1;
    delete m_ab_epilines_2;
    delete m_ab_reconstruction;
    delete m_ab_vm;
    // release matrices
    cvReleaseMat(&m_dis_coef_1);
    cvReleaseMat(&m_dis_coef_2);
    cvReleaseMat(&m_intrinsic_matrix_1);
    cvReleaseMat(&m_intrinsic_matrix_2);
    cvReleaseMat(&m_fundamental_matrix);
    cvReleaseMat(&m_essential_matrix);
    cvReleaseMat(&m_rotation_matrix_1);
    cvReleaseMat(&m_rotation_matrix_2);
    cvReleaseMat(&m_translation_matrix_1);
    cvReleaseMat(&m_translation_matrix_2);
    cvReleaseMat(&m_projection_matrix_1);
    cvReleaseMat(&m_projection_matrix_2);
    // release images
    cvReleaseImage(&m_left_temp_image);
    cvReleaseImage(&m_right_temp_image);
	// close devices
	m_left_capture->closeDevice();
    //m_right_capture->closeDevice();
	// free buffers
	free(buffer_1);
	//free(buffer_2);
	delete m_left_capture;
	//delete m_right_capture;
}

// function that initializes fundamental materials
void ABRobotScanner::init()
{
    char create_dir_command[MAX_CHAR];
    int result;

#ifdef __linux__
    bzero(create_dir_command, MAX_CHAR);
    // create two directory for cameras frames
    sprintf_s(create_dir_command, "%s%s","mkdir -p ", LEFT_CAM_DIR);
    result = system(create_dir_command);
    bzero(create_dir_command, MAX_CHAR);
    sprintf_s(create_dir_command, "%s%s","mkdir -p ", RIGHT_CAM_DIR);
    result = system(create_dir_command);
    bzero(create_dir_command, MAX_CHAR);
    sprintf_s(create_dir_command, "%s%s","mkdir -p ", CORR_DIR);
    result = system(create_dir_command);
    bzero(create_dir_command, MAX_CHAR);
    sprintf_s(create_dir_command, "%s%s","mkdir -p ", RECONS_DIR);
    result = system(create_dir_command);
    bzero(create_dir_command, MAX_CHAR);
#elif _WIN32
    memset(create_dir_command, MAX_CHAR, '\0');
    // create two directory for cameras frames
    sprintf_s(create_dir_command, "%s%s","md ", LEFT_CAM_DIR);
    result = system(create_dir_command);
    memset(create_dir_command, MAX_CHAR, '\0');
    sprintf_s(create_dir_command, "%s%s","md ", RIGHT_CAM_DIR);
    result = system(create_dir_command);
    memset(create_dir_command, MAX_CHAR, '\0');
    sprintf_s(create_dir_command, "%s%s","md ", CORR_DIR);
    result = system(create_dir_command);
    memset(create_dir_command, MAX_CHAR, '\0');
    sprintf_s(create_dir_command, "%s%s","md ", RECONS_DIR);
    result = system(create_dir_command);
    memset(create_dir_command, MAX_CHAR, '\0');
#else
    cout << "OS Incompatibility" << endl;
#endif

    // set space calibration button as checked
    ui.g_spacecalib_radiobutton->setChecked(true);
    ui.g_xyzext_radiobutton->setChecked(true);
	ui.g_stream_button->setEnabled(false);
	// allocate memory for data member
    m_intrinsic_matrix_1 = cvCreateMat(3, 3, CV_64FC1);
    m_intrinsic_matrix_2 = cvCreateMat(3, 3, CV_64FC1);
    m_dis_coef_1 = cvCreateMat(5, 1, CV_64FC1);
    m_dis_coef_2 = cvCreateMat(5, 1, CV_64FC1);
    m_fundamental_matrix = cvCreateMat(3, 3, CV_64FC1);
    m_essential_matrix   = cvCreateMat(3, 3, CV_64FC1);
    m_rotation_matrix_1  = cvCreateMat(3, 3, CV_64FC1);
    m_rotation_matrix_2  = cvCreateMat(3, 3, CV_64FC1);
    m_translation_matrix_1 = cvCreateMat(3, 1, CV_64FC1);
    m_translation_matrix_2 = cvCreateMat(3, 1, CV_64FC1);
    m_projection_matrix_1  = cvCreateMat(3, 4, CV_64FC1);
    m_projection_matrix_2  = cvCreateMat(3, 4, CV_64FC1);
    // initialize m_ab objects with NULL pointer
	m_left_capture = new ABxiQ();
	m_right_capture = new ABxiQ();
	m_ab_calibration_1 = new ABCalibration();
    m_ab_calibration_2 = new ABCalibration();
	m_ab_fd = new ABFeatureDetector(e_SIFT);
    m_ab_fm = new ABFundamentalMatrix();
	m_ab_em = new ABEssentialMatrix();
    m_ab_epilines_1 = new ABEpipolarLine();
    m_ab_epilines_2 = new ABEpipolarLine();
	m_ab_reconstruction = new ABReconstruction();
	m_ab_vm = new ABVisualModel();

    return;
}

// function that connects signals to slots
void ABRobotScanner::connectSignalsToSlots()
{
	connect(ui.g_start_button, SIGNAL(clicked()), this, SLOT(startSystem()));
	connect(ui.g_stream_button, SIGNAL(clicked()), this, SLOT(stream()));
    connect(ui.g_twocamcap_button, SIGNAL(clicked()), this, SLOT(twoCameraCapturing()));
    connect(ui.g_howtocalib_button, SIGNAL(clicked()), this, SLOT(howToCalibrating()));
    connect(ui.g_clearcache_button, SIGNAL(clicked()), this, SLOT(clearCache()));
	connect(ui.g_startcalib_button, SIGNAL(clicked()), this, SLOT(calibration()));
	connect(ui.g_refinesystem_button, SIGNAL(clicked()), this, SLOT(refineSystem()));
    connect(ui.g_reconstruction_button, SIGNAL(clicked()), this, SLOT(reconstruction()));
    connect(ui.g_visualmodel_button, SIGNAL(clicked()), this, SLOT(visualModel()));
    connect(ui.g_savepc_button, SIGNAL(clicked()), this, SLOT(savePointCloud()));
    connect(ui.g_helpfile_button, SIGNAL(clicked()), this, SLOT(helpFile()));
    connect(ui.g_exit_button, SIGNAL(clicked()), this, SLOT(exit()));

    return;
}

// function that starts system
void ABRobotScanner::startSystem()
{
    // allocate memory for captures
	m_left_capture  = new ABxiQ(1);
    //m_right_capture = new ABxiQ(1);

	const int buffer_size = WIDTH_RES * HEIGHT_RES * 3;
	buffer_1 = (void *)malloc(buffer_size);
	//buffer_2 = (void *)malloc(buffer_size);

	// set devices and open them
	m_left_capture->setImageProperties(buffer_1, buffer_size, XI_RGB24);
	//m_right_capture.setImageProperties(buffer_2, buffer_size, XI_RGB24);

	//create a message box about removing process
	QMessageBox message_box(NULL);
	QAbstractButton *ok_button = message_box.addButton("OK", QMessageBox::ActionRole);

	// set window title
	message_box.setWindowTitle("Device Info");

	if(m_left_capture->openDevice(XI_BP_SAFE, 10000)) {
        // set message of message box
		message_box.setText(m_left_capture->getDeviceInfo());
	}
	else {
        // set message of message box
        message_box.setText("DEVICE IS NOT OPENED.\n");
		return;
	}

    // execute message box
    message_box.exec();
    // after message_box execution, close window
    if(message_box.clickedButton() == ok_button)
        message_box.close();


	//m_right_capture.openDevice(XI_BP_SAFE, 10000);

	// disable button
    ui.g_start_button->setEnabled(false);
	ui.g_refinesystem_button->setEnabled(false);
	ui.g_stream_button->setEnabled(true);
}

// function that starts stream of cameras
void ABRobotScanner::stream()
{
	//if(m_left_capture->startAcquisition() && m_right_capture->startAcquisition()) {
		m_left_capture->startAcquisition();
		m_left_temp_image  = cvCreateImage(cvSize(WIDTH_RES, HEIGHT_RES), IPL_DEPTH_8U, 3);
		m_right_temp_image = cvCreateImage(cvSize(WIDTH_RES, HEIGHT_RES), IPL_DEPTH_8U, 3);
		// set bandwidths of cameras
		ui.g_leftcam_bw_lineedit->setText(QString::number(m_left_capture->getDataRate()));
		ui.g_rightcam_bw_lineedit->setText(QString::number(m_left_capture->getDataRate()));
		//ui.g_rightcam_bw_lineedit->setText(QString::number(m_right_capture->getDataRate()));

		cvNamedWindow("Left Camera", CV_WINDOW_NORMAL);
		cvNamedWindow("Right Camera", CV_WINDOW_NORMAL);
		cvResizeWindow("Left Camera", STREAM_WIN_WIDTH, STREAM_WIN_HEIGHT);
		cvResizeWindow("Right Camera", STREAM_WIN_WIDTH, STREAM_WIN_HEIGHT);
		cvMoveWindow("Left Camera", 20, 0);
		cvMoveWindow("Right Camera", 20, STREAM_WIN_HEIGHT + 10);

		// TO DO : parallel and capturing from diff. cameras
		while(true) {
			// m_right_temp_image = m_right_capture->xiQImage2IplImage();
			strcpy(m_left_temp_image->imageData, m_left_capture->xiQImage2IplImage()->imageData);
			strcpy(m_right_temp_image->imageData, m_left_temp_image->imageData);
			// allocate memory for shared pointer
			m_left_image  = boost::make_shared<IplImage>(*m_left_temp_image);
			m_right_image = boost::make_shared<IplImage>(*m_right_temp_image);
			cvShowImage("Left Camera", m_left_temp_image);
			cvShowImage("Right Camera", m_right_temp_image);
			// set cameras temperatures
			ui.g_leftcam_temp_lineedit->setText(QString::number(m_left_capture->getTemperature()));
			ui.g_rightcam_temp_lineedit->setText(QString::number(m_left_capture->getTemperature()));
			//ui.g_rightcam_temp_lineedit->setText(QString::number(m_right_capture->getTemperature()));
			cvWaitKey(1);
		}
	//}
	//else {
	//	std::cerr << "Data Acquisition can not started" << std::endl;
	//	QMessageBox message_box(this);
 //       QAbstractButton *ok_button = message_box.addButton(tr("Tamam"), QMessageBox::ActionRole);

 //       // set window title
 //       message_box.setWindowTitle("Veri Alma Hatasý");
 //       // set message of message box
 //       message_box.setText("Kameralardan veri alma iþlemi baþarýsýz.\nLütfen sistemi yeniden baþlatýnýz.");
 //       // execute message box
 //       message_box.exec();

 //       // after message_box execution, close window
 //       if(message_box.clickedButton() == ok_button)
 //           message_box.close();

 //       return false;
	//}
}

// function that captures frame from left cameras
void ABRobotScanner::leftCameraCapturing(const string dir_name, const string file_name)
{
    std::stringstream ss("");

    // create an unique id with random_generator
    if(file_name == "")
        ss << dir_name << "\\" << boost::uuids::random_generator()() << ".bmp";
    else
        ss << dir_name << "\\" << file_name << ".bmp";

    // save image with generated unique id
    cvSaveImage(ss.str().c_str(), m_left_image.get());
}

// function that captures frame from left cameras
void ABRobotScanner::rightCameraCapturing(const string dir_name, const string file_name)
{
    std::stringstream ss("");

    // create an unique id with random_generator
    if(file_name == "")
        ss << dir_name << "\\" << boost::uuids::random_generator()() << ".bmp";
    else
        ss << dir_name << "\\" << file_name << ".bmp";

    // save image with generated unique id
    cvSaveImage(ss.str().c_str(), m_right_image.get());
}

// function that captures frame from left cameras
void ABRobotScanner::twoCameraCapturing()
{
    if(ui.g_camcalib_radiobutton->isChecked()) {
        // save calibration images in left and right cameras directories
        leftCameraCapturing(LEFT_CAM_DIR);
        rightCameraCapturing(RIGHT_CAM_DIR);
    }
    else if(ui.g_spacecalib_radiobutton->isChecked()) {
        // save correspondence images in correspondence images directory
        leftCameraCapturing(CORR_DIR, "left");
        rightCameraCapturing(CORR_DIR, "right");
    }
}

// function that clears cache frames
void ABRobotScanner::clearCache()
{
    std::stringstream ss1(""), ss2(""), ss3(""), ss4("");
    int result;

#ifdef __linux__
    // remove left cameras images directory
    ss1 << "rm -r " << LEFT_CAM_DIR;
    // remove right cameras images directory
    ss2 << "rm -r " << RIGHT_CAM_DIR;
    // remove correspondence images directory
    ss3 << "rm -r " << CORR_DIR;
    // remove reconstruction images directory
    ss4 << "rm -r " << RECONS_DIR;
#elif _WIN32
    // remove left cameras images directory
    ss1 << "del /S /F /Q " << LEFT_CAM_DIR;
    // remove right cameras images directory
    ss2 << "del /S /F /Q " << RIGHT_CAM_DIR;
    // remove correspondence images directory
    ss3 << "del /S /F /Q " << CORR_DIR;
    // remove reconstruction images directory
    ss4 << ">del /S /F /Q " << RECONS_DIR;
#else
	std::cerr << "OS Incompatibility" << std::endl;
	exit(EXIT_FAILURE);
#endif

	result = system(ss1.str().c_str());
    result = system(ss2.str().c_str());
    result = system(ss3.str().c_str());
    result = system(ss4.str().c_str());

    // create a message box about removing process
    QMessageBox message_box(this);
    QAbstractButton *ok_button = message_box.addButton(tr("Tamam"), QMessageBox::ActionRole);

    // set window title
    message_box.setWindowTitle("Cache Temizleme");
    // set message of message box
    message_box.setText("Cache temizlendi.\n");
    // execute message box
    message_box.exec();
    // after message_box execution, close window
    if(message_box.clickedButton() == ok_button)
        message_box.close();
}

// function that calibrates two cameras
bool ABRobotScanner::calibration()
{
    // if user
    if(ui.g_squaresize_box->value() == 0.0 || ui.g_rowcornerscount_box->value() == 0 || ui.g_columncornerscount_box->value() == 0) {
        QMessageBox message_box(this);
        QAbstractButton *ok_button = message_box.addButton(tr("Tamam"), QMessageBox::ActionRole);

        // set window title
        message_box.setWindowTitle("Kalibrasyon");
        // set message of message box
        message_box.setText("Lutfen kalibrasyon verilerini tam olarak giriniz.\n");
        // execute message box
        message_box.exec();

        // after message_box execution, close window
        if(message_box.clickedButton() == ok_button)
            message_box.close();

        return false;
    }

    // allocate memory for calibration objects
    m_ab_calibration_1 = new ABCalibration(LEFT_CAM_DIR);
    m_ab_calibration_2 = new ABCalibration(RIGHT_CAM_DIR);

    // set chess board attributes of first camera
    m_ab_calibration_1->setChessboardAttributes(ui.g_squaresize_box->value(),          // square size of chessboard
                                                ui.g_rowcornerscount_box->value(),     // vertical(row) corners count
                                                ui.g_columncornerscount_box->value()   // horizontal(column) corners count
    );
    // set chess board attributes of second camera
    m_ab_calibration_2->setChessboardAttributes(ui.g_squaresize_box->value(),          // square size of chessboard
                                                ui.g_rowcornerscount_box->value(),     // vertical(row) corners count
                                                ui.g_columncornerscount_box->value()   // horizontal(column) corners count
    );

    // start calibration
    m_ab_calibration_1->run(LEFT_CAM);
    m_ab_calibration_2->run(RIGHT_CAM);

    // load matrices
    m_intrinsic_matrix_1 = (CvMat *)cvLoad("intrinsic_matrix_1.xml");
    m_intrinsic_matrix_2 = (CvMat *)cvLoad("intrinsic_matrix_2.xml");
    m_dis_coef_1 = (CvMat *)cvLoad("distortion_coefficient_1.xml");
    m_dis_coef_2 = (CvMat *)cvLoad("distortion_coefficient_2.xml");

    return true;
}

// function that refines system
void ABRobotScanner::refineSystem()
{
	boost::thread usb30_thread(system, "Intel\\Setup.exe -s -report c:\\windows\\temp\\USB_Intel_setup.log");
	usb30_thread.join();
}

// function that shows an article about space and camera calibration
void ABRobotScanner::howToCalibrating()
{
    boost::thread htc_thread = boost::thread(system, "app\\adobe_reader doc\\calibration.pdf");
}

// function that detects features of images
void ABRobotScanner::detectFeatures()
{
    char left_image_path[MAX_CHAR];
    char right_image_path[MAX_CHAR];
    
	// set image path
    sprintf_s(left_image_path, "%s\\left0.bmp", CORR_DIR);
    sprintf_s(right_image_path, "%s\\right0.bmp",  CORR_DIR);
    // distorted images
    IplImage *left_image  = cvLoadImage(left_image_path);
    IplImage *right_image = cvLoadImage(right_image_path);
    // extract features
	m_ab_fd = new ABFeatureDetector(e_SIFT, left_image, right_image);
	m_ab_fd->extractCorrespondedPoints();
}

// function that estimates fundamental matrix
void ABRobotScanner::fundamentalMatrix()
{
    m_ab_fm = new ABFundamentalMatrix(m_ab_fd->getObjectPoints(), m_ab_fd->getScenePoints());
    // find fundamental matrix
    m_fundamental_matrix = m_ab_fm->findFundamentalMat();
    // print fundamental matrix on screen
    printMatrix("Fundamental Matrix", m_fundamental_matrix);
    // save fundamental matrix
    cvSave("fundamental_matrix.xml", m_fundamental_matrix);
}

// function that calculates epipolar lines
void ABRobotScanner::epipolarLines()
{
    char left_image_path[MAX_CHAR];
    char right_image_path[MAX_CHAR];
    
	// set image path
    sprintf_s(left_image_path, "%s\\left.bmp", CORR_DIR);
    sprintf_s(right_image_path, "%s\\right.bmp",  CORR_DIR);
    // distorted images
    IplImage *left_image  = cvLoadImage(left_image_path);
    IplImage *right_image = cvLoadImage(right_image_path);
    // memory allocate for epipolar lines objects
    m_ab_epilines_1 = new ABEpipolarLine(m_ab_fm->getLeftImagePoints(),  m_fundamental_matrix);
    m_ab_epilines_2 = new ABEpipolarLine(m_ab_fm->getRightImagePoints(), m_fundamental_matrix);
    // find epipolar lines
    m_ab_epilines_1->findEpipolarLines(1);
    m_ab_epilines_2->findEpipolarLines(2);
    // find epipolar points
    m_ab_epilines_1->findEpipolarPoints(left_image->width);
    m_ab_epilines_2->findEpipolarPoints(right_image->width);
    // draw epipolar lines
    m_ab_epilines_1->drawEpipolarLines(right_image, m_ab_fm->getLeftImagePoints(), m_ab_fm->getRightImagePoints(), m_ab_epilines_1->getEpipolarLines(), m_ab_epilines_1->getEpipolarPoints());
    m_ab_epilines_2->drawEpipolarLines(left_image, m_ab_fm->getRightImagePoints(), m_ab_fm->getLeftImagePoints(),  m_ab_epilines_2->getEpipolarLines(), m_ab_epilines_2->getEpipolarPoints());

    // show results
    cvNamedWindow("Left Image",  CV_WINDOW_NORMAL);
    cvNamedWindow("Right Image", CV_WINDOW_NORMAL);
    cvResizeWindow("Left Image",  STREAM_WIN_WIDTH, STREAM_WIN_HEIGHT);
    cvResizeWindow("Right Image", STREAM_WIN_WIDTH, STREAM_WIN_HEIGHT);
    cvShowImage("Left Image",  left_image);
    cvShowImage("Right Image", right_image);
    cvWaitKey();
    cvDestroyWindow("Left Image");
    cvDestroyWindow("Right Image");
	// release image
    cvReleaseImage(&left_image);
    cvReleaseImage(&right_image);
}

// function that estimates essential matrix
void ABRobotScanner::essentialMatrix()
{
    m_ab_em = new ABEssentialMatrix(m_intrinsic_matrix_1, m_intrinsic_matrix_2, m_fundamental_matrix);
    m_ab_em->setImagePointVectors(m_ab_fm->getLeftImagePoints(), m_ab_fm->getRightImagePoints());
    // find essential matrix
    m_essential_matrix = m_ab_em->findEssentialMat();
    // print essential matrix on screen
    printMatrix("Essential Matrix", m_essential_matrix);
    // save essential matrix
    cvSave("essential_matrix.xml",m_essential_matrix);
}

// function that estimates rotation and translation matrices between cameras
void ABRobotScanner::rotationTranslation()
{
    // estimate rotation and translation matrices
    m_rotation_matrix_1 = m_ab_em->getRotationMatrix1();
    m_rotation_matrix_2 = m_ab_em->getRotationMatrix2();
    m_translation_matrix_1 = m_ab_em->getTranslationMatrix1();
    m_translation_matrix_2 = m_ab_em->getTranslationMatrix2();

    // print rotation matrices on screen
    printMatrix("Rotation Matrix 1", m_rotation_matrix_1);
    cvSave("rotation_matrix_1.xml",  m_rotation_matrix_1);
    printMatrix("Rotation Matrix 2", m_rotation_matrix_2);
    cvSave("rotation_matrix_2.xml",  m_rotation_matrix_2);

    // print translation matrices on screen
    printMatrix("Translation Matrix 1", m_translation_matrix_1);
    cvSave("translation_matrix_1.xml",  m_translation_matrix_1);
    printMatrix("Translation Matrix 2", m_translation_matrix_2);
    cvSave("translation_matrix_2.xml",  m_translation_matrix_2);
}

// function that estimate projection matrices
void ABRobotScanner::projectionMatrices()
{
    // estimate projection matrices
    m_ab_reconstruction = new ABReconstruction(m_intrinsic_matrix_1, m_intrinsic_matrix_2, m_rotation_matrix_1, m_translation_matrix_2);
    m_projection_matrix_1 = m_ab_reconstruction->getFirstProjectionMatrix();
    m_projection_matrix_2 = m_ab_reconstruction->getSecondProjectionMatrix();

    // print rectification projection matrix 1 on screen
    printMatrix("Projection Matrix 1", m_projection_matrix_1);
    cvSave("projection_matrix_1.xml",  m_projection_matrix_1);
    // print rectification projection matrix 2 matrix on screen
    printMatrix("Projection Matrix 2", m_projection_matrix_2);
    cvSave("projection_matrix_2.xml",  m_projection_matrix_2);
}

// function that reconstructs 3D Model
void ABRobotScanner::reconstruction()
{
    char create_dir_command[MAX_CHAR];
    int result;

#ifdef __linux__
    sprintf_s(create_dir_command, "%s%s","mkdir -p ", RECONS_DIR);
    result = system(create_dir_command);
    bzero(create_dir_command, MAX_CHAR);
#elif _WIN32
    sprintf_s(create_dir_command, "%s%s","md ", RECONS_DIR);
    result = system(create_dir_command);
    memset(create_dir_command, MAX_CHAR, '\0');
#else
    std::cerr << "OS Incompatibility" << std::endl;
#endif

    m_reconstruction_t[0] = boost::thread(boost::bind(&ABRobotScanner::setReconstructionDataset, this));
    // join thread ?
    //m_reconstruction_t[0].join();
}

// function that captures frames as independent from startSystem function
void ABRobotScanner::setReconstructionDataset()
{
	char left_image_path[MAX_CHAR];
	char right_image_path[MAX_CHAR];
	IplImage *left_image  = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U, 3);
	IplImage *right_image = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U, 3);

    for(int i=0 ; i < RECONS_FRAME_RATE ; ++i) {
        // get first frame of video and put into projector
        // run projector
		
        // set image path
		memset(left_image_path, '\0', MAX_CHAR);
		memset(right_image_path, '\0', MAX_CHAR);
		sprintf_s(left_image_path, "%s\\left%d.bmp", CORR_DIR, i);
		sprintf_s(right_image_path, "%s\\right%d.bmp", CORR_DIR, i);
        // open images
        left_image  = cvLoadImage(left_image_path);
        right_image = cvLoadImage(right_image_path);
        // set images and extract features from images
        m_ab_fd->setImages(left_image, right_image);
        m_ab_fd->extractCorrespondedPoints();
        // reconstruct 3D space
        m_ab_reconstruction->triangulatePoints(m_ab_fd->getObjectPoints(), m_ab_fd->getScenePoints(), m_fundamental_matrix);
        // sleep thread as 0.1 second
        boost::this_thread::sleep(boost::posix_time::seconds(0.1));
    }

	// release images
	cvReleaseImage(&left_image);
	cvReleaseImage(&right_image);
}

// function that show visual 3D Model
void ABRobotScanner::visualModel()
{
    char file_path[MAX_CHAR];
    CvMat *xml_file;
    pcl::PointXYZ one_point;
	int file_index = 0;

 //   // read all files into reconstruction directory
	//for(int file_index = 0 ; file_index < RECONS_FRAME_RATE ; ++file_index) {
 //       // zero file_path
 //       memset(file_path, '\0', MAX_CHAR);
 //       // set .xml file path
 //       sprintf_s(file_path, "%s\\%d.xml", RECONS_DIR, file_index);
 //       // load .xml file
 //       xml_file = (CvMat *)cvLoad(file_path);

 //       // copy all .xml files to PCL data vector
 //       for(int i=0 ; i < xml_file->cols ; ++i) {
 //           one_point.x = CV_MAT_ELEM(*xml_file, double, 0, i) / CV_MAT_ELEM(*xml_file, double, 3, i);
 //           one_point.y = CV_MAT_ELEM(*xml_file, double, 1, i) / CV_MAT_ELEM(*xml_file, double, 3, i);
 //           one_point.z = CV_MAT_ELEM(*xml_file, double, 2, i) / CV_MAT_ELEM(*xml_file, double, 3, i);
 //           // add point to surface
 //           m_surface_points.push_back(one_point);
 //       }
 //   } // end of for

	    // read all files into reconstruction directory
	for(int file_index = 0 ; file_index < RECONS_FRAME_RATE ; ++file_index) {
        // zero file_path
        memset(file_path, '\0', MAX_CHAR);
        // set .xml file path
        sprintf_s(file_path, "%s\\point_cloud.xml", RECONS_DIR);
        // load .xml file
        xml_file = (CvMat *)cvLoad(file_path);

        // copy all .xml files to PCL data vector
        for(int i=0 ; i < xml_file->cols ; ++i) {
            one_point.x = CV_MAT_ELEM(*xml_file, double, 0, i);
            one_point.y = CV_MAT_ELEM(*xml_file, double, 1, i);
            one_point.z = CV_MAT_ELEM(*xml_file, double, 2, i);
            // add point to surface
            m_surface_points.push_back(one_point);
        }
    } // end of for

	m_ab_vm->setPointCloud(m_surface_points);
	// resize GL and update point cloud
	m_ab_vm->resizeGL(PC_WIDTH, PC_HEIGHT);
	m_ab_vm->updatePointCloud(m_surface_points);
	// add widget on grid
	ui.m_visualization_gridlayout->addWidget(m_ab_vm);
}

// function that saves point cloud data
void ABRobotScanner::savePointCloud()
{
    // save point cloud as binary format
    if(0 != m_surface_points.size()) {
        if(ui.g_xyzext_radiobutton->isChecked())
			savePointCloudXYZFormat(XYZ_FILE_NAME, m_surface_points);
        else
            pcl::io::savePLYFileBinary<pcl::PointXYZ>(PLY_FILE_NAME, m_surface_points);

        std::cout << "Point Cloud is saved" << std::endl;
    }
    else
        std::cerr << "Point Cloud is empty" << std::endl;

	return;
}

// function that shows help file
void ABRobotScanner::helpFile()
{
    boost::thread hf_thread = boost::thread(system, "app\\adobe_reader doc\\helpfile.pdf");
}

// function that terminates application
void ABRobotScanner::exit()
{
    /* TO DO : this comment will be removen */
    /*
    int result;

    // remove correspondence images directory
    stringstream ss("");
    ss << "rm -r " << CORR_DIR;
    result = system(ss.str().c_str());
    */

    // close application
	if(m_left_capture)
		m_left_capture->closeDevice();
	//if(m_right_capture)
	//	m_right_capture->closeDevice();

    this->close();
	//m_right_capture->closeDevice();
    ::exit(EXIT_FAILURE);
}

// function that converts IplImage to QImage
QImage ABRobotScanner::iplImage2QImage(const IplImage *ipl_image, const int width, const int height)
{
    // return Qt image
    return QImage((const uchar *)ipl_image->imageData, ipl_image->width, ipl_image->height, QImage::Format_RGB888).rgbSwapped().scaled(width, height);
}

// function that converts xiQImage to QImage
QImage ABRobotScanner::xiQImage2QImage(const XI_IMG xiQ_image, const int width, const int height)
{
    // return Qt image
	return QImage((const uchar *)xiQ_image.bp, xiQ_image.width, xiQ_image.height, QImage::Format_RGB888).rgbSwapped().scaled(width, height);
}

// function that save point could by .xyz file extension format
bool ABRobotScanner::savePointCloudXYZFormat(const char *file_name, pcl::PointCloud<pcl::PointXYZ> &point_cloud)
{
	FILE *xyz_file = NULL;

	// open file
	if((xyz_file = fopen(file_name, "w")) == NULL) {
		std::cerr << "FILE CAN NOT SAVED" << std::endl;
		return false;
	}

	// write datas into file whose extension is xyz
	for(int i = 0 ; i < point_cloud.size() ; ++i)
		fprintf_s(xyz_file, "%f %f %f%s", point_cloud.points.at(i).x, point_cloud.points.at(i).y, point_cloud.points.at(i).z, "\n");

	// close file
	fclose(xyz_file);

	return true;
}

// function that print matrix on screen
void ABRobotScanner::printMatrix(const char *matrix_name, const CvMat *matrix)
{
    // print m matrix on screen
    std::cout << std::endl << "--------------------------------" << std::endl << matrix_name << std::endl << "--------------------------------" << std::endl;

    for(int i=0 ; i < matrix->rows ; ++i) {
        for(int j=0 ; j < matrix->cols ; ++j) {
            std::cout << setw(16) << CV_MAT_ELEM(*matrix, double, i, j)  << " ";
        }
        std::cout << std::endl;
    }
}