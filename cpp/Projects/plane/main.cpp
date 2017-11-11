#include <iostream>
#include "ABCalibration.h"
#include "ABVisualModel.h"

using namespace std;

int main()
{
    ABCalibration *calibration = new ABCalibration("dataset2");
    ABVisualModel *visual_model = new ABVisualModel();

    calibration->setChessboardAttributes(2.2, 6, 8);
    calibration->run(1);

    CvMat *corner_points = (CvMat *)cvLoad("image_points.xml");

    visual_model->setPoints(corner_points);
    visual_model->showModel();

    return 0;
}
