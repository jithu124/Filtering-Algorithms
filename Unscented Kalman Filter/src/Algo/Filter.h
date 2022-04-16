#ifndef MEAS_GROUNDTRUTH_DATA
#define MEAS_GROUNDTRUTH_DATA

#include "Eigen/Dense"
#include <cmath>
#include <vector>


class CombinedInputData{
    public :

    long long timestamp_;
    enum sensorSource{
            LIDAR,
            RADAR
        } sensorSource;
        Eigen::VectorXd measurementdetectionData;
        Eigen::VectorXd groundTruthData;

        void setMeasurementData(Eigen :: VectorXd Input);
        void setGroundTruthData(Eigen :: VectorXd Groudtruth);
        void setTimestamp(long long input);

        Eigen :: VectorXd radarPolarToCartesian(Eigen :: VectorXd polarData);
};
class UKF
{
    public:
       UKF();
       Eigen::MatrixXd H_radar = Eigen::MatrixXd(3,4);
       Eigen::MatrixXd F = Eigen::MatrixXd(4,4);
       Eigen::MatrixXd P = Eigen::MatrixXd(4,4);
       Eigen::MatrixXd R_radar = Eigen::MatrixXd(3,3) ;
       Eigen::MatrixXd R_lidar = Eigen::MatrixXd(2,2);
       Eigen::MatrixXd H_lidar = Eigen::MatrixXd(2,4);
       Eigen::MatrixXd Q = Eigen::MatrixXd(4,4);
       Eigen::Vector4d X;

       Eigen::Vector4d Predict(float cycletime);
       void setStateVector(Eigen::Vector4d initStateVector);


       void setFmatrix(float cycletime);
       void InitPMatrix();

       void InitRMatrixRadar();
       void InitHMatrixLidar();
       void InitRMatrixLidar();
       void InitQMatrix();
       void Update(Eigen::VectorXd Z,bool flag);
       void calculateHRadar();

       Eigen::Vector4d getCurrentState();

};

class SensorFusion{

    public:
       UKF UKalman;
       SensorFusion();

       float cycletime;
       float lasttimestamp;
       uint32_t counter;
       Eigen::Vector4d X;
       std::vector <Eigen::Vector4d> combinedState;
       Eigen::Vector4d fusedStateX();
       void Filteralgo(CombinedInputData CurrentCycleData);     


};


#endif
