#include "Declarations.h"



    long long timestamp_;
    enum sensorSource{
            LIDAR,
            RADAR
        } sensorSource;
        Eigen::VectorXd measurementdetectionData;
        Eigen::VectorXd groundTruthData;

        void CombinedInputData::setMeasurementData(Eigen :: VectorXd Input)
        {
            if(sensorSource == RADAR)
            {
                measurementdetectionData = radarPolarToCartesian(Input);
            }
            else
            {
                measurementdetectionData = Input;
            }
        }

        void CombinedInputData::setGroundTruthData(Eigen :: VectorXd Groudtruth)
        {
            groundTruthData = Groudtruth;
        }

        void CombinedInputData::setTimestamp(long long input)
        {
            timestamp_ = input;
        }

        Eigen :: VectorXd CombinedInputData::radarPolarToCartesian(Eigen :: VectorXd polarData)
        {
            float r,rdot,phi,x,y,vx,vy;
            r = polarData[0];
            phi = polarData[1];
            rdot = polarData[2];

            x = r*cos(phi);
            y = r*sin(phi);
            vx = rdot * cos(phi);
            vy = rdot * sin(phi);

            Eigen::VectorXd measdata = Eigen::VectorXd(4);
            measdata << x,y,vx,vy;
            return measdata;

        }

       KalmanFilter::KalmanFilter()
       {
           InitPMatrix();
           InitRMatrixRadar();
           InitHMatrixLidar();
           InitRMatrixLidar();
           InitQMatrix();
       }
       Eigen::MatrixXd H_radar = Eigen::MatrixXd(3,4);
       Eigen::MatrixXd F = Eigen::MatrixXd(4,4);
       Eigen::MatrixXd P = Eigen::MatrixXd(4,4);
       Eigen::MatrixXd R_radar = Eigen::MatrixXd(3,3) ;
       Eigen::MatrixXd R_lidar = Eigen::MatrixXd(2,2);
       Eigen::MatrixXd H_lidar = Eigen::MatrixXd(2,4);
       Eigen::MatrixXd Q = Eigen::MatrixXd(4,4);
       Eigen::Vector4d X;

       Eigen::Vector4d KalmanFilter::Predict(float cycletime)
      {
          setFmatrix(cycletime);
          Eigen::Vector4d X_temp;
          Eigen::MatrixXd Ft, P_temp;
          X_temp =  F * X;
          X = X_temp;

          Ft = F.transpose();
          P_temp = F*P*Ft + Q;
          P = P_temp;

      }
      void KalmanFilter::setStateVector(Eigen::Vector4d initStateVector)
      {
          X = initStateVector;
      }

      void KalmanFilter::setFmatrix(float cycletime)
       {
           
           F << 1, 0, cycletime, 0,
                0, 1,  0, cycletime,
                0, 0,  1 , 0,
                0, 0,  0,  1;
            
       }
       void KalmanFilter::InitPMatrix()
       {
           P << 10, 0, 0,  0,
                0, 10,  0, 0,
                0, 0,  100 , 0,
                0, 0,  0,  100;

       }

       void KalmanFilter::InitRMatrixRadar()
       {
           R_radar << 0.5, 0, 0,
                0, 0.1,  0,
                0, 0,  0.3 ;

       }
       void KalmanFilter::InitHMatrixLidar()
       {
           H_lidar << 1, 0, 0, 0,
                      0, 1, 0, 0;

       }
       void KalmanFilter::InitRMatrixLidar()
       {
           R_lidar << 0.5, 0,
                      0, 0.5;
       }
       void KalmanFilter::InitQMatrix()
       {
           Q << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
       }
      void KalmanFilter::Update(Eigen::VectorXd Z,bool flag)
       {
           Eigen::MatrixXd H,Ht,K,S,Sinv,I,KHP,R;
           Eigen::Vector4d X_temp;
           Eigen::VectorXd Innovation, HX_radar;

           if(flag)
           {
               H = H_lidar;
               R = R_lidar;
               Innovation = Z - H*X;

           }
           else
           {
               H = H_radar;
               R = R_radar;
               
               float r = sqrt(X[0]*X[0] + X[1]*X[1]);
               float phi = atan2(X[1],X[0]);
               float rdot = X[2]*cos(phi) + X[3]*sin(phi);
               HX_radar = Eigen::VectorXd(3);
               HX_radar << r,phi,rdot;
               Innovation = Z - HX_radar;

           }
       
           Ht = H.transpose();
           S = H*P*Ht + R;
           Sinv = S.inverse();
           K = P*Ht*Sinv;

           X_temp = X + K*Innovation;
           X = X_temp;


           KHP = K*H*P;
           P = P - KHP;
       }
       void KalmanFilter::calculateHRadar()
       {
           float x = X[0];
           float y = X[1];
           float vx = X[2];
           float vy = X[3];

           float rSquared = x*x + y*y;
           float r = sqrt(rSquared);
           float denom = r* rSquared;

           float h00 = x/r;
           float h01 = y/r;
           float h10 =-y/rSquared;
           float h11 = x/rSquared;
           float h20 = y*(y*vx-x*vy)/denom;
           float h21 = x*(x*vy-y*vx)/denom;
           float h22 = h00;
           float h23 = h01;
           
           H_radar << h00,h01,0,0,
                      h10,h11,0,0,
                      h20,h21,h22,h23;
       }

       Eigen::Vector4d KalmanFilter::getCurrentState()
       {
           return X;
       }





       SensorFusion::SensorFusion():counter(0)
       {

       }




       Eigen::Vector4d SensorFusion::fusedStateX()
       {
           return Kalman.getCurrentState();
       }

       void SensorFusion::Filteralgo(CombinedInputData CurrentCycleData)
       {
           if(counter == 0)
           {
               if(CurrentCycleData.sensorSource == CombinedInputData::RADAR)
               {
                   float r = CurrentCycleData.measurementdetectionData[0];
                   float phi = CurrentCycleData.measurementdetectionData[1];
                   float rdot = CurrentCycleData.measurementdetectionData[2];

                   X[0] = r*cos(phi);
                   X[1] = r*sin(phi);
                   X[2] = rdot * cos(phi);
                   X[3] = rdot * sin(phi);
                   X[0] = abs(X[0]) < 0.001 ? 0.001 :X[0];
                   X[1] = abs(X[1]) < 0.001 ? 0.001 :X[1];
                   X[2] = abs(X[2]) < 0.001 ? 0.001 :X[2];
                   X[3] = abs(X[3]) < 0.001 ? 0.001 :X[3];
                   
               }
               else
               {
                   X[0] = CurrentCycleData.measurementdetectionData[0];
                   X[1] = CurrentCycleData.measurementdetectionData[1];
                   X[2] = 0;
                   X[3] = 0;
                   X[0] = abs(X[0]) < 0.001 ? 0.001 :X[0];
                   X[1] = abs(X[1]) < 0.001 ? 0.001 :X[1];
               }
               lasttimestamp = CurrentCycleData.timestamp_;
               Kalman.setStateVector(X);
               combinedState.push_back(Kalman.getCurrentState());
               counter ++;
           }
           else
           {
               cycletime = (CurrentCycleData.timestamp_ - lasttimestamp)/1000000.0;
               
               Kalman.Predict(cycletime);
               if(CurrentCycleData.sensorSource == CombinedInputData::LIDAR)
               {
                   Kalman.Update(CurrentCycleData.measurementdetectionData,true);
                   
               }
               else
               {
                   Kalman.calculateHRadar();
                   Kalman.Update(CurrentCycleData.measurementdetectionData,false);
               }
               combinedState.push_back(Kalman.getCurrentState());
               counter++;

           }

       }        

