#include "../Algo/Declarations.h"

std::vector<CombinedInputData> getInput(std::string input_file_name)
{
      std::ifstream input_file(input_file_name.c_str(),std::ifstream::in);

      if (!input_file.is_open()) {
        std::cout << "Failed to open : " << input_file_name << std::endl;
        exit(0);
      }

        std::vector<CombinedInputData> InputData;
        std::string line;
        while(getline(input_file,line))
        {
            CombinedInputData Currentdata;
            std::string sensor;
            std::istringstream iss(line);
            long long timestamp;
            iss >> sensor;//The first line is the sensor identification data. Either L or R.

            if(sensor.compare("L") == 0)
            {
                Currentdata.sensorSource = CombinedInputData::LIDAR;
                float x,y;
                iss >> x;
                iss >> y;
                // Eigen::VectorXd input = Eigen::VectorXd(2);
                // input << x,y;
                // Currentdata.setMeasurementData(input);
                Currentdata.measurementdetectionData = Eigen::VectorXd(2);
                Currentdata.measurementdetectionData << x,y;
                
            }
            else if(sensor.compare("R") == 0)
            {
                Currentdata.sensorSource = CombinedInputData::RADAR;
                float rho,phi,rhodot;
                iss >> rho;
                iss >> phi;
                iss >> rhodot;
                // Eigen::VectorXd input = Eigen::VectorXd(3);
                // input << rho,phi,rhodot;
                // Currentdata.setMeasurementData(input);
                Currentdata.measurementdetectionData = Eigen::VectorXd(3);
                Currentdata.measurementdetectionData << rho,phi,rhodot;

            }
            iss >> timestamp;
            Currentdata.setTimestamp(timestamp);
            float x_gt, y_gt,vx_gt,vy_gt;
            iss >> x_gt;
            iss >> y_gt;
            iss >> vx_gt;
            iss >> vy_gt;
            // Eigen::VectorXd input_gt = Eigen::VectorXd(4);
            // input_gt << x_gt, y_gt,vx_gt,vy_gt;
            // Currentdata.setGroundTruthData(input_gt);
            Currentdata.groundTruthData = Eigen::VectorXd(4);
            Currentdata.groundTruthData << x_gt, y_gt,vx_gt,vy_gt;
            InputData.push_back(Currentdata);

        }
        std::cout<<"Data Loaded"<<std::endl;
        return  InputData;   
}
