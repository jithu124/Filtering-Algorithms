#include "../Eigen/Eigen"
#include <iostream>
#include "../Algo/Declarations.h"

void printit(Eigen::Vector4d X)
{
    std::cout<<X[0]<<" "<<X[1]<<" "<<X[2]<<" "<<X[3]<<std::endl;
}

void writeDataToFile(std::vector<CombinedInputData> input_data,SensorFusion FusionObject)
{
    FILE * f;
    f = fopen("Finaldata.csv","w");
    fprintf(f,"X,Y,Vx,Vy,X_gt,Y_gt,Vx_gt,Vy_gt\n");
    for(long long i = 0; i < FusionObject.combinedState.size(); i++)
    {
        fprintf(f,"%f,%f,%f,%f,%f,%f,%f,%f\n",
        FusionObject.combinedState[i][0],
        FusionObject.combinedState[i][1],
        FusionObject.combinedState[i][2],
        FusionObject.combinedState[i][3],
        input_data[i].groundTruthData[0],
        input_data[i].groundTruthData[1],
        input_data[i].groundTruthData[2],
        input_data[i].groundTruthData[3]);
    }
}
void writeDataToFile2(std::vector<CombinedInputData> input_data,SensorFusion FusionObject)
{
    FILE * f;
    f = fopen("Inputdata.csv","w");
    fprintf(f,"X/R,Y/Phi,Rho,Vy,X_gt,Y_gt,Vx_gt,Vy_gt\n");
    for(long long i = 0; i < input_data.size(); i++)
    {
        if(input_data[i].sensorSource == CombinedInputData::LIDAR)
        {

            fprintf(f,"%f,%f,%f,%f,%f,%f,%lld\n",
            input_data[i].measurementdetectionData[0],
            input_data[i].measurementdetectionData[1],       
            input_data[i].groundTruthData[0],
            input_data[i].groundTruthData[1],
            input_data[i].groundTruthData[2],
            input_data[i].groundTruthData[3],
            input_data[i].timestamp_);
        }
        else if(input_data[i].sensorSource == CombinedInputData::RADAR)
        {

            fprintf(f,"%f,%f,%f,%f,%f,%f,%f,%lld\n",
            input_data[i].measurementdetectionData[0],
            input_data[i].measurementdetectionData[1],  
            input_data[i].measurementdetectionData[2],     
            input_data[i].groundTruthData[0],
            input_data[i].groundTruthData[1],
            input_data[i].groundTruthData[2],
            input_data[i].groundTruthData[3],
            input_data[i].timestamp_);

        }
        else
        {

        }
    }
}