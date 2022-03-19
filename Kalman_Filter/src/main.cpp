#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include "Eigen/Eigen"
#include "Algo/Declarations.h"



int main(){
    std::string input_file_name = "../data/Sensor-data.txt";
    std::vector<CombinedInputData> inputData = getInput(input_file_name);
    SensorFusion FusionObject;
    for(int i = 0; i < inputData.size(); i++)
    {
        FusionObject.Filteralgo(inputData[i]);
        Eigen::Vector4d stateX = FusionObject.fusedStateX();
        printit(stateX);

    }

    writeDataToFile2(inputData,FusionObject);  
    writeDataToFile(inputData,FusionObject); 
    return 0;
}
