
#ifndef OUTPUT_FILE
#define OUTPUT_FILE
void printit(Eigen::Vector4d X);
void writeDataToFile(std::vector<CombinedInputData> input_data,SensorFusion FusionObject);
void writeDataToFile2(std::vector<CombinedInputData> input_data,SensorFusion FusionObject);

#endif