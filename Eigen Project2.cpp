#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include<Eigen/Eigen>
#include <vector>
#include <math.h>
using namespace Eigen;
using namespace std;

vector<string> ObjectByKamera[6];
vector<string> ObjectByKinect[6];
vector<string> ObjectByDerinlik[6];
vector<string> ObjectByWorld[6];

MatrixXi RobotStatus;

MatrixXi randomMatrix(int LowValue, int HighValue, int row, int column) {
    double range = HighValue - LowValue;
    MatrixXd m = MatrixXd::Random(row, column);
    m = (m + MatrixXd::Constant(row, column, 1.)) * range / 2.;
    m = (m + MatrixXd::Constant(row, column, LowValue));
    MatrixXi mint = m.cast<int>();
    return mint;
}

VectorXi randomVector(int LowValue, int HighValue, int size) {
    double range = HighValue - LowValue;
    VectorXd m = VectorXd::Random(size);
    m = (m + VectorXd::Constant(size, 1.)) * range / 2.;
    m = (m + VectorXd::Constant(size, LowValue));
    VectorXi mint = m.cast<int>();
    return mint;
}


void definition() {
    ObjectByKamera[0] = { "iris_robot", "8.150", "1.544", "-1.47945", "30", "0","0" };
    ObjectByKamera[1] = { "create robot","10.623","0.544","-2.719","0","0","0" };
    ObjectByKamera[2] = { "r2 robot","10.367","2.412","-1.643","-45","-60","50" };

    ObjectByKinect[0] = { "Husky Robot","3.837","-2.234","0.479","0","40","30" };
    ObjectByKinect[1] = { "yourobot","4.289","3.189","0.479","30","0","35" };
    ObjectByKinect[2] = { "turtlebot3_waffle","2.480","4.849","0.479","0","45","25" };

    ObjectByDerinlik[0] = { "pioneer3at","6.149","5.281","0.265","20","15","65" };
    ObjectByDerinlik[1] = { "pioneer2dx","0.948","6.729","0.265","-20","-15","0" };
    ObjectByDerinlik[2] = { "mars rover","9.378","6.643","1.265","-30","45","-15" };

    ObjectByWorld[0] = { "kamera","-5.538","-8.437","2.549","15","-45","25" };
    ObjectByWorld[1] = { "kinect","-3.800","-3.744","0","0","30","0" };
    ObjectByWorld[2] = { "derinlik","5.949","-2.590","0","45","45","0" };
    ObjectByWorld[3] = { "kutu","6.954","-7.144","0.149","0","0","0" };
    ObjectByWorld[4] = { "tuğla","6.954","-7.144","0.149","0","0","0" };
    ObjectByWorld[5] = { "geniş tuğla","5.618","-9.217","0","0","0","0" };
    ObjectByWorld[6] = { "varil","0.401","-8.934","0","0","0","0" };
    ObjectByWorld[7] = { "cop kutusu","1.458","-2.732","0","0","0","0" };
    ObjectByWorld[8] = { "kereste","4.435","-0.911","0.04","0","0","0" };
    ObjectByWorld[9] = { "tekerlek","2.414","-1.050","0.02","0","0","0" };

    RobotStatus = randomMatrix(0, 1, 3, 3);
}

void FindWorldPosition(vector<string>& Camera, vector<string>& Object, double* data) {
    data[0] = stod(Camera[1]) + stod(Object[1]);
    data[1] = stod(Camera[2]) + stod(Object[2]);
    data[2] = stod(Camera[3]) + stod(Object[3]);
}

void FindTargetPosition(vector<string>& Target, double* Robot, double* result) {
    result[0] = 0;
    result[1] = 0;
    result[2] = 0;

    if (stod(Target[1]) < Robot[0]){
        result[0] = Robot[0] - stod(Target[1]);
    }
    else {
        result[0] = stod(Target[1]) - Robot[0];
    }

    if (stod(Target[2]) < Robot[1]) {
        result[1] = Robot[1] - stod(Target[2]);
    }
    else {
        result[1] = stod(Target[2]) - Robot[1];
    }

    if (stod(Target[3]) < Robot[2]) {
        result[2] = Robot[2] - stod(Target[3]);
    }
    else {
        result[2] = stod(Target[3]) - Robot[2];
    }

}

void calculateDistance(vector<string>& Target, double* Robot, double* result) {
    result[0] = sqrt(pow(stod(Target[1]) - Robot[0], 2) + pow(stod(Target[2]) - Robot[1], 2) + pow(stod(Target[3]) - Robot[2], 2));
}

int findNearsetRobot(double* Distances,int length) {
    int IMV = -1;

    int MinData = 99999;
    for (int x = 0; x < length; x++) {
        if (Distances[x] < MinData) {
            MinData = Distances[x];
            IMV = x;
        }
    }

    return IMV;
}

int findNearsetRobotWOMAss(double* Distances, int length,double* Mass) {
    int IMV = -1;

    int MinData = 99999;
    for (int x = 0; x < length; x++) {
        if (Distances[x] < MinData && Mass[x] == 0) {
            MinData = Distances[x];
            IMV = x;
        }
    }

    return IMV;
}

void FindCameraPosition(vector<string>& Camera, vector<string>& Object) {
    double result[3];
    result[0] = 0;
    result[1] = 0;
    result[2] = 0;

    if (stod(Camera[1]) < stod(Object[1])) {
        result[0] = stod(Object[1]) - stod(Camera[1]);
    }
    else {
        result[0] = stod(Camera[1]) - stod(Object[1]);
    }

    if (stod(Camera[2]) < stod(Object[2])) {
        result[1] = stod(Object[2]) - stod(Camera[2]);
    }
    else {
        result[1] = stod(Camera[2]) - stod(Object[2]);
    }

    if (stod(Camera[3]) < stod(Object[3])) {
        result[2] = stod(Object[3]) - stod(Camera[3]);
    }
    else {
        result[2] = stod(Camera[3]) - stod(Object[3]);
    }
    cout << "Çöp kutusunun Kamera göre lokasyonu:" << endl;
    cout << "X:" + to_string(result[0]) << endl;
    cout << "Y:" + to_string(result[1]) << endl;
    cout << "Z:" + to_string(result[2]) << endl;
}

int main()
{
    definition();
    double RobotByWorld[9][3];
    double RobotByTarget[9][3];

    FindWorldPosition(ObjectByWorld[0], ObjectByKamera[0], RobotByWorld[0]);
    FindWorldPosition(ObjectByWorld[0], ObjectByKamera[1], RobotByWorld[1]);
    FindWorldPosition(ObjectByWorld[0], ObjectByKamera[2], RobotByWorld[2]);
    FindWorldPosition(ObjectByWorld[1], ObjectByKinect[0], RobotByWorld[3]);
    FindWorldPosition(ObjectByWorld[1], ObjectByKinect[1], RobotByWorld[4]);
    FindWorldPosition(ObjectByWorld[1], ObjectByKinect[2], RobotByWorld[5]);
    FindWorldPosition(ObjectByWorld[2], ObjectByDerinlik[0], RobotByWorld[6]);
    FindWorldPosition(ObjectByWorld[2], ObjectByDerinlik[1], RobotByWorld[7]);
    FindWorldPosition(ObjectByWorld[2], ObjectByDerinlik[2], RobotByWorld[8]);

    FindTargetPosition(ObjectByWorld[7], RobotByWorld[1], RobotByTarget[0]);
    FindTargetPosition(ObjectByWorld[7], RobotByWorld[2], RobotByTarget[1]);
    FindTargetPosition(ObjectByWorld[7], RobotByWorld[3], RobotByTarget[2]);
    FindTargetPosition(ObjectByWorld[7], RobotByWorld[4], RobotByTarget[3]);
    FindTargetPosition(ObjectByWorld[7], RobotByWorld[5], RobotByTarget[4]);
    FindTargetPosition(ObjectByWorld[7], RobotByWorld[6], RobotByTarget[5]);
    FindTargetPosition(ObjectByWorld[7], RobotByWorld[7], RobotByTarget[6]);
    FindTargetPosition(ObjectByWorld[7], RobotByWorld[8], RobotByTarget[7]);
    FindTargetPosition(ObjectByWorld[7], RobotByWorld[9], RobotByTarget[8]);

    double RobotDistance[9][1];
    for (int i = 0; i < 9; i++) {
        calculateDistance(ObjectByWorld[7], RobotByTarget[i], RobotDistance[i]);
    }

    double minRobotCamera[3], minRobotKinect[3], minRobotDirect[3], minRobotAll[9];
    minRobotAll[0] = minRobotCamera[0] = RobotDistance[0][0];
    minRobotAll[0] = minRobotCamera[1] = RobotDistance[1][0];
    minRobotAll[0] = minRobotCamera[2] = RobotDistance[2][0];
    minRobotAll[0] = minRobotCamera[0] = RobotDistance[0][0];
    minRobotAll[0] = minRobotCamera[1] = RobotDistance[1][0];
    minRobotAll[0] = minRobotCamera[2] = RobotDistance[2][0];
    minRobotAll[0] = minRobotCamera[0] = RobotDistance[0][0];
    minRobotAll[0] = minRobotCamera[1] = RobotDistance[1][0];
    minRobotAll[0] = minRobotCamera[2] = RobotDistance[2][0];


    int result = findNearsetRobot(minRobotCamera, 3);
    cout << "Kamera'a gore en yakin robot ismi:" + ObjectByKamera[result][0] << endl;
    int result2 = findNearsetRobot(minRobotKinect, 3);
    cout << "Kinect'a gore en yakin robot ismi:" + ObjectByKinect[result2][0] << endl;
    int result3 = findNearsetRobot(minRobotDirect, 3);
    cout << "Derinlik'a gore en yakin robot ismi:" + ObjectByDerinlik[result3][0] << endl;


    int IMV = findNearsetRobot(minRobotAll, 9);
    if (IMV < 4) {
        cout << "En yakin robot ismi:" + ObjectByKamera[IMV][0] << endl;
    }
    else if (IMV < 7) {
        IMV = IMV - 3;
        cout << "En yakin robot ismi:" + ObjectByKinect[IMV][0] << endl;
    }
    else if (IMV < 10) {
        IMV = IMV - 6;
        cout << "En yakin robot ismi:" + ObjectByDerinlik[IMV][0] << endl;
    }
    double CRM[3], KRM[3], DRM[3],ARM[9];
    ARM[0] = CRM[0] = RobotStatus(0);
    ARM[1] = CRM[1] = RobotStatus(1);
    ARM[2] = CRM[2] = RobotStatus(2);
    ARM[3] = KRM[0] = RobotStatus(3);
    ARM[4] = KRM[1] = RobotStatus(4);
    ARM[5] = KRM[2] = RobotStatus(5);
    ARM[6] = DRM[0] = RobotStatus(6);
    ARM[7] = DRM[1] = RobotStatus(7);
    ARM[8] = DRM[2] = RobotStatus(8);
    

    int result2_1 = findNearsetRobotWOMAss(minRobotCamera,3,CRM);
    cout << "Kamera'a gore en yakin agirliksiz robot ismi:" + ObjectByKamera[result2_1][0] << endl;
    int result2_2 = findNearsetRobotWOMAss(minRobotKinect,3,KRM);
    cout << "Kinect'a gore en yakin agirliksiz robot ismi:" + ObjectByKinect[result2_2][0] << endl;
    int result2_3 = findNearsetRobotWOMAss(minRobotDirect,3,DRM);
    cout << "Derinlik'a gore en yakin agirliksiz robot ismi:" + ObjectByDerinlik[result2_3][0] << endl;

    int IMV_WOMass = findNearsetRobotWOMAss(minRobotAll, 9,ARM);
    if (IMV_WOMass != -1) {
        if (IMV_WOMass < 4) {
            cout << "En yakin agirliksiz robot ismi:" + ObjectByKamera[IMV_WOMass][0] << endl;
        }
        else if (IMV_WOMass < 7) {
            IMV_WOMass = IMV_WOMass - 3;
            cout << "En yakin agirliksiz robot ismi:" + ObjectByKinect[IMV_WOMass][0] << endl;
        }
        else if (IMV_WOMass < 10) {
            IMV_WOMass = IMV_WOMass - 6;
            cout << "En yakin agirliksiz robot ismi:" + ObjectByDerinlik[IMV_WOMass][0] << endl;
        }
    }
    else {
        cout << "Bütün robotların agirligi vardir..."<< endl;
    }
    
    FindCameraPosition(ObjectByWorld[ 0 ], ObjectByWorld[ 7 ]);
    
}