#include "pch.h"
#include "HandEyeCalibration.h"
#include "area_scan_3d_camera/HandEyeCalibration.h"
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/api_util.h"


//HandEyeCalibration::HandEyeCalibration()
//{
//    char timeStamp[80];
//    GetTime(timeStamp);
//
//    plog::init(plog::debug, "./Logs/HandEyeCalib.txt", 5000, 3);
//
//    plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
//    plog::get()->addAppender(&consoleAppender);
//
//    //PLOG_INFO << "This is the first log..." << endl;
//}
//
//HandEyeCalibration::~HandEyeCalibration()
//{
//    // 关闭log.
//
//}

__declspec(dllexport) void __stdcall ConvertToGray(const char* inputPath, const char* outputPath)
{
    Mat image = imread(inputPath);

    if (image.empty())
        return;

    Mat grayImage;
    cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    imwrite(outputPath, grayImage);
}

__declspec(dllexport) void __stdcall GetCameraIntrinsic(CameraInstrinsic* cameraIntrinsic)
{
    // 获取相机内参。
    cameraMatrix.at<float>(0, 0) = cameraIntrinsic->Fx;
    cameraMatrix.at<float>(0, 1) = 0;
    cameraMatrix.at<float>(0, 2) = cameraIntrinsic->Cx;

    cameraMatrix.at<float>(1, 0) = 0;
    cameraMatrix.at<float>(1, 1) = cameraIntrinsic->Fy;
    cameraMatrix.at<float>(1, 2) = cameraIntrinsic->Cy;

    cameraMatrix.at<float>(2, 0) = 0;
    cameraMatrix.at<float>(2, 1) = 0;
    cameraMatrix.at<float>(2, 2) = 1;

    // 获取畸变系数。
    distortion.at<float>(0, 0) = cameraIntrinsic->K1;
    distortion.at<float>(0, 1) = cameraIntrinsic->K2;
    distortion.at<float>(0, 2) = cameraIntrinsic->P1;
    distortion.at<float>(0, 3) = cameraIntrinsic->P2;
    distortion.at<float>(0, 4) = cameraIntrinsic->K3;

    cout << "标定前相机内参：" << cameraMatrix << endl;
    cout << "标定前畸变系数：" << distortion << endl;
}

__declspec(dllexport) bool __stdcall CornerDetection(const char* inputPath, CornersPoints* corners)
{
    Settings setParam;

    Size boardSize(setParam.boardWidth, setParam.boardHeight);

    float grid_width = setParam.squareSize * (boardSize.width - 1);
    if (setParam.calibrationPattern == Settings::Pattern::CHARUCOBOARD) {
        grid_width = setParam.squareSize * (boardSize.width - 2);
    }

    //create CharucoBoard
    cv::aruco::Dictionary dictionary;
    if (setParam.calibrationPattern == Settings::CHARUCOBOARD) {
        if (setParam.arucoDictFileName == "") {
            cv::aruco::PredefinedDictionaryType arucoDict;
            if (setParam.arucoDictName == "DICT_4X4_50") { arucoDict = cv::aruco::DICT_4X4_50; }
            else if (setParam.arucoDictName == "DICT_4X4_100") { arucoDict = cv::aruco::DICT_4X4_100; }
            else if (setParam.arucoDictName == "DICT_4X4_250") { arucoDict = cv::aruco::DICT_4X4_250; }
            else if (setParam.arucoDictName == "DICT_4X4_1000") { arucoDict = cv::aruco::DICT_4X4_1000; }
            else if (setParam.arucoDictName == "DICT_5X5_50") { arucoDict = cv::aruco::DICT_5X5_50; }
            else if (setParam.arucoDictName == "DICT_5X5_100") { arucoDict = cv::aruco::DICT_5X5_100; }
            else if (setParam.arucoDictName == "DICT_5X5_250") { arucoDict = cv::aruco::DICT_5X5_250; }
            else if (setParam.arucoDictName == "DICT_5X5_1000") { arucoDict = cv::aruco::DICT_5X5_1000; }
            else if (setParam.arucoDictName == "DICT_6X6_50") { arucoDict = cv::aruco::DICT_6X6_50; }
            else if (setParam.arucoDictName == "DICT_6X6_100") { arucoDict = cv::aruco::DICT_6X6_100; }
            else if (setParam.arucoDictName == "DICT_6X6_250") { arucoDict = cv::aruco::DICT_6X6_250; }
            else if (setParam.arucoDictName == "DICT_6X6_1000") { arucoDict = cv::aruco::DICT_6X6_1000; }
            else if (setParam.arucoDictName == "DICT_7X7_50") { arucoDict = cv::aruco::DICT_7X7_50; }
            else if (setParam.arucoDictName == "DICT_7X7_100") { arucoDict = cv::aruco::DICT_7X7_100; }
            else if (setParam.arucoDictName == "DICT_7X7_250") { arucoDict = cv::aruco::DICT_7X7_250; }
            else if (setParam.arucoDictName == "DICT_7X7_1000") { arucoDict = cv::aruco::DICT_7X7_1000; }
            else if (setParam.arucoDictName == "DICT_ARUCO_ORIGINAL") { arucoDict = cv::aruco::DICT_ARUCO_ORIGINAL; }
            else if (setParam.arucoDictName == "DICT_APRILTAG_16h5") { arucoDict = cv::aruco::DICT_APRILTAG_16h5; }
            else if (setParam.arucoDictName == "DICT_APRILTAG_25h9") { arucoDict = cv::aruco::DICT_APRILTAG_25h9; }
            else if (setParam.arucoDictName == "DICT_APRILTAG_36h10") { arucoDict = cv::aruco::DICT_APRILTAG_36h10; }
            else if (setParam.arucoDictName == "DICT_APRILTAG_36h11") { arucoDict = cv::aruco::DICT_APRILTAG_36h11; }
            else {
                cout << "incorrect name of aruco dictionary \n";
                return 1;
            }

            dictionary = cv::aruco::getPredefinedDictionary(arucoDict);
        }
        else {
            cv::FileStorage dict_file(setParam.arucoDictFileName, cv::FileStorage::Mode::READ);
            cv::FileNode fn(dict_file.root());
            dictionary.readDictionary(fn);
        }
    }
    else {
        // default dictionary
        dictionary = cv::aruco::getPredefinedDictionary(0);
    }

    cv::aruco::CharucoBoard ch_board({ boardSize.width, boardSize.height },
        setParam.squareSize, setParam.markerSize, dictionary);

    cv::aruco::CharucoDetector ch_detector(ch_board);
    std::vector<int> markerIds;

    vector<vector<cv::Point2f> > imagePoints;
    cv::Size imageSize;
    clock_t prevTimestamp = 0;
    const cv::Scalar RED(0, 0, 255), GREEN(0, 255, 0);

    cv::Mat view;
    view = imread(inputPath);

    if (view.empty())
    {
        cout << "图像数据为空！" << endl;
        return false;
    }

    bitwise_not(view, view);

    imageSize = view.size();  // Format input image.

    int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

    bool found;
    vector<cv::Point2f> pointBuf;
    switch (setParam.calibrationPattern) // Find feature points on the input format
    {
    case Settings::CHESSBOARD:
        found = findChessboardCorners(view, boardSize, pointBuf, chessBoardFlags);
        break;
    case Settings::CHARUCOBOARD:
        ch_detector.detectBoard(view, pointBuf, markerIds);
        found = pointBuf.size() == (size_t)((boardSize.height - 1) * (boardSize.width - 1));
        break;
    case Settings::CIRCLES_GRID:
        found = findCirclesGrid(view, boardSize, pointBuf);
        break;
    case Settings::ASYMMETRIC_CIRCLES_GRID:
        found = findCirclesGrid(view, boardSize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);
        break;
    default:
        found = false;
        break;
    }

    // find image corners.
    if (found)                // If done with success,
    {
        // 获取相机角点。
        corners->corner_point_0[0] = pointBuf[0].x;
        corners->corner_point_0[1] = pointBuf[0].y;

        corners->corner_point_1[0] = pointBuf[1].x;
        corners->corner_point_1[1] = pointBuf[1].y;

        Mat dst;
        Point position;

        cv::String text = "Point 1";
        position.x = pointBuf[0].x;
        position.y = pointBuf[0].y;
        putText(view, text, Size(position.x - 10, position.y - 10), FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(0, 255, 0), 2);
        circle(view, position, 10, Scalar(0, 255, 0), -1, LINE_8); 

        Size newSize(800, 600);
        drawChessboardCorners(view, boardSize, cv::Mat(pointBuf), found);
        resize(view, dst, newSize, 0.0, 0.0, INTER_CUBIC);

        imshow("Corners", dst);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    else
    {
        return false;
    }

    return true;
}

__declspec(dllexport) bool __cdecl Run(const char* imagePath, const char* pointCloudPath, const char* robotPosePath,
                                       CameraInstrinsic* cameraIntrinsic, GeneralHandEyeResult* CalibResult)
{
    // 获取相机内参。
    cameraMatrix.at<float>(0, 0) = cameraIntrinsic->Fx;
    cameraMatrix.at<float>(0, 1) = 0;
    cameraMatrix.at<float>(0, 2) = cameraIntrinsic->Cx;

    cameraMatrix.at<float>(1, 0) = 0;
    cameraMatrix.at<float>(1, 1) = cameraIntrinsic->Fy;
    cameraMatrix.at<float>(1, 2) = cameraIntrinsic->Cy;

    cameraMatrix.at<float>(2, 0) = 0;
    cameraMatrix.at<float>(2, 1) = 0;
    cameraMatrix.at<float>(2, 2) = 1;

    // 获取畸变系数。
    distortion.at<float>(0, 0) = cameraIntrinsic->K1;
    distortion.at<float>(0, 1) = cameraIntrinsic->K2;
    distortion.at<float>(0, 2) = cameraIntrinsic->P1;
    distortion.at<float>(0, 3) = cameraIntrinsic->P2;
    distortion.at<float>(0, 4) = cameraIntrinsic->K3;

    cout << "标定前相机内参：" << cameraMatrix << endl;
    cout << "标定前畸变系数：" << distortion << endl;

    double RMS = 0;
	bool bGetCamMatrixSuccess = GetCameraMatrixChessboard(imagePath, cameraMatrix, distortion, myCameraRotation, myCameraTransform, RMS);
    if (!bGetCamMatrixSuccess)
    {
        // 获取相机矩阵失败，直接返回。
        return false;
    }
    CalibResult->RMS = RMS;

    bool GetRobotPoseSuccess = GetRobotPose(robotPosePath);
    if (!GetRobotPoseSuccess)
    {
        return false;
    }
    // 手眼标定。
    Mat R_cam2gripper(3, 3, CV_64FC1);
    Mat T_cam2gripper(3, 1, CV_64FC1);

    //cout << "机器人R矩阵大小：" << R_gripper2base.size() << endl;
    //cout << "机器人T矩阵大小：" << T_gripper2base.size() << endl;
    //cout << "相机R矩阵大小：" << myCameraRotation.size() << endl;
    //cout << "相机T矩阵大小：" << myCameraTransform.size() << endl;

    if (R_gripper2base.size() != myCameraRotation.size())
    {
        cout << "机器人位姿与相机姿态数量不相等！！！\n";
        return false;
    }

    calibrateHandEye(R_gripper2base, T_gripper2base, myCameraRotation, myCameraTransform, 
                     R_cam2gripper, T_cam2gripper, CALIB_HAND_EYE_TSAI);

    cout << "手眼R矩阵：" << R_cam2gripper << endl;
    cout << "手眼T矩阵：" << T_cam2gripper << endl;

    CalibResult->matrix[0] = R_cam2gripper.at<double>(0, 0);
    CalibResult->matrix[1] = R_cam2gripper.at<double>(1, 0);
    CalibResult->matrix[2] = R_cam2gripper.at<double>(2, 0);
    CalibResult->matrix[3] = T_cam2gripper.at<double>(0, 0);

    CalibResult->matrix[4] = R_cam2gripper.at<double>(0, 1);
    CalibResult->matrix[5] = R_cam2gripper.at<double>(1, 1);
    CalibResult->matrix[6] = R_cam2gripper.at<double>(2, 1);
    CalibResult->matrix[7] = T_cam2gripper.at<double>(1, 0);

    CalibResult->matrix[8] = R_cam2gripper.at<double>(0, 2);
    CalibResult->matrix[9] = R_cam2gripper.at<double>(1, 2);
    CalibResult->matrix[10] = R_cam2gripper.at<double>(2, 2);
    CalibResult->matrix[11] = T_cam2gripper.at<double>(2, 0);

    CalibResult->matrix[12] = 0;
    CalibResult->matrix[13] = 0;
    CalibResult->matrix[14] = 0;
    CalibResult->matrix[15] = 1;

    return true;
}

// 梅卡曼德相机手眼标定库。

__declspec(dllexport) bool __stdcall InitCamera()
{
    mmind::eye::HandEyeCalibration::CalibrationBoardModel boardModel = mmind::eye::HandEyeCalibration::CalibrationBoardModel::CGB_20;
    mmind::eye::HandEyeCalibration::CameraMountingMode mountingMode = mmind::eye::HandEyeCalibration::CameraMountingMode::EyeInHand;

    std::cout << "相机挂载方式：" << (int)mountingMode << endl;

    if (!findAndConnect(camera))
        return false;

    showError(calibration.initializeCalibration(camera, mountingMode, boardModel));
}

__declspec(dllexport) bool __stdcall GetPatternImage()
{
    mmind::eye::Color2DImage color2DImage;
    showError(calibration.testRecognition(camera, color2DImage));
    if (!color2DImage.isEmpty()) {
        std::string colorFile =
            "./Output/FeatureRecognitionResultForTest_" + std::to_string(poseIndex);
        colorFile += ".png";
        cv::Mat testImg = cv::Mat(color2DImage.height(), color2DImage.width(), CV_8UC3,
            color2DImage.data());

        // 图像增强。
        ImageEnhance(testImg);

        cv::namedWindow("Feature Recognition Result", 0);
        cv::resizeWindow("Feature Recognition Result", 800, 600); cv::imshow("Feature Recognition Result", testImg);
        std::cout << "Press any key to close the image." << std::endl;
        cv::waitKey(0);
        cv::destroyAllWindows();
        cv::imwrite(colorFile, testImg);
        std::cout << "Save the image to file " << colorFile << std::endl;
    }

    return true;
}

__declspec(dllexport) bool __stdcall AddRobotPose(double x, double y, double z, double rx, double ry, double rz)
{
    mmind::eye::HandEyeCalibration::Transformation robotPose = GetRobotPose(x, y, z, rx, ry, rz);

    mmind::eye::Color2DImage color2DImage;
    auto errStatus = calibration.addPoseAndDetect(camera, robotPose, color2DImage);
    showError(errStatus);
    if (!color2DImage.isEmpty()) {
        std::string colorFile = "./Output/FeatureRecognitionResult_" + std::to_string(poseIndex);
        colorFile += ".png";
        cv::Mat testImg = cv::Mat(color2DImage.height(), color2DImage.width(), CV_8UC3,
            color2DImage.data());
        //                cv::namedWindow("Feature Recognition Result", 0);
        //                cv::resizeWindow("Feature Recognition Result",
        //                color2DImage.width() / 2,
        //                                 color2DImage.height() / 2);
        //                cv::imshow("Feature Recognition Result", testImg);
        //                std::cout << "Press any key to close the image." << std::endl;
        //                cv::waitKey(0);
        //                cv::destroyAllWindows();
        cv::imwrite(colorFile, testImg);
        std::cout << "Save the image to file " << colorFile << std::endl;
        std::cout << "Successfully added the pose." << std::endl;
    }
    if (errStatus.isOK()) {
        poseIndex++;
    }

    return true;
}

__declspec(dllexport) bool __stdcall Calibrate(GeneralHandEyeResult* CalibResult)
{
    mmind::eye::ErrorStatus status = calibration.calculateExtrinsics(camera, cameraToBase);
    showError(status);
    if (status.isOK()) {

        CalibResult->HandEyeMatrix[0] = cameraToBase.x;
        CalibResult->HandEyeMatrix[1] = cameraToBase.y;
        CalibResult->HandEyeMatrix[2] = cameraToBase.z;
        CalibResult->HandEyeMatrix[3] = cameraToBase.qX;
        CalibResult->HandEyeMatrix[4] = cameraToBase.qY;
        CalibResult->HandEyeMatrix[5] = cameraToBase.qZ;
        CalibResult->HandEyeMatrix[6] = cameraToBase.qW;

        std::cout << "The extrinsic parameters are:" << std::endl;
        std::cout << cameraToBase.toString() << std::endl;
        saveExtrinsicParameters(cameraToBase.toString());
    }
    else
    {
        camera.disconnect();
        std::cout << "Disconnected from the camera successfully." << std::endl;

        return false;
    }
    camera.disconnect();
    std::cout << "Disconnected from the camera successfully." << std::endl;

    return true;
}

// Save extrinsic parameters to a TXT file named "ExtrinsicParameters (+time stamp)".
void saveExtrinsicParameters(const std::string& ExtrinsicParameters)
{
    char pStrPath1[1024];

    sprintf_s(pStrPath1, "ExtrinsicParameters.txt");

    //sprintf_s(pStrPath1, "ExtrinsicParameters%d%02d%02d%02d%02d%02d.txt", now);
    std::ofstream out(pStrPath1);
    out << "ExtrinsicParameters:" << std::endl;
    out << ExtrinsicParameters;
    out.close();
    std::cout << "Save result in file " << pStrPath1 << std::endl;
}

mmind::eye::HandEyeCalibration::Transformation GetRobotPose(double x, double y, double z, double rx, double ry, double rz)
{
    return EulerToQuad(5, x, y, z, rx, ry, rz);
}
// Convert Euler angles to quaternions. The unit of Euler angles is degree.
mmind::eye::HandEyeCalibration::Transformation EulerToQuad(int eulerType, double x, double y,
    double z, double R1, double R2, double R3)
{
    auto a1 = (R1 * PI / 180) / 2;
    auto a2 = (R2 * PI / 180) / 2;
    auto a3 = (R3 * PI / 180) / 2;
    double quadW = 0.0;
    double quadX = 0.0;
    double quadY = 0.0;
    double quadZ = 0.0;
    switch (eulerType) {
    case 1: // Z-Y'-X''
        quadW = sin(a1) * sin(a2) * sin(a3) + cos(a1) * cos(a2) * cos(a3);
        quadX = -sin(a1) * sin(a2) * cos(a3) + sin(a3) * cos(a1) * cos(a2);
        quadY = sin(a1) * sin(a3) * cos(a2) + sin(a2) * cos(a1) * cos(a3);
        quadZ = sin(a1) * cos(a2) * cos(a3) - sin(a2) * sin(a3) * cos(a1);
        break;

    case 2: // Z-Y'-Z''
        quadW = cos(a2) * cos(a1 + a3);
        quadX = -sin(a2) * sin(a1 - a3);
        quadY = sin(a2) * cos(a1 - a3);
        quadZ = cos(a2) * sin(a1 + a3);
        break;

    case 3: // X-Y'-Z''
        quadW = -sin(a1) * sin(a2) * sin(a3) + cos(a1) * cos(a2) * cos(a3);
        quadX = sin(a1) * cos(a2) * cos(a3) + sin(a2) * sin(a3) * cos(a1);
        quadY = -sin(a1) * sin(a3) * cos(a2) + sin(a2) * cos(a1) * cos(a3);
        quadZ = sin(a1) * sin(a2) * cos(a3) + sin(a3) * cos(a1) * cos(a2);
        break;

    case 4: // Z-X'-Z''
        quadW = cos(a2) * cos(a1 + a3);
        quadX = sin(a2) * cos(a1 - a3);
        quadY = sin(a2) * sin(a1 - a3);
        quadZ = cos(a2) * sin(a1 + a3);
        break;

    case 5: // X-Y-Z
        a1 = (R3 * PI / 180) / 2;
        a2 = (R2 * PI / 180) / 2;
        a3 = (R1 * PI / 180) / 2;
        quadW = sin(a1) * sin(a2) * sin(a3) + cos(a1) * cos(a2) * cos(a3);
        quadX = -sin(a1) * sin(a2) * cos(a3) + sin(a3) * cos(a1) * cos(a2);
        quadY = sin(a1) * sin(a3) * cos(a2) + sin(a2) * cos(a1) * cos(a3);
        quadZ = sin(a1) * cos(a2) * cos(a3) - sin(a2) * sin(a3) * cos(a1);
        break;

    default:

        break;
    }
    std::string split = ",";
    std::string quadResult = std::to_string(x) + split + std::to_string(y) + split +
        std::to_string(z) + split + std::to_string(quadW) + split +
        std::to_string(quadX) + split + std::to_string(quadY) + split +
        std::to_string(quadZ);
    std::string eulerResult = std::to_string(x) + split + std::to_string(y) + split +
        std::to_string(z) + split + std::to_string(R1) + split +
        std::to_string(R2) + split + std::to_string(R3);
    std::cout << "\nThe entered pose is: \n" << eulerResult << std::endl;
    std::cout << "The converted pose (Euler angles --> quaternions) is: \n"
        << quadResult << std::endl;
    return mmind::eye::HandEyeCalibration::Transformation(x, y, z, quadW, quadX, quadY, quadZ);
}

void GetTime(string timeStamp)
{
    time_t now = time(nullptr);  // 获取自1970年以来的秒数（UTC时间戳）
    tm* local = localtime(&now); // 转换为本地时间结构体

    // 自定义格式化输出
    char time[80];
    strftime(time, 80, "%Y-%m-%d %H:%M:%S", local);
    std::cout << "格式化时间: " << time << std::endl;

    timeStamp = time;
}

bool ReadStringList(const string& filename, vector<string>& imagePath)
{
    imagePath.clear();
    std::ifstream file(filename);  // 打开文件
    if (!file.is_open()) {             // 检查文件是否成功打开
        std::cerr << "无法打开文件！" << std::endl;
        return false;
    }

    int nLineNum = 0;
    std::string line;
    while (std::getline(file, line)) { // 逐行读取
        stringstream ss(line);

        imagePath.push_back(line);
        nLineNum++;
    }

    if (nLineNum < 8)
    {
        return false;
    }

    file.close();  // 关闭文件（ifstream 析构时会自动关闭）
    return true;
}

bool GetRobotPose(const char* robotPosePath)
{
    std::ifstream file(robotPosePath);  // 打开文件
    if (!file.is_open()) {             // 检查文件是否成功打开
        std::cerr << "无法打开文件！" << std::endl;
        return false;
    }

    std::string line;
    myRobotPose.clear();
    while (std::getline(file, line)) { // 逐行读取
        stringstream ss(line);
        string word;

        vector<string> words;

        while (ss >> word)
        {
            words.push_back(word);
        }
        
        // 输出解析结果。
        if (words.size() != 6)
        {
            //PLOG_INFO << "Robot pose length ERROR!" << endl;
            return false;
        }

        Mat Pose(1, 6, CV_64FC1);
        Pose.at<double>(0, 0) = std::stod(words[0]);
        Pose.at<double>(0, 1) = std::stod(words[1]);
        Pose.at<double>(0, 2) = std::stod(words[2]);
        Pose.at<double>(0, 3) = std::stod(words[3]);
        Pose.at<double>(0, 4) = std::stod(words[4]);
        Pose.at<double>(0, 5) = std::stod(words[5]);

        myRobotPose.push_back(Pose);
    }

    file.close();  // 关闭文件（ifstream 析构时会自动关闭）

    // 机器人姿态格式转换。
    R_gripper2base.clear();
    T_gripper2base.clear();
    for (int i = 0; i < myRobotPose.size(); i++)
    {
        Mat tempR;
        Mat tempT;
        attitudeVector2Matrix(myRobotPose[i], tempR, tempT, true);

        R_gripper2base.push_back(tempR);
        T_gripper2base.push_back(tempT);
    }

    return true;
}

bool GetCameraMatrixAruco(const char* imagePath, cv::Mat CameraMatrix, cv::Mat CameraDistortion, 
    vector<cv::Mat> &Rotation, vector<cv::Mat> &Transform, double& RMS)
{
    Settings setParam;

    //读取图像文件。
    bool bReadImageSuccess = ReadStringList(imagePath, setParam.imageList);
    if (!bReadImageSuccess)
    {
        cout  << "Load image data failed!";
        return false;
    }

    Size boardSize(setParam.boardWidth, setParam.boardHeight);

    float grid_width = setParam.squareSize * (boardSize.width - 1);
    if (setParam.calibrationPattern == Settings::Pattern::CHARUCOBOARD) {
        grid_width = setParam.squareSize * (boardSize.width - 2);
    }

    //create CharucoBoard
    cv::aruco::Dictionary dictionary;
    if (setParam.calibrationPattern == Settings::CHARUCOBOARD) {
        if (setParam.arucoDictFileName == "") {
            cv::aruco::PredefinedDictionaryType arucoDict;
            if (setParam.arucoDictName == "DICT_4X4_50") { arucoDict = cv::aruco::DICT_4X4_50; }
            else if (setParam.arucoDictName == "DICT_4X4_100") { arucoDict = cv::aruco::DICT_4X4_100; }
            else if (setParam.arucoDictName == "DICT_4X4_250") { arucoDict = cv::aruco::DICT_4X4_250; }
            else if (setParam.arucoDictName == "DICT_4X4_1000") { arucoDict = cv::aruco::DICT_4X4_1000; }
            else if (setParam.arucoDictName == "DICT_5X5_50") { arucoDict = cv::aruco::DICT_5X5_50; }
            else if (setParam.arucoDictName == "DICT_5X5_100") { arucoDict = cv::aruco::DICT_5X5_100; }
            else if (setParam.arucoDictName == "DICT_5X5_250") { arucoDict = cv::aruco::DICT_5X5_250; }
            else if (setParam.arucoDictName == "DICT_5X5_1000") { arucoDict = cv::aruco::DICT_5X5_1000; }
            else if (setParam.arucoDictName == "DICT_6X6_50") { arucoDict = cv::aruco::DICT_6X6_50; }
            else if (setParam.arucoDictName == "DICT_6X6_100") { arucoDict = cv::aruco::DICT_6X6_100; }
            else if (setParam.arucoDictName == "DICT_6X6_250") { arucoDict = cv::aruco::DICT_6X6_250; }
            else if (setParam.arucoDictName == "DICT_6X6_1000") { arucoDict = cv::aruco::DICT_6X6_1000; }
            else if (setParam.arucoDictName == "DICT_7X7_50") { arucoDict = cv::aruco::DICT_7X7_50; }
            else if (setParam.arucoDictName == "DICT_7X7_100") { arucoDict = cv::aruco::DICT_7X7_100; }
            else if (setParam.arucoDictName == "DICT_7X7_250") { arucoDict = cv::aruco::DICT_7X7_250; }
            else if (setParam.arucoDictName == "DICT_7X7_1000") { arucoDict = cv::aruco::DICT_7X7_1000; }
            else if (setParam.arucoDictName == "DICT_ARUCO_ORIGINAL") { arucoDict = cv::aruco::DICT_ARUCO_ORIGINAL; }
            else if (setParam.arucoDictName == "DICT_APRILTAG_16h5") { arucoDict = cv::aruco::DICT_APRILTAG_16h5; }
            else if (setParam.arucoDictName == "DICT_APRILTAG_25h9") { arucoDict = cv::aruco::DICT_APRILTAG_25h9; }
            else if (setParam.arucoDictName == "DICT_APRILTAG_36h10") { arucoDict = cv::aruco::DICT_APRILTAG_36h10; }
            else if (setParam.arucoDictName == "DICT_APRILTAG_36h11") { arucoDict = cv::aruco::DICT_APRILTAG_36h11; }
            else {
                cout << "incorrect name of aruco dictionary \n";
                return 1;
            }

            dictionary = cv::aruco::getPredefinedDictionary(arucoDict);
        }
        else {
            cv::FileStorage dict_file(setParam.arucoDictFileName, cv::FileStorage::Mode::READ);
            cv::FileNode fn(dict_file.root());
            dictionary.readDictionary(fn);
        }
    }
    else {
        // default dictionary
        dictionary = cv::aruco::getPredefinedDictionary(0);
    }

    cv::aruco::CharucoBoard ch_board({ boardSize.width, boardSize.height }, 
        setParam.squareSize, setParam.markerSize, dictionary);

    cv::aruco::CharucoDetector ch_detector(ch_board);
    std::vector<int> markerIds;

    vector<vector<cv::Point2f> > imagePoints;
    cv::Size imageSize;
    clock_t prevTimestamp = 0;
    const cv::Scalar RED(0, 0, 255), GREEN(0, 255, 0);

    int nImageNum = 0;
    vector<vector<cv::Point2f>> allImagePoints;
    vector<vector<cv::Point3f>> allObjectPoints;
    for (int i = 0; i < setParam.imageList.size(); i++)
    {
        cv::Mat view;
        view = imread(setParam.imageList[i]);

        if (view.empty())
        {
            cout << "图像数据为空！" << endl;
            return false;
        }

        imageSize = view.size();  // Format input image.
        int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

        bool found;
        vector<cv::Point2f> pointBuf;
        switch (setParam.calibrationPattern) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners(view, boardSize, pointBuf, chessBoardFlags);
            break;
        case Settings::CHARUCOBOARD:
            ch_detector.detectBoard(view, pointBuf, markerIds);
            found = pointBuf.size() == (size_t)((boardSize.height - 1) * (boardSize.width - 1));
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid(view, boardSize, pointBuf);
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(view, boardSize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);
            break;
        default:
            found = false;
            break;
        }

        // find image corners.
        if (found)                // If done with success,
        {
            // improve the found corners' coordinate accuracy for chessboard
            if (setParam.calibrationPattern == Settings::CHESSBOARD)
            {
                int winSize = 11;
                cv::Mat viewGray;
                cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
                cornerSubPix(viewGray, pointBuf, cv::Size(winSize, winSize),
                    cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
            }

            // Draw the corners.
            if (setParam.calibrationPattern == Settings::CHARUCOBOARD)
                drawChessboardCorners(view, cv::Size(boardSize.width - 1, boardSize.height - 1), cv::Mat(pointBuf), found);
            else
                drawChessboardCorners(view, boardSize, cv::Mat(pointBuf), found);
        }
        else
        {
            // 角点识别失败，直接返回。
            return false;
        }

        // 计算相机坐标系的点坐标。
        vector<cv::Point3f> ObjectPoints;
        vector<cv::Point2f> ImagePoints;
        ch_board.matchImagePoints(pointBuf, markerIds, ObjectPoints, ImagePoints);

        if (ObjectPoints.empty() || ImagePoints.empty())
        {
            cout << "Point matching failed,try again." << endl;
            continue;
        }

        int nSize = markerIds.size();
        putText(view, to_string(markerIds[0]), ImagePoints[0], FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
        putText(view, to_string(markerIds[6]), ImagePoints[6], FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);

        allImagePoints.push_back(ImagePoints);
        allObjectPoints.push_back(ObjectPoints);

        string strNum = to_string(nImageNum);
        cv::imwrite("./Output/Corners" + strNum + ".png", view);
        nImageNum++;
    }

    // Calibrate camera using ChArUco
    int flag = 0;
    flag |= CALIB_FIX_PRINCIPAL_POINT;
    flag |= CALIB_FIX_FOCAL_LENGTH;

    vector<Point3f> newObjPoints;
    RMS = calibrateCameraRO(allObjectPoints, allImagePoints, imageSize, -1, CameraMatrix, CameraDistortion,
        Rotation, Transform, newObjPoints, flag);

    cout << "相机内参：" << CameraMatrix << endl;
    cout << "相机畸变系数：" << CameraDistortion << endl;
    cout << "标定重投影误差：" << RMS << endl;


    return true;
}

bool GetCameraMatrixChessboard(const char* imagePath, cv::Mat CameraMatrix, cv::Mat CameraDistortion,
    vector<cv::Mat>& Rotation, vector<cv::Mat>& Transform, double& RMS)
{
    Settings setParam;

    //读取图像文件。
    bool bReadImageSuccess = ReadStringList(imagePath, setParam.imageList);
    if (!bReadImageSuccess)
    {
        cout << "Load image data failed!";
        return false;
    }

    Size boardSize(setParam.boardWidth, setParam.boardHeight);

    float grid_width = setParam.squareSize * (boardSize.width - 1);
    if (setParam.calibrationPattern == Settings::Pattern::CHARUCOBOARD) {
        grid_width = setParam.squareSize * (boardSize.width - 2);
    }

    vector<vector<cv::Point2f> > imagePoints;
    cv::Size imageSize;
    clock_t prevTimestamp = 0;
    const cv::Scalar RED(0, 0, 255), GREEN(0, 255, 0);

    int nImageNum = 0;
    vector<vector<cv::Point2f>> allImagePoints;
    vector<vector<cv::Point3f>> allObjectPoints;
    for (int i = 0; i < setParam.imageList.size(); i++)
    {
        cv::Mat view;
        view = imread(setParam.imageList[i]);

        if (view.empty())
        {
            cout << "图像数据为空！" << endl;
            return false;
        }

        imageSize = view.size();  // Format input image.

        int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

        bool found;
        vector<cv::Point2f> pointBuf;
        switch (setParam.calibrationPattern) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners(view, boardSize, pointBuf, chessBoardFlags);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid(view, boardSize, pointBuf);
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(view, boardSize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);
            break;
        default:
            found = false;
            break;
        }

        // find image corners.
        if (found)                // If done with success,
        {
            // improve the found corners' coordinate accuracy for chessboard
            if (setParam.calibrationPattern == Settings::CHESSBOARD)
            {
                int winSize = 11;
                cv::Mat viewGray;
                cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
                cornerSubPix(viewGray, pointBuf, cv::Size(winSize, winSize),
                    cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
            }

            // Draw the corners.
            if (setParam.calibrationPattern == Settings::CHARUCOBOARD)
                drawChessboardCorners(view, cv::Size(boardSize.width - 1, boardSize.height - 1), cv::Mat(pointBuf), found);
            else
                drawChessboardCorners(view, boardSize, cv::Mat(pointBuf), found);
        }
        else
        {
            // 角点识别失败，直接返回。
            return false;
        }

        allImagePoints.push_back(pointBuf);

        string strNum = to_string(nImageNum);
        cv::imwrite("./Output/Corners" + strNum + ".png", view);
        nImageNum++;
    }

    // 计算相机坐标系的点坐标。
    vector<vector<Point3f> > objectPoints(1);
    cv::Size sBoardSize(setParam.boardWidth, setParam.boardHeight);
    calcBoardCornerPositions(sBoardSize, setParam.squareSize, objectPoints[0], setParam.calibrationPattern);

    if (setParam.calibrationPattern == Settings::Pattern::CHARUCOBOARD) {
        //objectPoints[0][setParam.boardSize.width - 2].x = objectPoints[0][0].x + grid_width;
    }
    else {
        objectPoints[0][sBoardSize.width - 1].x = objectPoints[0][0].x + grid_width;
    }
    vector<Point3f> newObjPoints;
    newObjPoints = objectPoints[0];

    objectPoints.resize(allImagePoints.size(), objectPoints[0]);

    cout << "标定前的内参：" << endl;
    cout << "相机内参：" << CameraMatrix << endl;
    cout << "相机畸变系数：" << CameraDistortion << endl;

    // Calibrate camera using ChArUco
    int flag = 0;
    flag |= CALIB_USE_INTRINSIC_GUESS;
    //flag |= CALIB_FIX_ASPECT_RATIO;
    //flag |= CALIB_FIX_PRINCIPAL_POINT;

    Mat camIntrinsic;
    Mat camDistortion;
    RMS = calibrateCameraRO(objectPoints, allImagePoints, imageSize, -1, CameraMatrix, CameraDistortion,
        Rotation, Transform, newObjPoints, flag);

    cout << "标定后的内参：" << endl;
    cout << "相机内参：" << CameraMatrix << endl;
    cout << "相机畸变系数：" << CameraDistortion << endl;
    cout << "标定重投影误差：" << RMS << endl;

    return true;
}

void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
    Settings::Pattern patternType)
{
    corners.clear();

    switch (patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; ++i) {
            for (int j = 0; j < boardSize.width; ++j) {
                corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
            }
        }
        break;
    case Settings::CHARUCOBOARD:
        for (int i = 0; i < boardSize.height - 1; ++i) {
            for (int j = 0; j < boardSize.width - 1; ++j) {
                corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
            }
        }
        break;
    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; i++) {
            for (int j = 0; j < boardSize.width; j++) {
                corners.push_back(Point3f((2 * j + i % 2) * squareSize, i * squareSize, 0));
            }
        }
        break;
    default:
        break;
    }
}

void ImageEnhance(Mat image)
{
    // 图像增强。
    int nWidth = image.cols;
    int nHeight = image.rows;

    int ptNum = 0;
    int RedPointsX = 0;
    int RedPointsY = 0;
    for (int y = 0; y < nHeight; y++)
    {
        for (int x = 0; x < nWidth; x++)
        {
            int B = image.at<Vec3b>(y, x)[0];
            int G = image.at<Vec3b>(y, x)[1];
            int R = image.at<Vec3b>(y, x)[2];
            if (B == 0 && G == 0 && R == 255)
            {
                RedPointsX += x;
                RedPointsY += y;
                ptNum++;
            }
        }
    }

    // 定义圆心坐标（图像中心）、半径和颜色（红色）
    Point center;
    center.x = RedPointsX / ptNum;
    center.y = RedPointsY / ptNum;

    int radius = 8;
    Scalar color(0, 0, 255); // BGR 格式，红色

    // 绘制实心圆（厚度设为 -1 表示填充）
    circle(image, center, radius, color, -1);
}

bool attitudeVector2Matrix(Mat m, Mat &R, Mat &T, bool isEuler)
{
    R = Mat::eye(3, 3, CV_64FC1);
    T = Mat::eye(3, 1, CV_64FC1);

    if (m.empty())
    {
        cout << "矩阵为空！！！" << endl;
        return false;
    }

    Mat rotation(3, 1, CV_64FC1);

    rotation.at<double>(0, 0) = m.at<double>(0, 3);
    rotation.at<double>(1, 0) = m.at<double>(0, 4);
    rotation.at<double>(2, 0) = m.at<double>(0, 5);

    // 欧拉角
    if (isEuler)
    {
        R = EulerToRotationMatrix(rotation.at<double>(0, 0), rotation.at<double>(1, 0), rotation.at<double>(2, 0));
    }
    else
    {
        // 旋转向量。
        Rodrigues(rotation, R);
    }

    T.at<double>(0, 0) = m.at<double>(0, 0);
    T.at<double>(1, 0) = m.at<double>(0, 1);
    T.at<double>(2, 0) = m.at<double>(0, 2);

    return true;
}

Mat EulerToRotationMatrix(double pitch, double yaw, double roll)
{

    pitch /= 180 / PI;
    yaw /= 180 / PI;
    roll /= 180 / PI;

    // 计算各轴旋转矩阵
    Mat Rx = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos(pitch), -sin(pitch), 0, sin(pitch), cos(pitch));
    Mat Ry = (Mat_<double>(3, 3) << cos(yaw), 0, sin(yaw), 0, 1, 0, -sin(yaw), 0, cos(yaw));
    Mat Rz = (Mat_<double>(3, 3) << cos(roll), -sin(roll), 0, sin(roll), cos(roll), 0, 0, 0, 1);

    // 组合顺序（假设为 xyz）
    Mat R = Rz * Ry * Rx;

    return R;
}