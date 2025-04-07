#include "pch.h"
#include "HandEyeCalibration.h"


HandEyeCalibration::HandEyeCalibration()
{
    char timeStamp[80];
    GetTime(timeStamp);

    plog::init(plog::debug, "./Logs/HandEyeCalib.txt", 5000, 3);

    plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::get()->addAppender(&consoleAppender);

    //PLOG_INFO << "This is the first log..." << endl;
}

HandEyeCalibration::~HandEyeCalibration()
{
    // 关闭log.

}



void HandEyeCalibration::Run(const char* imagePath, const char* pointCloudPath, const char* robotPosePath, CameraInstrinsic* CameraMatrix)
{
    Mat cameraMatrix(3, 3, CV_32FC1);
    Mat distortion(1, 5, CV_32FC1);

    // 获取相机内参。
    cameraMatrix.at<float>(0, 0) = CameraMatrix->Fx;
    cameraMatrix.at<float>(0, 1) = 0;
    cameraMatrix.at<float>(0, 2) = CameraMatrix->Cx;

    cameraMatrix.at<float>(1, 0) = 0;
    cameraMatrix.at<float>(1, 1) = CameraMatrix->Fy;
    cameraMatrix.at<float>(1, 2) = CameraMatrix->Cy;

    cameraMatrix.at<float>(2, 0) = 0;
    cameraMatrix.at<float>(2, 1) = 0;
    cameraMatrix.at<float>(2, 2) = 1;

    // 获取畸变系数。
    distortion.at<float>(0, 0) = CameraMatrix->K1;
    distortion.at<float>(0, 1) = CameraMatrix->K2;
    distortion.at<float>(0, 2) = CameraMatrix->P1;
    distortion.at<float>(0, 3) = CameraMatrix->P2;
    distortion.at<float>(0, 4) = CameraMatrix->K3;

    cout << "标定前相机内参：" << cameraMatrix << endl;
    cout << "标定前畸变系数：" << distortion << endl;

	GetCameraMatrix(imagePath, cameraMatrix, distortion, myCameraRotation, myCameraTransform);

    GetRobotPose(robotPosePath);

    // 手眼标定。
    Mat R_cam2gripper(3, 3, CV_64FC1);
    Mat T_cam2gripper(3, 1, CV_64FC1);
    calibrateHandEye(R_gripper2base, T_gripper2base, myCameraRotation, myCameraTransform, 
                     R_cam2gripper, T_cam2gripper, CALIB_HAND_EYE_TSAI);

    cout << "手眼R矩阵：" << R_cam2gripper << endl;
    cout << "手眼T矩阵：" << T_cam2gripper << endl;
}

bool HandEyeCalibration::ReadStringList(const string& filename, vector<string>& imagePath)
{
    imagePath.clear();
    std::ifstream file(filename);  // 打开文件
    if (!file.is_open()) {             // 检查文件是否成功打开
        std::cerr << "无法打开文件！" << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) { // 逐行读取
        stringstream ss(line);

        imagePath.push_back(line);
    }

    file.close();  // 关闭文件（ifstream 析构时会自动关闭）
    return true;
}

bool HandEyeCalibration::GetRobotPose(const char* robotPosePath)
{
    std::ifstream file(robotPosePath);  // 打开文件
    if (!file.is_open()) {             // 检查文件是否成功打开
        std::cerr << "无法打开文件！" << std::endl;
        return false;
    }

    std::string line;
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
    for (int i = 0; i < myRobotPose.size(); i++)
    {
        Mat tempR;
        Mat tempT;
        attitudeVector2Matrix(myRobotPose[i], tempR, tempT, true);

        cout << "机器人R矩阵：" << tempR << endl;
        cout << "机器人T矩阵：" << tempT << endl;

        R_gripper2base.push_back(tempR);
        T_gripper2base.push_back(tempT);
    }

    return true;
}

bool HandEyeCalibration::GetCameraMatrix(const char* imagePath, cv::Mat CameraMatrix, cv::Mat CameraDistortion, 
    vector<cv::Mat> &Rotation, vector<cv::Mat> &Transform)
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
        if (setParam.flipVertical)    flip(view, view, 0);

        int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

        if (!setParam.useFisheye) {
            // fast check erroneously fails with high distortions like fisheye
            chessBoardFlags |= cv::CALIB_CB_FAST_CHECK;
        }

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

        // 计算相机坐标系的点坐标。
        vector<cv::Point3f> ObjectPoints;
        vector<cv::Point2f> ImagePoints;
        ch_board.matchImagePoints(pointBuf, markerIds, ObjectPoints, ImagePoints);

        if (ObjectPoints.empty() || ImagePoints.empty())
        {
            cout << "Point matching failed,try again." << endl;
            continue;
        }

        allImagePoints.push_back(ImagePoints);
        allObjectPoints.push_back(ObjectPoints);

        string strNum = to_string(nImageNum);
        cv::imwrite("./Output/Corners" + strNum + ".png", view);
        nImageNum++;
    }

    // Calibrate camera using ChArUco
    setParam.flag = CALIB_FIX_INTRINSIC;

    vector<Point3f> newObjPoints;
    double repError = calibrateCameraRO(allObjectPoints, allImagePoints, imageSize, -1, CameraMatrix, CameraDistortion,
        Rotation, Transform, newObjPoints, setParam.flag);

    cout << "相机内参：" << CameraMatrix << endl;
    cout << "相机畸变系数：" << CameraDistortion << endl;
    cout << "标定重投影误差：" << repError << endl;


    return true;
}
bool HandEyeCalibration::attitudeVector2Matrix(Mat m, Mat &R, Mat &T, bool isEuler)
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
Mat HandEyeCalibration::EulerToRotationMatrix(double pitch, double yaw, double roll)
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

IInterface* IInterface::CreateInterface()
{
	return new HandEyeCalibration();
}