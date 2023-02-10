#include "AI/yolo-onnxruntime/yolo.hpp"
#include "Aim/Camera/Camera.hpp"
#include "Aim/stereo-depth-estimation/src/stereo_reconstruct.h"
#include "Aim/Solver/solver.hpp"
#include "Aim/Serial/serial.h"
//#include "Auto/aiming/Aiming_CTRV.hpp"
#include "include/CLI11.hpp"
#include <algorithm>
//opencv include path: /usr/local/include/opencv4

const Net_config DETECTOR_PRESETS={0.4, 0.3, "../../yoloRM.onnx", "../runtime/labels.txt"};
const Mat testImg = imread("../runtime/test.png");
VideoCapture testVideo("../runtime/test.mp4");
const int KALMAN_DIM = 4;
const Mat TRANSITION_MATRIX = (Mat_<float>(KALMAN_DIM, KALMAN_DIM) <<
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1);
/*
    1, 0, 1, 0, 0, 0,
    0, 1, 0, 1, 0, 0,
    0, 0, 1, 0, 1, 0,
    0, 0, 0, 1, 0, 1,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1);
*/
YOLO ArmorDetector(DETECTOR_PRESETS);
MVCamera ForwardCamera;
Solver TargetSolver;
KalmanFilter KF(KALMAN_DIM, 2, 0);
int targetEmergedCount = 0;
const int EMERGEGATE = 3000;
Serial SerialSender;

int main(int argc, char **argv){
    CLI::App args{"RM Vision"};

    bool SERIAL = false;
    bool CAMERA = false;
    bool SHOWINFO = false;
    bool SHOWFRAME = false;
    bool ISRED = false;

    args.add_flag("-s", SERIAL, "Enable serial communication");
    args.add_flag("-c", CAMERA, "Use camera as input");
    args.add_flag("-i", SHOWINFO, "Show run info");
    args.add_flag("-f", SHOWFRAME, "Show frame");
    args.add_flag("-r", ISRED, "You are in red team");
    
    CLI11_PARSE(args, argc, argv);

    //Camera Preparations
    if(CAMERA) cout << (ForwardCamera.OpenCamera() ? "OpenCamera() == true\n" : "OpenCamera() == false\n");

    //Serial Preparations
    if(SERIAL) cout << (SerialSender.OpenSerial() ? "OpenSerial() == true\n" : "OpenSerial() == false\n");

    int cnt = 0;
    Mat inputImg;

    TRANSITION_MATRIX.copyTo(KF.transitionMatrix);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(1));
    Mat measurement = Mat::zeros(2, 1, CV_32F);

    while(waitKey(1) != 'q') {
        //Inpupt
        if(CAMERA) inputImg = ForwardCamera.ReadCamera()[0];
        else testVideo >> inputImg;
        //else testImg.copyTo(inputImg);
        //if(SHOWFRAME) imshow("Camera Input", inputImg);

        //Detect
        vector<Detection> result = ArmorDetector.detect(inputImg);

        if(SHOWFRAME)
        {
        	ArmorDetector.DrawOnImage(inputImg,result);
        	imshow("Detection Result", inputImg);
        }
        if(SHOWINFO) cout << "[info] frame " << cnt++ << ": " << getTickCount() / getTickFrequency() << endl;

        //Filter
        vector<Detection> red_armor, blue_armor, car;
        for(Detection Det : result) {
            if(Det.className == "armor_red") {
                red_armor.push_back(Det);
                //if(SHOWINFO) cout << "[info] Caught red armor: " << Point2f(Det.x1, Det.y1) << endl;
            }
            if(Det.className == "armor_blue" || Det.className == "armor_grey") {
                blue_armor.push_back(Det);
                //if(SHOWINFO) cout << "[info] Caught blue armor: " << Point2f(Det.x1, Det.y1) << endl;
            }
            if(Det.className == "car") {
                car.push_back(Det);
                //if(SHOWINFO) cout << "[info] Caught car: " << Point2f(Det.x1, Det.y1) << endl;
            }
        }

        //Solve
        vector<Point2f> red_solved, blue_solved, car_solved;
        for(Detection Det : red_armor) {
            red_solved.push_back(TargetSolver.solve(Point2f(Det.x1, Det.y1), Point2f(Det.x2, Det.y2), inputImg.size()));
            //if(SHOWINFO) cout << "[info] Solved red armor at: " << red_solved.back() << endl;
        }
        for(Detection Det : blue_armor) {
            blue_solved.push_back(TargetSolver.solve(Point2f(Det.x1, Det.y1), Point2f(Det.x2, Det.y2), inputImg.size()));
            //if(SHOWINFO) cout << "[info] Solved blue armor at: " << blue_solved.back() << endl;
        }
        for(Detection Det : car) {
            car_solved.push_back(TargetSolver.solve(Point2f(Det.x1, Det.y1), Point2f(Det.x2, Det.y2), inputImg.size()));
            //if(SHOWINFO) cout << "[info] Solved car at: " << car_solved.back() << endl;
        }

        //Target
        vector<Point2f> target = ISRED ? blue_solved : red_solved;//grey?

        Point2f targetChosen(0.0, 0.0);
        Point2f targetSend = targetChosen;

        if(!target.empty()) {
            targetChosen = target[0];
            //if(SHOWINFO) cout << "[info] Target chosen: " << targetChosen << endl;
            targetEmergedCount++;
            //if(SHOWINFO) cout << "[info] Target emerged count: " << targetEmergedCount << endl;

            Mat prediction;
            KF.predict().copyTo(prediction);
            //for(int i = 1; i <= 10; i++) prediction = TRANSITION_MATRIX * prediction;
            cout << prediction << endl;
            Point2f targetPredict = Point2f(prediction.at<float>(0), prediction.at<float>(1));

            measurement.at<float>(0) = targetChosen.x;
            measurement.at<float>(1) = targetChosen.y;

            KF.correct(measurement);

            if(targetEmergedCount > EMERGEGATE) targetSend = targetPredict;
            else targetSend = targetChosen;
        }
        else {
            KF.init(KALMAN_DIM, 2);
	        setIdentity(KF.measurementMatrix);
            setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
            setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
            setIdentity(KF.errorCovPost, Scalar::all(1));
            targetEmergedCount = 0;
        }

        //Serial
        if(SHOWINFO) cout << "[info] Serial ready to send: " << targetSend << endl;
        if(SERIAL) {
            SerialSender.send_control(
                targetSend.y,
                targetSend.x,
                0.0,
                0.0,
                0.0,
                0,
                !target.empty()
            );
        }

        //Render
        if(SHOWFRAME) {
            Mat Raidar = Mat::zeros(500, 500, CV_8UC3);
            circle(Raidar, Point(250, 250), 250, Scalar(0, 255, 0), 1);
            circle(Raidar, Point(250, 250), 200, Scalar(0, 255, 0), 1);
            circle(Raidar, Point(250, 250), 150, Scalar(0, 255, 0), 1);
            circle(Raidar, Point(250, 250), 100, Scalar(0, 255, 0), 1);
            circle(Raidar, Point(250, 250), 50, Scalar(0, 255, 0), 1);

            for(Point2f angle : red_solved) circle(Raidar, Point(250 + 200 * cos(angle.x), 250 - 200 * sin(angle.x)), 5, Scalar(0, 0, 255), -1);//anti-clockwise
            for(Point2f angle : blue_solved) circle(Raidar, Point(250 + 200 * cos(angle.x), 250 - 200 * sin(angle.x)), 5, Scalar(255, 0, 0), -1);
            for(Point2f angle : car_solved) circle(Raidar, Point(250 + 200 * cos(angle.x), 250 - 200 * sin(angle.x)), 15, Scalar(255, 0, 255), 1);

            line(Raidar, Point(250, 250), Point(500, 250), Scalar(255, 255, 255), 1);
            line(Raidar, Point(250, 250), Point(250 + 250 * cos(atan(TargetSolver.YAW_HALF_PERSPECT)), 250 - 250 * sin(atan(TargetSolver.YAW_HALF_PERSPECT))), Scalar(255, 255, 255), 1);
            line(Raidar, Point(250, 250), Point(250 + 250 * cos(atan(TargetSolver.YAW_HALF_PERSPECT)), 250 + 250 * sin(atan(TargetSolver.YAW_HALF_PERSPECT))), Scalar(255, 255, 255), 1);

            warpAffine(Raidar, Raidar, getRotationMatrix2D(Point2f(250, 250), 90, 1), Size(500, 500));
            imshow("Raidar Result", Raidar);

            Mat Track = Mat::zeros(500, 500, CV_8UC3);

            circle(Track, Point(250 + 1000 * targetChosen.x, 250 + 1000 * targetChosen.y), 5, Scalar(0, 0, 255), -1);
            circle(Track, Point(250 + 1000 * targetSend.x, 250 + 1000 * targetSend.y), 8, Scalar(255, 255, 0), 2);

            imshow("Track Result", Track);
        }

        //waitKey(0);
    }
    return 0;
}
