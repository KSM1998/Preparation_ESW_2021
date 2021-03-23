#include <opencv2/opencv.hpp>  
#include <opencv2/imgproc.hpp>
#include <gsl/gsl_fit.h>
#include <iostream>  
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <Windows.h>
#include<winsock.h> 

#pragma comment(lib, "ws2_32")      //위에서 선언한 헤더파일들을 가져다 쓰기위한 링크

#define PI 3.1415926535
#define SC 1.5               //영상 표시 스케일
#define PORT 4578               //예약된 포트를 제외하고 사용, 4자리 포트중 임의의 숫자를 할당
#define PACKET_SIZE 1024         //패킷사이즈를 정의


using namespace cv;
using  std::vector;
using  std::cout;
using  std::endl;
using  std::cin;
using  std::cerr;
using  std::string;



//Hough Transform 파라미터
float rho = 2; // Hough 그리드의 거리 분해능(픽셀 단위)
float theta = 1 * CV_PI / 180; // Hough 그리드의 라디안 단위의 각도 분해능
float hough_threshold = 15;    // 최소 투표 수(Hough 그리드 셀의 교차점)
float minLineLength = 30; // 라인을 구성하는 최소 픽셀 수
float maxLineGap = 3;   // 연결 가능한 선 세그먼트 사이의 최대 픽셀 간격


//Region - of - interest vertices, 관심 영역 범위 계산시 사용 
//이미지 하단에 하단 가장자리가 있는 사다리꼴 모양을 원합니다.
float trap_bottom_width = 0.3;  // 사다리꼴 하단 가장자리의 너비, 이미지 폭의 백분율로 표시됨
float trap_top_width = 0.3;     // 사다리꼴의 위쪽 가장자리에 대해 편집
float trap_height = 0.4;         // 이미지 높이의 백분율로 표시되는 사다리꼴 높이
float offset_h = 0;                // offset에 지정한 픽셀만큼 아래 빈공간을 둔다
float offset_w = 0.4;                // offset에 지정한 픽셀만큼 좌우로 벌어진다.
float offset_s = 0.1;                // offset에 지정한 픽셀만큼 기울기를 둔다


//차선 색깔 범위 
Scalar lower_white = Scalar(200, 200, 200); //흰색 차선 (RGB)
Scalar upper_white = Scalar(255, 255, 255);
Scalar lower_yellow = Scalar(10, 100, 100); //노란색 차선 (HSV)
Scalar upper_yellow = Scalar(40, 255, 255);

//통신변수
WSADATA wsaData;
SOCKET hListen;
SOCKADDR_IN tListenAddr = {};
SOCKADDR_IN tClntAddr = {};
int iClntSize;
SOCKET hClient;

//영상, 크기 임시 저장소
Mat img_combine_w, img_masked_w[2];
int width_w, height_w;

//통신
void tcp_server(float msg) {
    char temp[4];
    memcpy(temp, &msg, sizeof(float));
    send(hClient, temp, strlen(temp), 0);
    cout << "Server Msg : " << msg << endl;
}
//ROI
Mat region_of_interest(Mat img_edges, Point* points, int i)
{
    /*
    이미지 마스크를 적용합니다.

    이미지의 영역만 'vertice'에서 형성된 폴리곤으로 정의됩니다.
    이미지의 나머지 부분이 검은색으로 설정됩니다.
    */

    Mat img_mask = Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC1);


    Scalar ignore_mask_color = Scalar(255, 255, 255);
    const Point* ppt[1] = { points };
    int npt[] = { 4 };


    //채우기 색상으로 "filog"에 의해 정의된 폴리곤 내부의 픽셀 채우기
    fillPoly(img_mask, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);


    //마스크 픽셀이 0이 아닌 경우에만 이미지 반환
    Mat img_masked;
    bitwise_and(img_edges, img_mask, img_masked);

    img_mask.copyTo(img_masked_w[i]);
    return img_masked;
}




void filter_colors(Mat _img_bgr, Mat& img_filtered)
{
    // 노란색과 흰색 픽셀만 포함하도록 이미지 필터링
    UMat img_bgr;
    _img_bgr.copyTo(img_bgr);
    UMat img_hsv, img_combine;
    UMat white_mask, white_image;
    UMat yellow_mask, yellow_image;


    //흰색 픽셀 필터링
    inRange(img_bgr, lower_white, upper_white, white_mask);
    bitwise_and(img_bgr, img_bgr, white_image, white_mask);


    //노란색 픽셀 필터링(색조 30)
    cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV);


    inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
    bitwise_and(img_bgr, img_bgr, yellow_image, yellow_mask);


    //위의 두 이미지를 결합합니다.
    addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, img_combine);

    img_combine.copyTo(img_combine_w);
    img_combine.copyTo(img_filtered);
}



void draw_line(Mat& img_line, vector<Vec4i> lines)
{
    if (lines.size() == 0) return;

    /*
    참고: 감지한 선 세그먼트를 평균/외삽하여 레인의 전체 범위를 매핑하려는 경우
    (원시 선 - 예.mp4에서 P1_예.mp4에 표시된 결과로 이동)
    이 기능을 시작점으로 사용할 수 있습니다.

    선 세그먼트를 기울기(y2 - y1) / (x2 - x1)로 구분하는 것과 같은 방법을 생각해보고,
    어떤 세그먼트가 왼쪽 선 대 오른쪽 선의 일부인지 결정하십시오.
    그런 다음 각 선의 위치를 평균화하고 차선 상단과 하단으로 추정할 수 있습니다.

    이 함수는 색과 두께로 선을 그린다. 해당 이미지에 선이 그려집니다(이미지를 변형).
    선을 반투명하게 하려면 이 함수를 아래 weighted_img() 함수와 결합하는 것을 고려하십시오.
    */

    // 오류가 발생할 경우 선을 그리지 마십시오.
    bool draw_right = true;
    bool draw_left = true;
    int width = img_line.cols;
    int height = img_line.rows;


    //모든 선의 경사 찾기
    //그러나 abs(경사) > slope_threshold(경사)가 있는 선에만 주의하십시오.
    float slope_threshold = 0.5;
    vector<float> slopes;
    vector<Vec4i> new_lines;

    for (int i = 0; i < lines.size(); i++)
    {
        Vec4i line = lines[i];
        int x1 = line[0];
        int y1 = line[1];
        int x2 = line[2];
        int y2 = line[3];


        float slope;
        //경사 계산
        if (x2 - x1 == 0) //코너 케이스, 0으로 나누기 피함
            slope = 999.0; //사실상 무한 경사
        else
            slope = (y2 - y1) / (float)(x2 - x1);


        //경사를 기준으로 선 필터링
        if (abs(slope) > slope_threshold) {
            slopes.push_back(slope);
            new_lines.push_back(line);
        }
    }



    // 오른쪽과 왼쪽 차선 라인을 나타내는 오른쪽_라인과 왼쪽_라인으로 구분
    // 오른쪽/왼쪽 차선 라인은 양의/음의 기울기를 가져야 하며 이미지의 오른쪽/왼쪽 절반에 있어야 합니다.
    vector<Vec4i> right_lines;
    vector<Vec4i> left_lines;

    for (int i = 0; i < new_lines.size(); i++)
    {

        Vec4i line = new_lines[i];
        float slope = slopes[i];

        int x1 = line[0];
        int y1 = line[1];
        int x2 = line[2];
        int y2 = line[3];


        float cx = width * 0.5; //x 이미지 중앙의 좌표

        if (slope > 0 && x1 > cx && x2 > cx)
            right_lines.push_back(line);
        else if (slope < 0 && x1 < cx && x2 < cx)
            left_lines.push_back(line);
    }


    //선형 회귀 분석을 실행하여 오른쪽 및 왼쪽 차선 라인에 가장 적합한 선을 찾습니다.
    //우측 차선
    double right_lines_x[1000];
    double right_lines_y[1000];
    float right_m, right_b;


    int right_index = 0;
    for (int i = 0; i < right_lines.size(); i++) {

        Vec4i line = right_lines[i];
        int x1 = line[0];
        int y1 = line[1];
        int x2 = line[2];
        int y2 = line[3];

        right_lines_x[right_index] = x1;
        right_lines_y[right_index] = y1;
        right_index++;
        right_lines_x[right_index] = x2;
        right_lines_y[right_index] = y2;
        right_index++;
    }


    if (right_index > 0) {

        double c0, c1, cov00, cov01, cov11, sumsq;
        gsl_fit_linear(right_lines_x, 1, right_lines_y, 1, right_index,
            &c0, &c1, &cov00, &cov01, &cov11, &sumsq);

        //printf("# best fit: Y = %g + %g X\n", c0, c1);

        right_m = c1;
        right_b = c0;
    }
    else {
        right_m = right_b = 1;

        draw_right = false;
    }



    // 왼쪽 차선
    double left_lines_x[1000];
    double left_lines_y[1000];
    float left_m, left_b;

    int left_index = 0;
    for (int i = 0; i < left_lines.size(); i++) {

        Vec4i line = left_lines[i];
        int x1 = line[0];
        int y1 = line[1];
        int x2 = line[2];
        int y2 = line[3];

        left_lines_x[left_index] = x1;
        left_lines_y[left_index] = y1;
        left_index++;
        left_lines_x[left_index] = x2;
        left_lines_y[left_index] = y2;
        left_index++;
    }


    if (left_index > 0) {
        double c0, c1, cov00, cov01, cov11, sumsq;
        gsl_fit_linear(left_lines_x, 1, left_lines_y, 1, left_index,
            &c0, &c1, &cov00, &cov01, &cov11, &sumsq);

        //printf("# best fit: Y = %g + %g X\n", c0, c1);

        left_m = c1;
        left_b = c0;
    }
    else {
        left_m = left_b = 1;

        draw_left = false;
    }



    //선을 그리는 데 사용되는 오른쪽 및 왼쪽 선의 끝점 2개 찾기
    //y = m*x + b--> x = (y - b) / m
    int y1 = height;
    int y2 = height * (1 - trap_height);

    float right_x1 = (y1 - right_b) / right_m;
    float right_x2 = (y2 - right_b) / right_m;

    float left_x1 = (y1 - left_b) / left_m;
    float left_x2 = (y2 - left_b) / left_m;

    //각 차선의 기울기 계산
    float slope_r = atan2(y2 - y1, right_x2 - right_x1) * 180 / PI + 90;
    float slope_l = atan2(y2 - y1, left_x2 - left_x1) * 180 / PI + 90;
    float slope_s = atan2(height_w - y2, (right_x2 + left_x2) / 2 - width_w / 2) * 180 / PI;
    cout << "왼쪽차선 기울기 : " << slope_l << "\t오른쪽차선 기울기 : " << slope_r << "\n\t\t중앙선 기울기 : " << slope_s << endl;

    //계산된 끝점을 float에서 int로 변환
    y1 = int(y1);
    y2 = int(y2);
    right_x1 = int(right_x1);
    right_x2 = int(right_x2);
    left_x1 = int(left_x1);
    left_x2 = int(left_x2);


    //이미지에 오른쪽 및 왼쪽 선 그리기 및 통신
    if (draw_right)
        line(img_line, Point(right_x1, y1), Point(right_x2, y2), Scalar(255, 200, 0), 10);
    if (draw_left)
        line(img_line, Point(left_x1, y1), Point(left_x2, y2), Scalar(255, 200, 200), 10);
    if (draw_right && draw_left) {
        line(img_line, Point(width_w / 2, height_w), Point((right_x2 + left_x2) / 2, y2), Scalar(100, 100, 100), 10);
        tcp_server(slope_s);
    }
    else {
        tcp_server(-2);
    }
}



int main(int, char**)
{
    //통신 시작
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    hListen = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    tListenAddr.sin_family = AF_INET;
    tListenAddr.sin_port = htons(PORT);
    tListenAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(hListen, (SOCKADDR*)&tListenAddr, sizeof(tListenAddr));
    listen(hListen, SOMAXCONN);
    iClntSize = sizeof(tClntAddr);
    hClient = accept(hListen, (SOCKADDR*)&tClntAddr, &iClntSize);

    //영상처리 시작
    clock_t start, end;
    start = clock();
    char buf[256];
    Mat img_bgr, img_gray, img_edges, img_edge[2], img_hough, img_annotated;

    //VideoCapture videoCapture("http://192.168.0.34:8090/?action=stream");

    VideoCapture videoCapture("curb_line_test_encoding.mp4");

    if (!videoCapture.isOpened())
    {
        cout << "동영상 파일을 열수 없습니다. \n" << endl;

        char a;
        cin >> a;

        return 1;
    }



    videoCapture.read(img_bgr);
    if (img_bgr.empty()) return -1;

    VideoWriter writer;
    int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');  // 원하는 코덱 선택(런타임 시 사용할 수 있어야 함)
    double fps = 10.0;                          // 생성된 비디오 스트림 프레임률
    string filename = "curb_line_test_encoding_overwrapped.avi";             // 출력 비디오 파일의 이름
    writer.open(filename, codec, fps, img_bgr.size(), CV_8UC3);
    // 성공 여부 확인
    if (!writer.isOpened()) {
        cerr << "Could not open the output video file for write\n";
        return -1;
    }


    videoCapture.read(img_bgr);
    int width = img_bgr.size().width;
    int height = img_bgr.size().height;
    width_w = width;
    height_w = height;

    int count = 0;

    while (1)
    {

        //원본 영상을 읽어옴 
        videoCapture.read(img_bgr);
        if (img_bgr.empty()) break;


        //미리 정해둔 흰색, 노란색 범위 내에 있는 부분만 차선후보로 따로 저장함 
        Mat img_filtered;
        filter_colors(img_bgr, img_filtered);

        //3. 그레이스케일 영상으로 변환하여 에지 성분을 추출
        cvtColor(img_filtered, img_gray, COLOR_BGR2GRAY);
        GaussianBlur(img_gray, img_gray, Size(3, 3), 0, 0);
        Canny(img_gray, img_edges, 50, 150);



        int width = img_filtered.cols;
        int height = img_filtered.rows;


        //차선 검출할 영역을 제한함(진행방향 바닥에 존재하는 차선으로 한정)
        Point points[2][4], edges_points[2][4];
        for (int i = 0; i < 2; i++) {
            if (i == 0) {
                points[i][0] = Point(((width * (1 - trap_bottom_width)) / 2) - (width * offset_w), height - (height * offset_h));
                points[i][1] = Point(((width * (1 - trap_top_width)) / 2) - (width * offset_w) + (width * offset_s), height - height * trap_height);
                points[i][2] = Point((width - (width * (1 - trap_top_width)) / 2) - (width * offset_w) + (width * offset_s), height - height * trap_height);
                points[i][3] = Point((width - (width * (1 - trap_bottom_width)) / 2) - (width * offset_w), height - (height * offset_h));
                for (int j = 0; j < 4; j++) {
                    edges_points[i][j] = points[i][j];
                }
            }
            else {
                points[i][0] = Point(((width * (1 - trap_bottom_width)) / 2) + (width * offset_w), height - (height * offset_h));
                points[i][1] = Point(((width * (1 - trap_top_width)) / 2) + (width * offset_w) - (width * offset_s), height - height * trap_height);
                points[i][2] = Point((width - (width * (1 - trap_top_width)) / 2) + (width * offset_w) - (width * offset_s), height - height * trap_height);
                points[i][3] = Point((width - (width * (1 - trap_bottom_width)) / 2) + (width * offset_w), height - (height * offset_h));
                for (int j = 0; j < 4; j++) {
                    edges_points[i][j] = points[i][j];
                }
            }
            img_edge[i] = region_of_interest(img_edges, points[i], i);
        }
        addWeighted(img_edge[0], 1.0, img_edge[1], 1.0, 0.0, img_edges);

        //ROI 영역 스케치
        Mat edges_bgr;
        img_bgr.copyTo(edges_bgr);
        const Point* edges_temp[2] = { edges_points[0], edges_points[1] };
        int npt[2] = { 4, 4 };
        polylines(edges_bgr, edges_temp, npt, 2, 1, CV_RGB(0, 255, 0));


        UMat uImage_edges;
        img_edges.copyTo(uImage_edges);

        //직선 성분을 추출(각 직선의 시작좌표와 끝좌표를 계산함)
        vector<Vec4i> lines;
        HoughLinesP(uImage_edges, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);




        //5번에서 추출한 직선성분으로부터 좌우 차선에 있을 가능성있는 직선들만 따로 뽑아서
        //좌우 각각 하나씩 직선을 계산함 (Linear Least-Squares Fitting)
        Mat img_line = Mat::zeros(img_bgr.rows, img_bgr.cols, CV_8UC3);
        draw_line(img_line, lines);




        //원본 영상에 6번의 직선을 같이 보여줌 
        addWeighted(img_bgr, 0.8, img_line, 1.0, 0.0, img_annotated);


        //결과를 동영상 파일로 기록 
        writer << img_annotated;

        count++;
        if (count == 10) imwrite("curb_line_test_encoding_overwrapped.jpg", img_annotated);

        //결과를 화면에 보여줌 
        Mat img_result;
        Mat img_mask;
        addWeighted(img_masked_w[0], 1.0, img_masked_w[1], 1.0, 0.0, img_mask);
        resize(img_combine_w, img_combine_w, Size(width * SC, height * SC));
        resize(edges_bgr, edges_bgr, Size(width * SC, height * SC));
        resize(img_mask, img_mask, Size(width * SC, height * SC));
        hconcat(img_combine_w, edges_bgr, img_result);
        imshow("색상필터 / 마스킹 영상", img_result);
        //imshow("마스크 영상", img_mask);
        resize(img_annotated, img_annotated, Size(width * SC, height * SC));
        resize(img_edges, img_edges, Size(width * SC, height * SC));
        cvtColor(img_edges, img_edges, COLOR_GRAY2BGR);
        hconcat(img_edges, img_annotated, img_result);
        imshow("차선 영상", img_result);

        //클라이언트 통신 수신
        /*char cBuffer[PACKET_SIZE] = {};
        recv(hClient, cBuffer, PACKET_SIZE, 0);
        printf("Recv Msg : %s\n\n", cBuffer);*/

        if (waitKey(1) == 27) break; //ESC키 누르면 종료  
    }

    end = clock();
    printf("타이머 : %.2f초\n\n", ((float)(end - start) / CLOCKS_PER_SEC));

    tcp_server(-1);
    closesocket(hClient);
    closesocket(hListen);

    return 0;
}