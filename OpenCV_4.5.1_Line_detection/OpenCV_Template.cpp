#include <opencv2/opencv.hpp>  
#include <opencv2/imgproc.hpp>
#include <gsl/gsl_fit.h>
#include <iostream>  
#include <vector>

//#include "stdafx.h"
#include <stdio.h>
#include<winsock.h>					//소켓을 사용하기 위한 헤더파일

#pragma comment(lib, "ws2_32")		//위에서 선언한 헤더파일들을 가져다 쓰기위한 링크

#define PORT 4578					//예약된 포트를 제외하고 사용, 4자리 포트중 임의의 숫자를 할당
#define PACKET_SIZE 1024			//패킷사이즈를 정의

using namespace cv;
using std::vector;				//★using namespace std를 쓰면 프로그램이 그냥 종료됨★

//Hough Transform 파라미터
float rho = 2; // Hough 그리드의 거리 분해능(픽셀 단위)
float theta = 1 * CV_PI / 180; // Hough 그리드의 라디안 단위의 각도 분해능
float hough_threshold = 15;	 // 최소 투표 수(Hough 그리드 셀의 교차점)
float minLineLength = 10; //라인을 구성하는 최소 픽셀 
float maxLineGap = 20;	//연결 가능한 선 세그먼트 사이의 최대 픽셀 간격


//Region - of - interest vertices, 관심 영역 범위 계산시 사용 
//We want a trapezoid shape, with bottom edge at the bottom of the image
float trap_bottom_width = 1;  // 사다리꼴 하단 가장자리의 너비, 이미지 폭의 백분율로 표시됨
float trap_top_width = 0.6;     // 사다리꼴의 위쪽 가장자리에 대해 편집
float trap_height = 0.5;         // 이미지 높이의 백분율로 표시되는 사다리꼴 높이


//차선 색깔 범위 
Scalar lower_white = Scalar(200, 200, 200); //흰색 차선 (RGB)
Scalar upper_white = Scalar(255, 255, 255);
Scalar lower_yellow = Scalar(10, 100, 100); //노란색 차선 (HSV)
Scalar upper_yellow = Scalar(40, 255, 255);


// 통신 선언 변수들(일부) 전역변수로 test
WSADATA wsaData;				//Windows의 소켓 초기화 정보를 저장하기위한 구조체. 이미 선언되어있는 구조체이다.

SOCKET hListen;							//SOCKET 은 핸들이다
											//핸들이란 운영체제가 관리하는 커널오브젝트의 한 종류이다.
											//커널오브젝트는 운영체제가 관리하는 커널이라는 특수한 영역에 존재하는 오브젝트이다.
											//윈도우를 생성해도 해당 윈도우의 핸들이 생성되고 운영체제가 그 핸들을 이용해서어떤 프로그램인지를 구분한다던지 하는 기능을 제공한다.

										
SOCKADDR_IN tListenAddr = {};			//소켓의 구성요소를 담을 구조체 생성 및 값 할당 / 네트워크 표준은 빅엔디안을 활용


SOCKADDR_IN tCIntAddr = {};										//클라이언트 측 소켓 생성 및 정보를 담을 구조체 생성 및 값 할당, 클라이언트가 접속 요청하면 승인해주는 역할
int iCIntSize = sizeof(tCIntAddr);

//accept 함수를 이용하여 접속 요청을 수락해준다. 이 함수는 동기화된 방식으로 동작
//동기화된 방식이란 요청을 마무리 하기 전까지 계속 대기상태에 놓이게 되는 것
//즉 요청이 들어오기 전까지 이 함수는 안빠져나온다.
//접속 요청을 승인하면 연결된 소켓이 만들어져서 리턴된다.이렇게 만들어진 소켓을 이용해서 통신해야 한다.


																		//첫번째 인자로는 소켓을 넣어준다.
																		//두번째 인자로는 accept 할 클라이언트측 주소정보 구조체의 주소가 들어간다.
																		//세번째 인자로는 두번째 인자로 넣은 구조체의 크기를 저장해둔 변수의 주소


SOCKET hClient;

Mat region_of_interest(Mat img_edges, Point* points)
{
	/*
	Applies an image mask.
	Only keeps the region of the image defined by the polygon
	formed from `vertices`. The rest of the image is set to black.
	*/

	Mat img_mask = Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC1);					//a행 b열의 CV_... 타입 영행렬(역행렬!?)을 반환합니다.


	Scalar ignore_mask_color = Scalar(255, 255, 255);									//검은색?
	const Point* ppt[1] = { points };													//문법 이해 X... ppt는 꼭지점 역할? points가 4개의 점이므로 4개의 꼭지점!?
	int npt[] = { 4 };																	//꼭지점의 개수를 담는 변수


	//filling pixels inside the polygon defined by "vertices" with the fill color
	fillPoly(img_mask, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);						//1은 색으로 채워진 지역을 감싸는 가장자리의 개수  /  
																						// Scalar(255,255,255)는 색   /   LINE_8은 라인의 타입


	//rectangle(img_mask, Point(0, img_mask.rows / 2), Point(img_mask.cols, img_mask.rows), Scalar(255, 255, 255), -1, 8);


	//returning the image only where mask pixels are nonzero
	Mat img_masked;
	bitwise_and(img_edges, img_mask, img_masked);										//채워진 도형의 색과 and 비트연산하면 결국 엣지만 남게되는!?

	//영상 출력을 위한 추가

	//resize(img_mask, img_mask, Size(img_mask.cols * 0.5, img_mask.rows * 0.5));

	//imshow("마스킹 영상", img_mask);

	return img_masked;
}


//도로는 흰선과 노란선으로 이루어져 있고, 결국 그 선을 함께 인식해야하는데,
//영상처리에서는 흰선과 노란선을 한번에 인식할 수 없으므로(?) 따로 처리를 하고 합친다!?
//BGR 영상에서 흰색 차선 후보, HSV 영상에서 노란색 차선 후보를 검출합니다.
void filter_colors(Mat _img_bgr, Mat& img_filtered)								//노란색 차선 후보와 흰색 차선 후보를 합쳐서 차선 후보 영상을 만듭니다.
{
	// Filter the image to include only yellow and white pixels
	UMat img_bgr;													//원본영상(이미지들)을 카피하는 변수
	_img_bgr.copyTo(img_bgr);

	UMat img_hsv;													//노란 차선 인식을 위한 이미지 변수(cvtColor를 통해 변환)
	UMat img_combine;												//흰 차선과 노란차선을 합치기 위한 변수

	UMat white_mask;												//lower_white 와 upper_white 사이의 값 (= 흰색)에 해당한다면 흰색값을, 그렇지 않다면 검은색 값을 담는 변수
	UMat white_image;												//img_bgr로 받아오는 영상(이미지들)이 white_mask 영역 값으로 and 연산한 결과를 담는 변수

	UMat yellow_mask;												//white_mask와 동일한 기능
	UMat yellow_image;												//white_image와 동일한 기능


	//Filter white pixels
	inRange(img_bgr, lower_white, upper_white, white_mask);
	bitwise_and(img_bgr, img_bgr, white_image, white_mask);


	//Filter yellow pixels( Hue 30 )
	cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV);

	inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
	bitwise_and(img_bgr, img_bgr, yellow_image, yellow_mask);

	//Combine the two above images
	addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, img_combine);

	//영상 출력을 위한 추가
	//resize(img_combine, img_combine, Size(img_combine.cols * 0.5, img_combine.rows * 0.5));
	//imshow("filter_color 영상", img_combine);

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


 //draw_line 영상 송출 테스트
	Mat drawing_line;
	img_line.copyTo(drawing_line);



	// 오류가 발생할 경우 선을 그리지 마십시오.
	bool draw_right = true;
	bool draw_left = true;
	int width = img_line.cols;
	int height = img_line.rows;


	//모든 선의 경사 찾기
	//But abs(경사) > slope_threshold(경사)가 있는 선에만 주의하십시오.

	float slope_threshold = 0.5;
	vector<float> slopes;
	vector<Vec4i> new_lines;

	//cout << "lines_size() = "<<lines.size() << "\n";

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


	//cout << "right_lines_size() = " << right_lines.size() << "\n";

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


	//cout << "left_lines_size() = " << left_lines.size() << "\n";

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


	//계산된 끝점을 float에서 int로 변환
	y1 = int(y1);
	y2 = int(y2);
	right_x1 = int(right_x1);
	right_x2 = int(right_x2);
	left_x1 = int(left_x1);
	left_x2 = int(left_x2);


	int center_x1;

	int center_x2;

	center_x1 = (right_x1 + left_x1) / 2;
	center_x2 = (right_x2 + left_x2) / 2;



	int moving_point_x = (center_x1 + center_x2) / 2;
	int moving_point_y = (y1 + y2) / 2;

	/*	cout << "center_x1 = " << center_x1 << " center_x2 = " << center_x2 << "\n";
		cout << "y1 = " << y1 << " y2 = " << y2 <<"\n\n";

		cout << "moving_point (x,y) : " << moving_point_x << "  " << moving_point_y << "\n\n";*/
		// moving_point x좌표는 값이 계속 변함
		// y좌표는 그대로이지만 실제 주행때는 괜찮은게 맞는지!?
		// 영상은 계속 변하고(= 움직이고) ROI 영역은 일정
		// ROI자체의 y좌표는 0 ~ 특정 값까지 일정!
		// moivng_y 좌표의 값이 일정하게 ex) 810이면 그 값을 향해
		// 계속 주행!?
//이미지에 오른쪽 및 왼쪽 선 그리기
	if (draw_right)
		line(img_line, Point(right_x1, y1), Point(right_x2, y2), Scalar(0, 255, 0), 12);
	if (draw_left)
		line(img_line, Point(left_x1, y1), Point(left_x2, y2), Scalar(0, 255, 0), 12);


	// 송신 테스트
	char cMsg_go_straight[100] = "go_straight";

	char cMsg_turn_left[100] = "turn_left";

	char cMsg_turn_right[100] = "turn_right";

	if (draw_right == true && draw_left == true)
	{
		line(img_line, Point(center_x1, y1), Point(center_x2, y2), Scalar(255, 0, 0), 12);
		//printf("\n직진!!\n");


		send(hClient, cMsg_go_straight, strlen(cMsg_go_straight), 0);
	}

	else if (draw_right == true && draw_left == false)
	{
		//printf("\n좌회전!!\n");

		send(hClient, cMsg_turn_left, strlen(cMsg_turn_left), 0);
	}
	else if (draw_right == false && draw_left == true)
	{
		//printf("\n우회전!!\n");

		send(hClient, cMsg_turn_right, strlen(cMsg_turn_right), 0);
	}
	else
		printf("차선 인식 불가!\n");


	//test
	if (draw_right)
		line(drawing_line, Point(right_x1, y1), Point(right_x2, y2), Scalar(0, 255, 0), 12);
	if (draw_left)
		line(drawing_line, Point(left_x1, y1), Point(left_x2, y2), Scalar(0, 255, 0), 12);

	line(drawing_line, Point(center_x1, y1), Point(center_x2, y2), Scalar(0, 0, 255), 12);

	//test 영상 출력을 위한 추가
	resize(drawing_line, drawing_line, Size(drawing_line.cols * 0.4, drawing_line.rows * 0.4));
	imshow("draw_line 영상", drawing_line);
}


int main()
{
	
	WSAStartup(MAKEWORD(2, 2), &wsaData);	//이 함수를 호출해서 윈도우즈에 어느 소켓을 활용할 것인지 알려준다. 
											//첫번째 인자는 소켓 버전, 두번째 인자는 WSADATA 구조체의 포인터타입
											//그런데 2.2 버전은 실수이므로, 2.2라는 실수를 정수값으로 변환하여 넣어줄 수 있어야 한다.MAKEWORD 매크로를 이용해서 만들어준다.


	hListen = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);	//PF_INET 을 넣어주면 IPV4 타입을 사용한다는 것 / SOCK_STREAM 을 넣어주면 연결지향형(tcp) 소켓을 만들겠다는 의미 / IPPROTO_TCP 는 TCP를 사용하겠다고 지정해주는것


	
	tListenAddr.sin_family = AF_INET;					// sin_family 는 반드시 AF_INET 이어야 함
	tListenAddr.sin_port = htons(PORT);					//PORT 번호를 설정한다. 기본으로 정해진 포트를 제외한 포트번호를 설정해야 한다. 
	tListenAddr.sin_addr.s_addr = htonl(INADDR_ANY);	//INADDR_ANY를 넣어주면 현재 동작되는 컴퓨터의 IP 주소로 설정


	bind(hListen, (SOCKADDR*)&tListenAddr, sizeof(tListenAddr));	//bind 함수는 소켓에 주소정보를 연결
																	//첫번째 인자로는 위에 선언한 소켓을 넣어준다.
																	//두번째 인자로는 bind 될 소켓에 할당할 주소정보를 담고있는 구조체의 주소가 들어간다.
																	//세번째 인자로는 두번째 인자로 넣은 구조체의 크기가 들어간다.


	listen(hListen, SOMAXCONN);										//listen 함수는 연결을 수신하는상태로 소켓의 상태를 변경한다. 즉, 소켓을 접속 대기 상태로 만들어준다. 
																	//SOMAXCONN은 한꺼번에 요청 가능한 최대 접속승인 수를 의미


	hClient = accept(hListen, (SOCKADDR*)&tCIntAddr, &iCIntSize);


															// 클라이언트 측으로부터 정보를 받아오고 출력, 클라이언트에 정보 전송 
	char cBuffer[PACKET_SIZE] = {};
	recv(hClient, cBuffer, PACKET_SIZE, 0);
	printf("Recv Msg : %s\n", cBuffer);

	char cMsg[] = "Server Send";
	send(hClient, cMsg, strlen(cMsg), 0);

	//char cMsg_test[100];

	/*while (true)
	{
		printf("보낼 메세지 : ");
		scanf_s("%s", cMsg_test, 100);

		if (strcmp(cMsg_test, "end") == 0)
			break;

		else
			send(hClient, cMsg_test, strlen(cMsg_test), 0);
	}*/

	



	char buf[256];
	Mat img_bgr, img_gray, img_edges, img_hough, img_annotated;

	VideoCapture videoCapture("curb_line_test.mp4");		//비디오캡처 클래스 생성자(파일 이름)로 객체 생성

	if (!videoCapture.isOpened())							//클래스 객체가 비디오파일 또는 카메라를 위해 개방되었는지를 반환
	{
		printf("동영상 파일을 열수 없습니다. \n");

		/*char a;
		scanf_s("%c", a);*/

		return 1;
	}

	videoCapture.read(img_bgr);								//한 번의 호출로 프레임을 잡아서 읽어옴

	if (img_bgr.empty()) return -1;

	VideoWriter writer;										//비디오라이터 클래스 객체 생성

	int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');  // select desired codec (must be available at runtime)	//비디오 코덱을 XVID MPEG-4 코덱으로 설정
	double fps = 25.0;                          // framerate of the created video stream
	char filename[50] = "./curb_line_test_overwrapped.avi";             // name of the output video file
	writer.open(filename, codec, fps, img_bgr.size(), CV_8UC3);

	// check if we succeeded
	if (!writer.isOpened()) {
		printf("Could not open the output video file for write\n");
		return -1;
	}


	videoCapture.read(img_bgr);
	int width = img_bgr.size().width;
	int height = img_bgr.size().height;

	int count = 0;


	//■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■ 영상 처리 시작  ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■


	while (1)
	{
		//1. 원본 영상을 읽어옴 
		videoCapture.read(img_bgr);
		if (img_bgr.empty()) break;


		//2. 미리 정해둔 흰색, 노란색 범위 내에 있는 부분만 차선후보로 따로 저장함 
		Mat img_filtered;
		filter_colors(img_bgr, img_filtered);


		//3. 그레이스케일 영상으로 변환하여 에지 성분을 추출
		cvtColor(img_filtered, img_gray, COLOR_BGR2GRAY);
		GaussianBlur(img_gray, img_gray, Size(3, 3), 0, 0);
		Canny(img_gray, img_edges, 50, 150);

		int width = img_filtered.cols;					// row col 주의
		int height = img_filtered.rows;


		Point points[4];
		points[0] = Point((width * (1 - trap_bottom_width)) / 2, height);
		points[1] = Point((width * (1 - trap_top_width)) / 2, height - height * trap_height);
		points[2] = Point(width - (width * (1 - trap_top_width)) / 2, height - height * trap_height);
		points[3] = Point(width - (width * (1 - trap_bottom_width)) / 2, height);


		//4. 차선 검출할 영역을 제한함(진행방향 바닥에 존재하는 차선으로 한정)
		img_edges = region_of_interest(img_edges, points);


		UMat uImage_edges;
		img_edges.copyTo(uImage_edges);

		//5. 직선 성분을 추출(각 직선의 시작좌표와 끝좌표를 계산함)
		vector<Vec4i> lines;
		HoughLinesP(uImage_edges, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);




		//6. 5번에서 추출한 직선성분으로부터 좌우 차선에 있을 가능성있는 직선들만 따로 뽑아서
		//좌우 각각 하나씩 직선을 계산함 (Linear Least-Squares Fitting)
		Mat img_line = Mat::zeros(img_bgr.rows, img_bgr.cols, CV_8UC3);
		draw_line(img_line, lines);




		//7. 원본 영상에 6번의 직선을 같이 보여줌 
		addWeighted(img_bgr, 0.8, img_line, 1.0, 0.0, img_annotated);


		//8. 결과를 동영상 파일로 기록 
		writer << img_annotated;

		count++;
		if (count == 10) imwrite("curb_line_test_img.jpg", img_annotated);

		//9. 결과를 화면에 보여줌 
		Mat img_result;
		resize(img_annotated, img_annotated, Size(width * 0.4, height * 0.4));
		resize(img_edges, img_edges, Size(width * 0.4, height * 0.4));
		cvtColor(img_edges, img_edges, COLOR_GRAY2BGR);
		hconcat(img_edges, img_annotated, img_result);
		imshow("차선 영상", img_result);




		if (waitKey(1) == 27) break; //ESC키 누르면 종료  
	}


	printf("TCP 통신을 종료합니다(서버)\n");
	closesocket(hClient);					//해당 소켓을 닫아준다. 
	closesocket(hListen);

	WSACleanup();		//소켓을 활용하는것은 WSAStartup 함수와 WSACleanup 함수 사이에 작성해야 한다. 생성자와 소멸자 같은 개념이다.
						//WSACleanup 함수는 WSAStartup 을 하면서 지정한 내용을 지워준다.


	return 0;
}