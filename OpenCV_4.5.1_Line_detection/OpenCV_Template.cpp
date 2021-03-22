#include <opencv2/opencv.hpp>  
#include <opencv2/imgproc.hpp>
#include <gsl/gsl_fit.h>
#include <iostream>  
#include <vector>

//#include "stdafx.h"
#include <stdio.h>
#include<winsock.h>					//������ ����ϱ� ���� �������

#pragma comment(lib, "ws2_32")		//������ ������ ������ϵ��� ������ �������� ��ũ

#define PORT 4578					//����� ��Ʈ�� �����ϰ� ���, 4�ڸ� ��Ʈ�� ������ ���ڸ� �Ҵ�
#define PACKET_SIZE 1024			//��Ŷ����� ����

using namespace cv;
using std::vector;				//��using namespace std�� ���� ���α׷��� �׳� ����ʡ�

//Hough Transform �Ķ����
float rho = 2; // Hough �׸����� �Ÿ� ���ش�(�ȼ� ����)
float theta = 1 * CV_PI / 180; // Hough �׸����� ���� ������ ���� ���ش�
float hough_threshold = 15;	 // �ּ� ��ǥ ��(Hough �׸��� ���� ������)
float minLineLength = 10; //������ �����ϴ� �ּ� �ȼ� 
float maxLineGap = 20;	//���� ������ �� ���׸�Ʈ ������ �ִ� �ȼ� ����


//Region - of - interest vertices, ���� ���� ���� ���� ��� 
//We want a trapezoid shape, with bottom edge at the bottom of the image
float trap_bottom_width = 1;  // ��ٸ��� �ϴ� �����ڸ��� �ʺ�, �̹��� ���� ������� ǥ�õ�
float trap_top_width = 0.6;     // ��ٸ����� ���� �����ڸ��� ���� ����
float trap_height = 0.5;         // �̹��� ������ ������� ǥ�õǴ� ��ٸ��� ����


//���� ���� ���� 
Scalar lower_white = Scalar(200, 200, 200); //��� ���� (RGB)
Scalar upper_white = Scalar(255, 255, 255);
Scalar lower_yellow = Scalar(10, 100, 100); //����� ���� (HSV)
Scalar upper_yellow = Scalar(40, 255, 255);


// ��� ���� ������(�Ϻ�) ���������� test
WSADATA wsaData;				//Windows�� ���� �ʱ�ȭ ������ �����ϱ����� ����ü. �̹� ����Ǿ��ִ� ����ü�̴�.

SOCKET hListen;							//SOCKET �� �ڵ��̴�
											//�ڵ��̶� �ü���� �����ϴ� Ŀ�ο�����Ʈ�� �� �����̴�.
											//Ŀ�ο�����Ʈ�� �ü���� �����ϴ� Ŀ���̶�� Ư���� ������ �����ϴ� ������Ʈ�̴�.
											//�����츦 �����ص� �ش� �������� �ڵ��� �����ǰ� �ü���� �� �ڵ��� �̿��ؼ�� ���α׷������� �����Ѵٴ��� �ϴ� ����� �����Ѵ�.

										
SOCKADDR_IN tListenAddr = {};			//������ ������Ҹ� ���� ����ü ���� �� �� �Ҵ� / ��Ʈ��ũ ǥ���� �򿣵���� Ȱ��


SOCKADDR_IN tCIntAddr = {};										//Ŭ���̾�Ʈ �� ���� ���� �� ������ ���� ����ü ���� �� �� �Ҵ�, Ŭ���̾�Ʈ�� ���� ��û�ϸ� �������ִ� ����
int iCIntSize = sizeof(tCIntAddr);

//accept �Լ��� �̿��Ͽ� ���� ��û�� �������ش�. �� �Լ��� ����ȭ�� ������� ����
//����ȭ�� ����̶� ��û�� ������ �ϱ� ������ ��� �����¿� ���̰� �Ǵ� ��
//�� ��û�� ������ ������ �� �Լ��� �Ⱥ������´�.
//���� ��û�� �����ϸ� ����� ������ ��������� ���ϵȴ�.�̷��� ������� ������ �̿��ؼ� ����ؾ� �Ѵ�.


																		//ù��° ���ڷδ� ������ �־��ش�.
																		//�ι�° ���ڷδ� accept �� Ŭ���̾�Ʈ�� �ּ����� ����ü�� �ּҰ� ����.
																		//����° ���ڷδ� �ι�° ���ڷ� ���� ����ü�� ũ�⸦ �����ص� ������ �ּ�


SOCKET hClient;

Mat region_of_interest(Mat img_edges, Point* points)
{
	/*
	Applies an image mask.
	Only keeps the region of the image defined by the polygon
	formed from `vertices`. The rest of the image is set to black.
	*/

	Mat img_mask = Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC1);					//a�� b���� CV_... Ÿ�� �����(�����!?)�� ��ȯ�մϴ�.


	Scalar ignore_mask_color = Scalar(255, 255, 255);									//������?
	const Point* ppt[1] = { points };													//���� ���� X... ppt�� ������ ����? points�� 4���� ���̹Ƿ� 4���� ������!?
	int npt[] = { 4 };																	//�������� ������ ��� ����


	//filling pixels inside the polygon defined by "vertices" with the fill color
	fillPoly(img_mask, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);						//1�� ������ ä���� ������ ���δ� �����ڸ��� ����  /  
																						// Scalar(255,255,255)�� ��   /   LINE_8�� ������ Ÿ��


	//rectangle(img_mask, Point(0, img_mask.rows / 2), Point(img_mask.cols, img_mask.rows), Scalar(255, 255, 255), -1, 8);


	//returning the image only where mask pixels are nonzero
	Mat img_masked;
	bitwise_and(img_edges, img_mask, img_masked);										//ä���� ������ ���� and ��Ʈ�����ϸ� �ᱹ ������ ���ԵǴ�!?

	//���� ����� ���� �߰�

	//resize(img_mask, img_mask, Size(img_mask.cols * 0.5, img_mask.rows * 0.5));

	//imshow("����ŷ ����", img_mask);

	return img_masked;
}


//���δ� �򼱰� ��������� �̷���� �ְ�, �ᱹ �� ���� �Բ� �ν��ؾ��ϴµ�,
//����ó�������� �򼱰� ������� �ѹ��� �ν��� �� �����Ƿ�(?) ���� ó���� �ϰ� ��ģ��!?
//BGR ���󿡼� ��� ���� �ĺ�, HSV ���󿡼� ����� ���� �ĺ��� �����մϴ�.
void filter_colors(Mat _img_bgr, Mat& img_filtered)								//����� ���� �ĺ��� ��� ���� �ĺ��� ���ļ� ���� �ĺ� ������ ����ϴ�.
{
	// Filter the image to include only yellow and white pixels
	UMat img_bgr;													//��������(�̹�����)�� ī���ϴ� ����
	_img_bgr.copyTo(img_bgr);

	UMat img_hsv;													//��� ���� �ν��� ���� �̹��� ����(cvtColor�� ���� ��ȯ)
	UMat img_combine;												//�� ������ ��������� ��ġ�� ���� ����

	UMat white_mask;												//lower_white �� upper_white ������ �� (= ���)�� �ش��Ѵٸ� �������, �׷��� �ʴٸ� ������ ���� ��� ����
	UMat white_image;												//img_bgr�� �޾ƿ��� ����(�̹�����)�� white_mask ���� ������ and ������ ����� ��� ����

	UMat yellow_mask;												//white_mask�� ������ ���
	UMat yellow_image;												//white_image�� ������ ���


	//Filter white pixels
	inRange(img_bgr, lower_white, upper_white, white_mask);
	bitwise_and(img_bgr, img_bgr, white_image, white_mask);


	//Filter yellow pixels( Hue 30 )
	cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV);

	inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
	bitwise_and(img_bgr, img_bgr, yellow_image, yellow_mask);

	//Combine the two above images
	addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, img_combine);

	//���� ����� ���� �߰�
	//resize(img_combine, img_combine, Size(img_combine.cols * 0.5, img_combine.rows * 0.5));
	//imshow("filter_color ����", img_combine);

	img_combine.copyTo(img_filtered);
}

void draw_line(Mat& img_line, vector<Vec4i> lines)
{
	if (lines.size() == 0) return;

	/*
 ����: ������ �� ���׸�Ʈ�� ���/�ܻ��Ͽ� ������ ��ü ������ �����Ϸ��� ���
 (���� �� - ��.mp4���� P1_��.mp4�� ǥ�õ� ����� �̵�)
 �� ����� ���������� ����� �� �ֽ��ϴ�.

 �� ���׸�Ʈ�� ����(y2 - y1) / (x2 - x1)�� �����ϴ� �Ͱ� ���� ����� �����غ���,
 � ���׸�Ʈ�� ���� �� �� ������ ���� �Ϻ����� �����Ͻʽÿ�.
 �׷� ���� �� ���� ��ġ�� ���ȭ�ϰ� ���� ��ܰ� �ϴ����� ������ �� �ֽ��ϴ�.

 �� �Լ��� ���� �β��� ���� �׸���. �ش� �̹����� ���� �׷����ϴ�(�̹����� ����).
 ���� �������ϰ� �Ϸ��� �� �Լ��� �Ʒ� weighted_img() �Լ��� �����ϴ� ���� ����Ͻʽÿ�.
 */


 //draw_line ���� ���� �׽�Ʈ
	Mat drawing_line;
	img_line.copyTo(drawing_line);



	// ������ �߻��� ��� ���� �׸��� ���ʽÿ�.
	bool draw_right = true;
	bool draw_left = true;
	int width = img_line.cols;
	int height = img_line.rows;


	//��� ���� ��� ã��
	//But abs(���) > slope_threshold(���)�� �ִ� ������ �����Ͻʽÿ�.

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
		//��� ���
		if (x2 - x1 == 0) //�ڳ� ���̽�, 0���� ������ ����
			slope = 999.0; //��ǻ� ���� ���
		else
			slope = (y2 - y1) / (float)(x2 - x1);


		//��縦 �������� �� ���͸�
		if (abs(slope) > slope_threshold) {
			slopes.push_back(slope);
			new_lines.push_back(line);
		}
	}



	// �����ʰ� ���� ���� ������ ��Ÿ���� ������_���ΰ� ����_�������� ����
	// ������/���� ���� ������ ����/���� ���⸦ ������ �ϸ� �̹����� ������/���� ���ݿ� �־�� �մϴ�.
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


		float cx = width * 0.5; //x �̹��� �߾��� ��ǥ

		if (slope > 0 && x1 > cx && x2 > cx)
			right_lines.push_back(line);
		else if (slope < 0 && x1 < cx && x2 < cx)
			left_lines.push_back(line);
	}


	//���� ȸ�� �м��� �����Ͽ� ������ �� ���� ���� ���ο� ���� ������ ���� ã���ϴ�.

	//���� ����
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



	// ���� ����
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



	//���� �׸��� �� ���Ǵ� ������ �� ���� ���� ���� 2�� ã��
	//y = m*x + b--> x = (y - b) / m
	int y1 = height;
	int y2 = height * (1 - trap_height);

	float right_x1 = (y1 - right_b) / right_m;
	float right_x2 = (y2 - right_b) / right_m;

	float left_x1 = (y1 - left_b) / left_m;
	float left_x2 = (y2 - left_b) / left_m;


	//���� ������ float���� int�� ��ȯ
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
		// moving_point x��ǥ�� ���� ��� ����
		// y��ǥ�� �״�������� ���� ���ට�� �������� �´���!?
		// ������ ��� ���ϰ�(= �����̰�) ROI ������ ����
		// ROI��ü�� y��ǥ�� 0 ~ Ư�� ������ ����!
		// moivng_y ��ǥ�� ���� �����ϰ� ex) 810�̸� �� ���� ����
		// ��� ����!?
//�̹����� ������ �� ���� �� �׸���
	if (draw_right)
		line(img_line, Point(right_x1, y1), Point(right_x2, y2), Scalar(0, 255, 0), 12);
	if (draw_left)
		line(img_line, Point(left_x1, y1), Point(left_x2, y2), Scalar(0, 255, 0), 12);


	// �۽� �׽�Ʈ
	char cMsg_go_straight[100] = "go_straight";

	char cMsg_turn_left[100] = "turn_left";

	char cMsg_turn_right[100] = "turn_right";

	if (draw_right == true && draw_left == true)
	{
		line(img_line, Point(center_x1, y1), Point(center_x2, y2), Scalar(255, 0, 0), 12);
		//printf("\n����!!\n");


		send(hClient, cMsg_go_straight, strlen(cMsg_go_straight), 0);
	}

	else if (draw_right == true && draw_left == false)
	{
		//printf("\n��ȸ��!!\n");

		send(hClient, cMsg_turn_left, strlen(cMsg_turn_left), 0);
	}
	else if (draw_right == false && draw_left == true)
	{
		//printf("\n��ȸ��!!\n");

		send(hClient, cMsg_turn_right, strlen(cMsg_turn_right), 0);
	}
	else
		printf("���� �ν� �Ұ�!\n");


	//test
	if (draw_right)
		line(drawing_line, Point(right_x1, y1), Point(right_x2, y2), Scalar(0, 255, 0), 12);
	if (draw_left)
		line(drawing_line, Point(left_x1, y1), Point(left_x2, y2), Scalar(0, 255, 0), 12);

	line(drawing_line, Point(center_x1, y1), Point(center_x2, y2), Scalar(0, 0, 255), 12);

	//test ���� ����� ���� �߰�
	resize(drawing_line, drawing_line, Size(drawing_line.cols * 0.4, drawing_line.rows * 0.4));
	imshow("draw_line ����", drawing_line);
}


int main()
{
	
	WSAStartup(MAKEWORD(2, 2), &wsaData);	//�� �Լ��� ȣ���ؼ� ������� ��� ������ Ȱ���� ������ �˷��ش�. 
											//ù��° ���ڴ� ���� ����, �ι�° ���ڴ� WSADATA ����ü�� ������Ÿ��
											//�׷��� 2.2 ������ �Ǽ��̹Ƿ�, 2.2��� �Ǽ��� ���������� ��ȯ�Ͽ� �־��� �� �־�� �Ѵ�.MAKEWORD ��ũ�θ� �̿��ؼ� ������ش�.


	hListen = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);	//PF_INET �� �־��ָ� IPV4 Ÿ���� ����Ѵٴ� �� / SOCK_STREAM �� �־��ָ� ����������(tcp) ������ ����ڴٴ� �ǹ� / IPPROTO_TCP �� TCP�� ����ϰڴٰ� �������ִ°�


	
	tListenAddr.sin_family = AF_INET;					// sin_family �� �ݵ�� AF_INET �̾�� ��
	tListenAddr.sin_port = htons(PORT);					//PORT ��ȣ�� �����Ѵ�. �⺻���� ������ ��Ʈ�� ������ ��Ʈ��ȣ�� �����ؾ� �Ѵ�. 
	tListenAddr.sin_addr.s_addr = htonl(INADDR_ANY);	//INADDR_ANY�� �־��ָ� ���� ���۵Ǵ� ��ǻ���� IP �ּҷ� ����


	bind(hListen, (SOCKADDR*)&tListenAddr, sizeof(tListenAddr));	//bind �Լ��� ���Ͽ� �ּ������� ����
																	//ù��° ���ڷδ� ���� ������ ������ �־��ش�.
																	//�ι�° ���ڷδ� bind �� ���Ͽ� �Ҵ��� �ּ������� ����ִ� ����ü�� �ּҰ� ����.
																	//����° ���ڷδ� �ι�° ���ڷ� ���� ����ü�� ũ�Ⱑ ����.


	listen(hListen, SOMAXCONN);										//listen �Լ��� ������ �����ϴ»��·� ������ ���¸� �����Ѵ�. ��, ������ ���� ��� ���·� ������ش�. 
																	//SOMAXCONN�� �Ѳ����� ��û ������ �ִ� ���ӽ��� ���� �ǹ�


	hClient = accept(hListen, (SOCKADDR*)&tCIntAddr, &iCIntSize);


															// Ŭ���̾�Ʈ �����κ��� ������ �޾ƿ��� ���, Ŭ���̾�Ʈ�� ���� ���� 
	char cBuffer[PACKET_SIZE] = {};
	recv(hClient, cBuffer, PACKET_SIZE, 0);
	printf("Recv Msg : %s\n", cBuffer);

	char cMsg[] = "Server Send";
	send(hClient, cMsg, strlen(cMsg), 0);

	//char cMsg_test[100];

	/*while (true)
	{
		printf("���� �޼��� : ");
		scanf_s("%s", cMsg_test, 100);

		if (strcmp(cMsg_test, "end") == 0)
			break;

		else
			send(hClient, cMsg_test, strlen(cMsg_test), 0);
	}*/

	



	char buf[256];
	Mat img_bgr, img_gray, img_edges, img_hough, img_annotated;

	VideoCapture videoCapture("curb_line_test.mp4");		//����ĸó Ŭ���� ������(���� �̸�)�� ��ü ����

	if (!videoCapture.isOpened())							//Ŭ���� ��ü�� �������� �Ǵ� ī�޶� ���� ����Ǿ������� ��ȯ
	{
		printf("������ ������ ���� �����ϴ�. \n");

		/*char a;
		scanf_s("%c", a);*/

		return 1;
	}

	videoCapture.read(img_bgr);								//�� ���� ȣ��� �������� ��Ƽ� �о��

	if (img_bgr.empty()) return -1;

	VideoWriter writer;										//���������� Ŭ���� ��ü ����

	int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');  // select desired codec (must be available at runtime)	//���� �ڵ��� XVID MPEG-4 �ڵ����� ����
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


	//�������������������������������������� ���� ó�� ����  �����������������������������������������


	while (1)
	{
		//1. ���� ������ �о�� 
		videoCapture.read(img_bgr);
		if (img_bgr.empty()) break;


		//2. �̸� ���ص� ���, ����� ���� ���� �ִ� �κи� �����ĺ��� ���� ������ 
		Mat img_filtered;
		filter_colors(img_bgr, img_filtered);


		//3. �׷��̽����� �������� ��ȯ�Ͽ� ���� ������ ����
		cvtColor(img_filtered, img_gray, COLOR_BGR2GRAY);
		GaussianBlur(img_gray, img_gray, Size(3, 3), 0, 0);
		Canny(img_gray, img_edges, 50, 150);

		int width = img_filtered.cols;					// row col ����
		int height = img_filtered.rows;


		Point points[4];
		points[0] = Point((width * (1 - trap_bottom_width)) / 2, height);
		points[1] = Point((width * (1 - trap_top_width)) / 2, height - height * trap_height);
		points[2] = Point(width - (width * (1 - trap_top_width)) / 2, height - height * trap_height);
		points[3] = Point(width - (width * (1 - trap_bottom_width)) / 2, height);


		//4. ���� ������ ������ ������(������� �ٴڿ� �����ϴ� �������� ����)
		img_edges = region_of_interest(img_edges, points);


		UMat uImage_edges;
		img_edges.copyTo(uImage_edges);

		//5. ���� ������ ����(�� ������ ������ǥ�� ����ǥ�� �����)
		vector<Vec4i> lines;
		HoughLinesP(uImage_edges, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);




		//6. 5������ ������ �����������κ��� �¿� ������ ���� ���ɼ��ִ� �����鸸 ���� �̾Ƽ�
		//�¿� ���� �ϳ��� ������ ����� (Linear Least-Squares Fitting)
		Mat img_line = Mat::zeros(img_bgr.rows, img_bgr.cols, CV_8UC3);
		draw_line(img_line, lines);




		//7. ���� ���� 6���� ������ ���� ������ 
		addWeighted(img_bgr, 0.8, img_line, 1.0, 0.0, img_annotated);


		//8. ����� ������ ���Ϸ� ��� 
		writer << img_annotated;

		count++;
		if (count == 10) imwrite("curb_line_test_img.jpg", img_annotated);

		//9. ����� ȭ�鿡 ������ 
		Mat img_result;
		resize(img_annotated, img_annotated, Size(width * 0.4, height * 0.4));
		resize(img_edges, img_edges, Size(width * 0.4, height * 0.4));
		cvtColor(img_edges, img_edges, COLOR_GRAY2BGR);
		hconcat(img_edges, img_annotated, img_result);
		imshow("���� ����", img_result);




		if (waitKey(1) == 27) break; //ESCŰ ������ ����  
	}


	printf("TCP ����� �����մϴ�(����)\n");
	closesocket(hClient);					//�ش� ������ �ݾ��ش�. 
	closesocket(hListen);

	WSACleanup();		//������ Ȱ���ϴ°��� WSAStartup �Լ��� WSACleanup �Լ� ���̿� �ۼ��ؾ� �Ѵ�. �����ڿ� �Ҹ��� ���� �����̴�.
						//WSACleanup �Լ��� WSAStartup �� �ϸ鼭 ������ ������ �����ش�.


	return 0;
}