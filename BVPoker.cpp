# Dr.KM
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

#define EPSILON 1E-5

int IM_WIDTH = 1280;				//Werte auf eigene Spielkarten anpassen
int IM_HEIGHT = 720;				//Verhältnis Breite/Höhe muss stimmen --> keine Verzerrung

int CORNER_WIDTH = 27;
int CORNER_HEIGHT = 75;

int RANK_WIDTH = 70;               //fixe Größen für Breite und Höhe der Zahl 70x125 Pixel
int RANK_HEIGHT = 125;				//125/70 passt nicht für Dame etc.. RANK_HEIGHT auf 140 anpassen

int SUIT_WIDTH = 70;               //fixe Größe für Breite und Höhe des Symbols 70x100 Pixel
int SUIT_HEIGHT = 100;

int CARD_MAX_AREA = 120000;			//Werte anpassen 
int CARD_MIN_AREA = 25000;

Rect Corner;
Mat quad;


static double maximum(double number1, double number2, double number3) {
	return std::max(std::max(number1, number2), number3);
}


static bool almostEqual(double number1, double number2) {
	return (std::abs(number1 - number2) <= (EPSILON * maximum(1.0, std::abs(number1), std::abs(number2))));
}

static bool lineIntersection(const cv::Point2f &a1, const cv::Point2f &b1, const cv::Point2f &a2,  //Funktion zum Ausschneiden der Karten (Reckeck)
	const cv::Point2f &b2, cv::Point2f &intersection) {
	double A1 = b1.y - a1.y;
	double B1 = a1.x - b1.x;
	double C1 = (a1.x * A1) + (a1.y * B1);

	double A2 = b2.y - a2.y;
	double B2 = a2.x - b2.x;
	double C2 = (a2.x * A2) + (a2.y * B2);

	double det = (A1 * B2) - (A2 * B1);

	if (!almostEqual(det, 0)) {
		intersection.x = static_cast<float>(((C1 * B2) - (C2 * B1)) / (det));
		intersection.y = static_cast<float>(((C2 * A1) - (C1 * A2)) / (det));

		return true;
	}

	return false;
}

struct vector_sorter
{
	bool operator ()(const std::vector<cv::Point>& a, const std::vector<cv::Point> & b)
	{
		double dist_a = norm(a[0] - a[1]);
		double dist_b = norm(b[0] - b[1]);
		return dist_a > dist_b;
	}
};

void sortCorners(std::vector<cv::Point2f>& corners)         //Ecken ausrichten von Karten
{
	std::vector<cv::Point2f> top, bot;
	cv::Point2f center;
	// Get mass center
	for (int i = 0; i < corners.size(); i++)
		center += corners[i];
	center *= (1. / corners.size());

	for (int i = 0; i < corners.size(); i++)
	{
		if (corners[i].y < center.y)
			top.push_back(corners[i]);
		else
			bot.push_back(corners[i]);
	}
	corners.clear();

	if (top.size() == 2 && bot.size() == 2) {
		cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
		cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
		cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
		cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

		corners.push_back(tl);
		corners.push_back(tr);
		corners.push_back(br);
		corners.push_back(bl);
	}
}


//////////////////////////////////Rank matcher funktion////////////////////////////////////////////

int rankMatcher(Mat rnk)                    //Rang der Karte matchen
{
	Mat img;
	Mat dst;
	int qualities[13];
	int minindex = 0;

	string file[] = { "C:/Card_Imgs/Ranks/2.jpg","C:/Card_Imgs/Ranks/3.jpg" ,"C:/Card_Imgs/Ranks/4.jpg" , "C:/Card_Imgs/Ranks/5.jpg" ,
		"C:/Card_Imgs/Ranks/6.jpg" , "C:/Card_Imgs/Ranks/7.jpg" , "C:/Card_Imgs/Ranks/8.jpg" ,
		"C:/Card_Imgs/Ranks/9.jpg" , "C:/Card_Imgs/Ranks/10.jpg" , "C:/Card_Imgs/Ranks/jack.jpg" ,
		"C:/Card_Imgs/Ranks/queen.jpg" , "C:/Card_Imgs/Ranks/king.jpg" , "C:/Card_Imgs/Ranks/ace.jpg" , };

	for (int i = 0; i < 13; i++)			//Zahlen 2-10 und B,D,K,A 9+4=13
	{

		int match_quality = 0;
		img = imread(file[i]);								//einlesen der Bilder
		cvtColor(img, img, COLOR_BGR2GRAY);
		img = img > 120;

		resize(img, img, Size(RANK_WIDTH, RANK_HEIGHT));    //resizing auf bestimmte Größe der Bilder (Templates haben eigentlich schon die Größe)

		absdiff(img, rnk, dst);

		for (int y = 0; y < dst.rows; y++)
		{
			for (int x = 0; x < dst.cols; x++)
			{
				if (dst.at<uchar>(y, x) == 255)
					match_quality++;
			}
		}
		qualities[i] = match_quality;
		cout << "White Pixels in AbsDiff_Output: " << match_quality << "\n";
		imshow("Train Images", img);
		imshow("Diff", dst);
		//waitKey(1);
	}
	int min = qualities[0];

	for (int j = 0; j < 13; j++)
	{
		if (qualities[j]<min)
		{
			min = qualities[j];
			minindex = j;
		}
	}
	minindex += 1;
	cout << "Minimum White Pixel on Position: " << minindex;
	return minindex;
}

string putTextString(int mm)
{
	string Result;
	if (mm == 1)
	{
		Result = "Two";
	}
	else	if (mm == 2)
	{
		Result = "Three";
	}
	else	if (mm == 3)
	{
		Result = "Four";
	}
	else	if (mm == 4)
	{
		Result = "Five";
	}
	else	if (mm == 5)
	{
		Result = "Six";
	}
	else	if (mm == 6)
	{
		Result = "Seven";
	}
	else	if (mm == 7)
	{
		Result = "Eight";
	}
	else	if (mm == 8)
	{
		Result = "Nine";
	}
	else	if (mm == 9)
	{
		Result = "Ten";
	}
	else	if (mm == 10)
	{
		Result = "Jack";
	}
	else	if (mm == 11)
	{
		Result = "Queen";
	}
	else	if (mm == 12)
	{
		Result = "King";
	}
	else	if (mm == 13)
	{
		Result = "Ace";
	}
	return Result;
}


//////////////Suit der Karte matchen/////////////////////////////////////////////////

int suitMatcher(Mat rnk)                    //Suit der Karte matchen
{
	Mat img;
	Mat dst;
	int qualities[4];			//nur 4 unterschiedliche suits
	int minindex = 0;

	string file[] = { "C:/Card_Imgs/Suits/clubs.jpg","C:/Card_Imgs/Suits/diamonds.jpg" , "C:/Card_Imgs/Suits/hearts.jpg" ,"C:/Card_Imgs/Suits/spades.jpg" };

	for (int i = 0; i < 4; i++)			// clubs, diamonds, hearts, spades = 4
	{

		int match_quality = 0;
		img = imread(file[i]);								//einlesen der Bilder
		cvtColor(img, img, COLOR_BGR2GRAY);
		img = img > 120;

		resize(img, img, Size(SUIT_WIDTH, SUIT_HEIGHT));    //resizing auf bestimmte Größe der Bilder (Templates haben eigentlich schon die Größe)

		absdiff(img, rnk, dst);

		for (int y = 0; y < dst.rows; y++)
		{
			for (int x = 0; x < dst.cols; x++)
			{
				if (dst.at<uchar>(y, x) == 255)
					match_quality++;
			}
		}
		qualities[i] = match_quality;
		cout << "White Pixels in AbsDiff_Output: " << match_quality << "\n";
		imshow("Train Images", img);
		imshow("Diff", dst);
		//waitKey(1);
	}
	int min = qualities[0];

	for (int j = 0; j < 4; j++)
	{
		if (qualities[j]<min)
		{
			min = qualities[j];
			minindex = j;
		}
	}
	minindex += 1;
	cout << "Minimum White Pixel on Position: " << minindex;
	return minindex;
}

string putTextString2(int mm)
{
	string Result;
	if (mm == 1)
	{
		Result = "clubs";
	}
	else	if (mm == 2)
	{
		Result = "diamonds";
	}
	else	if (mm == 3)
	{
		Result = "hearts";
	}
	else	if (mm == 4)
	{
		Result = "spades";
	}

	return Result;
}

////////////////////////Ende der Funktion suit der Karte matchen////////////////////////////


void findROIs(Mat r, Mat s)
{
	vector<vector<Point> > r_cnts;
	vector<vector<Point> > s_cnts;

	Rect r_boundbox;
	Rect s_boundbox;

	Mat r_copy = r.clone();
	Mat s_copy = s.clone();

	findContours(r_copy, r_cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	sort(r_cnts.begin(), r_cnts.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
		return contourArea(c1, false) < contourArea(c2, false);
	});
	r_boundbox = boundingRect(r_cnts[r_cnts.size() - 1]);

	findContours(s_copy, s_cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	sort(s_cnts.begin(), s_cnts.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
		return contourArea(c1, false) < contourArea(c2, false);
	});
	s_boundbox = boundingRect(s_cnts[s_cnts.size() - 1]);

	Mat rr, ss;

	rr = r(r_boundbox);
	ss = s(s_boundbox);

	resize(rr, rr, Size(RANK_WIDTH, RANK_HEIGHT));
	resize(ss, ss, Size(SUIT_WIDTH, SUIT_HEIGHT));

	imshow("RANKROI", rr);
	imshow("SUITROI", ss);

	//imwrite("C:/Card_Imgs/Ranks" ".jpg", rr);			//speichert das Template für die Ranks ab
	//imwrite("C:/Card_Imgs/Suits" ".jpg", ss);			//speichert das Template für die Suits ab

	//rank und suit abspeichern als jpg oder mit Code von Maier


	int rnk_ID = rankMatcher(rr);

	putText(quad, putTextString(rnk_ID), Point(125, 150), 2, 0.8, Scalar(155, 155, 0), 2, 8);				//Text für rank ausgeben lassen
	imshow("Result Image", quad);


	int suit_ID = suitMatcher(ss);

	putText(quad, putTextString2(suit_ID), Point(30, 150), 2, 0.8, Scalar(155, 155, 0), 2, 8);				//Text für suit ausgeben lassen (Startpunkt anpassen)
	imshow("Result Image", quad);


}


void isolation(Mat islo)			//isolation of rank and ruits
{
	Rect rank, suit;
	rank.x = 10; rank.y = 10;
	rank.height = 150;			//passt nicht mehr für Bube,Dame,König etc. (130 ist zu niedrig)
	rank.width = 92;			//passt für 10 nicht mehr (92)
	suit.x = 10; suit.y = (rank.y + rank.height);
	suit.height = 130;
	suit.width = 92;
	Mat rnk, sut;

	rnk = islo(rank);
	sut = islo(suit);

	findROIs(rnk, sut);
}


void cardID(Mat Card)
{
	Corner.x = 5;
	Corner.y = 2;
	Corner.width = CORNER_WIDTH;
	Corner.height = CORNER_HEIGHT;
	rectangle(Card, Corner, Scalar(0, 0, 255), 1, 8);
	Mat ROI = Card(Corner);
	cvtColor(ROI, ROI, COLOR_BGR2GRAY);
	resize(ROI, ROI, Size((ROI.cols * 4), (ROI.rows * 4)));
	GaussianBlur(ROI, ROI, Size(5, 5), 0);
	threshold(ROI, ROI, 155, 255, THRESH_BINARY_INV);
	isolation(ROI);
}


int main(int argc, char** argv)
{

	bool showsteps = false; // set it to false to see only result; 
	Mat src, src_copy, edges;
	src = imread("C:/cards deck/6 spades.jpg");				//Bild wird eingelesen --> zu Video umändern! mit While-Schleife und am Ende break und waitKey(0) von anderem Programm übernehmen
	if (src.empty())
	{
		src = Mat(400, 400, CV_8UC3, Scalar(127, 127, 127));
		rectangle(src, Rect(20, 200, 150, 50), Scalar(0, 0, 255), 8);
		rectangle(src, Rect(200, 200, 50, 50), Scalar(0, 0, 255), 8);
	}
	resize(src, src, Size(IM_WIDTH, IM_HEIGHT));
	src_copy = src.clone();

	cvtColor(src, edges, COLOR_BGR2GRAY);
	GaussianBlur(edges, edges, Size(5, 5), 1.5, 1.5);

	erode(edges, edges, Mat());// these lines may need to be optimized 
	dilate(edges, edges, Mat());
	dilate(edges, edges, Mat());
	erode(edges, edges, Mat());

	Canny(edges, edges, 50, 150, 3); // canny parameters may need to be optimized 

									 ////////////Binarisierung mit Otsu-Methode, statt mit Canny-Filter///////////////

	cv::waitKey(1); // add this line

	vector<Point> selected_points;
	vector<vector<Point> > contours;


	findContours(edges, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);		//eges durch webcam ersetzen


	for (size_t i = 0; i < contours.size(); i++)
	{
		Rect minRect = boundingRect(contours[i]);

		if ((minRect.width > src.cols / 6) | (minRect.height > src.rows / 6)) // this line also need to be optimized 
		{
			selected_points.insert(selected_points.end(), contours[i].begin(), contours[i].end());

			if (showsteps)
			{
				drawContours(src_copy, contours, i, Scalar(0, 0, 255), 3);
			}
		}
	}

	if (showsteps) imshow("Selected contours", src_copy);
	waitKey(1);
	vector<Point2f> selected_points_f;
	vector<Point2f> corners;
	Mat(selected_points).convertTo(selected_points_f, CV_32F);
	Mat hull;
	convexHull(selected_points_f, hull, true, true);

	RotatedRect RRect = minAreaRect(hull);
	std::vector<cv::Point2f> RR_corners;
	Point2f four_points[4];
	RRect.points(four_points);
	RR_corners.push_back(four_points[0]);
	RR_corners.push_back(four_points[1]);
	RR_corners.push_back(four_points[2]);
	RR_corners.push_back(four_points[3]);

	for (int j = 0; j < 4; j++)
	{
		Point2f pt = RR_corners[j];
		Point2f nearest_pt = hull.at<Point2f>(j, 0);
		float dist = norm(pt - nearest_pt);
		for (int k = 1; k < hull.rows; k++)
		{
			Point2f hull_point = hull.at<Point2f>(k, 0);
			if (norm(pt - hull_point) < dist)
			{
				dist = norm(pt - hull_point);
				nearest_pt = hull_point;
			}
		}
		corners.push_back(nearest_pt);
	}
	sortCorners(corners);

	Mat(corners).convertTo(selected_points, CV_32S);

	Rect r = boundingRect(corners);
	quad = cv::Mat::zeros(norm(corners[1] - corners[2]), norm(corners[2] - corners[3]), CV_8UC3);

	std::vector<cv::Point2f> quad_pts;
	quad_pts.push_back(cv::Point2f(0, 0));
	quad_pts.push_back(cv::Point2f(quad.cols, 0));
	quad_pts.push_back(cv::Point2f(quad.cols, quad.rows));
	quad_pts.push_back(cv::Point2f(0, quad.rows));

	cv::Mat transmtx = cv::getPerspectiveTransform(corners, quad_pts);
	cv::warpPerspective(src, quad, transmtx, quad.size());

	if (showsteps)
	{
		src_copy = src.clone();
		polylines(src_copy, selected_points, true, Scalar(0, 0, 255), 3);
		imshow("selected quadrilateral part", src_copy);
	}

	resize(quad, quad, Size(200, 300));
	cardID(quad);
	if (showsteps)
	{
		imshow("edges", edges);
	}
	imshow("Result Image", quad);
	waitKey(0);

	if (showsteps)
	{
		imshow("edges", edges);
	}

	return 0;
}
