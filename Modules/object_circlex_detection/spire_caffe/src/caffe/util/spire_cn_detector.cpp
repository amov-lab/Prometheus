#ifdef USE_OPENCV
#include <caffe/util/spire_cn_detector.h>
#include <numeric>

using namespace std;
using namespace cv;

namespace spire {

int save_ellipse_cont_ = SAVE_ELLIPSE_START;

CNEllipseDetector::CNEllipseDetector(void) : times_(8, 0.0), timesSign_(8, 0)
{
	// Default Parameters Settings
	szPreProcessingGaussKernel_ = Size(5, 5);
	dPreProcessingGaussSigma_ = 1.0;
	fThrArcPosition_ = 1.0f;
	fMaxCenterDistance_ = 100.0f * 0.05f;
	fMaxCenterDistance2_ = fMaxCenterDistance_ * fMaxCenterDistance_;
	iMinEdgeLength_ = 16;
	fMinOrientedRectSide_ = 3.0f;
	fDistanceToEllipseContour_ = 0.1f;
	fMinScore_ = 0.7f;
	fMinReliability_ = 0.5f;
	uNs_ = 16;

#ifdef DETECT_MARKER_X_PLUS
	FileStorage _fs("x_moments.xml", FileStorage::READ);
	_fs["XE"] >> XMap_;
#endif
	srand(unsigned(time(NULL)));
}


CNEllipseDetector::~CNEllipseDetector(void)
{
}

void CNEllipseDetector::SetParameters(Size	szPreProcessingGaussKernel,
	double	dPreProcessingGaussSigma,
	float 	fThPosition,
	float	fMaxCenterDistance,
	int		iMinEdgeLength,
	float	fMinOrientedRectSide,
	float	fDistanceToEllipseContour,
	float	fMinScore,
	float	fMinReliability,
	int     iNs
	)
{
	szPreProcessingGaussKernel_ = szPreProcessingGaussKernel;
	dPreProcessingGaussSigma_ = dPreProcessingGaussSigma;
	fThrArcPosition_ = fThPosition;
	fMaxCenterDistance_ = fMaxCenterDistance;
	iMinEdgeLength_ = iMinEdgeLength;
	fMinOrientedRectSide_ = fMinOrientedRectSide;
	fDistanceToEllipseContour_ = fDistanceToEllipseContour;
	fMinScore_ = fMinScore;
	fMinReliability_ = fMinReliability;
	uNs_ = iNs;

	fMaxCenterDistance2_ = fMaxCenterDistance_ * fMaxCenterDistance_;

}

uint inline CNEllipseDetector::GenerateKey(uchar pair, ushort u, ushort v)
{
	return (pair << 30) + (u << 15) + v;
}

int CNEllipseDetector::FindMaxK(const int* v) const
{
	int max_val = 0;
	int max_idx = 0;
	for (int i = 0; i<ACC_R_SIZE; ++i)
	{
        (v[i] > max_val) ? max_val = v[i], max_idx = i : (int) NULL;
	}

	return max_idx + 90;
}

int CNEllipseDetector::FindMaxN(const int* v) const
{
	int max_val = 0;
	int max_idx = 0;
	for (int i = 0; i<ACC_N_SIZE; ++i)
	{
        (v[i] > max_val) ? max_val = v[i], max_idx = i : (int) NULL;
	}

	return max_idx;
}

int CNEllipseDetector::FindMaxA(const int* v) const
{
	int max_val = 0;
	int max_idx = 0;
	for (int i = 0; i<ACC_A_SIZE; ++i)
	{
        (v[i] > max_val) ? max_val = v[i], max_idx = i : (int) NULL;
	}

	return max_idx;
}

float CNEllipseDetector::GetMedianSlope(vector<Point2f>& med, Point2f& M, vector<float>& slopes)
{//input med slopes ;output:M return  
	// med		: vector of points  
	// M		: centroid of the points in med  
	// slopes	: vector of the slopes  

	unsigned iNofPoints = unsigned(med.size());
	//CV_Assert(iNofPoints >= 2);

	unsigned halfSize = iNofPoints >> 1;
	unsigned quarterSize = halfSize >> 1;

	vector<float> xx, yy;
	slopes.reserve(halfSize);
	xx.reserve(iNofPoints);
	yy.reserve(iNofPoints);

	for (unsigned i = 0; i < halfSize; ++i)
	{
		Point2f& p1 = med[i];
		Point2f& p2 = med[halfSize + i];

		xx.push_back(p1.x);
		xx.push_back(p2.x);
		yy.push_back(p1.y);
		yy.push_back(p2.y);

		float den = (p2.x - p1.x);
		float num = (p2.y - p1.y);

		if (den == 0) den = 0.00001f;

		slopes.push_back(num / den);
	}

	nth_element(slopes.begin(), slopes.begin() + quarterSize, slopes.end());
	nth_element(xx.begin(), xx.begin() + halfSize, xx.end());
	nth_element(yy.begin(), yy.begin() + halfSize, yy.end());
	M.x = xx[halfSize];
	M.y = yy[halfSize];

	return slopes[quarterSize];
}


void CNEllipseDetector::GetFastCenter(vector<Point>& e1, vector<Point>& e2, EllipseData& data)
{
	countOfGetFastCenter_++;
	data.isValid = true;

	unsigned size_1 = unsigned(e1.size());
	unsigned size_2 = unsigned(e2.size());

	unsigned hsize_1 = size_1 >> 1;
	unsigned hsize_2 = size_2 >> 1;

	Point& med1 = e1[hsize_1];
	Point& med2 = e2[hsize_2];

	Point2f M12, M34;
	float q2, q4;

	{// First to second Reference slope
		float dx_ref = float(e1[0].x - med2.x);
		float dy_ref = float(e1[0].y - med2.y);

		if (dy_ref == 0) dy_ref = 0.00001f;

		float m_ref = dy_ref / dx_ref;
		data.ra = m_ref;

		// Find points with same slope as reference
		vector<Point2f> med;
		med.reserve(hsize_2);

		unsigned minPoints = (uNs_ < hsize_2) ? uNs_ : hsize_2; // parallel chords

		vector<uint> indexes(minPoints);
		if (uNs_ < hsize_2)
		{	// hsize_2 bigger than uNs_
			unsigned iSzBin = hsize_2 / unsigned(uNs_);
			unsigned iIdx = hsize_2 + (iSzBin / 2);

			for (unsigned i = 0; i<uNs_; ++i)
			{
				indexes[i] = iIdx;
				iIdx += iSzBin;
			}
		}
		else
		{
			iota(indexes.begin(), indexes.end(), hsize_2); // convert to unsigned
		}
		for (uint ii = 0; ii<minPoints; ++ii)
		{// parallel chords in arc 2
			uint i = indexes[ii];

			float x1 = float(e2[i].x);
			float y1 = float(e2[i].y);

			uint begin = 0;
			uint end = size_1 - 1;
			// one point between the first and last point of parallel chords 1
			float xb = float(e1[begin].x);
			float yb = float(e1[begin].y);
			float res_begin = ((xb - x1) * dy_ref) - ((yb - y1) * dx_ref);
			int sign_begin = sgn(res_begin);
			if (sign_begin == 0)
			{
				// found
				med.push_back(Point2f((xb + x1)* 0.5f, (yb + y1)* 0.5f));
				continue;
			}

			float xe = float(e1[end].x);
			float ye = float(e1[end].y);
			float res_end = ((xe - x1) * dy_ref) - ((ye - y1) * dx_ref);
			int sign_end = sgn(res_end);
			if (sign_end == 0)
			{
				// found
				med.push_back(Point2f((xe + x1)* 0.5f, (ye + y1)* 0.5f));
				continue;
			}
			// if at the same side
			if ((sign_begin + sign_end) != 0)
			{// No parallel chords
				continue;
			}

			// search parallel chords
			uint j = (begin + end) >> 1;
			while (end - begin > 2)
			{
				float x2 = float(e1[j].x);
				float y2 = float(e1[j].y);
				float res = ((x2 - x1) * dy_ref) - ((y2 - y1) * dx_ref);
				int sign_res = sgn(res);

				if (sign_res == 0)
				{
					// found
					med.push_back(Point2f((x2 + x1)* 0.5f, (y2 + y1)* 0.5f));
					break;
				}

				if (sign_res + sign_begin == 0)
				{
					sign_end = sign_res;
					end = j;
				}
				else
				{
					sign_begin = sign_res;
					begin = j;
				}
				j = (begin + end) >> 1;
			}
			// search end error ?
			med.push_back(Point2f((e1[j].x + x1)* 0.5f, (e1[j].y + y1)* 0.5f));
		}

		if (med.size() < 2)
		{
			data.isValid = false;
			return;
		}

		q2 = GetMedianSlope(med, M12, data.Sa);//get Sa ta = q2 Ma
	}

	{// Second to first
		// Reference slope
		float dx_ref = float(med1.x - e2[0].x);
		float dy_ref = float(med1.y - e2[0].y);

		if (dy_ref == 0) dy_ref = 0.00001f;

		float m_ref = dy_ref / dx_ref;
		data.rb = m_ref;

		// Find points with same slope as reference
		vector<Point2f> med;
		med.reserve(hsize_1);

		uint minPoints = (uNs_ < hsize_1) ? uNs_ : hsize_1;

		vector<uint> indexes(minPoints);
		if (uNs_ < hsize_1)
		{
			unsigned iSzBin = hsize_1 / unsigned(uNs_);
			unsigned iIdx = hsize_1 + (iSzBin / 2);

			for (unsigned i = 0; i<uNs_; ++i)
			{
				indexes[i] = iIdx;
				iIdx += iSzBin;
			}
		}
		else
		{
			iota(indexes.begin(), indexes.end(), hsize_1);
		}


		for (uint ii = 0; ii<minPoints; ++ii)
		{
			uint i = indexes[ii];

			float x1 = float(e1[i].x);
			float y1 = float(e1[i].y);

			uint begin = 0;
			uint end = size_2 - 1;

			float xb = float(e2[begin].x);
			float yb = float(e2[begin].y);
			float res_begin = ((xb - x1) * dy_ref) - ((yb - y1) * dx_ref);
			int sign_begin = sgn(res_begin);
			if (sign_begin == 0)
			{
				// found
				med.push_back(Point2f((xb + x1)* 0.5f, (yb + y1)* 0.5f));
				continue;
			}

			float xe = float(e2[end].x);
			float ye = float(e2[end].y);
			float res_end = ((xe - x1) * dy_ref) - ((ye - y1) * dx_ref);
			int sign_end = sgn(res_end);
			if (sign_end == 0)
			{
				// found
				med.push_back(Point2f((xe + x1)* 0.5f, (ye + y1)* 0.5f));
				continue;
			}

			if ((sign_begin + sign_end) != 0)
			{
				continue;
			}

			uint j = (begin + end) >> 1;

			while (end - begin > 2)
			{
				float x2 = float(e2[j].x);
				float y2 = float(e2[j].y);
				float res = ((x2 - x1) * dy_ref) - ((y2 - y1) * dx_ref);
				int sign_res = sgn(res);

				if (sign_res == 0)
				{
					//found
					med.push_back(Point2f((x2 + x1)* 0.5f, (y2 + y1)* 0.5f));
					break;
				}

				if (sign_res + sign_begin == 0)
				{
					sign_end = sign_res;
					end = j;
				}
				else
				{
					sign_begin = sign_res;
					begin = j;
				}
				j = (begin + end) >> 1;
			}

			med.push_back(Point2f((e2[j].x + x1)* 0.5f, (e2[j].y + y1)* 0.5f));
		}

		if (med.size() < 2)
		{
			data.isValid = false;
			return;
		}
		q4 = GetMedianSlope(med, M34, data.Sb);
	}

	if (q2 == q4)
	{
		data.isValid = false;
		return;
	}

	float invDen = 1 / (q2 - q4);
	data.Cab.x = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
	data.Cab.y = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
	data.ta = q2;
	data.tb = q4;
	data.Ma = M12;
	data.Mb = M34;
}

#define	DISCARD_TCN2
void CNEllipseDetector::DetectEdges13(Mat1b& DP, VVP& points_1, VVP& points_3)
{
	// Vector of connected edge points
	VVP contours;
	int countedges = 0;
	// Labeling 8-connected edge points, discarding edge too small
	find_contours(DP, contours, iMinEdgeLength_); // put small arc edges to a vector
#ifdef DEBUG_PREPROCESSING
	Mat1b DP_show = DP.clone();
	show_contours(DP_show, contours, "PreProcessing->Contours13"); waitKey();
#endif
	
	int iContoursSize = int(contours.size());

	// VVP straightLines;
#ifdef DETECT_MARKER_X
	straightLines13_.clear();
#endif
	// For each edge
	for (int i = 0; i < iContoursSize; ++i)
	{
		VP& edgeSegment = contours[i];
#ifndef DISCARD_CONSTRAINT_OBOX

		// Selection strategy - Step 1 - See Sect [3.1.2] of the paper
		// Constraint on axes aspect ratio
		RotatedRect oriented = minAreaRect(edgeSegment);
		float o_min = min(oriented.size.width, oriented.size.height);

		if (o_min < fMinOrientedRectSide_)
		{
			countedges++;
			// straightLines.push_back(edgeSegment);
			continue;
		}
#endif
		// Order edge points of the same arc
		sort(edgeSegment.begin(), edgeSegment.end(), SortTopLeft2BottomRight);
		int iEdgeSegmentSize = int(edgeSegment.size());

		// Get extrema of the arc
		Point& left = edgeSegment[0];
		Point& right = edgeSegment[iEdgeSegmentSize - 1];
#ifndef DISCARD_TCN
#ifndef DISCARD_TCN2
		int flag = 0;
		for (int j = 0; j<iEdgeSegmentSize; j++){
			Point& mid = edgeSegment[j];
			float data[] = { left.x, left.y, 1, mid.x, mid.y, 1, right.x, right.y, 1 };
			Mat threePoints(Size(3, 3), CV_32FC1, data);
			double ans = determinant(threePoints);

			float dx = 1.0f*(left.x - right.x);
			float dy = 1.0f*(left.y - right.y);
			float edgelength2 = dx*dx + dy*dy;
			//double TCNl=ans/edgelength2;
			double TCNl = ans / (2 * sqrt(edgelength2));
			if (abs(TCNl)>T_TCN_L){
				flag = 1;
				break;
			}
		}
		if (0 == flag){
			countedges++;
			continue;
		}
#endif
#ifndef DISCARD_TCN1
		Point& mid = edgeSegment[iEdgeSegmentSize / 2];
        float data[] = { float(left.x), float(left.y), 1.0f, float(mid.x), float(mid.y),
                         1.0f, float(right.x), float(right.y), 1.0f };
		Mat threePoints(Size(3, 3), CV_32FC1, data);
		double ans = determinant(threePoints);

		float dx = 1.0f*(left.x - right.x);
		float dy = 1.0f*(left.y - right.y);
		float edgelength2 = dx*dx + dy*dy;
		// double TCNl = ans / edgelength2;
		double TCNl = ans / (2*sqrt(edgelength2));
		if (abs(TCNl)<T_TCN_L){
			countedges++;
			// straightLines.push_back(edgeSegment);
#ifdef DETECT_MARKER_X
			straightLines13_.push_back(edgeSegment);
#endif
			continue;
		}
#endif
#endif
		// Find convexity - See Sect [3.1.3] of the paper
		int iCountTop = 0;
		int xx = left.x;
		for (int k = 1; k < iEdgeSegmentSize; ++k)
		{
			if (edgeSegment[k].x == xx) continue;

			iCountTop += (edgeSegment[k].y - left.y);
			xx = edgeSegment[k].x;
		}

		int width = abs(right.x - left.x) + 1;
		int height = abs(right.y - left.y) + 1;
		int iCountBottom = (width * height) - iEdgeSegmentSize - iCountTop;

		if (iCountBottom > iCountTop)
		{	//1
			points_1.push_back(edgeSegment);
		}
		else if (iCountBottom < iCountTop)
		{	//3
			points_3.push_back(edgeSegment);
		}
	}
#ifdef DEBUG_PREPROCESSING
#ifdef DETECT_MARKER_X
	Mat1b DP_show_2 = DP.clone();
	show_contours(DP_show_2, straightLines13_, "PreProcessing->Contours_straight"); waitKey();
#endif
#endif
}


void CNEllipseDetector::DetectEdges24(Mat1b& DN, VVP& points_2, VVP& points_4)
{
	// Vector of connected edge points
	VVP contours;
	int countedges = 0;
	/// Labeling 8-connected edge points, discarding edge too small
	find_contours(DN, contours, iMinEdgeLength_);
#ifdef DEBUG_PREPROCESSING
	show_contours(DN, contours, "PreProcessing->Contours24"); waitKey();
#endif
	int iContoursSize = unsigned(contours.size());

#ifdef DETECT_MARKER_X
	straightLines24_.clear();
#endif

	// For each edge
	for (int i = 0; i < iContoursSize; ++i)
	{
		VP& edgeSegment = contours[i];

#ifndef DISCARD_CONSTRAINT_OBOX

		// Selection strategy - Step 1 - See Sect [3.1.2] of the paper
		// Constraint on axes aspect ratio
		RotatedRect oriented = minAreaRect(edgeSegment);
		float o_min = min(oriented.size.width, oriented.size.height);

		if (o_min < fMinOrientedRectSide_)
		{
			countedges++;
			continue;
		}
#endif
		// Order edge points of the same arc
		sort(edgeSegment.begin(), edgeSegment.end(), SortBottomLeft2TopRight);
		int iEdgeSegmentSize = unsigned(edgeSegment.size());

		// Get extrema of the arc
		Point& left = edgeSegment[0];
		Point& right = edgeSegment[iEdgeSegmentSize - 1];
#ifndef DISCARD_TCN
#ifndef DISCARD_TCN2
		int flag = 0;
		for (int j = 0; j<iEdgeSegmentSize; j++){
			Point& mid = edgeSegment[j];
			float data[] = { left.x, left.y, 1, mid.x, mid.y, 1, right.x, right.y, 1 };
			Mat threePoints(Size(3, 3), CV_32FC1, data);
			double ans = determinant(threePoints);

			float dx = 1.0f*(left.x - right.x);
			float dy = 1.0f*(left.y - right.y);
			float edgelength2 = dx*dx + dy*dy;
			//double TCNl=ans/edgelength2;
			double TCNl = ans / (2 * sqrt(edgelength2));
			if (abs(TCNl)>T_TCN_L){
				flag = 1;
				break;
			}
		}
		if (0 == flag){
			countedges++;
			continue;
		}
		else{
		}
#endif
#ifndef DISCARD_TCN1
		Point& mid = edgeSegment[iEdgeSegmentSize / 2];
        float data[] = { float(left.x), float(left.y), 1.0f, float(mid.x), float(mid.y),
                         1.0f, float(right.x), float(right.y), 1.0f };
		Mat threePoints(Size(3, 3), CV_32FC1, data);
		double ans = determinant(threePoints);

		float dx = 1.0f*(left.x - right.x);
		float dy = 1.0f*(left.y - right.y);
		float edgelength2 = dx*dx + dy*dy;
		// double TCNl = ans / edgelength2;
		double TCNl = ans / (2*sqrt(edgelength2));
		if (abs(TCNl)<T_TCN_L){
			countedges++;
#ifdef DETECT_MARKER_X
			straightLines24_.push_back(edgeSegment);
#endif
			continue;
		}
#endif
#endif
		// Find convexity - See Sect [3.1.3] of the paper
		int iCountBottom = 0;
		int xx = left.x;
		for (int k = 1; k < iEdgeSegmentSize; ++k)
		{
			if (edgeSegment[k].x == xx) continue;

			iCountBottom += (left.y - edgeSegment[k].y);
			xx = edgeSegment[k].x;
		}

		int width = abs(right.x - left.x) + 1;
		int height = abs(right.y - left.y) + 1;
		int iCountTop = (width *height) - iEdgeSegmentSize - iCountBottom;

		if (iCountBottom > iCountTop)
		{
			//2
			points_2.push_back(edgeSegment);
		}
		else if (iCountBottom < iCountTop)
		{
			//4
			points_4.push_back(edgeSegment);
		}
	}
}

// Most important function for detecting ellipses. See Sect[3.2.3] of the paper
void CNEllipseDetector::FindEllipses(Point2f& center,
	VP& edge_i, VP& edge_j, VP& edge_k,
	EllipseData& data_ij, EllipseData& data_ik,
	vector<Ellipse>& ellipses)
{
	countOfFindEllipse_++;
	// Find ellipse parameters

	// 0-initialize accumulators
	memset(accN, 0, sizeof(int)*ACC_N_SIZE);
	memset(accR, 0, sizeof(int)*ACC_R_SIZE);
	memset(accA, 0, sizeof(int)*ACC_A_SIZE);

	// Tac(3); //estimation

	// Get size of the 4 vectors of slopes (2 pairs of arcs)
	int sz_ij1 = int(data_ij.Sa.size());
	int sz_ij2 = int(data_ij.Sb.size());
	int sz_ik1 = int(data_ik.Sa.size());
	int sz_ik2 = int(data_ik.Sb.size());

	// Get the size of the 3 arcs
	size_t sz_ei = edge_i.size();
	size_t sz_ej = edge_j.size();
	size_t sz_ek = edge_k.size();

	// Center of the estimated ellipse
	float a0 = center.x;
	float b0 = center.y;


	// Estimation of remaining parameters
	// Uses 4 combinations of parameters. See Table 1 and Sect [3.2.3] of the paper.
	//ij1 and ik
	{
		float q1 = data_ij.ra;
		float q3 = data_ik.ra;
		float q5 = data_ik.rb;

		for (int ij1 = 0; ij1 < sz_ij1; ++ij1)
		{
			float q2 = data_ij.Sa[ij1]; // need iter \A3\BF

			float q1xq2 = q1*q2;
			// ij1 and ik1
			for (int ik1 = 0; ik1 < sz_ik1; ++ik1)
			{
				float q4 = data_ik.Sa[ik1]; // need iter \A3\BF

				float q3xq4 = q3*q4;

				// See Eq. [13-18] in the paper

				float a = (q1xq2 - q3xq4);//gama
				float b = (q3xq4 + 1)*(q1 + q2) - (q1xq2 + 1)*(q3 + q4);//beta
				float Kp = (-b + sqrt(b*b + 4 * a*a)) / (2 * a);//K+
				float zplus = ((q1 - Kp)*(q2 - Kp)) / ((1 + q1*Kp)*(1 + q2*Kp));
				// check  zplus and K is linear
				if (zplus >= 0.0f) continue;

				float Np = sqrt(-zplus);//N+
				float rho = atan(Kp);//rho tmp
				int rhoDeg;
				if (Np > 1.f)
				{
					Np = 1.f / Np;
					rhoDeg = cvRound((rho * 180 / CV_PI) + 180) % 180; // [0,180)					
				}
				else
				{
					rhoDeg = cvRound((rho * 180 / CV_PI) + 90) % 180; // [0,180)//rho angel rep and norm
				}

				int iNp = cvRound(Np * 100); // [0, 100]

				if (0 <= iNp	&& iNp < ACC_N_SIZE &&
					0 <= rhoDeg	&& rhoDeg < ACC_R_SIZE
					)
				{   // why iter all. beacause zplus and K is not linear?
					++accN[iNp];	// Increment N accumulator
					++accR[rhoDeg];	// Increment R accumulator
				}
			}
			// ij1 and ik2
			for (int ik2 = 0; ik2 < sz_ik2; ++ik2)
			{
				float q4 = data_ik.Sb[ik2];

				float q5xq4 = q5*q4;

				// See Eq. [13-18] in the paper

				float a = (q1xq2 - q5xq4);
				float b = (q5xq4 + 1)*(q1 + q2) - (q1xq2 + 1)*(q5 + q4);
				float Kp = (-b + sqrt(b*b + 4 * a*a)) / (2 * a);
				float zplus = ((q1 - Kp)*(q2 - Kp)) / ((1 + q1*Kp)*(1 + q2*Kp));

				if (zplus >= 0.0f)
				{
					continue;
				}

				float Np = sqrt(-zplus);
				float rho = atan(Kp);
				int rhoDeg;
				if (Np > 1.f)
				{
					Np = 1.f / Np;
					rhoDeg = cvRound((rho * 180 / CV_PI) + 180) % 180; // [0,180)					
				}
				else
				{
					rhoDeg = cvRound((rho * 180 / CV_PI) + 90) % 180; // [0,180)
				}

				int iNp = cvRound(Np * 100); // [0, 100]

				if (0 <= iNp	&& iNp < ACC_N_SIZE &&
					0 <= rhoDeg	&& rhoDeg < ACC_R_SIZE
					)
				{
					++accN[iNp];		// Increment N accumulator
					++accR[rhoDeg];		// Increment R accumulator
				}
			}

		}
	}

	//ij2 and ik
	{
		float q1 = data_ij.rb;
		float q3 = data_ik.rb;
		float q5 = data_ik.ra;

		for (int ij2 = 0; ij2 < sz_ij2; ++ij2)
		{
			float q2 = data_ij.Sb[ij2];

			float q1xq2 = q1*q2;
			//ij2 and ik2
			for (int ik2 = 0; ik2 < sz_ik2; ++ik2)
			{
				float q4 = data_ik.Sb[ik2];

				float q3xq4 = q3*q4;

				// See Eq. [13-18] in the paper

				float a = (q1xq2 - q3xq4);
				float b = (q3xq4 + 1)*(q1 + q2) - (q1xq2 + 1)*(q3 + q4);
				float Kp = (-b + sqrt(b*b + 4 * a*a)) / (2 * a);
				float zplus = ((q1 - Kp)*(q2 - Kp)) / ((1 + q1*Kp)*(1 + q2*Kp));

				if (zplus >= 0.0f)
				{
					continue;
				}

				float Np = sqrt(-zplus);
				float rho = atan(Kp);
				int rhoDeg;
				if (Np > 1.f)
				{
					Np = 1.f / Np;
					rhoDeg = cvRound((rho * 180 / CV_PI) + 180) % 180; // [0,180)
				}
				else
				{
					rhoDeg = cvRound((rho * 180 / CV_PI) + 90) % 180; // [0,180)
				}

				int iNp = cvRound(Np * 100); // [0, 100]

				if (0 <= iNp	&& iNp < ACC_N_SIZE &&
					0 <= rhoDeg	&& rhoDeg < ACC_R_SIZE
					)
				{
					++accN[iNp];		// Increment N accumulator
					++accR[rhoDeg];		// Increment R accumulator
				}
			}

			//ij2 and ik1
			for (int ik1 = 0; ik1 < sz_ik1; ++ik1)
			{
				float q4 = data_ik.Sa[ik1];

				float q5xq4 = q5*q4;

				// See Eq. [13-18] in the paper

				float a = (q1xq2 - q5xq4);
				float b = (q5xq4 + 1)*(q1 + q2) - (q1xq2 + 1)*(q5 + q4);
				float Kp = (-b + sqrt(b*b + 4 * a*a)) / (2 * a);
				float zplus = ((q1 - Kp)*(q2 - Kp)) / ((1 + q1*Kp)*(1 + q2*Kp));

				if (zplus >= 0.0f)
				{
					continue;
				}

				float Np = sqrt(-zplus);
				float rho = atan(Kp);
				int rhoDeg;
				if (Np > 1.f)
				{
					Np = 1.f / Np;
					rhoDeg = cvRound((rho * 180 / CV_PI) + 180) % 180; // [0,180)
				}
				else
				{
					rhoDeg = cvRound((rho * 180 / CV_PI) + 90) % 180; // [0,180)
				}

				int iNp = cvRound(Np * 100); // [0, 100]

				if (0 <= iNp	&& iNp < ACC_N_SIZE &&
					0 <= rhoDeg	&& rhoDeg < ACC_R_SIZE
					)
				{
					++accN[iNp];		// Increment N accumulator
					++accR[rhoDeg];		// Increment R accumulator
				}
			}

		}
	}

	// Find peak in N and K accumulator
	int iN = FindMaxN(accN);
	int iK = FindMaxK(accR);

	// Recover real values
	float fK = float(iK);
	float Np = float(iN) * 0.01f;
	float rho = fK * float(CV_PI) / 180.f;	//deg 2 rad
	float Kp = tan(rho);

	// Estimate A. See Eq. [19 - 22] in Sect [3.2.3] of the paper  
	// 
	// may optm
	for (ushort l = 0; l < sz_ei; ++l)
	{
		Point& pp = edge_i[l];
		float sk = 1.f / sqrt(Kp*Kp + 1.f);//cos rho
		float x0 = ((pp.x - a0) * sk) + (((pp.y - b0)*Kp) * sk);//may optm
		float y0 = -(((pp.x - a0) * Kp) * sk) + ((pp.y - b0) * sk);//may optm
		float Ax = sqrt((x0*x0*Np*Np + y0*y0) / ((Np*Np)*(1.f + Kp*Kp)));
		int A = cvRound(abs(Ax / cos(rho)));//may optm
		if ((0 <= A) && (A < ACC_A_SIZE))
		{
			++accA[A];
		}
	}

	for (ushort l = 0; l < sz_ej; ++l)
	{
		Point& pp = edge_j[l];
		float sk = 1.f / sqrt(Kp*Kp + 1.f);
		float x0 = ((pp.x - a0) * sk) + (((pp.y - b0)*Kp) * sk);
		float y0 = -(((pp.x - a0) * Kp) * sk) + ((pp.y - b0) * sk);
		float Ax = sqrt((x0*x0*Np*Np + y0*y0) / ((Np*Np)*(1.f + Kp*Kp)));
		int A = cvRound(abs(Ax / cos(rho)));
		if ((0 <= A) && (A < ACC_A_SIZE))
		{
			++accA[A];
		}
	}

	for (ushort l = 0; l < sz_ek; ++l)
	{
		Point& pp = edge_k[l];
		float sk = 1.f / sqrt(Kp*Kp + 1.f);
		float x0 = ((pp.x - a0) * sk) + (((pp.y - b0)*Kp) * sk);
		float y0 = -(((pp.x - a0) * Kp) * sk) + ((pp.y - b0) * sk);
		float Ax = sqrt((x0*x0*Np*Np + y0*y0) / ((Np*Np)*(1.f + Kp*Kp)));
		int A = cvRound(abs(Ax / cos(rho)));
		if ((0 <= A) && (A < ACC_A_SIZE))
		{
			++accA[A];
		}
	}

	// Find peak in A accumulator
	int A = FindMaxA(accA);
	float fA = float(A);

	// Find B value. See Eq [23] in the paper
	float fB = abs(fA * Np);

	// Got all ellipse parameters!
	Ellipse ell(a0, b0, fA, fB, fmod(rho + float(CV_PI)*2.f, float(CV_PI)));

	// Toc(3); //estimation
	// Tac(4); //validation

	// Get the score. See Sect [3.3.1] in the paper

	// Find the number of edge pixel lying on the ellipse
	float _cos = cos(-ell._rad);
	float _sin = sin(-ell._rad);

	float invA2 = 1.f / (ell._a * ell._a);
	float invB2 = 1.f / (ell._b * ell._b);

	float invNofPoints = 1.f / float(sz_ei + sz_ej + sz_ek);
	int counter_on_perimeter = 0;

	for (ushort l = 0; l < sz_ei; ++l)
	{
		float tx = float(edge_i[l].x) - ell._xc;
		float ty = float(edge_i[l].y) - ell._yc;
		float rx = (tx*_cos - ty*_sin);
		float ry = (tx*_sin + ty*_cos);

		float h = (rx*rx)*invA2 + (ry*ry)*invB2;
		if (abs(h - 1.f) < fDistanceToEllipseContour_)
		{
			++counter_on_perimeter;
		}
	}

	for (ushort l = 0; l < sz_ej; ++l)
	{
		float tx = float(edge_j[l].x) - ell._xc;
		float ty = float(edge_j[l].y) - ell._yc;
		float rx = (tx*_cos - ty*_sin);
		float ry = (tx*_sin + ty*_cos);

		float h = (rx*rx)*invA2 + (ry*ry)*invB2;
		if (abs(h - 1.f) < fDistanceToEllipseContour_)
		{
			++counter_on_perimeter;
		}
	}

	for (ushort l = 0; l < sz_ek; ++l)
	{
		float tx = float(edge_k[l].x) - ell._xc;
		float ty = float(edge_k[l].y) - ell._yc;
		float rx = (tx*_cos - ty*_sin);
		float ry = (tx*_sin + ty*_cos);

		float h = (rx*rx)*invA2 + (ry*ry)*invB2;
		if (abs(h - 1.f) < fDistanceToEllipseContour_)
		{
			++counter_on_perimeter;
		}
	}

	//no points found on the ellipse
	if (counter_on_perimeter <= 0)
	{
		// Toc(4); //validation
		return;
	}


	// Compute score
	float score = float(counter_on_perimeter) * invNofPoints;
	if (score < fMinScore_)
	{
		// Toc(4); //validation
		return;
	}

	// Compute reliability	
	// this metric is not described in the paper, mostly due to space limitations.
	// The main idea is that for a given ellipse (TD) even if the score is high, the arcs 
	// can cover only a small amount of the contour of the estimated ellipse. 
	// A low reliability indicate that the arcs form an elliptic shape by chance, but do not underlie
	// an actual ellipse. The value is normalized between 0 and 1. 
	// The default value is 0.4.

	// It is somehow similar to the "Angular Circumreference Ratio" saliency criteria 
	// as in the paper: 
	// D. K. Prasad, M. K. Leung, S.-Y. Cho, Edge curvature and convexity
	// based ellipse detection method, Pattern Recognition 45 (2012) 3204-3221.

	float di, dj, dk;
	{
		Point2f p1(float(edge_i[0].x), float(edge_i[0].y));
		Point2f p2(float(edge_i[sz_ei - 1].x), float(edge_i[sz_ei - 1].y));
		p1.x -= ell._xc;
		p1.y -= ell._yc;
		p2.x -= ell._xc;
		p2.y -= ell._yc;
		Point2f r1((p1.x*_cos - p1.y*_sin), (p1.x*_sin + p1.y*_cos));
		Point2f r2((p2.x*_cos - p2.y*_sin), (p2.x*_sin + p2.y*_cos));
		di = abs(r2.x - r1.x) + abs(r2.y - r1.y);
	}
	{
		Point2f p1(float(edge_j[0].x), float(edge_j[0].y));
		Point2f p2(float(edge_j[sz_ej - 1].x), float(edge_j[sz_ej - 1].y));
		p1.x -= ell._xc;
		p1.y -= ell._yc;
		p2.x -= ell._xc;
		p2.y -= ell._yc;
		Point2f r1((p1.x*_cos - p1.y*_sin), (p1.x*_sin + p1.y*_cos));
		Point2f r2((p2.x*_cos - p2.y*_sin), (p2.x*_sin + p2.y*_cos));
		dj = abs(r2.x - r1.x) + abs(r2.y - r1.y);
	}
	{
		Point2f p1(float(edge_k[0].x), float(edge_k[0].y));
		Point2f p2(float(edge_k[sz_ek - 1].x), float(edge_k[sz_ek - 1].y));
		p1.x -= ell._xc;
		p1.y -= ell._yc;
		p2.x -= ell._xc;
		p2.y -= ell._yc;
		Point2f r1((p1.x*_cos - p1.y*_sin), (p1.x*_sin + p1.y*_cos));
		Point2f r2((p2.x*_cos - p2.y*_sin), (p2.x*_sin + p2.y*_cos));
		dk = abs(r2.x - r1.x) + abs(r2.y - r1.y);
	}

	// This allows to get rid of thick edges
	float rel = min(1.f, ((di + dj + dk) / (3 * (ell._a + ell._b))));

	if (rel < fMinReliability_)
	{
		// Toc(4); //validation
		return;
	}

	// Assign the new score!
	ell._score = (score + rel) * 0.5f;//need to change

	// The tentative detection has been confirmed. Save it!
	ellipses.push_back(ell);

	// Toc(4); // Validation
};

// Get the coordinates of the center, given the intersection of the estimated lines. See Fig. [8] in Sect [3.2.3] in the paper.
Point2f CNEllipseDetector::GetCenterCoordinates(EllipseData& data_ij, EllipseData& data_ik)
{
	float xx[7];
	float yy[7];

	xx[0] = data_ij.Cab.x;
	xx[1] = data_ik.Cab.x;
	yy[0] = data_ij.Cab.y;
	yy[1] = data_ik.Cab.y;

	{
		//1-1
		float q2 = data_ij.ta;
		float q4 = data_ik.ta;
		Point2f& M12 = data_ij.Ma;
		Point2f& M34 = data_ik.Ma;

		float invDen = 1 / (q2 - q4);
		xx[2] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
		yy[2] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
	}

	{
		//1-2
		float q2 = data_ij.ta;
		float q4 = data_ik.tb;
		Point2f& M12 = data_ij.Ma;
		Point2f& M34 = data_ik.Mb;

		float invDen = 1 / (q2 - q4);
		xx[3] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
		yy[3] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
	}

	{
		//2-2
		float q2 = data_ij.tb;
		float q4 = data_ik.tb;
		Point2f& M12 = data_ij.Mb;
		Point2f& M34 = data_ik.Mb;

		float invDen = 1 / (q2 - q4);
		xx[4] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
		yy[4] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
	}

	{
		//2-1
		float q2 = data_ij.tb;
		float q4 = data_ik.ta;
		Point2f& M12 = data_ij.Mb;
		Point2f& M34 = data_ik.Ma;

		float invDen = 1 / (q2 - q4);
		xx[5] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
		yy[5] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
	}

	xx[6] = (xx[0] + xx[1]) * 0.5f;
	yy[6] = (yy[0] + yy[1]) * 0.5f;


	// Median
	nth_element(xx, xx + 3, xx + 7);
	nth_element(yy, yy + 3, yy + 7);
	float xc = xx[3];
	float yc = yy[3];

	return Point2f(xc, yc);
};

//123456 124 80 48 246
//#define T124 pil,pim,pif,pjl,pjm,pjf

#define T124 pjf,pjm,pjl,pif,pim,pil // origin
#define T231 pil,pim,pif,pjf,pjm,pjl
#define T342 pif,pim,pil,pjf,pjm,pjl
#define T413 pif,pim,pil,pjl,pjm,pjf



// Verify triplets of arcs with convexity: i=1, j=2, k=4
void CNEllipseDetector::Triplets124(VVP& pi,
	VVP& pj,
	VVP& pk,
	unordered_map<uint, EllipseData>& data,
	vector<Ellipse>& ellipses
	)
{
	// get arcs length
	ushort sz_i = ushort(pi.size());
	ushort sz_j = ushort(pj.size());
	ushort sz_k = ushort(pk.size());

	// For each edge i
	for (ushort i = 0; i < sz_i; ++i)
	{
		VP& edge_i = pi[i];
		ushort sz_ei = ushort(edge_i.size());

		Point& pif = edge_i[0];
		Point& pim = edge_i[sz_ei / 2];
		Point& pil = edge_i[sz_ei - 1];

		// 1,2 -> reverse 1, swap
		VP rev_i(edge_i.size());
		reverse_copy(edge_i.begin(), edge_i.end(), rev_i.begin());

		// For each edge j
		for (ushort j = 0; j < sz_j; ++j)
		{
			VP& edge_j = pj[j];
			ushort sz_ej = ushort(edge_j.size());

			Point& pjf = edge_j[0];
			Point& pjm = edge_j[sz_ej / 2];
			Point& pjl = edge_j[sz_ej - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
			// CONSTRAINTS on position
			if (pjl.x > pif.x + fThrArcPosition_) //is right	
				continue;
#endif
#ifdef CONSTRAINT_CNC_1
			// cnc constraint1  2se se1//pil,pim,pif,pjf,pjm,pjl pjf,pjm,pjl,pif,pim,pil
			if (fabs(value4SixPoints(T124) - 1)>T_CNC)
				continue;
#endif
			uint key_ij = GenerateKey(PAIR_12, i, j);

			//for each edge k
			for (ushort k = 0; k < sz_k; ++k)
			{
				VP& edge_k = pk[k];
				ushort sz_ek = ushort(edge_k.size());

                // Point& pkf = edge_k[0];
                // Point& pkm = edge_k[sz_ek / 2];
				Point& pkl = edge_k[sz_ek - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
				//CONSTRAINTS on position
				if (pkl.y < pil.y - fThrArcPosition_)
					continue;
#endif

#ifdef CONSTRAINT_CNC_2
				// cnc constraint2
				if (fabs(value4SixPoints(pif, pim, pil, pkf, pkm, pkl) - 1)>T_CNC)
					continue;
#endif
#ifdef CONSTRAINT_CNC_3
				// cnc constraint3
				if (fabs(value4SixPoints(pjf, pjm, pjl, pkf, pkm, pkl) - 1)>T_CNC)
					continue;
#endif

				uint key_ik = GenerateKey(PAIR_14, i, k);

				// Find centers

				EllipseData data_ij, data_ik;

				// If the data for the pair i-j have not been computed yet
				if (data.count(key_ij) == 0)
				{
					//1,2 -> reverse 1, swap

					// Compute data!
					GetFastCenter(edge_j, rev_i, data_ij);
					// Insert computed data in the hash table
					data.insert(pair<uint, EllipseData>(key_ij, data_ij));
				}
				else
				{
					// Otherwise, just lookup the data in the hash table
					data_ij = data.at(key_ij);
				}

				// If the data for the pair i-k have not been computed yet
				if (data.count(key_ik) == 0)
				{
					//1,4 -> ok

					// Compute data!
					GetFastCenter(edge_i, edge_k, data_ik);
					// Insert computed data in the hash table
					data.insert(pair<uint, EllipseData>(key_ik, data_ik));
				}
				else
				{
					// Otherwise, just lookup the data in the hash table
					data_ik = data.at(key_ik);
				}

				// INVALID CENTERS
				if (!data_ij.isValid || !data_ik.isValid)
				{
					continue;
				}

#ifndef DISCARD_CONSTRAINT_CENTER
				// Selection strategy - Step 3. See Sect [3.2.2] in the paper
				// The computed centers are not close enough
				if (ed2(data_ij.Cab, data_ik.Cab) > fMaxCenterDistance2_)
				{
					//discard
					continue;
				}
#endif
				// If all constraints of the selection strategy have been satisfied, 
				// we can start estimating the ellipse parameters

				// Find ellipse parameters

				// Get the coordinates of the center (xc, yc)
				Point2f center = GetCenterCoordinates(data_ij, data_ik);

				// Find remaining paramters (A,B,rho)
				FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ellipses);
			}
		}
	}
};



void CNEllipseDetector::Triplets231(VVP& pi,
	VVP& pj,
	VVP& pk,
	unordered_map<uint, EllipseData>& data,
	vector<Ellipse>& ellipses
	)
{
	ushort sz_i = ushort(pi.size());
	ushort sz_j = ushort(pj.size());
	ushort sz_k = ushort(pk.size());

	// For each edge i
	for (ushort i = 0; i < sz_i; ++i)
	{
		VP& edge_i = pi[i];
		ushort sz_ei = ushort(edge_i.size());

		Point& pif = edge_i[0];
		Point& pim = edge_i[sz_ei / 2];
		Point& pil = edge_i[sz_ei - 1];

		VP rev_i(edge_i.size());
		reverse_copy(edge_i.begin(), edge_i.end(), rev_i.begin());

		// For each edge j
		for (ushort j = 0; j < sz_j; ++j)
		{
			VP& edge_j = pj[j];
			ushort sz_ej = ushort(edge_j.size());

			Point& pjf = edge_j[0];
			Point& pjm = edge_j[sz_ej / 2];
			Point& pjl = edge_j[sz_ej - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
			// CONSTRAINTS on position
			if (pjf.y < pif.y - fThrArcPosition_)
			{
				//discard
				continue;
			}
#endif

#ifdef CONSTRAINT_CNC_1
			// cnc constraint1 2es se3 //pif,pim,pil,pjf,pjm,pjl pil,pim,pif,pjf,pjm,pjl
			if (fabs(value4SixPoints(T231) - 1)>T_CNC)
				continue;
#endif

			VP rev_j(edge_j.size());
			reverse_copy(edge_j.begin(), edge_j.end(), rev_j.begin());

			uint key_ij = GenerateKey(PAIR_23, i, j);

			// For each edge k
			for (ushort k = 0; k < sz_k; ++k)
			{
				VP& edge_k = pk[k];
                // ushort sz_ek = ushort(edge_k.size());

				Point& pkf = edge_k[0];
                // Point& pkm = edge_k[sz_ek / 2];
                // Point& pkl = edge_k[sz_ek - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
				// CONSTRAINTS on position
				if (pkf.x < pil.x - fThrArcPosition_)
				{
					//discard
					continue;
				}
#endif

#ifdef CONSTRAINT_CNC_2
				// cnc constraint2
				if (fabs(value4SixPoints(pif, pim, pil, pkf, pkm, pkl) - 1)>T_CNC)
					continue;
#endif
#ifdef CONSTRAINT_CNC_3
				// cnc constraint3
				if (fabs(value4SixPoints(pjf, pjm, pjl, pkf, pkm, pkl) - 1)>T_CNC)
					continue;
#endif
				uint key_ik = GenerateKey(PAIR_12, k, i);

				// Find centers

				EllipseData data_ij, data_ik;

				if (data.count(key_ij) == 0)
				{
					// 2,3 -> reverse 2,3

					GetFastCenter(rev_i, rev_j, data_ij);
					data.insert(pair<uint, EllipseData>(key_ij, data_ij));
				}
				else
				{
					data_ij = data.at(key_ij);
				}

				if (data.count(key_ik) == 0)
				{
					// 2,1 -> reverse 1
					VP rev_k(edge_k.size());
					reverse_copy(edge_k.begin(), edge_k.end(), rev_k.begin());

					GetFastCenter(edge_i, rev_k, data_ik);
					data.insert(pair<uint, EllipseData>(key_ik, data_ik));
				}
				else
				{
					data_ik = data.at(key_ik);
				}

				// INVALID CENTERS
				if (!data_ij.isValid || !data_ik.isValid)
				{
					continue;
				}

#ifndef DISCARD_CONSTRAINT_CENTER
				// CONSTRAINT ON CENTERS
				if (ed2(data_ij.Cab, data_ik.Cab) > fMaxCenterDistance2_)
				{
					//discard
					continue;
				}
#endif
				// Find ellipse parameters
				Point2f center = GetCenterCoordinates(data_ij, data_ik);

				FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ellipses);

			}
		}
	}
};


void CNEllipseDetector::Triplets342(VVP& pi,
	VVP& pj,
	VVP& pk,
	unordered_map<uint, EllipseData>& data,
	vector<Ellipse>& ellipses
	)
{
	ushort sz_i = ushort(pi.size());
	ushort sz_j = ushort(pj.size());
	ushort sz_k = ushort(pk.size());

	// For each edge i
	for (ushort i = 0; i < sz_i; ++i)
	{
		VP& edge_i = pi[i];
		ushort sz_ei = ushort(edge_i.size());

		Point& pif = edge_i[0];
		Point& pim = edge_i[sz_ei / 2];
		Point& pil = edge_i[sz_ei - 1];

		VP rev_i(edge_i.size());
		reverse_copy(edge_i.begin(), edge_i.end(), rev_i.begin());

		// For each edge j
		for (ushort j = 0; j < sz_j; ++j)
		{
			VP& edge_j = pj[j];
			ushort sz_ej = ushort(edge_j.size());

			Point& pjf = edge_j[0];
			Point& pjm = edge_j[sz_ej / 2];
			Point& pjl = edge_j[sz_ej - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
			//CONSTRAINTS on position
			if (pjf.x < pil.x - fThrArcPosition_) 		//is left
			{
				//discard
				continue;
			}
#endif

#ifdef CONSTRAINT_CNC_1
			// cnc constraint1 3se se4 // pil,pim,pif,pjf,pjm,pjl pif,pim,pil,pjf,pjm,pjl
			if (fabs(value4SixPoints(T342) - 1)>T_CNC)
				continue;
#endif

			VP rev_j(edge_j.size());
			reverse_copy(edge_j.begin(), edge_j.end(), rev_j.begin());

			uint key_ij = GenerateKey(PAIR_34, i, j);

			// For each edge k
			for (ushort k = 0; k < sz_k; ++k)
			{
				VP& edge_k = pk[k];
                // ushort sz_ek = ushort(edge_k.size());

				Point& pkf = edge_k[0];
                // Point& pkm = edge_k[sz_ek / 2];
                // Point& pkl = edge_k[sz_ek - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
				//CONSTRAINTS on position
				if (pkf.y > pif.y + fThrArcPosition_)
				{
					//discard
					continue;
				}
#endif

#ifdef CONSTRAINT_CNC_2
				// cnc constraint2
				if (fabs(value4SixPoints(pif, pim, pil, pkf, pkm, pkl) - 1)>T_CNC)
					continue;
#endif
#ifdef CONSTRAINT_CNC_3
				// cnc constraint3
				if (fabs(value4SixPoints(pjf, pjm, pjl, pkf, pkm, pkl) - 1)>T_CNC)
					continue;
#endif
				uint key_ik = GenerateKey(PAIR_23, k, i);

				// Find centers

				EllipseData data_ij, data_ik;

				if (data.count(key_ij) == 0)
				{
					//3,4 -> reverse 4

					GetFastCenter(edge_i, rev_j, data_ij);
					data.insert(pair<uint, EllipseData>(key_ij, data_ij));
				}
				else
				{
					data_ij = data.at(key_ij);
				}

				if (data.count(key_ik) == 0)
				{
					//3,2 -> reverse 3,2

					VP rev_k(edge_k.size());
					reverse_copy(edge_k.begin(), edge_k.end(), rev_k.begin());

					GetFastCenter(rev_i, rev_k, data_ik);

					data.insert(pair<uint, EllipseData>(key_ik, data_ik));
				}
				else
				{
					data_ik = data.at(key_ik);
				}


				// INVALID CENTERS
				if (!data_ij.isValid || !data_ik.isValid)
				{
					continue;
				}

#ifndef DISCARD_CONSTRAINT_CENTER
				// CONSTRAINT ON CENTERS
				if (ed2(data_ij.Cab, data_ik.Cab) > fMaxCenterDistance2_)
				{
					//discard
					continue;
				}
#endif
				// Find ellipse parameters
				Point2f center = GetCenterCoordinates(data_ij, data_ik);
				FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ellipses);
			}
		}

	}
};



void CNEllipseDetector::Triplets413(VVP& pi,
	VVP& pj,
	VVP& pk,
	unordered_map<uint, EllipseData>& data,
	vector<Ellipse>& ellipses
	)
{
	ushort sz_i = ushort(pi.size());
	ushort sz_j = ushort(pj.size());
	ushort sz_k = ushort(pk.size());

	// For each edge i
	for (ushort i = 0; i < sz_i; ++i)
	{
		VP& edge_i = pi[i];
		ushort sz_ei = ushort(edge_i.size());

		Point& pif = edge_i[0];
		Point& pim = edge_i[sz_ei / 2];
		Point& pil = edge_i[sz_ei - 1];

		VP rev_i(edge_i.size());
		reverse_copy(edge_i.begin(), edge_i.end(), rev_i.begin());

		// For each edge j
		for (ushort j = 0; j < sz_j; ++j)
		{
			VP& edge_j = pj[j];
			ushort sz_ej = ushort(edge_j.size());

			Point& pjf = edge_j[0];
			Point& pjm = edge_j[sz_ej / 2];
			Point& pjl = edge_j[sz_ej - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
			//CONSTRAINTS on position
			if (pjl.y > pil.y + fThrArcPosition_)  		//is below
			{
				//discard
				continue;
			}
#endif

#ifdef CONSTRAINT_CNC_1
			// cnc constraint1 4se es1//pif,pim,pil,pjf,pjm,pjl pil,pim,pif,pjl,pjm,pjf pif,pim,pil,pjl,pjm,pjf
			if (fabs(value4SixPoints(T413) - 1)>T_CNC)
				continue;
#endif

			uint key_ij = GenerateKey(PAIR_14, j, i);

			// For each edge k
			for (ushort k = 0; k < sz_k; ++k)
			{
				VP& edge_k = pk[k];
				ushort sz_ek = ushort(edge_k.size());

                // Point& pkf = edge_k[0];
                // Point& pkm = edge_k[sz_ek / 2];
				Point& pkl = edge_k[sz_ek - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
				//CONSTRAINTS on position
				if (pkl.x > pif.x + fThrArcPosition_)
				{
					//discard
					continue;
				}
#endif

#ifdef CONSTRAINT_CNC_2
				// cnc constraint2
				if (fabs(value4SixPoints(pif, pim, pil, pkf, pkm, pkl) - 1)>T_CNC)
					continue;
#endif
#ifdef CONSTRAINT_CNC_3
				// cnc constraint2
				if (fabs(value4SixPoints(pjf, pjm, pjl, pkf, pkm, pkl) - 1)>T_CNC)
					continue;
#endif
				uint key_ik = GenerateKey(PAIR_34, k, i);

				// Find centers

				EllipseData data_ij, data_ik;

				if (data.count(key_ij) == 0)
				{
					// 4,1 -> OK
					GetFastCenter(edge_i, edge_j, data_ij);
					data.insert(pair<uint, EllipseData>(key_ij, data_ij));
				}
				else
				{
					data_ij = data.at(key_ij);
				}

				if (data.count(key_ik) == 0)
				{
					// 4,3 -> reverse 4
					GetFastCenter(rev_i, edge_k, data_ik);
					data.insert(pair<uint, EllipseData>(key_ik, data_ik));
				}
				else
				{
					data_ik = data.at(key_ik);
				}

				// INVALID CENTERS
				if (!data_ij.isValid || !data_ik.isValid)
				{
					continue;
				}

#ifndef DISCARD_CONSTRAINT_CENTER
				// CONSTRAIN ON CENTERS
				if (ed2(data_ij.Cab, data_ik.Cab) > fMaxCenterDistance2_)
				{
					//discard
					continue;
				}
#endif
				// Find ellipse parameters
				Point2f center = GetCenterCoordinates(data_ij, data_ik);

				FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ellipses);

			}
		}
	}
};


void CNEllipseDetector::RemoveShortEdges(Mat1b& edges, Mat1b& clean)
{
	VVP contours;

	// Labeling and contraints on length
	find_contours(edges, contours, iMinEdgeLength_);

	int iContoursSize = contours.size();
	for (int i = 0; i < iContoursSize; ++i)
	{
		VP& edge = contours[i];
		unsigned szEdge = edge.size();

		// Constraint on axes aspect ratio
		RotatedRect oriented = minAreaRect(edge);
		if (oriented.size.width < fMinOrientedRectSide_ ||
			oriented.size.height < fMinOrientedRectSide_ ||
			oriented.size.width > oriented.size.height * fMaxRectAxesRatio_ ||
			oriented.size.height > oriented.size.width * fMaxRectAxesRatio_)
		{
			continue;
		}

		for (unsigned j = 0; j < szEdge; ++j)
		{
			clean(edge[j]) = (uchar)255;
		}
	}
}

// Jario Edit Start #################################################
void CNEllipseDetector::Triplets1234(VVP& p1, VVP& p2, VVP& p3, VVP& p4,
    std::vector<Ellipse>& ellipses) {
#define POINT_LINK_PER 0.5
#define CALC_POINTS_ABS(pf, ps) (abs((pf.x)-(ps.x))+abs((pf.y)-(ps.y)))
    vector<bool> del_table1(p1.size(), false);
    vector<bool> del_table2(p2.size(), false);
    vector<bool> del_table3(p3.size(), false);
    vector<bool> del_table4(p4.size(), false);
    VVP arcs1, arcs2, arcs3;
    // p1, p3  are SortTopLeft2BottomRight
    for (size_t i = 0; i < p1.size(); i++) {
        // TopLeft
        int p1_end = p1[i].size() - 1, f14(-1), f12(-1), dis14(1e4), dis12(1e4);
        for (size_t j = 0; j < p4.size(); j++) {
            int p4_end = p4[j].size() - 1;
            int point_link_thr = int(MIN(p1[i].size(), p4[j].size()) * POINT_LINK_PER);
            int pdist = CALC_POINTS_ABS(p1[i][0], p4[j][p4_end]);
            if (pdist < point_link_thr && pdist < dis14) {
                f14 = j; dis14 = pdist;
            }
        }
        // BottomRight
        for (size_t j = 0; j < p2.size(); j++) {
            int p2_end = p2[j].size() - 1;
            int pdist = CALC_POINTS_ABS(p1[i][p1_end], p2[j][p2_end]);
            int point_link_thr = int(MIN(p1[i].size(), p2[j].size()) * POINT_LINK_PER);
            if (pdist < point_link_thr && pdist < dis12) {
                f12 = j; dis12 = pdist;
            }
        }
        if (f14 >= 0 && f12 >= 0) {
            arcs3.push_back(p4[f14]);
            arcs2.push_back(p1[i]);
            arcs1.push_back(p2[f12]);
        } else if (f14 < 0 && f12 < 0) {
            del_table1[i] = true;
        }
    }
    // p2
    for (size_t i = 0; i < p2.size(); i++) {
        int p2_end = p2[i].size() - 1, f21(-1), f23(-1), dis21(1e4), dis23(1e4);
        // TopRight
        for (size_t j = 0; j < p1.size(); j++) {
            if (del_table1[j]) continue;
            int p1_end = p1[j].size() - 1;
            int pdist = CALC_POINTS_ABS(p2[i][p2_end], p1[j][p1_end]);
            int point_link_thr = int(MIN(p2[i].size(), p1[j].size()) * POINT_LINK_PER);
            if (pdist < point_link_thr && pdist < dis21) {
                f21 = j; dis21 = pdist;
            }
        }
        // BottomLeft
        for (size_t j = 0; j < p3.size(); j++) {
            int p3_end = p3[j].size() - 1;
            int pdist = CALC_POINTS_ABS(p2[i][0], p3[j][p3_end]);
            int point_link_thr = int(MIN(p2[i].size(), p3[j].size()) * POINT_LINK_PER);
            if (pdist < point_link_thr && pdist < dis23) {
                f23 = j; dis23 = pdist;
            }
        }
        if (f21 >= 0 && f23 >= 0) {
            arcs1.push_back(p1[f21]);
            arcs2.push_back(p2[i]);
            arcs3.push_back(p3[f23]);
        }
        else if (f21 < 0 && f23 < 0) {
            del_table2[i] = true;
        }
        /********
        else {
            Mat3b out(720, 1280, Vec3b(0, 0, 0));
            Vec3b color(0, 255, 0);
            for (unsigned j = 0; j<p2[i].size(); ++j)
                out(p2[i][j]) = color;
            if (f21 >= 0) {
                for (unsigned j = 0; j < p1[f21].size(); ++j)
                    out(p1[f21][j]) = color;
            }
            if (f23 >= 0) {
                for (unsigned j = 0; j < p3[f23].size(); ++j)
                    out(p3[f23][j]) = color;
            }
            imshow("test", out); waitKey(10);
            std::cout << f21 << " " << f23 << std::endl;
        }
        ********/
    }
    // p3
    for (size_t i = 0; i < p3.size(); i++) {
        int p3_end = p3[i].size() - 1, f32(-1), f34(-1), dis32(1e4), dis34(1e4);
        // TopLeft
        for (size_t j = 0; j < p4.size(); j++) {
            int pdist = CALC_POINTS_ABS(p3[i][0], p4[j][0]);
            int point_link_thr = int(MIN(p3[i].size(), p4[j].size()) * POINT_LINK_PER);
            if (pdist < point_link_thr && pdist < dis34) {
                f34 = j; dis34 = pdist;
            }
        }
        // BottomRight
        for (size_t j = 0; j < p2.size(); j++) {
            if (del_table2[j]) continue;
            int pdist = CALC_POINTS_ABS(p3[i][p3_end], p2[j][0]);
            int point_link_thr = int(MIN(p3[i].size(), p2[j].size()) * POINT_LINK_PER);
            if (pdist < point_link_thr && pdist < dis32) {
                f32 = j; dis32 = pdist;
            }
        }
        if (f32 >= 0 && f34 >= 0) {
            arcs1.push_back(p4[f34]);
            arcs2.push_back(p3[i]);
            arcs3.push_back(p2[f32]);
        }
        else if (f32 < 0 && f34 < 0) {
            del_table3[i] = true;
        }
        /********
        else {
            Mat3b out(480, 640, Vec3b(0, 0, 0));
            Vec3b color(0, 255, 0);
            for (unsigned j = 0; j<p3[i].size(); ++j)
                out(p3[i][j]) = color;
            if (f34 >= 0) {
                for (unsigned j = 0; j < p4[f34].size(); ++j)
                    out(p4[f34][j]) = color;
            }
            if (f32 >= 0) {
                for (unsigned j = 0; j < p2[f32].size(); ++j)
                    out(p2[f32][j]) = color;
            }
            imshow("test", out); waitKey(10);
            std::cout << f34 << " " << f32 << std::endl;
        }
        ********/
    }
    // p4
    for (size_t i = 0; i < p4.size(); i++) {
        int p4_end = p4[i].size() - 1, f41(-1), f43(-1), dis41(1e4), dis43(1e4);
        // BottomLeft
        for (size_t j = 0; j < p1.size(); j++) {
            if (del_table1[j]) continue;
            int pdist = CALC_POINTS_ABS(p4[i][p4_end], p1[j][0]);
            int point_link_thr = int(MIN(p4[i].size(), p1[j].size()) * POINT_LINK_PER);
            if (pdist < point_link_thr && pdist < dis41) {
                f41 = j; dis41 = pdist;
            }
        }
        // TopRight
        for (size_t j = 0; j < p3.size(); j++) {
            if (del_table3[j]) continue;
            int pdist = CALC_POINTS_ABS(p4[i][0], p3[j][0]);
            int point_link_thr = int(MIN(p4[i].size(), p3[j].size()) * POINT_LINK_PER);
            if (pdist < point_link_thr && pdist < dis43) {
                f43 = j; dis43 = pdist;
            }
        }
        if (f41 >= 0 && f43 >= 0) {
            arcs1.push_back(p3[f43]);
            arcs2.push_back(p4[i]);
            arcs3.push_back(p1[f41]);
        }
        else if (f41 < 0 && f43 < 0) {
            del_table4[i] = true;
        }
    }

    Point pt1, pt2, pt3, pt4, pt5, pt6;
    for (int i = 0; i < arcs1.size(); i++) {
        // Mat3b out(720, 1280, Vec3b(0, 0, 0));
        // Vec3b color(0, 255, 0);
        int as1 = (int)arcs1[i].size();
        if (as1 > 4) {
            pt1 = arcs1[i][as1 / 5]; pt2 = arcs1[i][as1 - 1 - as1 / 5];
        } else {
            pt1 = arcs1[i][0]; pt2 = arcs1[i][as1 - 1];
        }
        int as2 = (int)arcs2[i].size();
        if (as2 > 4) {
            pt3 = arcs2[i][as2 / 5]; pt4 = arcs2[i][as2 - 1 - as2 / 5];
        }
        else {
            pt3 = arcs2[i][0]; pt4 = arcs2[i][as2 - 1];
        }
        int as3 = (int)arcs3[i].size();
        if (as3 > 4) {
            pt5 = arcs3[i][as3 / 5]; pt6 = arcs3[i][as3 - 1 - as3 / 5];
        }
        else {
            pt5 = arcs3[i][0]; pt6 = arcs3[i][as3 - 1];
        }
        float cnc = value4SixPoints(pt1, pt2, pt3, pt4, pt5, pt6);
        /********
        circle(out, pt1, 2, color);
        circle(out, pt2, 2, color);
        circle(out, pt3, 2, color);
        circle(out, pt4, 2, color);
        circle(out, pt5, 2, color);
        circle(out, pt6, 2, color);
        imshow("test", out); waitKey(100);
        ********/
        if (abs(cnc - 1.0f) > .35f) {
            continue;
        }

        EllipseData data_12, data_13;

        GetFastCenter(arcs1[i], arcs2[i], data_12);
        GetFastCenter(arcs1[i], arcs3[i], data_13);

        // INVALID CENTERS
        if (!data_12.isValid || !data_13.isValid) {
            continue;
        }
        Point2f center = GetCenterCoordinates(data_12, data_13);
        FindEllipses(center, arcs1[i], arcs2[i], arcs3[i], data_12, data_13, ellipses);
    }
    /*
    Mat3b out(480, 640, Vec3b(0, 0, 0));
    Vec3b color(255, 0, 0);
    for (unsigned j = 0; j<arcs1[3].size(); ++j)
        out(arcs1[3][j]) = color;
    for (unsigned j = 0; j<arcs2[3].size(); ++j)
        out(arcs2[3][j]) = color;
    for (unsigned j = 0; j<arcs3[3].size(); ++j)
        out(arcs3[3][j]) = color;
    imshow("test", out); waitKey(10);
    */
    // p2, p4  are SortBottomLeft2TopRight
}
// Jario Edit End   #################################################

void CNEllipseDetector::PrePeocessing(Mat1b& I,
	Mat1b& DP,
	Mat1b& DN
	)
{
#ifdef DEBUG_SPEED
	Tic(1); // edge detection
#endif
	// Mid smooth
	//medianBlur(I,I,3);
	// Smooth image
	GaussianBlur(I, I, szPreProcessingGaussKernel_, dPreProcessingGaussSigma_);

	// Temp variables
	Mat1b E;				//edge mask
	Mat1s DX, DY;			//sobel derivatives

	// Detect edges
	Canny3(I, E, DX, DY, 3, false);

#ifdef DEBUG_PREPROCESSING
	imshow("PreProcessing->Edge", E); waitKey();
#endif
#ifdef DETECT_MARKER_X_PLUS
	E.copyTo(EdgeMap_);
#endif
#ifdef DEBUG_SPEED
	Toc(1, "edge detection"); // edge detection
	Tic(2); // preprocessing
#endif
	// For each edge points, compute the edge direction
	for (int i = 0; i<szIm_.height; ++i)
	{
		short* _dx = DX.ptr<short>(i);
		short* _dy = DY.ptr<short>(i);
		uchar* _e = E.ptr<uchar>(i);
		uchar* _dp = DP.ptr<uchar>(i);
		uchar* _dn = DN.ptr<uchar>(i);

		for (int j = 0; j<szIm_.width; ++j)
		{
			if (!((_e[j] <= 0) || (_dx[j] == 0) || (_dy[j] == 0)))
			{
				// Angle of the tangent
				float phi = -(float(_dx[j]) / float(_dy[j]));

				// Along positive or negative diagonal
				if (phi > 0)  _dp[j] = (uchar)255;
				else if (phi < 0)  _dn[j] = (uchar)255;
			}
		}
	}
#ifdef DEBUG_SPEED
	Toc(2, "preprocessing"); // preprocessing
#endif
}


void CNEllipseDetector::DetectAfterPreProcessing(vector<Ellipse>& ellipses, Mat1b& E, Mat1f& PHI)
{
	// Set the image size
	szIm_ = E.size();

	// Initialize temporary data structures
	Mat1b DP = Mat1b::zeros(szIm_);		// arcs along positive diagonal
	Mat1b DN = Mat1b::zeros(szIm_);		// arcs along negative diagonal

	// For each edge points, compute the edge direction
	for (int i = 0; i<szIm_.height; ++i)
	{
		float* _phi = PHI.ptr<float>(i);
		uchar* _e = E.ptr<uchar>(i);
		uchar* _dp = DP.ptr<uchar>(i);
		uchar* _dn = DN.ptr<uchar>(i);

		for (int j = 0; j<szIm_.width; ++j)
		{
			if ((_e[j] > 0) && (_phi[j] != 0))
			{
				// Angle

				// along positive or negative diagonal
				if (_phi[j] > 0)	_dp[j] = (uchar)255;
				else if (_phi[j] < 0)	_dn[j] = (uchar)255;
			}
		}
	}

	// Initialize accumulator dimensions
	ACC_N_SIZE = 101;
	ACC_R_SIZE = 180;
	ACC_A_SIZE = max(szIm_.height, szIm_.width);

	// Allocate accumulators
	accN = new int[ACC_N_SIZE];
	accR = new int[ACC_R_SIZE];
	accA = new int[ACC_A_SIZE];

	// Other temporary 
	VVP points_1, points_2, points_3, points_4;		//vector of points, one for each convexity class
	unordered_map<uint, EllipseData> centers;		//hash map for reusing already computed EllipseData

	// Detect edges and find convexities
	DetectEdges13(DP, points_1, points_3);
	DetectEdges24(DN, points_2, points_4);

	// Find triplets
	Triplets124(points_1, points_2, points_4, centers, ellipses);
	Triplets231(points_2, points_3, points_1, centers, ellipses);
	Triplets342(points_3, points_4, points_2, centers, ellipses);
	Triplets413(points_4, points_1, points_3, centers, ellipses);

	// Sort detected ellipses with respect to score
	sort(ellipses.begin(), ellipses.end());

	//free accumulator memory
	delete[] accN;
	delete[] accR;
	delete[] accA;

	//cluster detections
	//ClusterEllipses(ellipses);
}


void CNEllipseDetector::Detect(Mat1b& I, vector<Ellipse>& ellipses)
{
	countOfFindEllipse_ = 0;
	countOfGetFastCenter_ = 0;
#ifdef DEBUG_SPEED
	Tic(0); // prepare data structure
#endif

	// Set the image size
	szIm_ = I.size();

	// Initialize temporary data structures
	Mat1b DP = Mat1b::zeros(szIm_);		// arcs along positive diagonal
	Mat1b DN = Mat1b::zeros(szIm_);		// arcs along negative diagonal

	// Initialize accumulator dimensions
	ACC_N_SIZE = 101;
	ACC_R_SIZE = 180;
	ACC_A_SIZE = max(szIm_.height, szIm_.width);

	// Allocate accumulators
	accN = new int[ACC_N_SIZE];
	accR = new int[ACC_R_SIZE];
	accA = new int[ACC_A_SIZE];

	// Other temporary 
	VVP points_1, points_2, points_3, points_4;		// vector of points, one for each convexity class
	unordered_map<uint, EllipseData> centers;		// hash map for reusing already computed EllipseData
#ifdef DEBUG_SPEED
	Toc(0, "prepare data structure"); // prepare data structure
#endif

	// Preprocessing
	// From input image I, find edge point with coarse convexity along positive (DP) or negative (DN) diagonal
	PrePeocessing(I, DP, DN);
#ifdef DEBUG_SPEED
	Tic(3); // preprocessing
#endif
	// Detect edges and find convexities
	DetectEdges13(DP, points_1, points_3);
	DetectEdges24(DN, points_2, points_4);
#ifdef DEBUG_SPEED
	Toc(3, "preprocessing_2"); // preprocessing
#endif
	//// DEBUG
#ifdef DEBUG_PREPROCESSING
	Mat3b out(I.rows, I.cols, Vec3b(0,0,0));
	for(unsigned i=0; i<points_1.size(); ++i)
	{
		//Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
		Vec3b color(255,0,0);
		for(unsigned j=0; j<points_1[i].size(); ++j)
			out(points_1[i][j]) = color;
	}

	for(unsigned i=0; i<points_2.size(); ++i)
	{
		//Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
		Vec3b color(0,255,0);
		for(unsigned j=0; j<points_2[i].size(); ++j)
			out(points_2[i][j]) = color;
	}
	for(unsigned i=0; i<points_3.size(); ++i)
	{
		//Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
		Vec3b color(0,0,255);
		for(unsigned j=0; j<points_3[i].size(); ++j)
			out(points_3[i][j]) = color;
	}

	for(unsigned i=0; i<points_4.size(); ++i)
	{
		//Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
		Vec3b color(255,0,255);
		for(unsigned j=0; j<points_4[i].size(); ++j)
			out(points_4[i][j]) = color;
	}
	imshow("PreProcessing->Output", out); waitKey();
#endif 

	// time estimation, validation  inside
#ifdef DEBUG_SPEED
	Tic(4); // grouping
#endif
    // find triplets
    Triplets1234(points_1, points_4, points_3, points_2, ellipses);
    // Triplets124(points_1, points_2, points_4, centers, ellipses);
    // Triplets231(points_2, points_3, points_1, centers, ellipses);
    // Triplets342(points_3, points_4, points_2, centers, ellipses);
    // Triplets413(points_4, points_1, points_3, centers, ellipses);
#ifdef DEBUG_SPEED
	Toc(4, "grouping"); // grouping	
#endif
	// time estimation, validation inside
	// _times[2] -= (_times[3] + _times[4]); // ??
#ifdef DEBUG_SPEED
	Tic(5); // validation
#endif
	// Sort detected ellipses with respect to score
	sort(ellipses.begin(), ellipses.end());
#ifdef DEBUG_SPEED
	Toc(5, "validation"); // validation
#endif

	// Free accumulator memory
	delete[] accN;
	delete[] accR;
	delete[] accA;
#ifdef DEBUG_SPEED
	Tic(6);
#endif
	// Cluster detections
	ClusterEllipses(ellipses);
#ifdef DEBUG_SPEED
	Toc(6, "cluster detections");
#endif
	
#ifdef NMS_ELLIPSE_LOCATE
#ifdef DEBUG_SPEED
	Tic(7);
#endif
#ifdef USE_SELF_DEFINE_NMS
    size_t num = ellipses.size();
	int radius = 1600;
	vector<int> count(num, 0);
    for (size_t i = 0; i < num; i++) {
        if (count[i] == 1) {
			continue;
		}
        for (size_t j = i + 1; j < num; j++) {
			int dis = (ellipses[i]._xc - ellipses[j]._xc) * (ellipses[i]._xc - ellipses[j]._xc) + (ellipses[i]._yc - ellipses[j]._yc) * (ellipses[i]._yc - ellipses[j]._yc);
            if(dis >= radius) {
				continue;
			}
            else if (j - i > 1) {
				count[j] += 1;
			}
		}
	}

	vector<Ellipse>::iterator iters = ellipses.begin();
	int eCount = 0;
    while (iters != ellipses.end()) {
        if (count[eCount] > 0) {
            // cout << "********************" << endl;
			iters = ellipses.erase(iters);
			++eCount;
			continue;
        } else {
			++eCount;
			iters++;
			continue;
		}
	}
#endif
#ifdef DEBUG_SPEED
	Toc(7, "=====NMS=====");
#endif
#endif

#ifdef DETECT_MARKER_X_NET
    float xscore(0);
    int cnt_sign = -1;
    for (size_t i = 0; i < ellipses.size(); i++) {
        spire::Ellipse e1 = ellipses[i];
        int _rad = int(e1._b * .6f);

        int x1 = e1._xc - _rad, x2 = e1._xc + _rad;
        int y1 = e1._yc - _rad, y2 = e1._yc + _rad;
        if (x1 < 0) x1 = 0;
        if (y1 < 0) y1 = 0;
        if (x2 > I.cols - 1) x2 = I.cols - 1;
        if (y2 > I.rows - 1) y2 = I.rows - 1;
        if (x2 < (x1+5) || y2 < (y1+5)) continue;

        Mat I2 = I.rowRange(Range(y1, y2)).colRange(Range(x1, x2));
        resize(I2, I2, Size(28, 28));

        vector<caffe::PredictionInt> pred_I = cvmat_classifier_.Classify_LI(I2);
        if (pred_I[0].first == 0) { // ohoh, x
            if (pred_I[0].second > xscore) {
                xscore = pred_I[0].second;
                cnt_sign = (int)i;
            }
        }
    }

    if (cnt_sign >= 0 && xscore > 500) {
        spire::Ellipse e2 = ellipses[cnt_sign];
        ellipses.clear();
        ellipses.push_back(e2);
    } else {
        ellipses.clear();
    }
#endif

#ifdef DEBUG_SAVE_ELLISPE
	vector<Ellipse>::iterator iter = ellipses.begin();
	while (iter != ellipses.end()) {
		Ellipse e1 = *iter;
        int rad_x(0), rad_y(0);
        if (e1._rad < 0.7854 || e1._rad > 2.3562) {
            rad_x = int(e1._a * .6f);
            rad_y = int(e1._b * .6f);
        } else {
            rad_x = int(e1._b * .6f);
            rad_y = int(e1._a * .6f);
        }

        int x1 = e1._xc - rad_x + 1, x2 = e1._xc + rad_x + 1;
        int y1 = e1._yc - rad_y + 1, y2 = e1._yc + rad_y + 1;
		if (x1 < 0) x1 = 0;
		if (y1 < 0) y1 = 0;
		if (x2 > I.cols - 1) x2 = I.cols - 1;
		if (y2 > I.rows - 1) y2 = I.rows - 1;

        Mat1b I2 = I(cv::Rect(x1, y1, x2-x1, y2-y1));
		resize(I2, I2, Size(28, 28));

		char fn[256];
        sprintf(fn, "/opt/data/ellipse_train/ellipse_%d.jpg", save_ellipse_cont_);
		save_ellipse_cont_++;
		imwrite(fn, I2);
		// if (small_xe) {
		//	iter = ellipses.erase(iter); continue;
		// }
		iter++;
	}
#endif
#ifdef DETECT_MARKER_X_EDGE
	vector<Ellipse>::iterator iter = ellipses.begin();
	while (iter != ellipses.end()) {
		Ellipse e1 = *iter;
	
		int _rad = int(e1._b * .6f);

		double err(1e6);
		int x1 = e1._xc - _rad, x2 = e1._xc + _rad;
		int y1 = e1._yc - _rad, y2 = e1._yc + _rad;
		if (x1 < 0) x1 = 0;
		if (y1 < 0) y1 = 0;
		if (x2 > I.cols - 1) x2 = I.cols - 1;
		if (y2 > I.rows - 1) y2 = I.rows - 1;
		Mat1b I2 = I.rowRange(Range(y1, y2)).colRange(Range(x1, x2));
		resize(I2, I2, Size(40, 40));

		Mat1b E;				//edge mask
		Mat1s DX, DY;			//sobel derivatives
		Canny4(I2, E, DX, DY, 3, false);

		// XDetector xd; Mat EE;
		// xd.Detect(I2, EE);
		// imshow("EE", EE);
		// waitKey();
		vector<Vec4i> lines;
		HoughLinesP(E, lines, 1, CV_PI/180, 20, 20, 10);
		Mat dstImage;
		cvtColor(I2, dstImage, CV_GRAY2BGR);
		for (size_t i=0;i<lines.size();i++) {
			Vec4i l = lines[i];
			line(dstImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, LINE_AA);
		}
		imshow("LL", dstImage);
		waitKey(80);
#ifdef DEBUG_X_EDGE
		imshow("E", E);
		waitKey(10);
#endif

		VVP contours;
		int countedges = 0;
		// Labeling 8-connected edge points, discarding edge too small
		find_contours(E, contours, iMinEdgeLength_); // put small arc edges to a vector
// #define DEBUG_X_EDGE
#ifdef DEBUG_X_EDGE
		Mat1b DP_show = E.clone();
		show_contours(DP_show, contours, "PreProcessing->Contours13"); waitKey(10);
#endif
		if (contours.size() != 4) {
			iter = ellipses.erase(iter); continue;
		}
		bool small_xe = false;
		for (int i = 0; i < 4; i++) {
			if (contours[i].size() < 20) {
				small_xe = true; break;
			}
		}
		if (small_xe) {
			iter = ellipses.erase(iter); continue;
		}
		iter++;
	}
#endif

#ifdef DETECT_MARKER_X_Z
	vector<Ellipse>::iterator iter = ellipses.begin();
	while (iter != ellipses.end()) {
		Ellipse e1 = *iter;

		int rad; // = int(e1._b * .6f);
		int _rad = int(e1._b * .6f);
		int sr = 2; // int(e1._b * .1f);

		double err(1e6);
		int x1 = e1._xc - _rad, x2 = e1._xc + _rad;
		int y1 = e1._yc - _rad, y2 = e1._yc + _rad;
		if (x1 < 0) x1 = 0;
		if (y1 < 0) y1 = 0;
		if (x2 > I.cols - 1) x2 = I.cols - 1;
		if (y2 > I.rows - 1) y2 = I.rows - 1;
		Mat1b I2 = I.rowRange(Range(y1, y2)).colRange(Range(x1, x2));
		resize(I2, I2, Size(20, 20));
		cv::Mat kerl_x = (Mat_<float>(1, 3) << -1, 0, 1);
		Mat1f grad_x, grad_y;
		cv::filter2D(I2, grad_x, CV_32FC1, kerl_x, Point(-1, -1), 0, BORDER_DEFAULT);

		cv::Mat kerl_y = (Mat_<float>(3, 1) << -1, 0, 1);
		cv::filter2D(I2, grad_y, CV_32FC1, kerl_y, Point(-1, -1), 0, BORDER_DEFAULT);
		
		Mat mag, angle;
		cartToPolar(grad_x, grad_y, mag, angle, true);

		// normalize(mag, mag, 1.0, 0.0, NORM_MINMAX);
		// imshow("SS", mag);
		// cv::waitKey();
		rad = 8;
		int _yc = 10, _xc = 10;
		// int i_b(0), i_g(0), i_r(0);
// #define X_DEBUG
#ifdef X_DEBUG
		imshow("I_ori", I2);
		cv::waitKey(100);
#endif
		int saved_cy, saved_cx;
		for (int i = -sr; i <= sr; i++) for (int j = -sr; j <= sr; j++) {
			int cy = _yc + i, cx = _xc + j;
			
			// Mat mag_show;
			// normalize(mag, mag_show, 1.0, 0.0, NORM_MINMAX);
			// imshow("mag1", mag_show);
			// cv::waitKey(100);
			double err1 = CalcXHistError(mag, angle, cy, cx, rad);
			// normalize(mag, mag_show, 1.0, 0.0, NORM_MINMAX);
			// imshow("mag2", mag_show);
			// cv::waitKey(100);
			if (err1 < err) {
				err = err1; saved_cy = cy; saved_cx = cx;
			}
			// cout << err1;
		}
		/**********/
#ifdef X_DEBUG
		x1 = saved_cx - rad, x2 = saved_cx + rad;
		y1 = saved_cy - rad, y2 = saved_cy + rad;
		Mat1b II = I2.rowRange(Range(y1, y2)).colRange(Range(x1, x2));
		imshow("I_x", II);
		cv::waitKey(100);
#endif
		/**********/
		// err = err / e1._b;
 		if (err > 220.0) {
			iter = ellipses.erase(iter); continue;
		}
		iter++;
	}
#endif

#ifdef DETECT_MARKER_X_PLUS
#define BLOCK_SIZE 2
#define APERTURE_SIZE 3
#define K 0.04
	// static double _hu_x[7] = { 2.5889403129476252, 0.0013314422016213617, 0.48016793469244268,
	// 	0.035133369719300457, 0.0038287279957098983, -0.00045356486606164091, -0.0024827981335497759 };
	vector<Ellipse>::iterator iter = ellipses.begin();
	while (iter != ellipses.end()) {
		Ellipse e1 = *iter;
		int rad = int(e1._b * .6f);
		int x1 = e1._xc - rad, x2 = e1._xc + rad;
		int y1 = e1._yc - rad, y2 = e1._yc + rad;
		if (x1 < 0) x1 = 0;
		if (y1 < 0) y1 = 0;
		if (x2 > I.cols-1) x2 = I.cols-1;
		if (y2 > I.rows-1) y2 = I.rows-1;
		// Mat EI = EdgeMap_.rowRange(Range(y1, y2)).colRange(Range(x1, x2));
		Mat II = I.rowRange(Range(y1, y2)).colRange(Range(x1, x2));
		// imshow("SS", II);
		// waitKey(100);
		// double dbR(0);
		double dbR = matchShapes(II, XMap_, CV_CONTOURS_MATCH_I2, 0.0);
	
		// cv::FileStorage _fs("x_moments.xml", FileStorage::WRITE);
		// _fs <<"XE"<< II;
		// _fs.release();
		/*******
		double _hu[7];
		Moments _moments = moments(EI, true);
		HuMoments(_moments, _hu);
		
		double dbR(0), dSigmaST(0), dSigmaS(0), dSigmaT(0);
		for (int i = 0; i < 7; i++) {
			dSigmaST += fabs(_hu[i]*_hu_x[i]);
			dSigmaS += pow(_hu[i], 2);
			dSigmaT += pow(_hu_x[i], 2);
		}
		dbR = dSigmaST / (sqrt(dSigmaS)*sqrt(dSigmaT));
		
		double dbR2(0), tmp1(0), tmp2(0);
		for (int i = 0; i < 7; i++) {
			tmp1 += fabs(_hu[i] - _hu_x[i]);
			tmp2 += fabs(_hu[i] + _hu_x[i]);
		}
		dbR2 = 1.0 - (tmp1 / tmp2);
		*******/
		// I.locateROI(Size(rad, rad), Point(e1._xc, e1._yc));
		imshow("SS", II);
		waitKey(100);
		if (dbR < 0.1) {
			iter = ellipses.erase(iter); continue;
		}
		iter++;
	}
#endif
	// Jario Edit START
#ifdef DETECT_MARKER_X
	vector<Ellipse>::iterator iter = ellipses.begin();
	while (iter != ellipses.end()) {
		Ellipse e1 = *iter;
		VVP SL13, SL24;
		int N13(0), N24(0), C13(0), C24(0);
		int MeanX13(0), MeanX24(0);
		for (size_t k = 0; k < straightLines13_.size(); k++) {
			VP l1 = straightLines13_[k];
			int x = l1[l1.size() / 2].x, y = l1[l1.size() / 2].y;
			if (sqrt((x - e1._xc)*(x - e1._xc) + (y - e1._yc)*(y - e1._yc)) < e1._b) {
				SL13.push_back(l1); N13 += l1.size();
				MeanX13 += (l1[l1.size() - 1].x - l1[0].x); 
				C13++;
			}
		}
		if (N13 == 0) {
			iter = ellipses.erase(iter); continue;
		}
		MeanX13 = MeanX13 / C13;
		for (size_t k = 0; k < straightLines24_.size(); k++) {
			VP l1 = straightLines24_[k];
			int x = l1[l1.size() / 2].x, y = l1[l1.size() / 2].y;
			if (sqrt((x - e1._xc)*(x - e1._xc) + (y - e1._yc)*(y - e1._yc)) < e1._b) {
				SL24.push_back(l1); N24 += l1.size();
				MeanX24 += (l1[l1.size() - 1].x - l1[0].x);
				C24++;
			}
		}
		if (N24 == 0) {
			iter = ellipses.erase(iter); continue;
		}
		MeanX24 = MeanX24 / C24;
		Mat SL13A(N13, 2, CV_32FC1); Mat SL13b(N13, 1, CV_32FC1);
		Mat SL24A(N24, 2, CV_32FC1); Mat SL24b(N24, 1, CV_32FC1);
		int cnt(0);
		for (size_t i = 0; i < SL13.size(); i++) for (size_t j = 0; j < SL13[i].size(); j++) {
			SL13A.at<float>(cnt, 0) = SL13[i][j].x;
			SL13A.at<float>(cnt, 1) = 1.0f; 
			SL13b.at<float>(cnt) = SL13[i][j].y; 
			cnt++;
		}
		cnt = 0;
		for (size_t i = 0; i < SL24.size(); i++) for (size_t j = 0; j < SL24[i].size(); j++) {
			SL24A.at<float>(cnt, 0) = SL24[i][j].x;
			SL24A.at<float>(cnt, 1) = 1.0f; 
			SL24b.at<float>(cnt) = SL24[i][j].y;
			cnt++;
		}
		Mat R13 = (SL13A.t()*SL13A).inv()*SL13A.t()*SL13b;
		Mat R24 = (SL24A.t()*SL24A).inv()*SL24A.t()*SL24b;
		float rx = (R24.at<float>(1) - R13.at<float>(1)) / (R13.at<float>(0) - R24.at<float>(0));
		float ry = R13.at<float>(0)*rx + R13.at<float>(1);
		float c_err = sqrt((rx - e1._xc)*(rx - e1._xc) + (ry - e1._yc)*(ry - e1._yc));
		if (c_err > e1._a*.05f) {
			iter = ellipses.erase(iter); continue;
		}
		
		Point X13_1, X13_2;
		X13_1.x = cvRound(rx - MeanX13/2); X13_1.y = cvRound(R13.at<float>(0)*X13_1.x + R13.at<float>(1));
		X13_2.x = cvRound(rx + MeanX13/2); X13_2.y = cvRound(R13.at<float>(0)*X13_2.x + R13.at<float>(1));
		Point X24_1, X24_2;
		X24_1.x = cvRound(rx - MeanX24/2); X24_1.y = cvRound(R24.at<float>(0)*X24_1.x + R24.at<float>(1));
		X24_2.x = cvRound(rx + MeanX24/2); X24_2.y = cvRound(R24.at<float>(0)*X24_2.x + R24.at<float>(1));
		// line(I, X13_1, X13_2, Scalar(255, 255, 255));
		// line(I, X24_1, X24_2, Scalar(255, 255, 255));
		// imshow("I", I);
		X13_P1_.push_back(X13_1); X13_P2_.push_back(X13_2);
		X24_P1_.push_back(X24_1); X24_P2_.push_back(X24_2);

		iter++;
		
		// show_contours(I, SL24, "inner_lines");
		// waitKey();
	}
#endif
	// Jario Edit END
};

#ifdef DETECT_MARKER_X
// Jario Edit START
#define POW2(x) (x)*(x)
void CNEllipseDetector::DetectXMarker(cv::Mat3b& colorIm, std::vector<Ellipse>& ellipses) {
	Size sz = colorIm.size();

	// Convert to grayscale
	Mat1b gray;
	cvtColor(colorIm, gray, CV_BGR2GRAY);
	Detect(gray, ellipses);
	
	vector<Ellipse>::iterator iter = ellipses.begin();
	while (iter != ellipses.end()) {
		Ellipse e1 = *iter;

		int sr = round(e1._b*0.1);
		int i_b(0), i_g(0), i_r(0);
		for (int i = -sr; i <= sr; i++) for (int j = -sr; j <= sr; j++) {
			int cy = e1._yc + i, cx = e1._xc + j;
			Vec3b i_bgr = colorIm.at<Vec3b>(cy, cx);
			i_b += i_bgr[0]; i_g += i_bgr[1]; i_r += i_bgr[2];
		}
		int i_sum = POW2(sr * 2 + 1);
		i_b = i_b / i_sum;
		i_g = i_g / i_sum;
		i_r = i_r / i_sum;
		float i_err = POW2(i_b - 255.0f) + POW2(i_g - 255.0f) + POW2(i_r - 255.0f);
		
		if (i_err > 70000) {
			iter = ellipses.erase(iter); continue;
		}
		// circle(colorIm, Point(e1._xc, e1._yc), 2, Scalar(0, 0, 255), 1);
		// imshow("Im", colorIm);
		// waitKey();
		iter++;
	}
}
// Jario Edit END
#endif

#ifdef DETECT_MARKER_X_Z
void MinMaxInHW(int& y1, int& y2, int& x1, int& x2, int H, int W) {
	y1 = MAX(y1, 0); x1 = MAX(x1, 0);
	y2 = MIN(y2, H - 1); x2 = MIN(x2, W - 1);
}
double CNEllipseDetector::CalcXHistError(const cv::Mat& mag_in, const cv::Mat& I, int cy, int cx, int rad)
{
#define HIST_SIZE 8
#define HIST_MAX 360.0f
#define ANGLE_BLAS 180.0f
#define MAG_THRS 40.0f

	Mat mag = mag_in.clone();
	int y1, y2, x1, x2;
	// MinMaxInHW(y1, y2, x1, x2, I.rows, I.cols);
	vector<float> hist1, hist2, hist3, hist4;
	hist1.resize(HIST_SIZE);
	hist2.resize(HIST_SIZE);
	hist3.resize(HIST_SIZE);
	hist4.resize(HIST_SIZE);
	for (int i = 0; i < HIST_SIZE; i++) {
		hist1[i] = 0; hist2[i] = 0; hist3[i] = 0; hist4[i] = 0;
	}
	float hist_c1(0), hist_c2(0), hist_c3(0), hist_c4(0);
	// float hist1[10] = { 0 };
	// float hist2[10] = { 0 };
	// float hist3[10] = { 0 };
	// float hist4[10] = { 0 };
	// top right
	int cnt_ang1(0);
	y1 = cy - rad; y2 = cy - 1; x1 = cx; x2 = cx + rad - 1;
	// MinMaxInHW(y1, y2, x1, x2, I.rows, I.cols);
	for (int i = y1; i <= y2; i++) for (int j = x1; j <= x2; j++) {
		float mag1 = mag.at<float>(i, j);
		if (mag1 > MAG_THRS) {
			float gray = I.at<float>(i, j);
			// hist1[int(gray / HIST_MAX * HIST_SIZE)]++;
			hist_c1 += (gray - ANGLE_BLAS);
			cnt_ang1++;
		}
	}
// #define X_DEBUG_MAG
#ifdef X_DEBUG_MAG
	Mat mag_show = mag.rowRange(Range(y1, y2+1)).colRange(Range(x1, x2+1));
	normalize(mag_show, mag_show, 1.0, 0.0, NORM_MINMAX);
	imshow("SS1", mag_show);
	waitKey(100);
#endif
	// bottom right
	int cnt_ang2(0);
	y1 = cy; y2 = cy + rad - 1; x1 = cx; x2 = cx + rad - 1;
	// MinMaxInHW(y1, y2, x1, x2, I.rows, I.cols);
	for (int i = y1; i <= y2; i++) for (int j = x1; j <= x2; j++) {
		float mag1 = mag.at<float>(i, j);
		if (mag1 > MAG_THRS) {
			float gray = I.at<float>(i, j);
			// hist2[int(gray / HIST_MAX * HIST_SIZE)]++;
			hist_c2 += (gray - ANGLE_BLAS);
			cnt_ang2++;
		}
	}
#ifdef X_DEBUG_MAG
	mag_show = mag.rowRange(Range(y1, y2 + 1)).colRange(Range(x1, x2 + 1));
	normalize(mag_show, mag_show, 1.0, 0.0, NORM_MINMAX);
	imshow("SS2", mag_show);
	waitKey(100);
#endif
	// bottom left
	int cnt_ang3(0);
	y1 = cy; y2 = cy + rad - 1; x1 = cx - rad; x2 = cx - 1;
	// MinMaxInHW(y1, y2, x1, x2, I.rows, I.cols);
	for (int i = y1; i <= y2; i++) for (int j = x1; j <= x2; j++) {
		float mag1 = mag.at<float>(i, j);
		if (mag1 > MAG_THRS) {
			float gray = I.at<float>(i, j);
			// hist3[int(gray / HIST_MAX * HIST_SIZE)]++;
			hist_c3 += (gray - ANGLE_BLAS);
			cnt_ang3++;
		}
	}
#ifdef X_DEBUG_MAG
	mag_show = mag.rowRange(Range(y1, y2 + 1)).colRange(Range(x1, x2 + 1));
	normalize(mag_show, mag_show, 1.0, 0.0, NORM_MINMAX);
	imshow("SS3", mag_show);
	waitKey(100);
#endif
	// top left
	int cnt_ang4(0);
	y1 = cy - rad; y2 = cy - 1; x1 = cx - rad; x2 = cx - 1;
	for (int i = y1; i <= y2; i++) for (int j = x1; j <= x2; j++) {
		float mag1 = mag.at<float>(i, j);
		if (mag1 > MAG_THRS) {
			float gray = I.at<float>(i, j);
			// hist4[int(gray / HIST_MAX * HIST_SIZE)]++;
			hist_c4 += (gray - ANGLE_BLAS);
			cnt_ang4++;
		}
	}
#ifdef X_DEBUG_MAG
	mag_show = mag.rowRange(Range(y1, y2 + 1)).colRange(Range(x1, x2 + 1));
	normalize(mag_show, mag_show, 1.0, 0.0, NORM_MINMAX);
	imshow("SS4", mag_show);
	waitKey(100);
#endif
	// if (cnt_ang1 + cnt_ang3 < 20) return 100.0;
	// if (cnt_ang2 + cnt_ang4 < 20) return 100.0;
	double h13 = abs(hist_c1 + hist_c3) / MIN(cnt_ang1, cnt_ang3); // compareHist(Mat(hist1), Mat(hist3), CV_COMP_CHISQR);
	double h24 = abs(hist_c2 + hist_c4) / MIN(cnt_ang2, cnt_ang4); // compareHist(Mat(hist2), Mat(hist4), CV_COMP_CHISQR);
	double h13p24_ = h13 + h24;
	return h13p24_;
}
#endif

// Ellipse clustering procedure. See Sect [3.3.2] in the paper.
void CNEllipseDetector::ClusterEllipses(vector<Ellipse>& ellipses)
{
	float th_Da = 0.1f;
	float th_Db = 0.1f;
	float th_Dr = 0.1f;

	float th_Dc_ratio = 0.1f;
	float th_Dr_circle = 0.9f;

	int iNumOfEllipses = int(ellipses.size());
	if (iNumOfEllipses == 0) return;

	// The first ellipse is assigned to a cluster
	vector<Ellipse> clusters;
	clusters.push_back(ellipses[0]);

    // bool bFoundCluster = false;

	for (int i = 1; i<iNumOfEllipses; ++i)
	{
		Ellipse& e1 = ellipses[i];

		int sz_clusters = int(clusters.size());

		float ba_e1 = e1._b / e1._a;
        // float Decc1 = e1._b / e1._a;

		bool bFoundCluster = false;
		for (int j = 0; j<sz_clusters; ++j)
		{
			Ellipse& e2 = clusters[j];

			float ba_e2 = e2._b / e2._a;
			float th_Dc = min(e1._b, e2._b) * th_Dc_ratio;
			th_Dc *= th_Dc;

			// Centers
			float Dc = ((e1._xc - e2._xc)*(e1._xc - e2._xc) + (e1._yc - e2._yc)*(e1._yc - e2._yc));
			if (Dc > th_Dc)
			{
				//not same cluster
				continue;
			}

			// a
			float Da = abs(e1._a - e2._a) / max(e1._a, e2._a);
			if (Da > th_Da)
			{
				//not same cluster
				continue;
			}

			// b
			float Db = abs(e1._b - e2._b) / min(e1._b, e2._b);
			if (Db > th_Db)
			{
				//not same cluster
				continue;
			}

			// angle
			float Dr = GetMinAnglePI(e1._rad, e2._rad) / float(CV_PI);
			if ((Dr > th_Dr) && (ba_e1 < th_Dr_circle) && (ba_e2 < th_Dr_circle))
			{
				//not same cluster
				continue;
			}

			// Same cluster as e2
			bFoundCluster = true;//
			// Discard, no need to create a new cluster
			break;
		}

		if (!bFoundCluster)
		{
			// Create a new cluster			
			clusters.push_back(e1);
		}
	}

	clusters.swap(ellipses);
};



//Draw at most iTopN detected ellipses.
void CNEllipseDetector::DrawDetectedEllipses(Mat3b& output, vector<Ellipse>& ellipses, int iTopN, int thickness)
{
	int sz_ell = int(ellipses.size());
	int n = (iTopN == 0) ? sz_ell : min(iTopN, sz_ell);
	for (int i = 0; i < n; ++i)
	{
		Ellipse& e = ellipses[n - i - 1];
		int g = cvRound(e._score * 255.f);
		Scalar color(0, g, 0);
		ellipse(output, Point(cvRound(e._xc), cvRound(e._yc)), Size(cvRound(e._a), cvRound(e._b)), e._rad*180.0 / CV_PI, 0.0, 360.0, color, thickness);
        cv::circle(output, Point(e._xc, e._yc), 2, Scalar(0,0,255), 2);
#ifdef DETECT_X_MARKER
		line(output, X13_P1_[i], X13_P2_[i], Scalar(0, 0, 255));
		line(output, X24_P1_[i], X24_P2_[i], Scalar(0, 0, 255));
#endif
	}
}

}

#endif // USE_OPENCV
