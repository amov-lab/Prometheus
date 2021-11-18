#ifndef SPIRE_CN_DETECTOR_HPP_
#define SPIRE_CN_DETECTOR_HPP_
#ifdef USE_OPENCV

#include <opencv2/opencv.hpp>
#include <caffe/util/spire_ellipse.h>
#include <unordered_map>

#define SAVE_ELLIPSE_START 0
// #define DEBUG_PREPROCESSING
// #define DEBUG_SPEED
// #define DETECT_MARKER_X
// #define DETECT_MARKER_X_PLUS
// #define DETECT_MARKER_X_Z
// #define DETECT_MARKER_X_EDGE
#define DETECT_MARKER_X_NET
// #define DEBUG_SAVE_ELLISPE
// #define NMS_ELLIPSE_LOCATE

#ifdef DETECT_MARKER_X_NET
#include <caffe/util/spire_classifier_cvmat.hpp>
#endif

namespace spire {

// #define DISCARD_TCN
#define DISCARD_CONSTRAINT_OBOX

// #define DISCARD_CONSTRAINT_CONVEXITY
// #define DISCARD_CONSTRAINT_POSITION
#define CONSTRAINT_CNC_1
// #define CONSTRAINT_CNC_2
// #define CONSTRAINT_CNC_3
// #define DISCARD_CONSTRAINT_CENTER

#define T_CNC 0.2f
#define T_TCN_L 0.6f // filter lines


// Data available after selection strategy. 
// They are kept in an associative array to:
// 1) avoid recomputing data when starting from same arcs
// 2) be reused in firther proprecessing
// See Sect [] in the paper
struct EllipseData
{
	bool isValid;
	float ta;          // arc_a center line gradient
	float tb;          // arc_b
	float ra;          // gradient of a (slope of start of chord_1 and center of chord_2)
	float rb;          // gradient of b (slope of center of chord_1 and last of chord_2)
	cv::Point2f Ma;        // arc_a center of element
	cv::Point2f Mb;        // arc_b
	cv::Point2f Cab;       // center of ellipse
	std::vector<float> Sa;  // arc_a's center line of parallel chords
	std::vector<float> Sb;  //
};


class CNEllipseDetector
{
	// Parameters

	// Preprocessing - Gaussian filter. See Sect [] in the paper
	cv::Size	szPreProcessingGaussKernel_;	    // size of the Gaussian filter in preprocessing step
	double	dPreProcessingGaussSigma_;			// sigma of the Gaussian filter in the preprocessing step


	// Selection strategy - Step 1 - Discard noisy or straight arcs. See Sect [] in the paper
	int		iMinEdgeLength_;					// minimum edge size				
	float	fMinOrientedRectSide_;				// minumum size of the oriented bounding box containing the arc
	float	fMaxRectAxesRatio_;					// maximum aspect ratio of the oriented bounding box containing the arc

	// Selection strategy - Step 2 - Remove according to mutual convexities. See Sect [] in the paper
	float   fThrArcPosition_;

	// Selection Strategy - Step 3 - Number of points considered for slope estimation when estimating the center. See Sect [] in the paper
	unsigned uNs_;                              // Find at most Ns parallel chords.

	// Selection strategy - Step 3 - Discard pairs of arcs if their estimated center is not close enough. See Sect [] in the paper
	float	fMaxCenterDistance_;				// maximum distance in pixel between 2 center points
	float	fMaxCenterDistance2_;				// _fMaxCenterDistance * _fMaxCenterDistance

	// Validation - Points within a this threshold are considered to lie on the ellipse contour. See Sect [] in the paper
	float	fDistanceToEllipseContour_;			// maximum distance between a point and the contour. See equation [] in the paper

	// Validation - Assign a score. See Sect [] in the paper
	float	fMinScore_;							// minimum score to confirm a detection
	float	fMinReliability_;					// minimum auxiliary score to confirm a detection


	// auxiliary variables
	cv::Size	szIm_;			// input image size

	std::vector<double> times_;	// times_ is a vector containing the execution time of each step.

	int ACC_N_SIZE;			// size of accumulator N = B/A
	int ACC_R_SIZE;			// size of accumulator R = rho = atan(K)
	int ACC_A_SIZE;			// size of accumulator A

	int* accN;				// pointer to accumulator N
	int* accR;				// pointer to accumulator R
	int* accA;				// pointer to accumulator A

#ifdef DETECT_MARKER_X
	void DetectXMarker(cv::Mat3b& colorIm, std::vector<Ellipse>& ellipses);
	VVP straightLines13_;
	VVP straightLines24_;
	std::vector<cv::Point> X13_P1_;
	std::vector<cv::Point> X13_P2_;
	std::vector<cv::Point> X24_P1_;
	std::vector<cv::Point> X24_P2_;
#endif
#ifdef DETECT_MARKER_X_PLUS
	cv::Mat1b EdgeMap_;
	cv::Mat1b XMap_;
#endif
#ifdef DETECT_MARKER_X_Z
	double CalcXHistError(const cv::Mat& mag, const cv::Mat& angle, int cy, int cx, int rad);
#endif

public:
	float countOfFindEllipse_;
	float countOfGetFastCenter_;
	//Constructor and Destructor
	CNEllipseDetector(void);
	~CNEllipseDetector(void);

	void DetectAfterPreProcessing(std::vector<Ellipse>& ellipses, cv::Mat1b& E, cv::Mat1f& PHI);

	//Detect the ellipses in the gray image
	void Detect(cv::Mat1b& gray, std::vector<Ellipse>& ellipses);

	//Draw the first iTopN ellipses on output
	void DrawDetectedEllipses(cv::Mat3b& output, std::vector<Ellipse>& ellipses, int iTopN = 0, int thickness = 2);

	//Set the parameters of the detector
	void SetParameters(cv::Size	szPreProcessingGaussKernelSize,
		double	dPreProcessingGaussSigma,
		float 	fThPosition,
		float	fMaxCenterDistance,
		int		iMinEdgeLength,
		float	fMinOrientedRectSide,
		float	fDistanceToEllipseContour,
		float	fMinScore,
		float	fMinReliability,
		int     iNs
		);

	// Return the execution time
	double GetExecTime() { return times_[0] + times_[1] + times_[2] + times_[3] + times_[4] + times_[5] + times_[6]; }
	std::vector<double> GetTimes() { return times_; }
#ifdef DETECT_MARKER_X_NET
    caffe::SpireCvMatClassifier cvmat_classifier_;
#endif

private:

	//keys for hash table
	static const ushort PAIR_12 = 0x00;
	static const ushort PAIR_23 = 0x01;
	static const ushort PAIR_34 = 0x02;
	static const ushort PAIR_14 = 0x03;

	//generate keys from pair and indicse
	uint inline GenerateKey(uchar pair, ushort u, ushort v);

	void PrePeocessing(cv::Mat1b& I, cv::Mat1b& DP, cv::Mat1b& DN);

	void RemoveShortEdges(cv::Mat1b& edges, cv::Mat1b& clean);

	void ClusterEllipses(std::vector<Ellipse>& ellipses);

	int FindMaxK(const std::vector<int>& v) const;
	int FindMaxN(const std::vector<int>& v) const;
	int FindMaxA(const std::vector<int>& v) const;

	int FindMaxK(const int* v) const;
	int FindMaxN(const int* v) const;
	int FindMaxA(const int* v) const;

	float GetMedianSlope(std::vector<cv::Point2f>& med, cv::Point2f& M, std::vector<float>& slopes);
	void GetFastCenter(std::vector<cv::Point>& e1, std::vector<cv::Point>& e2, EllipseData& data);


	void DetectEdges13(cv::Mat1b& DP, VVP& points_1, VVP& points_3);
	void DetectEdges24(cv::Mat1b& DN, VVP& points_2, VVP& points_4);

	void FindEllipses(cv::Point2f& center,
		VP& edge_i,
		VP& edge_j,
		VP& edge_k,
		EllipseData& data_ij,
		EllipseData& data_ik,
		std::vector<Ellipse>& ellipses
		);

	cv::Point2f GetCenterCoordinates(EllipseData& data_ij, EllipseData& data_ik);
	cv::Point2f _GetCenterCoordinates(EllipseData& data_ij, EllipseData& data_ik);
    // Jario Edit Start
    void Triplets1234(VVP& p1, VVP& p2, VVP& p3, VVP& p4,
            std::vector<Ellipse>& ellipses);
    // Jario Edit End
	void Triplets124(VVP& pi,
		VVP& pj,
		VVP& pk,
		std::unordered_map<uint, EllipseData>& data,
		std::vector<Ellipse>& ellipses
		);

	void Triplets231(VVP& pi,
		VVP& pj,
		VVP& pk,
		std::unordered_map<uint, EllipseData>& data,
		std::vector<Ellipse>& ellipses
		);

	void Triplets342(VVP& pi,
		VVP& pj,
		VVP& pk,
		std::unordered_map<uint, EllipseData>& data,
		std::vector<Ellipse>& ellipses
		);

	void Triplets413(VVP& pi,
		VVP& pj,
		VVP& pk,
		std::unordered_map<uint, EllipseData>& data,
		std::vector<Ellipse>& ellipses
		);

	void Tic(unsigned idx = 0) //start
	{
		assert(timesSign_[idx] == 0);
		timesSign_[idx]++;
		times_[idx] = (double)cv::getTickCount();
    }

	void Toc(unsigned idx = 0, std::string step = "") //stop
	{
		assert(timesSign_[idx] == 1);
		timesSign_[idx]++;
		times_[idx] = ((double)cv::getTickCount() - times_[idx])*1000. / cv::getTickFrequency();
#ifdef DEBUG_SPEED
		std::cout << "Cost time: " << times_[idx] << " ms [" << idx << "] - " << step << std::endl;
		if (idx == times_.size()-1)
			std::cout << "Totally cost time: " << this->GetExecTime() << " ms" << std::endl;
#endif
    }
private:
	std::vector<int> timesSign_;
	

};

}
#endif // USE_OPENCV
#endif // SPIRE_CN_DETECTOR_HPP_
