#include "ellipse_detector.h"
#include <numeric>
#include <time.h>


using namespace std;
using namespace cv;



#ifndef USE_OMP
int omp_get_max_threads() { return 1; }
int omp_get_thread_num() { return 0; }
// int omp_set_num_threads(int){ return 0; }
#endif


void _list_dir(std::string dir, std::vector<std::string>& files, std::string suffixs, bool r) {
    // assert(_endswith(dir, "/") || _endswith(dir, "\\"));

    DIR *pdir;
    struct dirent *ent;
    string childpath;
    string absolutepath;
    pdir = opendir(dir.c_str());
    assert(pdir != NULL);

    vector<string> suffixd(0);
    if (!suffixs.empty() && suffixs != "") {
        suffixd = _split(suffixs, "|");
    }

    while ((ent = readdir(pdir)) != NULL) {
        if (ent->d_type & DT_DIR) {
            if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) {
                continue;
            }
            if (r) { // If need to traverse subdirectories
                childpath = dir + ent->d_name;
                _list_dir(childpath, files);
            }
        }
        else {
            if (suffixd.size() > 0) {
                bool can_push = false;
                for (int i = 0; i < (int)suffixd.size(); i++) {
                    if (_endswith(ent->d_name, suffixd[i]))
                        can_push = true;
                }
                if (can_push) {
                    absolutepath = dir + ent->d_name;
                    files.push_back(ent->d_name); // filepath
                }
            }
            else {
                absolutepath = dir + ent->d_name;
                files.push_back(ent->d_name); // filepath
            }
        }
    }
    sort(files.begin(), files.end()); //sort names
}

vector<string> _split(const string& srcstr, const string& delimeter)
{
    vector<string> ret(0); //use ret save the spilted reault
    if (srcstr.empty())    //judge the arguments
    {
        return ret;
    }
    string::size_type pos_begin = srcstr.find_first_not_of(delimeter); //find first element of srcstr

    string::size_type dlm_pos; //the delimeter postion
    string temp;               //use third-party temp to save splited element
    while (pos_begin != string::npos) //if not a next of end, continue spliting
    {
        dlm_pos = srcstr.find(delimeter, pos_begin); //find the delimeter symbol
        if (dlm_pos != string::npos)
        {
            temp = srcstr.substr(pos_begin, dlm_pos - pos_begin);
            pos_begin = dlm_pos + delimeter.length();
        }
        else
        {
            temp = srcstr.substr(pos_begin);
            pos_begin = dlm_pos;
        }
        if (!temp.empty())
            ret.push_back(temp);
    }
    return ret;
}

bool _startswith(const std::string& str, const std::string& start)
{
    size_t srclen = str.size();
    size_t startlen = start.size();
    if (srclen >= startlen)
    {
        string temp = str.substr(0, startlen);
        if (temp == start)
            return true;
    }

    return false;
}

bool _endswith(const std::string& str, const std::string& end)
{
    size_t srclen = str.size();
    size_t endlen = end.size();
    if (srclen >= endlen)
    {
        string temp = str.substr(srclen - endlen, endlen);
        if (temp == end)
            return true;
    }

    return false;
}

int inline randint(int l, int u)
{
    return l + rand() % (u - l + 1);
    // rand() % (u-l+1) -> [0, u-l]
    // [0, u-l] -> [l, u]
}

void _randperm(int n, int m, int arr[], bool sort_)
{
    int* x = (int*)malloc(sizeof(int)*n);
    for (int i = 0; i < n; ++i)
        x[i] = i;
    for (int i = 0; i < m; ++i)
    {
        int j = randint(i, n - 1);
        int t = x[i]; x[i] = x[j]; x[j] = t; // swap(x[i], x[j]);
    }
    if (sort_)
        sort(x, x + m);
    for (int i = 0; i < m; ++i)
        arr[i] = x[i];
    free(x);
}

#define FL_PI      3.14159265358979323846f
#define FL_1_2_PI  1.57079632679f
#define FL_2__PI   6.28318530718
float _atan2(float y, float x)
{
    float ang(0);
    if (x > 0)
        ang = atanf(y / x);
    else if (y >= 0 && x < 0)
        ang = atanf(y / x) + FL_PI;
    else if (y < 0 && x < 0)
        ang = atanf(y / x) - FL_PI;
    else if (y > 0 && x == 0)
        ang = FL_1_2_PI;
    else if (y < 0 && x == 0)
        ang = -FL_1_2_PI;
    else // (y == 0 && x == 0)
        ang = INFINITY;
    // if (ang < 0) ang += FL_2__PI;
    return ang;
}

void _mean_std(std::vector<float>& vec, float& mean, float& std)
{
    float sum = std::accumulate(std::begin(vec), std::end(vec), 0.0);
    mean = sum / vec.size();

    float accum = 0.0;
    std::for_each(std::begin(vec), std::end(vec), [&](const double d) {
        accum += (d - mean)*(d - mean);
    });

    std = sqrt(accum / (vec.size() - 1));
}

float _get_min_angle_PI(float alpha, float beta)
{
    float pi = float(CV_PI);
    float pi2 = float(2.0 * CV_PI);

    //normalize data in [0, 2*pi]
    float a = fmod(alpha + pi2, pi2);
    float b = fmod(beta + pi2, pi2);

    //normalize data in [0, pi]
    if (a > pi)
        a -= pi;
    if (b > pi)
        b -= pi;

    if (a > b)
    {
        swap(a, b);
    }

    float diff1 = b - a;
    float diff2 = pi - diff1;
    return min(diff1, diff2);
}


void _load_ellipse_GT(const string& gt_file_name, vector<Ellipse>& gt_ellipses, bool is_angle_radians)
{
    ifstream in(gt_file_name);
    if (!in.good())
    {
        cout << "Error opening: " << gt_file_name << endl;
        return;
    }

    unsigned n;
    in >> n;

    gt_ellipses.clear();
    gt_ellipses.reserve(n);

    while (in.good() && n--)
    {
        Ellipse e;
        in >> e.xc_ >> e.yc_ >> e.a_ >> e.b_ >> e.rad_;

        if (!is_angle_radians)
        {
            // convert to radians
            e.rad_ = float(e.rad_ * CV_PI / 180.0);
        }

        if (e.a_ < e.b_)
        {
            float temp = e.a_;
            e.a_ = e.b_;
            e.b_ = temp;

            e.rad_ = e.rad_ + float(0.5*CV_PI);
        }

        e.rad_ = fmod(float(e.rad_ + 2.f*CV_PI), float(CV_PI));
        e.score_ = 1.f;
        gt_ellipses.push_back(e);
    }
    in.close();
}

void _load_ellipse_DT(const string& dt_file_name, vector<Ellipse>& dt_ellipses, bool is_angle_radians)
{
    ifstream in(dt_file_name);
    if (!in.good())
    {
        cout << "Error opening: " << dt_file_name << endl;
        return;
    }

    unsigned n;
    in >> n;

    dt_ellipses.clear();
    dt_ellipses.reserve(n);

    while (in.good() && n--)
    {
        Ellipse e;
        in >> e.xc_ >> e.yc_ >> e.a_ >> e.b_ >> e.rad_ >> e.score_;

        if (!is_angle_radians)
        {
            // convert to radians
            e.rad_ = float(e.rad_ * CV_PI / 180.0);
        }

        if (e.a_ < e.b_)
        {
            float temp = e.a_;
            e.a_ = e.b_;
            e.b_ = temp;

            e.rad_ = e.rad_ + float(0.5*CV_PI);
        }

        e.rad_ = fmod(float(e.rad_ + 2.f*CV_PI), float(CV_PI));
        e.score_ = 1.f;
        dt_ellipses.push_back(e);
    }
    in.close();
}

bool _ellipse_overlap(const Mat1b& gt, const Mat1b& dt, float th)
{
    float f_and = float(countNonZero(gt & dt));
    float f_or = float(countNonZero(gt | dt));
    float f_sim = f_and / f_or;

    return (f_sim >= th);
}

float _ellipse_overlap_real(const Mat1b& gt, const Mat1b& dt)
{
    float f_and = float(countNonZero(gt & dt));
    float f_or = float(countNonZero(gt | dt));
    float f_sim = f_and / f_or;

    return f_sim;
}

int _bool_count(const std::vector<bool> vb)
{
    int counter = 0;
    for (unsigned i = 0; i<vb.size(); ++i)
    {
        if (vb[i]) ++counter;
    }
    return counter;
}

float _ellipse_evaluate_one(const vector<Ellipse>& ell_gt, const vector<Ellipse>& ell_dt, const Mat3b& img)
{
    float threshold_overlap = 0.8f;
    // float threshold = 0.95f;

    unsigned sz_gt = ell_gt.size();
    unsigned sz_dt = ell_dt.size();

    unsigned sz_dt_min = unsigned(min(1000, int(sz_dt)));

    vector<Mat1b> mat_gts(sz_gt);
    vector<Mat1b> mat_dts(sz_dt_min);

    // Draw each ground-Truth ellipse
    for (unsigned i = 0; i<sz_gt; ++i)
    {
        const Ellipse& e = ell_gt[i];

        Mat1b tmp(img.rows, img.cols, uchar(0));
        ellipse(tmp, Point((int)e.xc_, (int)e.yc_), Size((int)e.a_, (int)e.b_), e.rad_ * 180.0 / CV_PI, 0.0, 360.0, Scalar(255), -1);
        mat_gts[i] = tmp;
    }

    // Draw each detected ellipse
    for (unsigned i = 0; i<sz_dt_min; ++i)
    {
        const Ellipse& e = ell_dt[i];

        Mat1b tmp(img.rows, img.cols, uchar(0));
        ellipse(tmp, Point((int)e.xc_, (int)e.yc_), Size((int)e.a_, (int)e.b_), e.rad_ * 180.0 / CV_PI, 0.0, 360.0, Scalar(255), -1);
        mat_dts[i] = tmp;
    }

    Mat1b overlap(sz_gt, sz_dt_min, uchar(0));
    for (int r = 0; r < overlap.rows; ++r)
    {
        for (int c = 0; c < overlap.cols; ++c)
        {
            // The proportion of overlapping areas to the true area (If biger than, assign to 255)
            overlap(r, c) = _ellipse_overlap(mat_gts[r], mat_dts[c], threshold_overlap) ? uchar(255) : uchar(0);
        }
    }

    int counter = 0;
    vector<bool> vec_gt(sz_gt, false);
    vector<bool> vec_dt(sz_dt_min, false);

    // Each row in the matrix has one means the ellipse be found
    for (unsigned int i = 0; i < sz_dt_min; ++i)
    {
        for (unsigned int j = 0; j < sz_gt; ++j)
        {
            bool b_found = overlap(j, i) != 0;
            if (b_found)
            {
                vec_gt[j] = true;
                vec_dt[i] = true;
            }
        }
    }

    float tp = _bool_count(vec_gt);
    float fn = int(sz_gt) - tp;
    float fp = sz_dt - _bool_count(vec_dt); // !!!! sz_dt - _bool_count(vec_dt); //

    float pr(0.f);
    float re(0.f);
    float fmeasure(0.f);

    if (tp == 0) {
        if (fp == 0) {
            pr = 1.f;
            re = 0.f;
            fmeasure = (2.f * pr * re) / (pr + re);
        }
        else {
            pr = 0.f;
            re = 0.f;
            fmeasure = 0.f;
        }
    }
    else {
        pr = float(tp) / float(tp + fp);
        re = float(tp) / float(tp + fn);
        fmeasure = (2.f * pr * re) / (pr + re);
    }

    return fmeasure;
}

float _ellipse_evaluate(vector<string>& image_fns, vector<string>& gt_fns, vector<string>& dt_fns, bool gt_angle_radians)
{
    float fmeasure(0.f);
    for (int i = 0; i < image_fns.size(); i++) {
        Mat3b image = imread(image_fns[i]);

        vector<Ellipse> ell_gt, ell_dt;
        _load_ellipse_GT(gt_fns[i], ell_gt, gt_angle_radians);
        _load_ellipse_DT(dt_fns[i], ell_dt);

        int tp, fn, fp;
        fmeasure += _ellipse_evaluate_one(ell_gt, ell_dt, image);

    }

    fmeasure /= image_fns.size();
    return fmeasure;
}

Point2f inline _lineCrossPoint(Point2f l1p1, Point2f l1p2, Point2f l2p1, Point2f l2p2)
{
    Point2f crossPoint;
    float k1, k2, b1, b2;
    if (l1p1.x == l1p2.x&&l2p1.x == l2p2.x) {
        crossPoint = Point2f(0, 0); // invalid point
        return crossPoint;
    }
    if (l1p1.x == l1p2.x)
    {
        crossPoint.x = l1p1.x;
        k2 = (l2p2.y - l2p1.y) / (l2p2.x - l2p1.x);
        b2 = l2p1.y - k2*l2p1.x;
        crossPoint.y = k2*crossPoint.x + b2;
        return crossPoint;
    }
    if (l2p1.x == l2p2.x)
    {
        crossPoint.x = l2p1.x;
        k2 = (l1p2.y - l1p1.y) / (l1p2.x - l1p1.x);
        b2 = l1p1.y - k2*l1p1.x;
        crossPoint.y = k2*crossPoint.x + b2;
        return crossPoint;
    }

    k1 = (l1p2.y - l1p1.y) / (l1p2.x - l1p1.x);
    k2 = (l2p2.y - l2p1.y) / (l2p2.x - l2p1.x);
    b1 = l1p1.y - k1*l1p1.x;
    b2 = l2p1.y - k2*l2p1.x;
    if (k1 == k2)
    {
        crossPoint = Point2f(0, 0); // invalid point
    }
    else
    {
        crossPoint.x = (b2 - b1) / (k1 - k2);
        crossPoint.y = k1*crossPoint.x + b1;
    }
    return crossPoint;
}

void inline _point2Mat(Point2f p1, Point2f p2, float mat[2][2])
{
    mat[0][0] = p1.x;
    mat[0][1] = p1.y;
    mat[1][0] = p2.x;
    mat[1][1] = p2.y;
}

float _value4SixPoints(cv::Point2f p3, cv::Point2f p2, cv::Point2f p1, cv::Point2f p4, cv::Point2f p5, cv::Point2f p6)
{
    float result = 1;
    Mat A, B, C;
    float matB[2][2], matC[2][2];
    Point2f v, w, u;
    v = _lineCrossPoint(p1, p2, p3, p4);
    w = _lineCrossPoint(p5, p6, p3, p4);
    u = _lineCrossPoint(p5, p6, p1, p2);

    _point2Mat(u, v, matB);
    _point2Mat(p1, p2, matC);
    B = Mat(2, 2, CV_32F, matB);
    C = Mat(2, 2, CV_32F, matC);
    A = C*B.inv();

    // cout<<"u:\t"<<u<<endl;
    // cout<<"v:\t"<<v<<endl;
    // cout<<"B:\t"<<B<<endl;
    // cout<<A<<endl;

    result *= A.at<float>(0, 0)*A.at<float>(1, 0) / (A.at<float>(0, 1)*A.at<float>(1, 1));

    _point2Mat(p3, p4, matC);
    _point2Mat(v, w, matB);
    B = Mat(2, 2, CV_32F, matB);
    C = Mat(2, 2, CV_32F, matC);
    A = C*B.inv();
    result *= A.at<float>(0, 0)*A.at<float>(1, 0) / (A.at<float>(0, 1)*A.at<float>(1, 1));

    _point2Mat(p5, p6, matC);
    _point2Mat(w, u, matB);
    B = Mat(2, 2, CV_32F, matB);
    C = Mat(2, 2, CV_32F, matC);
    A = C*B.inv();
    result *= A.at<float>(0, 0)*A.at<float>(1, 0) / (A.at<float>(0, 1)*A.at<float>(1, 1));
    return result;
}

/*----------------------------------------------------------------------------*/
/** Compute ellipse foci, given ellipse params.
*/
void _ellipse_foci(float *param, float *foci)
{
    float f;
    /* check parameters */
    if (param == NULL) fprintf(stderr, "ellipse_foci: invalid input ellipse.");
    if (foci == NULL) fprintf(stderr, "ellipse_foci: 'foci' must be non null.");

    f = sqrt(param[2] * param[2] - param[3] * param[3]);
    foci[0] = param[0] + f * cos(param[4]);
    foci[1] = param[1] + f * sin(param[4]);
    foci[2] = param[0] - f * cos(param[4]);
    foci[3] = param[1] - f * sin(param[4]);
}

/*----------------------------------------------------------------------------*/
/** Signed angle difference.
*/
float angle_diff_signed(float a, float b)
{
    a -= b;
    while (a <= -M_PI) a += M_2__PI;
    while (a >   M_PI) a -= M_2__PI;
    return a;
}

/*----------------------------------------------------------------------------*/
/** Absolute value angle difference.
*/
float _angle_diff(float a, float b)
{
    a -= b;
    while (a <= -M_PI) a += M_2__PI;
    while (a >   M_PI) a -= M_2__PI;
    if (a < 0.0) a = -a;
    return a;
}

/*----------------------------------------------------------------------------*/
/** Compute the angle of the normal to a point belonging to an ellipse
using the focal property.
*/
float _ellipse_normal_angle(float x, float y, float *foci)
{
    float tmp1, tmp2, tmp3, theta;
    /* check parameters */
    if (foci == NULL) fprintf(stderr, "ellipse_normal_angle: 'foci' must be non null");

    tmp1 = atan2(y - foci[1], x - foci[0]);
    tmp2 = atan2(y - foci[3], x - foci[2]);
    tmp3 = angle_diff_signed(tmp1, tmp2);

    theta = tmp1 - tmp3 / 2.0;
    while (theta <= -M_PI) theta += M_2__PI;
    while (theta >   M_PI) theta -= M_2__PI;
    return theta;
}


void cv_canny(const void* srcarr, void* dstarr,
    void* dxarr, void* dyarr,
    int aperture_size, bool L2gradient, double percent_ne) {

    cv::AutoBuffer<char> buffer;
    std::vector<uchar*> stack;
    uchar **stack_top = 0, **stack_bottom = 0;

    CvMat srcstub, *src = cvGetMat(srcarr, &srcstub); // IplImage to cvMat
    CvMat dststub, *dst = cvGetMat(dstarr, &dststub);

    CvMat dxstub, *dx = cvGetMat(dxarr, &dxstub);
    CvMat dystub, *dy = cvGetMat(dyarr, &dystub);

    if (CV_MAT_TYPE(src->type) != CV_8UC1 ||
        CV_MAT_TYPE(dst->type) != CV_8UC1 ||
        CV_MAT_TYPE(dx->type) != CV_16SC1 ||
        CV_MAT_TYPE(dy->type) != CV_16SC1)
        CV_Error(CV_StsUnsupportedFormat, "");

    if (!CV_ARE_SIZES_EQ(src, dst))
        CV_Error(CV_StsUnmatchedSizes, "");

    aperture_size &= INT_MAX;
    if ((aperture_size & 1) == 0 || aperture_size < 3 || aperture_size > 7)
        CV_Error(CV_StsBadFlag, "");

    int i, j;
    CvSize size;
    size.width = src->cols;
    size.height = src->rows;

    cvSobel(src, dx, 1, 0, aperture_size);
    cvSobel(src, dy, 0, 1, aperture_size);

    // double min, max;
    // cv::minMaxLoc(Mat(dx->rows, dx->cols, CV_16SC1, dx->data.fl), &min, &max);
    // cout << "min: " << min << ", max: " << max << endl;

    Mat1f magGrad(size.height, size.width, 0.f);
    float maxGrad(0);
    float val(0);
    for (i = 0; i < size.height; ++i)
    {
        float* _pmag = magGrad.ptr<float>(i);
        const short* _dx = (short*)(dx->data.ptr + dx->step*i);
        const short* _dy = (short*)(dy->data.ptr + dy->step*i);
        for (j = 0; j < size.width; ++j)
        {
            val = float(abs(_dx[j]) + abs(_dy[j]));
            _pmag[j] = val;
            maxGrad = (val > maxGrad) ? val : maxGrad;
        }
    }
    // cout << "maxGrad: " << maxGrad << endl;

    // set magic numbers
    const int NUM_BINS = 64;
    const double percent_of_pixels_not_edges = percent_ne;
    const double threshold_ratio = 0.3;

    // compute histogram
    int bin_size = cvFloor(maxGrad / float(NUM_BINS) + 0.5f) + 1;
    if (bin_size < 1) bin_size = 1;
    int bins[NUM_BINS] = { 0 };
    for (i = 0; i < size.height; ++i)
    {
        float *_pmag = magGrad.ptr<float>(i);
        for (j = 0; j < size.width; ++j)
        {
            int hgf = int(_pmag[j]);
            bins[int(_pmag[j]) / bin_size]++;
        }
    }
    // for (int i = 0; i < NUM_BINS; i++)
    //     cout << "BIN " << i << " :" << bins[i] << endl;

    // Select the thresholds
    float total(0.f);
    float target = float(size.height * size.width * percent_of_pixels_not_edges);
    int low_thresh, high_thresh(0);

    while (total < target)
    {
        total += bins[high_thresh];
        high_thresh++;
    }
    high_thresh *= bin_size;
    low_thresh = cvFloor(threshold_ratio * float(high_thresh));
    // cout << "low_thresh: " << low_thresh << ", high_thresh: " << high_thresh << endl;

    int low, high, maxsize;
    int* mag_buf[3];
    uchar* map;
    ptrdiff_t mapstep;

    if (L2gradient) {
        Cv32suf ul, uh;
        ul.f = (float)low_thresh;
        uh.f = (float)high_thresh;

        low = ul.i;
        high = uh.i;
    }
    else {
        low = cvFloor(low_thresh);
        high = cvFloor(high_thresh);
    }

    buffer.allocate((size.width + 2)*(size.height + 2) + (size.width + 2) * 3 * sizeof(int));
    // cout << sizeof(int) << ", " << (size.width + 2)*(size.height + 2) + (size.width + 2) * 3 * sizeof(int) << endl;
    mag_buf[0] = (int*)(char*)buffer;
    mag_buf[1] = mag_buf[0] + size.width + 2;
    mag_buf[2] = mag_buf[1] + size.width + 2;
    map = (uchar*)(mag_buf[2] + size.width + 2);
    mapstep = size.width + 2;

    maxsize = MAX(1 << 10, size.width*size.height / 10);
    stack.resize(maxsize);
    stack_top = stack_bottom = &stack[0];

    memset(mag_buf[0], 0, (size.width + 2) * sizeof(int));
    memset(map, 1, mapstep);
    memset(map + mapstep*(size.height + 1), 1, mapstep);

    /* sector numbers
    (Top-Left Origin)

    1   2   3
    *  *  *
    * * *
    0*******0
    * * *
    *  *  *
    3   2   1
    */

#define CANNY_PUSH(d)    *(d) = (uchar)2, *stack_top++ = (d)
#define CANNY_POP(d)     (d) = *--stack_top

    CvMat mag_row = cvMat(1, size.width, CV_32F);

    // Mat push_show = Mat::zeros(size.height+1, size.width+1, CV_8U);

    // calculate magnitude and angle of gradient, perform non-maxima supression.
    // fill the map with one of the following values:
    //   0 - the pixel might belong to an edge
    //   1 - the pixel can not belong to an edge
    //   2 - the pixel does belong to an edge
    for (i = 0; i <= size.height; i++)
    {
        int* _mag = mag_buf[(i > 0) + 1] + 1;
        float* _magf = (float*)_mag;
        const short* _dx = (short*)(dx->data.ptr + dx->step*i);
        const short* _dy = (short*)(dy->data.ptr + dy->step*i);
        uchar* _map;
        int x, y;
        ptrdiff_t magstep1, magstep2;
        int prev_flag = 0;

        if (i < size.height)
        {
            _mag[-1] = _mag[size.width] = 0;

            if (!L2gradient)
                for (j = 0; j < size.width; j++)
                    _mag[j] = abs(_dx[j]) + abs(_dy[j]);
            else
            {
                for (j = 0; j < size.width; j++)
                {
                    x = _dx[j]; y = _dy[j];
                    _magf[j] = (float)std::sqrt((double)x*x + (double)y*y);
                }
            }
        }
        else
            memset(_mag - 1, 0, (size.width + 2) * sizeof(int));

        // at the very beginning we do not have a complete ring
        // buffer of 3 magnitude rows for non-maxima suppression
        if (i == 0)
            continue;

        _map = map + mapstep*i + 1;
        _map[-1] = _map[size.width] = 1;

        _mag = mag_buf[1] + 1; // take the central row
        _dx = (short*)(dx->data.ptr + dx->step*(i - 1));
        _dy = (short*)(dy->data.ptr + dy->step*(i - 1));

        magstep1 = mag_buf[2] - mag_buf[1];
        magstep2 = mag_buf[0] - mag_buf[1];

        if ((stack_top - stack_bottom) + size.width > maxsize)
        {
            int sz = (int)(stack_top - stack_bottom);
            maxsize = MAX(maxsize * 3 / 2, maxsize + 8);
            stack.resize(maxsize);
            stack_bottom = &stack[0];
            stack_top = stack_bottom + sz;
        }

#define CANNY_SHIFT 15
#define TG22  (int)(0.4142135623730950488016887242097*(1<<CANNY_SHIFT) + 0.5)

        for (j = 0; j < size.width; j++)
        {
            x = _dx[j];
            y = _dy[j];
            int s = x ^ y; // XOR
            int m = _mag[j];

            x = abs(x);
            y = abs(y);
            if (m > low)
            {
                int tg22x = x * TG22;
                int tg67x = tg22x + ((x + x) << CANNY_SHIFT);
                int tmp = 1 << CANNY_SHIFT;
                y <<= CANNY_SHIFT;

                if (y < tg22x) {
                    if (m > _mag[j - 1] && m >= _mag[j + 1]) {
                        if (m > high && !prev_flag && _map[j - mapstep] != 2) {
                            CANNY_PUSH(_map + j); // push_show.at<uchar>(i, j) = 255;
                            prev_flag = 1;
                        }
                        else {
                            _map[j] = (uchar)0;
                        }
                        continue;
                    }
                }
                else if (y > tg67x) {
                    if (m > _mag[j + magstep2] && m >= _mag[j + magstep1]) {
                        if (m > high && !prev_flag && _map[j - mapstep] != 2) {
                            CANNY_PUSH(_map + j); // push_show.at<uchar>(i, j) = 255;
                            prev_flag = 1;
                        }
                        else {
                            _map[j] = (uchar)0;
                        }
                        continue;
                    }
                }
                else {
                    s = s < 0 ? -1 : 1;
                    if (m > _mag[j + magstep2 - s] && m > _mag[j + magstep1 + s]) {
                        if (m > high && !prev_flag && _map[j - mapstep] != 2) {
                            CANNY_PUSH(_map + j); // push_show.at<uchar>(i, j) = 255;
                            prev_flag = 1;
                        }
                        else {
                            _map[j] = (uchar)0;
                        }
                        continue;
                    }
                }
            }
            prev_flag = 0;
            _map[j] = (uchar)1;
        }

        // scroll the ring buffer
        _mag = mag_buf[0];
        mag_buf[0] = mag_buf[1];
        mag_buf[1] = mag_buf[2];
        mag_buf[2] = _mag;
    }

    // imshow("mag", push_show); waitKey();
    // now track the edges (hysteresis thresholding)
    while (stack_top > stack_bottom)
    {
        uchar* m;
        if ((stack_top - stack_bottom) + 8 > maxsize)
        {
            int sz = (int)(stack_top - stack_bottom);
            maxsize = MAX(maxsize * 3 / 2, maxsize + 8);
            stack.resize(maxsize);
            stack_bottom = &stack[0];
            stack_top = stack_bottom + sz;
        }

        CANNY_POP(m);

        if (!m[-1])
            CANNY_PUSH(m - 1);
        if (!m[1])
            CANNY_PUSH(m + 1);
        if (!m[-mapstep - 1])
            CANNY_PUSH(m - mapstep - 1);
        if (!m[-mapstep])
            CANNY_PUSH(m - mapstep);
        if (!m[-mapstep + 1])
            CANNY_PUSH(m - mapstep + 1);
        if (!m[mapstep - 1])
            CANNY_PUSH(m + mapstep - 1);
        if (!m[mapstep])
            CANNY_PUSH(m + mapstep);
        if (!m[mapstep + 1])
            CANNY_PUSH(m + mapstep + 1);
    }

    // the final pass, form the final image
    for (i = 0; i < size.height; i++) {
        const uchar* _map = map + mapstep*(i + 1) + 1;
        uchar* _dst = dst->data.ptr + dst->step*i;

        for (j = 0; j < size.width; j++) {
            // if (_map[j] == 2)
            //     cout << (int)_map[j] << ", " << (int)(_map[j] >> 1) << ", " << (int)(uchar)-(_map[j] >> 1) << endl;
            _dst[j] = (uchar)-(_map[j] >> 1);
        }
    }
}

void _tag_canny(InputArray image, OutputArray _edges,
    OutputArray _sobel_x, OutputArray _sobel_y,
    int apertureSize, bool L2gradient, double percent_ne) {

    Mat src = image.getMat();
    _edges.create(src.size(), CV_8U);
    _sobel_x.create(src.size(), CV_16S);
    _sobel_y.create(src.size(), CV_16S);

    IplImage c_src = IplImage(src);
    IplImage c_dst = IplImage(_edges.getMat());
    IplImage c_dx = IplImage(_sobel_x.getMat());
    IplImage c_dy = IplImage(_sobel_y.getMat());

    cv_canny(&c_src, &c_dst,
        &c_dx, &c_dy,
        apertureSize, L2gradient, percent_ne);
}

void _find_contours_eight(cv::Mat1b& image, std::vector<VVP>& segments, int iMinLength)
{
    vector<Mat1b> ims; ims.resize(8);
    for (int i = 0; i < 8; i++) {
        ims[i] = Mat1b::zeros(image.size());
    }
    for (int r = 0; r < image.rows; r++) {
        uchar* _e8 = image.ptr<uchar>(r);
        vector<uchar*> _es; _es.resize(8);
        for (int i = 0; i < 8; i++)
            _es[i] = ims[i].ptr<uchar>(r);

        for (int c = 0; c < image.cols; c++) {
            for (int i = 0; i < 8; i++) {
                if (_e8[c] == (uchar)(i + 1)) {
                    _es[i][c] = (uchar)255;
                }
            }
        }
    }

    segments.resize(8);
    for (int i = 0; i < 8; i++) {
        _tag_find_contours(ims[i], segments[i], iMinLength);
    }
}

void _tag_find_contours(cv::Mat1b& image, VVP& segments, int iMinLength) {
#define RG_STACK_SIZE 8192*4


    // use stack to speed up the processing of global (even at the expense of memory occupied)
    int stack2[RG_STACK_SIZE];
#define RG_PUSH2(a) (stack2[sp2] = (a) , sp2++)
#define RG_POP2(a) (sp2-- , (a) = stack2[sp2])

    // use stack to speed up the processing of global (even at the expense of memory occupied)
    Point stack3[RG_STACK_SIZE];
#define RG_PUSH3(a) (stack3[sp3] = (a) , sp3++)
#define RG_POP3(a) (sp3-- , (a) = stack3[sp3])

    int i, w, h, iDim;
    int x, y;
    int x2, y2;
    int sp2; // stack pointer
    int sp3;

    Mat_<uchar> src = image.clone();
    w = src.cols;
    h = src.rows;
    iDim = w*h;

    Point point;
    for (y = 0; y<h; ++y)
    {
        for (x = 0; x<w; ++x)
        {
            if ((src(y, x)) != 0)   // found labelled point
            {
                // per point
                sp2 = 0;
                i = x + y*w;
                RG_PUSH2(i);

                // empty the list of points
                sp3 = 0;
                while (sp2>0)
                {   // rg traditional
                    RG_POP2(i);
                    x2 = i%w;
                    y2 = i / w;

                    point.x = x2;
                    point.y = y2;

                    if (src(y2, x2))
                    {
                        RG_PUSH3(point);
                        src(y2, x2) = 0;
                    }

                    // insert the new points in the stack only if there are
                    // and they are points labelled

                    // 4 connected
                    // left
                    if (x2>0 && (src(y2, x2 - 1) != 0))
                        RG_PUSH2(i - 1);
                    // up
                    if (y2>0 && (src(y2 - 1, x2) != 0))
                        RG_PUSH2(i - w);
                    // down
                    if (y2<h - 1 && (src(y2 + 1, x2) != 0))
                        RG_PUSH2(i + w);
                    // right
                    if (x2<w - 1 && (src(y2, x2 + 1) != 0))
                        RG_PUSH2(i + 1);

                    // 8 connected
                    if (x2>0 && y2>0 && (src(y2 - 1, x2 - 1) != 0))
                        RG_PUSH2(i - w - 1);
                    if (x2>0 && y2<h - 1 && (src(y2 + 1, x2 - 1) != 0))
                        RG_PUSH2(i + w - 1);
                    if (x2<w - 1 && y2>0 && (src(y2 - 1, x2 + 1) != 0))
                        RG_PUSH2(i - w + 1);
                    if (x2<w - 1 && y2<h - 1 && (src(y2 + 1, x2 + 1) != 0))
                        RG_PUSH2(i + w + 1);

                }

                if (sp3 >= iMinLength)
                {
                    vector<Point> component;
                    component.reserve(sp3);

                    // push it to the points
                    for (i = 0; i<sp3; i++) {
                        // push it
                        component.push_back(stack3[i]);
                    }
                    segments.push_back(component);
                }
            }
        }
    }





}



void _find_contours_oneway(cv::Mat1b& image, VVP& segments, int iMinLength) {

    int w, h, iDim;
    int x, y;
    int x2, y2;
    int sp1, sp2; // stack pointer
    float ang, ang_d, ang_t, ang_i, ang_dmin;
    float theta_s, theta_i;
    int xn, yn;

    Mat_<uchar> src = image.clone();
    w = src.cols;
    h = src.rows;
    iDim = w*h;

    Point point;
    for (y = 0; y<h; ++y)
    {
        for (x = 0; x<w; ++x)
        {
            if ((src(y, x)) != 0)   // found labelled point
            {
                // per point
                src(y, x) = 0;

                vector<Point> component;
                point.x = x;
                point.y = y;
                component.push_back(point);
                x2 = x;
                y2 = y;

                bool found = true;
                sp2 = 0;
                while (found && component.size() < 3)
                {
                    found = false;
                    if (x2 > 0 && y2 < h - 1 && (src(y2 + 1, x2 - 1) != 0))
                    {
                        src(y2 + 1, x2 - 1) = 0;  if (!found) { found = true; point.x = x2 - 1; point.y = y2 + 1; component.push_back(point); }
                    }
                    if (x2 < w - 1 && y2 < h - 1 && (src(y2 + 1, x2 + 1) != 0))
                    {
                        src(y2 + 1, x2 + 1) = 0;  if (!found) { found = true; point.x = x2 + 1; point.y = y2 + 1; component.push_back(point); }
                    }
                    if (y2 < h - 1 && (src(y2 + 1, x2) != 0))
                    {
                        src(y2 + 1, x2) = 0;  if (!found) { found = true; point.x = x2; point.y = y2 + 1; component.push_back(point); }
                    }
                    if (x2 > 0 && (src(y2, x2 - 1) != 0))
                    {
                        src(y2, x2 - 1) = 0;  if (!found) { found = true; point.x = x2 - 1; point.y = y2; component.push_back(point); }
                    }
                    if (x2 < w - 1 && (src(y2, x2 + 1) != 0))
                    {
                        src(y2, x2 + 1) = 0;  if (!found) { found = true; point.x = x2 + 1; point.y = y2; component.push_back(point); }
                    }
                    if (x2 > 0 && y2 > 0 && (src(y2 - 1, x2 - 1) != 0))
                    {
                        src(y2 - 1, x2 - 1) = 0;  if (!found) { found = true; point.x = x2 - 1; point.y = y2 - 1; component.push_back(point); }
                    }
                    if (x2 < w - 1 && y2 > 0 && (src(y2 - 1, x2 + 1) != 0))
                    {
                        src(y2 - 1, x2 + 1) = 0;  if (!found) { found = true; point.x = x2 + 1; point.y = y2 - 1; component.push_back(point); }
                    }
                    if (y2 > 0 && (src(y2 - 1, x2) != 0))
                    {
                        src(y2 - 1, x2) = 0;  if (!found) { found = true; point.x = x2; point.y = y2 - 1; component.push_back(point); }
                    }
                    if (found)
                    {
                        sp2++; x2 = component[sp2].x; y2 = component[sp2].y;
                    }
                }

                if (component.size() < 3) continue;
                ang = _atan2(component[2].y - component[0].y, component[2].x - component[0].x);
                sp1 = 0;

                found = true;
                while (found)
                {
                    ang_dmin = 1e3;
                    found = false;
                    if (x2 > 0 && y2 < h - 1 && (src(y2 + 1, x2 - 1) != 0))
                    {
                        ang_t = _atan2(y2 + 1 - component[sp1].y, x2 - 1 - component[sp1].x);
                        ang_d = abs(ang - ang_t);
                        src(y2 + 1, x2 - 1) = 0;  if (ang_d < ang_dmin) { ang_dmin = ang_d; ang_i = ang_t; xn = x2 - 1; yn = y2 + 1; }
                    }
                    if (x2 < w - 1 && y2 < h - 1 && (src(y2 + 1, x2 + 1) != 0))
                    {
                        ang_t = _atan2(y2 + 1 - component[sp1].y, x2 + 1 - component[sp1].x);
                        ang_d = abs(ang - ang_t);
                        src(y2 + 1, x2 + 1) = 0;  if (ang_d < ang_dmin) { ang_dmin = ang_d; ang_i = ang_t; xn = x2 + 1; yn = y2 + 1; }
                    }
                    if (y2 < h - 1 && (src(y2 + 1, x2) != 0))
                    {
                        ang_t = _atan2(y2 + 1 - component[sp1].y, x2 - component[sp1].x);
                        ang_d = abs(ang - ang_t);
                        src(y2 + 1, x2) = 0;  if (ang_d < ang_dmin) { ang_dmin = ang_d; ang_i = ang_t; xn = x2; yn = y2 + 1; }
                    }
                    if (x2 > 0 && (src(y2, x2 - 1) != 0))
                    {
                        ang_t = _atan2(y2 - component[sp1].y, x2 - 1 - component[sp1].x);
                        ang_d = abs(ang - ang_t);
                        src(y2, x2 - 1) = 0;  if (ang_d < ang_dmin) { ang_dmin = ang_d; ang_i = ang_t; xn = x2 - 1; yn = y2; }
                    }
                    if (x2 < w - 1 && (src(y2, x2 + 1) != 0))
                    {
                        ang_t = _atan2(y2 - component[sp1].y, x2 + 1 - component[sp1].x);
                        ang_d = abs(ang - ang_t);
                        src(y2, x2 + 1) = 0;  if (ang_d < ang_dmin) { ang_dmin = ang_d; ang_i = ang_t; xn = x2 + 1; yn = y2; }
                    }
                    if (x2 > 0 && y2 > 0 && (src(y2 - 1, x2 - 1) != 0))
                    {
                        ang_t = _atan2(y2 - 1 - component[sp1].y, x2 - 1 - component[sp1].x);
                        ang_d = abs(ang - ang_t);
                        src(y2 - 1, x2 - 1) = 0;  if (ang_d < ang_dmin) { ang_dmin = ang_d; ang_i = ang_t; xn = x2 - 1; yn = y2 - 1; }
                    }
                    if (x2 < w - 1 && y2 > 0 && (src(y2 - 1, x2 + 1) != 0))
                    {
                        ang_t = _atan2(y2 - 1 - component[sp1].y, x2 + 1 - component[sp1].x);
                        ang_d = abs(ang - ang_t);
                        src(y2 - 1, x2 + 1) = 0;  if (ang_d < ang_dmin) { ang_dmin = ang_d; ang_i = ang_t; xn = x2 + 1; yn = y2 - 1; }
                    }
                    if (y2 > 0 && (src(y2 - 1, x2) != 0))
                    {
                        ang_t = _atan2(y2 - 1 - component[sp1].y, x2 - component[sp1].x);
                        ang_d = abs(ang - ang_t);
                        src(y2 - 1, x2) = 0;  if (ang_d < ang_dmin) { ang_dmin = ang_d; ang_i = ang_t; xn = x2; yn = y2 - 1; }
                    }
                    if (ang_dmin < M_1_2_PI)
                    {
                        found = true; point.x = xn; point.y = yn; component.push_back(point);
                        x2 = xn;
                        y2 = yn;
                        sp1++; sp2++;
                        if (sp2 >= 9 && sp2 % 3 == 0)
                        {
                            float a1 = _atan2(component[sp2 - 6].y - component[sp2 - 9].y, component[sp2 - 6].x - component[sp2 - 9].x);
                            float a2 = _atan2(component[sp2 - 3].y - component[sp2 - 6].y, component[sp2 - 3].x - component[sp2 - 6].x);
                            theta_s = a1 - a2;

                            a1 = _atan2(component[sp2 - 3].y - component[sp2 - 6].y, component[sp2 - 3].x - component[sp2 - 6].x);
                            a2 = _atan2(component[sp2].y - component[sp2 - 3].y, component[sp2].x - component[sp2 - 3].x);
                            theta_i = a1 - a2;
                            if (abs(theta_s - theta_i) > 0.6) break;
                        }
                        ang = ang_i;
                    }
                }

                if (component.size() >= iMinLength)
                {
                    segments.push_back(component);
                }
            }
        }
    }
}



void _tag_show_contours(cv::Mat1b& image, VVP& segments, const char* title)
{
    Mat3b contoursIm(image.rows, image.cols, Vec3b(0, 0, 0));
    for (size_t i = 0; i<segments.size(); ++i)
    {
        Vec3b color(rand() % 255, 128 + rand() % 127, 128 + rand() % 127);
        for (size_t j = 0; j<segments[i].size(); ++j)
            contoursIm(segments[i][j]) = color;
    }
    imshow(title, contoursIm);
}

void _tag_show_contours(cv::Size& imsz, VVP& segments, const char* title)
{
    Mat3b contoursIm(imsz, Vec3b(0, 0, 0));
    for (size_t i = 0; i<segments.size(); ++i)
    {
        Vec3b color(rand() % 255, 128 + rand() % 127, 128 + rand() % 127);
        for (size_t j = 0; j<segments[i].size(); ++j)
            contoursIm(segments[i][j]) = color;
    }
    imshow(title, contoursIm);
}

void _show_contours_eight(cv::Mat1b& image, std::vector<VVP>& segments8, const char* title)
{
    Mat3b contoursIm(image.rows, image.cols, Vec3b(0, 0, 0));
    for (size_t i = 0; i < segments8.size(); ++i)
    {
        Vec3b color(rand() % 255, 128 + rand() % 127, 128 + rand() % 127);
        for (size_t j = 0; j < segments8[i].size(); ++j)
        {
            for (size_t k = 0; k < segments8[i][j].size(); ++k)
                contoursIm(segments8[i][j][k]) = color;
        }
    }
    imshow(title, contoursIm);
}

bool _SortBottomLeft2TopRight(const Point& lhs, const Point& rhs)
{
    if (lhs.x == rhs.x)
    {
        return lhs.y > rhs.y;
    }
    return lhs.x < rhs.x;
}

bool _SortBottomLeft2TopRight2f(const Point2f& lhs, const Point2f& rhs)
{
    if (lhs.x == rhs.x)
    {
        return lhs.y > rhs.y;
    }
    return lhs.x < rhs.x;
}

bool _SortTopLeft2BottomRight(const Point& lhs, const Point& rhs)
{
    if (lhs.x == rhs.x)
    {
        return lhs.y < rhs.y;
    }
    return lhs.x < rhs.x;
}






// #define DEBUG_SPEED
// #define DEBUG_ELLFIT
// #define DEBUG_PREPROCESSING

// #define DISCARD_TCN
#define DISCARD_TCN2
#define DISCARD_CONSTRAINT_OBOX

// #define DISCARD_CONSTRAINT_CONVEXITY
// #define DISCARD_CONSTRAINT_POSITION
#define CONSTRAINT_CNC_1
// #define CONSTRAINT_CNC_2
// #define CONSTRAINT_CNC_3
// #define DISCARD_CONSTRAINT_CENTER

// #define T_CNC 0.2f
// #define T_TCN_L 0.4f // filter lines
// #define T_TCN_P 0.6f

// #define Thre_r 0.2f


void _concate_arcs(VP& arc1, VP& arc2, VP& arc3, VP& arc)
{
    for (int i = 0; i < arc1.size(); i++)
        arc.push_back(arc1[i]);
    for (int i = 0; i < arc2.size(); i++)
        arc.push_back(arc2[i]);
    for (int i = 0; i < arc3.size(); i++)
        arc.push_back(arc3[i]);
}

void _concate_arcs(VP& arc1, VP& arc2, VP& arc)
{
    for (int i = 0; i < arc1.size(); i++)
        arc.push_back(arc1[i]);
    for (int i = 0; i < arc2.size(); i++)
        arc.push_back(arc2[i]);
}

EllipseDetector::EllipseDetector(void)
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
    dPercentNe_ = 0.9;

    fT_CNC_ = 0.2f;
    fT_TCN_L_ = 0.4f; // filter lines
    fT_TCN_P_ = 0.6f;
    fThre_r_ = 0.2f;

    srand(unsigned(time(NULL)));
}

EllipseDetector::~EllipseDetector(void)
{
}

void EllipseDetector::SetParameters(Size  szPreProcessingGaussKernel,
    double  dPreProcessingGaussSigma,
    float   fThPosition,
    float   fMaxCenterDistance,
    int     iMinEdgeLength,
    float   fMinOrientedRectSide,
    float   fDistanceToEllipseContour,
    float   fMinScore,
    float   fMinReliability,
    int     iNs,
    double  dPercentNe,
    float   fT_CNC,
    float   fT_TCN_L,
    float   fT_TCN_P,
    float   fThre_r
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
    dPercentNe_ = dPercentNe;

    fT_CNC_ = fT_CNC;
    fT_TCN_L_ = fT_TCN_L; // filter lines
    fT_TCN_P_ = fT_TCN_P;
    fThre_r_ = fThre_r;

    fMaxCenterDistance2_ = fMaxCenterDistance_ * fMaxCenterDistance_;
}

void EllipseDetector::RemoveStraightLine(VVP& segments, VVP& segments_update, int id)
{
    int countedges = 0;
    // For each edge
    for (int i = 0; i < segments.size(); ++i)
    {
        VP& edgeSegment = segments[i];

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
        if (id == 0 || id == 1 || id == 4 || id == 5) {
            sort(edgeSegment.begin(), edgeSegment.end(), _SortTopLeft2BottomRight);
        }
        else if (id == 2 || id == 3 || id == 6 || id == 7) {
            sort(edgeSegment.begin(), edgeSegment.end(), _SortBottomLeft2TopRight);
        }

        int iEdgeSegmentSize = int(edgeSegment.size());

        // Get extrema of the arc
        Point& left = edgeSegment[0];
        Point& right = edgeSegment[iEdgeSegmentSize - 1];

#ifndef DISCARD_TCN
#ifndef DISCARD_TCN2
        int flag = 0;
        for (int j = 0; j<iEdgeSegmentSize; j++) {
            Point& mid = edgeSegment[j];
            float data[] = { left.x, left.y, 1, mid.x, mid.y, 1, right.x, right.y, 1 };
            Mat threePoints(Size(3, 3), CV_32FC1, data);
            double ans = determinant(threePoints);

            float dx = 1.0f*(left.x - right.x);
            float dy = 1.0f*(left.y - right.y);
            float edgelength2 = dx*dx + dy*dy;
            // double TCNl = ans/edgelength2;
            double TCNl = ans / (2 * sqrt(edgelength2));
            if (abs(TCNl) > fT_TCN_L_) {
                flag = 1;
                break;
            }
        }
        if (0 == flag) {
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
        double TCNl = ans / (2 * pow(edgelength2, fT_TCN_P_));
        if (abs(TCNl) < fT_TCN_L_) {
            countedges++;
            continue;
        }
#endif
#endif

        segments_update.push_back(edgeSegment);
    }
}

void EllipseDetector::Detect(cv::Mat& I, std::vector<Ellipse>& ellipses)
{
    countOfFindEllipse_ = 0;
    countOfGetFastCenter_ = 0;

    Mat1b gray;
    cvtColor(I, gray, CV_BGR2GRAY);


    // Set the image size
    szIm_ = I.size();

    // Initialize temporary data structures
    Mat1b DP = Mat1b::zeros(szIm_);		// arcs along positive diagonal
    Mat1b DN = Mat1b::zeros(szIm_);		// arcs along negative diagonal


    ACC_N_SIZE = 101;
    ACC_R_SIZE = 180;
    ACC_A_SIZE = max(szIm_.height, szIm_.width);

    // Allocate accumulators
    accN = new int[ACC_N_SIZE];
    accR = new int[ACC_R_SIZE];
    accA = new int[ACC_A_SIZE];


    // Other temporary

    unordered_map<uint, EllipseData> centers;		// hash map for reusing already computed EllipseData

    PreProcessing(gray, DP, DN);


    points_1.clear();
    points_2.clear();
    points_3.clear();
    points_4.clear();
    // Detect edges and find convexities
    DetectEdges13(DP, points_1, points_3);
    DetectEdges24(DN, points_2, points_4);


    Triplets124(points_1, points_2, points_4, centers, ellipses);
    Triplets231(points_2, points_3, points_1, centers, ellipses);
    Triplets342(points_3, points_4, points_2, centers, ellipses);
    Triplets413(points_4, points_1, points_3, centers, ellipses);

    sort(ellipses.begin(), ellipses.end());


    // Free accumulator memory
    delete[] accN;
    delete[] accR;
    delete[] accA;

    // Cluster detections
    ClusterEllipses(ellipses);
}

void EllipseDetector::Detect(Mat3b& I, vector<Ellipse>& ellipses)
{
    countOfFindEllipse_ = 0;
    countOfGetFastCenter_ = 0;

#ifdef DEBUG_SPEED
    Tic(0); // prepare data structure
#endif

    // Convert to grayscale
    Mat1b gray;
    cvtColor(I, gray, CV_BGR2GRAY);

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

    unordered_map<uint, EllipseData> centers;		// hash map for reusing already computed EllipseData

#ifdef DEBUG_SPEED
    Toc(0, "prepare data structure"); // prepare data structure
#endif

                                      // Preprocessing
                                      // From input image I, find edge point with coarse convexity along positive (DP) or negative (DN) diagonal
    PreProcessing(gray, DP, DN);

#ifdef DEBUG_SPEED
    Tic(3); // preprocessing
#endif

    points_1.clear();
    points_2.clear();
    points_3.clear();
    points_4.clear();
    // Detect edges and find convexities
    DetectEdges13(DP, points_1, points_3);
    DetectEdges24(DN, points_2, points_4);

#ifdef DEBUG_SPEED
    Toc(3, "preprocessing_2"); // preprocessing
#endif

                               // #define DEBUG_PREPROCESSING_S4
#ifdef DEBUG_PREPROCESSING_S4
    Mat3b out(I.rows, I.cols, Vec3b(0, 0, 0));
    for (unsigned i = 0; i < points_1.size(); ++i)
    {
        // Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
        Vec3b color(255, 0, 0);
        for (unsigned j = 0; j < points_1[i].size(); ++j)
            out(points_1[i][j]) = color;
    }
    for (unsigned i = 0; i < points_2.size(); ++i)
    {
        // Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
        Vec3b color(0, 255, 0);
        for (unsigned j = 0; j < points_2[i].size(); ++j)
            out(points_2[i][j]) = color;
    }
    for (unsigned i = 0; i < points_3.size(); ++i)
    {
        // Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
        Vec3b color(0, 0, 255);
        for (unsigned j = 0; j < points_3[i].size(); ++j)
            out(points_3[i][j]) = color;
    }
    for (unsigned i = 0; i < points_4.size(); ++i)
    {
        // Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
        Vec3b color(255, 0, 255);
        for (unsigned j = 0; j < points_4[i].size(); ++j)
            out(points_4[i][j]) = color;
    }
    cv::imshow("PreProcessing->Output", out);
    waitKey();
#endif

#ifdef DEBUG_SPEED
    Tic(4); // grouping
#endif


    Triplets124(points_1, points_2, points_4, centers, ellipses);
    Triplets231(points_2, points_3, points_1, centers, ellipses);
    Triplets342(points_3, points_4, points_2, centers, ellipses);
    Triplets413(points_4, points_1, points_3, centers, ellipses);

#ifdef DEBUG_SPEED
    Toc(4, "grouping"); // grouping
#endif

#ifdef DEBUG_SPEED
    Tic(5);
#endif

    // Sort detected ellipses with respect to score
    sort(ellipses.begin(), ellipses.end());

    // Free accumulator memory
    delete[] accN;
    delete[] accR;
    delete[] accA;

    // Cluster detections
    ClusterEllipses(ellipses);

#ifdef DEBUG_SPEED
    Toc(5, "cluster detections");
#endif
}

void EllipseDetector::PreProcessing(Mat1b& I, Mat1b& arcs8)
{
    GaussianBlur(I, I, szPreProcessingGaussKernel_, dPreProcessingGaussSigma_);

    // Temp variables
    Mat1b E;				// edge mask
    Mat1s DX, DY;			// sobel derivatives

    _tag_canny(I, E, DX, DY, 3, false, dPercentNe_);  // Detect edges

                                         // Mat1f dxf, dyf;
                                         // normalize(DX, dxf, 0, 1, NORM_MINMAX);
                                         // normalize(DY, dyf, 0, 1, NORM_MINMAX);
    //Mat1f dx, dy;
    //DX.convertTo(dx, CV_32F);
    //DY.convertTo(dy, CV_32F);
    //// DYDX_ = dy / dx;

    ////Mat1f edx, edy;
    ////Sobel(E, edx, CV_32F, 1, 0);
    ////Sobel(E, edy, CV_32F, 0, 1);

    //DYDX_ = -1 / (dy / dx);
    //CV_Assert(DYDX_.type() == CV_32F);

    //cv::GaussianBlur(dx, dx, Size(5, 5), 0, 0);
    //cv::GaussianBlur(dy, dy, Size(5, 5), 0, 0);
    //cv::phase(dx, dy, EO_);

#define SIGN(n) ((n)<=0?((n)<0?-1:0):1)

    // cout << SIGN(0) << " " << SIGN(-1) << " " << SIGN(1) << endl;

    int sign_x, sign_y, sign_xy;
    for (int i = 0; i < szIm_.height; ++i) {
        short* _dx = DX.ptr<short>(i);
        short* _dy = DY.ptr<short>(i);
        uchar* _e = E.ptr<uchar>(i);
        uchar* _arc = arcs8.ptr<uchar>(i);

        for (int j = 0; j < szIm_.width; ++j) {
            if (!(_e[j] <= 0)) // !!!!!!
            {
                sign_x = SIGN(_dx[j]);
                sign_y = SIGN(_dy[j]);
                sign_xy = SIGN(abs(_dx[j]) - abs(_dy[j]));
                if (sign_x == 1 && sign_y == 1 && sign_xy == 1) {
                    _arc[j] = (uchar)3;
                }
                else if (sign_x == 1 && sign_y == 1 && sign_xy == -1) {
                    _arc[j] = (uchar)4;
                }
                else if (sign_x == 1 && sign_y == -1 && sign_xy == -1) {
                    _arc[j] = (uchar)1;
                }
                else if (sign_x == 1 && sign_y == -1 && sign_xy == 1) {
                    _arc[j] = (uchar)2;
                }
                else if (sign_x == -1 && sign_y == -1 && sign_xy == 1) {
                    _arc[j] = (uchar)7;
                }
                else if (sign_x == -1 && sign_y == -1 && sign_xy == -1) {
                    _arc[j] = (uchar)8;
                }
                else if (sign_x == -1 && sign_y == 1 && sign_xy == -1) {
                    _arc[j] = (uchar)5;
                }
                else if (sign_x == -1 && sign_y == 1 && sign_xy == 1) {
                    _arc[j] = (uchar)6;
                }
            }
        }
    }
}

void EllipseDetector::PreProcessing(Mat1b& I, Mat1b& DP, Mat1b& DN)
{
#ifdef DEBUG_SPEED
    Tic(1); // edge detection
#endif

    GaussianBlur(I, I, szPreProcessingGaussKernel_, dPreProcessingGaussSigma_);

    // Temp variables
    Mat1b E;				// edge mask
    Mat1s DX, DY;			// sobel derivatives

                            // Detect edges
    _tag_canny(I, E, DX, DY, 3, false, dPercentNe_);

    Mat1f dx, dy;
    DX.convertTo(dx, CV_32F);
    DY.convertTo(dy, CV_32F);
    //// cv::GaussianBlur(dx, dx, Size(5, 5), 0, 0);
    //// cv::GaussianBlur(dy, dy, Size(5, 5), 0, 0);
    cv::phase(dx, dy, EO_);

#ifdef DEBUG_PREPROCESSING
    imshow("PreProcessing->Edge", E); waitKey(50);
#endif

#ifdef DEBUG_SPEED
    Toc(1, "edge detection"); // edge detection
    Tic(2); // preprocessing
#endif

    float M_3_2_PI = M_PI + M_1_2_PI;
    // For each edge points, compute the edge direction
    for (int i = 0; i < szIm_.height; ++i) {
        float* _o = EO_.ptr<float>(i);
        uchar* _e = E.ptr<uchar>(i);
        uchar* _dp = DP.ptr<uchar>(i);
        uchar* _dn = DN.ptr<uchar>(i);

        for (int j = 0; j < szIm_.width; ++j) {
            if (!(_e[j] <= 0)) // !!!!!!
            {
                if (_o[j] == 0 || _o[j] == M_1_2_PI || _o[j] == M_PI || _o[j] == M_3_2_PI) {
                    _dn[j] = (uchar)255; _dp[j] = (uchar)255;
                }
                else if ((_o[j] > 0 && _o[j] < M_1_2_PI) || (_o[j] > M_PI && _o[j] < M_3_2_PI))  _dn[j] = (uchar)255;
                else _dp[j] = (uchar)255;
            }
        }
    }

#ifdef DEBUG_PREPROCESSING
    imshow("PreProcessing->DP", DP); waitKey(50);
    imshow("PreProcessing->DN", DN); waitKey();
#endif
#ifdef DEBUG_SPEED
    Toc(2, "preprocessing"); // preprocessing
#endif
}

void EllipseDetector::DetectEdges13(Mat1b& DP, VVP& points_1, VVP& points_3)
{
    // Vector of connected edge points
    VVP contours;
    int countedges = 0;
    // Labeling 8-connected edge points, discarding edge too small
    _tag_find_contours(DP, contours, iMinEdgeLength_); // put small arc edges to a vector

#ifdef DEBUG_PREPROCESSING
    Mat1b DP_show = DP.clone();
    _tag_show_contours(DP_show, contours, "PreProcessing->Contours13"); waitKey();
#endif


    // For each edge
    for (int i = 0; i < contours.size(); ++i)
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
        sort(edgeSegment.begin(), edgeSegment.end(), _SortTopLeft2BottomRight);
        int iEdgeSegmentSize = int(edgeSegment.size());

        // Get extrema of the arc
        Point& left = edgeSegment[0];
        Point& right = edgeSegment[iEdgeSegmentSize - 1];

#ifndef DISCARD_TCN
#ifndef DISCARD_TCN2
        int flag = 0;
        for (int j = 0; j<iEdgeSegmentSize; j++) {
            Point& mid = edgeSegment[j];
            float data[] = { left.x, left.y, 1, mid.x, mid.y, 1, right.x, right.y, 1 };
            Mat threePoints(Size(3, 3), CV_32FC1, data);
            double ans = determinant(threePoints);

            float dx = 1.0f*(left.x - right.x);
            float dy = 1.0f*(left.y - right.y);
            float edgelength2 = dx*dx + dy*dy;
            // double TCNl = ans/edgelength2;
            double TCNl = ans / (2 * sqrt(edgelength2));
            if (abs(TCNl) > fT_TCN_L_) {
                flag = 1;
                break;
            }
        }
        if (0 == flag) {
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
        double TCNl = ans / (2 * pow(edgelength2, fT_TCN_P_));
        if (abs(TCNl) < fT_TCN_L_) {
            countedges++;
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
        {	// 1
            points_1.push_back(edgeSegment);
        }
        else if (iCountBottom < iCountTop)
        {	// 3
            points_3.push_back(edgeSegment);
        }
    }

}

void EllipseDetector::DetectEdges24(Mat1b& DN, VVP& points_2, VVP& points_4)
{
    // Vector of connected edge points
    VVP contours;
    int countedges = 0;
    /// Labeling 8-connected edge points, discarding edge too small
    _tag_find_contours(DN, contours, iMinEdgeLength_);

#ifdef DEBUG_PREPROCESSING
    _tag_show_contours(DN, contours, "PreProcessing->Contours24"); waitKey();
#endif

    int iContoursSize = unsigned(contours.size());


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
        sort(edgeSegment.begin(), edgeSegment.end(), _SortBottomLeft2TopRight);
        int iEdgeSegmentSize = unsigned(edgeSegment.size());

        // Get extrema of the arc
        Point& left = edgeSegment[0];
        Point& right = edgeSegment[iEdgeSegmentSize - 1];

#ifndef DISCARD_TCN
#ifndef DISCARD_TCN2
        int flag = 0;
        for (int j = 0; j<iEdgeSegmentSize; j++) {
            Point& mid = edgeSegment[j];
            float data[] = { left.x, left.y, 1, mid.x, mid.y, 1, right.x, right.y, 1 };
            Mat threePoints(Size(3, 3), CV_32FC1, data);
            double ans = determinant(threePoints);

            float dx = 1.0f*(left.x - right.x);
            float dy = 1.0f*(left.y - right.y);
            float edgelength2 = dx*dx + dy*dy;
            // double TCNl = ans/edgelength2;
            double TCNl = ans / (2 * sqrt(edgelength2));
            if (abs(TCNl) > fT_TCN_L_) {
                flag = 1;
                break;
            }
        }
        if (0 == flag) {
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
        double TCNl = ans / (2 * pow(edgelength2, fT_TCN_P_));
        if (abs(TCNl) < fT_TCN_L_) {
            countedges++;
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
            // 2
            points_2.push_back(edgeSegment);
        }
        else if (iCountBottom < iCountTop)
        {
            // 4
            points_4.push_back(edgeSegment);
        }
    }

}

float inline ed2(const cv::Point& A, const cv::Point& B)
{
    return float(((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y)));
}


#define T124 pjf,pjm,pjl,pif,pim,pil // origin
#define T231 pil,pim,pif,pjf,pjm,pjl
#define T342 pif,pim,pil,pjf,pjm,pjl
#define T413 pif,pim,pil,pjl,pjm,pjf


// Verify triplets of arcs with convexity: i=1, j=2, k=4
void EllipseDetector::Triplets124(VVP& pi,
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
            vector<Ellipse> ellipses_i;

            VP& edge_j = pj[j];
            ushort sz_ej = ushort(edge_j.size());

            Point& pjf = edge_j[0];
            Point& pjm = edge_j[sz_ej / 2];
            Point& pjl = edge_j[sz_ej - 1];

#ifndef DISCARD_CONSTRAINT_POSITION

            //if (sqrt((pjl.x - pif.x)*(pjl.x - pif.x) + (pjl.y - pif.y)*(pjl.y - pif.y)) > MAX(edge_i.size(), edge_j.size()))
            //    continue;
            double tm1 = _tic();
            // CONSTRAINTS on position
            if (pjl.x > pif.x + fThrArcPosition_) //is right
                continue;

#endif

            tm1 = _tic();
            if (_ed2(pil, pjf) / _ed2(pif, pjl) < fThre_r_)
                continue;

#ifdef CONSTRAINT_CNC_1
            tm1 = _tic();
            // cnc constraint1  2se se1//pil,pim,pif,pjf,pjm,pjl pjf,pjm,pjl,pif,pim,pil
            if (fabs(_value4SixPoints(T124) - 1) > fT_CNC_)
                continue;
#endif

            tm1 = _tic();
            EllipseData data_ij;
            uint key_ij = GenerateKey(PAIR_12, i, j);
            // If the data for the pair i-j have not been computed yet
            if (data.count(key_ij) == 0)
            {
                // 1,2 -> reverse 1, swap
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

            // for each edge k
            for (ushort k = 0; k < sz_k; ++k)
            {
                VP& edge_k = pk[k];
                ushort sz_ek = ushort(edge_k.size());

                Point& pkf = edge_k[0];
                Point& pkm = edge_k[sz_ek / 2];
                Point& pkl = edge_k[sz_ek - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
                // CONSTRAINTS on position
                if (pkl.y < pil.y - fThrArcPosition_)
                    continue;
#endif
#ifdef CONSTRAINT_CNC_2
                // cnc constraint2
                if (fabs(_value4SixPoints(pif, pim, pil, pkf, pkm, pkl) - 1) > fT_CNC_)
                    continue;
#endif
#ifdef CONSTRAINT_CNC_3
                // cnc constraint3
                if (fabs(_value4SixPoints(pjf, pjm, pjl, pkf, pkm, pkl) - 1) > fT_CNC_)
                    continue;
#endif

                uint key_ik = GenerateKey(PAIR_14, i, k);

                // Find centers
                EllipseData data_ik;

                // If the data for the pair i-k have not been computed yet
                if (data.count(key_ik) == 0)
                {
                    // 1,4 -> ok
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
                    // discard
                    continue;
                }
#endif

                // If all constraints of the selection strategy have been satisfied,
                // we can start estimating the ellipse parameters

                // Find ellipse parameters
                // Get the coordinates of the center (xc, yc)
                Point2f center = GetCenterCoordinates(data_ij, data_ik);

                Ellipse ell;
                // Find remaining paramters (A,B,rho)
                FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ell);
                ellipses_i.push_back(ell);
            }
            /*-----------------------------------------------------------------*/
            int rt = -1;
            float rs = 0;
            for (int t = 0; t < (int)ellipses_i.size(); t++)
            {
                if (ellipses_i[t].score_ > rs)
                {
                    rs = ellipses_i[t].score_; rt = t;
                }
            }
            if (rt > -1)
            {
                ellipses.push_back(ellipses_i[rt]);
            }
            /*-----------------------------------------------------------------*/
        }
    }
}

void EllipseDetector::Triplets231(VVP& pi,
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
            vector<Ellipse> ellipses_i;

            VP& edge_j = pj[j];
            ushort sz_ej = ushort(edge_j.size());

            Point& pjf = edge_j[0];
            Point& pjm = edge_j[sz_ej / 2];
            Point& pjl = edge_j[sz_ej - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
            //if (sqrt((pjf.x - pif.x)*(pjf.x - pif.x) + (pjf.y - pif.y)*(pjf.y - pif.y)) > MAX(edge_i.size(), edge_j.size()))
            //    continue;

            double tm1 = _tic();
            // CONSTRAINTS on position
            if (pjf.y < pif.y - fThrArcPosition_)
                continue;

#endif

            tm1 = _tic();
            if (_ed2(pif, pjf) / _ed2(pil, pjl) < fThre_r_)
                continue;

#ifdef CONSTRAINT_CNC_1
            tm1 = _tic();
            // cnc constraint1 2es se3 // pif,pim,pil,pjf,pjm,pjl pil,pim,pif,pjf,pjm,pjl
            if (fabs(_value4SixPoints(T231) - 1) > fT_CNC_)
                continue;
#endif

            tm1 = _tic();
            VP rev_j(edge_j.size());
            reverse_copy(edge_j.begin(), edge_j.end(), rev_j.begin());

            EllipseData data_ij;
            uint key_ij = GenerateKey(PAIR_23, i, j);
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

            // For each edge k
            for (ushort k = 0; k < sz_k; ++k)
            {
                VP& edge_k = pk[k];

                ushort sz_ek = ushort(edge_k.size());

                Point& pkf = edge_k[0];
                Point& pkm = edge_k[sz_ek / 2];
                Point& pkl = edge_k[sz_ek - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
                // CONSTRAINTS on position
                if (pkf.x < pil.x - fThrArcPosition_)
                    continue;
#endif

#ifdef CONSTRAINT_CNC_2
                // cnc constraint2
                if (fabs(_value4SixPoints(pif, pim, pil, pkf, pkm, pkl) - 1) > fT_CNC_)
                    continue;
#endif
#ifdef CONSTRAINT_CNC_3
                // cnc constraint3
                if (fabs(_value4SixPoints(pjf, pjm, pjl, pkf, pkm, pkl) - 1) > fT_CNC_)
                    continue;
#endif
                uint key_ik = GenerateKey(PAIR_12, k, i);

                // Find centers

                EllipseData data_ik;


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
                    // discard
                    continue;
                }
#endif
                // Find ellipse parameters
                Point2f center = GetCenterCoordinates(data_ij, data_ik);

                Ellipse ell;
                FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ell);
                ellipses_i.push_back(ell);
            }
            /*-----------------------------------------------------------------*/
            int rt = -1;
            float rs = 0;
            for (int t = 0; t < (int)ellipses_i.size(); t++)
            {
                if (ellipses_i[t].score_ > rs)
                {
                    rs = ellipses_i[t].score_; rt = t;
                }
            }
            if (rt > -1)
            {
                ellipses.push_back(ellipses_i[rt]);
            }
            /*-----------------------------------------------------------------*/
        }
    }
}

void EllipseDetector::Triplets342(VVP& pi,
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
            vector<Ellipse> ellipses_i;

            VP& edge_j = pj[j];
            ushort sz_ej = ushort(edge_j.size());

            Point& pjf = edge_j[0];
            Point& pjm = edge_j[sz_ej / 2];
            Point& pjl = edge_j[sz_ej - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
            //if (sqrt((pjf.x - pil.x)*(pjf.x - pil.x) + (pjf.y - pil.y)*(pjf.y - pil.y)) > MAX(edge_i.size(), edge_j.size()))
            //    continue;

            double tm1 = _tic();
            // CONSTRAINTS on position
            if (pjf.x < pil.x - fThrArcPosition_) 		// is left
                continue;

#endif

            tm1 = _tic();
            if (_ed2(pil, pjf) / _ed2(pif, pjl) < fThre_r_)
                continue;

#ifdef CONSTRAINT_CNC_1
            tm1 = _tic();
            // cnc constraint1 3se se4 // pil,pim,pif,pjf,pjm,pjl pif,pim,pil,pjf,pjm,pjl
            if (fabs(_value4SixPoints(T342) - 1) > fT_CNC_)
                continue;

#endif

            tm1 = _tic();
            VP rev_j(edge_j.size());
            reverse_copy(edge_j.begin(), edge_j.end(), rev_j.begin());

            EllipseData data_ij;
            uint key_ij = GenerateKey(PAIR_34, i, j);

            if (data.count(key_ij) == 0)
            {
                // 3,4 -> reverse 4

                GetFastCenter(edge_i, rev_j, data_ij);
                data.insert(pair<uint, EllipseData>(key_ij, data_ij));
            }
            else
            {
                data_ij = data.at(key_ij);
            }


            // For each edge k
            for (ushort k = 0; k < sz_k; ++k)
            {
                VP& edge_k = pk[k];
                ushort sz_ek = ushort(edge_k.size());

                Point& pkf = edge_k[0];
                Point& pkm = edge_k[sz_ek / 2];
                Point& pkl = edge_k[sz_ek - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
                // CONSTRAINTS on position
                if (pkf.y > pif.y + fThrArcPosition_)
                    continue;
#endif

#ifdef CONSTRAINT_CNC_2
                // cnc constraint2
                if (fabs(_value4SixPoints(pif, pim, pil, pkf, pkm, pkl) - 1) > fT_CNC_)
                    continue;
#endif
#ifdef CONSTRAINT_CNC_3
                // cnc constraint3
                if (fabs(_value4SixPoints(pjf, pjm, pjl, pkf, pkm, pkl) - 1) > fT_CNC_)
                    continue;
#endif
                uint key_ik = GenerateKey(PAIR_23, k, i);

                // Find centers

                EllipseData data_ik;



                if (data.count(key_ik) == 0)
                {
                    // 3,2 -> reverse 3,2

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
                    // discard
                    continue;
                }
#endif
                // Find ellipse parameters
                Point2f center = GetCenterCoordinates(data_ij, data_ik);

                Ellipse ell;
                FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ell);
                ellipses_i.push_back(ell);
            }
            /*-----------------------------------------------------------------*/
            int rt = -1;
            float rs = 0;
            for (int t = 0; t < (int)ellipses_i.size(); t++)
            {
                if (ellipses_i[t].score_ > rs)
                {
                    rs = ellipses_i[t].score_; rt = t;
                }
            }
            if (rt > -1)
            {
                ellipses.push_back(ellipses_i[rt]);
            }
            /*-----------------------------------------------------------------*/
        }
    }
}

void EllipseDetector::Triplets413(VVP& pi,
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
            vector<Ellipse> ellipses_i;

            VP& edge_j = pj[j];
            ushort sz_ej = ushort(edge_j.size());

            Point& pjf = edge_j[0];
            Point& pjm = edge_j[sz_ej / 2];
            Point& pjl = edge_j[sz_ej - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
            //if (sqrt((pjl.x - pil.x)*(pjl.x - pil.x) + (pjl.y - pil.y)*(pjl.y - pil.y)) > MAX(edge_i.size(), edge_j.size()))
            //    continue;

            double tm1 = _tic();
            // CONSTRAINTS on position
            if (pjl.y > pil.y + fThrArcPosition_)  		// is below
                continue;

#endif

            tm1 = _tic();
            if (_ed2(pif, pjf) / _ed2(pil, pjl) < fThre_r_)
                continue;

#ifdef CONSTRAINT_CNC_1
            tm1 = _tic();
            // cnc constraint1 4se es1 // pif,pim,pil,pjf,pjm,pjl pil,pim,pif,pjl,pjm,pjf pif,pim,pil,pjl,pjm,pjf
            if (fabs(_value4SixPoints(T413) - 1) > fT_CNC_)
                continue;
#endif

            tm1 = _tic();
            EllipseData data_ij;
            uint key_ij = GenerateKey(PAIR_14, j, i);

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

            // For each edge k
            for (ushort k = 0; k < sz_k; ++k)
            {
                VP& edge_k = pk[k];
                ushort sz_ek = ushort(edge_k.size());

                Point& pkf = edge_k[0];
                Point& pkm = edge_k[sz_ek / 2];
                Point& pkl = edge_k[sz_ek - 1];

#ifndef DISCARD_CONSTRAINT_POSITION
                // CONSTRAINTS on position
                if (pkl.x > pif.x + fThrArcPosition_)
                    continue;
#endif

#ifdef CONSTRAINT_CNC_2
                // cnc constraint2
                if (fabs(_value4SixPoints(pif, pim, pil, pkf, pkm, pkl) - 1) > fT_CNC_)
                    continue;
#endif
#ifdef CONSTRAINT_CNC_3
                // cnc constraint2
                if (fabs(_value4SixPoints(pjf, pjm, pjl, pkf, pkm, pkl) - 1) > fT_CNC_)
                    continue;
#endif
                uint key_ik = GenerateKey(PAIR_34, k, i);

                // Find centers

                EllipseData data_ik;



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
                    // discard
                    continue;
                }
#endif
                // Find ellipse parameters
                Point2f center = GetCenterCoordinates(data_ij, data_ik);

                Ellipse ell;
                FindEllipses(center, edge_i, edge_j, edge_k, data_ij, data_ik, ell);
                ellipses_i.push_back(ell);
            }
            /*-----------------------------------------------------------------*/
            int rt = -1;
            float rs = 0;
            for (int t = 0; t < (int)ellipses_i.size(); t++)
            {
                if (ellipses_i[t].score_ > rs)
                {
                    rs = ellipses_i[t].score_; rt = t;
                }
            }
            if (rt > -1)
            {
                ellipses.push_back(ellipses_i[rt]);
            }
            /*-----------------------------------------------------------------*/
        }
    }
}

int EllipseDetector::FindMaxK(const int* v) const
{
    int max_val = 0;
    int max_idx = 0;
    for (int i = 0; i<ACC_R_SIZE; ++i)
    {
        (v[i] > max_val) ? max_val = v[i], max_idx = i : (int)NULL;
    }

    return max_idx + 90;
}

int EllipseDetector::FindMaxN(const int* v) const
{
    int max_val = 0;
    int max_idx = 0;
    for (int i = 0; i<ACC_N_SIZE; ++i)
    {
        (v[i] > max_val) ? max_val = v[i], max_idx = i : (int)NULL;
    }

    return max_idx;
}

int EllipseDetector::FindMaxA(const int* v) const
{
    int max_val = 0;
    int max_idx = 0;
    for (int i = 0; i<ACC_A_SIZE; ++i)
    {
        (v[i] > max_val) ? max_val = v[i], max_idx = i : (int)NULL;
    }

    return max_idx;
}

// Most important function for detecting ellipses. See Sect[3.2.3] of the paper
void EllipseDetector::FindEllipses(Point2f& center,
    VP& edge_i, VP& edge_j, VP& edge_k,
    EllipseData& data_ij, EllipseData& data_ik, Ellipse& ell)
{
    countOfFindEllipse_++;
    // Find ellipse parameters

    // 0-initialize accumulators
    memset(accN, 0, sizeof(int)*ACC_N_SIZE);
    memset(accR, 0, sizeof(int)*ACC_R_SIZE);
    memset(accA, 0, sizeof(int)*ACC_A_SIZE);

    // estimation
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
    // ij1 and ik
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

                float a = (q1xq2 - q3xq4); // gama
                float b = (q3xq4 + 1)*(q1 + q2) - (q1xq2 + 1)*(q3 + q4); // beta
                float Kp = (-b + sqrt(b*b + 4 * a*a)) / (2 * a);         // K+
                float zplus = ((q1 - Kp)*(q2 - Kp)) / ((1 + q1*Kp)*(1 + q2*Kp));
                // check  zplus and K is linear
                if (zplus >= 0.0f) continue;

                float Np = sqrt(-zplus); // N+
                float rho = atan(Kp);    // rho tmp
                int rhoDeg;
                if (Np > 1.f)
                {
                    Np = 1.f / Np;
                    rhoDeg = cvRound((rho * 180 / CV_PI) + 180) % 180; // [0,180)
                }
                else
                {
                    rhoDeg = cvRound((rho * 180 / CV_PI) + 90) % 180; // [0,180) rho angel rep and norm
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

    // ij2 and ik
    {
        float q1 = data_ij.rb;
        float q3 = data_ik.rb;
        float q5 = data_ik.ra;

        for (int ij2 = 0; ij2 < sz_ij2; ++ij2)
        {
            float q2 = data_ij.Sb[ij2];

            float q1xq2 = q1*q2;
            // ij2 and ik2
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
                    rhoDeg = cvRound((rho * 180 / CV_PI) + 90) % 180;  // [0,180)
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

            // ij2 and ik1
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
    float rho = fK * float(CV_PI) / 180.f;	// deg 2 rad
    float Kp = tan(rho);

    // Estimate A. See Eq. [19 - 22] in Sect [3.2.3] of the paper
    //
    // may optm
    for (ushort l = 0; l < sz_ei; ++l)
    {
        Point& pp = edge_i[l];
        float sk = 1.f / sqrt(Kp*Kp + 1.f); // cos rho
        float x0 = ((pp.x - a0) * sk) + (((pp.y - b0)*Kp) * sk);    // may optm
        float y0 = -(((pp.x - a0) * Kp) * sk) + ((pp.y - b0) * sk); // may optm
        float Ax = sqrt((x0*x0*Np*Np + y0*y0) / ((Np*Np)*(1.f + Kp*Kp)));
        int A = cvRound(abs(Ax / cos(rho))); // may optm
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
    // Ellipse ell(a0, b0, fA, fB, fmod(rho + float(CV_PI)*2.f, float(CV_PI)));
    ell.xc_ = a0;
    ell.yc_ = b0;
    ell.a_ = fA;
    ell.b_ = fB;
    ell.rad_ = fmod(rho + float(CV_PI)*2.f, float(CV_PI));

    // estimation end
    // validation start
    // Get the score. See Sect [3.3.1] in the paper

    // Find the number of edge pixel lying on the ellipse
    float _cos = cos(-ell.rad_);
    float _sin = sin(-ell.rad_);

    float invA2 = 1.f / (ell.a_ * ell.a_);
    float invB2 = 1.f / (ell.b_ * ell.b_);

    float invNofPoints = 1.f / float(sz_ei + sz_ej + sz_ek);
    int counter_on_perimeter = 0;
    float probc_on_perimeter = 0;

    for (ushort l = 0; l < sz_ei; ++l)
    {
        float tx = float(edge_i[l].x) - ell.xc_;
        float ty = float(edge_i[l].y) - ell.yc_;
        float rx = (tx*_cos - ty*_sin);
        float ry = (tx*_sin + ty*_cos);

        float h = (rx*rx)*invA2 + (ry*ry)*invB2;
        if (abs(h - 1.f) < fDistanceToEllipseContour_)
        {
            ++counter_on_perimeter;
            probc_on_perimeter += 1; // (fDistanceToEllipseContour_ - abs(h - 1.f)) / fDistanceToEllipseContour_;
        }
    }

    for (ushort l = 0; l < sz_ej; ++l)
    {
        float tx = float(edge_j[l].x) - ell.xc_;
        float ty = float(edge_j[l].y) - ell.yc_;
        float rx = (tx*_cos - ty*_sin);
        float ry = (tx*_sin + ty*_cos);

        float h = (rx*rx)*invA2 + (ry*ry)*invB2;
        if (abs(h - 1.f) < fDistanceToEllipseContour_)
        {
            ++counter_on_perimeter;
            probc_on_perimeter += 1; // (fDistanceToEllipseContour_ - abs(h - 1.f)) / fDistanceToEllipseContour_;
        }
    }

    for (ushort l = 0; l < sz_ek; ++l)
    {
        float tx = float(edge_k[l].x) - ell.xc_;
        float ty = float(edge_k[l].y) - ell.yc_;
        float rx = (tx*_cos - ty*_sin);
        float ry = (tx*_sin + ty*_cos);

        float h = (rx*rx)*invA2 + (ry*ry)*invB2;
        if (abs(h - 1.f) < fDistanceToEllipseContour_)
        {
            ++counter_on_perimeter;
            probc_on_perimeter += 1; // (fDistanceToEllipseContour_ - abs(h - 1.f)) / fDistanceToEllipseContour_;
        }
    }


    // no points found on the ellipse
    if (counter_on_perimeter <= 0)
    {
        // validation
        return;
    }

    // Compute score
    // float score = float(counter_on_perimeter) * invNofPoints;
    float score = probc_on_perimeter * invNofPoints;
    if (score < fMinScore_)
    {
        // validation
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
        p1.x -= ell.xc_;
        p1.y -= ell.yc_;
        p2.x -= ell.xc_;
        p2.y -= ell.yc_;
        Point2f r1((p1.x*_cos - p1.y*_sin), (p1.x*_sin + p1.y*_cos));
        Point2f r2((p2.x*_cos - p2.y*_sin), (p2.x*_sin + p2.y*_cos));
        di = abs(r2.x - r1.x) + abs(r2.y - r1.y);
    }
    {
        Point2f p1(float(edge_j[0].x), float(edge_j[0].y));
        Point2f p2(float(edge_j[sz_ej - 1].x), float(edge_j[sz_ej - 1].y));
        p1.x -= ell.xc_;
        p1.y -= ell.yc_;
        p2.x -= ell.xc_;
        p2.y -= ell.yc_;
        Point2f r1((p1.x*_cos - p1.y*_sin), (p1.x*_sin + p1.y*_cos));
        Point2f r2((p2.x*_cos - p2.y*_sin), (p2.x*_sin + p2.y*_cos));
        dj = abs(r2.x - r1.x) + abs(r2.y - r1.y);
    }
    {
        Point2f p1(float(edge_k[0].x), float(edge_k[0].y));
        Point2f p2(float(edge_k[sz_ek - 1].x), float(edge_k[sz_ek - 1].y));
        p1.x -= ell.xc_;
        p1.y -= ell.yc_;
        p2.x -= ell.xc_;
        p2.y -= ell.yc_;
        Point2f r1((p1.x*_cos - p1.y*_sin), (p1.x*_sin + p1.y*_cos));
        Point2f r2((p2.x*_cos - p2.y*_sin), (p2.x*_sin + p2.y*_cos));
        dk = abs(r2.x - r1.x) + abs(r2.y - r1.y);
    }

    // This allows to get rid of thick edges
    float rel = min(1.f, ((di + dj + dk) / (3 * (ell.a_ + ell.b_))));

    if (rel < fMinReliability_)
    {
        // validation
        return;
    }
    if (_isnan(rel))
        return;

    // Assign the new score!
    ell.score_ = (score + rel) * 0.5f; // need to change
                                       // ell.score_ = (score*rel); // need to change

    if (ell.score_ < fMinScore_)
    {
        return;
    }
    // The tentative detection has been confirmed. Save it!
    // ellipses.push_back(ell);

    // Validation end
}



// Ellipse clustering procedure. See Sect [3.3.2] in the paper.
void EllipseDetector::ClusterEllipses(vector<Ellipse>& ellipses)
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

        float ba_e1 = e1.b_ / e1.a_;
        // float Decc1 = e1._b / e1._a;

        bool bFoundCluster = false;
        for (int j = 0; j<sz_clusters; ++j)
        {
            Ellipse& e2 = clusters[j];

            float ba_e2 = e2.b_ / e2.a_;
            float th_Dc = min(e1.b_, e2.b_) * th_Dc_ratio;
            th_Dc *= th_Dc;

            // Centers
            float Dc = ((e1.xc_ - e2.xc_)*(e1.xc_ - e2.xc_) + (e1.yc_ - e2.yc_)*(e1.yc_ - e2.yc_));
            if (Dc > th_Dc)
            {
                //not same cluster
                continue;
            }

            // a
            float Da = abs(e1.a_ - e2.a_) / max(e1.a_, e2.a_);
            if (Da > th_Da)
            {
                //not same cluster
                continue;
            }

            // b
            float Db = abs(e1.b_ - e2.b_) / min(e1.b_, e2.b_);
            if (Db > th_Db)
            {
                //not same cluster
                continue;
            }

            // angle
            float Dr = GetMinAnglePI(e1.rad_, e2.rad_) / float(CV_PI);
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
}

float EllipseDetector::GetMedianSlope(vector<Point2f>& med, Point2f& M, vector<float>& slopes)
{
    // input med slopes, output M, return slope
    // med		: vector of points
    // M		: centroid of the points in med
    // slopes	: vector of the slopes

    unsigned iNofPoints = unsigned(med.size());
    // CV_Assert(iNofPoints >= 2);

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

int inline sgn(float val) {
    return (0.f < val) - (val < 0.f);
}

void EllipseDetector::GetFastCenter(vector<Point>& e1, vector<Point>& e2, EllipseData& data)
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

        /****************************************
        Mat3b out(480, 640, Vec3b(0, 0, 0));
        Vec3b color(0, 255, 0);
        for (int ci = 0; ci < med.size(); ci++)
        circle(out, med[ci], 2, color);
        imshow("test", out); waitKey(100);
        ****************************************/
        q2 = GetMedianSlope(med, M12, data.Sa); //get Sa ta = q2 Ma
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

float EllipseDetector::GetMinAnglePI(float alpha, float beta)
{
    float pi = float(CV_PI);
    float pi2 = float(2.0 * CV_PI);

    // normalize data in [0, 2*pi]
    float a = fmod(alpha + pi2, pi2);
    float b = fmod(beta + pi2, pi2);

    // normalize data in [0, pi]
    if (a > pi)
        a -= pi;
    if (b > pi)
        b -= pi;

    if (a > b)
    {
        swap(a, b);
    }

    float diff1 = b - a;
    float diff2 = pi - diff1;
    return min(diff1, diff2);
}

// Get the coordinates of the center, given the intersection of the estimated lines. See Fig. [8] in Sect [3.2.3] in the paper.
Point2f EllipseDetector::GetCenterCoordinates(EllipseData& data_ij, EllipseData& data_ik)
{
    float xx[7];
    float yy[7];

    xx[0] = data_ij.Cab.x;
    xx[1] = data_ik.Cab.x;
    yy[0] = data_ij.Cab.y;
    yy[1] = data_ik.Cab.y;

    {
        // 1-1
        float q2 = data_ij.ta;
        float q4 = data_ik.ta;
        Point2f& M12 = data_ij.Ma;
        Point2f& M34 = data_ik.Ma;

        float invDen = 1 / (q2 - q4);
        xx[2] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
        yy[2] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
    }

    {
        // 1-2
        float q2 = data_ij.ta;
        float q4 = data_ik.tb;
        Point2f& M12 = data_ij.Ma;
        Point2f& M34 = data_ik.Mb;

        float invDen = 1 / (q2 - q4);
        xx[3] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
        yy[3] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
    }

    {
        // 2-2
        float q2 = data_ij.tb;
        float q4 = data_ik.tb;
        Point2f& M12 = data_ij.Mb;
        Point2f& M34 = data_ik.Mb;

        float invDen = 1 / (q2 - q4);
        xx[4] = (M34.y - q4*M34.x - M12.y + q2*M12.x) * invDen;
        yy[4] = (q2*M34.y - q4*M12.y + q2*q4*(M12.x - M34.x)) * invDen;
    }

    {
        // 2-1
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
}

uint inline EllipseDetector::GenerateKey(uchar pair, ushort u, ushort v)
{
    return (pair << 30) + (u << 15) + v;
}

// Draw at most iTopN detected ellipses.
void EllipseDetector::DrawDetectedEllipses(Mat& output, vector<Ellipse>& ellipses, int iTopN, int thickness)
{
    int sz_ell = int(ellipses.size());
    int n = (iTopN == 0) ? sz_ell : min(iTopN, sz_ell);
    for (int i = 0; i < n; ++i)
    {
        Ellipse& e = ellipses[n - i - 1];
        int g = cvRound(e.score_ * 255.f);
        Scalar color(0, g, 0);
        ellipse(output, Point(cvRound(e.xc_), cvRound(e.yc_)), Size(cvRound(e.a_), cvRound(e.b_)), e.rad_*180.0 / CV_PI, 0.0, 360.0, color, thickness);
        // cv::circle(output, Point(e.xc_, e.yc_), 2, Scalar(0, 0, 255), 2);
    }
}

bool inline convex_check(VP& vp1, int start, int end)
{
    int x_min(4096), x_max(0), y_min(4096), y_max(0);
    int integral_u(0), integral_d(0);
    for (int i = start; i <= end; i++)
    {
        Point& val = vp1[i];
        x_min = MIN(x_min, val.x);
        x_max = MAX(x_max, val.x);
        y_min = MIN(y_min, val.y);
        y_max = MAX(y_max, val.y);
    }
    for (int i = start; i <= end; i++)
    {
        Point& val = vp1[i];
        integral_u += (val.y - y_min);
        integral_d += (y_max - val.y);
    }
    if (integral_u > integral_d)
        return false;
    else
        return true;
}

bool inline concave_check(VP& vp1, int start, int end)
{
    int x_min(4096), x_max(0), y_min(4096), y_max(0);
    int integral_u(0), integral_d(0);
    for (int i = start; i <= end; i++)
    {
        Point& val = vp1[i];
        x_min = MIN(x_min, val.x);
        x_max = MAX(x_max, val.x);
        y_min = MIN(y_min, val.y);
        y_max = MAX(y_max, val.y);
    }
    for (int i = start; i <= end; i++)
    {
        Point& val = vp1[i];
        integral_u += (val.y - y_min);
        integral_d += (y_max - val.y);
    }
    if (integral_u < integral_d)
        return false;
    else
        return true;
}

void EllipseDetector::ArcsCheck1234(VVP& points_1, VVP& points_2, VVP& points_3, VVP& points_4)
{
    static int nchecks = 21;
    int i, j;
    VVP vps_1, vps_2, vps_3, vps_4;
    Mat3b out(480, 640, Vec3b(0, 0, 0));
    for (i = 0; i < points_2.size(); ++i)
    {
        // Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
        Vec3b color(255, 0, 0);
        for (j = 0; j < points_2[i].size(); ++j)
            out(points_2[i][j]) = color;
    }
    imshow("out", out); waitKey();

    for (i = 0; i < points_1.size(); i++)
    {
        VP& vp1 = points_1[i];
        if (vp1.size() > nchecks)
        {
            VP vpn;
            for (j = 0; j <= vp1.size() - nchecks; j++)
            {
                if (convex_check(vp1, j, j + nchecks - 1))
                {
                    vpn.push_back(vp1[j]);
                }
                else
                {
                    vps_1.push_back(vpn); vpn.clear();
                }
            }
            vps_1.push_back(vpn);
        }
        else
        {
            cout << "==== small arc I ====" << endl;
        }
    }
    for (i = 0; i < points_2.size(); i++)
    {
        VP& vp1 = points_2[i];
        if (vp1.size() > nchecks)
        {
            VP vpn;
            for (j = 0; j <= vp1.size() - nchecks; j++)
            {
                if (concave_check(vp1, j, j + nchecks - 1))
                {
                    vpn.push_back(vp1[j]);
                }
                else
                {
                    vps_2.push_back(vpn); vpn.clear();
                }
            }
            vps_2.push_back(vpn);
        }
        else
        {
            cout << "==== small arc II ====" << endl;
        }
    }
    Mat3b out2(480, 640, Vec3b(0, 0, 0));
    for (i = 0; i < vps_2.size(); ++i)
    {
        // Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
        Vec3b color(255, 0, 0);
        for (j = 0; j < vps_2[i].size(); ++j)
            out2(vps_2[i][j]) = color;
    }
    imshow("out2", out2); waitKey();
    for (i = 0; i < points_3.size(); i++)
    {
        VP& vp1 = points_3[i];
        if (vp1.size() > nchecks)
        {
            VP vpn;
            for (j = 0; j <= vp1.size() - nchecks; j++)
            {
                if (concave_check(vp1, j, j + nchecks - 1))
                {
                    vpn.push_back(vp1[j]);
                }
                else
                {
                    vps_3.push_back(vpn); vpn.clear();
                }
            }
            vps_3.push_back(vpn);
        }
        else
        {
            cout << "==== small arc III ====" << endl;
        }
    }
    for (i = 0; i < points_4.size(); i++)
    {
        VP& vp1 = points_4[i];
        if (vp1.size() > nchecks)
        {
            VP vpn;
            for (j = 0; j <= vp1.size() - nchecks; j++)
            {
                if (convex_check(vp1, j, j + nchecks - 1))
                {
                    vpn.push_back(vp1[j]);
                }
                else
                {
                    vps_4.push_back(vpn); vpn.clear();
                }
            }
            vps_4.push_back(vpn);
        }
        else
        {
            cout << "==== small arc IV ====" << endl;
        }
    }
    points_1 = vps_1; points_2 = vps_2; points_3 = vps_3; points_4 = vps_4;
}
