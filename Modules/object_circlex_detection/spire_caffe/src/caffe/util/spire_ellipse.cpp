#ifdef USE_OPENCV
#include <caffe/util/spire_ellipse.h>

using namespace std;
using namespace cv;

namespace spire {

void cvCanny2(const void* srcarr, void* dstarr,
	double low_thresh, double high_thresh,
	void* dxarr, void* dyarr,
	int aperture_size)
{
	//cv::Ptr<CvMat> dx, dy;
	cv::AutoBuffer<char> buffer;
	std::vector<uchar*> stack;
	uchar **stack_top = 0, **stack_bottom = 0;

	CvMat srcstub, *src = cvGetMat(srcarr, &srcstub);
	CvMat dststub, *dst = cvGetMat(dstarr, &dststub);

	CvMat dxstub, *dx = cvGetMat(dxarr, &dxstub);
	CvMat dystub, *dy = cvGetMat(dyarr, &dystub);


	CvSize size;
	int flags = aperture_size;
	int low, high;
	int* mag_buf[3];
	uchar* map;
	ptrdiff_t mapstep;
	int maxsize;
	int i, j;
	CvMat mag_row;

	if (CV_MAT_TYPE(src->type) != CV_8UC1 ||
		CV_MAT_TYPE(dst->type) != CV_8UC1 ||
		CV_MAT_TYPE(dx->type) != CV_16SC1 ||
		CV_MAT_TYPE(dy->type) != CV_16SC1)
		CV_Error(CV_StsUnsupportedFormat, "");

	if (!CV_ARE_SIZES_EQ(src, dst))
		CV_Error(CV_StsUnmatchedSizes, "");

	if (low_thresh > high_thresh)
	{
		double t;
		CV_SWAP(low_thresh, high_thresh, t);
	}

	aperture_size &= INT_MAX;
	if ((aperture_size & 1) == 0 || aperture_size < 3 || aperture_size > 7)
		CV_Error(CV_StsBadFlag, "");

	size.width = src->cols;
	size.height = src->rows;

	//size = cvGetMatSize( src );

	//dx = cvCreateMat( size.height, size.width, CV_16SC1 );
	//dy = cvCreateMat( size.height, size.width, CV_16SC1 );

	//aperture_size = -1; //SCHARR
	cvSobel(src, dx, 1, 0, aperture_size);
	cvSobel(src, dy, 0, 1, aperture_size);

	//Mat ddx(dx,true);
	//Mat ddy(dy,true);


	if (flags & CV_CANNY_L2_GRADIENT)
	{
		Cv32suf ul, uh;
		ul.f = (float)low_thresh;
		uh.f = (float)high_thresh;

		low = ul.i;
		high = uh.i;
	}
	else
	{
		low = cvFloor(low_thresh);
		high = cvFloor(high_thresh);
	}

	buffer.allocate((size.width + 2)*(size.height + 2) + (size.width + 2) * 3 * sizeof(int));

	mag_buf[0] = (int*)(char*)buffer;
	mag_buf[1] = mag_buf[0] + size.width + 2;
	mag_buf[2] = mag_buf[1] + size.width + 2;
	map = (uchar*)(mag_buf[2] + size.width + 2);
	mapstep = size.width + 2;

	maxsize = MAX(1 << 10, size.width*size.height / 10);
	stack.resize(maxsize);
	stack_top = stack_bottom = &stack[0];

	memset(mag_buf[0], 0, (size.width + 2)*sizeof(int));
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

	mag_row = cvMat(1, size.width, CV_32F);

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

			if (!(flags & CV_CANNY_L2_GRADIENT))
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
			memset(_mag - 1, 0, (size.width + 2)*sizeof(int));

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

		for (j = 0; j < size.width; j++)
		{
#define CANNY_SHIFT 15
#define TG22  (int)(0.4142135623730950488016887242097*(1<<CANNY_SHIFT) + 0.5)

			x = _dx[j];
			y = _dy[j];
			int s = x ^ y;
			int m = _mag[j];

			x = abs(x);
			y = abs(y);
			if (m > low)
			{
				int tg22x = x * TG22;
				int tg67x = tg22x + ((x + x) << CANNY_SHIFT);

				y <<= CANNY_SHIFT;

				if (y < tg22x)
				{
					if (m > _mag[j - 1] && m >= _mag[j + 1])
					{
						if (m > high && !prev_flag && _map[j - mapstep] != 2)
						{
							CANNY_PUSH(_map + j);
							prev_flag = 1;
						}
						else
							_map[j] = (uchar)0;
						continue;
					}
				}
				else if (y > tg67x)
				{
					if (m > _mag[j + magstep2] && m >= _mag[j + magstep1])
					{
						if (m > high && !prev_flag && _map[j - mapstep] != 2)
						{
							CANNY_PUSH(_map + j);
							prev_flag = 1;
						}
						else
							_map[j] = (uchar)0;
						continue;
					}
				}
				else
				{
					s = s < 0 ? -1 : 1;
					if (m > _mag[j + magstep2 - s] && m > _mag[j + magstep1 + s])
					{
						if (m > high && !prev_flag && _map[j - mapstep] != 2)
						{
							CANNY_PUSH(_map + j);
							prev_flag = 1;
						}
						else
							_map[j] = (uchar)0;
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
	for (i = 0; i < size.height; i++)
	{
		const uchar* _map = map + mapstep*(i + 1) + 1;
		uchar* _dst = dst->data.ptr + dst->step*i;

		for (j = 0; j < size.width; j++)
		{
			_dst[j] = (uchar)-(_map[j] >> 1);
		}
	}
};

void Canny2(InputArray image, OutputArray _edges,
	OutputArray _sobel_x, OutputArray _sobel_y,
	double threshold1, double threshold2,
	int apertureSize, bool L2gradient)
{
	Mat src = image.getMat();
	_edges.create(src.size(), CV_8U);
	_sobel_x.create(src.size(), CV_16S);
	_sobel_y.create(src.size(), CV_16S);


	CvMat c_src = src, c_dst = _edges.getMat();
	CvMat c_dx = _sobel_x.getMat();
	CvMat c_dy = _sobel_y.getMat();


	cvCanny2(&c_src, &c_dst, threshold1, threshold2,
		&c_dx, &c_dy,
		apertureSize + (L2gradient ? CV_CANNY_L2_GRADIENT : 0));
};

void show_contours(cv::Mat1b& image, VVP& segments, const char* title) {
	Mat3b contoursIm(image.rows, image.cols, Vec3b(0, 0, 0));
	for (size_t i = 0; i<segments.size(); ++i)
	{
		Vec3b color(rand()%255, 128+rand()%127, 128+rand()%127);
		for (size_t j = 0; j<segments[i].size(); ++j)
			contoursIm(segments[i][j]) = color;
	}
	imshow(title, contoursIm);
}

void find_contours(cv::Mat1b& image, VVP& segments, int iMinLength) {
#define RG_STACK_SIZE 2048

	// Uso stack globali per velocizzare l'elaborazione (anche a scapito della memoria occupata)
	int stack2[RG_STACK_SIZE];
#define RG_PUSH2(a) (stack2[sp2] = (a) , sp2++)
#define RG_POP2(a) (sp2-- , (a) = stack2[sp2])

	// Uso stack globali per velocizzare l'elaborazione (anche a scapito della memoria occupata)
	Point stack3[RG_STACK_SIZE];
#define RG_PUSH3(a) (stack3[sp3] = (a) , sp3++)
#define RG_POP3(a) (sp3-- , (a) = stack3[sp3])

    int i, w, h; // iDim;
	int x, y;
	int x2, y2;
	int sp2; // stack pointer
	int sp3;

	Mat_<uchar> src = image.clone();
	w = src.cols;
	h = src.rows;
    // iDim = w*h;

	Point point;
	for (y = 0; y<h; ++y)
	{
		for (x = 0; x<w; ++x)
		{
			if ((src(y, x)) != 0)   //punto non etichettato: seme trovato
			{
				// per ogni oggetto
				sp2 = 0;
				i = x + y*w;
				RG_PUSH2(i);

				// vuoto la lista dei punti
				sp3 = 0;
				while (sp2>0)
				{// rg tradizionale
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

					// Inserisco i nuovi punti nello stack solo se esistono
					// e sono punti da etichettare

					// 4 connessi
					// sx
					if (x2>0 && (src(y2, x2 - 1) != 0))
						RG_PUSH2(i - 1);
					// sotto
					if (y2>0 && (src(y2 - 1, x2) != 0))
						RG_PUSH2(i - w);
					// sopra
					if (y2<h - 1 && (src(y2 + 1, x2) != 0))
						RG_PUSH2(i + w);
					// dx
					if (x2<w - 1 && (src(y2, x2 + 1) != 0))
						RG_PUSH2(i + 1);

					// 8 connessi
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

					// etichetto il punto
					for (i = 0; i<sp3; i++){
						// etichetto
						component.push_back(stack3[i]);
					}
					segments.push_back(component);
				}
			}
		}
	}
}


void find_contours_rect(Mat1b& image, VVP& segments, int iMinLength, vector<Rect>& bboxes)
{

#define _RG_STACK_SIZE 10000

	// Uso stack globali per velocizzare l'elaborazione (anche a scapito della memoria occupata)
	int stack2[_RG_STACK_SIZE];
#define _RG_PUSH2(a) (stack2[sp2] = (a) , sp2++)
#define _RG_POP2(a) (sp2-- , (a) = stack2[sp2])

	// Uso stack globali per velocizzare l'elaborazione (anche a scapito della memoria occupata)
	Point stack3[_RG_STACK_SIZE];
#define _RG_PUSH3(a) (stack3[sp3] = (a) , sp3++)
#define _RG_POP3(a) (sp3-- , (a) = stack3[sp3])

    int i, w, h; // iDim;
	int x, y;
	int x2, y2;
	int sp2; /* stack pointer */
	int sp3;

	Mat_<uchar> src = image.clone();
	w = src.cols;
	h = src.rows;
    // iDim = w*h;

	Point point;
	for (y = 0; y<h; y++)
	{
		for (x = 0; x<w; x++)
		{
			if ((src(y, x)) != 0)   //punto non etichettato: seme trovato
			{
				// per ogni oggetto	
				sp2 = 0;
				i = x + y*w;
				_RG_PUSH2(i);

				// vuoto la lista dei punti
				sp3 = 0;
				while (sp2>0)
				{// rg tradizionale

					_RG_POP2(i);
					x2 = i%w;
					y2 = i / w;

					src(y2, x2) = 0;

					point.x = x2;
					point.y = y2;
					_RG_PUSH3(point);

					// Inserisco i nuovi punti nello stack solo se esistono
					// e sono punti da etichettare

					// 4 connessi
					// sx
					if (x2>0 && (src(y2, x2 - 1) != 0))
						_RG_PUSH2(i - 1);
					// sotto
					if (y2>0 && (src(y2 - 1, x2) != 0))
						_RG_PUSH2(i - w);
					// sopra
					if (y2<h - 1 && (src(y2 + 1, x2) != 0))
						_RG_PUSH2(i + w);
					// dx
					if (x2<w - 1 && (src(y2, x2 + 1) != 0))
						_RG_PUSH2(i + 1);

					// 8 connessi
					if (x2>0 && y2>0 && (src(y2 - 1, x2 - 1) != 0))
						_RG_PUSH2(i - w - 1);
					if (x2>0 && y2<h - 1 && (src(y2 + 1, x2 - 1) != 0))
						_RG_PUSH2(i + w - 1);
					if (x2<w - 1 && y2>0 && (src(y2 - 1, x2 + 1) != 0))
						_RG_PUSH2(i - w + 1);
					if (x2<w - 1 && y2<h - 1 && (src(y2 + 1, x2 + 1) != 0))
						_RG_PUSH2(i + w + 1);

				}

				if (sp3 >= iMinLength)
				{
					vector<Point> component;

					int iMinx, iMaxx, iMiny, iMaxy;
					iMinx = iMaxx = stack3[0].x;
					iMiny = iMaxy = stack3[0].y;

					// etichetto il punto
					for (i = 0; i<sp3; i++){
						point = stack3[i];
						// etichetto
						component.push_back(point);

						if (iMinx > point.x)  iMinx = point.x;
						if (iMiny > point.y)  iMiny = point.y;
						if (iMaxx < point.x)  iMaxx = point.x;
						if (iMaxy < point.y)  iMaxy = point.y;
					}

					bboxes.push_back(Rect(Point(iMinx, iMiny), Point(iMaxx + 1, iMaxy + 1)));
					segments.push_back(component);

				}
			}
		}
	}
}



// Thinning Zhang e Suen 
void thinning(Mat1b& imgMask, uchar byF, uchar byB)
{
	int r = imgMask.rows;
	int c = imgMask.cols;

	Mat_<uchar> imgIT(r, c), imgM(r, c);

	for (int i = 0; i<r; ++i)
	{
		for (int j = 0; j<c; ++j)
		{
			imgIT(i, j) = imgMask(i, j) == byF ? 1 : 0;
		}
	}

	bool bSomethingDone = true;
    // int iCount = 0;

	while (bSomethingDone) {
		bSomethingDone = false;
		fill(imgM.begin(), imgM.end(), 0);

		//prima iterazione
		for (int y = 1; y<r - 2; y++) {
			for (int x = 1; x<c - 2; x++) {

#define c_P0 imgIT(y-1,x-1)==1
#define c_P1 imgIT(y-1,x)==1
#define c_P2 imgIT(y-1,x+1)==1
#define c_P3 imgIT(y-1,x+2)==1
#define c_P4 imgIT(y,x-1)==1
#define c_P5 imgIT(y,x)==1
#define c_P6 imgIT(y,x+1)==1
#define c_P7 imgIT(y,x+2)==1
#define c_P8 imgIT(y+1,x-1)==1
#define c_P9 imgIT(y+1,x)==1
#define c_P10 imgIT(y+1,x+1)==1
#define c_P11 imgIT(y+1,x+2)==1
#define c_P12 imgIT(y+2,x-1)==1
#define c_P13 imgIT(y+2,x)==1
#define c_P14 imgIT(y+2,x+1)==1
#define c_P15 imgIT(y+2,x+2)==1

				if (c_P5) {
					if (c_P9) {
						if (c_P6) {
							if (c_P10) {
								if (c_P4) {
									if (c_P8) {
										if (c_P1) {
											continue;
										}
										else {
											if (c_P13) {
												if (c_P2) {
													if (c_P0) {
														continue;
													}
													else {
														goto a_2;
													}
												}
												else {
													goto a_2;
												}
											}
											else {
												if (c_P14) {
													if (c_P12) {
														if (c_P2) {
															if (c_P0) {
																continue;
															}
															else {
																goto a_2;
															}
														}
														else {
															goto a_2;
														}
													}
													else {
														continue;
													}
												}
												else {
													continue;
												}
											}
										}
									}
									else {
										continue;
									}
								}
								else {
									if (c_P1) {
										if (c_P2) {
											if (c_P7) {
												if (c_P8) {
													if (c_P0) {
														continue;
													}
													else {
														goto a_2;
													}
												}
												else {
													goto a_2;
												}
											}
											else {
												if (c_P11) {
													if (c_P3) {
														if (c_P8) {
															if (c_P0) {
																continue;
															}
															else {
																goto a_2;
															}
														}
														else {
															goto a_2;
														}
													}
													else {
														continue;
													}
												}
												else {
													continue;
												}
											}
										}
										else {
											continue;
										}
									}
									else {
										if (c_P0) {
											continue;
										}
										else {
											if (c_P8) {
												goto a_2;
											}
											else {
												if (c_P2) {
													goto a_2;
												}
												else {
													if (c_P14) {
														if (c_P13) {
															if (c_P11) {
																goto a_2;
															}
															else {
																if (c_P7) {
																	goto a_2;
																}
																else {
																	if (c_P3) {
																		goto a_2;
																	}
																	else {
																		continue;
																	}
																}
															}
														}
														else {
															goto a_2;
														}
													}
													else {
														if (c_P13) {
															goto a_2;
														}
														else {
															if (c_P12) {
																goto a_2;
															}
															else {
																if (c_P11) {
																	if (c_P7) {
																		continue;
																	}
																	else {
																		goto a_2;
																	}
																}
																else {
																	if (c_P15) {
																		goto a_2;
																	}
																	else {
																		if (c_P7) {
																			goto a_2;
																		}
																		else {
																			if (c_P3) {
																				goto a_2;
																			}
																			else {
																				continue;
																			}
																		}
																	}
																}
															}
														}
													}
												}
											}
										}
									}
								}
							}
							else {
								continue;
							}
						}
						else {
							if (c_P0) {
								if (c_P8) {
									if (c_P4) {
										if (c_P2) {
											if (c_P10) {
												continue;
											}
											else {
												if (c_P1) {
													goto a_2;
												}
												else {
													continue;
												}
											}
										}
										else {
											goto a_2;
										}
									}
									else {
										continue;
									}
								}
								else {
									continue;
								}
							}
							else {
								if (c_P2) {
									continue;
								}
								else {
									if (c_P1) {
										continue;
									}
									else {
										if (c_P8) {
											goto a_2;
										}
										else {
											if (c_P10) {
												if (c_P4) {
													continue;
												}
												else {
													goto a_2;
												}
											}
											else {
												continue;
											}
										}
									}
								}
							}
						}
					}
					else {
						if (c_P6) {
							if (c_P0) {
								if (c_P2) {
									if (c_P1) {
										if (c_P8) {
											if (c_P10) {
												continue;
											}
											else {
												if (c_P4) {
													goto a_2;
												}
												else {
													continue;
												}
											}
										}
										else {
											goto a_2;
										}
									}
									else {
										continue;
									}
								}
								else {
									continue;
								}
							}
							else {
								if (c_P8) {
									continue;
								}
								else {
									if (c_P4) {
										continue;
									}
									else {
										if (c_P2) {
											goto a_2;
										}
										else {
											if (c_P10) {
												if (c_P1) {
													continue;
												}
												else {
													goto a_2;
												}
											}
											else {
												continue;
											}
										}
									}
								}
							}
						}
						else {
							if (c_P10) {
								continue;
							}
							else {
								if (c_P4) {
									if (c_P1) {
										if (c_P0) {
											goto a_2;
										}
										else {
											continue;
										}
									}
									else {
										if (c_P2) {
											continue;
										}
										else {
											if (c_P8) {
												goto a_2;
											}
											else {
												if (c_P0) {
													goto a_2;
												}
												else {
													continue;
												}
											}
										}
									}
								}
								else {
									if (c_P8) {
										continue;
									}
									else {
										if (c_P1) {
											if (c_P2) {
												goto a_2;
											}
											else {
												if (c_P0) {
													goto a_2;
												}
												else {
													continue;
												}
											}
										}
										else {
											continue;
										}
									}
								}
							}
						}
					}
				}
				else {
					continue;
				}


			a_2:
				imgM(y, x) = 1;
				bSomethingDone = true;
			}
		}

		for (int r = 0; r<imgIT.rows; ++r) {
			for (int c = 0; c<imgIT.cols; ++c) {
				if (imgM(r, c) == 1)
					imgIT(r, c) = 0;
			}
		}
	}

	for (int i = 0; i<r; ++i)
	{
		for (int j = 0; j<c; ++j)
		{
			imgMask(i, j) = imgIT(i, j) == 1 ? byF : byB;
		}
	}
};



bool SortBottomLeft2TopRight(const Point& lhs, const Point& rhs)
{
	if (lhs.x == rhs.x)
	{
		return lhs.y > rhs.y;
	}
	return lhs.x < rhs.x;
};

bool SortBottomLeft2TopRight2f(const Point2f& lhs, const Point2f& rhs)
{
	if (lhs.x == rhs.x)
	{
		return lhs.y > rhs.y;
	}
	return lhs.x < rhs.x;
};


bool SortTopLeft2BottomRight(const Point& lhs, const Point& rhs)
{
	if (lhs.x == rhs.x)
	{
		return lhs.y < rhs.y;
	}
	return lhs.x < rhs.x;
};


void cvCanny3(const void* srcarr, void* dstarr,
	void* dxarr, void* dyarr,
	int aperture_size, double percent_ne)
{
	//cv::Ptr<CvMat> dx, dy;
	cv::AutoBuffer<char> buffer;
	std::vector<uchar*> stack;
	uchar **stack_top = 0, **stack_bottom = 0;

	CvMat srcstub, *src = cvGetMat(srcarr, &srcstub); // IplImage to cvMat
	CvMat dststub, *dst = cvGetMat(dstarr, &dststub);

	CvMat dxstub, *dx = cvGetMat(dxarr, &dxstub);
	CvMat dystub, *dy = cvGetMat(dyarr, &dystub);


	CvSize size;
	int flags = aperture_size;
	int low, high;
	int* mag_buf[3];
	uchar* map;
	ptrdiff_t mapstep;
	int maxsize;
	int i, j;
	CvMat mag_row;

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


	size.width = src->cols;
	size.height = src->rows;
	// size = cvGetMatSize( src );


	//dx = cvCreateMat( size.height, size.width, CV_16SC1 );
	//dy = cvCreateMat( size.height, size.width, CV_16SC1 );

	//aperture_size = -1; //SCHARR
	cvSobel(src, dx, 1, 0, aperture_size);
	cvSobel(src, dy, 0, 1, aperture_size);

	//const Mat sobel_x(dx);	//Mat_<unsigned short>
	//const Mat sobel_y(dy);

	//% Calculate Magnitude of Gradient
	//magGrad = hypot(dx, dy);

	Mat1f magGrad(size.height, size.width, 0.f);
	float maxGrad(0);
	float val(0);
	for (i = 0; i<size.height; ++i)
	{
		float* _pmag = magGrad.ptr<float>(i);
		const short* _dx = (short*)(dx->data.ptr + dx->step*i);
		const short* _dy = (short*)(dy->data.ptr + dy->step*i);
		for (j = 0; j<size.width; ++j)
		{
			val = float(abs(_dx[j]) + abs(_dy[j]));
			_pmag[j] = val;
			maxGrad = (val > maxGrad) ? val : maxGrad;
		}
	}

	//% Normalize for threshold selection
	//normalize(magGrad, magGrad, 0.0, 1.0, NORM_MINMAX);

	//% Determine Hysteresis Thresholds

	//set magic numbers
	const int NUM_BINS = 64;
	const double percent_of_pixels_not_edges = percent_ne;
	const double threshold_ratio = 0.3;

	//compute histogram
	int bin_size = cvFloor(maxGrad / float(NUM_BINS) + 0.5f) + 1;
	if (bin_size < 1) bin_size = 1;
	int bins[NUM_BINS] = { 0 };
	for (i = 0; i<size.height; ++i)
	{
		float *_pmag = magGrad.ptr<float>(i);
		for (j = 0; j<size.width; ++j)
		{
            // int hgf = int(_pmag[j]);
			bins[int(_pmag[j]) / bin_size]++;
		}
	}


	//% Select the thresholds
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

	if (flags & CV_CANNY_L2_GRADIENT)
	{
		Cv32suf ul, uh;
		ul.f = (float)low_thresh;
		uh.f = (float)high_thresh;

		low = ul.i;
		high = uh.i;
	}
	else
	{
		low = cvFloor(low_thresh);
		high = cvFloor(high_thresh);
	}


	buffer.allocate((size.width + 2)*(size.height + 2) + (size.width + 2) * 3 * sizeof(int));
	mag_buf[0] = (int*)(char*)buffer;
	mag_buf[1] = mag_buf[0] + size.width + 2;
	mag_buf[2] = mag_buf[1] + size.width + 2;
	map = (uchar*)(mag_buf[2] + size.width + 2);
	mapstep = size.width + 2;

	maxsize = MAX(1 << 10, size.width*size.height / 10);
	stack.resize(maxsize);
	stack_top = stack_bottom = &stack[0];

	memset(mag_buf[0], 0, (size.width + 2)*sizeof(int));
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

	mag_row = cvMat(1, size.width, CV_32F);

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

			if (!(flags & CV_CANNY_L2_GRADIENT))
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
			memset(_mag - 1, 0, (size.width + 2)*sizeof(int));

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

		for (j = 0; j < size.width; j++)
		{
#define CANNY_SHIFT 15
#define TG22  (int)(0.4142135623730950488016887242097*(1<<CANNY_SHIFT) + 0.5)

			x = _dx[j];
			y = _dy[j];
			int s = x ^ y;
			int m = _mag[j];

			x = abs(x);
			y = abs(y);
			if (m > low)
			{
				int tg22x = x * TG22;
				int tg67x = tg22x + ((x + x) << CANNY_SHIFT);

				y <<= CANNY_SHIFT;

				if (y < tg22x)
				{
					if (m > _mag[j - 1] && m >= _mag[j + 1])
					{
						if (m > high && !prev_flag && _map[j - mapstep] != 2)
						{
							CANNY_PUSH(_map + j);
							prev_flag = 1;
						}
						else
							_map[j] = (uchar)0;
						continue;
					}
				}
				else if (y > tg67x)
				{
					if (m > _mag[j + magstep2] && m >= _mag[j + magstep1])
					{
						if (m > high && !prev_flag && _map[j - mapstep] != 2)
						{
							CANNY_PUSH(_map + j);
							prev_flag = 1;
						}
						else
							_map[j] = (uchar)0;
						continue;
					}
				}
				else
				{
					s = s < 0 ? -1 : 1;
					if (m > _mag[j + magstep2 - s] && m > _mag[j + magstep1 + s])
					{
						if (m > high && !prev_flag && _map[j - mapstep] != 2)
						{
							CANNY_PUSH(_map + j);
							prev_flag = 1;
						}
						else
							_map[j] = (uchar)0;
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
	for (i = 0; i < size.height; i++)
	{
		const uchar* _map = map + mapstep*(i + 1) + 1;
		uchar* _dst = dst->data.ptr + dst->step*i;

		for (j = 0; j < size.width; j++)
		{
			_dst[j] = (uchar)-(_map[j] >> 1);
		}
	}
};

void Canny3(InputArray image, OutputArray _edges,
	OutputArray _sobel_x, OutputArray _sobel_y,
	int apertureSize, bool L2gradient)
{
	Mat src = image.getMat();
	_edges.create(src.size(), CV_8U);
	_sobel_x.create(src.size(), CV_16S);
	_sobel_y.create(src.size(), CV_16S);


	CvMat c_src = src, c_dst = _edges.getMat();
	CvMat c_dx = _sobel_x.getMat();
	CvMat c_dy = _sobel_y.getMat();


	cvCanny3(&c_src, &c_dst,
		&c_dx, &c_dy,
		apertureSize + (L2gradient ? CV_CANNY_L2_GRADIENT : 0), 0.9);
};

void Canny4(InputArray image, OutputArray _edges,
	OutputArray _sobel_x, OutputArray _sobel_y,
	int apertureSize, bool L2gradient)
{
	Mat src = image.getMat();
	_edges.create(src.size(), CV_8U);
	_sobel_x.create(src.size(), CV_16S);
	_sobel_y.create(src.size(), CV_16S);


	CvMat c_src = src, c_dst = _edges.getMat();
	CvMat c_dx = _sobel_x.getMat();
	CvMat c_dy = _sobel_y.getMat();


	cvCanny3(&c_src, &c_dst,
		&c_dx, &c_dy,
		apertureSize + (L2gradient ? CV_CANNY_L2_GRADIENT : 0), 0.5);
};

float GetMinAnglePI(float alpha, float beta)
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

Point2f lineCrossPoint(Point2f l1p1, Point2f l1p2, Point2f l2p1, Point2f l2p2)
{
	Point2f crossPoint;
	float k1, k2, b1, b2;
	if (l1p1.x == l1p2.x&&l2p1.x == l2p2.x){
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

void point2Mat(Point2f p1, Point2f p2, float mat[2][2])
{
	mat[0][0] = p1.x;
	mat[0][1] = p1.y;
	mat[1][0] = p2.x;
	mat[1][1] = p2.y;
}

float value4SixPoints(V2SP)
{
	float result = 1;
	Mat A, B, C;
	float matB[2][2], matC[2][2];
	Point2f v, w, u;
	v = lineCrossPoint(p1, p2, p3, p4);
	w = lineCrossPoint(p5, p6, p3, p4);
	u = lineCrossPoint(p5, p6, p1, p2);

	point2Mat(u, v, matB);
	point2Mat(p1, p2, matC);
	B = Mat(2, 2, CV_32F, matB);
	C = Mat(2, 2, CV_32F, matC);
	A = C*B.inv();
	/*
	cout<<"u:\t"<<u<<endl;
	cout<<"v:\t"<<v<<endl;
	cout<<"B:\t"<<B<<endl;
	cout<<A<<endl;
	*/
	result *= A.at<float>(0, 0)*A.at<float>(1, 0) / (A.at<float>(0, 1)*A.at<float>(1, 1));

	point2Mat(p3, p4, matC);
	point2Mat(v, w, matB);
	B = Mat(2, 2, CV_32F, matB);
	C = Mat(2, 2, CV_32F, matC);
	A = C*B.inv();
	result *= A.at<float>(0, 0)*A.at<float>(1, 0) / (A.at<float>(0, 1)*A.at<float>(1, 1));

	point2Mat(p5, p6, matC);
	point2Mat(w, u, matB);
	B = Mat(2, 2, CV_32F, matB);
	C = Mat(2, 2, CV_32F, matC);
	A = C*B.inv();
	result *= A.at<float>(0, 0)*A.at<float>(1, 0) / (A.at<float>(0, 1)*A.at<float>(1, 1));
	return result;
}

}

#endif // USE_OPENCV
