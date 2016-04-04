#include <jni.h>
#include <android/log.h>
#include <android/bitmap.h>
#include <android/native_window_jni.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d/features2d.hpp>

#include <algorithm>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

using namespace std;
using namespace cv;

#ifndef int64_t
#define int64_t long long
#endif

#ifndef uint8_t
#define uint8_t unsigned char
#endif

#ifndef int32_t
#define int32_t int
#endif

#ifndef uint32_t
#define uint32_t unsigned int
#endif

#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, "myrobot", __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "myrobot", __VA_ARGS__)

#define DEBUG 1
#define IMAGE_FORMAT_YV12 842094169

// Round x up to a multiple of mask.
// E.g., ALIGN(x, 16) means round x up to the nearest multiple of 16.
#define ALIGN(x, mask) (((x) + (mask)-1) & ~((mask)-1))

typedef struct b_line_
{
    float m_alpha;
    int m_dis;
} b_line;

bool checkBufferSizesMatch(int srcWidth, int srcHeight,
        const ANativeWindow_Buffer *buf) {
    return (srcWidth == buf->width && srcHeight == buf->height);
}

#define PI 3.14159

int calc_distance(int width, int height, Point p0, Point p1, float & alpha ) {
    int x0;
    int y0;
    int x1;
    int y1;
    int x_org;
    int y_org;
    float a;
    float b;
//    float alpha;
    x_org = width/2;
    y_org = height;

    x0 = p0.x - x_org;
    y0 = y_org - p0.y;

    x1 = p1.x - x_org;
    y1 = y_org - p1.y;

    int d;

    if( (x1 != x0) && (x1 != 0) ) {
        a = (float) (y1 - y0);
        a = a / ((float) (x1 - x0));
        b = y0 - a * (float) x0;
        alpha = atan(a);
        d = (int)((b/a) * sin(alpha));

        alpha *= 180/PI;
    }
    else {
        alpha = 90;
        d = x1;
    }
    d = abs(d);
    //LOGE("bob alpha:%f distance:%d",alpha, d);
    return d;
}

void LaneDetect(Mat & img_rgba, const char * str, int saveFile) {

    Mat img_hsv;
    Mat img_3;
    Mat img_dis;
    FILE *fp = NULL;

//    cvtColor(img_rgba, img_gray, COLOR_RGBA2GRAY);
//    cvtColor(img_gray, img_3, COLOR_GRAY2RGB);
//    cv::cvtColor(img_3, img_rgba , CV_RGB2RGBA);
    cv::Mat img_gray(img_rgba.size(), CV_8UC1);
    cv::cvtColor(img_rgba, img_hsv, CV_RGB2HSV);

    if (saveFile) {
        char fn[100];
        sprintf(fn, "%s/hsv.txt", str);
        fp = fopen(fn, "w");
    }

    int h_max;
    int h_min;
    int s_max;
    int s_min;
    int v_max;
    int v_min;

    uchar *buf;
    buf = img_hsv.data;

//    h_max = 100;    h_min = 40;    s_max = 180;    s_min = 30;    v_max = 255;    v_min = 160;
    h_max = 100;    h_min = 50;    s_max = 240;    s_min = 20;    v_max = 255;    v_min = 110;

    if( fp ) {
        h_max = 100;    h_min = 50;    s_max = 250;    s_min = 10;    v_max = 255;    v_min = 50;
    }

    int i;
    int j;
    for (i = 0; i < img_hsv.rows; i++) {
        for (j = 0; j < img_hsv.cols; j++) {
            uchar *b;
            b = buf + i * img_hsv.cols * 3 + j * 3;
            if (((b[0] <= h_max) && (b[0] >= h_min) && ((b[1] <= s_max) && (b[1] >= s_min)) && ((b[2] <= v_max) && (b[2] >= v_min)))) {
                if (fp) {
                    fprintf(fp, "(%03d,%03d,%03d) ", b[0], b[1], b[2]);
                }
                img_gray.at<uchar>(i, j) = 255;
            }
            else {
                if (fp) {
                    fprintf(fp, "(---,---,---) ");
                }
                img_gray.at<uchar>(i, j) = 0;
            }
        }
        if (fp) {
            fprintf(fp, "\n");
        }
    }
    if (fp) {
        fclose(fp);
    }
    LOGE("bob 11111");

//    inRange(img_hsv, Scalar(0, 0, 20), Scalar(80, 100, 255), img_gray);
//    inRange(img_hsv, Scalar(0, 0, 20), Scalar(140, 150, 255), img_gray);
//    inRange(img_hsv, Scalar(20, 0, 70), Scalar(40, 50, 200), img_gray);

//    inRange(img_hsv, Scalar(0, 0, 3), Scalar(180, 50, 255), img_gray);

//    inRange(img_hsv, Scalar(10, 4, 150), Scalar(170, 180, 255), img_gray);
//    inRange(img_hsv, Scalar(10, 0, 170), Scalar(70, 50, 255), img_gray);

    // HSV: 45. 77, 91
//    inRange(img_hsv, Scalar(10,150,170), Scalar(25, 255, 255),img_gray);

//    inRange(img_hsv, Scalar(65,50,220), Scalar(70, 160, 255),img_gray);

    img_dis = img_gray;
    cvtColor(img_dis, img_3, COLOR_GRAY2RGB);
    cv::cvtColor(img_3, img_rgba , CV_RGB2RGBA);

    cv::blur(img_gray, img_gray, Size(15, 15));
    threshold(img_gray, img_gray, 100, 255, CV_THRESH_BINARY);
    LOGE("bob 22222222222");
#if 1
    Mat img_contours;
    Canny(img_gray,img_contours,50,250);

    //Mat img_contoursInv;
//    threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

    vector<Vec4i> lines;
//    HoughLinesP( img_contours, lines, 1, CV_PI/180, 80, 30, 10 );
//    HoughLinesP( img_contours, lines, 1, CV_PI/180, 40, 0, 30 );
    HoughLinesP( img_contours, lines, 1, CV_PI/180, 40, 100, 30 );

    LOGE("bob lines count:%d", lines.size());
    float alpha;
    int width;
    int height;
    int distance;

    b_line m_lines[10];
    int line_count;
    width = img_rgba.cols;
    height = img_rgba.rows;
    line_count = 0;
    for( i=0;i<10;i++) {
        m_lines[i].m_alpha = 0.0;
        m_lines[i].m_dis = -1;
    }
    for( i = 0; i < lines.size(); i++ )
    {
        line( img_rgba, Point(lines[i][0], lines[i][1]),
              Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );

        distance = calc_distance(width, height, Point(lines[i][0],lines[i][1]), Point(lines[i][2], lines[i][3]), alpha);
        if( m_lines[0].m_dis == -1 )
        {
            m_lines[0].m_dis = distance;
            m_lines[0].m_alpha = alpha;
            line_count = 1;
        }
        else
        {
            int j;
            int findit = 0;
            for( j=0; j<line_count; j++ )
            {
                if( (m_lines[j].m_alpha < alpha + 5 ) && (m_lines[j].m_alpha > alpha - 5) )
                {
                    if( m_lines[j].m_dis > distance )
                    {
                        m_lines[j].m_dis = distance;
                        m_lines[j].m_alpha = alpha;
                    }
                    findit = 1;
                    break;
                }
                else {

                }
            }
            if( !findit ) {
                m_lines[line_count].m_alpha = alpha;
                m_lines[line_count].m_dis = distance;
                line_count ++;
            }

        }

        LOGE("bob lines dis (%d,%d) - (%d,%d) alpha:%f distance:%d\n", lines[i][0], lines[i][1], lines[i][2], lines[i][3], alpha, distance);
    }
    LOGE("bob lines ------\n");
    for (i = 0; i<line_count; i++) {
        LOGE("bob lines result:%d alpha:%f dis:%d \n", i, m_lines[i].m_alpha, m_lines[i].m_dis);
    }
    LOGE("bob line detect done");
#endif
//    erode(img_gray, img_gray, kernel_ero);
}

void BallDetect(Mat & img_rgba, int &x, int &y, int &r, Mat & outLines) {
    int width;
    int height;
    width = img_rgba.cols;
    height = img_rgba.rows;
    Mat hsv_frame;

    cv::Mat* thresholded = new cv::Mat(cv::Size(width, height), CV_8UC1);

    cv::Rect roi( cv::Point( 0, 0 ), Size(width, height) );
    CvSize size = cvSize(width, height);

    cvtColor(img_rgba, hsv_frame, CV_RGB2HSV);
    inRange(hsv_frame, Scalar(1, 80, 80), Scalar(7, 250, 250), *thresholded);

    GaussianBlur(*thresholded, *thresholded, Size(9,9), 0, 0);
    vector<Vec3f> circles;
    HoughCircles(*thresholded, circles, CV_HOUGH_GRADIENT,1.5, height/4, 100, 40, 15, 80 );

    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        if( radius < 5 )
            continue;
        // draw the circle center
        circle( img_rgba, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // draw the circle outline
        circle( img_rgba, center, radius, Scalar(0,0,255), 3, 8, 0 );
        x = cvRound(circles[i][0]);
        y = cvRound(circles[i][1]);
        r = radius;
        break;
    }

    delete thresholded;

}

extern "C" {
JNIEXPORT bool JNICALL Java_com_neza_myrobot_JNIUtils_blit(
    JNIEnv *env, jobject obj, jint srcWidth, jint srcHeight,
    jobject srcLumaByteBuffer, jint srcLumaRowStrideBytes,
    jobject srcChromaUByteBuffer, jobject srcChromaVByteBuffer,
    jint srcChromaElementStrideBytes, jint srcChromaRowStrideBytes,
    jobject dstSurface, jstring path, jint saveFile, jlong outPtr) {

    Mat* mLinesOut = (Mat*) outPtr;

    const char *str = env->GetStringUTFChars(path,NULL);
    LOGE("bob path:%s saveFile=%d", str, saveFile);

    if (srcChromaElementStrideBytes != 1 && srcChromaElementStrideBytes != 2) {
		LOGE("!!!blit ERROR NOT YUV format!!!\n");
        return false;
    }

    uint8_t *srcLumaPtr = reinterpret_cast<uint8_t *>(
        env->GetDirectBufferAddress(srcLumaByteBuffer));
    uint8_t *srcChromaUPtr = reinterpret_cast<uint8_t *>(
        env->GetDirectBufferAddress(srcChromaUByteBuffer));
    uint8_t *srcChromaVPtr = reinterpret_cast<uint8_t *>(
        env->GetDirectBufferAddress(srcChromaVByteBuffer));
	
    if (srcLumaPtr == nullptr || srcChromaUPtr == nullptr || srcChromaVPtr == nullptr) {
        LOGE("blit NULL pointer ERROR");
        return false;
    }

    int dstWidth;
    int dstHeight;

//    LOGE("blit src: width=%d height=%d\n", srcWidth, srcHeight);

    cv::Mat mYuv(srcHeight + srcHeight/2,srcWidth, CV_8UC1, srcLumaPtr);

    uint8_t *srcChromaUVInterleavedPtr = nullptr;
    bool swapDstUV;
    if (srcChromaElementStrideBytes == 2) {
        if (srcChromaVPtr - srcChromaUPtr == 1) {
            srcChromaUVInterleavedPtr = srcChromaUPtr;  // UVUVUV...
            swapDstUV = false;
        } else if (srcChromaUPtr - srcChromaVPtr == 1) {
            srcChromaUVInterleavedPtr = srcChromaVPtr;  // VUVUVU...
            swapDstUV = true;
        } else {
            // stride is 2 but the pointers are not off by 1.
            return false;
        }
    }

    ANativeWindow *win = ANativeWindow_fromSurface(env, dstSurface);
    ANativeWindow_acquire(win);
	
    ANativeWindow_Buffer buf;
	
    dstWidth = srcHeight;
    dstHeight = srcWidth;
//    dstWidth = srcWidth;
//    dstHeight = srcHeight;

    ANativeWindow_setBuffersGeometry(win, dstWidth, dstHeight, 0 /*format unchanged*/);

    if (int32_t err = ANativeWindow_lock(win, &buf, NULL)) {
        LOGE("ANativeWindow_lock failed with error code %d\n", err);
        ANativeWindow_release(win);
        return false;
    }

    uint8_t *dstLumaPtr = reinterpret_cast<uint8_t *>(buf.bits);
    Mat dstRgba(dstHeight, buf.stride, CV_8UC4, dstLumaPtr);		// TextureView buffer, use stride as width
    Mat srcRgba(srcHeight, srcWidth, CV_8UC4);
    Mat flipRgba(dstHeight, dstWidth, CV_8UC4);

	// convert YUV -> RGBA
    cv::cvtColor(mYuv, srcRgba, CV_YUV2RGBA_NV21);

	// Rotate 90 degree
    cv::transpose(srcRgba, flipRgba);
    cv::flip(flipRgba, flipRgba, 1);

#if 0
    int ball_x;
    int ball_y;
    int ball_r;

    ball_r = 0;

    BallDetect(flipRgba, ball_x, ball_y, ball_r);
    if( ball_r > 0)
        LOGE("ball x:%d y:%d r:%d", ball_x, ball_y, ball_r);
    else
        LOGE("ball not detected");
#endif

    LaneDetect(flipRgba, str, saveFile, mLinesOut);

    // copy to TextureView surface
    uchar * dbuf;
    uchar * sbuf;
    dbuf = dstRgba.data;
    sbuf = flipRgba.data;
    int i;
    for(i=0;i<flipRgba.rows;i++) {
        dbuf = dstRgba.data + i * buf.stride * 4;
        memcpy(dbuf, sbuf, flipRgba.cols * 4);
        sbuf += flipRgba.cols * 4;
    }

	// Draw some rectangles
    Point p1(100,100);
    Point p2(300,300);
    cv::rectangle(dstRgba,p1,p2,Scalar(255,255,255));
    cv::rectangle(dstRgba,Point(10,10),Point(dstWidth-1,dstHeight-1),Scalar(255,255,255));
    cv::rectangle(dstRgba,Point(100,100),Point(dstWidth/2,dstWidth/2),Scalar(255,255,255));

    LOGE("bob dstWidth=%d height=%d", dstWidth, dstHeight);
    ANativeWindow_unlockAndPost(win);
    ANativeWindow_release(win);
    return 0;
}
}

extern "C" {
JNIEXPORT void JNICALL
Java_com_neza_myrobot_JNIUtils_test(JNIEnv *env, jobject assetManager) {
}
}

extern "C" {
JNIEXPORT bool JNICALL Java_com_neza_myrobot_JNIUtils_blitraw(
        JNIEnv *env, jobject obj, jint srcWidth, jint srcHeight,
        jlong srcYuv, jobject dstSurface) {


    // Check that if src chroma channels are interleaved if element stride is 2.
    // Our Halide kernel "directly deinterleaves" UVUVUVUV --> UUUU, VVVV
    // to handle VUVUVUVU, just swap the destination pointers.

    Mat mYuv = *((Mat*)srcYuv);

    //cv::Mat mYuv(srcHeight,srcWidth, CV_8UC3, srcLumaPtr);

    ANativeWindow *win = ANativeWindow_fromSurface(env, dstSurface);
    ANativeWindow_acquire(win);

    ANativeWindow_Buffer buf;

    if (int32_t err = ANativeWindow_lock(win, &buf, NULL)) {
        LOGE("ANativeWindow_lock failed with error code %d\n", err);
        ANativeWindow_release(win);
        return false;
    }
    LOGE("bob dst: width=%d height=%d\n", buf.width, buf.height);

    ANativeWindow_setBuffersGeometry(win, srcWidth, srcHeight, 0 /*format unchanged*/);
    uint8_t *dstLumaPtr = reinterpret_cast<uint8_t *>(buf.bits);
    cv::Mat mRgba(srcHeight, srcWidth, CV_8UC4, dstLumaPtr);

    cv::cvtColor(mYuv, mRgba, CV_YUV2RGBA_NV21);
//    mRgba = mYuv;
//    cv::transpose(mYuv, mRgba);

    ANativeWindow_unlockAndPost(win);
    ANativeWindow_release(win);
    return 0;
}
}
