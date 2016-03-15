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

bool checkBufferSizesMatch(int srcWidth, int srcHeight,
        const ANativeWindow_Buffer *buf) {
    return (srcWidth == buf->width && srcHeight == buf->height);
}

#define PI 3.14159

void LaneDetect(Mat & img_rgba) {
    Mat img_gray;
    Mat img_hsv;
    Mat img_3;
    int houghVote = 200;
//    cvtColor(img_rgba, img_gray, COLOR_RGBA2GRAY);
//    cvtColor(img_gray, img_3, COLOR_GRAY2RGB);
//    cv::cvtColor(img_3, img_rgba , CV_RGB2RGBA);

    cv::cvtColor(img_rgba, img_hsv, CV_RGB2HSV);
    inRange(img_hsv, Scalar(0, 0, 20), Scalar(80, 100, 255), img_gray);
//    inRange(img_hsv, Scalar(0, 0, 20), Scalar(80, 100, 255), img_gray);

    cv::blur(img_gray, img_gray, Size(15, 15));
    threshold(img_gray, img_gray, 100, 255, CV_THRESH_BINARY);
#if 1
    Mat img_contours;
    Canny(img_gray,img_contours,50,250);
    Mat img_contoursInv;
//    threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

    vector<Vec4i> lines;
    HoughLinesP( img_contours, lines, 1, CV_PI/180, 80, 30, 10 );
    LOGE("bob lines count:%d", lines.size());
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( img_rgba, Point(lines[i][0], lines[i][1]),
              Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
    }
#endif

#if 0
    std::vector<Vec2f> lines;
    if (houghVote < 1 or lines.size() > 2){ // we lost all lines. reset
        houghVote = 200;
    }
    else{ houghVote += 25;}
    while(lines.size() < 5 && houghVote > 0){
        HoughLines(contours,lines,1,PI/180, houghVote);
        houghVote -= 5;
    }

    Mat result(img_gray.size(),CV_8U,Scalar(255));
    img_gray.copyTo(result);

    // Draw the limes
    std::vector<Vec2f>::const_iterator it= lines.begin();
    Mat hough(img_gray.size(),CV_8U,Scalar(0));
    while (it!=lines.end()) {

        float rho= (*it)[0];   // first element is distance rho
        float theta= (*it)[1]; // second element is angle theta

        if ( theta > 0.09 && theta < 1.48 || theta < 3.14 && theta > 1.66 ) { // filter to remove vertical and horizontal lines

            // point of intersection of the line with first row
            Point pt1(rho/cos(theta),0);
            // point of intersection of the line with last row
            Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
            // draw a white line
            line( result, pt1, pt2, Scalar(255), 8);
            line( hough, pt1, pt2, Scalar(255), 8);
        }

        //std::cout << "line: (" << rho << "," << theta << ")\n";
        ++it;
    }
#endif
//    cvtColor(img_contours, img_3, COLOR_GRAY2RGB);
//    cvtColor(img_gray, img_3, COLOR_GRAY2RGB);
//    cv::cvtColor(img_3, img_rgba , CV_RGB2RGBA);

//    erode(img_gray, img_gray, kernel_ero);
}

void BallDetect(Mat & img_rgba, int &x, int &y, int &r) {
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
    jobject dstSurface) {
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

    LaneDetect(flipRgba);

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
