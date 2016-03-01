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

// The source buffers must be YUV_420_888:
//   chroma_width = luma_width / 2
//   chroma_height = luma_height / 2
//   luma pixel stride is guaranteed to be 1.
//   both chroma planes have the same pixel stride and row stride.
//
//   We support chroma element strides of 1 or 2.
//   If the element stride is 1, then they are not interleaved. Just memcpy().
//   If the element stride is 2, then we further require that the chroma
//   planes be interleaved (the pointers differ by 1, it does not matter which
//   is first).
//
// The destination Surface must be YV12.
//
// The src and dst buffers have the same width, height, and row stride.
extern "C" {
JNIEXPORT bool JNICALL Java_com_neza_myrobot_JNIUtils_blit(
    JNIEnv *env, jobject obj, jint srcWidth, jint srcHeight,
    jobject srcLumaByteBuffer, jint srcLumaRowStrideBytes,
    jobject srcChromaUByteBuffer, jobject srcChromaVByteBuffer,
    jint srcChromaElementStrideBytes, jint srcChromaRowStrideBytes,
    jobject dstSurface) {
    if (srcChromaElementStrideBytes != 1 && srcChromaElementStrideBytes != 2) {
        return false;
    }

    uint8_t *srcLumaPtr = reinterpret_cast<uint8_t *>(
        env->GetDirectBufferAddress(srcLumaByteBuffer));
    uint8_t *srcChromaUPtr = reinterpret_cast<uint8_t *>(
        env->GetDirectBufferAddress(srcChromaUByteBuffer));
    uint8_t *srcChromaVPtr = reinterpret_cast<uint8_t *>(
        env->GetDirectBufferAddress(srcChromaVByteBuffer));

    int dstWidth;
    int dstHeight;

    LOGE("blit src: width=%d height=%d\n", srcWidth, srcHeight);

    cv::Mat mYuv(srcHeight + srcHeight/2,srcWidth, CV_8UC1, srcLumaPtr);

    if (srcLumaPtr == nullptr || srcChromaUPtr == nullptr || srcChromaVPtr == nullptr) {
        LOGE("blit NULL pointer ERROR");
        return false;
    }

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
    LOGE("blit 111");

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
    int winHeight = ANativeWindow_getHeight(win);
    int winWidth = ANativeWindow_getWidth(win);

//    LOGE("bob dst: width=%d height=%d winWidth=%d winHeight=%d format:%d stride:%d\n", buf.width, buf.height, winWidth, winHeight, buf.format, buf.stride);

    uint8_t *dstLumaPtr = reinterpret_cast<uint8_t *>(buf.bits);
    LOGE("blit 111 5555");
    Mat mr(srcHeight, srcWidth, CV_8UC4);
    cv::cvtColor(mYuv, mr, CV_YUV2RGBA_NV21);
    LOGE("blit 222");
    cv::Mat mRgba(dstHeight, buf.stride, CV_8UC4, dstLumaPtr);

    cv::Mat mm(dstHeight, dstWidth, CV_8UC4);

    cv::transpose(mr, mm);
//    LOGE("bob mm:%d x %d", mm.rows, mm.cols);
    cv::flip(mm, mm, 1);
    LOGE("blit 333");
    uchar * dbuf;
    uchar * sbuf;
    dbuf = mRgba.data;
    sbuf = mm.data;
    int i;
    int j;
    int k;
    for(i=0;i<mm.rows;i++) {
        dbuf = mRgba.data + i * buf.stride * 4;
        memcpy(dbuf, sbuf, mm.cols * 4);
        sbuf += mm.cols * 4;

//        for(j=0;j<mm.cols;j++){
//            for(k=0;k<4;k++) {
//                *dbuf++ = *sbuf++;
//            }
//        }
    }
//    LOGE("bob mm2:%d x %d type:%d mrgba:%d x %d type:%d", mm.rows, mm.cols, mm.type(), mRgba.rows, mRgba.cols, mRgba.type());
    LOGE("blit 444");
    Point p1(100,100);
    Point p2(300,300);
    cv::rectangle(mRgba,p1,p2,Scalar(255,255,255));
    cv::rectangle(mRgba,Point(10,10),Point(1079,1439),Scalar(255,255,255));
    cv::rectangle(mRgba,Point(100,100),Point(900,900),Scalar(255,255,255));


    ANativeWindow_unlockAndPost(win);
    ANativeWindow_release(win);
    LOGE("blit done");
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


#if 0
// src luma must have an element stride of 1.
extern "C" {
JNIEXPORT bool JNICALL Java_com_example_hogcamera_JNIUtils_edgeDetect(
    JNIEnv *env, jobject obj, jint srcWidth, jint srcHeight,
    jobject srcLumaByteBuffer, jint srcLumaRowStrideBytes, jobject dstSurface) {
    uint8_t *srcLumaPtr = reinterpret_cast<uint8_t *>(
        env->GetDirectBufferAddress(srcLumaByteBuffer));
    if (srcLumaPtr == NULL) {
        return false;
    }

    ANativeWindow *win = ANativeWindow_fromSurface(env, dstSurface);
    ANativeWindow_acquire(win);

    ANativeWindow_Buffer buf;
    if (int err = ANativeWindow_lock(win, &buf, NULL)) {
        LOGE("ANativeWindow_lock failed with error code %d\n", err);
        ANativeWindow_release(win);
        return false;
    }

    ANativeWindow_setBuffersGeometry(win, srcWidth, srcHeight, 0 /*format unchanged*/);

    uint8_t *dstLumaPtr = reinterpret_cast<uint8_t *>(buf.bits);
    if (dstLumaPtr == NULL) {
        ANativeWindow_unlockAndPost(win);
        ANativeWindow_release(win);
        return false;
    }

    if (buf.format != IMAGE_FORMAT_YV12) {
        LOGE("bob ANativeWindow buffer locked but its format was not YV12.");
        ANativeWindow_unlockAndPost(win);
        ANativeWindow_release(win);
        return false;
    }

    if (!checkBufferSizesMatch(srcWidth, srcHeight, &buf)) {
        LOGE("ANativeWindow buffer locked but its size was %d x %d, expected "
                "%d x %d", buf.width, buf.height, srcWidth, srcHeight);
        ANativeWindow_unlockAndPost(win);
        ANativeWindow_release(win);
        return false;
    }

    uint32_t dstLumaSizeBytes = buf.stride * buf.height;
    uint32_t dstChromaRowStrideBytes = ALIGN(buf.stride / 2, 16);
    // Size of one chroma plane.
    uint32_t dstChromaSizeBytes = dstChromaRowStrideBytes * buf.height / 2;
    uint8_t *dstChromaVPtr = dstLumaPtr + dstLumaSizeBytes;
    uint8_t *dstChromaUPtr = dstLumaPtr + dstLumaSizeBytes + dstChromaSizeBytes;

    // Make these static so that we can reuse device allocations across frames.
    // It doesn't matter now, but useful for GPU backends.
    static buffer_t srcBuf = { 0 };
    static buffer_t dstBuf = { 0 };
    static buffer_t dstChromaBuf = { 0 };

    srcBuf.host = srcLumaPtr;
    srcBuf.host_dirty = true;
    srcBuf.extent[0] = srcWidth;
    srcBuf.extent[1] = srcHeight;
    srcBuf.extent[2] = 0;
    srcBuf.extent[3] = 0;
    srcBuf.stride[0] = 1;
    srcBuf.stride[1] = srcLumaRowStrideBytes;
    srcBuf.min[0] = 0;
    srcBuf.min[1] = 0;
    srcBuf.elem_size = 1;

    dstBuf.host = dstLumaPtr;
    dstBuf.extent[0] = buf.width;  // src and dst width/height actually match.
    dstBuf.extent[1] = buf.height;
    dstBuf.extent[2] = 0;
    dstBuf.extent[3] = 0;
    dstBuf.stride[0] = 1;
    dstBuf.stride[1] = buf.stride;  // src and dst strides actually match.
    dstBuf.min[0] = 0;
    dstBuf.min[1] = 0;
    dstBuf.elem_size = 1;

    static bool first_call = true;
    static unsigned counter = 0;
    static unsigned times[16];
    if (first_call) {
        LOGD("According to Halide, host system has %d cpus\n",
             halide_host_cpu_count());
        first_call = false;
        for (int t = 0; t < 16; t++) {
            times[t] = 0;
        }
    }

    // Set chrominance to 128 to appear grayscale.
    // The dst chroma is guaranteed to be tightly packed since it's YV12.
    memset(dstChromaVPtr, 128, dstChromaSizeBytes * 2);

    int64_t t1 = halide_current_time_ns();
    int err = edge_detect(&srcBuf, &dstBuf);
    if (err != halide_error_code_success) {
        LOGE("edge_detect failed with error code: %d", err);
    }

    int64_t t2 = halide_current_time_ns();
    unsigned elapsed_us = (t2 - t1) / 1000;

    times[counter & 15] = elapsed_us;
    counter++;
    unsigned min = times[0];
    for (int i = 1; i < 16; i++) {
        if (times[i] < min) {
            min = times[i];
        }
    }
    LOGD("Time taken: %d us (minimum: %d us)", elapsed_us, min);

    ANativeWindow_unlockAndPost(win);
    ANativeWindow_release(win);

    return (err != halide_error_code_success);
}

}
#endif
