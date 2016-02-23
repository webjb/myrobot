package com.neza.myrobot;

import android.graphics.ImageFormat;
import android.media.Image;
import android.media.Image.Plane;
import android.view.Surface;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;


import java.nio.ByteBuffer;

public class JNIUtils {
    private static final String TAG = "JNIUtils";

    // Load native Halide shared library.
    static {
        System.loadLibrary("myrobot");
        System.loadLibrary("opencv_java3");
    }


    /**
     * Use native code to copy the contents of src to dst. src must have format
     * YUV_420_888, dst must be YV12 and have been configured with
     * {@code configureSurface()}.
     */
    public static boolean blit(Image src, Surface dst) {
        Mat                    mRgba;
        if (src.getFormat() != ImageFormat.YUV_420_888) {
            throw new IllegalArgumentException("src must have format YUV_420_888.");
        }
        Plane[] planes = src.getPlanes();
        // Spec guarantees that planes[0] is luma and has pixel stride of 1.
        // It also guarantees that planes[1] and planes[2] have the same row and
        // pixel stride.
        if (planes[1].getPixelStride() != 1 && planes[1].getPixelStride() != 2) {
            throw new IllegalArgumentException(
                    "src chroma plane must have a pixel stride of 1 or 2: got "
                    + planes[1].getPixelStride());
        }
        mRgba = new Mat();
        mRgba.release();
        return blit(src.getWidth(), src.getHeight(), planes[0].getBuffer(),
                planes[0].getRowStride(), planes[1].getBuffer(), planes[2].getBuffer(),
                planes[1].getPixelStride(), planes[1].getRowStride(), dst);
    }


//    private static native void configureSurfaceNative(Surface surface, int width, int height);

    private static native boolean blit(int srcWidth, int srcHeight, ByteBuffer srcLuma,
            int srcLumaRowStride, ByteBuffer srcChromaU, ByteBuffer srcChromaV,
            int srcChromaUElementStride, int srcChromaURowStride, Surface dst);

    public static native void test();
}
