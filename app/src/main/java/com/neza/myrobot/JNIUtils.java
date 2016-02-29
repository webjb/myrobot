package com.neza.myrobot;

import android.graphics.ImageFormat;
import android.media.Image;
import android.media.Image.Plane;
import android.view.Surface;

import org.opencv.core.CvType;
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

    public static Mat imageToMat(Image image) {
        ByteBuffer buffer;
        int rowStride;
        int pixelStride;
        int width = image.getWidth();
        int height = image.getHeight();
        int offset = 0;

        Image.Plane[] planes = image.getPlanes();
        byte[] data = new byte[image.getWidth() * image.getHeight() * 3 /2];// ImageFormat.getBitsPerPixel(ImageFormat.YUV_420_888) / 8];
        byte[] rowData = new byte[planes[0].getRowStride()];

        for (int i = 0; i < planes.length; i++) {
            buffer = planes[i].getBuffer();
            rowStride = planes[i].getRowStride();
            pixelStride = planes[i].getPixelStride();
            int w = (i == 0) ? width : width / 2;
            int h = (i == 0) ? height : height / 2;
            for (int row = 0; row < h; row++) {
                int bytesPerPixel = ImageFormat.getBitsPerPixel(ImageFormat.YUV_420_888) / 8;
                if (pixelStride == bytesPerPixel) {
                    int length = w * bytesPerPixel;
                    buffer.get(data, offset, length);

                    // Advance buffer the remainder of the row stride, unless on the last row.
                    // Otherwise, this will throw an IllegalArgumentException because the buffer
                    // doesn't include the last padding.
                    if (h - row != 1) {
                        buffer.position(buffer.position() + rowStride - length);
                    }
                    offset += length;
                } else {

                    // On the last row only read the width of the image minus the pixel stride
                    // plus one. Otherwise, this will throw a BufferUnderflowException because the
                    // buffer doesn't include the last padding.
                    if (h - row == 1) {
                        buffer.get(rowData, 0, width - pixelStride + 1);
                    } else {
                        buffer.get(rowData, 0, rowStride);
                    }

                    for (int col = 0; col < w; col++) {
                        data[offset++] = rowData[col * pixelStride];
                    }
                }
            }
        }

        // Finally, create the Mat.
        Mat mat = new Mat(height + height / 2, width, CvType.CV_8UC1);
        mat.put(0, 0, data);

        return mat;
    }

    public static boolean blitraw(Image src, Surface dst) {
        Plane[] planes = src.getPlanes();
        // Spec guarantees that planes[0] is luma and has pixel stride of 1.
        // It also guarantees that planes[1] and planes[2] have the same row and
        // pixel stride.
        Mat mmt = imageToMat(src);
        return blitraw(src.getWidth(), src.getHeight(), mmt.getNativeObjAddr(), dst);
    };

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


    private static native boolean blitraw(int srcWidth, int srcHeight, long yuv,
                                           Surface dst);

    public static native void test();
}
