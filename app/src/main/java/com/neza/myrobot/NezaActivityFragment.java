package com.neza.myrobot;

//import android.support.v4.app.Fragment;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.DialogFragment;
import android.app.Fragment;
import android.content.Context;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.content.res.Configuration;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.Point;
import android.graphics.RectF;

import android.graphics.SurfaceTexture;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureFailure;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.DngCreator;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.hardware.camera2.params.InputConfiguration;
import android.media.Image;
import android.media.ImageReader;
import android.media.MediaScannerConnection;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.Message;
import android.os.SystemClock;
import android.support.v13.app.FragmentCompat;
import android.support.v4.app.ActivityCompat;
import android.util.Log;
import android.util.Size;
import android.util.SparseIntArray;
import android.view.LayoutInflater;
import android.view.OrientationEventListener;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.ViewGroup;
import android.widget.Toast;

import com.android.ex.camera2.blocking.BlockingCameraManager;
import com.android.ex.camera2.blocking.BlockingCameraManager.BlockingOpenException;
import com.android.ex.camera2.blocking.BlockingSessionCallback;
import com.android.ex.camera2.blocking.BlockingStateCallback;
import com.android.ex.camera2.exceptions.TimeoutRuntimeException;


import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

public class NezaActivityFragment extends Fragment
        implements View.OnClickListener, FragmentCompat.OnRequestPermissionsResultCallback {
        
    private static final String TAG = "myRobot";
    private static final int SESSION_WAIT_TIMEOUT_MS = 2500;
    private static final int REQUEST_CAMERA_PERMISSIONS = 1;

    private static final String[] CAMERA_PERMISSIONS = {
            Manifest.permission.CAMERA,
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
    };

    private static final int MAX_PREVIEW_WIDTH = 1920;
    private static final int MAX_PREVIEW_HEIGHT = 1080;
    private static final double ASPECT_RATIO_TOLERANCE = 0.005;

    private Size mCameraSize = new Size(800,600);
//    private Size mCameraSize = new Size(1280,960);

    private static final SparseIntArray ORIENTATIONS = new SparseIntArray();

    static {
        ORIENTATIONS.append(Surface.ROTATION_0, 0);
        ORIENTATIONS.append(Surface.ROTATION_90, 90);
        ORIENTATIONS.append(Surface.ROTATION_180, 180);
        ORIENTATIONS.append(Surface.ROTATION_270, 270);
    }

    private AutoFitSurfaceView mSurfaceView;
    private Surface mSurface;
    private final Object mStateLock = new Object();
    private final Semaphore mCameraOpenCloseLock = new Semaphore(1);

    private String mCameraId;
    private CameraCaptureSession mCaptureSession;
    private CameraDevice mCameraDevice;
    private Size mPreviewSize;
    private CameraCharacteristics mCharacteristics;
	private Handler mBackgroundHandler;
	
    private RefCountedAutoCloseable<ImageReader> mImageReader;
    private CaptureRequest.Builder mPreviewRequestBuilder;
    private BlockingStateCallback mDeviceCallback;

    private HandlerThread mBackgroundThread;
//    private final int mImageFormat = ImageFormat.FLEX_RGB_888;
    private final int mImageFormat = ImageFormat.YUV_420_888;
//    private final int mImageFormat = ImageFormat.RAW_SENSOR;

    private OrientationEventListener mOrientationListener;


    CaptureCallbackWaiter mPreCaptureCallback = new CaptureCallbackWaiter();

    private final TextureView.SurfaceTextureListener mSurfaceTextureListener
            = new TextureView.SurfaceTextureListener() {

        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture texture, int width, int height) {
            Log.d(TAG, "bob onSurfaceTextureAvailable");
            configureTransform(width, height);
        }

        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture texture, int width, int height) {
            Log.d(TAG, "bob onSurfaceTextureSizeChanged");
            configureTransform(width, height);
        }

        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture texture) {
            return true;
        }

        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture texture) {
        }

    };
/*

    private final SurfaceHolder.Callback mSurfaceCallback
            = new SurfaceHolder.Callback() {

        @Override
            holder.setFormat(ImageFormat.YV12);
            mSurface = holder.getSurface();

        }

        @Override
        public void surfaceChanged(SurfaceHolder holder, int format, int width,
                int height) {
            // Deliberately left empty. Everything is initialized in surfaceCreated().
            // If you decide to allow device rotation and to decouple the Surface
            // lifecycle from the Activity/Fragment UI lifecycle, then proper handling
            // of the YV12+JNI Surface configuration is much more complicated. You
            // will need to handle the logic in all three callbacks.
        }

        @Override
        public void surfaceDestroyed(SurfaceHolder holder) {
            // Deliberately left empty. Everything is initialized in surfaceCreated().
            // If you decide to allow device rotation and to decouple the Surface
            // lifecycle from the Activity/Fragment UI lifecycle, then proper handling
            // of the YV12+JNI Surface configuration is much more complicated. You
            // will need to handle the logic in all three callbacks.
        }

    };
*/
    private final CameraDevice.StateCallback mStateCallback = new CameraDevice.StateCallback() {

        @Override
        public void onOpened(CameraDevice cameraDevice) {
            // This method is called when the camera is opened.  We start camera preview here if
            // the TextureView displaying this has been set up.
            synchronized (mStateLock) {
                mCameraOpenCloseLock.release();
                mCameraDevice = cameraDevice;

                // Start the preview session if the TextureView has been set up already.
//                if (mPreviewSize != null && mSurfaceView.isAvailable()) {
                    createCameraPreviewSession();
//                }
            }
        }

        @Override
        public void onDisconnected(CameraDevice cameraDevice) {
            synchronized (mStateLock) {
                mCameraOpenCloseLock.release();
                cameraDevice.close();
                mCameraDevice = null;
            }
        }

        @Override
        public void onError(CameraDevice cameraDevice, int error) {
            Log.e(TAG, "Received camera device error: " + error);
            synchronized (mStateLock) {
                mCameraOpenCloseLock.release();
                cameraDevice.close();
                mCameraDevice = null;
            }
            Activity activity = getActivity();
            if (null != activity) {
                activity.finish();
            }
        }
    };
	
    private final CameraCaptureSession.CaptureCallback mCaptureCallback
            = new CameraCaptureSession.CaptureCallback() {
        @Override
        public void onCaptureStarted(CameraCaptureSession session, CaptureRequest request,
                                     long timestamp, long frameNumber) {
//            Log.d(TAG, "bob onCaptureStarted");

        }

        @Override
        public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request,
                                       TotalCaptureResult result) {
            // bob
//            Log.d(TAG, "bob onCaptureCompleted");
        }

        @Override
        public void onCaptureFailed(CameraCaptureSession session, CaptureRequest request,
                                    CaptureFailure failure) {
            // bob
            Log.d(TAG, "bob onCaptureFailed");
            /*
            int requestId = (int) request.getTag();

            synchronized (mCameraStateLock) {
                mRawResultQueue.remove(requestId);
                finishedCaptureLocked();
            }
            showToast("Capture failed!");
            */
        }

    };
	
    private final ImageReader.OnImageAvailableListener mOnImageAvailableListener
            = new ImageReader.OnImageAvailableListener() {

        @Override
        public void onImageAvailable(ImageReader reader) {
            //bob sang
            //Log.d(TAG, "bob image avail RAW");
            Image image;

            try {
                image = reader.acquireLatestImage();
                if( image == null) {
                    return;
                }
                int fmt = reader.getImageFormat();
                Log.d(TAG,"bob image fmt:"+ fmt);

                JNIUtils.blit(image, mSurface);
                //JNIUtils.blitraw(image, mSurface);
            } catch (IllegalStateException e) {
                Log.e(TAG, "Too many images queued for saving, dropping image for request: ");
//                        entry.getKey());
//                pendingQueue.remove(entry.getKey());
                return;
            }
            image.close();

        }
    };

	private void createCameraPreviewSession() {
		try {
            SurfaceTexture texture = mSurfaceView.getSurfaceTexture();
            // We configure the size of default buffer to be the size of camera preview we want.
            texture.setDefaultBufferSize(mPreviewSize.getWidth(), mPreviewSize.getHeight());

            // This is the output Surface we need to start preview.
            Surface surface = new Surface(texture);
            mSurface = surface;

            Log.d(TAG, "bob createCameraPreviewSession ========================");

			// We set up a CaptureRequest.Builder with the output Surface.
			mPreviewRequestBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            //mPreviewRequestBuilder.addTarget(surface);
			mPreviewRequestBuilder.addTarget(mImageReader.get().getSurface());

			BlockingSessionCallback sessionCallback = new BlockingSessionCallback();

            List<Surface> outputSurfaces = new ArrayList<Surface>();
            outputSurfaces.add(mImageReader.get().getSurface());
            //outputSurfaces.add(surface);

			mCameraDevice.createCaptureSession(outputSurfaces, sessionCallback, mBackgroundHandler);

			try {
				Log.d(TAG, "waiting on session.");
				mCaptureSession = sessionCallback.waitAndGetSession(SESSION_WAIT_TIMEOUT_MS);
				try {
					mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE,
						CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);

					// Comment out the above and uncomment this to disable continuous autofocus and
					// instead set it to a fixed value of 20 diopters. This should make the picture
					// nice and blurry for denoised edge detection.
					// mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE,
					//		   CaptureRequest.CONTROL_AF_MODE_OFF);
					// mPreviewRequestBuilder.set(CaptureRequest.LENS_FOCUS_DISTANCE, 20.0f);
					// Finally, we start displaying the camera preview.

					Log.d(TAG, "setting repeating request");

					mCaptureSession.setRepeatingRequest(mPreviewRequestBuilder.build(),
							mCaptureCallback, mBackgroundHandler);
				} catch (CameraAccessException e) {
					e.printStackTrace();
				}
			} catch (TimeoutRuntimeException e) {
				showToast("Failed to configure capture session.");
			}
		} catch (CameraAccessException e) {
			e.printStackTrace();
		}
	}

    public static NezaActivityFragment newInstance() {
        return new NezaActivityFragment();
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_main, container, false);
    }

    @Override
    public void onViewCreated(final View view, Bundle savedInstanceState) {
//        view.findViewById(R.id.picture).setOnClickListener(this);

		mSurfaceView = (AutoFitSurfaceView) view.findViewById(R.id.surface_view);
//		mSurfaceView.setAspectRatio(DESIRED_IMAGE_READER_SIZE.getWidth(),
//                DESIRED_IMAGE_READER_SIZE.getHeight());
	// This must be called here, before the initial buffer creation.
	// Putting this inside surfaceCreated() is insufficient.
//		mSurfaceView.getHolder().setFormat(ImageFormat.YV12);
        //view.findViewById(R.id.toggle).setOnClickListener(this);

        mOrientationListener = new OrientationEventListener(getActivity(),
                SensorManager.SENSOR_DELAY_NORMAL) {
            @Override
            public void onOrientationChanged(int orientation) {
                if (mSurfaceView != null && mSurfaceView.isAvailable()) {
                    Log.d(TAG, "OnViewCreated");
                    configureTransform(mSurfaceView.getWidth(), mSurfaceView.getHeight());
                }
            }
        };
    }

    @Override
    public void onResume() {
        super.onResume();
        startBackgroundThread();

        // Make the SurfaceView VISIBLE so that on resume, surfaceCreated() is called,
        // and on pause, surfaceDestroyed() is called.
//        mSurfaceView.setVisibility(View.VISIBLE);
//        mSurfaceView.getHolder().addCallback(mSurfaceCallback);
        openCamera(mSurfaceView.getWidth(), mSurfaceView.getHeight());

        if (mSurfaceView.isAvailable()) {
            Log.d(TAG, "onResume");
            configureTransform(mSurfaceView.getWidth(), mSurfaceView.getHeight());
        } else {
            mSurfaceView.setSurfaceTextureListener(mSurfaceTextureListener);
        }

        if (mOrientationListener != null && mOrientationListener.canDetectOrientation()) {
            mOrientationListener.enable();
        }
    }

    @Override
    public void onPause() {
        if (mOrientationListener != null) {
            mOrientationListener.disable();
        }
        closeCamera();
        stopBackgroundThread();
		
        super.onPause();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        if (requestCode == REQUEST_CAMERA_PERMISSIONS) {
            for (int result : grantResults) {
                if (result != PackageManager.PERMISSION_GRANTED) {
                    showMissingPermissionError();
                    return;
                }
            }
        } else {
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        }
    }

    @Override
    public void onClick(View view) {

    }

    private void showMissingPermissionError() {
        Activity activity = getActivity();
        if (activity != null) {
            Toast.makeText(activity, R.string.request_permission, Toast.LENGTH_SHORT).show();
            activity.finish();
        }
    }


    private void startBackgroundThread() {
        mBackgroundThread = new HandlerThread("CameraBackground");
        mBackgroundThread.start();
        synchronized (mStateLock) {
            mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
        }
    }

    private void stopBackgroundThread() {
        mBackgroundThread.quitSafely();
        try {
            mBackgroundThread.join();
            mBackgroundThread = null;
            synchronized (mStateLock) {
                mBackgroundHandler = null;
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
	
	private boolean setUpCameraOutputs(int width, int height) {
		Activity activity = getActivity();
		CameraManager manager = (CameraManager) activity.getSystemService(Context.CAMERA_SERVICE);
		if (manager == null) {
			ErrorDialog.buildErrorDialog("This device doesn't support Camera2 API.").
					show(getFragmentManager(), "dialog");
			return false;
		}
		try {
			// Find a CameraDevice that supports RAW captures, and configure state.
			for (String cameraId : manager.getCameraIdList()) {
				CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);

				// We only use a camera that supports RAW in this sample.
				if (!contains(characteristics.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES),
						CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_RAW)) {
					continue;
				}

				StreamConfigurationMap map = characteristics.get(
						CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);

                int[] of = map.getOutputFormats();
                for(int i=0;i<of.length;i++) {
                    Log.d(TAG, "bob output:" + i+" "+ of[i]);
                }
				Size[] sizes = map.getOutputSizes(mImageFormat);
                for(int i=0;i<sizes.length;i++)
                {
                    Log.d(TAG,"bob OutputSizes:"+i+" " + sizes[i].getWidth() + " x "+sizes[i].getHeight());
                }

				//Size largest = Collections.max(Arrays.asList(map.getOutputSizes(mImageFormat)),new CompareSizesByArea());
                //Size largest = new Size(1080,1440);
                //mCameraSize = largest;

				synchronized (mStateLock) {
					// Set up ImageReaders for JPEG and RAW outputs.  Place these in a reference
					// counted wrapper to ensure they are only closed when all background tasks
					// using them are finished.

                    Log.d(TAG, "bob Image size:" + mCameraSize.getWidth() + "x" + mCameraSize.getHeight());
					if (mImageReader == null || mImageReader.getAndRetain() == null) {
						mImageReader = new RefCountedAutoCloseable<>(
                          ImageReader.newInstance(mCameraSize.getWidth(), mCameraSize.getHeight(),mImageFormat,5));
					}
					mImageReader.get().setOnImageAvailableListener(mOnImageAvailableListener, mBackgroundHandler);
                    mCharacteristics = characteristics;

					mCameraId = cameraId;
				}
				return true;
			}
		} catch (CameraAccessException e) {
			e.printStackTrace();
		}

		// If we found no suitable cameras for capturing RAW, warn the user.
		ErrorDialog.buildErrorDialog("This device doesn't support capturing RAW photos").
				show(getFragmentManager(), "dialog");
		return false;
	}

    /**
     * Configures the necessary {@link android.graphics.Matrix} transformation to `mTextureView`.
     * This method should be called after the camera preview size is determined in
     * setUpCameraOutputs and also the size of `mTextureView` is fixed.
     *
     * @param viewWidth  The width of `mTextureView`
     * @param viewHeight The height of `mTextureView`
     */

    private void configureTransform(int viewWidth, int viewHeight) {
        Activity activity = getActivity();
        synchronized (mStateLock) {
            if (null == mSurfaceView || null == activity) {
                return;
            }

            StreamConfigurationMap map = mCharacteristics.get(
                    CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);

            // For still image captures, we always use the largest available size.
//            Size largestJpeg = Collections.max(Arrays.asList(map.getOutputSizes(mImageFormat)), new CompareSizesByArea());

            // Find the rotation of the device relative to the native device orientation.
            int deviceRotation = activity.getWindowManager().getDefaultDisplay().getRotation();
            Point displaySize = new Point();
            activity.getWindowManager().getDefaultDisplay().getSize(displaySize);

            // Find the rotation of the device relative to the camera sensor's orientation.
            int totalRotation = sensorToDeviceRotation(mCharacteristics, deviceRotation);

            // Swap the view dimensions for calculation as needed if they are rotated relative to
            // the sensor.
            Log.d(TAG, "bob rotation:"+ deviceRotation + " x " + totalRotation + " size:"+ viewWidth + "x" + viewHeight + " disp:"+displaySize.x+"x"+displaySize.y);
            boolean swappedDimensions = totalRotation == 90 || totalRotation == 270;
            int rotatedViewWidth = viewWidth;
            int rotatedViewHeight = viewHeight;
            int maxPreviewWidth = displaySize.x;
            int maxPreviewHeight = displaySize.y;

            //swappedDimensions = false;
            if (swappedDimensions) {
                Log.d(TAG, "bob swapped "+ swappedDimensions);
                rotatedViewWidth = viewHeight;
                rotatedViewHeight = viewWidth;
                maxPreviewWidth = displaySize.y;
                maxPreviewHeight = displaySize.x;
            }

            // Preview should not be larger than display size and 1080p.
            if (maxPreviewWidth > MAX_PREVIEW_WIDTH) {
                maxPreviewWidth = MAX_PREVIEW_WIDTH;
            }

            if (maxPreviewHeight > MAX_PREVIEW_HEIGHT) {
                maxPreviewHeight = MAX_PREVIEW_HEIGHT;
            }

            // Find the best preview size for these view dimensions and configured JPEG size.
            Size previewSize = chooseOptimalSize(map.getOutputSizes(SurfaceTexture.class),
                    rotatedViewWidth, rotatedViewHeight, maxPreviewWidth, maxPreviewHeight,
                    mCameraSize);

            if (swappedDimensions) {
                mSurfaceView.setAspectRatio(previewSize.getHeight(), previewSize.getWidth());
            } else {
                mSurfaceView.setAspectRatio(previewSize.getWidth(), previewSize.getHeight());
            }

            // Find rotation of device in degrees (reverse device orientation for front-facing
            // cameras).
            int rotation = (mCharacteristics.get(CameraCharacteristics.LENS_FACING) ==
                    CameraCharacteristics.LENS_FACING_FRONT) ?
                    (360 + ORIENTATIONS.get(deviceRotation)) % 360 :
                    (360 - ORIENTATIONS.get(deviceRotation)) % 360;

            Matrix matrix = new Matrix();
            RectF viewRect = new RectF(0, 0, viewWidth, viewHeight);
            RectF bufferRect = new RectF(0, 0, previewSize.getHeight(), previewSize.getWidth());
            float centerX = viewRect.centerX();
            float centerY = viewRect.centerY();

            // Initially, output stream images from the Camera2 API will be rotated to the native
            // device orientation from the sensor's orientation, and the TextureView will default to
            // scaling these buffers to fill it's view bounds.  If the aspect ratios and relative
            //     in the native device orientation) to the TextureView's dimension.
            //   - Apply a scale-to-fill from the output buffer's rotated dimensions
            //     (i.e. its dimensions in the current device orientation) to the TextureView's
            //     dimensions.
            //   - Apply the rotation from the native device orientation to the current device
            //     rotation.
            if (Surface.ROTATION_90 == deviceRotation || Surface.ROTATION_270 == deviceRotation) {
                bufferRect.offset(centerX - bufferRect.centerX(), centerY - bufferRect.centerY());
                matrix.setRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.FILL);
                float scale = Math.max(
                        (float) viewHeight / previewSize.getHeight(),
                        (float) viewWidth / previewSize.getWidth());
                matrix.postScale(scale, scale, centerX, centerY);

            }
            matrix.postRotate(rotation, centerX, centerY);
            Log.d(TAG, "bob rotation:"+rotation + " centerX:"+ centerX + " centerY:"+ centerY);
            mSurfaceView.setTransform(matrix);

            // Start or restart the active capture session if the preview was initialized or
            // if its aspect ratio changed significantly.
            if (mPreviewSize == null || !checkAspectsEqual(previewSize, mPreviewSize)) {
                mPreviewSize = previewSize;
//                if (mState != STATE_CLOSED) {
                    createCameraPreviewSession();
//                }
            }
            Log.d(TAG, "bob rotation previewSize:"+mPreviewSize.getWidth() +" "+ mPreviewSize.getHeight() );
        }
    }
   /*
    private void configureTransform(int viewWidth, int viewHeight) {
        Activity activity = getActivity();
        if (null == mSurfaceView || null == mPreviewSize || null == activity) {
            return;
        }
        } else if (Surface.ROTATION_180 == rotation) {
            matrix.postRotate(180, centerX, centerY);
        }
        mSurfaceView.setTransform(matrix);
    }
*/
    private void openCamera(int width, int height) {
        if (!setUpCameraOutputs(width, height)) {
            return;
        }
        if (!hasAllPermissionsGranted()) {
            requestCameraPermissions();
            return;
        }

        Activity activity = getActivity();
        CameraManager manager = (CameraManager) activity.getSystemService(Context.CAMERA_SERVICE);
        BlockingCameraManager blockingManager = new BlockingCameraManager(manager);
        try {
            mDeviceCallback = new BlockingStateCallback() {

                @Override
                public void onDisconnected(CameraDevice camera) {
                    camera.close();
                    mCameraDevice = null;
                }

                @Override
                public void onError(CameraDevice camera, int error) {
                    camera.close();
                    mCameraDevice = null;
                    Activity activity = getActivity();
                    if (activity != null) {
                        activity.finish();
                    }
                }
            };

            mCameraDevice = blockingManager.openCamera(mCameraId, mDeviceCallback,mBackgroundHandler);

        } catch (BlockingOpenException|TimeoutRuntimeException e) {
            showToast("Timed out opening camera.");
        } catch (CameraAccessException e) {
            showToast("Failed to open camera."); // failed immediately.
        }
     }


    private void closeCamera() {
        try {
            mCameraOpenCloseLock.acquire();
            synchronized (mStateLock) {

                // Reset state and clean up resources used by the camera.
                // Note: After calling this, the ImageReaders will be closed after any background
                // tasks saving Images from these readers have been completed.
                if (null != mCaptureSession) {
                    mCaptureSession.close();
                    mCaptureSession = null;
                }
                if (null != mCameraDevice) {
                    mCameraDevice.close();
                    mCameraDevice = null;
                }
                if (null != mImageReader) {
                    mImageReader.close();
                    mImageReader = null;
                }
            }
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted while trying to lock camera closing.", e);
        } finally {
            mCameraOpenCloseLock.release();
        }
    }

    /**
     * Comparator based on area of the given {@link Size} objects.
     */
    static class CompareSizesByArea implements Comparator<Size> {

        @Override
        public int compare(Size lhs, Size rhs) {
            // We cast here to ensure the multiplications won't overflow
            return Long.signum((long) lhs.getWidth() * lhs.getHeight() -
                    (long) rhs.getWidth() * rhs.getHeight());
        }

    }

    /**
     * A dialog fragment for displaying non-recoverable errors; this {@ling Activity} will be
     * finished once the dialog has been acknowledged by the user.
     */
    public static class ErrorDialog extends DialogFragment {

        private String mErrorMessage;

        public ErrorDialog() {
            mErrorMessage = "Unknown error occurred!";
        }

        // Build a dialog with a custom message (Fragments require default constructor).
        public static ErrorDialog buildErrorDialog(String errorMessage) {
            ErrorDialog dialog = new ErrorDialog();
            dialog.mErrorMessage = errorMessage;
            return dialog;
        }

        @Override
        public Dialog onCreateDialog(Bundle savedInstanceState) {
            final Activity activity = getActivity();
            return new AlertDialog.Builder(activity)
                    .setMessage(mErrorMessage)
                    .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialogInterface, int i) {
                            activity.finish();
                        }
                    })
                    .create();
        }
    }

    public static class RefCountedAutoCloseable<T extends AutoCloseable> implements AutoCloseable {
		private T mObject;
		private long mRefCount = 0;

		/**
		 * Wrap the given object.
		 *
		 * @param object an object to wrap.
		 */
		public RefCountedAutoCloseable(T object) {
			if (object == null) throw new NullPointerException();
			mObject = object;
		}

		/**
		 * Increment the reference count and return the wrapped object.
		 *
		 * @return the wrapped object, or null if the object has been released.
		 */
		public synchronized T getAndRetain() {
			if (mRefCount < 0) {
				return null;
			}
			mRefCount++;
			return mObject;
		}

		/**
		 * Return the wrapped object.
		 *
		 * @return the wrapped object, or null if the object has been released.
		 */
		public synchronized T get() {
			return mObject;
		}

		/**
		 * Decrement the reference count and release the wrapped object if there are no other
		 * users retaining this object.
		 */
		@Override
		public synchronized void close() {
			if (mRefCount >= 0) {
				mRefCount--;
				if (mRefCount < 0) {
					try {
						mObject.close();
					} catch (Exception e) {
						throw new RuntimeException(e);
					} finally {
						mObject = null;
					}
				}
			}
		}
	}

    private class CaptureCallbackWaiter extends CameraCaptureSession.CaptureCallback {
        private final LinkedBlockingQueue<TotalCaptureResult> mResultQueue =
                new LinkedBlockingQueue<>();

        @Override
        public void onCaptureStarted(CameraCaptureSession session, CaptureRequest request,
                                     long timestamp, long frameNumber) {
        }

        @Override
        public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request,
                                       TotalCaptureResult result) {
            try {
                mResultQueue.put(result);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        @Override
        public void onCaptureFailed(CameraCaptureSession session, CaptureRequest request,
                                    CaptureFailure failure) {
            Log.e(TAG, "Script error: capture failed");
        }

        public TotalCaptureResult getResult(long timeoutMs) {
            TotalCaptureResult result = null;
            try {
                result = mResultQueue.poll(timeoutMs, TimeUnit.MILLISECONDS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if (result == null) {
                Log.d(TAG, "Getting an image timed out after " + timeoutMs +
                        "ms");
            }

            return result;
        }
    }

    public static class PermissionConfirmationDialog extends DialogFragment {

        public static PermissionConfirmationDialog newInstance() {
            return new PermissionConfirmationDialog();
        }

        @Override
        public Dialog onCreateDialog(Bundle savedInstanceState) {
            final Fragment parent = getParentFragment();
            return new AlertDialog.Builder(getActivity())
                    .setMessage(R.string.request_permission)
                    .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            FragmentCompat.requestPermissions(parent, CAMERA_PERMISSIONS,
                                    REQUEST_CAMERA_PERMISSIONS);
                        }
                    })
                    .setNegativeButton(android.R.string.cancel,
                            new DialogInterface.OnClickListener() {
                                @Override
                                public void onClick(DialogInterface dialog, int which) {
                                    getActivity().finish();
                                }
                            })
                    .create();
        }

    }
    /**
     * Given {@code choices} of {@code Size}s supported by a camera, choose the smallest one that
     * is at least as large as the respective texture view size, and that is at most as large as the
     * respective max size, and whose aspect ratio matches with the specified value. If such size
     * doesn't exist, choose the largest one that is at most as large as the respective max size,
     * and whose aspect ratio matches with the specified value.
     *
     * @param choices           The list of sizes that the camera supports for the intended output
     *                          class
     * @param textureViewWidth  The width of the texture view relative to sensor coordinate
     * @param textureViewHeight The height of the texture view relative to sensor coordinate
     * @param maxWidth          The maximum width that can be chosen
     * @param maxHeight         The maximum height that can be chosen
     * @param aspectRatio       The aspect ratio
     * @return The optimal {@code Size}, or an arbitrary one if none were big enough
     */
    private static Size chooseOptimalSize(Size[] choices, int textureViewWidth,
                                          int textureViewHeight, int maxWidth, int maxHeight, Size aspectRatio) {
        // Collect the supported resolutions that are at least as big as the preview Surface
        List<Size> bigEnough = new ArrayList<>();
        // Collect the supported resolutions that are smaller than the preview Surface
        List<Size> notBigEnough = new ArrayList<>();
        int w = aspectRatio.getWidth();
        int h = aspectRatio.getHeight();
        for (Size option : choices) {
            if (option.getWidth() <= maxWidth && option.getHeight() <= maxHeight &&
                    option.getHeight() == option.getWidth() * h / w) {
                if (option.getWidth() >= textureViewWidth &&
                        option.getHeight() >= textureViewHeight) {
                    bigEnough.add(option);
                } else {
                    notBigEnough.add(option);
                }
            }
        }

        // Pick the smallest of those big enough. If there is no one big enough, pick the
        // largest of those not big enough.
        if (bigEnough.size() > 0) {
            return Collections.min(bigEnough, new CompareSizesByArea());
        } else if (notBigEnough.size() > 0) {
            return Collections.max(notBigEnough, new CompareSizesByArea());
        } else {
            Log.e(TAG, "Couldn't find any suitable preview size");
            return choices[0];
        }
    }

    /**
     * Generate a string containing a formatted timestamp with the current date and time.
     *
     * @return a {@link String} representing a time.
     */
    private static String generateTimestamp() {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss_SSS", Locale.US);
        return sdf.format(new Date());
    }

    /**
     * Cleanup the given {@link OutputStream}.
     *
     * @param outputStream the stream to close.
     */
    private static void closeOutput(OutputStream outputStream) {
        if (null != outputStream) {
            try {
                outputStream.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Return true if the given array contains the given integer.
     *
     * @param modes array to check.
     * @param mode  integer to get for.
     * @return true if the array contains the given integer, otherwise false.
     */
    private static boolean contains(int[] modes, int mode) {
        if (modes == null) {
            return false;
        }
        for (int i : modes) {
            if (i == mode) {
                return true;
            }
        }
        return false;
    }

    /**
     * Return true if the two given {@link Size}s have the same aspect ratio.
     *
     * @param a first {@link Size} to compare.
     * @param b second {@link Size} to compare.
     * @return true if the sizes have the same aspect ratio, otherwise false.
     */
    private static boolean checkAspectsEqual(Size a, Size b) {
        double aAspect = a.getWidth() / (double) a.getHeight();
        double bAspect = b.getWidth() / (double) b.getHeight();
        return Math.abs(aAspect - bAspect) <= ASPECT_RATIO_TOLERANCE;
    }

    /**
     * Rotation need to transform from the camera sensor orientation to the device's current
     * orientation.
     *
     * @param c                 the {@link CameraCharacteristics} to query for the camera sensor
     *                          orientation.
     * @param deviceOrientation the current device orientation relative to the native device
     *                          orientation.
     * @return the total rotation from the sensor orientation to the current device orientation.
     */
    private static int sensorToDeviceRotation(CameraCharacteristics c, int deviceOrientation) {
        int sensorOrientation = c.get(CameraCharacteristics.SENSOR_ORIENTATION);

        // Get device orientation in degrees
        deviceOrientation = ORIENTATIONS.get(deviceOrientation);

        // Reverse device orientation for front-facing cameras
        if (c.get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_FRONT) {
            deviceOrientation = -deviceOrientation;
        }

        // Calculate desired JPEG orientation relative to camera orientation to make
        // the image upright relative to the device orientation
        return (sensorOrientation + deviceOrientation + 360) % 360;
    }
    private void requestCameraPermissions() {
        if (shouldShowRationale()) {
            PermissionConfirmationDialog.newInstance().show(getChildFragmentManager(), "dialog");
        } else {
            FragmentCompat.requestPermissions(this, CAMERA_PERMISSIONS, REQUEST_CAMERA_PERMISSIONS);
        }
    }

    /**
     * Tells whether all the necessary permissions are granted to this app.
     *
     * @return True if all the required permissions are granted.
     */
    private boolean hasAllPermissionsGranted() {
        for (String permission : CAMERA_PERMISSIONS) {
            if (ActivityCompat.checkSelfPermission(getActivity(), permission)
                    != PackageManager.PERMISSION_GRANTED) {
                return false;
            }
        }
        return true;
    }

    /**
     * Gets whether you should show UI with rationale for requesting the permissions.
     *
     * @return True if the UI should be shown.
     */
    private boolean shouldShowRationale() {
        for (String permission : CAMERA_PERMISSIONS) {
            if (FragmentCompat.shouldShowRequestPermissionRationale(this, permission)) {
                return true;
            }
        }
        return false;
    }

    private final Handler mMessageHandler = new Handler(Looper.getMainLooper()) {
        @Override
        public void handleMessage(Message msg) {
            Activity activity = getActivity();
            if (activity != null) {
                Toast.makeText(activity, (String) msg.obj, Toast.LENGTH_SHORT).show();
            }
        }
    };

	private void showToast(String text) {
		// We show a Toast by sending request message to mMessageHandler. This makes sure that the
		// Toast is shown on the UI thread.
		Message message = Message.obtain();
		message.obj = text;
		mMessageHandler.sendMessage(message);
	}

}
