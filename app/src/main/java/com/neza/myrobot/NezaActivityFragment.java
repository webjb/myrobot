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
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.Point;
import android.graphics.RectF;

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

    private HandlerThread mBackgroundThread;
	
    CaptureCallbackWaiter mPreCaptureCallback = new CaptureCallbackWaiter();

    private final SurfaceHolder.Callback mSurfaceCallback
            = new SurfaceHolder.Callback() {

        @Override
        public void surfaceCreated(SurfaceHolder holder) {
            // The surface is ready. Make its buffers use the YV12 format and use the NDK to
            // reconfigure its buffers to be of a different size than on screen (i.e., use the
            // resolution of the ImageReader instead and use the hardware scaler to interpolate).
            // (If you don't set it here, funny things happen when if you sleep the device).
            holder.setFormat(ImageFormat.YV12);
            mSurface = holder.getSurface();
            openCamera();
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
            Log.d(TAG, "bob onCaptureStarted");

        }

        @Override
        public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request,
                                       TotalCaptureResult result) {
            // bob
            Log.d(TAG, "bob onCaptureCompleted");
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
            Log.d(TAG, "bob image avail RAW");
            Image image;

            try {
                image = reader.acquireLatestImage();
                if( image == null) {
                    return;
                }
                JNIUtils.blit(image, mSurface);
                //JNIUtils.test();
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

			// We set up a CaptureRequest.Builder with the output Surface.
			mPreviewRequestBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
//			  mPreviewRequestBuilder.addTarget(surface);
			mPreviewRequestBuilder.addTarget(mImageReader.get().getSurface());

			BlockingSessionCallback sessionCallback = new BlockingSessionCallback();

			mCameraDevice.createCaptureSession(
					Arrays.asList(mImageReader.get().getSurface()), sessionCallback, mBackgroundHandler);

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
        view.findViewById(R.id.picture).setOnClickListener(this);

		mSurfaceView = (AutoFitSurfaceView) view.findViewById(R.id.surface_view);
		mSurfaceView.setAspectRatio(800,600);
	// This must be called here, before the initial buffer creation.
	// Putting this inside surfaceCreated() is insufficient.
		mSurfaceView.getHolder().setFormat(ImageFormat.YV12);
	

    }

    @Override
    public void onResume() {
        super.onResume();
        startBackgroundThread();

        // Make the SurfaceView VISIBLE so that on resume, surfaceCreated() is called,
        // and on pause, surfaceDestroyed() is called.
        mSurfaceView.setVisibility(View.VISIBLE);
        mSurfaceView.getHolder().addCallback(mSurfaceCallback);
		
//        if (mOrientationListener != null && mOrientationListener.canDetectOrientation()) {
//            mOrientationListener.enable();
//        }
    }

    @Override
    public void onPause() {
//        if (mOrientationListener != null) {
//            mOrientationListener.disable();
//        }
        closeCamera();
        stopBackgroundThread();
        // Make the SurfaceView GONE so that on resume, surfaceCreated() is called,
        // and on pause, surfaceDestroyed() is called.
        mSurfaceView.getHolder().removeCallback(mSurfaceCallback);
        mSurfaceView.setVisibility(View.GONE);
		
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
	
	private boolean setUpCameraOutputs() {
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

				// For still image captures, we use the largest available size.
				Size largestYuv = Collections.max(
						Arrays.asList(map.getOutputSizes(ImageFormat.YUV_420_888)),
						new CompareSizesByArea());

				synchronized (mStateLock) {
					// Set up ImageReaders for JPEG and RAW outputs.  Place these in a reference
					// counted wrapper to ensure they are only closed when all background tasks
					// using them are finished.
					//bob sang
					Log.d(TAG, "bob Image size:"+largestYuv.getWidth() + "x" + largestYuv.getHeight());
					if (mImageReader == null || mImageReader.getAndRetain() == null) {
						mImageReader = new RefCountedAutoCloseable<>(
//                                ImageReader.newInstance(largestYuv.getWidth(),largestYuv.getHeight(),ImageFormat.YUV_420_888,5));
                          ImageReader.newInstance(800,600,ImageFormat.YUV_420_888,5));

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

    private void openCamera() {
        if (!setUpCameraOutputs()) {
            return;
        }
        if (!hasAllPermissionsGranted()) {
            requestCameraPermissions();
            return;
        }

        Activity activity = getActivity();
        CameraManager manager = (CameraManager) activity.getSystemService(Context.CAMERA_SERVICE);
        try {
            // Wait for any previously running session to finish.
            if (!mCameraOpenCloseLock.tryAcquire(2500, TimeUnit.MILLISECONDS)) {
                throw new RuntimeException("Time out waiting to lock camera opening.");
            }

            String cameraId;
            Handler backgroundHandler;
            synchronized (mStateLock) {
                cameraId = mCameraId;
                backgroundHandler = mBackgroundHandler;
            }

            // Attempt to open the camera. mStateCallback will be called on the background handler's
            // thread when this succeeds or fails.
            manager.openCamera(cameraId, mStateCallback, backgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted while trying to lock camera opening.", e);
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
