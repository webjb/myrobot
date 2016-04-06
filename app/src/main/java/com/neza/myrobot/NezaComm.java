package com.neza.myrobot;

import android.app.Activity;
import android.content.Context;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.InetAddress;
import java.net.UnknownHostException;

import android.support.v4.app.ActivityCompat;


/**
 * Created by bobsang on 3/21/2016.
 */
public class NezaComm {
    private final String TAG = "myRobot";

    public Socket mSocket;
    Thread serverThread = null;
    public static final int SERVERPORT = 6000;
    private static final String SERVER_IP = "192.168.2.77";

    private int mSocketOpened = 0;
    public int send_message(String str)
    {
        if( mSocketOpened == 0 )
            return 0;

        try {
            PrintWriter out = new PrintWriter(new BufferedWriter(
                    new OutputStreamWriter(mSocket.getOutputStream())),
                    true);
            out.println(str);
            Log.d(TAG, "bob send OK " + str);

        } catch (UnknownHostException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return 0;
    }

    public int send_lane(String str) {
        str = "<cmd>laneloc="+str+ "</cmd>";
        return send_message(str);
    }
    public boolean isConnected(Activity activity)
    {
        ConnectivityManager cm = (ConnectivityManager) activity.getSystemService(Context.CONNECTIVITY_SERVICE);
        NetworkInfo net = cm.getActiveNetworkInfo();
        if (net!=null && net.isAvailable() && net.isConnected()) {
            return true;
        } else {
            return false;
        }
    }

    public class DetectThread implements Runnable {
        public void run() {

            int ball_x;
            int ball_y;
            int ball_r;
            int need_send_ball_pos;
            int [] BallCenter;
            int [] DoorLocation;

            ball_x = 0;
            ball_y = 0;
            ball_r = 0;
            need_send_ball_pos = 0;

            BallCenter = new int[3];
            BallCenter[0] = 0;
            BallCenter[1] = 0;
            BallCenter[2] = 0;

            DoorLocation = new int[2];
            DoorLocation[0] = 0;
            DoorLocation[1] = 1;

            while( true) {
                try{
                    //if(BallCenter[0] > 200)
                    //Speech = (TextView)findViewById(R.id.speech);
                    //Speech.setText("ccc="+BallCenter[0]);
                    String str;
                    if( (ball_x != BallCenter[0]) || (ball_y != BallCenter[1]) || (ball_r != BallCenter[2]) )
                    {
                        need_send_ball_pos = 1;
                        ball_x = BallCenter[0];
                        ball_y = BallCenter[1];
                        ball_r = BallCenter[2];
                    }
                    if( need_send_ball_pos == 1 )
                    {
                        str = "<cmd>position="+BallCenter[0]+";"+BallCenter[1]+";"+BallCenter[2]+"</cmd>";
                        send_message(str);
                        need_send_ball_pos = 0;
                        //Log.i(TAG, "send ="+str);
                    }

                    str = "<cmd>doorloc="+DoorLocation[0]+";"+ DoorLocation[1]+"</cmd>";
                    send_message(str);

                    Thread.sleep(2000,0);
                } catch (InterruptedException e) {
                    System.out.println("Thread " + " interrupted.");
                }
            }
         }
    }

    class ServerThread implements Runnable {
        @Override
        public void run() {
            mSocketOpened = 0;
            try {
                Log.d(TAG, "bob comm run ...");
                InetAddress serverAddr = InetAddress.getByName(SERVER_IP);
                mSocket = new Socket(serverAddr, SERVERPORT);
                mSocketOpened = 1;
                Log.d(TAG, "bob open socket");
            } catch (UnknownHostException e1) {
                e1.printStackTrace();
            } catch (IOException e1) {
                e1.printStackTrace();
            }
            if( mSocketOpened == 0) {
                Log.d(TAG," bob open socket ERROR");
            }
        }
    }

    public int start()
    {
        Log.d(TAG, "bob start...");
        serverThread = new Thread(new ServerThread());
        serverThread.start();
        return 0;
    }

    public int stop()
    {
        if( serverThread != null )
        {
            try
            {
                serverThread.join();
                serverThread = null;

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        return 0;
    }

}
