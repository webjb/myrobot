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
    private static final String TAG = "myRobot";

    private Socket socket;
    Thread serverThread = null;
    public static final int SERVERPORT = 6000;
    private static final String SERVER_IP = "192.168.2.77";
    int [] BallCenter;
    int [] DoorLocation;

    public int send_message(String str)
    {
        try {
            PrintWriter out = new PrintWriter(new BufferedWriter(
                    new OutputStreamWriter(socket.getOutputStream())),
                    true);
            out.println(str);
        } catch (UnknownHostException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return 0;
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

    class DetectThread implements Runnable {
        public void run() {

            int ball_x;
            int ball_y;
            int ball_r;
            int need_send_ball_pos;
            ball_x = 0;
            ball_y = 0;
            ball_r = 0;
            need_send_ball_pos = 0;

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

                    Thread.sleep(200,0);
                } catch (InterruptedException e) {
                    System.out.println("Thread " + " interrupted.");
                }
            }
        }
    }

    class ClientThread implements Runnable {
        @Override
        public void run() {
            try {
                InetAddress serverAddr = InetAddress.getByName(SERVER_IP);
                socket = new Socket(serverAddr, SERVERPORT);
                Log.i(TAG, "open socket");
            } catch (UnknownHostException e1) {
                e1.printStackTrace();
            } catch (IOException e1) {
                e1.printStackTrace();
            }
        }
    }
}
