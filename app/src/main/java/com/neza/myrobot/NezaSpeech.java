package com.neza.myrobot;

/**
 * Created by bobsang on 3/23/2016.
 */

import android.content.ActivityNotFoundException;
import android.speech.RecognizerIntent;
import android.widget.TextView;

import android.content.Context;
import android.content.Intent;

import java.util.ArrayList;
import android.widget.Toast;
import android.app.Activity;

public class NezaSpeech {
    TextView Speech;
    ArrayList<String> matches_text;
    private final int REQ_CODE_SPEECH_INPUT = 100;

    public int start(Intent data)
    {
        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
                RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
//        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.getDefault());
//        intent.putExtra(RecognizerIntent.EXTRA_PROMPT,
//                getString(R.string.speech_prompt));
        try {
            //startActivityForResult(intent, REQ_CODE_SPEECH_INPUT);
        } catch (ActivityNotFoundException a) {
//            Toast.makeText(getApplicationContext(),getString(R.string.speech_not_supported), Toast.LENGTH_SHORT).show();
        }

        matches_text = data.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);
//        String str;
// 			Speech.setText("You have said " +matches_text.get(position));
//        str = "<cmd>speech:"+ matches_text.get(0)+"</cmd>";
//        send_message(str);
        return 0;
   }
}
