package com.example.myapplication;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import android.graphics.Color;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.TextView;

import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

import io.github.controlwear.virtual.joystick.android.JoystickView;

public class ControlTest extends AppCompatActivity {
    TextView oTv,ConnectTv,UpTv,DownTv,RightTv,LeftTv,PingTv;
    JoystickView Joy;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_control_test);

        oTv = (TextView) findViewById(R.id.FPSTv);
        ConnectTv = (TextView) findViewById(R.id.ConnectTv);
        UpTv = (TextView) findViewById(R.id.UpTv);
        DownTv = (TextView) findViewById(R.id.DownTv);
        RightTv = (TextView) findViewById(R.id.RightTv);
        LeftTv = (TextView) findViewById(R.id.LeftTv);
        PingTv = (TextView) findViewById(R.id.FPSTv);
        Joy = (JoystickView) findViewById(R.id.jst);
        FirebaseDatabase database = FirebaseDatabase.getInstance();
        DatabaseReference mydata = FirebaseDatabase.getInstance().getReference();
        DatabaseReference Control = database.getReference("Control");
        Control.setValue("O");
        Joy.setOnMoveListener(new JoystickView.OnMoveListener() {
            @Override
            public void onMove(int angle, int strength) {
                //oTv.setText(String.valueOf(angle));
                if((angle>0&&angle<70)||(angle>300&&angle<360)){
                    Control.setValue("R");
                    oTv.setText("R");
                }else if(angle>=70&&angle<=130){
                    Control.setValue("U");
                    oTv.setText("U");
                }else if(angle>130&&angle<240){
                    Control.setValue("L");
                    oTv.setText("L");
                }else if(angle>=240&&angle<300){
                    Control.setValue("D");
                    oTv.setText("D");
                }else {
                    Control.setValue("O");
                    oTv.setText("O");
                }
                //Control.setValue("O");

            }
        });


        mydata.child("AngleNow").addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot snapshot) {
                if(snapshot.getValue().toString().equals("1")){
                    ConnectTv.setBackgroundColor(Color.parseColor("#ED2617"));
                    //ConnectTv.setBackgroundResource(R.drawable.circle);
                }
                if(snapshot.getValue().toString().equals("-1")){
                    ConnectTv.setBackgroundColor(Color.parseColor("#20D828"));
                }

            }

            @Override
            public void onCancelled(@NonNull DatabaseError error) {

            }
        });
        final boolean[] isDownU = {false};
        UpTv.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                if(action == MotionEvent.ACTION_DOWN){
                    Control.setValue("U");
                    UpTv.setBackgroundResource(R.drawable.circle3);
                    isDownU[0] =true;

                }else if(action == MotionEvent.ACTION_UP){
                    if(isDownU[0]){
                        Control.setValue("O");
                        UpTv.setBackgroundResource(R.drawable.circle2);
                        isDownU[0] =false;
                    }

                }
                return true;
            }
        });

        final boolean[] isDownD = {false};
        DownTv.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                if(action == MotionEvent.ACTION_DOWN){
                    Control.setValue("D");
                    DownTv.setBackgroundResource(R.drawable.circle3);
                    isDownD[0] =true;

                }else if(action == MotionEvent.ACTION_UP){
                    if(isDownD[0]){
                        Control.setValue("O");
                        DownTv.setBackgroundResource(R.drawable.circle2);
                        isDownD[0] =false;
                    }

                }
                return true;
            }
        });

        final boolean[] isDownR = {false};
        RightTv.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                if(action == MotionEvent.ACTION_DOWN){
                    Control.setValue("R");
                    RightTv.setBackgroundResource(R.drawable.circle3);
                    isDownR[0] =true;

                }else if(action == MotionEvent.ACTION_UP){
                    if(isDownR[0]){
                        Control.setValue("O");
                        RightTv.setBackgroundResource(R.drawable.circle2);
                        isDownR[0] =false;
                    }

                }
                return true;
            }
        });

        final boolean[] isDownL = {false};
        LeftTv.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                int action = event.getAction();
                if(action == MotionEvent.ACTION_DOWN){
                    Control.setValue("L");
                    LeftTv.setBackgroundResource(R.drawable.circle3);
                    isDownL[0] =true;

                }else if(action == MotionEvent.ACTION_UP){
                    if(isDownL[0]){
                        Control.setValue("O");
                        LeftTv.setBackgroundResource(R.drawable.circle2);
                        isDownL[0] =false;
                    }

                }
                return true;
            }
        });


    }
}