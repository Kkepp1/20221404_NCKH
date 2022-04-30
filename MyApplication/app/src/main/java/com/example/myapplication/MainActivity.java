package com.example.myapplication;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;

import com.google.firebase.database.ChildEventListener;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;
import com.jjoe64.graphview.GraphView;
import com.jjoe64.graphview.helper.StaticLabelsFormatter;
import com.jjoe64.graphview.series.DataPoint;
import com.jjoe64.graphview.series.LineGraphSeries;
import com.jjoe64.graphview.series.PointsGraphSeries;

import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends AppCompatActivity {
    LineGraphSeries<DataPoint> series;
    LineGraphSeries<DataPoint> spseries;
    GraphView graph;
    int x=0;
    float y=0;
    int pre_x=0;
    float pre_y=0;
    String c="0";
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        FirebaseDatabase database = FirebaseDatabase.getInstance();
//        DatabaseReference teta = database.getReference("angle");
        DatabaseReference mydata = FirebaseDatabase.getInstance().getReference();
        graph = (GraphView) findViewById(R.id.graph);

        graph.setTitle("Angle per second");
        graph.getGridLabelRenderer().setVerticalAxisTitle("Angle(°)");
        graph.getGridLabelRenderer().setHorizontalAxisTitle("t(s)");
        graph.getViewport().setScrollable(true);
        graph.getViewport().setYAxisBoundsManual(true);
        graph.getViewport().setMaxY(300);
        graph.getViewport().setMinY(90);
  //      graph.getGridLabelRenderer().setNumVerticalLabels(5);
       StaticLabelsFormatter staticLabelsFormatter = new StaticLabelsFormatter(graph);
        staticLabelsFormatter.setVerticalLabels((new String[] {0+"",180+"\n SP",270+""}));
        graph.getGridLabelRenderer().setLabelFormatter(staticLabelsFormatter);
        Timer timer = new Timer();
//Set the schedule function
        timer.scheduleAtFixedRate(new TimerTask() {
                                      @Override
                                      public void run() {
                                          // update data angle
                                          mydata.child("AngleNow").addValueEventListener(new ValueEventListener() {
                                              @Override
                                              public void onDataChange(@NonNull DataSnapshot snapshot) {
                                                  float a=(float)snapshot.getValue(Integer.class);
                                                  y=a;
                                              }

                                              @Override
                                              public void onCancelled(@NonNull DatabaseError error) {

                                              }
                                          });
                                          // update data sp series
                                         x++;
                                          series = new LineGraphSeries<DataPoint>(new DataPoint[]{
                                                  new DataPoint(pre_x,pre_y),
                                                  new DataPoint(x, y)

                                          });
                                          pre_x=x;
                                          pre_y=y;
                                          // series
                                          series.setDrawDataPoints(true);
                                          series.setDataPointsRadius(11);
                                          series.appendData(new DataPoint(x,y),false, 40);
                                          graph.addSeries(series);
                                          // viewport
                                          graph.getViewport().setXAxisBoundsManual(true);
                                          if(x<=10){
                                              graph.getViewport().setMinX(0);
                                          }else{
                                              graph.getViewport().setMinX(x-10);
                                          }
                                          graph.getViewport().setMaxX(x+1);


                                      }
                                  },
                0, 1000);   // 1000 Millisecond  = 1 second
    }
}