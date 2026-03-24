package com.example.car_ab_control;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.widget.TextView;

import com.example.car_ab_control.databinding.ActivityMainBinding;

public class MainActivity extends AppCompatActivity {

    // Used to load the 'car_ab_control' library on application startup.
    static {
        System.loadLibrary("car_ab_control");
    }

    private ActivityMainBinding binding;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        // Example of a call to a native method
        TextView tv = binding.sampleText;
        tv.setText(stringFromJNI());
    }

    /**
     * A native method that is implemented by the 'car_ab_control' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();
}