package com.example.audiosafety

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.TextView

class MainActivity : AppCompatActivity() {

    // Load compiled Rust library (.so file)
    companion object {
        init {
            System.loadLibrary("audio_safety_engine")
        }
    }

    // Declare native C/Rust functions
    private external fun initEngine(): Long

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        val textView = TextView(this)
        textView.text = "Acoustic Safety Engine Running..."
        textView.textSize = 20f
        setContentView(textView)

        // Initialize Rust DSP engine
        val enginePtr = initEngine()
    }
}
