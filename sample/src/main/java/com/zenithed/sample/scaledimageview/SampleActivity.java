package com.zenithed.sample.scaledimageview;

import android.app.Activity;
import android.app.Fragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import com.zenithed.core.widget.scaledimageview.ScaledImageView;

public class SampleActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sample);

        if (savedInstanceState == null) {
            getFragmentManager().beginTransaction()
                    .add(R.id.container, new SampleFragment())
                    .commit();
        }
    }

    public static class SampleFragment extends Fragment {

        @Override
        public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
            final View rootView =  inflater.inflate(R.layout.fragment_sample, container, false);

            // gets the view and sets a nice image to it :)
            final ScaledImageView imageView = (ScaledImageView) rootView.findViewById(R.id.sample_image);
            imageView.setImageAsset("lazy-sloth.png");

            return rootView;
        }
    }

}
