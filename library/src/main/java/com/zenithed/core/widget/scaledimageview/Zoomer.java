/*
 * Copyright 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.zenithed.core.widget.scaledimageview;

import android.content.Context;
import android.os.SystemClock;
import android.view.animation.DecelerateInterpolator;
import android.view.animation.Interpolator;

/**
 * A simple class that animates double-touch zoom gestures. Functionally similar to a {@link
 * android.widget.Scroller}.
 */
public class Zoomer {
    /**
     * The interpolator, used for making zooms animate 'naturally.'
     */
    private Interpolator mInterpolator;


    private int mAnimationDurationMillis;
    private int mDefaultAnimationDurationMillis;

    /**
     * Whether or not the current zoom has finished.
     */
    private boolean mFinished = true;

    /**
     * The current zoom value; computed by {@link #computeZoom()}.
     */
    private float mCurrentZoom;

    /**
     * The time the zoom started, computed using {@link android.os.SystemClock#elapsedRealtime()}.
     */
    private long mStartRTC;

    private float mStartZoom;
    private float mEndZoom;
    private float mDeltaZoom;

    public Zoomer(Context context) {
        mInterpolator = new DecelerateInterpolator();
        mDefaultAnimationDurationMillis = context.getResources().getInteger(android.R.integer.config_shortAnimTime);
    }

    /**
     * Forces the zoom finished state to the given value. Unlike {@link #abortAnimation()}, the
     * current zoom value isn't set to the ending value.
     *
     * @see android.widget.Scroller#forceFinished(boolean)
     */
    public void forceFinished(boolean finished) {
        mFinished = finished;
    }

    /**
     * Aborts the animation, setting the current zoom value to the ending value.
     *
     * @see android.widget.Scroller#abortAnimation()
     */
    public void abortAnimation() {
        mFinished = true;
        mCurrentZoom = mEndZoom;
    }

    /**
     * Starts a zoom from startZoom to endZoom.
     *
     * @see android.widget.Scroller#startScroll(int, int, int, int)
     */
    public void startZoom(float startZoom, float endZoom) {
        startZoomHelper(startZoom, endZoom, mDefaultAnimationDurationMillis);
    }

    /**
     * Starts a zoom from startZoom to endZoom.
     *
     * @see android.widget.Scroller#startScroll(int, int, int, int)
     */
    public void startZoom(float startZoom, float endZoom, int duration) {
        startZoomHelper(startZoom, endZoom, duration);
    }

    /**
     * Computes the current zoom level, returning true if the zoom is still active and false if the
     * zoom has finished.
     *
     * @see android.widget.Scroller#computeScrollOffset()
     */
    public boolean computeZoom() {
        if (mFinished) {
            return false;
        }

        final long tRTC = SystemClock.elapsedRealtime() - mStartRTC;
        if (tRTC >= mAnimationDurationMillis) {
            mFinished = true;
            mCurrentZoom = mEndZoom;
            return false;
        }

        final float t = tRTC * 1f / mAnimationDurationMillis;
        mCurrentZoom = mStartZoom + (mDeltaZoom * Math.min(1, Math.max(0, mInterpolator.getInterpolation(t))));
        return true;
    }

    /**
     * Returns the current zoom level.
     *
     * @see android.widget.Scroller#getCurrX()
     */
    public float getCurrZoom() {
        return mCurrentZoom;
    }

    private void startZoomHelper(float startZoom, float endZoom, int duration) {
        mAnimationDurationMillis = duration;
        mStartRTC = SystemClock.elapsedRealtime();
        mStartZoom = startZoom;
        mEndZoom = endZoom;

        mDeltaZoom = mEndZoom - mStartZoom;

        mFinished = false;
        mCurrentZoom = mStartZoom;
    }

}
