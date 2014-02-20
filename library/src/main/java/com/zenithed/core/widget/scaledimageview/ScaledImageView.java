/*
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
package com.zenithed.core.widget.scaledimageview;

import android.content.Context;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.Bitmap.Config;
import android.graphics.BitmapFactory;
import android.graphics.BitmapRegionDecoder;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.PointF;
import android.graphics.Rect;
import android.graphics.RectF;
import android.os.AsyncTask;
import android.support.v4.view.GestureDetectorCompat;
import android.support.v4.view.ViewCompat;
import android.util.AttributeSet;
import android.util.Log;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
import android.widget.Scroller;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Displays an image subsampled as necessary to avoid loading too much image data into memory. After a pinch to zoom in,
 * a set of image tiles subsampled at higher resolution are loaded and displayed over the base layer. During pinch and
 * zoom, tiles off screen or higher/lower resolution than required are discarded from memory.
 *
 * Tiles over 2048px are not used due to hardware rendering limitations.
 *
 * This view will not work very well with images that are far larger in one dimension than the other because the tile grid
 * for each subsampling level has the same number of rows as columns, so each tile has the same width:height ratio as
 * the source image. This could result in image data totalling several times the screen area being loaded.
 *
 * v prefixes - coordinates, translations and distances measured in screen (view) pixels
 * s prefixes - coordinates, translations and distances measured in source image pixels (scaled)
 */
public class ScaledImageView extends View {

    private static final String TAG = ScaledImageView.class.getSimpleName();


    protected static final int DEFAULT_SCROLL_DURATION = 250;
    protected static final float DEFAULT_SCALE = 1f;

    public static final float DEFAULT_SCALE_REAL_SIZE = 1f;
    public static final float DEFAULT_SCALE_VIEW_SIZE = 0f;

    // Max scale allowed (prevent infinite zoom)
    private float mMaxScale = 2F;

    private Context mContext;

    private Scroller mScroller;
    private GestureDetectorCompat mMovementGestureDetector;
    private ScaleGestureDetector mScaleGestureDetector;
    private final MovementGestureListener mMovementGestureListener =
                                                    new MovementGestureListener();
    private final ScaleGestureListener mScaleGestureListener =
                                                    new ScaleGestureListener();

    // Whether a ready notification has been sent to subclasses
    private boolean mImageReadyNotified = false;

    // Current scale and scale at start of zoom
    private float mDefaultScale = DEFAULT_SCALE_VIEW_SIZE;
    private float mScale = mDefaultScale;

    /**
     * Calculated as negative offset of the left and top corners
     * of the content rectangle relative to the viewport.
     */
    private PointF mTranslate;

    // Source image dimensions
    private int mContentWidth;
    private int mContentHeight;

    // Sample size used to display the whole image when fully zoomed out
    private int mFullImageSampleSize;

    // Tile decoder
    private BitmapRegionDecoder mBitmapRegionDecoder;
    // Map of zoom level to tile grid
    private Map<Integer, List<Tile>> mTileMap;

    private static Paint sDrawingPaint = new Paint();
    static {
        sDrawingPaint.setAntiAlias(true);
        sDrawingPaint.setFilterBitmap(true);
        sDrawingPaint.setDither(true);
    }

    private final Rect mViewPortRect = new Rect();


    public ScaledImageView(Context context) {
        super(context);
        initialize(context);
    }

    public ScaledImageView(Context context, AttributeSet attr) {
        super(context, attr);
        initialize(context);
    }

    public ScaledImageView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        initialize(context);
    }


    // =================================================
    // View's functional methods and lifecycle.
    // =================================================

    /**
     * Sets up gesture detection and finds the original image dimensions. Nothing else is done until onDraw() is called
     * because the view dimensions will normally be unknown when this method is called.
     */
    private void initialize(Context context) {
        mContext = context;

        mScroller = new Scroller(mContext);
        mMovementGestureDetector = new GestureDetectorCompat(mContext, mMovementGestureListener);
        mScaleGestureDetector = new ScaleGestureDetector(mContext, mScaleGestureListener);
        // by default the view must be enabled since it doesn't contain any content.
        setEnabled(false);
    }

    /**
     * Reset all state before setting/changing image.
     */
    private void reset(boolean resetScale) {
        setOnTouchListener(null);
        if (mBitmapRegionDecoder != null) {
            synchronized (mBitmapRegionDecoder) {
                mBitmapRegionDecoder.recycle();
            }
            mBitmapRegionDecoder = null;
        }
        if (mTileMap != null) {
            for (Map.Entry<Integer, List<Tile>> tileMapEntry : mTileMap.entrySet()) {
                for (Tile tile : tileMapEntry.getValue()) {
                    if (tile.bitmap != null) {
                        tile.bitmap.recycle();
                        tile.bitmap = null;
                    }
                }
            }
        }

        // sometime we just replace the image.
        if (resetScale) {
            mScale = mDefaultScale;
        }
        mTranslate = null;
        mContentWidth = 0;
        mContentHeight = 0;
        mFullImageSampleSize = 0;
        mTileMap = null;
        mImageReadyNotified = false;
        // by default the view must be enabled since it doesn't contain any content.
        setEnabled(false);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);

        mViewPortRect.set(getPaddingLeft(),
                getPaddingTop(),
                w - getPaddingRight(),
                h - getPaddingBottom());
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        // if the view is in the middle of loading the image it might not intercept touches.
        if (!isEnabled())
            return false;

        boolean retVal = mScaleGestureDetector.onTouchEvent(event);
        retVal = mMovementGestureDetector.onTouchEvent(event) || retVal;
        return retVal || this.handleDefaultTouchEvent(event);
    }

    /**
     * Handler for the touch event if the user has done performing the gesture.
     */
    private boolean handleDefaultTouchEvent(MotionEvent event) {
        switch (event.getAction()) {
            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_POINTER_UP:
            case MotionEvent.ACTION_POINTER_2_UP:
                // Trigger load of tiles now required
                refreshRequiredTiles(true);
                return true;
        }
        return false;
    }

    /**
     * Gesture detector listener that manages the behavior of the view's scroller.
     */
    private class MovementGestureListener extends GestureDetector.SimpleOnGestureListener {

        @Override
        public boolean onDown(MotionEvent e) {

            mScroller.forceFinished(true);
            ViewCompat.postInvalidateOnAnimation(ScaledImageView.this);
            return true;
        }

        @Override
        public boolean onScroll(MotionEvent e1, MotionEvent e2, float distanceX, float distanceY) {

            mScroller.computeScrollOffset();
            final int startX = mScroller.getCurrX();
            final int startY = mScroller.getCurrY();

            mScroller.startScroll(startX, startY, (int) distanceX, (int) distanceY, 0);
            ViewCompat.postInvalidateOnAnimation(ScaledImageView.this);

            return true;
        }

        @Override
        public boolean onFling(MotionEvent e1, MotionEvent e2, float velocityX, float velocityY) {

            final int velX = (int) -velocityX;
            final int velY = (int) -velocityY;

            mScroller.computeScrollOffset();
            mScroller.fling(mScroller.getCurrX(),
                            mScroller.getCurrY(),
                            velX,
                            velY,
                            0, (int) (mContentWidth * mScale),
                            0, (int) (mContentHeight * mScale));
            ViewCompat.postInvalidateOnAnimation(ScaledImageView.this);
            return true;
        }

    }

    /**
     * Gesture detector listener that manages the behavior of the view's zoom.
     */
    private class ScaleGestureListener extends ScaleGestureDetector.SimpleOnScaleGestureListener {

        float initialScale;
        PointF initialPoint;

        @Override
        public boolean onScaleBegin(ScaleGestureDetector detector) {

            initialScale = mScale;
            initialPoint = new PointF(Math.abs(mTranslate.x) + detector.getFocusX(),
                                            Math.abs(mTranslate.y) + detector.getFocusY());

            return true;
        }

        @Override
        public boolean onScale(ScaleGestureDetector detector) {

            mScale = Math.min(mMaxScale, detector.getScaleFactor() * mScale);

            /**
             * Every point is related to the original selected on, the only differences
             * when moving with the scale motions is the tendency of drawing content scale - initialScale (original mScale).
             */
            PointF currentPoint = new PointF((initialPoint.x) * (mScale / initialScale),
                                             (initialPoint.y) * (mScale / initialScale));

            // gets the top and the left points of the viewport.
            final int pointX = (int) (currentPoint.x - detector.getFocusX());
            final int pointY = (int) (currentPoint.y - detector.getFocusY());

            mScroller.computeScrollOffset();
            final int startX = mScroller.getCurrX();
            final int startY = mScroller.getCurrY();

            mScroller.startScroll(startX, startY, pointX - startX, pointY - startY, 0);

            fitToBounds();
            refreshRequiredTiles(false);
            ViewCompat.postInvalidateOnAnimation(ScaledImageView.this);
            return true;
        }
    }

    @Override
    public void computeScroll() {
        super.computeScroll();

        if (isEnabled() && mContentWidth > 0 && mContentHeight > 0 && mScroller.computeScrollOffset()) {
            final int currX = mScroller.getCurrX();
            final int currY = mScroller.getCurrY();

            final int maxOffsetX = (int) (mContentWidth * mScale) - getWidth();
            final int maxOffsetY = (int) (mContentHeight * mScale) - getHeight();

            if (mScroller.getFinalX() > maxOffsetX) {
                mScroller.setFinalX(maxOffsetX);
            }

            if (mScroller.getFinalY() > maxOffsetY) {
                mScroller.setFinalY(maxOffsetY);
            }

            mTranslate.x = currX * -1;
            mTranslate.y = currY * -1;

            fitToBounds();
            refreshRequiredTiles(false);
            ViewCompat.postInvalidateOnAnimation(ScaledImageView.this);
        }

    }

    /**
     * Draw method should not be called until the view has dimensions so the first calls are used as triggers to calculate
     * the scaling and tiling required. Once the view is setup, tiles are displayed as they are loaded.
     */
    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        // If image or view dimensions are not known yet, abort.
        if (mContentWidth == 0 || mContentHeight == 0 ||
                mBitmapRegionDecoder == null || getWidth() == 0 || getHeight() == 0) {
            return;
        }

        // On first render with no tile map ready, initialise it and kick off async base image loading.
        if (mTileMap == null) {
            initialiseBaseLayer();
            return;
        }

        // On first display of base image set up position, and in other cases make sure scale is correct.
        fitToBounds();

        // Everything is set up and coordinates are valid. Inform subclasses.
        if (!mImageReadyNotified) {
            setEnabled(true);
            onImageReady();
            mImageReadyNotified = true;
        }

        canvas.clipRect(mViewPortRect);

        // Optimum sample size for current scale
        final int sampleSize = Math.min(mFullImageSampleSize,
                calculateInSampleSize((int) (mContentWidth * mScale), (int) (mContentHeight * mScale)));

        // First check for missing tiles - if there are any we need the base layer underneath to avoid gaps
        boolean hasMissingTiles = false;
        for (Map.Entry<Integer, List<Tile>> tileMapEntry : mTileMap.entrySet()) {
            if (tileMapEntry.getKey() == sampleSize) {
                for (Tile tile : tileMapEntry.getValue()) {
                    if (tile.visible && (tile.loading || tile.bitmap == null)) {
                        hasMissingTiles = true;
                    }
                }
            }
        }

        // Render all loaded tiles. LinkedHashMap used for bottom up rendering - lower res tiles underneath.
        for (Map.Entry<Integer, List<Tile>> tileMapEntry : mTileMap.entrySet()) {
            if (tileMapEntry.getKey() == sampleSize || hasMissingTiles) {
                for (Tile tile : tileMapEntry.getValue()) {
                    if (!tile.loading && tile.bitmap != null) {
                        canvas.drawBitmap(tile.bitmap, null, convertRect(sourceToViewRect(tile.sRect)), sDrawingPaint);
                    }
                }
            }
        }

    }


    // =================================================
    // Private helper methods.
    // =================================================

    /**
     * Called on first draw when the view has dimensions. Calculates the initial sample size and starts async loading of
     * the base layer image - the whole source subsampled as necessary.
     */
    private synchronized void initialiseBaseLayer() {

        fitToBounds();

        // Load double resolution - next level will be split into four tiles and at the center all four are required,
        // so don't bother with tiling until the next level 16 tiles are needed.
        mFullImageSampleSize = calculateInSampleSize((int)(mContentWidth * mScale), (int)(mContentHeight * mScale));
        if (mFullImageSampleSize > 1) {
            mFullImageSampleSize /= 2;
        }

        initialiseTileMap();

        List<Tile> baseGrid = mTileMap.get(mFullImageSampleSize);
        for (Tile baseTile : baseGrid) {
            BitmapTileTask task = new BitmapTileTask(this, mBitmapRegionDecoder, baseTile);
            task.execute();
        }

    }

    /**
     * Loads the optimum tiles for display at the current scale and translate, so the screen can be filled with tiles
     * that are at least as high resolution as the screen. Frees up bitmaps that are now off the screen.
     * @param load Whether to load the new tiles needed. Use false while scrolling/panning for performance.
     */
    private void refreshRequiredTiles(boolean load) {
        final int sampleSize = Math.min(mFullImageSampleSize,
                calculateInSampleSize((int) (mScale * mContentWidth), (int) (mScale * mContentHeight)));
        final RectF vVisRect = new RectF(0, 0, getWidth(), getHeight());
        final RectF sVisRect = viewToSourceRect(vVisRect);

        // Load tiles of the correct sample size that are on screen. Discard tiles off screen, and those that are higher
        // resolution than required, or lower res than required but not the base layer, so the base layer is always present.
        for (Map.Entry<Integer, List<Tile>> tileMapEntry : mTileMap.entrySet()) {
            for (Tile tile : tileMapEntry.getValue()) {
                if (tile.sampleSize < sampleSize || (tile.sampleSize > sampleSize && tile.sampleSize != mFullImageSampleSize)) {
                    tile.visible = false;
                    if (tile.bitmap != null) {
                        tile.bitmap.recycle();
                        tile.bitmap = null;
                    }
                }
                if (tile.sampleSize == sampleSize) {
                    if (RectF.intersects(sVisRect, convertRect(tile.sRect))) {
                        tile.visible = true;
                        if (!tile.loading && tile.bitmap == null && load) {
                            BitmapTileTask task = new BitmapTileTask(this, mBitmapRegionDecoder, tile);
                            task.execute();
                        }
                    } else if (tile.sampleSize != mFullImageSampleSize) {
                        tile.visible = false;
                        if (tile.bitmap != null) {
                            tile.bitmap.recycle();
                            tile.bitmap = null;
                        }
                    }
                } else if (tile.sampleSize == mFullImageSampleSize) {
                    tile.visible = true;
                }
            }
        }

    }

    /**
     * Calculates sample size to fit the source image in given bounds.
     */
    private int calculateInSampleSize(int reqWidth, int reqHeight) {
        // Raw height and width of image
        int inSampleSize = 1;
        if (reqWidth == 0 || reqHeight == 0) {
            return 32;
        }

        if (mContentHeight > reqHeight || mContentWidth > reqWidth) {

            // Calculate ratios of height and width to requested height and width
            final int heightRatio = Math.round((float) mContentHeight / (float) reqHeight);
            final int widthRatio = Math.round((float) mContentWidth / (float) reqWidth);

            // Choose the smallest ratio as inSampleSize value, this will guarantee
            // a final image with both dimensions larger than or equal to the
            // requested height and width.
            inSampleSize = heightRatio < widthRatio ? heightRatio : widthRatio;
        }

        // We want the actual sample size that will be used, so round down to nearest power of 2.
        int power = 1;
        while (power * 2 < inSampleSize) {
            power = power * 2;
        }

        return power;
    }

    /**
     * Adjusts scale and translate values to keep scale within the allowed range and the image on screen. Minimum scale
     * is set to one dimension fills the view and the image is centered on the other dimension.
     */
    private void fitToBounds() {
        if (mTranslate == null) {
            mTranslate = new PointF(0, 0);
        }

        float minScale = Math.min(getWidth() / (float) mContentWidth, getHeight() / (float) mContentHeight);
        mScale = Math.max(minScale, mScale);
        mScale = Math.min(mMaxScale, mScale);

        float scaleWidth = mScale * mContentWidth;
        float scaleHeight = mScale * mContentHeight;

        mTranslate.x = Math.max(mTranslate.x, getWidth() - scaleWidth);
        mTranslate.y = Math.max(mTranslate.y, getHeight() - scaleHeight);

        float maxTx = Math.max(0, (getWidth() - scaleWidth) / 2);
        float maxTy = Math.max(0, (getHeight() - scaleHeight) / 2);

        mTranslate.x = Math.min(mTranslate.x, maxTx);
        mTranslate.y = Math.min(mTranslate.y, maxTy);
    }

    /**
     * Once source image and view dimensions are known, creates a map of sample size to tile grid.
     */
    private void initialiseTileMap() {
        this.mTileMap = new LinkedHashMap<Integer, List<Tile>>();
        int sampleSize = mFullImageSampleSize;
        int tilesPerSide = 1;
        while (true) {
            int sTileWidth = mContentWidth /tilesPerSide;
            int sTileHeight = mContentHeight /tilesPerSide;
            int subTileWidth = sTileWidth/sampleSize;
            int subTileHeight = sTileHeight/sampleSize;
            while (subTileWidth > 2048 || subTileHeight > 2048) {
                tilesPerSide *= 2;
                sTileWidth = mContentWidth /tilesPerSide;
                sTileHeight = mContentHeight /tilesPerSide;
                subTileWidth = sTileWidth/sampleSize;
                subTileHeight = sTileHeight/sampleSize;
            }
            List<Tile> tileGrid = new ArrayList<Tile>(tilesPerSide * tilesPerSide);
            for (int x = 0; x < tilesPerSide; x++) {
                for (int y = 0; y < tilesPerSide; y++) {
                    Tile tile = new Tile();
                    tile.sampleSize = sampleSize;
                    tile.sRect = new Rect(
                            x * sTileWidth,
                            y * sTileHeight,
                            (x + 1) * sTileWidth,
                            (y + 1) * sTileHeight
                    );
                    tileGrid.add(tile);
                }
            }
            mTileMap.put(sampleSize, tileGrid);
            tilesPerSide = (tilesPerSide == 1) ? 4 : tilesPerSide * 2;
            if (sampleSize == 1) {
                break;
            } else {
                sampleSize /= 2;
            }
        }
    }

    /**
     * Called by worker task when decoder is ready and image size is known.
     */
    private void onImageInited(BitmapRegionDecoder decoder, int sWidth, int sHeight) {
        this.mBitmapRegionDecoder = decoder;
        this.mContentWidth = sWidth;
        this.mContentHeight = sHeight;
        invalidate();
    }

    /**
     * Called by worker task when a tile has loaded. Redraws the view.
     */
    private void onTileLoaded() {
        invalidate();
    }

    /**
     * Async task used to get image details without blocking the UI thread.
     */
    private static class BitmapInitTask extends AsyncTask<Void, Void, Point> {
        private final WeakReference<ScaledImageView> viewRef;
        private final WeakReference<Context> contextRef;
        private final String source;
        private final boolean sourceIsAsset;
        private WeakReference<BitmapRegionDecoder> decoderRef;

        public BitmapInitTask(ScaledImageView view, Context context, String source, boolean sourceIsAsset) {
            this.viewRef = new WeakReference<ScaledImageView>(view);
            this.contextRef = new WeakReference<Context>(context);
            this.source = source;
            this.sourceIsAsset = sourceIsAsset;
        }

        @Override
        protected Point doInBackground(Void... params) {
            try {
                if (viewRef != null && contextRef != null) {
                    Context context = contextRef.get();
                    if (context != null) {
                        BitmapRegionDecoder decoder;
                        if (sourceIsAsset) {
                            decoder = BitmapRegionDecoder.newInstance(context.getAssets().open(source, AssetManager.ACCESS_RANDOM), true);
                        } else {
                            decoder = BitmapRegionDecoder.newInstance(source, true);
                        }
                        decoderRef = new WeakReference<BitmapRegionDecoder>(decoder);
                        return new Point(decoder.getWidth(), decoder.getHeight());
                    }
                }
            } catch (Exception e) {
                Log.e(TAG, "Failed to initialise bitmap decoder", e);
            }
            return null;
        }

        @Override
        protected void onPostExecute(Point point) {
            if (viewRef != null && decoderRef != null) {
                final ScaledImageView scaledImageView = viewRef.get();
                final BitmapRegionDecoder decoder = decoderRef.get();
                if (scaledImageView != null && decoder != null && point != null) {
                    scaledImageView.onImageInited(decoder, point.x, point.y);
                }
            }
        }
    }

    /**
     * Async task used to load images without blocking the UI thread.
     */
    private static class BitmapTileTask extends AsyncTask<Void, Void, Bitmap> {
        private final WeakReference<ScaledImageView> viewRef;
        private final WeakReference<BitmapRegionDecoder> decoderRef;
        private final WeakReference<Tile> tileRef;

        public BitmapTileTask(ScaledImageView view, BitmapRegionDecoder decoder, Tile tile) {
            this.viewRef = new WeakReference<ScaledImageView>(view);
            this.decoderRef = new WeakReference<BitmapRegionDecoder>(decoder);
            this.tileRef = new WeakReference<Tile>(tile);
            tile.loading = true;
        }

        @Override
        protected Bitmap doInBackground(Void... params) {
            try {
                if (decoderRef != null && tileRef != null && viewRef != null) {
                    final BitmapRegionDecoder decoder = decoderRef.get();
                    final Tile tile = tileRef.get();
                    if (decoder != null && tile != null && !decoder.isRecycled()) {
                        synchronized (decoder) {
                            BitmapFactory.Options options = new BitmapFactory.Options();
                            options.inSampleSize = tile.sampleSize;
                            options.inPreferredConfig = Config.RGB_565;
                            return decoder.decodeRegion(tile.sRect, options);
                        }
                    }
                }
            } catch (Exception e) {
                Log.e(TAG, "Failed to decode tile", e);
            }
            return null;
        }

        @Override
        protected void onPostExecute(Bitmap bitmap) {
            if (viewRef != null && tileRef != null && bitmap != null) {
                final ScaledImageView scaledImageView = viewRef.get();
                final Tile tile = tileRef.get();
                if (scaledImageView != null && tile != null) {
                    tile.bitmap = bitmap;
                    tile.loading = false;
                    scaledImageView.onTileLoaded();
                }
            }
        }
    }

    private static class Tile {

        private Rect sRect;
        private int sampleSize;
        private Bitmap bitmap;
        private boolean loading;
        private boolean visible;

    }

    /**
     * Convert source rect to screen rect.
     */
    private RectF sourceToViewRect(Rect sRect) {
        return sourceToViewRect(convertRect(sRect));
    }

    /**
     * Convert source rect to screen rect.
     */
    private RectF sourceToViewRect(RectF sRect) {
        PointF vLT = sourceToViewCoord(new PointF(sRect.left, sRect.top));
        PointF vRB = sourceToViewCoord(new PointF(sRect.right, sRect.bottom));
        return new RectF(vLT.x, vLT.y, vRB.x, vRB.y);
    }

    /**
     * Convert screen rect to source rect.
     */
    private RectF viewToSourceRect(RectF vRect) {
        PointF sLT = viewToSourceCoord(new PointF(vRect.left, vRect.top));
        PointF sRB = viewToSourceCoord(new PointF(vRect.right, vRect.bottom));
        return new RectF(sLT.x, sLT.y, sRB.x, sRB.y);
    }

    /**
     * Int to float rect conversion.
     */
    private RectF convertRect(Rect rect) {
        return new RectF(rect.left, rect.top, rect.right, rect.bottom);
    }

    /**
     * Float to int rect conversion.
     */
    private Rect convertRect(RectF rect) {
        return new Rect((int)rect.left, (int)rect.top, (int)rect.right, (int)rect.bottom);
    }

    private void scrollToHelper(float sx, float sy, int duration) {
        final int portCenterX = getWidth() / 2;
        final int portCenterY = getHeight() / 2;

        mScroller.computeScrollOffset();
        final int startX = mScroller.getCurrX();
        final int startY = mScroller.getCurrY();

        final int deltaX = ((startX + portCenterX) - (int) sx) * -1;
        final int deltaY = ((startY + portCenterY) - (int) sy) * -1;

        mScroller.startScroll(startX, startY, deltaX, deltaY, duration);
        invalidate();
    }

    private void setImageFileHelper(String extFile, boolean resetScale) {
        reset(resetScale);
        BitmapInitTask task = new BitmapInitTask(this, getContext(), extFile, false);
        task.execute();
        invalidate();
    }

    private void setImageAssetHelper(String assetName, boolean resetScale) {
        reset(resetScale);
        BitmapInitTask task = new BitmapInitTask(this, getContext(), assetName, true);
        task.execute();
        invalidate();
    }


    // =================================================
    // Public methods.
    // =================================================

    /**
     * Returns the source point at the center of the view.
     */
    protected PointF getCenter() {
        int mX = getWidth()/2;
        int mY = getHeight()/2;
        return viewToSourceCoord(mX, mY);
    }

    /**
     * Returns the current scale value.
     */
    protected float getScale() {
        return mScale;
    }

    protected float getScaleImageToViewScale() {
        final float imageAspect = mContentWidth / mContentHeight;
        final float viewAspect = getWidth() / getHeight();

        if (imageAspect < viewAspect) {
            // scale by width ratio.
            return getWidth() / (mContentWidth * mScale);

        } else {
            // scale by height ratio.
            return getHeight() / (mContentHeight * mScale);
        }
    }

    protected float getContentWidth() {
        return mContentWidth * mScale;
    }

    protected float getContentHeight() {
        return mContentHeight * mScale;
    }

    /**
     * Retrieves the current view port translation x.
     * Note that is the value is negative, the content port is within the view port (smaller than).
     * @return distance in pixels between the view port's left corner to the content's left corner.
     */
    protected float getViewPortTranslateX() {
        return mTranslate != null ? (mTranslate.x * -1) : 0;
    }

    /**
     * Retrieves the current view port translation y.
     * Note that is the value is negative, the content port is within the view port (smaller than).
     * @return distance in pixels between the view port's top corner to the content's top corner.
     */
    protected float getViewPortTranslateY() {
        return mTranslate != null ? (mTranslate.y * -1) : 0;
    }

    /**
     * Subclasses can override this method to be informed when the view is set up and ready for rendering, so they can
     * skip their own rendering until the base layer (and its scale and translate) are known.
     */
    protected void onImageReady() {}


    /**
     * Convert screen coordinate to source coordinate.
     */
    public PointF viewToSourceCoord(PointF vxy) {
        return viewToSourceCoord(vxy.x, vxy.y);
    }

    /**
     * Convert screen coordinate to source coordinate.
     */
    public PointF viewToSourceCoord(float vx, float vy) {
        if (mTranslate == null) {
            return null;
        }
        float sx = (vx - mTranslate.x)/ mScale;
        float sy = (vy - mTranslate.y)/ mScale;
        return new PointF(sx, sy);
    }

    /**
     * Convert source coordinate to screen coordinate.
     */
    public PointF sourceToViewCoord(PointF sxy) {
        return sourceToViewCoord(sxy.x, sxy.y);
    }

    /**
     * Convert source coordinate to screen coordinate.
     */
    public PointF sourceToViewCoord(float sx, float sy) {
        float vx = (sx * mScale) + mTranslate.x;
        float vy = (sy * mScale) + mTranslate.y;
        return new PointF(vx, vy);
    }

    /**
     * Scrolls and centers the view in the given point.
     * @param sx coordination X of the point relative to the image's size.
     * @param sy coordination X of the point relative to the image's size.
     */
    public void scrollTo(float sx, float sy) {
        scrollToHelper(sx, sy, DEFAULT_SCROLL_DURATION);
    }

    /**
     * Scrolls and centers the view in the given point.
     * @param sx coordination X of the point relative to the image's size.
     * @param sy coordination X of the point relative to the image's size.
     * @param duration of the animation, any value from 0 and higher.
     */
    public void scrollTo(float sx, float sy, int duration) {
        scrollToHelper(sx, sy, duration);
    }


    /**
     * Call from subclasses to find whether the view is initialised and ready for rendering tiles.
     */
    public boolean isImageReady() {
        return mImageReadyNotified;
    }

    /**
     * Display an image from a file in internal or external storage
     * @param extFile URI of the file to display
     */
    public void setImageFile(String extFile) {
        setImageFileHelper(extFile, true);
    }

    /**
     * Display an image from a file in internal or external storage
     * @param extFile URI of the file to display.
     * @param resetScale determines if to set the image's initial scale to the default value or not.
     * @throws java.io.IOException
     */
    public void setImageFile(String extFile, boolean resetScale) {
        setImageFileHelper(extFile, resetScale);
    }

    /**
     * Display an image from a file in assets.
     * @param assetName asset name.
     */
    public void setImageAsset(String assetName) {
        setImageAssetHelper(assetName, true);
    }

    /**
     * Display an image from a file in assets.
     * @param assetName asset name.
     * @param resetScale  determines if to set the image's initial scale to the default value or not.
     * @throws java.io.IOException
     */
    public void setImageAsset(String assetName, boolean resetScale) {
        setImageAssetHelper(assetName, resetScale);
    }

    /**
     * Set the maximum scale allowed
     */
    public void setMaxScale(float maxScale) {
        this.mMaxScale = maxScale;
    }

    /**
     * Sets the default scale value of the view. Call this
     * before setting the desired image to load.
     * @param defaultScale to be applied when a new image is loaded.
     */
    public void setDefaultScale(float defaultScale) {
        mDefaultScale = defaultScale;
        mScale = mDefaultScale;
    }

}
