package org.firstinspires.ftc.teamcode.vision.visionprocessors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

public class DrawRectangleProcessor implements VisionProcessor {
    public Rect rectangle = new Rect(20, 20, 50, 50);
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    private android.graphics.Rect makeGraphicsRectangle(Rect rectangle, float scaleBmpPxToCanvasPx){
        int left = Math.round(rectangle.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rectangle.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rectangle.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rectangle.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawRect(makeGraphicsRectangle(rectangle, scaleBmpPxToCanvasPx), rectPaint);
    }
}
