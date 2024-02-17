package frc.lib.helpers;

public class BilinearInterpolator
{
    private final double[] xSamplePoints;
    private final double[] ySamplePoints;

    private final double[][] samples;

    public BilinearInterpolator(double[] xSamplePoints, double[] ySamplePoints, double[][] samples)
    {
        ExceptionHelpers.Assert(xSamplePoints != null && xSamplePoints.length != 0, "xSamplePoints cannot be null!");
        ExceptionHelpers.Assert(ySamplePoints != null && ySamplePoints.length != 0, "ySamplePoints cannot be null!");
        ExceptionHelpers.Assert(samples != null, "samples cannot be null!");

        this.xSamplePoints = xSamplePoints;
        this.ySamplePoints = ySamplePoints;

        this.samples = samples;

        this.verifySamples();
    }

    private void verifySamples()
    {
        ExceptionHelpers.Assert(this.samples.length == this.xSamplePoints.length, "x samples length (%d) doesn't match x sample points length (%d)", this.samples.length, this.xSamplePoints.length);
        for (int x = 0; x < this.xSamplePoints.length; x++)
        {
            ExceptionHelpers.Assert(this.samples[x].length == this.ySamplePoints.length, "y samples length (%d) doesn't match y sample points length (%d) on row %d", this.samples[x].length, this.ySamplePoints.length, x);
        }
    }

    public double sample(double x, double y)
    {
        int row1 = 0;
        int row2 = -1;
        for (int i = 1; i < this.xSamplePoints.length; i++)
        {
            if (this.xSamplePoints[i] > x)
            {
                row2 = i;
                break;
            }
            else if (this.xSamplePoints[i] <= x)
            {
                row1 = i;
            }
        }

        int col1 = 0;
        int col2 = -1;
        for (int i = 1; i < this.ySamplePoints.length; i++)
        {
            if (this.ySamplePoints[i] > y)
            {
                row2 = i;
                break;
            }
            else if (this.ySamplePoints[i] <= y)
            {
                row1 = i;
            }
        }

        if (col1 != -1 && col2 != -1 && row1 != -1 && row2 != -1)
        {
            double x1 = this.xSamplePoints[row1];
            double x2 = this.xSamplePoints[row2];
            double y1 = this.ySamplePoints[col1];
            double y2 = this.ySamplePoints[col2];

            double sample11 = this.samples[row1][col1];
            double sample12 = this.samples[row1][col2];
            double sample21 = this.samples[row2][col1];
            double sample22 = this.samples[row2][col2];

            return (1 / ((x2 - x1) * (y2 - y1))) *
                (sample11 * (x2 - x) * (y2 - y) +
                 sample21 * (x - x1) * (y2 - y) +
                 sample12 * (x2 - x) * (y - y1) +
                 sample22 * (x - x1) * (y - y1));
        }

        throw new RuntimeException("Couldn't find row/col!!");
    }
}
