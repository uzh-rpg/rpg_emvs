#include <mapper_emvs/median_filtering.hpp>
#include <glog/logging.h>


#define FORWARD     1
#define BACKWARD  (-1)

int compute_median_histogram(const int h[], int num_elements)
{
  int middle = (num_elements+1)/2;
  int m, n;
  for(m=0,n=0; n < 256; n++)
  {
    m += h[n];
    if(m >= middle)
      break;
  }
  return n;
}


inline int get_value(const cv::Mat& img, const cv::Mat& mask, int row, int col)
{
  if(row >= 0 && col >= 0 && row < img.rows && col < img.cols && mask.at<uint8_t>(row,col) > 0)
    return img.at<uchar>(row,col);
  else
    return -1;
}


// 2D median filter. Complexity: O(p) where p is the patch size
// "T. Huang, G. Yang, and G. Tang, "A Fast Two-Dimensional Median Filtering Algorithm"
// Code adapted from: http://www.sergejusz.com/engineering_tips/median_filter.htm
void huangMedianFilter(const cv::Mat& img, cv::Mat& out_img, const cv::Mat& mask, int patch_size)
{
  CHECK(img.data);
  CHECK(mask.data);
  CHECK_GT(img.rows, 0);
  CHECK_GT(img.cols, 0);
  CHECK_EQ(img.rows, mask.rows);
  CHECK_EQ(img.cols, mask.cols);
  CHECK_EQ(img.type(), CV_8U);
  CHECK_EQ(patch_size % 2, 1);

  out_img = img.clone();

  int p = patch_size / 2;

  int med;
  int prev, next;
  int h[256];
  int direction = FORWARD;
  int row1, row2, col1, col2;
  int row, col, r, c;

  memset(h, 0, sizeof(h));

  int num_elements = 0;

  // Histogram For (0,0)-element
  for(row = -p; row <= p; row++)
  {
    for(col = -p; col <= p; col++)
    {
      int val = get_value(img, mask, row, col);
      if(val >= 0)
      {
        h[val]++;
        num_elements++;
      }
    }
  }

  // Median
  med = compute_median_histogram(h, num_elements);

  // Now, Median Is Defined For (0,0)-element
  // Begin Scanning: direction - FORWARD
  out_img.at<uchar>(0,0)=med;

  // main loop
  for(col=1, row=0; row<img.rows; row++)
  {
    // Prepare to Horizontal Scanning
    row1=row-p;
    row2=row+p;

    for(; col>=0 && col<img.cols; col += direction)
    {
      // Determine Previous and Next Columns
      // Pay Attention To Defined Direction !!!
      prev = col-direction*(p+1);
      next = col+direction*p;

      // Now Change Old Histogram
      // New Histogram
      // delete previous

      for(r=row1; r<=row2; r++)
      {
        int value_out = get_value(img, mask, r, prev);
        int value_in = get_value(img, mask, r, next);
        if(value_in >= 0 && value_out == value_in)
          continue;
        if(value_out >= 0)
        {
          h[value_out]--;
          num_elements--;
        }
        if(value_in >= 0)
        {
          h[value_in]++;
          num_elements++;
        }
      }

      // Update new median
      med = compute_median_histogram(h, num_elements);
      out_img.at<uchar>(row,col) = med;
    } // end of column loop

    if(row == img.rows-1)
      return;

    // go back to the last/first pixel of the line
    col -= direction;
    // change direction to the opposite
    direction *= -1;

    // Shift Down One Line
    prev = row1;
    next = row2+1;

    col1 = col - p;
    col2 = col + p;

    for(c=col1; c<=col2; c++)
    {
      int value_out = get_value(img, mask, prev, c);
      int value_in = get_value(img, mask, next, c);
      if(value_in >= 0 && value_out == value_in)
        continue;
      if(value_out >= 0)
      {
        h[value_out]--;
        num_elements--;
      }
      if(value_in >= 0)
      {
        h[value_in]++;
        num_elements++;
      }
    }

    med = compute_median_histogram(h, num_elements);
    out_img.at<uchar>(row+1,col) = med;
    col += direction;
  }
}
