#include "camera_pipeline.hpp"

typedef struct surrounding_average {
float val;
int num_neighbors;
} surrounding_average;

surrounding_average vertical_neighbors_average(
        CameraSensorData<float>* raw_data, int width, int height, int row,
        int col, int start, int end, int stride) {
  surrounding_average sa = { 0.f, 0 };

  for (int i = start; i < end; i+= stride) {
    if (row - i >= 0) {
      sa.val += raw_data->data(row - i, col);
      sa.num_neighbors++;
    }
    if (row + i < height) {
      sa.val += raw_data->data(row + i, col);
      sa.num_neighbors++;
    }
  }

  return sa;
}

surrounding_average horizontal_neighbors_average(
        CameraSensorData<float>* raw_data, int width, int height, int row,
        int col, int start, int end, int stride) {
  surrounding_average sa = { 0.f, 0 };

  for (int i = start; i < end; i+= stride) {
    if (col - i >= 0) {
      sa.val += raw_data->data(row, col - i);
      sa.num_neighbors++;
    }
    if (col + i < width) {
      sa.val += raw_data->data(row, col + i);
      sa.num_neighbors++;
    }
  }

  return sa;
}

surrounding_average diagonal_neighbors_average(
        CameraSensorData<float>* raw_data, int width, int height, int row,
        int col, int start, int end, int stride) {
  surrounding_average sa = { 0.f, 0 };

  for (int i = start; i < end; i+= stride) {
    if (row - i >= 0 && col - i >= 0) {
      sa.val += raw_data->data(row-i, col-i);
      sa.num_neighbors++;
    }
    if (row - i >= 0 && col + i < width) {
      sa.val += raw_data->data(row-i, col+i);
      sa.num_neighbors++;
    }
    if (row + i < height && col - i >= 0) {
      sa.val += raw_data->data(row+i, col-i);
      sa.num_neighbors++;
    }
    if (row + i < height && col + i < width) {
      sa.val += raw_data->data(row+i, col+i);
      sa.num_neighbors++;
    }
  }

  return sa;
}

surrounding_average nondiagonal_neighbors_average(
        CameraSensorData<float>* raw_data, int width, int height, int row,
        int col, int start, int end, int stride) {
  surrounding_average sa = { 0.f, 0 };

  for (int i = start; i < end; i+= stride) {
    if (row - i >= 0) {
      sa.val += raw_data->data(row-i, col);
      sa.num_neighbors++;
    }
    if (row + i < height) {
      sa.val += raw_data->data(row+i, col);
      sa.num_neighbors++;
    }
    if (col - i >= 0) {
      sa.val += raw_data->data(row, col-i);
      sa.num_neighbors++;
    }
    if (col + i < width) {
      sa.val += raw_data->data(row, col+i);
      sa.num_neighbors++;
    }
  }

  return sa;
}

surrounding_average all_neighbors_average(
        CameraSensorData<float>* raw_data, int width, int height, int row,
        int col, int start, int end, int stride) {
  surrounding_average sa = { 0.f, 0 };

  for (int i = start; i < end; i+= stride) {
    for (int j = 0; j < 2 * i; j++) {
      // top side
      if (row - i >= 0 && col - i + j >= 0 && col - i + j < width) {
        sa.val += raw_data->data(row-i, col-i+j);
        sa.num_neighbors++;
      }
      // right side
      if (row - i + j >= 0 && row - i + j < height && col + i < width) {
        sa.val += raw_data->data(row - i + j, col + i);
        sa.num_neighbors++;
      }
      // bottom side
      if (row + i < height && col + i - j >= 0 && col + i - j < width) {
        sa.val += raw_data->data(row + i, col + i - j);
        sa.num_neighbors++;
      }
      // left side
      if (row + i - j >= 0 && row + i - j < height && col - i >= 0) {
        sa.val += raw_data->data(row + i - j, col - i);
        sa.num_neighbors++;
      }
    } 
  }

  return sa;
}

std::unique_ptr<Image<RgbPixel>> CameraPipeline::ProcessShot() const {
    
  // BEGIN: CS348K STUDENTS MODIFY THIS CODE

  // get a "dark frame"
  sensor_->SetLensCap(true);
  const int width = sensor_->GetSensorWidth();
  const int height = sensor_->GetSensorHeight();
  auto dark_frame = sensor_->GetSensorData(0, 0, width, height);

  // put the lens cap on if you'd like to measure a "dark frame"
  sensor_->SetLensCap(false);
    
  // grab RAW pixel data from sensor
  auto raw_data = sensor_->GetSensorData(0, 0, width, height);

  // correct for bad pixels
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      const auto dark_val = dark_frame->data(row, col);

      if (dark_val >= .99) {
        // need to replace this value with average of similar neighbors
        if ((row % 2) == (col % 2)) {
          // this is a green pixel
          surrounding_average sa_diag = diagonal_neighbors_average(
              raw_data.get(), width, height, row, col, 1, 2, 1);
          raw_data->data(row, col) = sa_diag.val / sa_diag.num_neighbors;
        } else { 
          // this is a green pixel, get diag and non-diag neighbors from
          // two pixels away
          surrounding_average sa_diag = diagonal_neighbors_average(
              raw_data.get(), width, height, row, col, 2, 3, 1);
          surrounding_average sa_nondiag = nondiagonal_neighbors_average(
              raw_data.get(), width, height, row, col, 2, 3, 1);
          raw_data->data(row, col) = (sa_diag.val + sa_nondiag.val) /
              (sa_diag.num_neighbors + sa_nondiag.num_neighbors);
        }
      }
    }
  }
    
  // In this function you should implement your full RAW image processing pipeline.
  //   (1) Demosaicing
  //   (2) Address sensing defects such as bad pixels and image noise.
  //   (3) Apply local tone mapping based on the local laplacian filter or exposure fusion.
  //   (4) gamma correction
    
  // allocate 3-channel RGB output buffer to hold the results after processing 
  std::unique_ptr<Image<RgbPixel>> image(new Image<RgbPixel>(width, height));
  
  // The starter code copies the raw data from the sensor to all rgb
  // channels. This results in a gray image that is just a
  // visualization of the sensor's contents.
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      const auto val = raw_data->data(row, col);
      auto& pixel = (*image)(row, col);

      if (row == 0 && col == 0) {
        printf("%f\n", val * 255.f);
      }

      if ((row % 2) == (col % 2)) {
        // This is a green pixel
        pixel.g = val * 255.f;
        
        // Calculate average of neighboring vertical and neighboring
        // horizontal photos
        surrounding_average sa_vert = vertical_neighbors_average(
                raw_data.get(), width, height, row, col, 1, 2, 1);
        surrounding_average sa_hor = horizontal_neighbors_average(
                raw_data.get(), width, height, row, col, 1, 2, 1);

        if ((row % 2) == 0) {
          // vert is blue, hor is red
          pixel.b = sa_vert.val / sa_vert.num_neighbors * 255.f;
          pixel.r = sa_hor.val / sa_hor.num_neighbors * 255.f;
        } else {
          // vert is red, hor is blue
          pixel.r = sa_vert.val / sa_vert.num_neighbors * 255.f;
          pixel.b = sa_hor.val / sa_hor.num_neighbors * 255.f;
        }

      } else {
        // This is a blue or red pixel
        
        surrounding_average sa_diags = diagonal_neighbors_average(
            raw_data.get(), width, height, row, col, 1, 2, 1);
        surrounding_average sa_nondiags = nondiagonal_neighbors_average(
            raw_data.get(), width, height, row, col, 1, 2, 1);

        float diags = sa_diags.val / sa_diags.num_neighbors; 
        float nondiags = sa_nondiags.val / sa_nondiags.num_neighbors; 

        if ((row % 2) == 0) {
          // this is a red pixel
          pixel.r = val * 255.f;

          // diagonal values are blue
          pixel.b = diags * 255.f;
        } else {
          // this is a blue pixel
          pixel.b = val * 255.f;

          // diagonal values are red
          pixel.r = diags * 255.f;
        }

        // non-diagonal values are always green
        pixel.g = nondiags * 255.f;
      }
      if (row == 0 && col == 0) {
        printf("r: %f, g: %f, b: %f\n", pixel.r, pixel.g, pixel.b);
      }
    }
  }
  
  // return processed image output
  return image;

  // END: CS348K STUDENTS MODIFY THIS CODE  
}
