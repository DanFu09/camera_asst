#include "camera_pipeline.hpp"
#include <limits>
#include <algorithm>

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

RgbPixel convolution(Image<RgbPixel>* image, float kernel[], int conv_size,
        int row, int col, int width, int height) {
  RgbPixel pixel = RgbPixel(0.f, 0.f, 0.f);
  for (int i = -1 * (conv_size / 2); i <= conv_size / 2; i++) {
    for (int j = -1 * (conv_size / 2); j <= conv_size / 2; j++) {
      if (row + i >= 0 && row + i < height &&
              col + j >= 0 && col + j < width) {
        float multiplier = kernel[(i + conv_size / 2) * conv_size + j +
            conv_size / 2];
        pixel += (*image)(row + i, col + j) * multiplier;
      }
    }
  }
  return pixel;
}

void demosaic(
    Image<RgbPixel>* image,
    CameraSensorData<float>* raw_data,
    int width,
    int height) {

  // Demosaic
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      const auto val = raw_data->data(row, col);
      auto& pixel = (*image)(row, col);

      if ((row % 2) == (col % 2)) {
        // This is a green pixel
        surrounding_average sa_diag = diagonal_neighbors_average(
                raw_data, width, height, row, col, 1, 3, 1);
        surrounding_average sa_nondiag = nondiagonal_neighbors_average(
                raw_data, width, height, row, col, 2, 3, 1);
        pixel.g = (val + sa_diag.val + sa_nondiag.val) /
            (sa_diag.num_neighbors + sa_nondiag.num_neighbors + 1);
        
        // Calculate average of neighboring vertical and neighboring
        // horizontal photos
        surrounding_average sa_vert = vertical_neighbors_average(
                raw_data, width, height, row, col, 1, 4, 2);
        surrounding_average sa_hor = horizontal_neighbors_average(
                raw_data, width, height, row, col, 1, 4, 2);

        if ((row % 2) == 0) {
          // vert is blue, hor is red
          pixel.b = sa_vert.val / sa_vert.num_neighbors;
          pixel.r = sa_hor.val / sa_hor.num_neighbors;
        } else {
          // vert is red, hor is blue
          pixel.r = sa_vert.val / sa_vert.num_neighbors;
          pixel.b = sa_hor.val / sa_hor.num_neighbors;
        }

      } else {
        // This is a blue or red pixel
        
        surrounding_average sa_diags = diagonal_neighbors_average(
            raw_data, width, height, row, col, 1, 2, 1);
        surrounding_average sa_diags_two = diagonal_neighbors_average(
            raw_data, width, height, row, col, 2, 3, 1);
        surrounding_average sa_nondiags = nondiagonal_neighbors_average(
            raw_data, width, height, row, col, 1, 2, 1);
        surrounding_average sa_nondiags_two = nondiagonal_neighbors_average(
            raw_data, width, height, row, col, 2, 3, 1);

        float diags = sa_diags.val / sa_diags.num_neighbors; 
        float nondiags = sa_nondiags.val / sa_nondiags.num_neighbors; 

        if ((row % 2) == 0) {
          // this is a red pixel
          pixel.r = (val + sa_diags_two.val + sa_nondiags_two.val) / (1 +
                  sa_diags_two.num_neighbors + sa_nondiags_two.num_neighbors);

          // diagonal values are blue
          pixel.b = diags;
        } else {
          // this is a blue pixel
          pixel.b = (val + sa_diags_two.val + sa_nondiags_two.val) / (1 +
                  sa_diags_two.num_neighbors + sa_nondiags_two.num_neighbors);

          // diagonal values are red
          pixel.r = diags;
        }

        // non-diagonal values are always green
        pixel.g = nondiags;
      }
    }
  }
}

void RgbToYuv(
    Image<YuvPixel>* output_image,
    Image<RgbPixel>* input_image,
    int width,
    int height) {
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      (*output_image)(row, col) =
          Float3Pixel::RgbToYuv((*input_image)(row, col));
    }
  }
}

void YuvToRgb(
    Image<RgbPixel>* output_image,
    Image<YuvPixel>* input_image,
    int width,
    int height) {
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      (*output_image)(row, col) =
          Float3Pixel::YuvToRgb((*input_image)(row, col));
    }
  }
}

void to_grayscale(
    Image<YuvPixel>* output_grayscale,
    Image<YuvPixel>* output_color,
    Image<YuvPixel>* input,
    int width,
    int height) {
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      auto& output_gray_pixel = (*output_grayscale)(row, col);
      auto& output_color_pixel = (*output_color)(row, col);
      auto& input_pixel = (*input)(row, col);

      output_gray_pixel.y = input_pixel.y;
      output_gray_pixel.u = 0.f;  
      output_gray_pixel.v = 0.f; 

      output_color_pixel.y = 0.5f;
      output_color_pixel.u = input_pixel.u;
      output_color_pixel.v = input_pixel.v;
    }
  }
}

void from_grayscale(
    Image<YuvPixel>* output,
    Image<YuvPixel>* input_color,
    Image<YuvPixel>* input_brightness,
    int width,
    int height) {
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      auto& output_pixel = (*output)(row, col);
      auto& input_color_pixel = (*input_color)(row, col);
      auto& input_brightness_pixel = (*input_brightness)(row, col);

      output_pixel.y = input_brightness_pixel.y;
      output_pixel.u = input_color_pixel.u;  
      output_pixel.v = input_color_pixel.v; 
    }
  }
}

void scale_up_brightness(
    Image<YuvPixel>* output_image,
    Image<YuvPixel>* input_image,
    int width,
    int height,
    float scale_factor) {
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      auto& out_pixel = (*output_image)(row, col);
      auto& in_pixel = (*input_image)(row, col);

      out_pixel.u = in_pixel.u;
      out_pixel.v = in_pixel.v;

      out_pixel.y = fmin(in_pixel.y * scale_factor, 1.f);
    }
  }
}

/**
 * Downsample by 2x.
 **/
void downsample(
    Image<Float3Pixel>* output_image,
    Image<Float3Pixel>* input_image,
    int width,
    int height) {
  float weights[] = {
      (float)1.f/64.f, (float)3.f/64.f, (float)3.f/64.f, (float)1.f/64.f,
      (float)3.f/64.f, (float)9.f/64.f, (float)9.f/64.f, (float)3.f/64.f,
      (float)3.f/64.f, (float)9.f/64.f, (float)9.f/64.f, (float)3.f/64.f,
      (float)1.f/64.f, (float)3.f/64.f, (float)3.f/64.f, (float)1.f/64.f
  };

  for (int j = 0; j < height / 2; j++) {
    for (int i = 0; i < width / 2; i++) {
      Float3Pixel tmp;
      float total_weight = 0.f;
      for (int jj = 0; jj < 4; jj++) {
        for (int ii = 0; ii < 4; ii++) {
          int row = 2 * j + jj;
          int col = 2 * i + ii;
          if (row >= 0 && row < height && col >= 0 && col < width) {
            auto& input_pixel = (*input_image)(row, col);
            tmp += input_pixel * weights[jj * 4 + ii];
            total_weight += weights[jj*4 + ii];
          }
        }
      }
      tmp = tmp * (1.f / total_weight);

      (*output_image)(j, i) = tmp;
    }
  }
}

/**
 * Upsample by 2x.
 **/
void upsample(
    Image<Float3Pixel>* output_image,
    Image<Float3Pixel>* input_image,
    int width,
    int height) {
  for (int j = 0; j < 2 * height; j++) {
    for (int i = 0; i < 2 * width; i++) {
      int row = j / 2;
      int col = i / 2;
      float w1 = (i % 2) ? .75f : .25f;
      float w2 = (j % 2) ? .75f : .25f;

      Float3Pixel& output_pixel = (*output_image)(j, i);
      Float3Pixel& input_pixel1 = (*input_image)(row, col);
      Float3Pixel& input_pixel2 = (*input_image)(row, col+1);
      Float3Pixel& input_pixel3 = (*input_image)(row+1, col);
      Float3Pixel& input_pixel4 = (*input_image)(row+1, col+1);

      output_pixel =
          input_pixel1 * (w1 * w2) +
          input_pixel2 * ((1.f - w1) * w2) +
          input_pixel3 * (w1 * (1.f - w2)) +
          input_pixel4 * ((1.f - w1) * (1.f - w2));
    }
  }
}

void image_plus(
    Image<Float3Pixel>* output_image,
    Image<Float3Pixel>* input_imagea,
    Image<Float3Pixel>* input_imageb,
    int width,
    int height) {
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      (*output_image)(row, col) = 
          (*input_imagea)(row, col) + (*input_imageb)(row, col);
    }
  }
}

void image_minus(
    Image<Float3Pixel>* output_image,
    Image<Float3Pixel>* input_imagea,
    Image<Float3Pixel>* input_imageb,
    int width,
    int height) {
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      (*output_image)(row, col) = 
          (*input_imagea)(row, col) - (*input_imageb)(row, col);
    }
  }
}

void copy(
    Image<Float3Pixel>* output_image,
    Image<Float3Pixel>* input_image,
    int width,
    int height) {
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      (*output_image)(row, col) = (*input_image)(row, col);
    }
  }
}

void laplacian(
    Image<Float3Pixel>* output,
    Image<Float3Pixel>* input,
    int width,
    int height) {
  std::unique_ptr<Image<YuvPixel>> g1(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> upg1(new Image<YuvPixel>(width, height));

  downsample(g1.get(), input, width, height);
  upsample(upg1.get(), g1.get(), width / 2, height / 2);

  image_minus(output, input, upg1.get(), width, height);
}

Float3Pixel cross_product(Float3Pixel pix1, Float3Pixel pix2) {
  return Float3Pixel(pix1.r * pix2.r, pix1.g * pix2.g, pix1.b * pix2.b);
}

void calculate_weights(
    Image<YuvPixel>* weight_map,
    Image<YuvPixel>* color_image,
    int width,
    int height) {
  // initialize weight map
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      (*weight_map)(row, col) = YuvPixel(1.f, 0.f, 0.f);
    }
  }

  std::unique_ptr<Image<YuvPixel>> grayscale(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> color_copy(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> colors(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> l0(new Image<YuvPixel>(width, height));

  to_grayscale(grayscale.get(), colors.get(), color_image, width, height);

  // Calculate laplacian
  laplacian(l0.get(), grayscale.get(), width, height);

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      (*weight_map)(row, col) = (*weight_map)(row, col) *
          pow((*l0)(row, col).y, 1);
    }
  }

  copy(color_copy.get(), color_image, width, height);
  // Calculate saturation
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      RgbPixel pix = Float3Pixel::YuvToRgb((*color_copy)(row, col));
      float mean = (pix.r + pix.b + pix.g) / 3.f;
      float stddev = sqrt((pow(pix.r - mean, 2) + pow(pix.b - mean, 2) +
              pow(pix.g - mean, 2)) / 3.f);
      (*weight_map)(row, col) = (*weight_map)(row, col) * pow(stddev, 5);
    }
  }

  // Calculate well-exposedness
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      auto pixel = Float3Pixel::YuvToRgb((*color_copy)(row, col));
      (*weight_map)(row, col) = (*weight_map)(row, col) *
          exp(-1 * pow(pixel.r - 0.5, 2) / (2 * 0.2 * 0.2)) *
          exp(-1 * pow(pixel.b - 0.5, 2) / (2 * 0.2 * 0.2)) *
          exp(-1 * pow(pixel.g - 0.5, 2) / (2 * 0.2 * 0.2));    
    }
  }
}

void calculate_laplacian_pyramid(
    std::vector<std::unique_ptr<Image<Float3Pixel>>>* pyramid,
    Image<Float3Pixel>* image,
    int width,
    int height,
    int layers) {
  // allocate space for gnext
  std::unique_ptr<Image<Float3Pixel>> gcurrent(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<Float3Pixel>> gnext(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<Float3Pixel>> upgnext(new Image<YuvPixel>(width, height));

  copy(gnext.get(), image, width, height);

  int cur_width = width;
  int cur_height = height;
  // for int i = 0; i < layers... calculate l_i
  for (int i = 0; i < layers - 1; i++) {
    copy(gcurrent.get(), gnext.get(), cur_width, cur_height);
    downsample(gnext.get(), gcurrent.get(), cur_width, cur_height);
    upsample(upgnext.get(), gnext.get(), cur_width / 2, cur_height / 2);

    image_minus(pyramid->at(i).get(), gcurrent.get(), upgnext.get(), cur_width, cur_height);
    
    cur_width = cur_width / 2;
    cur_height = cur_height / 2;
  }

  // fill in the last layer
  copy(pyramid->at(layers - 1).get(), gnext.get(), cur_width, cur_height);
}

void calculate_gaussian_pyramid(
    std::vector<std::unique_ptr<Image<Float3Pixel>>>* pyramid,
    Image<Float3Pixel>* image,
    int width,
    int height,
    int layers) {
  std::unique_ptr<Image<Float3Pixel>> gcurrent(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<Float3Pixel>> gnext(new Image<YuvPixel>(width, height));

  int cur_width = width;
  int cur_height = height;
  copy(gcurrent.get(), image, cur_width, cur_height);
  for (int i = 0; i < layers; i++) {
    copy(pyramid->at(i).get(), gcurrent.get(), cur_width, cur_height);

    downsample(gnext.get(), gcurrent.get(), cur_width, cur_height);
    copy(gcurrent.get(), gnext.get(), cur_width, cur_height);

    cur_width = cur_width / 2;
    cur_height = cur_height / 2;
  }
}

void flatten_laplacian_pyramid(
    Image<Float3Pixel>* out,
    std::vector<std::unique_ptr<Image<Float3Pixel>>>* pyramid,
    int width,
    int height,
    int layers) {
  std::unique_ptr<Image<Float3Pixel>> gcurrent(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<Float3Pixel>> gcurrentup(new Image<YuvPixel>(width, height));

  copy(gcurrent.get(), pyramid->at(layers - 1).get(), width, height);
  for (int i = layers - 2; i >= 0; i--) {
    upsample(gcurrentup.get(), gcurrent.get(), width / 2, height / 2);
    image_plus(gcurrent.get(), gcurrentup.get(), pyramid->at(i).get(), width, height);
  } 

  copy(out, gcurrent.get(), width, height);
}


Float3Pixel invert(Float3Pixel pix) {
  Float3Pixel out;
  if (pix.r > 0)
    out.r = 1.f / pix.r;
  if (pix.b > 0)
    out.b = 1.f / pix.b;
  if (pix.g > 0)
    out.g = 1.f / pix.g;

  return out;
}

void blend(
    Image<YuvPixel>* blended_output,
    Image<YuvPixel>* image1,
    Image<YuvPixel>* image1weight,
    Image<YuvPixel>* image2,
    Image<YuvPixel>* image2weight,
    int width,
    int height,
    int layers,
    float gamma_correction) {
  image1->GammaCorrect(gamma_correction);
  image2->GammaCorrect(gamma_correction);

  // allocate space for the Laplacian pyramids for each image
  std::vector<std::unique_ptr<Image<YuvPixel>>> image1laplacians;
  std::vector<std::unique_ptr<Image<YuvPixel>>> image2laplacians;
  for (int i = 0; i < layers; i++) {
      std::unique_ptr<Image<YuvPixel>> layer1(
              new Image<YuvPixel>(width, height));
      image1laplacians.push_back(std::move(layer1));

      std::unique_ptr<Image<YuvPixel>> layer2(
              new Image<YuvPixel>(width, height));
      image2laplacians.push_back(std::move(layer2));
  }

  // allocate space for the gaussian pyramids for each image
  std::vector<std::unique_ptr<Image<YuvPixel>>> weight1gaussians;
  std::vector<std::unique_ptr<Image<YuvPixel>>> weight2gaussians;
  for (int i = 0; i < layers; i++) {
      std::unique_ptr<Image<YuvPixel>> layer1(
              new Image<YuvPixel>(width, height));
      weight1gaussians.push_back(std::move(layer1));

      std::unique_ptr<Image<YuvPixel>> layer2(
              new Image<YuvPixel>(width, height));
      weight2gaussians.push_back(std::move(layer2));
  }

  // calculate laplacian pyramids
  calculate_laplacian_pyramid(&image1laplacians, image1, width, height, layers);
  calculate_laplacian_pyramid(&image2laplacians, image2, width, height, layers);
  // calculate gaussian pyramids
  calculate_gaussian_pyramid(&weight1gaussians, image1, width, height, layers);
  calculate_gaussian_pyramid(&weight2gaussians, image2, width, height, layers);

  printf("Calculated all pyramids\n");

  // blend the pyramids
  // allocate space for the fused pyramid
  std::vector<std::unique_ptr<Image<YuvPixel>>> fusedLaplacians;
  for (int i = 0; i < layers; i++) {
      std::unique_ptr<Image<YuvPixel>> layer(
              new Image<YuvPixel>(width, height));
      fusedLaplacians.push_back(std::move(layer));
  }

  // blend based on weights
  for (int i = 0; i < layers; i++) {
    for (int row = 0; row < height; row++) {
      for (int col = 0; col < width; col++) {
        auto& pixel_out = (*(fusedLaplacians.at(i)))(row, col);
        auto& image1_pix = (*(image1laplacians.at(i)))(row, col);
        auto& image2_pix = (*(image2laplacians.at(i)))(row, col);
        auto& weight1_pix = (*(weight1gaussians.at(i)))(row, col);
        auto& weight2_pix = (*(weight2gaussians.at(i)))(row, col);

        float weight1 = weight1_pix.y / (weight1_pix.y + weight2_pix.y);
        float weight2 = weight2_pix.y / (weight1_pix.y + weight2_pix.y);

        pixel_out = image1_pix * weight1 + image2_pix * weight2;
      }
    }
  }

  printf("Finished blending\n");

  flatten_laplacian_pyramid(blended_output, &fusedLaplacians, width, height, layers);

  printf("Flattened pyramid\n");

  blended_output->GammaCorrect(1.f / gamma_correction);

  image1->GammaCorrect(1.f / gamma_correction);
  image2->GammaCorrect(1.f / gamma_correction);
}

void
local_tone_mapping(
    Image<Float3Pixel>* image,
    float dark_gain,
    float bright_gain,
    int layers,
    float gamma_correction,
    int width,
    int height) {
  // allocate space for grayscale images
  std::unique_ptr<Image<YuvPixel>> yuv(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> yuv_dark(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> yuv_bright(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> dark_colors(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> dark_gray(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> bright_colors(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> bright_gray(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> blended_gray(new Image<YuvPixel>(width, height));

  RgbToYuv(yuv.get(), image, width, height);
  scale_up_brightness(yuv_dark.get(), yuv.get(), width, height, dark_gain); 
  scale_up_brightness(yuv_bright.get(), yuv.get(), width, height, bright_gain); 

  // allocate space for the weight maps
  std::unique_ptr<Image<YuvPixel>> dark_weights(new Image<YuvPixel>(width, height));
  std::unique_ptr<Image<YuvPixel>> bright_weights(new Image<YuvPixel>(width, height));

  calculate_weights(dark_weights.get(), yuv_dark.get(), width, height);
  calculate_weights(bright_weights.get(), yuv_bright.get(), width, height);

  printf("Weights calculated\n");
  to_grayscale(dark_gray.get(), dark_colors.get(), yuv_dark.get(), width, height);
  to_grayscale(bright_gray.get(), bright_colors.get(), yuv_bright.get(), width, height);

  blend(blended_gray.get(), dark_gray.get(), dark_weights.get(),
          bright_gray.get(), bright_weights.get(), width, height, layers,
          gamma_correction);

  printf("Blending done\n");

  from_grayscale(yuv.get(), dark_colors.get(), blended_gray.get(), width, height); 

  YuvToRgb(image, yuv.get(), width, height);
}

void
correct_bad_pixels(
    std::vector<std::unique_ptr<CameraSensorData<float>>>* raw_data_burst,
    CameraSensorData<float>* dark_frame,
    int width,
    int height) {
  // correct for bad pixels
  for (int i = 0; i < raw_data_burst->size(); i++) {
    auto& raw_data = raw_data_burst->at(i);

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
  }
}

void
calculate_alignments(
    std::vector<std::unique_ptr<Image<YuvPixel>>>* alignments,
    std::vector<std::unique_ptr<CameraSensorData<float>>>* raw_data_burst,
    int layers,
    int tile_size,
    int width,
    int height) {
  // convert raw Bayer images to grayscale
  std::vector<std::unique_ptr<Image<YuvPixel>>> burst_images; 
  for (int i = 0; i < raw_data_burst->size(); i++) {
    std::unique_ptr<Image<RgbPixel>> image(new Image<YuvPixel>(width / 2, height / 2));

    auto& raw_data = raw_data_burst->at(i);

    for (int row = 0; row < height / 2; row++) {
      for (int col = 0; col < width / 2; col++) {
        auto& pixel = (*image)(row, col);
        int num_valid = 0;
        for (int i = 0; i < 2; i++) {
          for (int j = 0; j < 2; j++) {
            if (2 * row + i < height && 2 * col + j < width) {
              pixel.y += raw_data->data(2*row + i, 2*col + j);
              num_valid++;
            }
          }
        }
        pixel.y *= 1.f / num_valid;
      }
    }

    burst_images.push_back(std::move(image));
  }

  printf("Converted burst to grayscale\n");

  std::vector<std::unique_ptr<Image<YuvPixel>>> burst_gaussians;
  for (int i = 0; i < raw_data_burst->size(); i++) {
    // allocate space for gaussians
    std::vector<std::unique_ptr<Image<YuvPixel>>> gaussians;
    int cur_width = width / 2;
    int cur_height = height / 2;
    for (int i = 0; i < layers; i++) {
        std::unique_ptr<Image<YuvPixel>> layer(
                new Image<YuvPixel>(cur_width, cur_height));
        gaussians.push_back(std::move(layer));
        cur_width = cur_width / 2;
        cur_height = cur_height / 2;
    }

    calculate_gaussian_pyramid(&gaussians, burst_images.at(i).get(),
            width / 2, height / 2, layers);

    for (int i = 0; i < layers; i++) {
      burst_gaussians.push_back(std::move(gaussians.at(i)));
    }
  }

  printf("Calculated pyramids for alignment\n");

  std::unique_ptr<Image<YuvPixel>> cur_tile(new Image<YuvPixel>(tile_size, tile_size));
  std::unique_ptr<Image<YuvPixel>> ref_tile(new Image<YuvPixel>(tile_size, tile_size));

  for (int pyramid_level = layers - 1; pyramid_level >= 0; pyramid_level--) {
    auto& ref_pyramid = burst_gaussians.at(pyramid_level);
    int cur_width = width / pow(2, pyramid_level + 1);
    int cur_height = height / pow(2, pyramid_level + 1);
    // calculate the best offset for this reference tile
    for (int row = 0; row < cur_height; row += tile_size) {
      for (int col = 0; col < cur_width; col += tile_size) {
        // populate our current tile
        for (int i = 0; i < tile_size; i++) {
          for (int j = 0; j < tile_size; j++) {
            if (row + i < cur_height && col + j < cur_width) {
              (*ref_tile)(i, j) = (*ref_pyramid)(row + i, col + j);
            } else {
              (*ref_tile)(i, j).y = 0.f;
              (*ref_tile)(i, j).u = 0.f;
              (*ref_tile)(i, j).v = 0.f;
            }
          }
        } 

        for (int image = 1; image < raw_data_burst->size(); image++) {
          auto& cur_pyramid = burst_gaussians.at(image * layers + pyramid_level);

          int cur_alignment_index = (image - 1) * (layers + 1) + pyramid_level + 1;
          auto& alignment = alignments->at(cur_alignment_index);

          int child_row = (row / (2 * tile_size)) * tile_size;
          int child_col = (col / (2 * tile_size)) * tile_size;

          int best_x = 2 * (*alignment)(child_row, child_col).u;
          int best_y = 2 * (*alignment)(child_row, child_col).v;
          float best_diff = std::numeric_limits<float>::max();

          int start_x = best_x;
          int start_y = best_y;

          // search for the best offset, tile_size in either direction
          for (int test_x = tile_size * -1; test_x < tile_size * 1; test_x++) {
            for (int test_y = tile_size * -1; test_y < tile_size * 1; test_y++) {
              // calculate the diff
              int offset_x = col + start_x + test_x;
              int offset_y = row + start_y + test_y;

              int num_overlap = 0;
              float diff = 0.f;
              for (int i = 0; i < tile_size; i++) {
                for (int j = 0; j < tile_size; j++) {
                  if (row + i + offset_y < cur_height && 
                          row + i + offset_y >= 0 &&
                          col + j + offset_x < cur_width &&
                          col + j + offset_x >= 0) {
                    auto& cur_pix = (*cur_pyramid)(row + i + offset_y,
                            col + j + offset_x);
                    diff += abs(cur_pix.y - (*ref_tile)(i, j).y);
                    num_overlap++;
                  }
                }
              } 
              if (num_overlap == 0) {
                  continue;
              }
              if (diff < best_diff) {
                best_diff = diff;
                best_x = offset_x;
                best_y = offset_y;
              } 
            }
          }

          // save the best alignment one alignment up
          auto& up_alignment = alignments->at(cur_alignment_index - 1);
          (*up_alignment)(row, col).u = best_x;
          (*up_alignment)(row, col).v = best_y;
        }
      }
    }
  }

  printf("Calculated alignments\n");
  
}

void
merge_burst(
    CameraSensorData<float>* new_raw_data,
    std::vector<std::unique_ptr<CameraSensorData<float>>>* raw_data_burst,
    std::vector<std::unique_ptr<Image<YuvPixel>>>* alignments,
    int layers,
    int tile_size,
    int tolerance,
    int width,
    int height) {
  auto& ref_image = raw_data_burst->at(0);
  std::unique_ptr<Image<YuvPixel>> ref_tile(new Image<YuvPixel>(tile_size, tile_size));
  std::unique_ptr<Image<YuvPixel>> new_tile(new Image<YuvPixel>(tile_size, tile_size));

  // merge images
  for (int row = 0; row < height; row += tile_size / 2) {
    for (int col = 0; col < width; col += tile_size / 2) {
      // populate the reference tile, and initialize the new tile
      for (int i = 0; i < tile_size; i++) {
        for (int j = 0; j < tile_size; j++) {
          if (row + i < height && col + j < width) {
            (*ref_tile)(i, j).y = ref_image->data(row + i, col + j);
          } else {
            (*ref_tile)(i, j).y = 0.f;
          }
          (*new_tile)(i, j).y = (*ref_tile)(i, j).y / raw_data_burst->size();
        }
      } 
      int num_tiles_merged = 0;
      for (int image = 1; image < raw_data_burst->size(); image++) {
        int alignment_index = (image - 1) * (layers + 1);
        auto& alignment = alignments->at(alignment_index);
        auto& merge_image = raw_data_burst->at(image);

        int child_row = (row / (2 * tile_size)) * tile_size;
        int child_col = (col / (2 * tile_size)) * tile_size;
        
        int offset_x = 2 * (*alignment)(child_row, child_col).u;
        int offset_y = 2 * (*alignment)(child_row, child_col).v;

        // calculate the diff
        float diff = 0.f;
        for (int i = 0; i < tile_size; i++) {
          for (int j = 0; j < tile_size; j++) {
            if (row + i + offset_y < height && 
                    row + i + offset_y >= 0 &&
                    col + j + offset_x < width &&
                    col + j + offset_x >= 0) {
              auto& ref_data = (*ref_tile)(i, j).y;
              auto& merge_data = merge_image->data(row + i + offset_y,
                      col + j + offset_x);
              diff += abs(ref_data - merge_data);
            }
          }
        } 
        if (diff < tolerance) {
            num_tiles_merged++;
        }
        for (int i = 0; i < tile_size; i++) {
          for (int j = 0; j < tile_size; j++) {
            if (row + i + offset_y < height && 
                    row + i + offset_y >= 0 &&
                    col + j + offset_x < width &&
                    col + j + offset_x >= 0 &&
                    diff < tolerance) {
              // add in the data to the new tile
              auto& merge_data = merge_image->data(row + i + offset_y,
                      col + j + offset_x);
              (*new_tile)(i, j).y += merge_data / raw_data_burst->size();
            } else {
              (*new_tile)(i, j).y += (*ref_tile)(i, j).y / raw_data_burst->size();
            }
          }
        } 
      }
      // add the new tile into the new image
      for (int i = 0; i < tile_size; i++) {
        for (int j = 0; j < tile_size; j++) {
          if (row + i < height && col + j < width) {
            float dft_i = 0.5f - 0.5f * cos(2.f * M_PI * (i + 0.5f) / tile_size);
            float dft_j = 0.5f - 0.5f * cos(2.f * M_PI * (j + 0.5f) / tile_size);
            new_raw_data->data(row + i, col + j) +=
                (*new_tile)(i, j).y * dft_i * dft_j;
          }
        }
      } 
    }
  }

  printf("Merged images\n");
}

void
calculate_alignments_and_merge(
    CameraSensorData<float>* new_raw_data,
    std::vector<std::unique_ptr<CameraSensorData<float>>>* raw_data_burst,
    int layers,
    int tile_size,
    int tolerance,
    int width,
    int height) {
  // Allocate space for alignments
  // Use Yuv pixels, u is x difference, y is y difference
  std::vector<std::unique_ptr<Image<YuvPixel>>> alignments;
  for (int i = 0; i < (layers + 1) * (raw_data_burst->size() -1); i++) {
    // allocate space
    int cur_width = width / pow(2, (i % (layers + 1)));
    int cur_height = height / pow(2, (i % (layers + 1)));
    std::unique_ptr<Image<YuvPixel>> layer(
            new Image<YuvPixel>(cur_width, cur_height));

    alignments.push_back(std::move(layer));
  }

  calculate_alignments(&alignments, raw_data_burst, layers, tile_size, width, height);

  merge_burst(new_raw_data, raw_data_burst, &alignments, layers,
          tile_size, tolerance, width, height);
}

void
denoise_final_image(
    Image<Float3Pixel>* image,
    int width,
    int height) {
  std::unique_ptr<Image<RgbPixel>> image_copy(new Image<RgbPixel>(width, height));
  copy(image_copy.get(), image, width, height);

  std::vector<float> channel1;
  std::vector<float> channel2;
  std::vector<float> channel3;

  // denoise with a median filter
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      int median_size = 5;
      channel1.clear();
      channel2.clear();
      channel3.clear();
      for (int i = 0; i < median_size; i++) {
        for (int j = 0; j < median_size; j++) {
          int offset_x = i - median_size / 2;
          int offset_y = j - median_size / 2;
          if (row + offset_y >= 0 && row + offset_y < height &&
                  col + offset_x >= 0 && col + offset_x < width) {
            channel1.push_back((*image_copy)(row + offset_y, col + offset_x).r);
            channel2.push_back((*image_copy)(row + offset_y, col + offset_x).b);
            channel3.push_back((*image_copy)(row + offset_y, col + offset_x).g);
          }
        }
      }
      sort(channel1.begin(), channel1.end());
      sort(channel2.begin(), channel2.end());
      sort(channel3.begin(), channel3.end());
      (*image)(row, col).r = channel1.at(channel1.size() / 2);
      (*image)(row, col).b = channel2.at(channel2.size() / 2);
      (*image)(row, col).g = channel3.at(channel3.size() / 2);
    }
  } 
  printf("Finished median filter\n");

  copy(image_copy.get(), image, width, height);
  // denoise with a 3x3 blur convolution
  float sharpen_kernel[9] =
    {0, -1.f, 0, -1.f, 5.f, -1.f, 0, -1.f, 0};
  float kernel_3[9] =
    {1./9, 1./9, 1./9, 1./9, 1./9, 1./9, 1./9, 1./9, 1./9};
  float kernel_5[25] =
    {1./25, 1./25, 1./25, 1./25, 1./25,
        1./25, 1./25, 1./25, 1./25, 1./25,
        1./25, 1./25, 1./25, 1./25, 1./25,
        1./25, 1./25, 1./25, 1./25, 1./25,
        1./25, 1./25, 1./25, 1./25, 1./25};
  float gaussian_kernel_5[25] =
  {0.003765, 0.015019, 0.023792, 0.015019, 0.003765,
    0.015019, 0.059912, 0.094907, 0.059912, 0.015019,
    0.023792, 0.094907, 0.150342, 0.094907, 0.023792,
    0.015019, 0.059912, 0.094907, 0.059912, 0.015019,
    0.003765, 0.015019, 0.023792, 0.015019, 0.003765};
  float kernel_7[49] = 
  {1./49, 1./49, 1./49, 1./49, 1./49, 1./49, 1./49,
      1./49, 1./49, 1./49, 1./49, 1./49, 1./49, 1./49,
      1./49, 1./49, 1./49, 1./49, 1./49, 1./49, 1./49,
      1./49, 1./49, 1./49, 1./49, 1./49, 1./49, 1./49,
      1./49, 1./49, 1./49, 1./49, 1./49, 1./49, 1./49,
      1./49, 1./49, 1./49, 1./49, 1./49, 1./49, 1./49,
      1./49, 1./49, 1./49, 1./49, 1./49, 1./49, 1./49};
  //for (int row = 0; row < height; row++) {
  //  for (int col = 0; col < width; col++) {
  //    (*image)(row, col) = convolution(image_copy.get(), sharpen_kernel, 3,
  //            row, col, width, height);
  //  }
  //}

  //printf("Finished sharpening\n");
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
  auto raw_data_burst = sensor_->GetBurstSensorData(0, 0, width, height);

  correct_bad_pixels(&raw_data_burst, dark_frame.get(), width, height);

  int layers = 4;
  int tile_size = 16;
  int tolerance = 5;

  std::unique_ptr<CameraSensorData<float>>
      raw_data(new CameraSensorData<float>(width, height));

  bool use_burst = true;
  if (use_burst) {
    calculate_alignments_and_merge(raw_data.get(), &raw_data_burst, layers,
          tile_size, tolerance, width, height);
  } else {
    for (int row = 0; row < height; row++) {
      for (int col = 0; col < width; col++) {
        raw_data->data(row, col) = raw_data_burst.at(0)->data(row, col);
      }
    }
  }

  // allocate 3-channel RGB output buffer to hold the results after processing 
  std::unique_ptr<Image<RgbPixel>> image(new Image<RgbPixel>(width, height));
 
  demosaic(image.get(), raw_data.get(), width, height); 

  float dark_gain = 0.6;
  float bright_gain = 2.0f;
  int blend_layers = 4;
  float gamma_correction = .4f;

  local_tone_mapping(image.get(), dark_gain, bright_gain, blend_layers, 
          gamma_correction, width, height);

  denoise_final_image(image.get(), width, height);

  image->GammaCorrect(0.4);

  // scale up to 255
  float scalar = 1.0;
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      auto& pixel = (*image)(row, col);
      pixel.r *= scalar * 255.f; 
      pixel.b *= scalar * 255.f; 
      pixel.g *= scalar * 255.f; 
    }
  }
  
  // return processed image output
  return image;

  // END: CS348K STUDENTS MODIFY THIS CODE  
}
