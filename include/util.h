#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <libconfig.h>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include <Eigen/Dense>
#include "daisy/daisy.h"

struct JPP_Config {
  int CALIB_IMG_WIDTH;
  int CALIB_IMG_HEIGHT;
  int RECT_IMG_WIDTH;
  int RECT_IMG_HEIGHT;
  double DR;
  int DQ;
  int DT;
  int DH;
  int BOT_LENGTH;
  int BOT_WIDTH;
  int BOT_HEIGHT;
  int GRID_SIZE;
  double CONF_POS_THRESH;
  double CONF_NEG_THRESH;
  int SAD_WINDOW_SIZE;
  int SPATIAL_FILTER_WINDOW;
  int SPATIAL_FILTER_INC;
  double SPATIAL_FILTER_RATIO;
  int CONF_NEG_INC;
  double CONF_NEG_FILTER_RATIO;
  int START_X;
  int MAX_X;
  int MAX_Y;
  int CONVEX_WORLD;
  
  JPP_Config() {}
  JPP_Config(config_t *cf) {
    config_lookup_int(cf, "calib_img_width", &CALIB_IMG_WIDTH);
    config_lookup_int(cf, "calib_img_height", &CALIB_IMG_HEIGHT);
    config_lookup_int(cf, "rect_img_width", &RECT_IMG_WIDTH);
    config_lookup_int(cf, "rect_img_height", &RECT_IMG_HEIGHT);
    config_lookup_float(cf, "daisy_params.R", &DR);
    config_lookup_int(cf, "daisy_params.Q", &DQ);
    config_lookup_int(cf, "daisy_params.T", &DT);
    config_lookup_int(cf, "daisy_params.H", &DH);
    config_lookup_int(cf, "bot_length", &BOT_LENGTH);
    config_lookup_int(cf, "bot_width", &BOT_WIDTH);
    config_lookup_int(cf, "bot_height", &BOT_HEIGHT);
    config_lookup_int(cf, "grid_size", &GRID_SIZE);
    config_lookup_float(cf, "conf_pos_thresh", &CONF_POS_THRESH);
    config_lookup_float(cf, "conf_neg_thresh", &CONF_NEG_THRESH);
    config_lookup_int(cf, "SAD_window_size", &SAD_WINDOW_SIZE);
    config_lookup_int(cf, "spatial_filter_window", &SPATIAL_FILTER_WINDOW);
    config_lookup_int(cf, "spatial_filter_inc", &SPATIAL_FILTER_INC);
    config_lookup_float(cf, "spatial_filter_ratio", &SPATIAL_FILTER_RATIO);
    config_lookup_int(cf, "conf_neg_inc", &CONF_NEG_INC);
    config_lookup_float(cf, "conf_neg_filter_ratio", &CONF_NEG_FILTER_RATIO);
    config_lookup_int(cf, "start_x", &START_X);
    config_lookup_int(cf, "max_x", &MAX_X);
    config_lookup_int(cf, "max_y", &MAX_Y);
    config_lookup_bool(cf, "convex_world", &CONVEX_WORLD);
    //diagnostic
    /*
    printf("CALIB_IMG_WIDTH: %d\n", CALIB_IMG_WIDTH);
    printf("CALIB_IMG_HEIGHT: %d\n", CALIB_IMG_HEIGHT);
    printf("RECT_IMG_WIDTH: %d\n", RECT_IMG_WIDTH);
    printf("RECT_IMG_HEIGHT: %d\n", RECT_IMG_HEIGHT);
    printf("DR: %lf\n", DR);
    printf("DQ: %d\n", DQ);
    printf("DT: %d\n", DT);
    printf("DH: %d\n", DH);
    printf("BOT_LENGTH: %d\n", BOT_LENGTH);
    printf("BOT_WIDTH: %d\n", BOT_WIDTH);
    printf("BOT_HEIGHT: %d\n", BOT_HEIGHT);
    printf("GRID_SIZE: %d\n", GRID_SIZE);
    printf("CONF_POS_THRESH: %f\n", CONF_POS_THRESH);
    printf("CONF_NEG_THRESH: %f\n", CONF_NEG_THRESH);
    printf("SAD_WINDOW_SIZE: %d\n", SAD_WINDOW_SIZE);
    printf("SPATIAL_FILTER_WINDOW: %d\n", SPATIAL_FILTER_WINDOW);
    printf("SPATIAL_FILTER_INC: %d\n", SPATIAL_FILTER_INC);
    printf("SPATIAL_FILTER_RATIO: %f\n", SPATIAL_FILTER_RATIO);
    printf("CONF_NEG_INC: %d\n", CONF_NEG_INC);
    printf("CONF_NEG_FILTER_RATIO: %f\n", CONF_NEG_FILTER_RATIO);
    printf("START_X: %d\n", START_X);
    printf("MAX_X: %d\n", MAX_X);
    printf("MAX_Y: %d\n", MAX_Y);
    printf("CONVEX_WORLD: %d\n", CONVEX_WORLD);
    */
  }
};

#endif // UTIL_H