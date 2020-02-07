#ifndef GUI_H
#define GUI_H

#define IMGUI_IMPL_OPENGL_LOADER_GL3W

//IMGUI_IMPL_OPENGL_LOADER_GL3W

#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>    // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>    // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>  // Initialize with gladLoadGL()
//#else
//#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

#include "imgui.h"
#include "imgui_impl_sdl.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"
#include <stdio.h>
#include <SDL.h>
#include <iostream>


enum STEREO_VISION {

    BLF_DISABLED = 0,
    BLF_ORIGINAL,
    BLF_CUDA,
    WLS_DISABLED,
    WLS_ENABLED,
    WLS_LRCHECK,
    SGBM_OPENCV,
    SGBM_CUDA,
    LIBELAS
};


class GUI
{

private:

    SDL_GLContext gl_context;
    SDL_Window* window;
    ImGuiIO io;
    ImVec4 clear_color;
    bool done;
    int val;
    bool updated;
    bool recalibrate;

    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int numberOfDisparities;
    int SADWindowSize;
    int minDisparity;
    int preFilterCap;
    int disp12MaxDiff;
    float sigmaColorBLF;
    float sigmaSpaceBLF;
    float wls_lambda;
    float wls_sigma;
//    bool useWLS;
//    bool useBLF;
//    bool left_right;

    STEREO_VISION BLFfiltering;
    STEREO_VISION WLSfiltering;
    STEREO_VISION stereo_matching;

    int BLFfiltering_id;
    int WLSfiltering_id;
    int stereo_matching_id;


public:


    void killGUI();
    GUI();
    ~GUI();
    int initializeGUI();
    int initializeGUI(int minDisparity, int numberOfDisparities, int SADWindowSize,
                                             int disp12MaxDiff, int preFilterCap, int uniquenessRatio,
                                             int speckleWindowSize, int speckleRange, double sigmaColorBLF,
                                             double sigmaSpaceBLF, double wls_lambda, double wls_sigma,
                                             STEREO_VISION BLFfiltering, STEREO_VISION WLSfiltering,
                                             STEREO_VISION stereo_matching);
    void updateGUI();
    void setVal(int);
    int getVal();
    void getParams(int& minDisparity, int& numberOfDisparities, int& SADWindowSize,
                   int& disp12MaxDiff, int& preFilterCap, int& uniquenessRatio,
                   int& speckleWindowSize, int& speckleRange, double& sigmaColorBLF,
                   double& sigmaSpaceBLF, double& wls_lambda, double& wls_sigma,
                   STEREO_VISION& BLFfiltering, STEREO_VISION& WLSfiltering,
                   STEREO_VISION& stereo_matching);
    bool isDone();
    bool isUpdated();
    void setUpdated(bool);
    void setUpdated(bool, bool);

    bool toRecalibrate();

    void convertIDToEnum();
    void convertEnumToID();
};

#endif // GUI_H
