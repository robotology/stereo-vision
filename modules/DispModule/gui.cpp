#include "gui.h"


GUI::GUI()
{
    this->updated = false;
}

bool GUI::isUpdated()
{
    return this->updated;
}

int GUI::initializeGUI(int minDisparity, int numberOfDisparities, int SADWindowSize,
                       int disp12MaxDiff, int preFilterCap, int uniquenessRatio,
                       int speckleWindowSize, int speckleRange, double sigmaColorBLF,
                       double sigmaSpaceBLF, double wls_lambda, double wls_sigma,
                       STEREO_VISION BLFfiltering, STEREO_VISION WLSfiltering,
                       STEREO_VISION stereo_matching)
{

    this->minDisparity = minDisparity;
    this->numberOfDisparities = numberOfDisparities;
    this->SADWindowSize = SADWindowSize;
    this->disp12MaxDiff = disp12MaxDiff;
    this->preFilterCap = preFilterCap;
    this->uniquenessRatio = uniquenessRatio;
    this->speckleWindowSize = speckleWindowSize;
    this->speckleRange = speckleRange;
    this->sigmaColorBLF = sigmaColorBLF;
    this->sigmaSpaceBLF = sigmaSpaceBLF;
    this->wls_lambda = wls_lambda;
    this->wls_sigma = wls_sigma;
    this->BLFfiltering = BLFfiltering;
    this->WLSfiltering = WLSfiltering;
    this->stereo_matching = stereo_matching;

    this->convertEnumToID();

    return this->initializeGUI();
}

int GUI::initializeGUI()
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
        {
            printf("Error: %s\n", SDL_GetError());
            return -1;
        }

        // Decide GL+GLSL versions
    #if __APPLE__
        // GL 3.2 Core + GLSL 150
        const char* glsl_version = "#version 150";
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); // Always required on Mac
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    #else
        // GL 3.0 + GLSL 130
        const char* glsl_version = "#version 130";
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    #endif

        // Create window with graphics context
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
        SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
        this->window = SDL_CreateWindow("SGBM parameters", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 585, 415, window_flags);
        this->gl_context = SDL_GL_CreateContext(window);
        SDL_GL_SetSwapInterval(1); // Enable vsync

        // Initialize OpenGL loader
    #if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
        bool err = gl3wInit() != 0;
    #elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
        bool err = glewInit() != GLEW_OK;
    #elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
        bool err = gladLoadGL() == 0;
    #else
        bool err = false; // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader is likely to requires some form of initialization.
    #endif
        if (err)
        {
            fprintf(stderr, "Failed to initialize OpenGL loader!\n");
            return 1;
        }

        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        this->io = ImGui::GetIO();
        this->io;

        // Setup Dear ImGui style
        ImGui::StyleColorsDark();

        // Setup Platform/Renderer bindings
        ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
        ImGui_ImplOpenGL3_Init(glsl_version);

        this->clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

        // Main loop
        this->done = false;
}

void GUI::killGUI()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(this->gl_context);
    SDL_DestroyWindow(this->window);
    SDL_Quit();
}

void GUI::updateGUI()
{

    // GUI UPDATE

    SDL_Event event;

    this->updated = false;

    while (SDL_PollEvent(&event))
    {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT)
            this->done = true;
        if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window))
            this->done = true;
    }

    // Start the Dear preFilterCapImGui frame

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame(this->window);
    ImGui::NewFrame();

    // Stereo matching parameters

    this->updated |= ImGui::SliderInt("MinDisparity", &this->minDisparity, 0, 20);

    ImGui::Text("numberOfDisparities:  "); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("32", &this->numberOfDisparities, 32); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("64", &this->numberOfDisparities, 64); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("96", &this->numberOfDisparities, 96); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("128", &this->numberOfDisparities, 128);

    this->updated |= ImGui::SliderInt("SADWindowSize", &this->SADWindowSize, 3, 31);
    this->updated |= ImGui::SliderInt("disp12MaxDiff", &this->disp12MaxDiff, 0, 30);
    this->updated |= ImGui::SliderInt("preFilterCap", &this->preFilterCap, 0, 100);
    this->updated |= ImGui::SliderInt("uniquenessRatio", &this->uniquenessRatio, 5, 20);

    // Speckle filtering parameters

    this->updated |= ImGui::SliderInt("speckleWindowSize", &this->speckleWindowSize, 0, 200);
    this->updated |= ImGui::SliderInt("speckleRange", &this->speckleRange, 1, 16);

    // BLF filter parameters

    this->updated |= ImGui::SliderFloat("sigmaColorBLF", &this->sigmaColorBLF, 1.0f, 30.0f, "%.2f");
    this->updated |= ImGui::SliderFloat("sigmaSpaceBLF", &this->sigmaSpaceBLF, 1.0f, 30.0f, "%.2f");

    // WLS filter parameters

    this->updated |= ImGui::InputFloat("WLS lambda", &this->wls_lambda, 500, 1000, "%.1f");
    this->updated |= ImGui::InputFloat("WLS sigma", &this->wls_sigma, 0.1f, 0.5f, "%.2f");

    // Alternatives for the stereo matching algorithm to be used

    ImGui::Text("Stereo Matching Algorithm: "); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("SGBM", &this->stereo_matching_id, 0); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("CUDA SGBM", &this->stereo_matching_id, 1); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("LibElas", &this->stereo_matching_id, 2);

    // Alternatives for the BLF filtering

    ImGui::Text("Bilateral Filtering:       "); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("No BLF", &this->BLFfiltering_id, 0); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("Original BLF", &this->BLFfiltering_id, 1); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("CUDA BLF", &this->BLFfiltering_id, 2);

    // Alternatives for the WLS filtering

    ImGui::Text("Weighted LS Filtering:     "); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("No WLS", &this->WLSfiltering_id, 0); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("WLS", &this->WLSfiltering_id, 1); ImGui::SameLine();
    this->updated |= ImGui::RadioButton("WLS with l/r cons.", &this->WLSfiltering_id, 2);

    // Button to trigger the recalibration of the camera

    this->recalibrate = ImGui::Button("Recalibrate"); ImGui::SameLine();
    this->updated |= this->recalibrate;

    // ------------------------------

    // Rendering

    ImGui::Render();
    SDL_GL_MakeCurrent(window, gl_context);
    glViewport(0, 0, (int)this->io.DisplaySize.x, (int)this->io.DisplaySize.y);
    glClearColor(this->clear_color.x, this->clear_color.y, this->clear_color.z, this->clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(this->window);

}

void GUI::getParams(int& minDisparity, int& numberOfDisparities, int& SADWindowSize,
                    int& disp12MaxDiff, int& preFilterCap, int& uniquenessRatio,
                    int& speckleWindowSize, int& speckleRange, double& sigmaColorBLF,
                    double& sigmaSpaceBLF, double& wls_lambda, double& wls_sigma,
                    STEREO_VISION& BLFfiltering, STEREO_VISION& WLSfiltering,
                    STEREO_VISION& stereo_matching)
{
    this->convertIDToEnum();

    minDisparity = this->minDisparity;
    numberOfDisparities = this->numberOfDisparities;
    SADWindowSize = this->SADWindowSize;
    disp12MaxDiff = this->disp12MaxDiff;
    preFilterCap = this->preFilterCap;
    uniquenessRatio = this->uniquenessRatio;
    speckleWindowSize = this->speckleWindowSize;
    speckleRange = this->speckleRange;
    sigmaColorBLF = this->sigmaColorBLF;
    sigmaSpaceBLF = this->sigmaSpaceBLF;
    wls_lambda = this->wls_lambda;
    wls_sigma = this->wls_sigma;
    BLFfiltering = this->BLFfiltering;
    WLSfiltering = this->WLSfiltering;
    stereo_matching = this->stereo_matching;
}

void GUI::convertIDToEnum()
{

    switch(this->stereo_matching_id)
    {
        case 0:
            this->stereo_matching = STEREO_VISION::SGBM_OPENCV;
            break;
        case 1:
            this->stereo_matching = STEREO_VISION::SGBM_CUDA;
            break;
        case 2:
            this->stereo_matching = STEREO_VISION::LIBELAS;
            break;
    }

    switch(this->BLFfiltering_id)
    {
        case 0:
            this->BLFfiltering = STEREO_VISION::BLF_DISABLED;
            break;
        case 1:
            this->BLFfiltering = STEREO_VISION::BLF_ORIGINAL;
            break;
        case 2:
            this->BLFfiltering = STEREO_VISION::BLF_CUDA;
            break;
    }

    switch(this->WLSfiltering_id)
    {
        case 0:
            this->WLSfiltering = STEREO_VISION::WLS_DISABLED;
            break;
        case 1:
            this->WLSfiltering = STEREO_VISION::WLS_ENABLED;
            break;
        case 2:
            this->WLSfiltering = STEREO_VISION::WLS_LRCHECK;
            break;
    }

}

void GUI::convertEnumToID()
{

    switch(this->stereo_matching)
    {
        case STEREO_VISION::SGBM_OPENCV:
            this->stereo_matching_id = 0;
            break;
        case STEREO_VISION::SGBM_CUDA:
            this->stereo_matching_id = 1;
            break;
        case STEREO_VISION::LIBELAS:
            this->stereo_matching_id = 2;
            break;

    }

    switch(this->BLFfiltering)
    {
        case STEREO_VISION::BLF_DISABLED:
            this->BLFfiltering_id = 0;
            break;
        case STEREO_VISION::BLF_ORIGINAL:
            this->BLFfiltering_id = 1;
            break;
        case STEREO_VISION::BLF_CUDA:
            this->BLFfiltering_id = 2;
            break;

    }

    switch(this->WLSfiltering)
    {
        case STEREO_VISION::WLS_DISABLED:
            this->WLSfiltering_id = 0;
            break;
        case STEREO_VISION::WLS_ENABLED:
            this->WLSfiltering_id = 1;
            break;
        case STEREO_VISION::WLS_LRCHECK:
            this->WLSfiltering_id = 2;
            break;

    }



}

bool GUI::isDone()
{
    return this->done;
}

GUI::~GUI()
{
    this->killGUI();
}

bool GUI::toRecalibrate()
{
    return this->recalibrate;
}


void GUI::setUpdated(bool v)
{
    this->updated = v;
}

void GUI::setUpdated(bool v, bool v2)
{
    this->updated = v;
    this->recalibrate = v2;
}
