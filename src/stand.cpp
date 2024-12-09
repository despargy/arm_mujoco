#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>


#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <iostream>
#include <vector>
//simulation end time
// double simend = 30;

// PD controller gains
const double kp = 50.0; // Proportional gain
const double kd = 0.2;  // Derivative gain
#include <map>

// Desired joint positions for standing
std::map<std::string, double> init_positions = {
    {"back_bkz", 0.0}, {"back_bky", 0.0}, {"back_bkx", 0.0},
    {"l_leg_hpz", 0.0}, {"l_leg_hpx", 0.0}, {"l_leg_hpy", 0.0},
    {"l_leg_kny", 0.0}, {"l_leg_aky", 0.0}, {"l_leg_akx", 0.0},
    {"r_leg_hpz", 0.0}, {"r_leg_hpx", 0.0}, {"r_leg_hpy", 0.0},
    {"r_leg_kny", 0.0}, {"r_leg_aky", 0.0}, {"r_leg_akx", 0.0},
    {"l_arm_shz", 0.0}, {"l_arm_shx", 0.0}, {"l_arm_ely", 0.0},
    {"l_arm_elx", 0.0}, {"l_arm_uwy", 0.0}, {"l_arm_mwx", 0.0}, {"l_arm_lwy", 0.0},
    {"r_arm_shz", 0.0}, {"r_arm_shx", 0.0}, {"r_arm_ely", 0.0},
    {"r_arm_elx", 0.0}, {"r_arm_uwy", 0.0}, {"r_arm_mwx", 0.0}, {"r_arm_lwy", 0.0},
    {"neck_ay", 0.0}
};


//related to writing data to a file
int loop_index = 0;

//Change the xml file
char path[] = "../../../atlas-mujoco/";
char xmlfile[] = "model/scene.xml"; //

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
mjvFigure fig;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
// float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

/// HERE ///
double time_ctrl_0 = 0.1;

void set_initial_pos(const mjModel* m, mjData* d)
{
    for(const auto& pos : init_positions)
    {
        const char* joint_name = pos.first.c_str(); // joint names per act
        int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_name);
        d->qpos[m->jnt_qposadr[joint_id]] = pos.second;
        std::cout<< "qpos for "<<joint_name <<std::endl;
    }
    for(int i=0; i<m->nq; i++)
        d->qvel[i] = 0.0; // set all vel to zero
    
    mj_resetData(m,d);
    std::cout<<"Init pos once"<<std::endl;

}   
void apply_force_to_torso(const mjModel* m, mjData* d) {
    // Get the body ID for the torso
    const char* torso_name = "ltorso";  // Replace with the actual name of your torso body
    int torso_id = mj_name2id(m, mjOBJ_BODY, torso_name);

    if (torso_id == -1) {
        std::cerr << "Error: Torso body not found!" << std::endl;
        return;
    }

    // Apply force to the torso
    // Note: `force` is a 3-element array for x, y, z components of the force
    std::memset(d->xfrc_applied + 6 * torso_id, 0, 6 * sizeof(double));  // Clear previous values
    d->xfrc_applied[6 * torso_id + 0] = 0;//force[0];  // Force in X direction
    d->xfrc_applied[6 * torso_id + 1] = 200;//force[1];  // Force in Y direction
    d->xfrc_applied[6 * torso_id + 2] = 0;//force[2];  // Force in Z direction

} 
void my_controller(const mjModel* m, mjData* d)
{
    static bool init_ONCE = false;
    if (!init_ONCE)
    {
        set_initial_pos(m, d);
        init_ONCE = true;
    }
    for (int i = 0; i < m->nu; ++i) {
        int joint_id = m->actuator_trnid[i*2];  // This retrieves the object affected by the actuator
        // Retrieve the joint name using the joint ID
        const char* joint_name = mj_id2name(m, mjOBJ_JOINT, joint_id);
        if (!joint_name) {
            std::cerr << "Joint name not found for actuator " << i << std::endl;
            continue;
        }

        // Check if the joint name is in the desired_positions map (it should be after initialization)
        auto it = init_positions.find(joint_name);
        if (it != init_positions.end()) {
            double q_desired = it->second;

            // Use joint ID to get position and velocity indices
            int qpos_adr = m->jnt_qposadr[joint_id];
            int qvel_adr = m->jnt_dofadr[joint_id];

            // Get the current state of the joint
            double q_current = d->qpos[qpos_adr];
            double q_velocity = d->qvel[qvel_adr];

            // Compute control torque using PD control
            double torque = kp * (q_desired - q_current) - kd * q_velocity;

            // Set the actuator control signal (torque)
            d->ctrl[i] = torque;

            // // Optionally print the joint's desired and current position for debugging
            std::cout << "Joint " << joint_name 
                    << " - Desired position: " << q_desired 
                    << ", Current position: " << q_current 
                    // << ", Velocity: " << q_velocity 
                    << ", Torque: " << torque << std::endl;
        }
    }
    apply_force_to_torso(m,d);

}

// main function
int main(int argc, const char** argv)
{

    char xmlpath[100]={};

    strcat(xmlpath,path);
    strcat(xmlpath,xmlfile);

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(xmlpath, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    // mjv_defaultFigure(&fig);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context
    
    // mjrRect fig_rec{0,0,120,300};
    // mjr_figure(fig_rec, &fig, &con);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {-129.477651, -3.102665, 2.209726, -0.047404, -0.001591, 0.330533}; //view the left side (for ll, lh, left_side)
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];


    // install control callback
    mjcb_control = my_controller;  // set the myTopLevel instead of the default Mujoco's one


    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }    

        // if (d->time>(time_ctrl_0 - 0.002 ))
        // {
        //    init_pos(m,d);
        // }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        // opt.frame = mjFRAME_WORLD; // Remove this one HERE
        cam.lookat[0] = d->qpos[0] ;
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        // printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    // mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}