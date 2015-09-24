/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY HEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */
#define GL_GLEXT_PROTOTYPES

// Fuck you, C++
#define protected public
#define protected public

#include <error.h>
#include <stdio.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/sbl/SBL.h>

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/amino_gl.h"
#include "amino/rx/amino_sdl.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_collision.h"
#include "amino/rx/scene_ompl.h"

const int SCREEN_WIDTH = 1000;
const int SCREEN_HEIGHT = 1000;


// class rxGoalSampleableRegion : public ompl::base::GoalSampleableRegion
// {
// public:
//     rxGoalSampleableRegion(const ompl::base::SpaceInformationPtr &si) :
//         ompl::base::GoalSampleableRegion(si) {
//         setThreshold(0.1);
//     }
//     virtual double distanceGoal(const ompl::base::State *st) const {

//         // perform any operations and return a double indicating the distance to the goal
//     }

//     virtual void sampleGoal(ompl::base::State *st) const {

//     }


//     virtual unsigned int maxSampleCount( ) const {

//     }
// };


AA_API struct aa_rx_sg *generate_scenegraph(struct aa_rx_sg *sg);
struct aa_rx_sg *scenegraph;

amino::rxSpace *g_space;
amino::rxStateValidityChecker *g_checker;

static void motion_plan( const struct aa_rx_sg *sg)
{
    // Motion Planning
    //ompl::base::StateSpacePtr ss( new ompl::base::RealVectorStateSpace(7));
    //ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(ss));
    //ompl::base::ScopedState<> start_state(ss);
    //for( size_t i = 0; i < 7; i ++ ) (start_state)[i] = .1;

    const char *names[] = {"right_s0",
                           "right_s1",
                           "right_e0",
                           "right_e1",
                           "right_w0",
                           "right_w1",
                           "right_w2"};
    g_space = new amino::rxSpace (scenegraph, 7, names );
    double q0[g_space->dim_all()];
    AA_MEM_ZERO(q0, g_space->dim_all());
    g_checker = new amino::rxStateValidityChecker(g_space, q0);


    //ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

    //rxGoalSampleableRegion goal_sampler(si);

    // ompl::base::PlannerPtr planner(new ompl::geometric::SBL(si));
    // planner->setProblemDefinition(pdef);
    // planner->solve(1.0);
    // if (pdef->getSolutionPath())
    // {
    //     // do something with the solution
    // }

}


void check_error( const char *name ){
    for (GLenum err = glGetError(); err != GL_NO_ERROR; err = glGetError()) {
        fprintf(stderr, "error %s: %d: %s\n",  name,  (int)err, gluErrorString(err));
    }
}

struct display_cx {
    const struct aa_gl_globals *globals;
    struct aa_rx_cl *cl;
    double q;
    aa_rx_config_id i_q;
    struct timespec last;
};

int display( void *cx_, int updated, const struct timespec *now )
{
    struct display_cx *cx = (struct display_cx *)cx_;
    const struct aa_gl_globals *globals = cx->globals;


    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    check_error("glClearColor");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    check_error("glClear");

    aa_rx_frame_id n = aa_rx_sg_frame_count(scenegraph);
    aa_rx_frame_id m = aa_rx_sg_config_count(scenegraph);
    double q[m];
    AA_MEM_ZERO(q,m);


    ompl::base::ScopedState<> state(g_space->state_space);
    AA_MEM_ZERO(&state[0], g_space->dim_set());

    if( cx->last.tv_sec || cx->last.tv_nsec ) {
        double dt = aa_tm_timespec2sec( aa_tm_sub(*now, cx->last) );
        cx->q += dt * 45 * (M_PI/180);

    }
    state[0] = cx->q;
    g_space->state_set(state.get(),q);
    bool col = g_checker->isValid(state.get());

    //q[ cx->i_q ] = cx->q;

    double TF_rel[7*n];
    double TF_abs[7*n];
    aa_rx_sg_tf(scenegraph, m, q,
                n,
                TF_rel, 7,
                TF_abs, 7 );
    aa_rx_sg_render( scenegraph, globals,
                     (size_t)n, TF_abs, 7 );

    memcpy( &cx->last, now, sizeof(*now) );

    //int col = aa_rx_cl_check( cx->cl, n, TF_abs, 7, NULL );

    printf("in collision: %s\n",
           col ? "yes" : "no" );

    return 1;
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;
    SDL_Window* window = NULL;
    SDL_GLContext gContext = NULL;

    // Initialize scene graph
    scenegraph = generate_scenegraph(NULL);
    aa_rx_sg_index(scenegraph);
    aa_rx_sg_cl_init(scenegraph);

    // Do Planning
    motion_plan(scenegraph);

    // Open GL
    aa_sdl_gl_window( "SDL Test",
                      SDL_WINDOWPOS_UNDEFINED,
                      SDL_WINDOWPOS_UNDEFINED,
                      SCREEN_WIDTH,
                      SCREEN_HEIGHT,
                      SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE,
                      &window, &gContext);

    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));
    aa_rx_sg_gl_init(scenegraph);

    // Initialize globals
    struct aa_gl_globals *globals = aa_gl_globals_create();
    // global camera
    {
        double world_E_camera_home[7] = AA_TF_QUTR_IDENT_INITIALIZER;
        double eye[3] = {3,2,1.25};
        double target[3] = {0,0,0};
        double up[3] = {0,0,1};
        aa_tf_qutr_mzlook( eye, target, up, world_E_camera_home );
        aa_gl_globals_set_camera_home( globals, world_E_camera_home );
        aa_gl_globals_home_camera( globals );

    }

    // global lighting
    {
        double v_light[3] = {.5,1,5};
        double ambient[3] = {.1,.1,.1};
        aa_gl_globals_set_light_position( globals, v_light );
        aa_gl_globals_set_ambient(globals, ambient);
    }

    aa_gl_globals_set_show_visual(globals, 0);
    aa_gl_globals_set_show_collision(globals, 1);

    struct display_cx cx = {0};
    cx.globals = globals;
    cx.i_q = aa_rx_sg_config_id(scenegraph, "left_s0");
    cx.cl = aa_rx_cl_create( scenegraph );

    {
        aa_rx_frame_id n = aa_rx_sg_frame_count(scenegraph);
        aa_rx_frame_id m = aa_rx_sg_config_count(scenegraph);
        double q[m];
        AA_MEM_ZERO(q,m);
        double TF_rel[7*n];
        double TF_abs[7*n];
        aa_rx_sg_tf(scenegraph, m, q,
                    n,
                    TF_rel, 7,
                    TF_abs, 7 );

        struct aa_rx_cl_set *allowed = aa_rx_cl_set_create( scenegraph );
        int col = aa_rx_cl_check( cx.cl, n, TF_abs, 7, allowed );
        aa_rx_cl_allow_set( cx.cl, allowed );
        aa_rx_cl_set_destroy( allowed );
    }

    aa_sdl_display_loop( window, globals,
                         display,
                         &cx );

    SDL_GL_DeleteContext(gContext);
    SDL_DestroyWindow( window );

    SDL_Quit();
    return 0;
}
