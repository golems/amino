#include <amino.h>
#include <amino/rx.h>

#include "amino/rx/scene_win.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/octree_geom.hpp"
#include "amino/rx/scene_plugin.h"

#include <stdio.h>

static const int SCREEN_WIDTH = 1000;
static const int SCREEN_HEIGHT = 1000;

int
main (int argc, char **argv)
{
    if( 2 != argc ) {
        fprintf(stderr, "USAGE: octomap_test OCTREE_FILE");
    }

    struct aa_rx_octree* oCloud =
        aa_rx_geom_read_octree_from_file(argv[1]);

    std::cout << "Octree created" << std::endl;

    struct aa_rx_sg *sg = aa_rx_sg_create();

    std::cout << "scenegraph created" << std::endl;

    static const double q[4] = {0, 0, 0, 1};
    static const double v[3] = {0, 0, 0};


    // aa_rx_sg_add_frame_fixed(sg, "", "oct", q, v);

    // std::cout << "frame added" << std::endl;

    // struct aa_rx_geom *geom;
    {
        struct aa_rx_geom_opt *opt = aa_rx_geom_opt_create();
        std::cout << "options created" << std::endl;
        aa_rx_geom_opt_set_color3(opt, 0.5, 0.0, 0.5);
        aa_rx_geom_opt_set_alpha(opt, 1.0);
        aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
        aa_rx_geom_opt_set_visual(opt, 1);
        aa_rx_geom_opt_set_collision(opt, 1);
        aa_rx_geom_opt_set_no_shadow(opt, 0);

        // geom = aa_rx_geom_octree(opt, oCloud);
        aa_rx_sg_add_octree(sg, "", "octomap", oCloud, opt);
        std::cout << "octree added" << std::endl;
        // aa_rx_geom_attach(sg, "oct", geom);
        aa_rx_geom_opt_destroy(opt);
    }

    aa_rx_sg_init(sg); /* initialize scene graph internal structures */
    printf("Frame Count: %d\n", aa_rx_sg_frame_count(sg));


    // Remove frames from large octomaps
    char **names = AA_NEW_AR(char *, aa_rx_sg_frame_count(sg));
    for( size_t i = 0; i < aa_rx_sg_frame_count(sg); i ++ ) {
        names[i] = (char*)aa_rx_sg_frame_name(sg,i);
    }
    for( size_t i = 100; i < aa_rx_sg_frame_count(sg); i ++ ) {
        aa_rx_sg_rm_frame(sg,names[i]);
    }


    // Add a grid
    {
        aa_rx_sg_add_frame_fixed(sg, "", "grid",
                                 aa_tf_quat_ident, aa_tf_vec_ident);
        struct aa_rx_geom_opt *opt = aa_rx_geom_opt_create();
        aa_rx_geom_opt_set_color3(opt, 0.0, 0.0, 1.0);
        aa_rx_geom_opt_set_alpha(opt, 1.0);
        aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
        aa_rx_geom_opt_set_visual(opt, 1);
        aa_rx_geom_opt_set_collision(opt, 1);
        aa_rx_geom_opt_set_no_shadow(opt, 0);

        double dim[2]={10,10};
        double delta[2]={1,1};

        aa_rx_geom_attach(sg, "grid",
                          aa_rx_geom_grid(opt, dim, delta, 0.01));

        aa_rx_geom_opt_destroy(opt);
    }


    aa_rx_sg_init(sg); /* re-initialize after removing frames */
    printf("Displayed Frame Count: %d\n", aa_rx_sg_frame_count(sg));


    struct aa_rx_win * win =
        aa_rx_win_default_create ( "Octree", SCREEN_WIDTH, SCREEN_HEIGHT );

    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));

    // setup scene graph
    aa_rx_win_sg_gl_init(win, sg); /* Initialize scene graph GL-rendering objects */
    aa_rx_win_set_sg(win, sg); /* Set the scenegraph for the window */

    // start display
    aa_rx_win_run();

    // Cleanup
    aa_rx_sg_destroy(sg);
    aa_rx_win_destroy(win);
    SDL_Quit();

    return (0);
}
