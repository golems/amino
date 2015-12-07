#include "amino.h"
#include "amino/rx/scene_fcl.h"

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_collision.h"


static void test_box()
{
    struct aa_rx_sg *sg = aa_rx_sg_create();
    struct aa_rx_sg *sg1 = aa_rx_sg_create();

    struct aa_rx_geom_opt *opt_cl = aa_rx_geom_opt_create();
    aa_rx_geom_opt_set_collision(opt_cl, 1);


    aa_rx_sg_add_frame_fixed( sg,
                              "", "a",
                              aa_tf_quat_ident, aa_tf_vec_ident );
    aa_rx_sg_add_frame_fixed( sg,
                              "", "b",
                              aa_tf_quat_ident, aa_tf_vec_ident );

    double v1[3] = {10,0,0};
    aa_rx_sg_add_frame_fixed( sg1,
                              "", "a",
                              aa_tf_quat_ident, aa_tf_vec_ident );
    aa_rx_sg_add_frame_fixed( sg1,
                              "", "b",
                              aa_tf_quat_ident, v1 );


    double d[3] = {.1, .1, .1};
    aa_rx_geom_attach( sg, "a", aa_rx_geom_box(opt_cl, d) );
    aa_rx_geom_attach( sg, "b", aa_rx_geom_box(opt_cl, d) );

    aa_rx_geom_attach( sg1, "a", aa_rx_geom_box(opt_cl, d) );
    aa_rx_geom_attach( sg1, "b", aa_rx_geom_box(opt_cl, d) );

    aa_rx_sg_init(sg);
    aa_rx_sg_init(sg1);

    aa_rx_sg_cl_init(sg);
    aa_rx_sg_cl_init(sg1);

    {
        struct aa_rx_cl *cl = aa_rx_cl_create(sg);
        size_t n = aa_rx_sg_frame_count(sg);
        double TF_rel[7*n];
        double TF_abs[7*n];
        aa_rx_sg_tf(sg, 0, NULL,
                    n,
                    TF_rel, 7,
                    TF_abs, 7 );
        int collision = aa_rx_cl_check( cl, (size_t)n, TF_abs, 7, NULL );
        assert( collision );
    }

    {
        struct aa_rx_cl *cl = aa_rx_cl_create(sg1);
        size_t n = aa_rx_sg_frame_count(sg1);
        double TF_rel[7*n];
        double TF_abs[7*n];
        aa_rx_sg_tf(sg1, 0, NULL,
                    n,
                    TF_rel, 7,
                    TF_abs, 7 );
        int collision = aa_rx_cl_check( cl, (size_t)n, TF_abs, 7, NULL );
        assert( !collision );
    }

}

void test_cylinder()
{

    struct aa_rx_sg *sg = aa_rx_sg_create();
    struct aa_rx_geom_opt *opt_cl = aa_rx_geom_opt_create();
    aa_rx_geom_opt_set_collision(opt_cl, 1);


    aa_rx_sg_add_frame_fixed( sg,
                              "", "a",
                              aa_tf_quat_ident, aa_tf_vec_ident );
    double vb[3] = {0,0,.51};
    aa_rx_sg_add_frame_fixed( sg,
                              "", "b",
                              aa_tf_quat_ident, vb );

    double vc[3] = {0,0,-.49};
    aa_rx_sg_add_frame_fixed( sg,
                              "", "c",
                              aa_tf_quat_ident, vc );

    double d[3] = {1, 1, 1};
    aa_rx_geom_attach( sg, "a", aa_rx_geom_box(opt_cl, d) );
    aa_rx_geom_attach( sg, "b", aa_rx_geom_cylinder(opt_cl, 1, .5) );


    aa_rx_sg_init(sg);
    aa_rx_sg_cl_init(sg);

    {
        struct aa_rx_cl *cl = aa_rx_cl_create(sg);
        size_t n = aa_rx_sg_frame_count(sg);
        double TF_rel[7*n];
        double TF_abs[7*n];
        aa_rx_sg_tf(sg, 0, NULL,
                    n,
                    TF_rel, 7,
                    TF_abs, 7 );
        int collision = aa_rx_cl_check( cl, (size_t)n, TF_abs, 7, NULL );
        assert( !collision );
    }

    aa_rx_geom_attach( sg, "c", aa_rx_geom_cylinder(opt_cl, 1, .5) );
    aa_rx_sg_init(sg);
    aa_rx_sg_cl_init(sg);
    {
        struct aa_rx_cl *cl = aa_rx_cl_create(sg);
        struct aa_rx_cl_set *set = aa_rx_cl_set_create(sg);
        size_t n = aa_rx_sg_frame_count(sg);
        double TF_rel[7*n];
        double TF_abs[7*n];
        aa_rx_sg_tf(sg, 0, NULL,
                    n,
                    TF_rel, 7,
                    TF_abs, 7 );
        int collision = aa_rx_cl_check( cl, (size_t)n, TF_abs, 7, set );
        assert( collision );
        assert( 0 == aa_rx_cl_set_get( set,
                                       aa_rx_sg_frame_id(sg, "a"),
                                       aa_rx_sg_frame_id(sg, "b") ) );
        assert( 1 == aa_rx_cl_set_get( set,
                                       aa_rx_sg_frame_id(sg, "a"),
                                       aa_rx_sg_frame_id(sg, "c") ) );
        assert( 0 == aa_rx_cl_set_get( set,
                                       aa_rx_sg_frame_id(sg, "b"),
                                       aa_rx_sg_frame_id(sg, "c") ) );
    }
}


int main( int argc, char **argv)
{
    (void) argc; (void) argv;
    aa_rx_cl_init();
    test_box();
    test_cylinder();

    return 0;
}
