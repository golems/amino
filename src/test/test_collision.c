#include "amino.h"
#include "amino/rx/amino_fcl.h"

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_collision.h"



int main( int argc, char **argv)
{
    (void) argc; (void) argv;

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

    aa_rx_sg_index(sg);
    aa_rx_sg_index(sg1);

    aa_rx_sg_cl_init(sg);
    aa_rx_sg_cl_init(sg1);

    {
        struct aa_rx_cl *cl = aa_rx_cl_create(sg);
        aa_rx_frame_id n = aa_rx_sg_frame_count(sg);
        double TF_rel[7*n];
        double TF_abs[7*n];
        aa_rx_sg_tf(sg, 0, NULL,
                    n,
                    TF_rel, 7,
                    TF_abs, 7 );
        int collision = aa_rx_cl_check( cl, (size_t)n, TF_abs, 7 );
        assert( collision );
    }

    {
        struct aa_rx_cl *cl = aa_rx_cl_create(sg1);
        aa_rx_frame_id n = aa_rx_sg_frame_count(sg1);
        double TF_rel[7*n];
        double TF_abs[7*n];
        aa_rx_sg_tf(sg1, 0, NULL,
                    n,
                    TF_rel, 7,
                    TF_abs, 7 );
        int collision = aa_rx_cl_check( cl, (size_t)n, TF_abs, 7 );
        assert( !collision );
    }

    return 0;
}
