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
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "config.h"

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/rxtype_internal.h"

#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"

#include "amino/rx/scene_geom.h"


#include "amino/rx/scene_collision.h"


#include <fcl/fcl.h>

#include "amino/rx/scene_collision_internal.h"
#include "amino/rx/scene_fcl.h"


static pthread_once_t cl_once = PTHREAD_ONCE_INIT;

static void
cl_init_once( void )
{
    aa_rx_cl_geom_destroy_fun = aa_rx_cl_geom_destroy;
}

/* Initialize collision handling */
AA_API void
aa_rx_cl_init( )
{
    if( pthread_once( &cl_once, cl_init_once ) ) {
        perror("pthread_once");
        abort();
    }
}

struct aa_rx_cl_geom {
    std::shared_ptr<::amino::fcl::CollisionGeometry> ptr;
    aa_rx_cl_geom( ::amino::fcl::CollisionGeometry *ptr_) :
        ptr(ptr_) { }

    ~aa_rx_cl_geom() { }
};

AA_API void
aa_rx_cl_geom_destroy( struct aa_rx_cl_geom *cl_geom ) {
    delete cl_geom;
}


static ::amino::fcl::CollisionGeometry *
cl_init_mesh( double scale, const struct aa_rx_mesh *mesh )
{

    //printf("mesh\n");
    //printf("n_verts: %u\n",mesh->n_vertices );
    //printf("n_indices: %u\n",mesh->n_indices );
    /* TODO: FCL should be able to reference external arrays for mesh
     * vertices and indices so that we don't have to copy the mesh.
     */
    std::vector<::amino::fcl::Vec3> vertices;
    std::vector<fcl::Triangle> triangles;


    /* fill vertices */
    {
        size_t n;
        const float *v = aa_rx_mesh_get_vertices(mesh, &n);

        for( unsigned i = 0, j=0; i < n; i++ ) {
            unsigned x=j++;
            unsigned y=j++;
            unsigned z=j++;
            vertices.push_back( ::amino::fcl::Vec3( scale*v[x],
                                                    scale*v[y],
                                                    scale*v[z]) );
        }
    }
    //printf("filled verts\n");
    /* fill faces*/
    {
        size_t n;
        const unsigned *f = aa_rx_mesh_get_indices(mesh, &n);
        //printf("n_indices: %u\n",mesh->n_indices );
        for( unsigned i=0, j=0; i < n; i++ ) {
            unsigned x=j++;
            unsigned y=j++;
            unsigned z=j++;
            //printf("tri: %u, %u, %u, (%u)\n", x,y,z, 3*mesh->n_indices);
            triangles.push_back( fcl::Triangle(f[x], f[y], f[z]) );
        }
    }
    //printf("filled tris\n");

    auto model = new(fcl::BVHModel<::amino::fcl::OBBRSS>);
    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();

    return model;
}

static void cl_init_helper( void *cx, aa_rx_frame_id frame_id, struct aa_rx_geom *geom )
{
    (void)cx; (void)frame_id;
    struct aa_rx_sg *sg = (struct aa_rx_sg*)cx;
    const struct aa_rx_geom_opt* opt = aa_rx_geom_get_opt(geom);

    /* not a collision geometry */
    if( ! aa_rx_geom_opt_get_collision(opt) ) {
        //printf("  not a collision geom\n");
        return;
    }

    /* already have collision object */
    if( aa_rx_geom_get_collision(geom) ) {
        //printf("  already has collision geom\n");
        return;
    }


    /* Ok, now do it */
    ::amino::fcl::CollisionGeometry *ptr = NULL;
    enum aa_rx_geom_shape shape_type;
    void *shape_ = aa_rx_geom_shape(geom, &shape_type);
    double scale = aa_rx_geom_opt_get_scale(opt);

    // printf("creating collision geometry for frame %s (%s)\n",
    //        aa_rx_sg_frame_name(sg, frame_id),
    //        aa_rx_geom_shape_str(shape_type) );

    switch( shape_type ) {
    case AA_RX_NOSHAPE: {
        break;
    }
    case AA_RX_MESH: {
        struct aa_rx_mesh *shape = (struct aa_rx_mesh *)  shape_;
        ptr = cl_init_mesh(scale,shape);
        break;
    }
    case AA_RX_BOX: {
        struct aa_rx_shape_box *shape = (struct aa_rx_shape_box *)  shape_;
        ptr = new ::amino::fcl::Box(scale*shape->dimension[0],
                                    scale*shape->dimension[1],
                                    scale*shape->dimension[2]);
        break;
    }
    case AA_RX_SPHERE: {
        struct aa_rx_shape_sphere *shape = (struct aa_rx_shape_sphere *)  shape_;
        ptr = new ::amino::fcl::Sphere(scale*shape->radius);
        break;
    }
    case AA_RX_CYLINDER: {
        struct aa_rx_shape_cylinder *shape = (struct aa_rx_shape_cylinder *)  shape_;
        ptr = new ::amino::fcl::Cylinder(scale*shape->radius, scale*shape->height);
        break;
    }
    case AA_RX_CONE: {
        // struct aa_rx_shape_cone *shape = (struct aa_rx_shape_cone *)  shape_;
        break;
    }
    case AA_RX_GRID: {
        // struct aa_rx_shape_grid *shape = (struct aa_rx_shape_grid *)  shape_;
        break;
    }
    }

    if(ptr) {
        struct aa_rx_cl_geom *cl_geom = new aa_rx_cl_geom(ptr);
        cl_geom->ptr->setUserData(geom); // FCL user data is the amino geometry object
        aa_rx_geom_set_collision(geom, cl_geom); // Set the amino geometry collision object
    } else {
        fprintf(stderr, "Unimplemented collision type: %s\n", aa_rx_geom_shape_str( shape_type ) );
    }
}


void aa_rx_sg_cl_init( struct aa_rx_sg *scene_graph )
{
    if(  aa_rx_sg_is_clean_collision(scene_graph) ) {
        return;
    }

    aa_rx_cl_init();

    aa_rx_sg_map_geom( scene_graph, &cl_init_helper, scene_graph );

    amino::SceneGraph *sg = scene_graph->sg;
    sg->allowed_indices1.clear();
    sg->allowed_indices2.clear();

    for (auto it = sg->allowed.begin(); it!= sg->allowed.end(); it++){
        sg->allowed_indices1.push_back(aa_rx_sg_frame_id(scene_graph, it->first));
        sg->allowed_indices2.push_back(aa_rx_sg_frame_id(scene_graph, it->second));
    }

    aa_rx_sg_clean_collision(scene_graph);
}


struct aa_rx_cl
{
    const struct aa_rx_sg *sg;
    ::amino::fcl::BroadPhaseCollisionManager *manager;
    std::vector<::amino::fcl::CollisionObject*> *objects;

    // A bit-matrix of allowable collisions
    struct aa_rx_cl_set *allowed;
};

static void cl_create_helper( void *cx_, aa_rx_frame_id frame_id, struct aa_rx_geom *geom )
{
    struct aa_rx_cl *cx = (struct aa_rx_cl*)cx_;

    //printf("adding cl_geom for %s\n", aa_rx_sg_frame_name(cx->sg, frame_id) );

    struct aa_rx_cl_geom *cl_geom = aa_rx_geom_get_collision(geom);
    if( NULL == cl_geom ) return;

    ::amino::fcl::CollisionObject *obj = new ::amino::fcl::CollisionObject( cl_geom->ptr );
    obj->setUserData( (void*) ((intptr_t) frame_id) );
    cx->manager->registerObject(obj);
    cx->objects->push_back( obj );
}

struct aa_rx_cl *
aa_rx_cl_create( const struct aa_rx_sg *scene_graph )
{
    aa_rx_sg_ensure_clean_collision(scene_graph);

    struct aa_rx_cl *cl = new aa_rx_cl;
    cl->sg = scene_graph;
    cl->objects = new ::std::vector<::amino::fcl::CollisionObject*>;
    cl->manager = new ::amino::fcl::DynamicAABBTreeCollisionManager();

    cl->allowed = aa_rx_cl_set_create(scene_graph);
    aa_rx_sg_cl_set_copy(scene_graph, cl->allowed);

    aa_rx_sg_map_geom( scene_graph, &cl_create_helper, cl );

    cl->manager->setup();

    return cl;
}

void
aa_rx_cl_destroy( struct aa_rx_cl *cl )
{
    for( ::amino::fcl::CollisionObject *o : *cl->objects ) {
        delete o;
    }

    delete cl->manager;
    delete cl->objects;
    aa_rx_cl_set_destroy( cl->allowed );
    delete cl;
}

AA_API void
aa_rx_cl_allow( struct aa_rx_cl *cl,
                aa_rx_frame_id id0,
                aa_rx_frame_id id1,
                int allowed )
{
    aa_rx_cl_set_set( cl->allowed, id0, id1, allowed );
}

AA_API void
aa_rx_cl_allow_set( struct aa_rx_cl *cl,
                    const struct aa_rx_cl_set *set )
{
    aa_rx_cl_set_fill( cl->allowed, set );
}


AA_API void
aa_rx_cl_allow_name( struct aa_rx_cl *cl,
                const char *frame0,
                const char *frame1,
                int allowed )
{
    aa_rx_cl_allow( cl,
                    aa_rx_sg_frame_id(cl->sg, frame0),
                    aa_rx_sg_frame_id(cl->sg, frame1),
                    allowed );
}

struct cl_check_data {
    int result;
    struct aa_rx_cl *cl;
    struct aa_rx_cl_set *cl_set;
};

static bool
cl_check_callback( ::amino::fcl::CollisionObject *o1,
                   ::amino::fcl::CollisionObject *o2,
                   void *data_ )
{
    struct cl_check_data *data = (struct cl_check_data*)data_;
    aa_rx_frame_id id1 = (intptr_t) o1->getUserData();
    aa_rx_frame_id id2 = (intptr_t) o2->getUserData();

    //const struct aa_rx_sg *sg = data->cl->sg;
    //const char *name1 = aa_rx_sg_frame_name(sg, id1);
    //const char *name2 = aa_rx_sg_frame_name(sg, id2);

    /* Skip geometry in the same frame */
    if( id1 == id2 ) return false;

    /* Check if allowed collision */
    if( aa_rx_cl_set_get(data->cl->allowed,id1,id2) ) {
        return false;
    }


    ::amino::fcl::CollisionRequest request;
    ::amino::fcl::CollisionResult result;
    ::fcl::collide(o1, o2, request, result);

    if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts)) {
        //printf("collide: %s x %s\n", name1, name2 );
        /* In Collision */
        data->result = 1;

        /* Short Circuit? */
        if( data->cl_set ) {
            // printf("Filling collision\n");
            aa_rx_cl_set_set( data->cl_set, id1, id2, 1 );
        } else {
            // printf("Short circuit\n");
            return true;
        }
    }

    return false;
}


static void
s_update_tf( const struct aa_rx_cl *cl,
            void (*f)(const void *cx, aa_rx_frame_id id, double E[7]),
            const void *cx )
{
    /* Update Transforms */
    for( auto itr = cl->objects->begin();
         itr != cl->objects->end();
         itr++ )
    {
        ::amino::fcl::CollisionObject *obj = *itr;
        aa_rx_frame_id id = (intptr_t) obj->getUserData();
        double TF_obj[7];
        f(cx,id,TF_obj);

        enum aa_rx_geom_shape shape_type;
        struct aa_rx_geom *geom = (struct aa_rx_geom*)obj->collisionGeometry()->getUserData();
        void *shape_ = aa_rx_geom_shape( geom, &shape_type);

        /* Special case cylinders.
         * Amino cylinders extend in +Z
         * FCL cylinders extend in both +/- Z.
         */
        if( AA_RX_CYLINDER == shape_type ) {
            struct aa_rx_shape_cylinder *shape = (struct aa_rx_shape_cylinder *)  shape_;
            double E[7] = {0,0,0,1, 0,0, shape->height/2};
            double E1[7];
            aa_tf_qutr_mul(TF_obj, E, E1);
            obj->setTransform(amino::fcl::qutr2fcltf(E1));
        } else {
            obj->setTransform( amino::fcl::qutr2fcltf(TF_obj) );
        }
    }
    cl->manager->update();

}


static int
s_cl_check( struct aa_rx_cl *cl,
            void (*f)(const void *cx, aa_rx_frame_id id, double E[7]),
            const void *cx,
            struct aa_rx_cl_set *cl_set )
{
    s_update_tf(cl,f,cx);

    /* Check Collision */
    struct cl_check_data data;
    data.result = 0;
    data.cl = cl;
    data.cl_set = cl_set;

    cl->manager->collide( &data, cl_check_callback );
    return data.result;
}

struct check_cx_array {
    const double *TF;
    size_t ldTF;
};

void check_helper_array( const void *vcx, aa_rx_frame_id id, double E[7] )
{
    struct check_cx_array *cx = (struct check_cx_array *) vcx;
    const double *TF_obj = cx->TF+id*cx->ldTF;
    AA_MEM_CPY(E, TF_obj, 7);
}

int
aa_rx_cl_check( struct aa_rx_cl *cl,
                size_t n_tf,
                const double *TF, size_t ldTF,
                struct aa_rx_cl_set *cl_set )
{
    struct check_cx_array cx;
    cx.TF = TF;
    cx.ldTF = ldTF;
    return s_cl_check(cl, check_helper_array, &cx, cl_set);
}


void check_helper_fk( const void *vcx, aa_rx_frame_id id, double E[7] )
{
    aa_rx_fk_get_abs_qutr( (struct aa_rx_fk *)vcx, id, E );
}

AA_API int
aa_rx_cl_check_fk( struct aa_rx_cl *cl,
                   struct aa_rx_fk *fk,
                   struct aa_rx_cl_set *cl_set ) {
    return s_cl_check(cl, check_helper_fk, fk, cl_set);
}

AA_API void
aa_rx_sg_get_collision(const struct aa_rx_sg* scene_graph, size_t n_q_arg, const double* q, struct aa_rx_cl_set* cl_set)
{
    size_t n_f = aa_rx_sg_frame_count(scene_graph);
    size_t n_q = aa_rx_sg_config_count(scene_graph);

    assert(n_q == n_q_arg);



    double TF_rel[7*n_f];
    double TF_abs[7*n_f];

    aa_rx_sg_tf(scene_graph, n_q, q,
                n_f,
                TF_rel, 7,
                TF_abs, 7 );

    struct aa_rx_cl *cl = aa_rx_cl_create(scene_graph);
    struct aa_rx_cl_set* allowed = aa_rx_cl_set_create(scene_graph);

    aa_rx_cl_check(cl, n_f, TF_abs, 7, cl_set);
    aa_rx_cl_set_merge(cl_set, allowed);
    aa_rx_cl_set_destroy(allowed);

    aa_rx_cl_destroy(cl);
}

AA_API void aa_rx_sg_allow_config( struct aa_rx_sg* scene_graph, size_t n_q, const double* q)
{
    struct aa_rx_cl_set* allowed = aa_rx_cl_set_create(scene_graph);
    aa_rx_sg_get_collision(scene_graph, n_q, q, allowed);

    for (size_t i = 0; i<aa_rx_sg_frame_count(scene_graph); i++){
        for (size_t j=0; j<i; j++){
            if (aa_rx_cl_set_get(allowed, i, j)) {
                aa_rx_sg_allow_collision(scene_graph, i, j, 1);
            }
        }
    }

    aa_rx_cl_set_destroy(allowed);

}


/*----------*/
/* Distance */
/*----------*/

static bool
cl_dist_callback( ::amino::fcl::CollisionObject *o1,
                  ::amino::fcl::CollisionObject *o2,
                  void *data_,
                  ::amino::fcl::fcl_scalar &dist );



struct dist_ent {
    double dist;
    double point0[3];
    double point1[3];
};

struct aa_rx_cl_dist {
    const struct aa_rx_cl *cl;

    int in_collision;

    // frame_cnt x frame_cnt
    struct dist_ent *data;
};


struct dist_ent *
s_get_dist_ent( const struct aa_rx_cl_dist *cl_dist,
                aa_rx_frame_id id1, aa_rx_frame_id id2 )
{
    assert( id1 >= id2 );
    size_t n = aa_rx_sg_frame_count(cl_dist->cl->sg);
    return cl_dist->data + id2*n + id1;
}

AA_API struct aa_rx_cl_dist *
aa_rx_cl_dist_create( const struct aa_rx_cl * cl )
{
    aa_rx_sg_ensure_clean_frames( cl->sg );

    struct aa_rx_cl_dist *r = AA_NEW(struct aa_rx_cl_dist);
    r->cl = cl;

    size_t n = aa_rx_sg_frame_count(cl->sg);
    r->data = AA_NEW0_AR(struct dist_ent, n*n);

    return r;
}

AA_API void
aa_rx_cl_dist_destroy( struct aa_rx_cl_dist* dist )
{
    free(dist->data);
    free(dist);
}

AA_API int
aa_rx_cl_dist_check( struct aa_rx_cl_dist *cl_dist,
                     const struct aa_rx_fk *fk )
{
    /* Set Transforms*/
    s_update_tf(cl_dist->cl, check_helper_fk, fk);

    /* Initialize */
    cl_dist->in_collision = 0;
    size_t n = aa_rx_sg_frame_count(cl_dist->cl->sg);
    for (size_t j = 0; j < n; j ++ ) {
        /* zero diagaonal (frame collision with itself) */
        struct dist_ent * ent_diag = s_get_dist_ent(cl_dist, j, j);
        ent_diag->dist = 0;
        AA_MEM_ZERO( ent_diag->point0, 3 );
        AA_MEM_ZERO( ent_diag->point1, 3 );
        // Set non-diagonal distances entries to a big number
        // (entries stored as lower triangular matrix)
        for (size_t i = j + 1; i < n; i ++ ) {
            struct dist_ent * ent = s_get_dist_ent(cl_dist, i, j);
            ent->dist = DBL_MAX;
        }
    }

    /* Check Distance */
    cl_dist->cl->manager->distance( cl_dist, cl_dist_callback );

    /* Result */
    return cl_dist->in_collision;
}

static void s_normalize_ids( aa_rx_frame_id *id1, aa_rx_frame_id *id2 )
{
    if( *id1 < *id2 ) {
        aa_rx_frame_id tmpid = *id1;
        *id1 = *id2;
        *id2 = tmpid;
    }
}

// static void
// s_correct_cylinder_dist(::amino::fcl::CollisionObject *obj,
//                         double *point )
// {
//     struct aa_rx_geom *geom = (struct aa_rx_geom*)obj->collisionGeometry()->getUserData();
//     enum aa_rx_geom_shape shape_type;
//     void *shape_ = aa_rx_geom_shape( geom, &shape_type);
//     if( AA_RX_CYLINDER == shape_type ) {
//         struct aa_rx_shape_cylinder *shape = (struct aa_rx_shape_cylinder *)  shape_;
//         point[2] += shape->height/2;
//     }
// }

static bool
cl_dist_callback( ::amino::fcl::CollisionObject *o1,
                  ::amino::fcl::CollisionObject *o2,
                  void *data_,
                  ::amino::fcl::fcl_scalar &dist )
{
    struct aa_rx_cl_dist *data = (struct aa_rx_cl_dist *)data_;
    aa_rx_frame_id id1 = (intptr_t) o1->getUserData();
    aa_rx_frame_id id2 = (intptr_t) o2->getUserData();

    // Normalize ID order
    if( id1 < id2 ) {
        aa_rx_frame_id tmpid = id1;
        id1 = id2;
        id2 = tmpid;

        ::amino::fcl::CollisionObject *tmpo = o1;
        o1 = o2;
        o2 = tmpo;
    } else if( id1 == id2 ) { // no same-frame collisions
        return false;
    }

    /* Elide allowed collisions */
    if( aa_rx_cl_set_get(data->cl->allowed,id1,id2) ) {
        return false;
    }

    struct dist_ent *ent = s_get_dist_ent(data, id1, id2);

    ::amino::fcl::DistanceRequest request(true);
    ::amino::fcl::DistanceResult result;
    ::fcl::distance(o1, o2, request, result);

    double min_dist = result.min_distance;
    double p0[3];
    double p1[3];

    for( size_t i = 0; i < 3; i ++){
        p0[i]= result.nearest_points[0][i];
        p1[i]= result.nearest_points[1][i];
    }

    if( min_dist <= 0 ) {
        min_dist = 0;
        data->in_collision = 1;

        // If we're in collision lets figure out where we are colliding
        ::amino::fcl::CollisionRequest colRequest(1, true);
        ::amino::fcl::CollisionResult colResult;
        ::fcl::collide(o1, o2, colRequest, colResult);

        size_t numContacts = colResult.numContacts();
        for( size_t i = 0; i < numContacts; i++ ){

            ::amino::fcl::Contact contact = colResult.getContact(i);

            if( min_dist < contact.penetration_depth ) {
                min_dist = contact.penetration_depth;

                for( size_t j = 0; j < 3; j++){
                    p0[j] = contact.pos[j];
                    p1[j] = contact.normal[j]*min_dist+p0[j];
                }
            }
        }
        min_dist = -min_dist;
    }

    /* Frames may have multiple collision objects.
     * Take the minimum. */
    if( ent->dist > min_dist ) {
        ent->dist = min_dist;

        for( size_t i = 0; i < 3; i ++ ) {
            ent->point0[i] = p0[i];
            ent->point1[i] = p1[i];
        }

        // as of FCL 0.6, closest points are in world coords
        //s_correct_cylinder_dist(o1, ent->point0);
        //s_correct_cylinder_dist(o2, ent->point1);

    }
    return false;
}

AA_API double
aa_rx_cl_dist_get_min_dist(const struct aa_rx_cl_dist *cl_dist,
                           aa_rx_frame_id id0, aa_rx_frame_id *id1, double* point_arr)
{
    double min_dist=DBL_MAX;
    size_t frames = aa_rx_sg_frame_count(cl_dist->cl->sg);
    const struct aa_rx_sg* sg = cl_dist->cl->sg;

    for ( aa_rx_frame_id tmp_id = 0; tmp_id < (aa_rx_frame_id) frames; tmp_id++ ){
        if(id0 == tmp_id) continue;
        double tmp_dist = aa_rx_cl_dist_get_dist(cl_dist, id0, tmp_id);
        if (tmp_dist < min_dist){
            min_dist = tmp_dist;
            if ( point_arr ){
                double p0[3], p1[3];
                aa_rx_cl_dist_get_points(cl_dist, id0, tmp_id, p0, p1);
                for( size_t j = 0; j<3; j++){
                    point_arr[j] = p0[j];
                    point_arr[j+3] = p1[j];
                }
            }

            if ( id1 ){
                *id1 = tmp_id;
            }
        }
    }
    return min_dist;
}


AA_API double
aa_rx_cl_dist_get_dist( const struct aa_rx_cl_dist *cl_dist,
                        aa_rx_frame_id id0, aa_rx_frame_id id1 )
{
    s_normalize_ids(&id0, &id1);
    s_get_dist_ent(cl_dist, id0, id1);
    struct dist_ent * ent = s_get_dist_ent(cl_dist, id0, id1);
    return ent->dist;
}

AA_API double
aa_rx_cl_dist_get_points( const struct aa_rx_cl_dist *cl_dist,
                          aa_rx_frame_id id0, aa_rx_frame_id id1,
                          double point0[3], double point1[3] )
{
    struct dist_ent * ent;
    double *p0, *p1;
    aa_rx_frame_id i0, i1;
    if( id0 > id1 ) {
        i0 = id0;
        i1 = id1;
        p0 = point0;
        p1 = point1;
    } else {
        i0 = id1;
        i1 = id0;
        p0 = point1;
        p1 = point0;
    }
    ent = s_get_dist_ent(cl_dist, i0, i1);
    AA_MEM_CPY(p0, ent->point0, 3);
    AA_MEM_CPY(p1, ent->point1, 3);
    return ent->dist;
}
