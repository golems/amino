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


#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/rxtype_internal.h"

#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"

#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"

#include "amino/rx/scene_collision.h"

#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/BVH/BVH_model.h>

#include "amino/rx/scene_collision_internal.h"
#include "amino/rx/scene_fcl.h"


/* Initialize collision handling */
AA_API void
aa_rx_cl_init( )
{
    aa_rx_cl_geom_destroy_fun = aa_rx_cl_geom_destroy;
}

struct aa_rx_cl_geom {
    std::shared_ptr<fcl::CollisionGeometry> ptr;
    aa_rx_cl_geom( fcl::CollisionGeometry *ptr_) :
        ptr(ptr_) { }

    ~aa_rx_cl_geom() { }
};

AA_API void
aa_rx_cl_geom_destroy( struct aa_rx_cl_geom *cl_geom ) {
    delete cl_geom;
}


static fcl::CollisionGeometry *
cl_init_mesh( const struct aa_rx_mesh *mesh )
{

    //printf("mesh\n");
    //printf("n_verts: %u\n",mesh->n_vertices );
    //printf("n_indices: %u\n",mesh->n_indices );
    /* TODO: FCL should be able to reference external arrays for mesh
     * vertices and indices so that we don't have to copy the mesh.
     */
    std::vector<fcl::Vec3f> vertices;
    std::vector<fcl::Triangle> triangles;


    /* fill vertices */
    {
        const float *v = mesh->vertices;
        for( unsigned i = 0, j=0; i < mesh->n_vertices; i++ ) {
            unsigned x=j++;
            unsigned y=j++;
            unsigned z=j++;
            vertices.push_back( fcl::Vec3f(v[x], v[y], v[z]) );
        }
    }
    //printf("filled verts\n");
    /* fill faces*/
    {
        const unsigned *f = mesh->indices;
        //printf("n_indices: %u\n",mesh->n_indices );
        for( unsigned i=0, j=0; i < mesh->n_indices; i++ ) {
            unsigned x=j++;
            unsigned y=j++;
            unsigned z=j++;
            //printf("tri: %u, %u, %u, (%u)\n", x,y,z, 3*mesh->n_indices);
            triangles.push_back( fcl::Triangle(f[x], f[y], f[z]) );
        }
    }
    //printf("filled tris\n");

    auto model = new(fcl::BVHModel<fcl::OBBRSS>);
    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();

    return model;
}

static void cl_init_helper( void *cx, aa_rx_frame_id frame_id, struct aa_rx_geom *geom )
{
    (void)cx; (void)frame_id;
    struct aa_rx_sg *sg = (struct aa_rx_sg*)cx;

    /* not a collision geometry */
    if( ! geom->opt.collision ) {
        //printf("  not a collision geom\n");
        return;
    }

    /* already have collision object */
    if( geom->cl_geom ) {
        //printf("  already has collision geom\n");
        return;
    }


    /* Ok, now do it */
    fcl::CollisionGeometry *ptr = NULL;
    enum aa_rx_geom_shape shape_type;
    void *shape_ = aa_rx_geom_shape(geom, &shape_type);

    // printf("creating collision geometry for frame %s (%s)\n",
    //        aa_rx_sg_frame_name(sg, frame_id),
    //        aa_rx_geom_shape_str(shape_type) );

    switch( shape_type ) {
    case AA_RX_NOSHAPE: {
        break;
    }
    case AA_RX_MESH: {
        struct aa_rx_mesh *shape = (struct aa_rx_mesh *)  shape_;
        ptr = cl_init_mesh(shape);
        break;
    }
    case AA_RX_BOX: {
        struct aa_rx_shape_box *shape = (struct aa_rx_shape_box *)  shape_;
        ptr = new fcl::Box(shape->dimension[0],shape->dimension[1],shape->dimension[2]);
        break;
    }
    case AA_RX_SPHERE: {
        struct aa_rx_shape_sphere *shape = (struct aa_rx_shape_sphere *)  shape_;
        ptr = new fcl::Sphere(shape->radius);
        break;
    }
    case AA_RX_CYLINDER: {
        struct aa_rx_shape_cylinder *shape = (struct aa_rx_shape_cylinder *)  shape_;
        ptr = new fcl::Cylinder(shape->radius, shape->height);
        break;
    }
    case AA_RX_CONE: {
        struct aa_rx_shape_cone *shape = (struct aa_rx_shape_cone *)  shape_;
        break;
    }
    case AA_RX_GRID: {
        struct aa_rx_shape_grid *shape = (struct aa_rx_shape_grid *)  shape_;
        break;
    }
    }

    if(ptr) {
        struct aa_rx_cl_geom *cl_geom = new aa_rx_cl_geom(ptr);
        cl_geom->ptr->setUserData(geom); // FCL user data is the amino geometry object
        geom->cl_geom = cl_geom;   // Set the amino geometry collision object
    } else {
        fprintf(stderr, "Unimplemented collision type: %s\n", aa_rx_geom_shape_str( shape_type ) );
    }
}


void aa_rx_sg_cl_init( struct aa_rx_sg *scene_graph )
{

    if( NULL == aa_rx_cl_geom_destroy_fun ) {
        fprintf(stderr, "ERROR: collision module not initialized, call aa_rx_cl_init()\n");
        abort();
        exit(EXIT_FAILURE);
    }
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
    fcl::BroadPhaseCollisionManager *manager;
    std::vector<fcl::CollisionObject*> *objects;

    // A bit-matrix of allowable collisions
    struct aa_rx_cl_set *allowed;
};

static void cl_create_helper( void *cx_, aa_rx_frame_id frame_id, struct aa_rx_geom *geom )
{
    struct aa_rx_cl *cx = (struct aa_rx_cl*)cx_;

    //printf("adding cl_geom for %s\n", aa_rx_sg_frame_name(cx->sg, frame_id) );

    if( ! geom->cl_geom ) return;

    fcl::CollisionObject *obj = new fcl::CollisionObject( geom->cl_geom->ptr );
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
    cl->objects = new std::vector<fcl::CollisionObject*>;
    cl->manager = new fcl::DynamicAABBTreeCollisionManager();

    cl->allowed = aa_rx_cl_set_create(scene_graph);

    aa_rx_sg_map_geom( scene_graph, &cl_create_helper, cl );

    cl->manager->setup();

    return cl;
}

void
aa_rx_cl_destroy( struct aa_rx_cl *cl )
{
    for( fcl::CollisionObject *o : *cl->objects ) {
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
cl_check_callback( ::fcl::CollisionObject *o1,
                   ::fcl::CollisionObject *o2,
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


    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    fcl::collide(o1, o2, request, result);

    if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts)) {
        //printf("collide: %s x %s\n", name1, name2 );
        /* In Collision */
        data->result = 1;

        /* Short Circuit? */
        if( data->cl_set ) {
            aa_rx_cl_set_set( data->cl_set, id1, id2, 1 );
        } else {
            return true;
        }
    }

    return false;
}

int
aa_rx_cl_check( struct aa_rx_cl *cl,
                size_t n_tf,
                const double *TF, size_t ldTF,
                struct aa_rx_cl_set *cl_set )
{
    /* Update Transforms */
    for( auto itr = cl->objects->begin();
         itr != cl->objects->end();
         itr++ )
    {
        fcl::CollisionObject *obj = *itr;
        aa_rx_frame_id id = (intptr_t) obj->getUserData();
        const double *TF_obj = TF+id*ldTF;

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

    /* Check Collision */
    struct cl_check_data data;
    data.result = 0;
    data.cl = cl;
    data.cl_set = cl_set;

    cl->manager->collide( &data, cl_check_callback );
    return data.result;
}

AA_API void aa_rx_cl_allow_config( struct aa_rx_sg* scene_graph, const size_t n_q, const double* q)
{
    size_t n_f = aa_rx_sg_frame_count(scene_graph);
    if (n_q != aa_rx_sg_config_count(scene_graph)){
        printf("Error: DoF specified different from scenegraph. Allowable config not set.\n");
        return;
    }
    double TF_rel[7*n_f];
    double TF_abs[7*n_f];
    aa_rx_sg_tf(scene_graph, n_q, q,
                n_f,
                TF_rel, 7,
                TF_abs, 7 );


    struct aa_rx_cl *cl = aa_rx_cl_create(scene_graph);
    struct aa_rx_cl_set* allowed = aa_rx_cl_set_create(scene_graph);
    aa_rx_cl_check(cl, n_f, TF_abs, 7, allowed);

    for (size_t i = 0; i<n_f; i++){
        for (size_t j=0; j<i; j++){
            if (aa_rx_cl_set_get(allowed, i, j)) {
                aa_rx_sg_allow_collision(scene_graph, i, j, 1);
            }
        }
    }

    aa_rx_cl_set_destroy(allowed);
    aa_rx_cl_destroy(cl);

}
