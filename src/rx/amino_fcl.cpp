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
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"
#include "amino/rx/scene_collision.h"

#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/broadphase/broadphase.h>

#include "amino/rx/scene_collision_internal.h"
#include "amino/rx/amino_fcl.h"

struct aa_rx_cl_geom {
    boost::shared_ptr<fcl::CollisionGeometry> *ptr;
};


static void cl_init_helper( void *cx, aa_rx_frame_id frame_id, struct aa_rx_geom *geom )
{
    (void)cx; (void)frame_id;
    struct aa_rx_sg *sg = (struct aa_rx_sg*)cx;
    //printf("creating collision geometry for frame %s\n",
           //aa_rx_sg_frame_name(sg, frame_id) );

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
    boost::shared_ptr<fcl::CollisionGeometry> *ptr = NULL;
    enum aa_rx_geom_shape shape_type;
    void *shape_ = aa_rx_geom_shape(geom, &shape_type);

    switch( shape_type ) {
    case AA_RX_NOSHAPE: {
        break;
    }
    case AA_RX_MESH: {
        struct aa_rx_mesh *shape = (struct aa_rx_mesh *)  shape_;
        break;
    }
    case AA_RX_BOX: {
        struct aa_rx_shape_box *shape = (struct aa_rx_shape_box *)  shape_;
        ptr = new boost::shared_ptr<fcl::CollisionGeometry> (
            new fcl::Box(shape->dimension[0],shape->dimension[1],shape->dimension[2]));
        break;
    }
    case AA_RX_SPHERE: {
        struct aa_rx_shape_sphere *shape = (struct aa_rx_shape_sphere *)  shape_;
        ptr = new boost::shared_ptr<fcl::CollisionGeometry> (
            new fcl::Sphere(shape->radius) );
        break;
    }
    case AA_RX_CYLINDER: {
        struct aa_rx_shape_cylinder *shape = (struct aa_rx_shape_cylinder *)  shape_;
        ptr = new boost::shared_ptr<fcl::CollisionGeometry> (
            new fcl::Cylinder(shape->radius, shape->height) );
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
        struct aa_rx_cl_geom *cl_geom = new aa_rx_cl_geom;
        cl_geom->ptr = ptr;
        geom->cl_geom = cl_geom;
        //printf("  created\n");
    } else {
        //printf("  skipped\n");
        fprintf(stderr, "Unimplemented collision type: %s\n", aa_rx_geom_shape_str( shape_type ) );
    }
}


void aa_rx_sg_cl_init( struct aa_rx_sg *scene_graph )
{
    aa_rx_sg_map_geom( scene_graph, &cl_init_helper, scene_graph );
}




struct aa_rx_cl
{
    const struct aa_rx_sg *sg;
    fcl::BroadPhaseCollisionManager *manager;
    std::vector<fcl::CollisionObject*> *objects;
};

static void cl_create_helper( void *cx_, aa_rx_frame_id frame_id, struct aa_rx_geom *geom )
{
    struct aa_rx_cl *cx = (struct aa_rx_cl*)cx_;

    //printf("adding cl_geom for %s\n", aa_rx_sg_frame_name(cx->sg, frame_id) );

    if( ! geom->cl_geom ) return;

    fcl::CollisionObject *obj = new fcl::CollisionObject( *geom->cl_geom->ptr );
    obj->setUserData( (void*) ((intptr_t) frame_id) );
    cx->manager->registerObject(obj);
    cx->objects->push_back( obj );
}

struct aa_rx_cl *
aa_rx_cl_create( const struct aa_rx_sg *scene_graph )
{
    struct aa_rx_cl *cl = new aa_rx_cl;
    cl->sg = scene_graph;
    cl->objects = new std::vector<fcl::CollisionObject*>;
    cl->manager = new fcl::DynamicAABBTreeCollisionManager();

    aa_rx_sg_map_geom( scene_graph, &cl_create_helper, cl );

    cl->manager->setup();

    return cl;
}

void
aa_rx_cl_destroy( struct aa_rx_cl *cl )
{
    delete cl->manager;
    delete cl->objects;
    delete cl;
}

static bool
cl_check_callback( ::fcl::CollisionObject *o1,
                   ::fcl::CollisionObject *o2,
                   void *cdata )
{
    //aa_rx_frame_id id1 = (intptr_t) o1->getUserData();
    //aa_rx_frame_id id2 = (intptr_t) o2->getUserData();

    int *x = (int*)cdata;

    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    fcl::collide(o1, o2, request, result);

    if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts)) {
        *x = 1;
        return true;
    }

    return false;
}

int
aa_rx_cl_check( struct aa_rx_cl *cl,
                size_t n_tf,
                double *TF, size_t ldTF )
{
    /* Update Transforms */
    for( auto itr = cl->objects->begin();
         itr != cl->objects->end();
         itr++ )
    {
        fcl::CollisionObject *obj = *itr;
        aa_rx_frame_id id = (intptr_t) obj->getUserData();
        obj->setTransform( amino::fcl::qutr2fcltf(TF + id*ldTF) );
    }
    cl->manager->update();

    /* Check Collision */
    int collision = 0;
    cl->manager->collide( &collision, cl_check_callback );
    return collision;
}
