#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include "amino.h"
#include "amino/rx/scene_fcl.h"



int main( int argc, char **argv)
{
    (void) argc; (void) argv;

    double Ea[7] = AA_TF_QUTR_IDENT_INITIALIZER;
    double Eb[7] = AA_TF_QUTR_IDENT_INITIALIZER;
    double Ec[7] = {0,0,0,1, 2,0,0};

    std::shared_ptr<fcl::CollisionGeometry> a( new fcl::Box(1,1,1) );

    fcl::CollisionObject* obj_a = new fcl::CollisionObject(a, amino::fcl::qutr2fcltf(Ea));
    fcl::CollisionObject* obj_b = new fcl::CollisionObject(a, amino::fcl::qutr2fcltf(Eb));
    fcl::CollisionObject* obj_c = new fcl::CollisionObject(a, amino::fcl::qutr2fcltf(Ec));

    {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        fcl::collide(obj_a, obj_b, request, result);
        assert( result.isCollision() );
    }

    {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        fcl::collide(obj_a, obj_c, request, result);
        assert( ! result.isCollision() );
    }


    return 0;

}
