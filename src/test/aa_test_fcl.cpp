#include "config.h"


#include <fcl/fcl.h>

#include "amino.h"
#include "amino/rx/scene_fcl.h"



int main( int argc, char **argv)
{
    (void) argc; (void) argv;

    double Ea[7] = AA_TF_QUTR_IDENT_INITIALIZER;
    double Eb[7] = AA_TF_QUTR_IDENT_INITIALIZER;
    double Ec[7] = {0,0,0,1, 2,0,0};

    std::shared_ptr<::amino::fcl::CollisionGeometry> a( new ::amino::fcl::Box(1,1,1) );

    ::amino::fcl::CollisionObject* obj_a =
          new ::amino::fcl::CollisionObject(a, amino::fcl::qutr2fcltf(Ea));
    ::amino::fcl::CollisionObject* obj_b =
          new ::amino::fcl::CollisionObject(a, amino::fcl::qutr2fcltf(Eb));
    ::amino::fcl::CollisionObject* obj_c =
          new ::amino::fcl::CollisionObject(a, amino::fcl::qutr2fcltf(Ec));

    {
        ::amino::fcl::CollisionRequest request;
        ::amino::fcl::CollisionResult result;
        ::fcl::collide(obj_a, obj_b, request, result);
        assert( result.isCollision() );
    }

    {
        ::amino::fcl::CollisionRequest request;
        ::amino::fcl::CollisionResult result;
        ::fcl::collide(obj_a, obj_c, request, result);
        assert( ! result.isCollision() );
    }


    return 0;

}
