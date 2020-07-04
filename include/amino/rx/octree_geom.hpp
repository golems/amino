#ifndef OCTREE_GEOM_HPP
#define OCTREE_GEOM_HPP

#include "amino/rx/scene_geom.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap/AbstractOcTree.h>

struct aa_rx_geom_octree {
    struct aa_rx_geom *base;
    struct aa_rx_octree *shape;
};

struct aa_rx_octree {
  octomap::OcTree* otree;
};

AA_API struct aa_rx_octree*
aa_rx_geom_read_octree_from_file( const char* file);

/**
 * Attach a octree to a frame.
 */

AA_API struct aa_rx_geom *
aa_rx_geom_octree (
                   struct aa_rx_geom_opt *opt,
                   struct aa_rx_octree *octree);


#endif // OCTREE_OCTREE_HPP
