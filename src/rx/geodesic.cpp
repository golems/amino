#include <vector>
#include <cstdlib>
#include <cmath>

void geodesic_round( std::vector<double> &verts,
                     const std::vector<unsigned> &faces,
                     std::vector<unsigned> &new_faces,
                     double radius )
{
    for( unsigned i = 0; i < faces.size(); i += 3 ) {
        unsigned j0 = i;
        unsigned j1 = i+1;
        unsigned j2 = i+2;
        double *v0 = &verts.front() + 3*faces[j0];
        double *v1 = &verts.front() + 3*faces[j1];
        double *v2 = &verts.front() + 3*faces[j2];

        // New Vertices
        double u01[3], u02[3], u12[3];
        double a01=0, a02=0, a12=0;
        for( size_t j = 0; j < 3; j++ ) {
            u01[j] = (v0[j] + v1[j]) / 2;
            u02[j] = (v0[j] + v2[j]) / 2;
            u12[j] = (v1[j] + v2[j]) / 2;

            a01 += u01[j]*u01[j];
            a02 += u02[j]*u02[j];
            a12 += u12[j]*u12[j];
        }
        // Normalize
        {
            double s01 = radius / sqrt(a01);
            double s02 = radius / sqrt(a02);
            double s12 = radius / sqrt(a12);
            for( size_t j = 0; j < 3; j++ ) {
                u01[j] *= s01;
                u02[j] *= s02;
                u12[j] *= s12;
            }
        }

        // Add to list
        unsigned k01 = (unsigned)(verts.size() / 3);
        for( size_t j = 0; j < 3; j++ ) {
            verts.push_back( u01[j] );
        }

        unsigned k02 = (unsigned)(verts.size() / 3);
        for( size_t j = 0; j < 3; j++ ) {
            verts.push_back( u02[j] );
        }

        unsigned k12 = (unsigned)(verts.size() / 3);
        for( size_t j = 0; j < 3; j++ ) {
            verts.push_back( u12[j] );
        }

        // New Faces
        new_faces.push_back(j0);
        new_faces.push_back(k01);
        new_faces.push_back(k02);

        new_faces.push_back(j1);
        new_faces.push_back(k01);
        new_faces.push_back(k12);

        new_faces.push_back(j2);
        new_faces.push_back(k02);
        new_faces.push_back(k12);

        new_faces.push_back(k01);
        new_faces.push_back(k02);
        new_faces.push_back(k12);
    }
}
