#include <amino.hpp>

using namespace amino;


//static void afeq( double a, double b, double tol ) {
    //assert( aa_feq(a,b,tol) );
//}


//static void aneq( double a, double b, double tol ) {
    //assert( !aa_feq(a,b,tol) );
//}

void mat() {
    // add
    {
        Vec<1> a;
        Vec<1> b;
        a[0] = 1;
        b[0] = 2;
        Vec<1> c = a + b;
        Vec<1> cp;
        cp[0] = 1 + 2;
        assert( c.eq(cp) );

        c = a*4;
        assert(aa_feq(c[0],4,0));

        c = a/4;
        assert(aa_feq(c[0],1.0/4,0));

        //c = a+4;
        //assert(aa_feq(c[0],1.0+4,0));

    }
}

void tf() {
    {
        Tf tf( 0, -1,  0, 1,
               1,  0,  0, 2,
               0,  0,  1, 3 );
        Mat<3> p(AA_FAR(3, 5, 7));
        Mat<3> qr(AA_FAR(-4,5,10));
        Mat<3> q = tf*p;
        assert( q.eq(qr) );
    }
}

int main( int argc, char **argv) {
    (void) argc; (void) argv;
    mat();
    tf();
    return 0;
}
