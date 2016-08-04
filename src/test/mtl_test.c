#include "amino.h"
#include "amino/rx/wavefront.h"



int main(int argc, char **argv) {
    (void) argc; (void) argv;
    if( argc < 2 ) {
        fprintf(stderr, "Input not specified\n");
        exit(EXIT_FAILURE);
    }

    const char *filename = argv[1];
    fprintf(stdout, "Input: %s\n", filename);
    struct aa_rx_wf_mtl *mtl = aa_rx_wf_mtl_parse(filename);

    if(mtl) {
        fprintf(stdout, "done\n");
        aa_rx_wf_mtl_destroy(mtl);
    } else {
        fprintf(stdout, "error\n");
    }


    return 0;
}
