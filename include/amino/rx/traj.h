#define AA_RX_TRAJPT_FD                         \
    size_t i;                                   \
    aa_rx_trajpt_t *prev, *next;

#define AA_RX_TRAJSEG_FD                                    \
    int (*value)(aa_rx_traj_t *traj, aa_rx_trajseg_t *seg,  \
                 void *v, double t);                        \
    size_t i;                                               \
    double t_s, t_f;                                        \
    aa_rx_trajseg_t *prev, *next;                           \

typedef struct aa_rx_traj aa_rx_traj_t;
typedef struct aa_rx_trajpt aa_rx_trajpt_t;
typedef struct aa_rx_trajseg aa_rx_trajseg_t;

typedef struct aa_rx_trajpt {
    AA_RX_TRAJPT_FD;
} aa_rx_trajpt_t;

typedef struct aa_rx_trajseg {
    AA_RX_TRAJSEG_FD;
} aa_rx_trajseg_t;

typedef struct aa_rx_traj {
    struct aa_rx_sg *scenegraph;
    size_t n_q;

    aa_mem_region_t *reg;

    aa_mem_rlist_t *points;
    aa_rx_trajpt_t *tail_pt;

    aa_mem_rlist_t *segments;
    aa_rx_trajseg_t *tail_seg;

    struct aa_mem_cons *last_seg;

    double t_o;
    int (*generate)(aa_rx_traj_t *traj, void *cx);
} aa_rx_traj_t;


typedef struct aa_rx_pb_trajval {
    double *q;
    double *dq;
} aa_rx_pb_trajval_t;

typedef struct aa_rx_pb_trajpt_t {
    AA_RX_TRAJPT_FD;
    double *q;
} aa_rx_pb_trajpt_t;

typedef struct aa_rx_pb_trajseg aa_rx_pb_trajseg_t;
typedef struct aa_rx_pb_trajseg {
    AA_RX_TRAJSEG_FD;
    aa_rx_pb_trajpt_t *pt;
    double *dq, *ddq;
    double dt, b;
} aa_rx_pb_trajseg_t;

void aa_rx_traj_init(aa_rx_traj_t *traj, struct aa_rx_sg *scenegraph,
                     aa_mem_region_t *reg,
                     int (*generate)(aa_rx_traj_t *traj, void *cx));

int aa_rx_traj_generate(aa_rx_traj_t *traj, void *cx);

int aa_rx_traj_value(aa_rx_traj_t *traj, void *v, double t);

int aa_rx_pb_traj_generate(aa_rx_traj_t *traj, void *cx);

void aa_rx_trajpt_add(aa_rx_traj_t *traj, aa_rx_trajpt_t *pt);
