

struct gptimer12_timer {
    char *name;
    unsigned int expire_time;  // unit second 
    int (*expire_callback) (unsigned long);
    unsigned long data;
    bool active;
};

struct gptimer12_manager {
    struct gptimer12_timer timer;
    unsigned int remain_time; // unit second 
};


int init_gptimer12 (void);
int finish_gptimer12 (void);

int request_gptimer12(struct gptimer12_timer *timer);

int release_gptimer12(struct gptimer12_timer *timer);

int expire_gptimer12(void);




