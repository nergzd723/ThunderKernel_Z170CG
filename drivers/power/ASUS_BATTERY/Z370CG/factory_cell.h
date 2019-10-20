typedef enum _cell_type {
    TYPE_COS_LIGHT=0,
    TYPE_LG,
    TYPE_ATL,
} cell_type;

struct bq27xx_dffs_data {
    u32 cell_type;
    u32 num_op;
    struct update_op *op; 
};
